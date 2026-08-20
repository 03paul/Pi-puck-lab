"""Fit Tier 0's travel-time model t = a + d/v_eff against Tier 1 (Webots) measurements.

Usage:
    python -m experiments.calibrate_travel_model --runs logs/webots_calibration.csv
    python -m experiments.calibrate_travel_model --runs logs/webots_calibration.csv --apply

The input CSV has two columns, one row per point-to-point run:
    distance_m,elapsed_s

`elapsed_s` is measured from the drive command being issued to the robot settling within
`arrival_tolerance` of the target (same semantics as `Robot.is_at_target()` in
`backends/abstract.py`), so the fitted constants are directly comparable to
`parameters/default.json`.

Without `--apply` the script only reports the fit. With `--apply` it rewrites
`startup_delay` and `effective_speed` in `parameters/default.json` in place; run
`git diff` afterwards to review the change before committing it.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

import numpy as np

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "parameters" / "default.json"


def load_runs(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    distances: list[float] = []
    times: list[float] = []
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or {"distance_m", "elapsed_s"} - set(reader.fieldnames):
            raise ValueError("CSV must have columns distance_m,elapsed_s")
        for row in reader:
            distances.append(float(row["distance_m"]))
            times.append(float(row["elapsed_s"]))
    if len(distances) < 8:
        raise ValueError(
            f"only {len(distances)} runs given; the calibration protocol calls for at least "
            "~20 spread across the arena diagonal for a defensible fit"
        )
    return np.asarray(distances, dtype=float), np.asarray(times, dtype=float)


def fit_travel_model(distances: np.ndarray, times: np.ndarray) -> tuple[float, float, float]:
    """Ordinary least squares fit of t = a + b*d, returned as (a, v_eff=1/b, r_squared)."""

    design = np.vstack([np.ones_like(distances), distances]).T
    (a, slope), *_ = np.linalg.lstsq(design, times, rcond=None)
    if slope <= 0.0:
        raise ValueError("fitted slope is non-positive; travel time must increase with distance")
    predicted = design @ np.array([a, slope])
    residuals = times - predicted
    ss_res = float(np.sum(residuals**2))
    ss_tot = float(np.sum((times - times.mean()) ** 2))
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 0.0 else float("nan")
    return float(a), float(1.0 / slope), r_squared


def bootstrap_ci(
    distances: np.ndarray,
    times: np.ndarray,
    rng: np.random.Generator,
    samples: int = 20_000,
) -> dict[str, float]:
    """Case-resampling bootstrap CI for a and v_eff, matching experiments/compare.py's style."""

    count = len(distances)
    indices = rng.integers(0, count, size=(samples, count))
    a_samples = np.empty(samples)
    v_samples = np.empty(samples)
    for i, idx in enumerate(indices):
        try:
            a_samples[i], v_samples[i], _ = fit_travel_model(distances[idx], times[idx])
        except ValueError:
            a_samples[i], v_samples[i] = np.nan, np.nan
    a_samples = a_samples[~np.isnan(a_samples)]
    v_samples = v_samples[~np.isnan(v_samples)]
    return {
        "a_ci_low": float(np.quantile(a_samples, 0.025)),
        "a_ci_high": float(np.quantile(a_samples, 0.975)),
        "v_eff_ci_low": float(np.quantile(v_samples, 0.025)),
        "v_eff_ci_high": float(np.quantile(v_samples, 0.975)),
        "dropped_resamples": float(samples - len(a_samples)),
    }


def apply_to_config(config_path: Path, a: float, v_eff: float) -> tuple[float, float]:
    """Rewrite only startup_delay/effective_speed in place, leaving every other line untouched.

    A full json.dumps round-trip would also reformat untouched numbers (e.g. 0.00008 ->
    8e-05), producing a noisy git diff. A targeted regex substitution keeps the diff to
    exactly the two calibrated fields.
    """

    text = config_path.read_text(encoding="utf-8")
    data = json.loads(text)
    old_startup = data["simulation"]["startup_delay"]
    old_speed = data["simulation"]["effective_speed"]
    new_startup, new_speed = round(a, 4), round(v_eff, 4)

    def replace_field(source: str, field: str, old_value: float, new_value: float) -> str:
        pattern = re.compile(rf'("{field}"\s*:\s*){re.escape(json.dumps(old_value))}(?=\s*,)')
        updated, count = pattern.subn(rf"\g<1>{new_value}", source, count=1)
        if count != 1:
            raise ValueError(f"could not locate a unique '{field}' field to update in {config_path}")
        return updated

    text = replace_field(text, "startup_delay", old_startup, new_startup)
    text = replace_field(text, "effective_speed", old_speed, new_speed)
    config_path.write_text(text, encoding="utf-8")
    return old_startup, old_speed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", required=True, help="CSV with distance_m,elapsed_s columns")
    parser.add_argument("--apply", action="store_true", help="write fitted values into parameters/default.json")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="config file to update with --apply")
    parser.add_argument("--seed", type=int, default=1234)
    parser.add_argument("--samples", type=int, default=20_000)
    args = parser.parse_args()

    distances, times = load_runs(args.runs)
    a, v_eff, r_squared = fit_travel_model(distances, times)
    ci = bootstrap_ci(distances, times, np.random.default_rng(args.seed), args.samples)

    print(f"runs                 : {len(distances)}")
    print(f"distance range [m]   : {distances.min():.3f} - {distances.max():.3f}")
    print(f"startup_delay a [s]  : {a:.4f}  (95% CI [{ci['a_ci_low']:.4f}, {ci['a_ci_high']:.4f}])")
    print(f"effective_speed [m/s]: {v_eff:.4f}  (95% CI [{ci['v_eff_ci_low']:.4f}, {ci['v_eff_ci_high']:.4f}])")
    print(f"R^2                  : {r_squared:.4f}")
    if r_squared < 0.8:
        print(
            "WARNING: R^2 < 0.8 - the linear travel-time model fits poorly. Check for turning "
            "overhead, acceleration ramps, or outlier runs before trusting this fit."
        )

    if args.apply:
        config_path = Path(args.config)
        old_startup, old_speed = apply_to_config(config_path, a, v_eff)
        print(f"\nUpdated {config_path}:")
        print(f"  startup_delay:   {old_startup} -> {round(a, 4)}")
        print(f"  effective_speed: {old_speed} -> {round(v_eff, 4)}")
        print("Review with `git diff` and re-run experiments.compare to check the ranking still holds.")


if __name__ == "__main__":
    main()
