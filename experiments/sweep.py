"""Grid-search market weights on training seeds and retain the Pareto front."""

from __future__ import annotations

import itertools
import json
import statistics
from dataclasses import asdict
from pathlib import Path
from typing import Any

from allocation.bidding import BidWeights
from allocation.strategies import Strategy, StrategyName
from sim.config import SimulationConfig
from sim.world import run_simulation

from .common import quality_score, read_json, write_rows

PARETO_AXES = {
    "completion_rate": 1,
    "mean_completion_time": -1,
    "load_imbalance": -1,
    "max_cell_idleness": -1,
    "mean_final_battery": 1,
}


def _dominates(left: dict[str, Any], right: dict[str, Any]) -> bool:
    no_worse = all(PARETO_AXES[key] * left[key] >= PARETO_AXES[key] * right[key] for key in PARETO_AXES)
    strictly_better = any(PARETO_AXES[key] * left[key] > PARETO_AXES[key] * right[key] for key in PARETO_AXES)
    return no_worse and strictly_better


def parameter_sweep(
    config: SimulationConfig,
    search_space: dict[str, Any],
    output_dir: str | Path,
) -> tuple[BidWeights, list[dict[str, Any]], list[dict[str, Any]]]:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    raw_rows: list[dict[str, Any]] = []
    combinations = itertools.product(
        search_space["beta"],
        search_space["gamma"],
        search_space["delta"],
        search_space.get("theta_hyst", [0.1]),
    )
    for beta, gamma, delta, theta_hyst in combinations:
        weights = BidWeights(1.0, float(beta), float(gamma), float(delta), float(theta_hyst))
        for seed in search_space["training_seeds"]:
            result = run_simulation(config, Strategy(StrategyName.MARKET, weights), int(seed))
            row = {**asdict(weights), **result.metrics}
            row["quality_score"] = quality_score(result.metrics, config.duration, config.robot_count)
            raw_rows.append(row)

    metric_keys = [
        "quality_score",
        "completion_rate",
        "mean_completion_time",
        "load_imbalance",
        "max_cell_idleness",
        "mean_final_battery",
        "mean_detection_latency",
        "messages_per_completed",
    ]
    groups: dict[tuple[float, float, float, float], list[dict[str, Any]]] = {}
    for row in raw_rows:
        key = (row["beta"], row["gamma"], row["delta"], row["theta_hyst"])
        groups.setdefault(key, []).append(row)
    summary: list[dict[str, Any]] = []
    for key, rows in groups.items():
        item: dict[str, Any] = {"beta": key[0], "gamma": key[1], "delta": key[2], "theta_hyst": key[3]}
        for metric in metric_keys:
            item[metric] = statistics.fmean(float(row[metric]) for row in rows)
        summary.append(item)
    summary.sort(key=lambda row: (-row["quality_score"], row["beta"], row["gamma"], row["delta"]))
    pareto = [
        row for row in summary if not any(_dominates(other, row) for other in summary if other is not row)
    ]
    pareto.sort(key=lambda row: -row["quality_score"])
    best = summary[0]
    best_weights = BidWeights(1.0, best["beta"], best["gamma"], best["delta"], best["theta_hyst"])
    write_rows(output / "sweep_metrics.csv", raw_rows)
    write_rows(output / "sweep_summary.csv", summary)
    write_rows(output / "pareto_parameters.csv", pareto)
    (output / "best_parameters.json").write_text(
        json.dumps(
            {"market_weights": asdict(best_weights), "training_quality_score": best["quality_score"]},
            indent=2,
        ),
        encoding="utf-8",
    )
    return best_weights, summary, pareto


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    config = SimulationConfig.from_json(root / "parameters" / "default.json")
    search = read_json(root / "parameters" / "search_space.json")
    best, _, pareto = parameter_sweep(config, search, root / "results")
    print(json.dumps({"best": asdict(best), "pareto_points": len(pareto)}, indent=2))


if __name__ == "__main__":
    main()
