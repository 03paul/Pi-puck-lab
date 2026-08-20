"""Training/holdout benchmark with paired CIs and randomisation tests."""

from __future__ import annotations

import json
import statistics
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

from allocation.bidding import BidWeights
from allocation.strategies import Strategy, StrategyName
from sim.config import SimulationConfig
from sim.world import run_simulation

from .common import quality_score, read_json, write_rows
from .plots import make_plots
from .robustness import run_robustness
from .sweep import parameter_sweep

METRICS: dict[str, tuple[str, int]] = {
    "quality_score": ("Composite operational score", 1),
    "completion_rate": ("Completion rate", 1),
    "mean_completion_time": ("Mean completion time [s]", -1),
    "load_imbalance": ("Load imbalance [tasks]", -1),
    "coverage_fraction": ("Coverage fraction", 1),
    "mean_cell_idleness": ("Mean cell idleness [s]", -1),
    "max_cell_idleness": ("Max cell idleness [s]", -1),
    "mean_detection_latency": ("Detection latency [s]", -1),
    "messages_per_completed": ("Messages/completed task", -1),
    "mean_final_battery": ("Mean final battery", 1),
    "depleted_robots": ("Depleted robots", -1),
}


def _paired_inference(
    market: np.ndarray,
    baseline: np.ndarray,
    direction: int,
    rng: np.random.Generator,
    samples: int = 20_000,
) -> dict[str, float]:
    improvement = direction * (market - baseline)
    count = len(improvement)
    indices = rng.integers(0, count, size=(samples, count))
    bootstrap = improvement[indices].mean(axis=1)
    observed = float(improvement.mean())
    signs = rng.choice(np.array([-1.0, 1.0]), size=(samples, count))
    null = (signs * improvement).mean(axis=1)
    p_value = (float(np.count_nonzero(np.abs(null) >= abs(observed))) + 1.0) / (samples + 1.0)
    return {
        "mean_improvement": observed,
        "ci_low": float(np.quantile(bootstrap, 0.025)),
        "ci_high": float(np.quantile(bootstrap, 0.975)),
        "p_raw": p_value,
        "win_rate": float(
            (np.count_nonzero(improvement > 0.0) + 0.5 * np.count_nonzero(improvement == 0.0)) / count
        ),
        "relative_improvement": observed / max(1e-12, float(np.mean(np.abs(baseline)))),
    }


def _holm_adjust(rows: list[dict[str, Any]]) -> None:
    ordered = sorted(enumerate(rows), key=lambda item: item[1]["p_raw"])
    running = 0.0
    total = len(rows)
    adjusted = [1.0] * total
    for rank, (original_index, row) in enumerate(ordered):
        running = max(running, min(1.0, (total - rank) * float(row["p_raw"])))
        adjusted[original_index] = running
    for row, value in zip(rows, adjusted):
        row["p_holm"] = value


def _aggregate(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    strategies = sorted({str(row["strategy"]) for row in rows})
    for strategy in strategies:
        group = [row for row in rows if row["strategy"] == strategy]
        for metric, (label, _) in METRICS.items():
            values = [float(row[metric]) for row in group]
            output.append(
                {
                    "strategy": strategy,
                    "metric": metric,
                    "label": label,
                    "mean": statistics.fmean(values),
                    "std": statistics.stdev(values) if len(values) > 1 else 0.0,
                    "n": len(values),
                }
            )
    return output


def _report(
    path: Path,
    best_weights: BidWeights,
    seeds: list[int],
    aggregate: list[dict[str, Any]],
    effects: list[dict[str, Any]],
    robustness: list[dict[str, Any]],
    pareto_count: int,
) -> None:
    aggregates = {(row["strategy"], row["metric"]): row["mean"] for row in aggregate}
    lines = [
        "# Holdout-Vergleich: dezentrale Marktauktion vs. Baselines",
        "",
        (
            f"Auswertung auf {len(seeds)} zuvor nicht zur Parametersuche verwendeten Seeds "
            f"({min(seeds)}-{max(seeds)}), jeweils als gepaarter Vergleich desselben Szenarios."
        ),
        "",
        "## Gewählte Parameter",
        "",
        (
            f"`alpha={best_weights.alpha}`, `beta={best_weights.beta}`, `gamma={best_weights.gamma}`, "
            f"`delta={best_weights.delta}`, `theta_hyst={best_weights.theta_hyst}`. "
            f"Die Trainingssuche ergab {pareto_count} nicht-dominierte Parameterpunkte; der gewählte Punkt maximiert "
            "den vorab dokumentierten operationalen Score."
        ),
        "",
        "## Mittelwerte auf dem Holdout-Set",
        "",
        "| Strategie | Score | Completion | Zeit [s] | Load-SD | Max Idleness [s] | Detektionslatenz [s] | Msg/Task |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    order = ["market", "nearest_greedy", "nearest_greedy_no_survey", "random_assignment"]
    for strategy in order:
        lines.append(
            f"| {strategy} | {aggregates[(strategy, 'quality_score')]:.3f} | "
            f"{aggregates[(strategy, 'completion_rate')]:.3f} | "
            f"{aggregates[(strategy, 'mean_completion_time')]:.2f} | "
            f"{aggregates[(strategy, 'load_imbalance')]:.2f} | "
            f"{aggregates[(strategy, 'max_cell_idleness')]:.2f} | "
            f"{aggregates[(strategy, 'mean_detection_latency')]:.2f} | "
            f"{aggregates[(strategy, 'messages_per_completed')]:.2f} |"
        )
    lines.extend(
        [
            "",
            "## Gepaarte Effekte zugunsten des Marktes",
            "",
            "Positive Werte bedeuten: Market ist besser. 95%-Bootstrap-CI; p-Werte sind je Baseline mit Holm korrigiert.",
            "",
            "| Baseline | Metrik | Verbesserung | 95%-CI | Win-Rate | p(Holm) | Befund |",
            "|---|---|---:|---:|---:|---:|---|",
        ]
    )
    for row in effects:
        finding = (
            "Vorteil"
            if row["ci_low"] > 0 and row["p_holm"] < 0.05
            else ("Nachteil" if row["ci_high"] < 0 and row["p_holm"] < 0.05 else "unklar/Trade-off")
        )
        lines.append(
            f"| {row['baseline']} | {row['label']} | {row['mean_improvement']:.4f} | "
            f"[{row['ci_low']:.4f}, {row['ci_high']:.4f}] | {row['win_rate']:.2%} | {row['p_holm']:.4f} | {finding} |"
        )
    lines.extend(
        [
            "",
            "## Lambda-Robustheit (Poisson-Zellereignisse)",
            "",
            "| lambda_model / lambda_true | Detektionslatenz [s] | Detection rate | Completion rate | Max Idleness [s] |",
            "|---:|---:|---:|---:|---:|",
        ]
    )
    for row in robustness:
        lines.append(
            f"| {row['lambda_factor']:.3g} | {row['mean_detection_latency']:.2f} | "
            f"{row['detection_rate']:.3f} | {row['completion_rate']:.3f} | {row['max_cell_idleness']:.2f} |"
        )
    lines.extend(
        [
            "",
            "## Was damit bewiesen ist - und was nicht",
            "",
            (
                "Die Tabellen sind ein reproduzierbarer empirischer Nachweis für diese Simulator-, Parameter- und "
                "Szenariofamilie. Ein universeller mathematischer Dominanzbeweis ist nicht möglich: Nearest Greedy kann bei "
                "reiner lokaler Fahrzeit optimal sein, und die marktbasierte Methode bezahlt Last- und Energiebalance mit "
                "zusätzlichen Wegen. Deshalb werden jede Einzelmetrik, Konfidenzintervalle, Trade-offs und der "
                "Kommunikationspreis offengelegt."
            ),
            "",
            (
                "Die Trennung von Trainings- und Holdout-Seeds verhindert, dass derselbe Zufall sowohl zur Parameterwahl als "
                "auch zum Nachweis verwendet wird. Alle Strategien nutzen exakt dasselbe Nachrichtenprotokoll und dieselben "
                "Szenario-Seeds."
            ),
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    results_dir = root / "results"
    results_dir.mkdir(exist_ok=True)
    config_data = read_json(root / "parameters" / "default.json")
    config = SimulationConfig(**config_data["simulation"])
    search = read_json(root / "parameters" / "search_space.json")
    best_weights, _, pareto = parameter_sweep(config, search, results_dir)

    strategies = [
        Strategy(StrategyName.MARKET, best_weights),
        Strategy(StrategyName.GREEDY, best_weights),
        Strategy(StrategyName.GREEDY_NO_SURVEY, best_weights),
        Strategy(StrategyName.RANDOM, best_weights),
    ]
    run_rows: list[dict[str, Any]] = []
    for seed in search["evaluation_seeds"]:
        for strategy in strategies:
            result = run_simulation(config, strategy, int(seed))
            row = dict(result.metrics)
            row["quality_score"] = quality_score(result.metrics, config.duration, config.robot_count)
            run_rows.append(row)
    write_rows(results_dir / "run_metrics.csv", run_rows)
    aggregate = _aggregate(run_rows)
    write_rows(results_dir / "aggregate_metrics.csv", aggregate)

    rng = np.random.default_rng(20260811)
    effects: list[dict[str, Any]] = []
    baselines = [item.name.value for item in strategies[1:]]
    for baseline in baselines:
        baseline_effects: list[dict[str, Any]] = []
        for metric, (label, direction) in METRICS.items():
            market_values = np.array(
                [float(row[metric]) for row in run_rows if row["strategy"] == StrategyName.MARKET.value]
            )
            baseline_values = np.array(
                [float(row[metric]) for row in run_rows if row["strategy"] == baseline]
            )
            inference = _paired_inference(market_values, baseline_values, direction, rng)
            baseline_effects.append({"baseline": baseline, "metric": metric, "label": label, **inference})
        _holm_adjust(baseline_effects)
        effects.extend(baseline_effects)
    write_rows(results_dir / "paired_effects.csv", effects)

    robustness = run_robustness(
        config,
        best_weights,
        [int(seed) + 20_000 for seed in search["training_seeds"] + search["evaluation_seeds"][:12]],
        results_dir,
    )
    make_plots(results_dir)
    _report(
        results_dir / "comparison_summary.md",
        best_weights,
        [int(seed) for seed in search["evaluation_seeds"]],
        aggregate,
        effects,
        robustness,
        len(pareto),
    )
    (results_dir / "benchmark_manifest.json").write_text(
        json.dumps(
            {
                "config": config.to_dict(),
                "best_market_weights": asdict(best_weights),
                "training_seeds": search["training_seeds"],
                "evaluation_seeds": search["evaluation_seeds"],
                "quality_score_weights": {
                    "completion_rate": 0.30,
                    "detection_rate": 0.10,
                    "completion_time": 0.10,
                    "load_balance": 0.15,
                    "coverage": 0.05,
                    "mean_idleness": 0.08,
                    "detection_latency": 0.07,
                    "final_battery": 0.05,
                    "non_depletion": 0.10,
                },
            },
            indent=2,
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    print(json.dumps({"best_weights": asdict(best_weights), "results": str(results_dir)}, indent=2))


if __name__ == "__main__":
    main()
