"""Measure sensitivity to a threefold error in the modelled event rate."""

from __future__ import annotations

import statistics
from dataclasses import replace
from pathlib import Path
from typing import Any

from allocation.bidding import BidWeights
from allocation.strategies import Strategy, StrategyName
from sim.config import SimulationConfig
from sim.world import run_simulation

from .common import write_rows


def run_robustness(
    config: SimulationConfig,
    weights: BidWeights,
    seeds: list[int],
    output_dir: str | Path,
) -> list[dict[str, Any]]:
    true_rate = config.lambda_true
    factors = [1.0 / 3.0, 1.0, 3.0]
    raw: list[dict[str, Any]] = []
    poisson_config = replace(config, event_generation_mode="poisson_cells")
    for factor in factors:
        run_config = replace(poisson_config, lambda_model=true_rate * factor)
        for seed in seeds:
            result = run_simulation(run_config, Strategy(StrategyName.MARKET, weights), seed)
            raw.append({"lambda_factor": factor, **result.metrics})
    summary: list[dict[str, Any]] = []
    for factor in factors:
        rows = [row for row in raw if row["lambda_factor"] == factor]
        summary.append(
            {
                "lambda_factor": factor,
                "mean_detection_latency": statistics.fmean(
                    float(row["mean_detection_latency"]) for row in rows
                ),
                "detection_rate": statistics.fmean(float(row["detection_rate"]) for row in rows),
                "completion_rate": statistics.fmean(float(row["completion_rate"]) for row in rows),
                "max_cell_idleness": statistics.fmean(float(row["max_cell_idleness"]) for row in rows),
            }
        )
    output = Path(output_dir)
    write_rows(output / "robustness_runs.csv", raw)
    write_rows(output / "robustness_summary.csv", summary)
    return summary
