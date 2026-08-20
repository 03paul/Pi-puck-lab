"""Shared scoring and CSV helpers for experiments."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any


def quality_score(metrics: dict[str, Any], duration: float, robot_count: int) -> float:
    """Transparent operational utility used only to select one Pareto point."""

    bounded = lambda value: min(1.0, max(0.0, float(value)))
    return (
        0.30 * bounded(metrics["completion_rate"])
        + 0.10 * bounded(metrics["detection_rate"])
        + 0.10 * (1.0 - bounded(metrics["mean_completion_time"] / duration))
        + 0.15 * (1.0 - bounded(metrics["load_imbalance"] / max(1.0, robot_count / 2.0)))
        + 0.05 * bounded(metrics["coverage_fraction"])
        + 0.08 * (1.0 - bounded(metrics["mean_cell_idleness"] / duration))
        + 0.07 * (1.0 - bounded(metrics["mean_detection_latency"] / duration))
        + 0.05 * bounded(metrics["mean_final_battery"])
        + 0.10 * (1.0 - bounded(metrics["depleted_robots"] / robot_count))
    )


def write_rows(path: str | Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        raise ValueError("cannot write an empty table")
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def read_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))
