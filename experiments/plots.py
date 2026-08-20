"""Report-ready plots from experiment CSV outputs."""

from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

LABELS = {
    "market": "Market",
    "nearest_greedy": "Nearest Greedy",
    "nearest_greedy_no_survey": "Greedy (no SURVEY)",
    "random_assignment": "Random",
}


def make_plots(results_dir: str | Path) -> list[Path]:
    output = Path(results_dir)
    with (output / "run_metrics.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        grouped[row["strategy"]].append(row)
    order = [key for key in LABELS if key in grouped]
    metrics = [
        ("completion_rate", "Completion rate", True),
        ("mean_completion_time", "Completion time [s]", False),
        ("load_imbalance", "Load imbalance [tasks]", False),
        ("max_cell_idleness", "Max. cell idleness [s]", False),
        ("mean_detection_latency", "Detection latency [s]", False),
        ("messages_per_completed", "Messages / completed task", False),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(12, 7), constrained_layout=True)
    colors = ["#136f63", "#d97706", "#7c3aed", "#64748b"]
    for axis, (metric, title, _) in zip(axes.flat, metrics):
        values = [np.array([float(row[metric]) for row in grouped[strategy]]) for strategy in order]
        means = [array.mean() for array in values]
        errors = [1.96 * array.std(ddof=1) / np.sqrt(len(array)) for array in values]
        axis.bar(range(len(order)), means, yerr=errors, color=colors[: len(order)], capsize=4)
        axis.set_title(title)
        axis.set_xticks(range(len(order)), [LABELS[key] for key in order], rotation=22, ha="right")
        axis.grid(axis="y", alpha=0.25)
    comparison_path = output / "comparison.png"
    fig.savefig(comparison_path, dpi=180)
    plt.close(fig)

    with (output / "sweep_summary.csv").open(newline="", encoding="utf-8") as stream:
        sweep = list(csv.DictReader(stream))
    pareto_keys: set[tuple[str, str, str, str]] = set()
    with (output / "pareto_parameters.csv").open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            pareto_keys.add((row["beta"], row["gamma"], row["delta"], row["theta_hyst"]))
    fig, axis = plt.subplots(figsize=(7.5, 5.2), constrained_layout=True)
    for row in sweep:
        key = (row["beta"], row["gamma"], row["delta"], row["theta_hyst"])
        on_front = key in pareto_keys
        axis.scatter(
            float(row["mean_completion_time"]),
            float(row["load_imbalance"]),
            s=80 if on_front else 28,
            c="#136f63" if on_front else "#94a3b8",
            alpha=0.95 if on_front else 0.55,
            edgecolors="white",
            linewidths=0.5,
        )
    axis.set_xlabel("Mean completion time [s] (lower is better)")
    axis.set_ylabel("Load imbalance [tasks] (lower is better)")
    axis.set_title("Market parameter sweep and Pareto points")
    axis.grid(alpha=0.25)
    pareto_path = output / "pareto.png"
    fig.savefig(pareto_path, dpi=180)
    plt.close(fig)
    return [comparison_path, pareto_path]
