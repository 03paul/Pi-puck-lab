"""Recompute portable metrics exclusively from events.csv and state.csv."""

from __future__ import annotations

import csv
import json
import statistics
from collections import defaultdict
from math import ceil, dist
from pathlib import Path


def analyse_logs(
    events_path: str | Path,
    state_path: str | Path,
    arena_width: float = 2.0,
    arena_height: float = 2.0,
    coverage_cell: float = 0.05,
    detection_radius: float = 0.30,
) -> dict[str, float | int]:
    with Path(events_path).open(newline="", encoding="utf-8") as stream:
        events = list(csv.DictReader(stream))
    with Path(state_path).open(newline="", encoding="utf-8") as stream:
        states = list(csv.DictReader(stream))

    announced: dict[str, float] = {}
    task_types: dict[str, str] = {}
    done: dict[str, float] = {}
    awards: dict[str, list[str]] = {}
    messages = 0
    message_bytes = 0
    completed_by_robot: dict[str, int] = defaultdict(int)
    for row in events:
        event_type = row["event_type"]
        payload = json.loads(row["payload_json"])
        if event_type in {"ANNOUNCE", "BID", "AWARD", "DONE", "RELEASE"}:
            messages += 1
            message_bytes += len(
                json.dumps({"kind": event_type, **payload}, separators=(",", ":"), sort_keys=True).encode()
            )
        if event_type == "ANNOUNCE" and row["task_id"] not in announced:
            announced[row["task_id"]] = float(row["t"])
            task_types[row["task_id"]] = str(payload.get("type", ""))
        elif event_type == "AWARD":
            awards[row["task_id"]] = [str(robot_id) for robot_id in payload.get("winners", [])]
        elif event_type == "DONE":
            done[row["task_id"]] = float(row["t"])
            if task_types.get(row["task_id"]) != "survey":
                for robot_id in awards.get(row["task_id"], [row["robot_id"]]):
                    completed_by_robot[robot_id] += 1

    roi_done = [task_id for task_id in done if task_types.get(task_id) != "survey"]
    completion_times = [done[task_id] - announced[task_id] for task_id in roi_done if task_id in announced]
    cols = ceil(arena_width / coverage_cell)
    rows_count = ceil(arena_height / coverage_cell)
    covered: set[int] = set()
    for row in states:
        x, y = float(row["x"]), float(row["y"])
        col_min = max(0, int((x - detection_radius) / coverage_cell))
        col_max = min(cols - 1, int((x + detection_radius) / coverage_cell))
        row_min = max(0, int((y - detection_radius) / coverage_cell))
        row_max = min(rows_count - 1, int((y + detection_radius) / coverage_cell))
        for grid_row in range(row_min, row_max + 1):
            for col in range(col_min, col_max + 1):
                centre = ((col + 0.5) * coverage_cell, (grid_row + 0.5) * coverage_cell)
                if dist((x, y), centre) <= detection_radius:
                    covered.add(grid_row * cols + col)
    robot_ids = sorted({row["robot_id"] for row in states})
    loads = [completed_by_robot[robot_id] for robot_id in robot_ids]
    return {
        "roi_completed": len(roi_done),
        "mean_completion_time": statistics.fmean(completion_times) if completion_times else 0.0,
        "median_completion_time": statistics.median(completion_times) if completion_times else 0.0,
        "coverage_fraction": len(covered) / (cols * rows_count),
        "load_imbalance": statistics.pstdev(loads) if len(loads) > 1 else 0.0,
        "messages_per_completed": messages / max(1, len(roi_done)),
        "bytes_per_completed": message_bytes / max(1, len(roi_done)),
    }
