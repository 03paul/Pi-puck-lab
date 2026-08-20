"""Stable two-CSV logger required by all execution tiers."""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

EVENT_FIELDS = ["t", "event_type", "task_id", "robot_id", "payload_json"]
STATE_FIELDS = ["t", "robot_id", "x", "y", "battery", "workload", "state"]


@dataclass(slots=True)
class RunLogger:
    events: list[dict[str, Any]] = field(default_factory=list)
    states: list[dict[str, Any]] = field(default_factory=list)

    def event(
        self,
        t: float,
        event_type: str,
        task_id: str = "",
        robot_id: str = "",
        payload: dict[str, Any] | None = None,
    ) -> None:
        self.events.append(
            {
                "t": f"{t:.3f}",
                "event_type": event_type,
                "task_id": task_id,
                "robot_id": robot_id,
                "payload_json": json.dumps(payload or {}, separators=(",", ":"), sort_keys=True),
            }
        )

    def state(
        self,
        t: float,
        robot_id: str,
        x: float,
        y: float,
        battery: float,
        workload: int,
        state: str,
    ) -> None:
        self.states.append(
            {
                "t": f"{t:.3f}",
                "robot_id": robot_id,
                "x": f"{x:.6f}",
                "y": f"{y:.6f}",
                "battery": f"{battery:.6f}",
                "workload": workload,
                "state": state,
            }
        )

    def write(self, directory: str | Path) -> tuple[Path, Path]:
        output = Path(directory)
        output.mkdir(parents=True, exist_ok=True)
        events_path = output / "events.csv"
        states_path = output / "state.csv"
        with events_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, EVENT_FIELDS, lineterminator="\n")
            writer.writeheader()
            writer.writerows(self.events)
        with states_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, STATE_FIELDS, lineterminator="\n")
            writer.writeheader()
            writer.writerows(self.states)
        return events_path, states_path
