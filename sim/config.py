"""Typed simulator parameters with JSON loading."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class SimulationConfig:
    arena_width: float = 2.0
    arena_height: float = 2.0
    duration: float = 300.0
    dt: float = 0.1
    robot_count: int = 7
    roi_event_count: int = 28
    event_generation_mode: str = "fixed_hotspots"
    detection_radius: float = 0.30
    effective_speed: float = 0.08
    startup_delay: float = 1.2
    arrival_tolerance: float = 0.035
    initial_battery_min: float = 0.26
    initial_battery_max: float = 1.0
    battery_idle_drain: float = 0.00008
    battery_move_drain_per_m: float = 0.0094
    depleted_threshold: float = 0.05
    workload_cap: int = 3
    bid_window: float = 1.0
    award_timeout: float = 3.0
    max_auction_retries: int = 3
    assignment_timeout: float = 90.0
    preemption_cooldown: float = 15.0
    latency_min: float = 0.005
    latency_max: float = 0.040
    packet_loss: float = 0.02
    communication_range: float | None = None
    coverage_cell: float = 0.05
    survey_cell: float = 0.40
    survey_announce_interval: float = 2.0
    survey_cooldown: float = 18.0
    survey_priority_threshold: float = 0.12
    max_survey_announcements: int = 3
    lambda_true: float = 0.00693
    lambda_model: float = 0.00693
    priority_k: float = 1.0
    roi_aging_lambda: float = 0.00693
    priority_push: float = 0.4
    priority_surround: float = 0.8
    priority_guard: float = 0.6
    state_log_interval: float = 0.5

    def __post_init__(self) -> None:
        if min(self.arena_width, self.arena_height, self.duration, self.dt, self.effective_speed) <= 0.0:
            raise ValueError("arena, duration, timestep and speed must be positive")
        if self.robot_count < 1 or self.workload_cap < 1:
            raise ValueError("robot_count and workload_cap must be positive")
        if not 0.0 <= self.packet_loss < 1.0:
            raise ValueError("packet_loss must be in [0, 1)")
        if self.event_generation_mode not in {"fixed_hotspots", "poisson_cells"}:
            raise ValueError("event_generation_mode must be fixed_hotspots or poisson_cells")

    @classmethod
    def from_json(cls, path: str | Path, overrides: dict[str, Any] | None = None) -> SimulationConfig:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        config_data = data.get("simulation", data)
        allowed = {field.name for field in fields(cls)}
        unknown = set(config_data) - allowed
        if unknown:
            raise ValueError(f"unknown simulation parameters: {sorted(unknown)}")
        merged = {**config_data, **(overrides or {})}
        return cls(**merged)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
