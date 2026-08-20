"""The eight capabilities required by allocation code on every tier."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True, slots=True)
class Detection:
    detection_id: str
    position: tuple[float, float]
    kind: str


class RobotBackend(Protocol):
    def get_pose(self) -> tuple[float, float, float]: ...

    def get_battery(self) -> float: ...

    def drive_to(self, x: float, y: float) -> None: ...

    def is_at_target(self) -> bool: ...

    def broadcast(self, msg: bytes) -> None: ...

    def receive(self) -> list[bytes]: ...

    def detect_rois(self) -> list[Detection]: ...

    def now(self) -> float: ...
