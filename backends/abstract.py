"""Minimal adapter used by Tier 0; Webots and Pi-Puck can match this API later."""

from __future__ import annotations

from dataclasses import dataclass, field

from .base import Detection


@dataclass(slots=True)
class AbstractBackend:
    robot_id: str
    pose: tuple[float, float, float]
    battery: float
    clock: float = 0.0
    target: tuple[float, float] | None = None
    inbox: list[bytes] = field(default_factory=list)
    outbox: list[bytes] = field(default_factory=list)
    detections: list[Detection] = field(default_factory=list)

    def get_pose(self) -> tuple[float, float, float]:
        return self.pose

    def get_battery(self) -> float:
        return self.battery

    def drive_to(self, x: float, y: float) -> None:
        self.target = (x, y)

    def is_at_target(self) -> bool:
        return self.target is None or self.pose[:2] == self.target

    def broadcast(self, msg: bytes) -> None:
        self.outbox.append(msg)

    def receive(self) -> list[bytes]:
        messages, self.inbox = self.inbox, []
        return messages

    def detect_rois(self) -> list[Detection]:
        detections, self.detections = self.detections, []
        return detections

    def now(self) -> float:
        return self.clock
