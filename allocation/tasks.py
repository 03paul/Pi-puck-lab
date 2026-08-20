"""Immutable task definitions shared by simulation and robot backends."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import exp


class TaskType(str, Enum):
    PUSH = "push"
    SURROUND = "surround"
    GUARD = "guard"
    SURVEY = "survey"


class TaskState(str, Enum):
    OPEN = "open"
    BIDDING = "bidding"
    ASSIGNED = "assigned"
    ACTIVE = "active"
    DONE = "done"
    FAILED = "failed"


@dataclass(frozen=True, slots=True)
class PriorityFn:
    """Bounded aging: p(dt)=p0+(1-p0)*(1-exp(-(lam*dt)**k))."""

    p0: float
    lam: float
    k: float
    t_ref: float

    def __post_init__(self) -> None:
        if not 0.0 <= self.p0 <= 1.0:
            raise ValueError("p0 must be in [0, 1]")
        if self.lam < 0.0 or self.k <= 0.0:
            raise ValueError("lam must be non-negative and k positive")

    def at(self, t: float) -> float:
        dt = max(0.0, t - self.t_ref)
        value = self.p0 + (1.0 - self.p0) * (1.0 - exp(-((self.lam * dt) ** self.k)))
        return min(1.0, max(self.p0, value))


@dataclass(frozen=True, slots=True)
class Task:
    task_id: str
    task_type: TaskType
    position: tuple[float, float]
    n_req: int
    priority_fn: PriorityFn
    duration: float
    announced_at: float

    def __post_init__(self) -> None:
        if self.n_req < 1:
            raise ValueError("n_req must be positive")
        if self.duration < 0.0:
            raise ValueError("duration must be non-negative")

    @property
    def counts_toward_workload(self) -> bool:
        return self.task_type is not TaskType.SURVEY
