"""Pure bid and preemption calculations with comparable normalised terms."""

from __future__ import annotations

from dataclasses import dataclass
from math import dist, sqrt

from .tasks import Task


@dataclass(frozen=True, slots=True)
class BidWeights:
    alpha: float = 1.0
    beta: float = 0.6
    gamma: float = 0.8
    delta: float = 1.0
    theta_hyst: float = 0.1

    def __post_init__(self) -> None:
        if self.alpha <= 0.0 or min(self.beta, self.gamma, self.delta, self.theta_hyst) < 0.0:
            raise ValueError("bid weights must be non-negative and alpha positive")


def compute_cost(
    robot_pose: tuple[float, float],
    battery: float,
    workload: int,
    task: Task,
    weights: BidWeights,
    d_max: float,
    workload_cap: int = 3,
) -> float:
    """Return bid cost; a lower value is better."""

    if d_max <= 0.0 or workload_cap <= 0:
        raise ValueError("normalisation constants must be positive")
    distance_term = min(dist(robot_pose, task.position) / d_max, 1.0)
    workload_term = min(workload / workload_cap, 1.0)
    battery_term = 1.0 - min(1.0, max(0.0, battery))
    return weights.alpha * distance_term + weights.beta * workload_term + weights.gamma * battery_term


def should_preempt(
    current_task: Task,
    new_task: Task,
    current_cost: float,
    new_cost: float,
    now: float,
    weights: BidWeights,
) -> bool:
    """Apply cross-auction priority gain with hysteresis."""

    priority_gain = new_task.priority_fn.at(now) - current_task.priority_fn.at(now)
    cost_increase = new_cost - current_cost
    return weights.delta * priority_gain - cost_increase > weights.theta_hyst


def arena_diagonal(width: float, height: float) -> float:
    return sqrt(width * width + height * height)
