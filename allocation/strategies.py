"""Bid policies sharing the same auction/message protocol."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from random import Random

from .bidding import BidWeights, compute_cost
from .tasks import Task


class StrategyName(str, Enum):
    MARKET = "market"
    GREEDY = "nearest_greedy"
    GREEDY_NO_SURVEY = "nearest_greedy_no_survey"
    RANDOM = "random_assignment"


@dataclass(frozen=True, slots=True)
class Strategy:
    name: StrategyName
    weights: BidWeights

    @property
    def survey_enabled(self) -> bool:
        return self.name is not StrategyName.GREEDY_NO_SURVEY

    @property
    def preemption_enabled(self) -> bool:
        return self.name is StrategyName.MARKET

    def cost(
        self,
        pose: tuple[float, float],
        battery: float,
        workload: int,
        task: Task,
        d_max: float,
        rng: Random,
    ) -> float:
        if self.name is StrategyName.RANDOM:
            return rng.random()
        weights = self.weights
        if self.name in {StrategyName.GREEDY, StrategyName.GREEDY_NO_SURVEY}:
            weights = BidWeights(alpha=1.0, beta=0.0, gamma=0.0, delta=0.0, theta_hyst=weights.theta_hyst)
        return compute_cost(pose, battery, workload, task, weights, d_max)
