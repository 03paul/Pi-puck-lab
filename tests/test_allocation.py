from __future__ import annotations

import math
import random
import unittest
from itertools import pairwise

from allocation.auction import HostedAuction
from allocation.bidding import BidWeights, compute_cost, should_preempt
from allocation.messages import Message, announce_message
from allocation.strategies import Strategy, StrategyName
from allocation.tasks import PriorityFn, Task, TaskType


class AllocationTests(unittest.TestCase):
    def task(self, task_type: TaskType = TaskType.GUARD) -> Task:
        return Task("t", task_type, (1.0, 1.0), 1, PriorityFn(0.4, 0.01, 1.0, 0.0), 10.0, 0.0)

    def test_priority_bounds_and_monotonicity_property(self) -> None:
        rng = random.Random(11)
        for _ in range(500):
            p0 = rng.random()
            fn = PriorityFn(p0, rng.random(), rng.uniform(0.1, 3.0), rng.uniform(0.0, 50.0))
            times = sorted(rng.uniform(fn.t_ref, fn.t_ref + 1000.0) for _ in range(20))
            values = [fn.at(t) for t in times]
            self.assertTrue(all(p0 <= value <= 1.0 for value in values))
            self.assertTrue(all(left <= right for left, right in pairwise(values)))

    def test_cost_directions_and_non_negative(self) -> None:
        task = self.task()
        weights = BidWeights(beta=1.0, gamma=1.0)
        near = compute_cost((0.9, 0.9), 1.0, 0, task, weights, math.sqrt(8.0))
        far = compute_cost((0.0, 0.0), 1.0, 0, task, weights, math.sqrt(8.0))
        loaded = compute_cost((0.9, 0.9), 1.0, 3, task, weights, math.sqrt(8.0))
        empty_battery = compute_cost((0.9, 0.9), 0.0, 0, task, weights, math.sqrt(8.0))
        self.assertGreater(far, near)
        self.assertGreater(loaded, near)
        self.assertGreater(empty_battery, near)
        self.assertGreaterEqual(near, 0.0)

    def test_greedy_equivalence(self) -> None:
        task = self.task()
        market = Strategy(StrategyName.MARKET, BidWeights(beta=0.0, gamma=0.0, delta=0.0))
        greedy = Strategy(StrategyName.GREEDY, BidWeights())
        poses = [(0.1, 0.2), (1.5, 1.5), (0.8, 1.0)]
        market_costs = [market.cost(pose, 0.2, 3, task, 3.0, random.Random(1)) for pose in poses]
        greedy_costs = [greedy.cost(pose, 0.9, 0, task, 3.0, random.Random(2)) for pose in poses]
        self.assertEqual(market_costs, greedy_costs)
        self.assertEqual(
            min(range(3), key=market_costs.__getitem__), min(range(3), key=greedy_costs.__getitem__)
        )

    def test_message_round_trip_is_lossless_and_compact(self) -> None:
        message = announce_message(self.task(), 1.0, "r00", 0.0)
        payload = message.to_bytes()
        self.assertEqual(message, Message.from_bytes(payload))
        self.assertNotIn(b" ", payload)

    def test_auction_selects_lowest_cost_with_id_tie_break(self) -> None:
        auction = HostedAuction("t", 2, 1.0)
        auction.submit("r02", 0.2, 0.5)
        auction.submit("r01", 0.2, 0.5)
        auction.submit("r03", 0.8, 0.5)
        self.assertEqual(auction.winners(), ["r01", "r02"])

    def test_preemption_uses_priority_across_auctions(self) -> None:
        current = Task("old", TaskType.GUARD, (0.0, 0.0), 1, PriorityFn(0.2, 0.0, 1.0, 0.0), 1.0, 0.0)
        urgent = Task("new", TaskType.SURROUND, (0.0, 0.0), 3, PriorityFn(0.9, 0.0, 1.0, 0.0), 1.0, 0.0)
        self.assertTrue(should_preempt(current, urgent, 0.2, 0.3, 1.0, BidWeights(delta=1.0, theta_hyst=0.1)))

    def test_survey_does_not_count_toward_workload(self) -> None:
        self.assertFalse(self.task(TaskType.SURVEY).counts_toward_workload)
        self.assertTrue(self.task(TaskType.GUARD).counts_toward_workload)


if __name__ == "__main__":
    unittest.main()
