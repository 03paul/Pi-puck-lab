from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from random import Random

from allocation.bidding import BidWeights
from allocation.messages import announce_message
from allocation.strategies import Strategy, StrategyName
from allocation.tasks import PriorityFn, Task, TaskType
from experiments.interactive_replay import create_interactive_replay
from experiments.replay import create_replay
from metrics.logger import RunLogger
from sim.config import SimulationConfig
from sim.robot import Robot, RobotState
from sim.world import EventSpec, World, run_simulation


class SimulationInvariantTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = SimulationConfig(duration=90.0, roi_event_count=10, packet_loss=0.0)
        cls.strategy = Strategy(StrategyName.MARKET, BidWeights(beta=0.5, gamma=0.0, delta=1.0))
        cls.result = run_simulation(cls.config, cls.strategy, seed=17)

    def test_conservation(self) -> None:
        self.assertEqual(self.result.metrics["conservation_error"], 0)

    def test_capacity(self) -> None:
        self.assertLessEqual(self.result.metrics["max_observed_workload"], self.config.workload_cap)

    def test_n_req_exact_during_execution(self) -> None:
        self.assertEqual(self.result.metrics["n_req_violations"], 0)

    def test_metric_ranges(self) -> None:
        self.assertGreaterEqual(self.result.metrics["completion_rate"], 0.0)
        self.assertLessEqual(self.result.metrics["completion_rate"], 1.0)
        self.assertGreaterEqual(self.result.metrics["coverage_fraction"], 0.0)
        self.assertLessEqual(self.result.metrics["coverage_fraction"], 1.0)
        self.assertGreaterEqual(self.result.metrics["min_final_battery"], 0.0)

    def test_deterministic_event_log_hash(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            first = run_simulation(self.config, self.strategy, 99, Path(directory) / "a")
            second = run_simulation(self.config, self.strategy, 99, Path(directory) / "b")
            self.assertEqual(first.events_hash, second.events_hash)

    def test_allocation_imports_are_platform_independent(self) -> None:
        root = Path(__file__).resolve().parents[1] / "allocation"
        forbidden = ("sim", "backends", "metrics")
        for path in root.glob("*.py"):
            source = path.read_text(encoding="utf-8")
            self.assertFalse(
                any(f"from {name}" in source or f"import {name}" in source for name in forbidden)
            )

    def test_preemption_is_forbidden_after_team_formation(self) -> None:
        config = SimulationConfig(duration=10.0, packet_loss=0.0)
        strategy = Strategy(StrategyName.MARKET, BidWeights(beta=0.5, gamma=0.0, delta=2.0))
        current = Task("old", TaskType.GUARD, (1.0, 1.0), 1, PriorityFn(0.1, 0.0, 1.0, 0.0), 1.0, 0.0)
        urgent = Task("urgent", TaskType.SURROUND, (0.0, 0.0), 3, PriorityFn(0.95, 0.0, 1.0, 0.0), 1.0, 0.0)
        for state in (RobotState.WAIT_PEERS, RobotState.EXECUTE):
            robot = Robot("r00", (0.0, 0.0), 1.0, strategy, Random(1), config)
            robot.known_tasks[current.task_id] = current
            robot.assignments.append(current.task_id)
            robot.state = state
            robot.receive([announce_message(urgent, 1.0, "r01", 0.0).to_bytes()], 0.0)
            self.assertNotIn(urgent.task_id, robot.preempt_intent)

    def test_trivial_one_robot_case_matches_all_strategies(self) -> None:
        metrics = []
        config = SimulationConfig(
            duration=30.0,
            robot_count=1,
            roi_event_count=0,
            packet_loss=0.0,
            survey_priority_threshold=1.0,
        )
        for name in (StrategyName.MARKET, StrategyName.GREEDY, StrategyName.RANDOM):
            world = World(config, Strategy(name, BidWeights(beta=0.5)), seed=5)
            position = world.robots[0].position
            world.events = [EventSpec("event-000", 0.0, position, TaskType.GUARD, 1, 15.0)]
            metrics.append(world.run().metrics)
        self.assertEqual([row["roi_completed"] for row in metrics], [1, 1, 1])
        self.assertEqual(len({row["mean_completion_time"] for row in metrics}), 1)

    def test_no_starvation_in_sufficient_capacity_run(self) -> None:
        config = SimulationConfig(duration=300.0, roi_event_count=4, packet_loss=0.0)
        result = run_simulation(config, self.strategy, seed=3)
        self.assertEqual(result.metrics["roi_open_end"], 0)

    def test_replay_is_rendered_from_portable_logs(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            logger = RunLogger()
            for t, x in ((0.0, 0.1), (1.0, 0.2)):
                logger.state(t, "r00", x, 0.1, 1.0, 0, "EXPLORE")
            logger.write(directory)
            output = create_replay(directory, Path(directory) / "replay.gif", speed=10.0, fps=2, dpi=50)
            self.assertTrue(output.exists())
            self.assertGreater(output.stat().st_size, 100)

    def test_interactive_replay_embeds_logs_and_controls(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            logger = RunLogger()
            logger.state(0.0, "r00", 0.1, 0.1, 1.0, 0, "EXPLORE")
            logger.state(1.0, "r00", 0.2, 0.2, 0.9, 0, "NAVIGATE")
            logger.write(directory)
            output = create_interactive_replay(directory, Path(directory) / "replay.html")
            document = output.read_text(encoding="utf-8")
            self.assertNotIn("__REPLAY_DATA__", document)
            self.assertIn('id="play-toggle"', document)
            self.assertIn('id="timeline"', document)
            self.assertIn('"id":"r00"', document)


if __name__ == "__main__":
    unittest.main()
