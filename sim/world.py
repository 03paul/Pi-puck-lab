"""Tier-0 world; robots decide assignments exclusively through messages."""

from __future__ import annotations

import hashlib
import json
import statistics
from dataclasses import dataclass
from math import ceil, dist
from pathlib import Path
from random import Random
from typing import Callable

from allocation.messages import Message, MessageType, done_message, release_message
from allocation.strategies import Strategy
from allocation.tasks import PriorityFn, Task, TaskState, TaskType
from metrics.logger import RunLogger

from backends.base import RobotBackend

from .comms import MessageBus
from .config import SimulationConfig
from .robot import Robot


@dataclass(frozen=True, slots=True)
class EventSpec:
    event_id: str
    release_at: float
    position: tuple[float, float]
    task_type: TaskType
    n_req: int
    duration: float


@dataclass(slots=True)
class TaskRuntime:
    task: Task
    state: TaskState
    host: str
    source_event_id: str | None = None
    survey_cell: int | None = None
    winners: list[str] | None = None
    award_at: float | None = None
    active_at: float | None = None
    done_at: float | None = None
    failed_at: float | None = None


@dataclass(slots=True)
class SimulationResult:
    seed: int
    strategy: str
    metrics: dict[str, float | int | str]
    logger: RunLogger | None = None
    events_hash: str | None = None


class World:
    def __init__(
        self,
        config: SimulationConfig,
        strategy: Strategy,
        seed: int,
        capture_logs: bool = False,
        backend_factory: Callable[[str], RobotBackend] | None = None,
    ) -> None:
        self.config = config
        self.strategy = strategy
        self.seed = seed
        self.scenario_rng = Random(seed)
        self.comm_rng = Random(seed ^ 0xA5A5_7E57)
        self.logger = RunLogger() if capture_logs else None
        self.now = 0.0
        # Tier 1/2 hook: when set, called once per robot_id to build its movement backend
        # (see backends/webots.py). Default None preserves Tier 0 exactly - every Robot gets
        # backend=None, unchanged from before this parameter existed. self.now stays a plain
        # public attribute deliberately: a Webots-driving caller can set world.now =
        # supervisor.getTime() before each world.step() instead of using World.run()'s
        # self-incrementing loop, without needing any further change here.
        self.backend_factory = backend_factory
        robot_ids = [f"r{i:02d}" for i in range(config.robot_count)]
        self.robots = self._make_robots(robot_ids)
        self.bus = MessageBus(
            robot_ids,
            self.comm_rng,
            config.latency_min,
            config.latency_max,
            config.packet_loss,
            config.communication_range,
        )
        self.events = self._make_events()
        self.detected_events: set[str] = set()
        self.detection_latencies: list[float] = []
        self.tasks: dict[str, TaskRuntime] = {}
        self.survey_cols = ceil(config.arena_width / config.survey_cell)
        self.survey_rows = ceil(config.arena_height / config.survey_cell)
        self.survey_centres = self._grid_centres(config.survey_cell, self.survey_cols, self.survey_rows)
        self.survey_last_seen = [0.0 for _ in self.survey_centres]
        self.survey_cooldown_until = [0.0 for _ in self.survey_centres]
        self.active_survey_cells: dict[int, str] = {}
        self.survey_counter = 0
        self.coverage_cols = ceil(config.arena_width / config.coverage_cell)
        self.coverage_rows = ceil(config.arena_height / config.coverage_cell)
        self.covered: set[int] = set()
        self.idleness_sum = 0.0
        self.idleness_samples = 0
        self.max_idleness = 0.0
        self.next_survey_announcement = 0.0
        self.next_state_log = 0.0
        self.first_depletion_time: float | None = None
        self.max_observed_workload = 0
        self.n_req_violations = 0
        self.explore_indices = [0 for _ in self.robots]
        self.explore_waypoints = [self._sector_waypoints(index) for index in range(len(self.robots))]

    def _make_robots(self, robot_ids: list[str]) -> list[Robot]:
        positions: list[tuple[float, float]] = []
        for index in range(len(robot_ids)):
            angle_slot = index / max(1, len(robot_ids) - 1)
            positions.append((0.12 + 1.76 * angle_slot, 0.12 + 0.08 * (index % 2)))
        batteries = [
            self.config.initial_battery_min
            + (self.config.initial_battery_max - self.config.initial_battery_min)
            * index
            / max(1, len(robot_ids) - 1)
            for index in range(len(robot_ids))
        ]
        self.scenario_rng.shuffle(batteries)
        return [
            Robot(
                robot_id,
                positions[index],
                batteries[index],
                self.strategy,
                Random(self.seed * 10_007 + index * 97 + 31),
                self.config,
                self.backend_factory(robot_id) if self.backend_factory is not None else None,
            )
            for index, robot_id in enumerate(robot_ids)
        ]

    def _make_events(self) -> list[EventSpec]:
        if self.config.event_generation_mode == "poisson_cells":
            return self._make_poisson_events()
        hotspots = [
            (self.scenario_rng.uniform(0.25, 1.75), self.scenario_rng.uniform(0.35, 1.75)) for _ in range(3)
        ]
        specs: list[EventSpec] = []
        types = [TaskType.GUARD, TaskType.GUARD, TaskType.PUSH, TaskType.SURROUND]
        for index in range(self.config.roi_event_count):
            release_at = self.scenario_rng.uniform(5.0, self.config.duration * 0.78)
            centre = self.scenario_rng.choice(hotspots)
            if self.scenario_rng.random() < 0.78:
                x = min(1.94, max(0.06, self.scenario_rng.gauss(centre[0], 0.18)))
                y = min(1.94, max(0.06, self.scenario_rng.gauss(centre[1], 0.18)))
            else:
                x = self.scenario_rng.uniform(0.06, self.config.arena_width - 0.06)
                y = self.scenario_rng.uniform(0.06, self.config.arena_height - 0.06)
            task_type = self.scenario_rng.choice(types)
            n_req = 1 if task_type is TaskType.GUARD else (3 if task_type is TaskType.SURROUND else 2)
            duration = {TaskType.GUARD: 15.0, TaskType.PUSH: 20.0, TaskType.SURROUND: 10.0}[task_type]
            specs.append(EventSpec(f"event-{index:03d}", release_at, (x, y), task_type, n_req, duration))
        return sorted(specs, key=lambda event: (event.release_at, event.event_id))

    def _make_poisson_events(self) -> list[EventSpec]:
        cols = ceil(self.config.arena_width / self.config.survey_cell)
        rows = ceil(self.config.arena_height / self.config.survey_cell)
        centres = self._grid_centres(self.config.survey_cell, cols, rows)
        raw: list[tuple[float, tuple[float, float]]] = []
        horizon = self.config.duration * 0.78
        if self.config.lambda_true <= 0.0:
            return []
        for centre in centres:
            occurrence = 5.0 + self.scenario_rng.expovariate(self.config.lambda_true)
            while occurrence <= horizon:
                jitter = self.config.survey_cell * 0.35
                position = (
                    min(
                        self.config.arena_width - 0.04,
                        max(0.04, centre[0] + self.scenario_rng.uniform(-jitter, jitter)),
                    ),
                    min(
                        self.config.arena_height - 0.04,
                        max(0.04, centre[1] + self.scenario_rng.uniform(-jitter, jitter)),
                    ),
                )
                raw.append((occurrence, position))
                occurrence += self.scenario_rng.expovariate(self.config.lambda_true)
        raw.sort(key=lambda item: item[0])
        if self.config.roi_event_count > 0:
            raw = raw[: self.config.roi_event_count]
        types = [TaskType.GUARD, TaskType.GUARD, TaskType.PUSH, TaskType.SURROUND]
        specs: list[EventSpec] = []
        for index, (release_at, position) in enumerate(raw):
            task_type = self.scenario_rng.choice(types)
            n_req = 1 if task_type is TaskType.GUARD else (3 if task_type is TaskType.SURROUND else 2)
            duration = {TaskType.GUARD: 15.0, TaskType.PUSH: 20.0, TaskType.SURROUND: 10.0}[task_type]
            specs.append(EventSpec(f"event-{index:03d}", release_at, position, task_type, n_req, duration))
        return specs

    def _grid_centres(self, cell: float, cols: int, rows: int) -> list[tuple[float, float]]:
        return [
            (
                min(self.config.arena_width, (col + 0.5) * cell),
                min(self.config.arena_height, (row + 0.5) * cell),
            )
            for row in range(rows)
            for col in range(cols)
        ]

    def _sector_waypoints(self, robot_index: int) -> list[tuple[float, float]]:
        left = robot_index * self.config.arena_width / len(self.robots)
        right = (robot_index + 1) * self.config.arena_width / len(self.robots)
        x_a, x_b = left + 0.03, right - 0.03
        rows = max(2, ceil(self.config.arena_height / (2.0 * self.config.detection_radius)))
        waypoints: list[tuple[float, float]] = []
        for row in range(rows):
            y = min(self.config.arena_height - 0.05, 0.05 + row * self.config.arena_height / max(1, rows - 1))
            waypoints.append((x_a if row % 2 == 0 else x_b, y))
            waypoints.append((x_b if row % 2 == 0 else x_a, y))
        return waypoints

    def _positions(self) -> dict[str, tuple[float, float]]:
        return {robot.robot_id: robot.position for robot in self.robots}

    def _emit(self, message: Message) -> None:
        if self.logger is not None:
            payload = dict(message.fields)
            self.logger.event(
                self.now,
                message.kind.value,
                str(message.fields["task_id"]),
                str(message.fields["sender"]),
                payload,
            )
        if message.kind is MessageType.ANNOUNCE:
            task_id = str(message.fields["task_id"])
            if task_id in self.tasks and self.tasks[task_id].state not in {TaskState.DONE, TaskState.FAILED}:
                runtime = self.tasks[task_id]
                runtime.state = TaskState.BIDDING
                runtime.winners = None
                runtime.award_at = None
        self.bus.broadcast(message, self.now, self._positions())

    def _announce_event(self, event: EventSpec, detector: Robot) -> None:
        p0 = {
            TaskType.PUSH: self.config.priority_push,
            TaskType.SURROUND: self.config.priority_surround,
            TaskType.GUARD: self.config.priority_guard,
        }[event.task_type]
        task = Task(
            event.event_id.replace("event", "roi"),
            event.task_type,
            event.position,
            event.n_req,
            PriorityFn(p0, self.config.roi_aging_lambda, self.config.priority_k, self.now),
            event.duration,
            self.now,
        )
        self.tasks[task.task_id] = TaskRuntime(task, TaskState.BIDDING, detector.robot_id, event.event_id)
        self.detected_events.add(event.event_id)
        self.detection_latencies.append(self.now - event.release_at)
        if self.logger is not None:
            self.logger.event(
                self.now,
                "DETECTED",
                task.task_id,
                detector.robot_id,
                {"event_created_at": event.release_at, "latency": self.now - event.release_at},
            )
        self._emit(detector.start_auction(task, self.now))

    def _announce_surveys(self) -> None:
        if not self.strategy.survey_enabled or self.now + 1e-9 < self.next_survey_announcement:
            return
        self.next_survey_announcement = self.now + self.config.survey_announce_interval
        candidates: list[tuple[float, int]] = []
        for index, last_seen in enumerate(self.survey_last_seen):
            if index in self.active_survey_cells or self.survey_cooldown_until[index] > self.now:
                continue
            priority = PriorityFn(0.0, self.config.lambda_model, self.config.priority_k, last_seen).at(
                self.now
            )
            if priority >= self.config.survey_priority_threshold:
                candidates.append((priority, index))
        candidates.sort(key=lambda item: (-item[0], item[1]))
        for _, cell_index in candidates[: self.config.max_survey_announcements]:
            living = [robot for robot in self.robots if not robot.depleted]
            if not living:
                break
            preferred = cell_index % len(self.robots)
            host = next(
                (robot for robot in self.robots[preferred:] + self.robots[:preferred] if not robot.depleted),
                living[0],
            )
            task_id = f"survey-{cell_index:02d}-{self.survey_counter:04d}"
            self.survey_counter += 1
            task = Task(
                task_id,
                TaskType.SURVEY,
                self.survey_centres[cell_index],
                1,
                PriorityFn(
                    0.0, self.config.lambda_model, self.config.priority_k, self.survey_last_seen[cell_index]
                ),
                2.0,
                self.now,
            )
            self.tasks[task_id] = TaskRuntime(task, TaskState.BIDDING, host.robot_id, survey_cell=cell_index)
            self.active_survey_cells[cell_index] = task_id
            self._emit(host.start_auction(task, self.now))

    def _deliver_messages(self) -> None:
        delivered = self.bus.deliver(self.now)
        for robot in self.robots:
            for message in robot.receive(delivered[robot.robot_id], self.now):
                self._emit(message)

    def _process_auctions(self) -> None:
        for robot in self.robots:
            outgoing, awards, failed = robot.process_auctions(self.now)
            for task_id, winners in awards.items():
                runtime = self.tasks.get(task_id)
                if runtime is not None and runtime.state not in {TaskState.DONE, TaskState.FAILED}:
                    runtime.state = TaskState.ASSIGNED
                    runtime.winners = winners
                    runtime.award_at = self.now
            for task_id in failed:
                runtime = self.tasks.get(task_id)
                if runtime is not None and runtime.state is not TaskState.DONE:
                    runtime.state = TaskState.FAILED
                    runtime.failed_at = self.now
                    if runtime.survey_cell is not None:
                        self.active_survey_cells.pop(runtime.survey_cell, None)
                        self.survey_cooldown_until[runtime.survey_cell] = (
                            self.now + self.config.survey_cooldown
                        )
                    if self.logger is not None:
                        self.logger.event(self.now, "FAILED", task_id, robot.robot_id)
            for message in outgoing:
                self._emit(message)

    def _explore_target(self, index: int, robot: Robot) -> tuple[float, float]:
        if self.strategy.survey_enabled:
            ranked = sorted(
                range(len(self.survey_centres)),
                key=lambda cell_index: (self.survey_last_seen[cell_index], cell_index),
            )
            candidate = self.survey_centres[ranked[index % len(ranked)]]
            if (
                robot.explore_target is None
                or dist(robot.position, robot.explore_target) <= self.config.arrival_tolerance
            ):
                robot.explore_target = candidate
            return robot.explore_target
        waypoints = self.explore_waypoints[index]
        target = waypoints[self.explore_indices[index] % len(waypoints)]
        if dist(robot.position, target) <= self.config.arrival_tolerance:
            self.explore_indices[index] += 1
            target = waypoints[self.explore_indices[index] % len(waypoints)]
        return target

    def _move_robots(self) -> None:
        active = {task_id for task_id, runtime in self.tasks.items() if runtime.state is TaskState.ACTIVE}
        previously_depleted = sum(robot.depleted for robot in self.robots)
        for index, robot in enumerate(self.robots):
            for message in robot.advance(
                self.now, self.config.dt, active, self._explore_target(index, robot)
            ):
                self._emit(message)
            self.max_observed_workload = max(self.max_observed_workload, robot.workload)
        if (
            self.first_depletion_time is None
            and sum(robot.depleted for robot in self.robots) > previously_depleted
        ):
            self.first_depletion_time = self.now

    def _update_observation(self) -> None:
        radius = self.config.detection_radius
        for robot in self.robots:
            if robot.depleted:
                continue
            col_min = max(0, int((robot.position[0] - radius) / self.config.coverage_cell))
            col_max = min(
                self.coverage_cols - 1, int((robot.position[0] + radius) / self.config.coverage_cell)
            )
            row_min = max(0, int((robot.position[1] - radius) / self.config.coverage_cell))
            row_max = min(
                self.coverage_rows - 1, int((robot.position[1] + radius) / self.config.coverage_cell)
            )
            for row in range(row_min, row_max + 1):
                for col in range(col_min, col_max + 1):
                    centre = (
                        (col + 0.5) * self.config.coverage_cell,
                        (row + 0.5) * self.config.coverage_cell,
                    )
                    if dist(robot.position, centre) <= radius:
                        self.covered.add(row * self.coverage_cols + col)
            for cell_index, centre in enumerate(self.survey_centres):
                if dist(robot.position, centre) <= radius:
                    self.survey_last_seen[cell_index] = self.now

        for event in self.events:
            if event.event_id in self.detected_events or event.release_at > self.now:
                continue
            detectors = [
                robot
                for robot in self.robots
                if not robot.depleted and dist(robot.position, event.position) <= radius
            ]
            if detectors:
                detector = min(
                    detectors, key=lambda robot: (dist(robot.position, event.position), robot.robot_id)
                )
                self._announce_event(event, detector)

        idlenesses = [self.now - last_seen for last_seen in self.survey_last_seen]
        self.idleness_sum += sum(idlenesses) / len(idlenesses)
        self.idleness_samples += 1
        self.max_idleness = max(self.max_idleness, max(idlenesses))

    def _update_execution(self) -> None:
        for task_id, runtime in list(self.tasks.items()):
            if runtime.state is TaskState.ASSIGNED and runtime.winners:
                if (
                    runtime.award_at is not None
                    and self.now - runtime.award_at > self.config.assignment_timeout
                ):
                    for robot in self.robots:
                        robot.abandon_task(task_id)
                    host = next((robot for robot in self.robots if robot.robot_id == runtime.host), None)
                    if host is None or host.depleted:
                        runtime.state = TaskState.FAILED
                        runtime.failed_at = self.now
                    else:
                        self._emit(release_message(task_id, host.robot_id, "coordination_timeout", self.now))
                        self._emit(host.start_auction(runtime.task, self.now))
                        runtime.state = TaskState.BIDDING
                        runtime.winners = None
                    continue
                winners = [
                    next(robot for robot in self.robots if robot.robot_id == winner)
                    for winner in runtime.winners
                ]
                ready = all(
                    not robot.depleted
                    and robot.current_task_id == task_id
                    and dist(robot.position, runtime.task.position) <= self.config.arrival_tolerance
                    for robot in winners
                )
                if ready and len(winners) == runtime.task.n_req:
                    runtime.state = TaskState.ACTIVE
                    runtime.active_at = self.now
                elif ready:
                    self.n_req_violations += 1
            elif runtime.state is TaskState.ACTIVE and runtime.active_at is not None:
                if self.now - runtime.active_at + 1e-9 >= runtime.task.duration:
                    runtime.state = TaskState.DONE
                    runtime.done_at = self.now
                    winners = runtime.winners or []
                    for robot in self.robots:
                        if robot.robot_id in winners:
                            robot.complete_task(task_id)
                    sender = min(winners) if winners else runtime.host
                    self._emit(done_message(task_id, sender, self.now))
                    if runtime.survey_cell is not None:
                        cell = runtime.survey_cell
                        self.survey_last_seen[cell] = self.now
                        self.active_survey_cells.pop(cell, None)
                        self.survey_cooldown_until[cell] = self.now + self.config.survey_cooldown

    def _log_state(self) -> None:
        if self.now + 1e-9 < self.next_state_log:
            return
        self.next_state_log = self.now + self.config.state_log_interval
        if self.logger is not None:
            for robot in self.robots:
                self.logger.state(
                    self.now,
                    robot.robot_id,
                    robot.position[0],
                    robot.position[1],
                    robot.battery,
                    robot.workload,
                    robot.state.value,
                )

    def step(self) -> None:
        self._deliver_messages()
        self._announce_surveys()
        self._process_auctions()
        self._move_robots()
        if self.now + 1e-9 >= self.next_state_log:
            self._update_observation()
        self._update_execution()
        self._log_state()
        self.now = round(self.now + self.config.dt, 10)

    def run(self) -> SimulationResult:
        while self.now <= self.config.duration + 1e-9:
            self.step()
        metrics = self._metrics()
        return SimulationResult(self.seed, self.strategy.name.value, metrics, self.logger)

    def _metrics(self) -> dict[str, float | int | str]:
        roi_tasks = [
            runtime for runtime in self.tasks.values() if runtime.task.task_type is not TaskType.SURVEY
        ]
        completed = [
            runtime
            for runtime in roi_tasks
            if runtime.state is TaskState.DONE and runtime.done_at is not None
        ]
        completion_times = [
            runtime.done_at - runtime.task.announced_at
            for runtime in completed
            if runtime.done_at is not None
        ]
        loads = [robot.completed_roi_participations for robot in self.robots]
        final_batteries = [robot.battery for robot in self.robots]
        completed_count = len(completed)
        total_coverage_cells = self.coverage_cols * self.coverage_rows
        open_count = sum(runtime.state not in {TaskState.DONE, TaskState.FAILED} for runtime in roi_tasks)
        failed_count = sum(runtime.state is TaskState.FAILED for runtime in roi_tasks)
        return {
            "seed": self.seed,
            "strategy": self.strategy.name.value,
            "events_injected": len(self.events),
            "events_detected": len(self.detected_events),
            "roi_announced": len(roi_tasks),
            "roi_completed": completed_count,
            "roi_failed": failed_count,
            "roi_open_end": open_count,
            "completion_rate": completed_count / len(self.events) if self.events else 1.0,
            "detection_rate": len(self.detected_events) / len(self.events) if self.events else 1.0,
            "mean_completion_time": statistics.fmean(completion_times)
            if completion_times
            else self.config.duration,
            "median_completion_time": statistics.median(completion_times)
            if completion_times
            else self.config.duration,
            "coverage_fraction": len(self.covered) / total_coverage_cells,
            "coverage_cells_per_s": len(self.covered) / self.config.duration,
            "load_imbalance": statistics.pstdev(loads) if len(loads) > 1 else 0.0,
            "messages_per_completed": self.bus.sent_messages / max(1, completed_count),
            "bytes_per_completed": self.bus.sent_bytes / max(1, completed_count),
            "messages_total": self.bus.sent_messages,
            "bytes_total": self.bus.sent_bytes,
            "mean_cell_idleness": self.idleness_sum / max(1, self.idleness_samples),
            "max_cell_idleness": self.max_idleness,
            "mean_detection_latency": statistics.fmean(self.detection_latencies)
            if self.detection_latencies
            else self.config.duration,
            "preemptions": sum(robot.preemption_count for robot in self.robots),
            "mean_final_battery": statistics.fmean(final_batteries),
            "min_final_battery": min(final_batteries),
            "battery_imbalance": statistics.pstdev(final_batteries) if len(final_batteries) > 1 else 0.0,
            "depleted_robots": sum(robot.depleted for robot in self.robots),
            "first_depletion_time": self.first_depletion_time
            if self.first_depletion_time is not None
            else self.config.duration,
            "distance_travelled": sum(robot.distance_travelled for robot in self.robots),
            "max_observed_workload": self.max_observed_workload,
            "n_req_violations": self.n_req_violations,
            "conservation_error": len(roi_tasks) - completed_count - failed_count - open_count,
        }


def run_simulation(
    config: SimulationConfig,
    strategy: Strategy,
    seed: int,
    log_dir: str | Path | None = None,
) -> SimulationResult:
    world = World(config, strategy, seed, capture_logs=log_dir is not None)
    result = world.run()
    if log_dir is not None and result.logger is not None:
        events_path, _ = result.logger.write(log_dir)
        result.events_hash = hashlib.sha256(events_path.read_bytes()).hexdigest()
        metadata = {
            "seed": seed,
            "strategy": strategy.name.value,
            "config": config.to_dict(),
            "metrics": result.metrics,
            "events_sha256": result.events_hash,
        }
        Path(log_dir, "metadata.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8"
        )
    return result
