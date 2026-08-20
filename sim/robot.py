"""Local robot state: bidding, hosted auctions, assignments and point motion."""

from __future__ import annotations

from enum import Enum
from math import atan2, dist
from random import Random

from allocation.auction import HostedAuction
from allocation.bidding import arena_diagonal, compute_cost, should_preempt
from allocation.messages import (
    Message,
    MessageType,
    announce_message,
    award_message,
    bid_message,
    release_message,
)
from allocation.strategies import Strategy
from allocation.tasks import PriorityFn, Task, TaskType

from backends.base import RobotBackend

from .config import SimulationConfig


class RobotState(str, Enum):
    EXPLORE = "EXPLORE"
    NAVIGATE = "NAVIGATE"
    WAIT_PEERS = "WAIT_PEERS"
    EXECUTE = "EXECUTE"
    DEPLETED = "DEPLETED"


class Robot:
    def __init__(
        self,
        robot_id: str,
        position: tuple[float, float],
        battery: float,
        strategy: Strategy,
        rng: Random,
        config: SimulationConfig,
        backend: RobotBackend | None = None,
    ) -> None:
        self.robot_id = robot_id
        self.position = position
        self.heading = 0.0
        self.battery = battery
        self.strategy = strategy
        self.rng = rng
        self.config = config
        # Tier 1/2 hook: when set, movement and battery are delegated to a real (or
        # Webots-simulated) backend instead of the abstract point-kinematics below. Default
        # None preserves Tier 0 exactly - this parameter changes nothing for existing callers.
        self.backend = backend
        self.state = RobotState.EXPLORE
        self.assignments: list[str] = []
        self.known_tasks: dict[str, Task] = {}
        self.owned_tasks: dict[str, Task] = {}
        self.hosted: dict[str, HostedAuction] = {}
        self.reauction_queue: set[str] = set()
        self.pending_bids: dict[str, float] = {}
        self.preempt_intent: set[str] = set()
        self.last_preemption = -float("inf")
        self.preemption_count = 0
        self.completed_participations = 0
        self.completed_roi_participations = 0
        self.distance_travelled = 0.0
        self.explore_target: tuple[float, float] | None = None
        self.motion_target_key = ""
        self.startup_remaining = 0.0

    @property
    def depleted(self) -> bool:
        return self.state is RobotState.DEPLETED

    @property
    def workload(self) -> int:
        return sum(
            self.known_tasks[task_id].counts_toward_workload
            for task_id in self.assignments
            if task_id in self.known_tasks
        )

    @property
    def current_task_id(self) -> str | None:
        return self.assignments[0] if self.assignments else None

    def start_auction(self, task: Task, now: float) -> Message:
        self.known_tasks[task.task_id] = task
        self.owned_tasks[task.task_id] = task
        auction = HostedAuction(task.task_id, task.n_req, now + self.config.bid_window)
        self.hosted[task.task_id] = auction
        self.reauction_queue.discard(task.task_id)
        return announce_message(task, auction.deadline, self.robot_id, now)

    def _task_from_announce(self, message: Message) -> Task:
        fields = message.fields
        priority = PriorityFn(
            float(fields["p0"]), float(fields["lam"]), float(fields["k"]), float(fields["t_ref"])
        )
        return Task(
            str(fields["task_id"]),
            TaskType(str(fields["type"])),
            (float(fields["pos"][0]), float(fields["pos"][1])),
            int(fields["n_req"]),
            priority,
            float(fields["duration"]),
            float(fields["t"]),
        )

    def _cleanup_pending(self, now: float) -> None:
        expired = [task_id for task_id, expiry in self.pending_bids.items() if expiry < now]
        for task_id in expired:
            self.pending_bids.pop(task_id, None)
            self.preempt_intent.discard(task_id)

    def _pending_workload(self) -> int:
        return sum(
            self.known_tasks[task_id].counts_toward_workload
            for task_id in self.pending_bids
            if task_id in self.known_tasks
        )

    def receive(self, payloads: list[bytes], now: float) -> list[Message]:
        self._cleanup_pending(now)
        outgoing: list[Message] = []
        for payload in payloads:
            message = Message.from_bytes(payload)
            fields = message.fields
            task_id = str(fields["task_id"])
            if message.kind is MessageType.ANNOUNCE:
                task = self._task_from_announce(message)
                self.known_tasks[task_id] = task
                outgoing.extend(self._on_announce(task, float(fields["deadline"]), now))
            elif message.kind is MessageType.BID and task_id in self.hosted:
                self.hosted[task_id].submit(str(fields["sender"]), float(fields["cost"]), now)
            elif message.kind is MessageType.AWARD:
                outgoing.extend(self._on_award(task_id, [str(item) for item in fields["winners"]], now))
            elif message.kind is MessageType.DONE:
                self._remove_assignment(task_id)
                self.owned_tasks.pop(task_id, None)
                self.hosted.pop(task_id, None)
            elif message.kind is MessageType.RELEASE:
                self._remove_assignment(task_id)
                if task_id in self.owned_tasks and task_id not in self.hosted:
                    self.reauction_queue.add(task_id)
        return outgoing

    def _on_announce(self, task: Task, deadline: float, now: float) -> list[Message]:
        if self.depleted or task.task_id in self.assignments or task.task_id in self.pending_bids:
            return []
        if task.task_type is TaskType.SURVEY:
            if not self.strategy.survey_enabled:
                return []
            if any(
                self.known_tasks[task_id].task_type is TaskType.SURVEY
                for task_id in self.assignments + list(self.pending_bids)
                if task_id in self.known_tasks
            ):
                return []

        can_preempt = False
        current_id = self.current_task_id
        if (
            self.strategy.preemption_enabled
            and current_id is not None
            and self.state is RobotState.NAVIGATE
            and now - self.last_preemption >= self.config.preemption_cooldown
        ):
            current = self.known_tasks[current_id]
            current_cost = compute_cost(
                self.position,
                self.battery,
                self.workload,
                current,
                self.strategy.weights,
                arena_diagonal(self.config.arena_width, self.config.arena_height),
                self.config.workload_cap,
            )
            new_cost = compute_cost(
                self.position,
                self.battery,
                self.workload,
                task,
                self.strategy.weights,
                arena_diagonal(self.config.arena_width, self.config.arena_height),
                self.config.workload_cap,
            )
            can_preempt = current.task_type is TaskType.SURVEY or should_preempt(
                current, task, current_cost, new_cost, now, self.strategy.weights
            )

        if task.counts_toward_workload:
            capacity_used = self.workload + self._pending_workload()
            if capacity_used >= self.config.workload_cap and not can_preempt:
                return []
        elif self.workload > 0:
            return []

        cost = self.strategy.cost(
            self.position,
            self.battery,
            self.workload,
            task,
            arena_diagonal(self.config.arena_width, self.config.arena_height),
            self.rng,
        )
        self.pending_bids[task.task_id] = deadline + self.config.award_timeout
        if can_preempt:
            self.preempt_intent.add(task.task_id)
        return [bid_message(task.task_id, cost, self.robot_id, now)]

    def _on_award(self, task_id: str, winners: list[str], now: float) -> list[Message]:
        self.pending_bids.pop(task_id, None)
        if self.robot_id not in winners or task_id not in self.known_tasks:
            self.preempt_intent.discard(task_id)
            return []
        task = self.known_tasks[task_id]
        outgoing: list[Message] = []
        if task_id in self.preempt_intent and self.current_task_id is not None:
            old_task_id = self.current_task_id
            self._remove_assignment(old_task_id)
            outgoing.append(release_message(old_task_id, self.robot_id, "preempted", now))
            self.last_preemption = now
            self.preemption_count += 1
        elif task.counts_toward_workload and self.workload >= self.config.workload_cap:
            self.preempt_intent.discard(task_id)
            return [release_message(task_id, self.robot_id, "capacity", now)]

        survey_ids = [
            assigned
            for assigned in self.assignments
            if self.known_tasks[assigned].task_type is TaskType.SURVEY
        ]
        if task.counts_toward_workload:
            for survey_id in survey_ids:
                self._remove_assignment(survey_id)
                outgoing.append(release_message(survey_id, self.robot_id, "roi_priority", now))
        if task_id not in self.assignments:
            if task.counts_toward_workload:
                self.assignments.append(task_id)
            else:
                self.assignments.insert(0, task_id)
        self.preempt_intent.discard(task_id)
        return outgoing

    def process_auctions(self, now: float) -> tuple[list[Message], dict[str, list[str]], list[str]]:
        outgoing: list[Message] = []
        awards: dict[str, list[str]] = {}
        failed: list[str] = []
        for task_id in sorted(self.reauction_queue):
            if task_id not in self.hosted and task_id in self.owned_tasks:
                outgoing.append(self.start_auction(self.owned_tasks[task_id], now))
        for task_id, auction in list(self.hosted.items()):
            if now + 1e-9 < auction.deadline:
                continue
            winners = auction.winners()
            if winners:
                outgoing.append(award_message(task_id, winners, self.robot_id, now))
                awards[task_id] = winners
                self.hosted.pop(task_id)
            elif auction.attempt < self.config.max_auction_retries:
                retry = auction.retry(now, self.config.bid_window)
                self.hosted[task_id] = retry
                outgoing.append(
                    announce_message(self.owned_tasks[task_id], retry.deadline, self.robot_id, now)
                )
            else:
                self.hosted.pop(task_id)
                self.owned_tasks.pop(task_id, None)
                failed.append(task_id)
        return outgoing, awards, failed

    def advance(
        self,
        now: float,
        dt: float,
        active_tasks: set[str],
        explore_target: tuple[float, float],
    ) -> list[Message]:
        if self.depleted:
            return []
        current_id = self.current_task_id
        if current_id is not None and current_id not in self.known_tasks:
            self._remove_assignment(current_id)
            current_id = self.current_task_id

        if current_id is not None:
            task = self.known_tasks[current_id]
            target = task.position
            target_key = current_id
            if current_id in active_tasks:
                self.state = RobotState.EXECUTE
                moving = False
            elif dist(self.position, target) <= self.config.arrival_tolerance:
                self.state = RobotState.WAIT_PEERS
                moving = False
            else:
                self.state = RobotState.NAVIGATE
                moving = True
        else:
            self.state = RobotState.EXPLORE
            target = explore_target
            target_key = f"explore:{target[0]:.3f}:{target[1]:.3f}"
            moving = dist(self.position, target) > self.config.arrival_tolerance

        if self.backend is None:
            travelled = self._advance_kinematic(target, target_key, moving, dt)
            self.distance_travelled += travelled
            self.battery = max(
                0.0,
                self.battery
                - self.config.battery_idle_drain * dt
                - self.config.battery_move_drain_per_m * travelled,
            )
        else:
            travelled = self._advance_via_backend(target, moving)
            self.distance_travelled += travelled
            self.battery = self.backend.get_battery()

        if self.battery > self.config.depleted_threshold:
            return []
        outgoing = [release_message(task_id, self.robot_id, "depleted", now) for task_id in self.assignments]
        self.assignments.clear()
        self.state = RobotState.DEPLETED
        return outgoing

    def _advance_kinematic(
        self,
        target: tuple[float, float],
        target_key: str,
        moving: bool,
        dt: float,
    ) -> float:
        """Tier 0: abstract point-kinematics, unchanged from the original single-path version."""

        travelled = 0.0
        if moving:
            if target_key != self.motion_target_key:
                self.motion_target_key = target_key
                self.startup_remaining = self.config.startup_delay
            if self.startup_remaining > 0.0:
                self.startup_remaining = max(0.0, self.startup_remaining - dt)
            else:
                distance_to_target = dist(self.position, target)
                travelled = min(self.config.effective_speed * dt, distance_to_target)
                if travelled > 0.0:
                    ratio = travelled / distance_to_target
                    dx = target[0] - self.position[0]
                    dy = target[1] - self.position[1]
                    self.heading = atan2(dy, dx)
                    self.position = (self.position[0] + ratio * dx, self.position[1] + ratio * dy)
        return travelled

    def _advance_via_backend(self, target: tuple[float, float], moving: bool) -> float:
        """Tier 1/2: delegate movement to the injected backend; only read pose back."""

        assert self.backend is not None
        previous_position = self.position
        if moving:
            self.backend.drive_to(target[0], target[1])
        x, y, heading = self.backend.get_pose()
        self.position = (x, y)
        self.heading = heading
        return dist(previous_position, self.position)

    def complete_task(self, task_id: str) -> None:
        if task_id in self.assignments:
            self.completed_participations += 1
            if self.known_tasks[task_id].counts_toward_workload:
                self.completed_roi_participations += 1
        self._remove_assignment(task_id)

    def abandon_task(self, task_id: str) -> None:
        self._remove_assignment(task_id)

    def _remove_assignment(self, task_id: str) -> None:
        if task_id in self.assignments:
            self.assignments.remove(task_id)
        self.pending_bids.pop(task_id, None)
        self.preempt_intent.discard(task_id)
