"""Webots adapter: one instance per robot, running inside that robot's own controller process.

Webots ties device control (motors, sensors) to whichever process is attached as controller of
a given Robot node - a Supervisor cannot drive another robot's motors. So unlike the Tier-0
simulator (one Python process holding every `sim.robot.Robot`), a Webots fleet is N independent
OS processes, each running docs/webots_market_robot_controller.py with its own WebotsBackend.

Design matches the proposal's own boundary (§2.1): "Localisation may be centralised. Decision-
making may not." Bidding/auction decisions run inside allocation.Robot, completely unchanged,
once per robot process - genuinely decentralized. Position ground truth (this backend's
get_pose) comes from each robot briefly acting as its own Supervisor (self-inspection only,
never touching another robot's node) - the simulated stand-in for the overhead camera. A
separate, non-fleet Supervisor process (docs/webots_market_supervisor_controller.py) plays the
overhead-camera role for cross-robot bookkeeping (RoI/survey detection triggers, n_req arrival
checks) that Tier 0 already centralises in sim/world.py - nothing here re-decides *who* wins an
auction, only *whether enough winners have physically arrived*.

Movement reuses the rotate-then-cruise design validated in docs/webots_calibration_controller.py,
rewritten as a non-blocking, one-tick-at-a-time state machine: drive_to() only records intent
(the RobotBackend protocol must not block - nothing else in a robot's own process needs to run
while waiting), and tick() - called once per Webots time step from the controller's main loop -
advances the wheels by one increment.

Communication piggybacks on Emitter/Receiver devices (Webots' built-in radio simulation) rather
than the abstract sim.comms.MessageBus, since that bus is a shared in-process object and Tier 1
has no single process to hold it. Two payload shapes share the same channel/bytes stream:
  - real allocation.messages.Message bytes (ANNOUNCE/BID/AWARD/DONE/RELEASE) - the actual,
    unmodified decentralized protocol, broadcast exactly as in Tier 0.
  - small JSON control-plane payloads (see the CTL_* constants below) from the supervisor
    only: RoI/survey hosting triggers, n_req-arrival ("ACTIVE") and coordination-timeout
    notices. These are NOT part of allocation/messages.py by design (this stage deliberately
    does not touch allocation/) and are entirely a Tier-1 orchestration detail, parsed only by
    this module.
receive() and detect_rois() are how a robot controller tells these two apart without needing to
know about Message/control-plane framing itself.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Any

from allocation.tasks import PriorityFn, Task, TaskType

from .base import Detection

WHEEL_RADIUS = 0.0205  # m -- VERIFY against your PROTO
AXLE_LENGTH = 0.053  # m -- VERIFY against your PROTO
MAX_WHEEL_SPEED = 6.28  # rad/s -- VERIFY against your PROTO
# Deliberately larger than parameters/default.json's 0.035 - a considered Tier-1 deviation, not
# a leftover bug. Tier 0 models dimensionless point robots; a real e-puck has ~0.037 m physical
# radius, already bigger than 0.035 on its own. For any task with n_req>1 converging on the same
# single task.position (SURROUND n_req=3 especially - Tier 0 has no notion of a formation/ring
# around a target, just one shared point), the winners physically block each other before they
# can all individually get that close - live evidence: 3 winners settled at 0.035/0.041/0.060 m
# from a SURROUND target, close packed against each other, never all inside 0.035 m at once, so
# ACTIVE never triggered even though the robots visibly *were* surrounding it. The packing lower
# bound for n circles of radius r touching a shared centre point is r/sin(pi/n) (~0.043 m for
# n=3, e.g.) - 0.08 m gives comfortable margin above that for imperfect real settling. Applied to
# both sim.config.SimulationConfig.arrival_tolerance (via dataclasses.replace() in both Tier-1
# controller scripts - see docs/webots_market_robot_controller.py and
# docs/webots_market_supervisor_controller.py) and this module constant, which must stay in sync
# since WebotsBackend.is_at_target() and sim/robot.py's own arrival check must agree on what
# "arrived" means or a robot can end up satisfied by one and still cruising toward the other.
ARRIVAL_TOLERANCE = 0.08  # m

# -pi/2 was the value confirmed during single-robot calibration (docs/WEBOTS_CALIBRATION.md's
# step 8) - but the TEST_SINGLE_OBJECT isolation test (see docs/webots_market_supervisor_controller.py)
# showed 3 winners circling their (stationary) target at a *constant* radius for the entire run,
# never converging. Constant-radius orbiting is the signature of driving tangentially rather than
# radially - i.e. a fixed heading-reading error, not noise. Worked out analytically from that
# run's own position log (state_r02/r05/r06.csv): actual travel direction consistently led the
# computed bearing-to-target by ~+90 degrees at three independent robots/timestamps, meaning
# _read_heading() was over-rotating by 90 degrees - i.e. HEADING_OFFSET should be 0, not -pi/2,
# for this world/PROTO placement. Left unexplained: why the single-robot calibration test found
# -pi/2 in the first place (different world file, different initial `rotation` field, or a test
# that only checked arrival within a short hop rather than straight-line trajectory - a 90-degree
# bearing error only becomes obvious over a long, unobstructed cruise). If you re-run
# WEBOTS_CALIBRATION.md's step 8 and it now points to something other than 0, trust that fresh
# measurement over this comment - re-derive from a live position log, same method as here.
HEADING_OFFSET = 0.0  # rad

ROTATE_GAIN = 3.0
ROTATE_TOLERANCE_DEG = 3.0
# Tried 4.0 rad/s once (to close the gap with parameters/default.json's effective_speed=0.08 m/s
# - see docs/webots_market_supervisor_controller.py's ASSIGNMENT_TIMEOUT_OVERRIDE_S comment for
# that reasoning) - reverted. Evidence from a live run: at 4.0, 4 of 7 robots ended the 300s run
# pinned against a wall (x or y frozen at the arena boundary minus ~arrival_tolerance, e.g.
# x=1.963 repeated across multiple robots) with *zero* tasks reaching ACTIVE the whole run - worse
# than at 3.0. Back to 3.0 rad/s (0.0615 m/s), the value actually validated to drive straight in
# the isolation test; closing the speed gap with Tier 0 is not worth reintroducing this. If
# throughput is still too low with just the assignment_timeout override, look at workload_cap or
# task-generation rate before touching this again - not speed.
CRUISE_SPEED = 3.0  # rad/s
CRUISE_RAMP_S = 1.0
REALIGN_THRESHOLD_DEG = 5.0

# Stuck detection: some ROI/explore targets legitimately sit close to the arena wall (Tier 0's
# own event generator allows positions down to a 0.06 m margin, sized for a dimensionless point
# robot, not a physical e-puck with ~0.037 m radius plus a 0.035 m arrival tolerance - the two
# margins overlap, so driving straight at such a target can walk the robot into the wall with no
# recovery). Rather than shrink Tier 0's own margins (would change the event schedule shared
# with comparison_summary.md), detect "commanded to move, barely moving" and back off - this
# also self-heals other unmodelled snags (e.g. brushing another robot), not just wall contact.
# A first attempt at 0.02 m/5s (0.004 m/s) missed a real case: a robot wedged against a wall at
# a shallow angle can slide along it at ~0.008 m/s - slow enough to never usefully arrive within
# assignment_timeout, but just above that first threshold, so backoff never triggered. Nominal
# cruise speed is ~0.06 m/s; 0.10 m/5s (0.02 m/s, ~1/3 of nominal) comfortably catches wall-slide
# creep while still tolerating normal realignment-induced slowdowns.
STUCK_CHECK_INTERVAL_S = 5.0
STUCK_DISTANCE_THRESHOLD = 0.10  # m of real progress expected per check interval while cruising
BACKOFF_SPEED = -2.5  # rad/s, both wheels equal (straight reverse)
BACKOFF_DURATION_S = 1.5

# Control-plane payloads are tagged {"ctl": <one of these>, ...}; real protocol Message bytes
# are tagged {"kind": <ANNOUNCE|BID|AWARD|DONE|RELEASE>, ...} and are never touched here except
# to tell the two apart - allocation.messages.Message.from_bytes/to_bytes stays canonical.
CTL_DETECT_ROI = "DETECT_ROI"  # targeted: only the discovering robot ("to") acts on it
CTL_HOST_SURVEY = "HOST_SURVEY"  # targeted: only the chosen host ("to") acts on it
CTL_EXPLORE_TARGET = "EXPLORE_TARGET"  # targeted: where to go while idle (mirrors World._explore_target)
CTL_ACTIVE = "ACTIVE"  # broadcast: n_req winners have arrived, task_id enters active_tasks
CTL_COORD_TIMEOUT = "COORD_TIMEOUT"  # broadcast: assignment_timeout expired for task_id
_BROADCAST_CTL_KINDS = (CTL_ACTIVE, CTL_COORD_TIMEOUT)
_TARGETED_CTL_KINDS = (CTL_DETECT_ROI, CTL_HOST_SURVEY, CTL_EXPLORE_TARGET)


class _Phase:
    IDLE = "idle"
    ROTATE = "rotate"
    CRUISE = "cruise"
    BACKOFF = "backoff"  # briefly reverse, then re-enter ROTATE - see STUCK_* constants above


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(slots=True)
class WebotsBackend:
    """RobotBackend implementation for one Webots robot, driven from its own controller process.

    `node`, `left_motor`, `right_motor`, `emitter`, `receiver` are Webots device handles
    obtained by the controller script via `robot.getSelf()`/`robot.getDevice(...)` - this class
    has no Webots import of its own (see docs/webots_market_robot_controller.py for wiring),
    so it stays importable outside Webots for tooling/tests.
    """

    robot_id: str
    node: Any
    left_motor: Any
    right_motor: Any
    emitter: Any
    receiver: Any
    clock: Any  # callable, e.g. robot.getTime
    battery_idle_drain: float = 8e-5  # per second, must match parameters/default.json
    battery_move_drain_per_m: float = 0.0094
    battery: float = 1.0
    target: tuple[float, float] | None = None
    _phase: str = _Phase.IDLE
    _cruise_start: float = 0.0
    _backoff_until: float = 0.0
    _stuck_check_pos: tuple[float, float] | None = None
    _stuck_check_time: float = 0.0
    _protocol_inbox: list[bytes] = field(default_factory=list)
    _detections: list[Detection] = field(default_factory=list)
    _control_inbox: list[dict[str, Any]] = field(default_factory=list)

    # --- RobotBackend protocol -------------------------------------------------------------

    def get_pose(self) -> tuple[float, float, float]:
        position = self.node.getPosition()
        return (position[0], position[1], self._read_heading())

    def get_battery(self) -> float:
        return self.battery

    def drive_to(self, x: float, y: float) -> None:
        if self.target != (x, y):
            self.target = (x, y)
            self._phase = _Phase.ROTATE

    def is_at_target(self) -> bool:
        if self.target is None:
            return True
        px, py, _ = self.get_pose()
        return math.hypot(self.target[0] - px, self.target[1] - py) <= ARRIVAL_TOLERANCE

    def broadcast(self, msg: bytes) -> None:
        self.emitter.send(msg)

    def receive(self) -> list[bytes]:
        self._drain_receiver()
        messages, self._protocol_inbox = self._protocol_inbox, []
        return messages

    def detect_rois(self) -> list[Detection]:
        self._drain_receiver()
        detections, self._detections = self._detections, []
        return detections

    def now(self) -> float:
        return float(self.clock())

    # --- Tier-1-only extensions, not part of RobotBackend ----------------------------------

    def pop_control_messages(self) -> list[dict[str, Any]]:
        """Supervisor control-plane payloads (host-survey / coordination-timeout triggers).

        Kept separate from detect_rois()/receive() because these aren't RobotBackend concepts;
        the controller script consumes them explicitly (see docs/webots_market_robot_controller.py).
        """

        self._drain_receiver()
        messages, self._control_inbox = self._control_inbox, []
        return messages

    def tick(self, dt: float) -> None:
        """Advance the wheel motors by one Webots time step toward the current target.

        Called once per robot per world tick, before the shared supervisor.step() - see the
        module docstring on why this can't be a blocking call inside drive_to() the way the
        single-robot calibration controller did it.
        """

        if self._phase == _Phase.BACKOFF:
            if self.now() < self._backoff_until:
                self.left_motor.setVelocity(BACKOFF_SPEED)
                self.right_motor.setVelocity(BACKOFF_SPEED)
                self._drain_battery(dt, abs(BACKOFF_SPEED) * WHEEL_RADIUS * dt)
                return
            self._phase = _Phase.IDLE  # falls through to a fresh ROTATE below, target unchanged

        if self.target is None or self.is_at_target():
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
            self._phase = _Phase.IDLE
            self._drain_battery(dt, 0.0)
            return

        px, py, heading = self.get_pose()
        bearing = math.atan2(self.target[1] - py, self.target[0] - px)

        if self._phase == _Phase.IDLE:
            self._phase = _Phase.ROTATE

        if self._phase == _Phase.ROTATE:
            error = _wrap(bearing - heading)
            if abs(error) <= math.radians(ROTATE_TOLERANCE_DEG):
                self._phase = _Phase.CRUISE
                self._cruise_start = self.now()
                self._stuck_check_pos = (px, py)
                self._stuck_check_time = self.now()
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
            else:
                turn = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, ROTATE_GAIN * error))
                self.left_motor.setVelocity(-turn)
                self.right_motor.setVelocity(turn)
            self._drain_battery(dt, 0.0)
            return

        drift_deg = math.degrees(abs(_wrap(bearing - heading)))
        if drift_deg > REALIGN_THRESHOLD_DEG:
            self._phase = _Phase.ROTATE
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
            self._drain_battery(dt, 0.0)
            return

        # Stuck check: a target close to a wall (or another robot in the way) can be driven
        # toward forever without ever getting there - see the STUCK_* constants' comment.
        # Checked only in CRUISE (not ROTATE, where standing still turning is expected).
        if self.now() - self._stuck_check_time >= STUCK_CHECK_INTERVAL_S:
            progress = math.hypot(px - self._stuck_check_pos[0], py - self._stuck_check_pos[1])
            self._stuck_check_pos = (px, py)
            self._stuck_check_time = self.now()
            if progress < STUCK_DISTANCE_THRESHOLD:
                self._phase = _Phase.BACKOFF
                self._backoff_until = self.now() + BACKOFF_DURATION_S
                self.left_motor.setVelocity(BACKOFF_SPEED)
                self.right_motor.setVelocity(BACKOFF_SPEED)
                self._drain_battery(dt, abs(BACKOFF_SPEED) * WHEEL_RADIUS * dt)
                return

        ramp_fraction = min(1.0, (self.now() - self._cruise_start) / CRUISE_RAMP_S)
        speed = CRUISE_SPEED * ramp_fraction
        self.left_motor.setVelocity(speed)
        self.right_motor.setVelocity(speed)
        self._drain_battery(dt, speed * WHEEL_RADIUS * dt)

    # --- internals ---------------------------------------------------------------------------

    def _drain_battery(self, dt: float, travelled_m: float) -> None:
        self.battery = max(
            0.0,
            self.battery - self.battery_idle_drain * dt - self.battery_move_drain_per_m * travelled_m,
        )

    def _read_heading(self) -> float:
        rotation_matrix = self.node.getOrientation()
        return math.atan2(rotation_matrix[3], rotation_matrix[0]) + HEADING_OFFSET

    def _drain_receiver(self) -> None:
        while self.receiver.getQueueLength() > 0:
            raw = bytes(self.receiver.getBytes())
            self.receiver.nextPacket()
            self._route_incoming(raw)

    def _route_incoming(self, raw: bytes) -> None:
        try:
            payload = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        if "kind" in payload:
            self._protocol_inbox.append(raw)
            return
        ctl = payload.get("ctl")
        if ctl in _TARGETED_CTL_KINDS:
            if payload.get("to") != self.robot_id:
                return
            if ctl == CTL_DETECT_ROI:
                self._detections.append(
                    Detection(str(payload["event_id"]), tuple(payload["pos"]), str(payload["task_type"]))
                )
            self._control_inbox.append(payload)
        elif ctl in _BROADCAST_CTL_KINDS:
            self._control_inbox.append(payload)
        # unrecognised control payloads are ignored, not raised - keeps the shared channel
        # forward-compatible if the supervisor script gains new message kinds later


def task_from_detection_payload(payload: dict[str, Any], roi_aging_lambda: float, priority_k: float, now: float) -> Task:
    """Build the same Task sim/world.py's World._announce_event() would, from a CTL_DETECT_ROI payload."""

    return Task(
        str(payload["event_id"]).replace("event", "roi"),
        TaskType(payload["task_type"]),
        tuple(payload["pos"]),
        int(payload["n_req"]),
        PriorityFn(float(payload["p0"]), roi_aging_lambda, priority_k, now),
        float(payload["duration"]),
        now,
    )


def task_from_host_survey_payload(payload: dict[str, Any], lambda_model: float, priority_k: float, now: float) -> Task:
    """Build the same Task sim/world.py's World._announce_surveys() would, from a CTL_HOST_SURVEY payload."""

    return Task(
        str(payload["task_id"]),
        TaskType.SURVEY,
        tuple(payload["pos"]),
        1,
        PriorityFn(0.0, lambda_model, priority_k, float(payload["last_seen"])),
        2.0,
        now,
    )


def find_node_by_name(root: Any, target_name: str) -> Any | None:
    """Find a node by its `name` field, walking the scene tree from root.getField("children").

    Used by the supervisor script instead of getFromDef()/DEF labels - those require
    hand-editing the .wbt file, which is fragile in some worlds (an EXTERNPROTO-based one
    crashed entirely on a hand-added DEF during this project's own setup). `name` fields are
    already set correctly by construction (every fleet robot needs name "r00", "r01", ... per
    docs/webots_market_robot_controller.py's docstring anyway) and are edited safely through
    the normal Scene Tree field panel - no text editing required.

    `root`/the returned node only need a Webots-like `getField(str)` API (Field objects with
    `getCount()`/`getMFNode(int)` for "children", `getSFString()` for "name") - see
    tests/test_webots_backend.py for a fake-node exercise of this without real Webots.
    """

    children_field = root.getField("children") if root is not None else None
    if children_field is None:
        return None
    for i in range(children_field.getCount()):
        child = children_field.getMFNode(i)
        if child is None:
            continue
        name_field = child.getField("name")
        if name_field is not None and name_field.getSFString() == target_name:
            return child
        found = find_node_by_name(child, target_name)
        if found is not None:
            return found
    return None
