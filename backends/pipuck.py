"""Real Pi-Puck adapter: one instance per robot, mirroring backends/webots.py's
architecture and reusing its rotate-then-cruise movement state machine
unchanged - only the transport differs. Retargeted to the lab's actual
stack: MQTT (paho-mqtt) instead of Webots' Emitter/Receiver, and pose from
the lab's existing RCPS overhead-tracking system (published on the
`robot_pos/all` MQTT topic) instead of a Webots Node read or a
self-written camera script - the lab already runs the "overhead camera",
nothing here needs to reimplement it.

STATUS: written ahead of the first hardware pilot session, not yet run on
real Pi-Pucks - same disclosure this project has given every backend before
its first live test. THREE integration points are genuinely uncertain and
MUST be verified on-site before first use (see docs/LAB_PILOT_CHECKLIST.md):
1. The MQTT broker address (two different IPs appear across the lab's own
   documents - confirm with the exercise supervisor).
2. The exact JSON schema of `robot_pos/all` (assumed below from the RCPS
   dashboard screenshot: a dict keyed by numeric tracking id, each value
   holding "position": [x, y] in metres and "angle" in degrees - VERIFY
   against one real received payload before trusting it).
3. The motor command range `_MotorDriver` sends to `set_motor_speeds()`
   (assumed [-1000, 1000] per the e-puck2 firmware cheat sheet's
   `motors.h`, scaled from this module's internal rad/s convention).

Architecture, matching the project's own boundary ("localisation may be
centralised, decision-making may not"): this module never decides who wins
an auction - it only realises RobotBackend's eight methods on top of (a) an
MQTT client shared with every other robot and the lab's tracking system,
and (b) the pi-puck package's motor driver.
"""

from __future__ import annotations

import json
import math
import queue
import time
from dataclasses import dataclass, field
from typing import Any

from allocation.tasks import PriorityFn, Task, TaskType

from .base import Detection

# --- MQTT ------------------------------------------------------------------
# VERIFY on-site (see module docstring, point 1): the lab's own documents
# show two different broker IPs (192.168.178.43 in the setup guide's RCPS
# dashboard/MQTT instructions, 192.168.178.56 in the example client.py) -
# confirm which is live before running anything.
MQTT_BROKER = "192.168.178.43"
MQTT_PORT = 1883
POS_TOPIC = "robot_pos/all"  # published by the lab's existing tracking system - do not publish here, only subscribe
WIRE_TOPIC = "mrta/wire"  # this project's own shared topic: allocation.Message bytes + control-plane payloads

# --- robot identity ----------------------------------------------------------
# The tracking system identifies every tracked thing by the same kind of
# numeric id (see the RCPS dashboard screenshot: "Robot ID: 22") - both
# actual robots AND task-object props (a cardboard cylinder with a marker
# reads exactly like a robot to the tracker). Two separate mappings from
# that shared numeric-id space, both EDIT-ME before the pilot:
TRACKING_ID_TO_ROBOT_ID = {34: "r00", 1: "r01"}  # VERIFY against the RCPS dashboard: your two Pi-Pucks
# The pilot's one GUARD task-object marker (e.g. taped to a small cardboard
# cylinder) - its position is read LIVE from robot_pos/all every tick, not
# hardcoded, so it can be moved between runs without touching any code.
# VERIFY against the RCPS dashboard: whichever numeric id is the cylinder,
# not a robot.
PILOT_TASK_TRACKING_ID = 44

# --- movement ----------------------------------------------------------------
# Same rotate-then-cruise design validated in backends/webots.py; constants
# start from Tier-1's values but are NOT assumed to transfer. Expect to
# retune live during the pilot, exactly as happened for Tier 1.
WHEEL_RADIUS = 0.0205  # m -- VERIFY against the e-puck2/Pi-Puck datasheet
MAX_WHEEL_SPEED = 6.28  # rad/s -- the e-puck's documented physical max; used only to scale into the firmware's command range, see _MotorDriver
ARRIVAL_TOLERANCE = 0.08  # m -- start from the Tier-1 value; retune once real pose noise is visible
HEADING_OFFSET = 0.0  # rad -- do NOT assume Webots' value transfers; re-derive empirically (see docs/LAB_PILOT_CHECKLIST.md)

ROTATE_GAIN = 3.0
# VERIFIED too tight on-site (2026-08-20 pilot logs): real robot_pos/all
# heading noise was ~5-9 deg even while roughly pointed at the target
# (bearing steady at ~15-16 deg, heading oscillating 7-12 deg) - at 3 deg
# the robot could never satisfy the tolerance and sat in ROTATE forever,
# never reaching CRUISE. Raised to clear that noise floor with margin.
# REALIGN_THRESHOLD_DEG raised together, same ~1.7x ratio as before
# (3.0/5.0), so CRUISE doesn't immediately bounce back into ROTATE the
# tick after exiting it at the new, looser tolerance.
ROTATE_TOLERANCE_DEG = 8.0
CRUISE_SPEED = 2.0  # rad/s -- deliberately conservative for the first physical run
CRUISE_RAMP_S = 1.0
REALIGN_THRESHOLD_DEG = 14.0
STUCK_CHECK_INTERVAL_S = 5.0
STUCK_DISTANCE_THRESHOLD = 0.05  # m per interval
BACKOFF_SPEED = -2.0
BACKOFF_DURATION_S = 1.5
POSE_STALE_AFTER_S = 2.5  # treat a robot as "lost" if the tracker hasn't reported it this recently
# VERIFY on-site: pilot logs show robot_pos/all landing roughly every ~1.2-1.3s
# for a tracked marker in normal operation (not a fixed/guaranteed rate anywhere
# in the lab's own docs), well above a 15 Hz control loop's tick. 1.0s used to
# sit *below* that normal gap, so both this warning and tick()'s stale-pose
# motor freeze (see PiPuckBackend.tick) fired on almost every ordinary update
# cycle, not just on genuine tracker loss - looked like "the tracker keeps
# losing the robot" and, combined with the freeze, could look like the robot
# barely moving/jittering instead of steering smoothly between updates. 2.5s
# gives a couple of missed updates worth of headroom before treating it as
# actually lost.

# Control-plane wire tags carried inside WIRE_TOPIC payloads (identical in
# spirit to backends/webots.py's CTL_* tags, transport differs):
CAM_DETECT = "DETECT"  # targeted: {"cam": "DETECT", "to": ..., "event_id": ..., ...} - simulates on-board RoI detection
CTL_ACTIVE = "ACTIVE"  # broadcast: n_req winners have arrived
CTL_COORD_TIMEOUT = "COORD_TIMEOUT"  # broadcast: assignment_timeout expired


class _Phase:
    IDLE = "idle"
    ROTATE = "rotate"
    CRUISE = "cruise"
    BACKOFF = "backoff"


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class _MotorDriver:
    """The ONE place hardware-specific motor control lives - isolated so
    that if the installed `pi-puck` package's real API or unit convention
    differs from what is assumed here, only this class needs to change.

    ASSUMED API (the `pi-puck` PyPI package, confirmed correct in the lab's
    own client.py example): `PiPuck(epuck_version=2).epuck.set_motor_speeds(
    left, right)`. UNVERIFIED: the unit convention - the e-puck2 firmware
    cheat sheet documents the underlying C API's range as [-1000, 1000]
    (motors.h), so this class scales from the rad/s convention used
    everywhere else in this file into that range. If real motion looks far
    too fast/slow relative to what the commanded rad/s value implies,
    this scale factor - not the rotate-then-cruise logic - is the first
    thing to check.
    """

    def __init__(self) -> None:
        from pipuck.pipuck import PiPuck  # deferred: only importable on the robot itself

        self._robot = PiPuck(epuck_version=2)
        self._scale = 1000.0 / MAX_WHEEL_SPEED

    def set_velocity(self, left: float, right: float) -> None:
        self._robot.epuck.set_motor_speeds(int(round(left * self._scale)), int(round(right * self._scale)))


class _MqttTransport:
    """Thin wrapper around paho-mqtt: subscribes to POS_TOPIC (read-only,
    published by the lab's tracking system) and WIRE_TOPIC (read/write,
    this project's own channel), and buffers incoming messages in a
    thread-safe queue for the synchronous tick() loop to drain - paho's
    on_message callback fires on its own network thread.
    """

    def __init__(self) -> None:
        import paho.mqtt.client as mqtt

        self._inbox: queue.Queue[tuple[str, bytes]] = queue.Queue()
        self._client = mqtt.Client()
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self._client.loop_start()

    def _on_connect(self, client, _userdata, _flags, _rc) -> None:
        client.subscribe(POS_TOPIC)
        client.subscribe(WIRE_TOPIC)

    def _on_message(self, _client, _userdata, msg) -> None:
        self._inbox.put((msg.topic, msg.payload))

    def publish(self, payload: bytes) -> None:
        self._client.publish(WIRE_TOPIC, payload)

    def drain(self) -> list[tuple[str, bytes]]:
        items = []
        while True:
            try:
                items.append(self._inbox.get_nowait())
            except queue.Empty:
                return items

    def close(self) -> None:
        self._client.loop_stop()
        self._client.disconnect()


def parse_tracked_positions(payload: bytes) -> dict[int, tuple[float, float, float]]:
    """Parse one `robot_pos/all` MQTT message into {tracking_id: (x, y, heading_rad)},
    for EVERYTHING the tracker reports - actual robots and task-object props
    alike, since the tracker has no notion of the difference (a marker on a
    cardboard cylinder looks exactly like a marker on a robot to it). Use
    this directly (not _parse_robot_pos_payload below) when looking up a
    task object's live position by its own tracking id - see
    docs/pipuck_task_simulator.py.

    UNVERIFIED schema (module docstring point 2) - inferred from the RCPS
    dashboard screenshot showing per-entry "Position: [x, y]" and
    "Angle: <deg>°". If the real payload differs, only this function needs
    editing; everything downstream just consumes the returned dict.
    """

    data = json.loads(payload.decode("utf-8"))
    positions: dict[int, tuple[float, float, float]] = {}
    # Try the two most likely shapes: a dict keyed by tracking id, or a list
    # of per-entry records each carrying its own id.
    items = data.items() if isinstance(data, dict) else enumerate(data)
    for key, record in items:
        if not isinstance(record, dict):
            continue
        tracking_id = record.get("id", key)
        try:
            tracking_id = int(tracking_id)
        except (TypeError, ValueError):
            continue
        pos = record.get("position")
        angle_deg = record.get("angle")
        if pos is None or angle_deg is None:
            continue
        positions[tracking_id] = (float(pos[0]), float(pos[1]), math.radians(float(angle_deg)))
    return positions


def _parse_robot_pos_payload(payload: bytes) -> dict[str, tuple[float, float, float]]:
    """Like parse_tracked_positions(), but keyed by this project's "r00"
    style robot_id and filtered to only the entries TRACKING_ID_TO_ROBOT_ID
    maps - i.e. actual fleet robots, not task-object props. This is what
    PiPuckBackend itself uses (a robot only ever needs its own pose)."""

    return {
        TRACKING_ID_TO_ROBOT_ID[tracking_id]: pose
        for tracking_id, pose in parse_tracked_positions(payload).items()
        if tracking_id in TRACKING_ID_TO_ROBOT_ID
    }


@dataclass(slots=True)
class PiPuckBackend:
    """RobotBackend implementation for one real Pi-Puck.

    Movement is the identical rotate-then-cruise, non-blocking state machine
    validated in backends/webots.py's tick() - only the actuation target (a
    real motor driver) and the pose source (MQTT, not a Supervisor
    self-read) differ.
    """

    robot_id: str
    motors: _MotorDriver
    transport: _MqttTransport
    clock: Any = time.monotonic  # callable; injectable for tests
    battery_idle_drain: float = 8e-5
    battery_move_drain_per_m: float = 0.0094
    battery: float = 1.0
    target: tuple[float, float] | None = None
    _phase: str = _Phase.IDLE
    _cruise_start: float = 0.0
    _backoff_until: float = 0.0
    _stuck_check_pos: tuple[float, float] | None = None
    _stuck_check_time: float = 0.0
    _last_pose: tuple[float, float, float] | None = None
    _last_pose_at: float = 0.0
    _protocol_inbox: list[bytes] = field(default_factory=list)
    _detections: list[Detection] = field(default_factory=list)
    _control_inbox: list[dict[str, Any]] = field(default_factory=list)
    _last_rotate_debug_at: float = -1e9
    # Hard physical-safety bounds, independent of the navigation logic above -
    # defaults impose no bound (existing tests/webots don't set these). The
    # pilot controller sets these to the real taped field plus a small
    # margin; see docs/pipuck_market_robot_controller.py.
    field_min: tuple[float, float] = (-1e9, -1e9)
    field_max: tuple[float, float] = (1e9, 1e9)

    # --- RobotBackend protocol ----------------------------------------------
    def get_pose(self) -> tuple[float, float, float]:
        self._drain()
        if self._last_pose is None:
            raise RuntimeError(
                f"{self.robot_id}: no pose received yet on '{POS_TOPIC}' - is the lab's tracking "
                f"system publishing, and is TRACKING_ID_TO_ROBOT_ID mapped correctly for this robot?"
            )
        if self.now() - self._last_pose_at > POSE_STALE_AFTER_S:
            print(f"  WARNING: {self.robot_id} pose is {self.now() - self._last_pose_at:.2f}s stale - lost by the tracker?")
        return self._last_pose

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
        self.transport.publish(msg)

    def receive(self) -> list[bytes]:
        self._drain()
        messages, self._protocol_inbox = self._protocol_inbox, []
        return messages

    def detect_rois(self) -> list[Detection]:
        self._drain()
        detections, self._detections = self._detections, []
        return detections

    def now(self) -> float:
        return float(self.clock())

    # --- Tier-2-only extension, mirrors backends/webots.py's -----------------
    def pop_control_messages(self) -> list[dict[str, Any]]:
        self._drain()
        messages, self._control_inbox = self._control_inbox, []
        return messages

    def tick(self, dt: float) -> None:
        """Advance the motors by one control-loop step. Call this at a fixed
        rate (10-20 Hz suggested) from the robot's controller script - same
        state machine as backends/webots.py's tick()."""

        # Hard safety stop, ahead of and independent of every phase below
        # (including BACKOFF, which otherwise never even reads pose) - a
        # tracked position outside the physical field means something is
        # already wrong (overshoot, a bad waypoint, a tracking glitch), and
        # continuing to drive on the strength of the navigation logic's own
        # judgement is exactly how a robot ends up off the table. Requires
        # a pose to already exist, which main()'s startup loop guarantees
        # before tick() is ever called.
        if self._last_pose is not None:
            safety_x, safety_y, _ = self._last_pose
            if not (self.field_min[0] <= safety_x <= self.field_max[0] and self.field_min[1] <= safety_y <= self.field_max[1]):
                print(
                    f"  SAFETY STOP: {self.robot_id} tracked at ({safety_x:+.3f},{safety_y:+.3f}), "
                    f"outside field bounds {self.field_min}-{self.field_max} - halting."
                )
                self.motors.set_velocity(0.0, 0.0)
                self._phase = _Phase.IDLE
                self._drain_battery(dt, 0.0)
                return

        if self._phase == _Phase.BACKOFF:
            if self.now() < self._backoff_until:
                self.motors.set_velocity(BACKOFF_SPEED, BACKOFF_SPEED)
                self._drain_battery(dt, abs(BACKOFF_SPEED) * WHEEL_RADIUS * dt)
                return
            self._phase = _Phase.IDLE

        if self.target is None or self.is_at_target():
            self.motors.set_velocity(0.0, 0.0)
            self._phase = _Phase.IDLE
            self._drain_battery(dt, 0.0)
            return

        px, py, heading = self.get_pose()
        if self.now() - self._last_pose_at > POSE_STALE_AFTER_S:
            # Feedback is too old to steer on safely - acting on it anyway is
            # exactly what produced in-place jittering on real hardware: the
            # rotate controller corrects against a position/heading that's
            # already >1s out of date, overshoots, "corrects" again against
            # the next equally-stale reading, and limit-cycles instead of
            # converging. Hold still until the tracker catches up again
            # rather than steering on stale data (get_pose() already printed
            # the staleness warning above).
            self.motors.set_velocity(0.0, 0.0)
            self._drain_battery(dt, 0.0)
            return
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
                self.motors.set_velocity(0.0, 0.0)
            else:
                turn = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, ROTATE_GAIN * error))
                # VERIFIED WRONG on-site (2026-08-20 pilot logs): heading kept
                # climbing tick over tick while `turn` stayed negative and
                # grew toward -MAX_WHEEL_SPEED - the real robot turns the
                # opposite way from what this command intends, whether that's
                # robot_pos/all's angle sign convention or the physical
                # left/right wiring. Swapped which side gets +turn/-turn to
                # correct the closed loop's sign; forward driving (CRUISE/
                # BACKOFF below) is untouched since both wheels always get
                # the same value there.
                self.motors.set_velocity(turn, -turn)
                # Diagnostic for the on-site heading-convention/motor-polarity
                # check (see module docstring point 2: robot_pos/all's angle
                # sign/reference axis was never actually verified against a
                # real payload) - if `error` doesn't shrink tick over tick
                # while this prints, or the robot visibly spins opposite to
                # what a positive/negative `turn` implies, that convention is
                # wrong, not the control law. Throttled to ~3 Hz so it stays
                # readable at 15 Hz control rate.
                if self.now() - self._last_rotate_debug_at >= 0.3:
                    self._last_rotate_debug_at = self.now()
                    print(
                        f"    [rotate] {self.robot_id} heading={math.degrees(heading):+7.2f} deg "
                        f"bearing={math.degrees(bearing):+7.2f} deg error={math.degrees(error):+7.2f} deg "
                        f"turn={turn:+.2f} rad/s pos=({px:+.3f},{py:+.3f}) target=({self.target[0]:+.3f},{self.target[1]:+.3f})"
                    )
            self._drain_battery(dt, 0.0)
            return

        drift_deg = math.degrees(abs(_wrap(bearing - heading)))
        if drift_deg > REALIGN_THRESHOLD_DEG:
            self._phase = _Phase.ROTATE
            self.motors.set_velocity(0.0, 0.0)
            self._drain_battery(dt, 0.0)
            return

        if self.now() - self._stuck_check_time >= STUCK_CHECK_INTERVAL_S:
            progress = math.hypot(px - self._stuck_check_pos[0], py - self._stuck_check_pos[1])
            self._stuck_check_pos = (px, py)
            self._stuck_check_time = self.now()
            if progress < STUCK_DISTANCE_THRESHOLD:
                self._phase = _Phase.BACKOFF
                self._backoff_until = self.now() + BACKOFF_DURATION_S
                self.motors.set_velocity(BACKOFF_SPEED, BACKOFF_SPEED)
                self._drain_battery(dt, abs(BACKOFF_SPEED) * WHEEL_RADIUS * dt)
                return

        ramp_fraction = min(1.0, (self.now() - self._cruise_start) / CRUISE_RAMP_S)
        speed = CRUISE_SPEED * ramp_fraction
        self.motors.set_velocity(speed, speed)
        self._drain_battery(dt, speed * WHEEL_RADIUS * dt)

    # --- internals -----------------------------------------------------------
    def _drain_battery(self, dt: float, travelled_m: float) -> None:
        self.battery = max(
            0.0, self.battery - self.battery_idle_drain * dt - self.battery_move_drain_per_m * travelled_m
        )

    def _drain(self) -> None:
        for topic, raw in self.transport.drain():
            if topic == POS_TOPIC:
                poses = _parse_robot_pos_payload(raw)
                pose = poses.get(self.robot_id)
                if pose is not None:
                    self._last_pose = pose
                    self._last_pose_at = self.now()
            elif topic == WIRE_TOPIC:
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
        cam = payload.get("cam")
        if cam == CAM_DETECT:
            if payload.get("to") == self.robot_id:
                self._detections.append(
                    Detection(str(payload["event_id"]), tuple(payload["pos"]), str(payload["task_type"]))
                )
                self._control_inbox.append(payload)
            return
        ctl = payload.get("ctl")
        if ctl in (CTL_ACTIVE, CTL_COORD_TIMEOUT):
            self._control_inbox.append(payload)
        # unrecognised payloads are ignored, not raised - keeps the shared
        # topic forward-compatible, same policy as backends/webots.py


def task_from_detection_payload(payload: dict[str, Any], roi_aging_lambda: float, priority_k: float, now: float) -> Task:
    """Build the same Task sim/world.py's World._announce_event() would, from
    a CAM_DETECT payload - identical in spirit to
    backends/webots.py's task_from_detection_payload()."""

    return Task(
        str(payload["event_id"]).replace("event", "roi"),
        TaskType(payload["task_type"]),
        tuple(payload["pos"]),
        int(payload["n_req"]),
        PriorityFn(float(payload["p0"]), roi_aging_lambda, priority_k, now),
        float(payload["duration"]),
        now,
    )
