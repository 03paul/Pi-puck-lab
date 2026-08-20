"""Webots "overhead camera" controller: privileged position reads + cross-robot bookkeeping.

STATUS: untested starting point, not a verified controller - same caveat as every other
docs/webots_*.py file in this repository; nothing below has run inside Webots.

Plays exactly the role the proposal assigns the overhead camera (§2.1): "Localisation may be
centralised. Decision-making may not." This process never decides who wins an auction - it
only (a) tells a robot when it has detected an RoI or should host a stale SURVEY cell, mirroring
sim/world.py's World._update_observation()/_announce_surveys(), and (b) tells the fleet when
enough winners have physically arrived at a task, mirroring World._update_execution(). Both are
already centralised in Tier 0 (World has privileged access to every Robot object); this process
gets the same information the same way Tier 0 does - by reading ground-truth positions - just
over the scene tree's Field API instead of direct Python attribute access, and it learns *who
is assigned to what* by passively listening to the fleet's own AWARD broadcasts rather than
being told out of band.

Setup:
  1. Add ONE extra Robot node (no wheels needed) with `supervisor TRUE`, an Emitter and a
     Receiver on the SAME channel as the fleet, named e.g. "camera". Assign this controller
     (`webots_market_supervisor_controller`) to it.
  2. FLEET_SIZE/SEED/STRATEGY_NAME below must match docs/webots_market_robot_controller.py's.
  3. After the run, merge this process's events.csv with each robot's state_r*.csv (see
     docs/WEBOTS_MARKET_DEMO.md) and run metrics/analysis.py - same portable log schema as
     Tier 0, so the same analysis code applies unchanged.
"""

from __future__ import annotations

import json
import sys
from dataclasses import replace
from math import ceil, dist
from pathlib import Path

from controller import Supervisor  # type: ignore[import-not-found]

# EDIT ME: absolute path to the decentralized-mrta repo checkout on this machine - keep this
# identical to docs/webots_market_robot_controller.py's REPO_ROOT. Can't be derived from
# __file__ here either, same reason (see that file's comment).
REPO_ROOT = Path(r"/Users/03paul_/Desktop/Studium/Praktikum Verteilte Robotiksysteme/decentralized-mrta")
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from allocation.messages import Message, MessageType, done_message
from allocation.tasks import TaskType
from allocation.bidding import BidWeights
from allocation.strategies import Strategy, StrategyName
from backends.webots import (
    ARRIVAL_TOLERANCE,
    CTL_ACTIVE,
    CTL_COORD_TIMEOUT,
    CTL_DETECT_ROI,
    CTL_EXPLORE_TARGET,
    CTL_HOST_SURVEY,
    find_node_by_name,
)
from metrics.logger import RunLogger
from sim.config import SimulationConfig
from sim.world import EventSpec, World  # World only to harvest the RoI schedule when TEST_SINGLE_OBJECT is off

TIME_STEP = 64  # ms
FLEET_SIZE = 7  # must match docs/webots_market_robot_controller.py's FLEET_SIZE
SEED = 1000  # a Tier-0 holdout seed (1000-1039), for direct comparability with comparison_summary.md
STRATEGY_NAME = StrategyName.RANDOM  # must match the fleet robots' STRATEGY_NAME

# Tier-1-only calibration override, NOT a change to allocation/ or to parameters/default.json's
# canonical 90.0 (Tier-0 comparison_summary.md numbers are unaffected - assignment_timeout is
# only ever read here, see sim/world.py's World._update_execution(), which this loop mirrors;
# allocation/ never touches it). 90s was calibrated against Tier 0's abstract effective_speed
# (0.08 m/s, instantaneous heading). Real Webots travel is slower AND has overhead Tier 0 doesn't
# model (rotation, ramp-up, realignment) - see backends/webots.py's CRUISE_SPEED comment - so a
# robot with workload>1 (assignments[0]-only navigation, matches Tier 0) routinely never even
# starts its 2nd/3rd queued task before 90s elapses, and SURROUND (n_req>1) additionally needs
# *multiple* winners to each clear that bar at once. First live run at 90s: only 1 of ~30
# announced tasks ever reached ACTIVE in a 300s run. Raised well above the diagonal-crossing time
# (~35-45s one-way at the new CRUISE_SPEED, plus queueing) - still bounded well under `duration`
# so a genuinely stuck task doesn't block a robot for the whole run, but expect to retune further.
ASSIGNMENT_TIMEOUT_OVERRIDE_S = 240.0

# Minimal isolation test, per explicit request: spawn exactly ONE object and see whether the
# market can get n_req robots to it and simultaneously ACTIVE at all - stripped of every other
# source of churn (continuous RoI generation, survey hosting, preemption from competing tasks)
# that made diagnosing the general-throughput collapse hard. SURROUND (n_req=3, duration=10.0,
# see sim/world.py's event generator) is the task type that matches "3 Roboter, die das Objekt
# umschließen" exactly. Served its purpose (found the HEADING_OFFSET and ARRIVAL_TOLERANCE bugs,
# both fixed above) - GUARD/SURROUND/SURVEY now reach ACTIVE/DONE repeatedly in a full run.
TEST_SINGLE_OBJECT = False
TEST_EVENT_RELEASE_AT = 5.0  # s - spawn shortly after start, not immediately (mirrors real events)

# Visible task objects: the RoI detection itself is already the proposal's own "virtual
# detection sensor" fallback (§2.2) - the supervisor knows event ground truth and tells the
# nearest robot once it's within detection_radius, no on-board vision required. What was
# missing is the physical prop side: in the real lab these are cardboard cylinders with a
# QR-style marker on top (detected the same way peer robots are, but the ID tells the robot
# "this is a task object, not a robot"). Spawning a real Solid at the event's scheduled
# position/time - not just tracking it as an abstract coordinate - is the Webots equivalent.
#
# Purely visual, no boundingObject/physics, for every task type including PUSH: a real
# collision-physics prop (winners shoving it to a target zone, completion decided by the
# object's position) was built and live-tested, but the object never reliably moved - Webots'
# default contact friction easily exceeds what a light e-puck can push through. Reverted by
# explicit decision rather than chasing WorldInfo.contactProperties tuning: PUSH is excluded
# from the real hardware demo anyway ("Push will not be demonstrated on hardware... requires
# force closure we do not believe we can achieve reliably"), and GUARD/SURROUND/SURVEY already
# demonstrate the thing Tier 1 actually needs to show. PUSH now completes the same way GUARD/
# SURROUND do: n_req winners arrive and hold position for `duration`, object disappears, DONE.
OBJECT_APPEARANCE = {
    "push": (0.62, 0.47, 0.32),  # cardboard-cylinder tan, matches the real lab prop colour
    "guard": (0.85, 0.2, 0.15),  # a located find - marked red
    "surround": (0.9, 0.75, 0.1),  # a spreading hazard - warning yellow
}
OBJECT_RADIUS = 0.03  # m
OBJECT_HEIGHT = 0.06  # m

CONFIG_PATH = REPO_ROOT / "parameters" / "default.json"
# Per-strategy subfolder, not a flat "webots_logs" - every run used to write the exact same
# filenames, so switching STRATEGY_NAME and re-running silently overwrote the previous strategy's
# logs with no warning (lost an entire Market run this way). Keyed off STRATEGY_NAME so Market/
# Greedy/Random runs coexist on disk automatically - nothing to remember to rename by hand. Must
# resolve to the same subfolder docs/webots_market_robot_controller.py's own STRATEGY_NAME does.
LOG_DIR = Path("webots_logs") / STRATEGY_NAME.value
DEBUG = True  # console prints for every auction-lifecycle event; set False once runs are reliable


def _sector_waypoints(config: SimulationConfig, robot_count: int, robot_index: int) -> list[tuple[float, float]]:
    """Mirrors sim/world.py's World._sector_waypoints() - kept in sync manually, see module docstring."""

    left = robot_index * config.arena_width / robot_count
    right = (robot_index + 1) * config.arena_width / robot_count
    x_a, x_b = left + 0.03, right - 0.03
    rows = max(2, ceil(config.arena_height / (2.0 * config.detection_radius)))
    waypoints: list[tuple[float, float]] = []
    for row in range(rows):
        y = min(config.arena_height - 0.05, 0.05 + row * config.arena_height / max(1, rows - 1))
        waypoints.append((x_a if row % 2 == 0 else x_b, y))
        waypoints.append((x_b if row % 2 == 0 else x_a, y))
    return waypoints


def _grid_centres(config: SimulationConfig, cell: float, cols: int, rows: int) -> list[tuple[float, float]]:
    """Mirrors sim/world.py's World._grid_centres() - kept in sync manually, see module docstring."""

    return [
        (min(config.arena_width, (col + 0.5) * cell), min(config.arena_height, (row + 0.5) * cell))
        for row in range(rows)
        for col in range(cols)
    ]


def spawn_task_object(supervisor: Supervisor, position: tuple[float, float], task_type: str, name: str):
    """Import a visible Solid prop at (x, y) - the physical stand-in for a lab cardboard
    cylinder. Returns the new Node, or None if this task_type has no visual (e.g. survey).

    `name` is set on the node so a fleet robot could find *this specific* object later via
    find_node_by_name(), the same name-based lookup already used for fleet robots - kept even
    though nothing currently needs to (no live PUSH-tracking), since it costs nothing and keeps
    every spawned node individually identifiable for debugging.

    Deliberately no boundingObject/physics (visual only) - see OBJECT_APPEARANCE's comment for
    why a real-physics version was tried and reverted. Completion for every task type here runs
    on the arrival + duration timer (see the n_req arrival + completion loop below), not on
    anything about the object's own physical state.
    """

    color = OBJECT_APPEARANCE.get(task_type)
    if color is None:
        return None
    vrml = (
        "Solid {\n"
        f"  translation {position[0]} {position[1]} {OBJECT_HEIGHT / 2}\n"
        f'  name "{name}"\n'
        "  children [\n"
        "    Shape {\n"
        f"      appearance PBRAppearance {{ baseColor {color[0]} {color[1]} {color[2]} }}\n"
        f"      geometry Cylinder {{ height {OBJECT_HEIGHT} radius {OBJECT_RADIUS} }}\n"
        "    }\n"
        "  ]\n"
        "}\n"
    )
    children_field = supervisor.getRoot().getField("children")
    children_field.importMFNodeFromString(-1, vrml)
    return children_field.getMFNode(children_field.getCount() - 1)


class TaskRuntime:
    __slots__ = (
        "position",
        "n_req",
        "duration",
        "task_type",
        "host",
        "survey_cell",
        "winners",
        "award_at",
        "active_at",
    )

    def __init__(
        self,
        position: tuple[float, float],
        n_req: int,
        duration: float,
        task_type: str,
        host: str,
        survey_cell: int | None,
    ) -> None:
        self.position = position
        self.n_req = n_req
        self.duration = duration
        self.task_type = task_type
        self.host = host
        self.survey_cell = survey_cell
        self.winners: list[str] | None = None
        self.award_at: float | None = None
        self.active_at: float | None = None


def main() -> None:
    supervisor = Supervisor()
    config = SimulationConfig.from_json(CONFIG_PATH)
    # frozen dataclass - replace() rather than attribute assignment; see the constants' comments
    # above and backends.webots.ARRIVAL_TOLERANCE's comment for arrival_tolerance - must match
    # docs/webots_market_robot_controller.py's override or the two processes disagree on arrival
    config = replace(config, assignment_timeout=ASSIGNMENT_TIMEOUT_OVERRIDE_S, arrival_tolerance=ARRIVAL_TOLERANCE)
    strategy = Strategy(STRATEGY_NAME, BidWeights())  # weights unused: only world.events is harvested below

    emitter = supervisor.getDevice("emitter")
    receiver = supervisor.getDevice("receiver")
    receiver.enable(TIME_STEP)

    fleet_ids = [f"r{i:02d}" for i in range(FLEET_SIZE)]
    fleet_nodes = {robot_id: find_node_by_name(supervisor.getRoot(), robot_id) for robot_id in fleet_ids}
    missing = [robot_id for robot_id, node in fleet_nodes.items() if node is None]
    if missing:
        raise RuntimeError(
            f"could not find a node with name field {missing} anywhere in the scene tree - check "
            "each fleet robot's `name` field in the Scene Tree panel matches exactly (e.g. \"r00\")"
        )

    if TEST_SINGLE_OBJECT:
        # One SURROUND object at the arena centre, nothing else - see TEST_SINGLE_OBJECT's comment.
        centre = (config.arena_width / 2.0, config.arena_height / 2.0)
        events = [EventSpec("event-000", TEST_EVENT_RELEASE_AT, centre, TaskType.SURROUND, 3, 10.0)]
    else:
        # Reuse Tier 0's own deterministic scenario generator for the RoI schedule, so the exact
        # same event timings/positions/types used in results/comparison_summary.md are available
        # here for a like-for-like comparison. Everything else about this World instance (its own
        # robots, its own message bus) is discarded - only .events is used.
        events = World(config, strategy, SEED).events

    survey_cols = ceil(config.arena_width / config.survey_cell)
    survey_rows = ceil(config.arena_height / config.survey_cell)
    survey_centres = _grid_centres(config, config.survey_cell, survey_cols, survey_rows)
    survey_last_seen = [0.0 for _ in survey_centres]
    survey_cooldown_until = [0.0 for _ in survey_centres]
    active_survey_cells: dict[int, str] = {}
    survey_counter = 0
    next_survey_announcement = 0.0

    detected_events: set[str] = set()
    tasks: dict[str, TaskRuntime] = {}
    explore_waypoints = [_sector_waypoints(config, FLEET_SIZE, index) for index in range(FLEET_SIZE)]
    explore_indices = [0 for _ in range(FLEET_SIZE)]

    # Visible task-object props: spawned at each event's scheduled release time (not at
    # detection time - the object exists physically before anyone notices it, same as in the
    # lab), removed once the corresponding task completes. task_id_to_event_id lets the DONE
    # handler below find which spawned object a completed roi-* task corresponds to.
    spawned_objects: dict[str, object] = {}  # event_id -> Solid node
    task_id_to_event_id: dict[str, str] = {}

    logger = RunLogger()

    def positions() -> dict[str, tuple[float, float]]:
        # Defensive per-robot try/except: a transient Webots hiccup on one robot's
        # getPosition() must not crash the whole 300s run and take every robot's progress
        # down with it. A robot missing from the returned dict is simply treated as "not
        # ready yet" wherever pose is consulted below (all lookups use pose.get(), never
        # pose[robot_id] directly) rather than raising.
        result: dict[str, tuple[float, float]] = {}
        for robot_id, node in fleet_nodes.items():
            try:
                result[robot_id] = tuple(node.getPosition()[:2])
            except Exception as exc:  # noqa: BLE001 - deliberately broad, see comment above
                if DEBUG:
                    print(f"  WARNING: getPosition() failed for {robot_id}: {exc!r} - skipping this tick")
        return result

    def log_message(now: float, message: Message) -> None:
        logger.event(now, message.kind.value, str(message.fields["task_id"]), str(message.fields["sender"]), dict(message.fields))
        if DEBUG:
            print(f"  t={now:6.1f}s overheard {message.kind.value:8s} task={message.fields['task_id']} from={message.fields['sender']}")

    def complete_task(task_id: str, runtime: "TaskRuntime", now: float) -> None:
        """Send DONE and tear down every side-effect of a finished task."""

        sender = min(runtime.winners) if runtime.winners else runtime.host
        message = done_message(task_id, sender, now)
        emitter.send(message.to_bytes())
        log_message(now, message)
        if runtime.survey_cell is not None:
            survey_last_seen[runtime.survey_cell] = now
            active_survey_cells.pop(runtime.survey_cell, None)
            survey_cooldown_until[runtime.survey_cell] = now + config.survey_cooldown
        event_id = task_id_to_event_id.pop(task_id, None)
        if event_id is not None:
            node = spawned_objects.pop(event_id, None)
            if node is not None:
                node.remove()  # n_req robots surrounded/guarded it for `duration` - resolved
        tasks.pop(task_id, None)

    while supervisor.step(TIME_STEP) != -1:
        now = supervisor.getTime()
        pose = positions()

        # --- passively listen to the fleet's own broadcasts -------------------------------
        while receiver.getQueueLength() > 0:
            raw = bytes(receiver.getBytes())
            receiver.nextPacket()
            try:
                message = Message.from_bytes(raw)
            except (ValueError, KeyError, UnicodeDecodeError, json.JSONDecodeError):
                continue  # a control-plane payload from this same process, or malformed - ignore
            log_message(now, message)
            if message.kind is MessageType.ANNOUNCE:
                fields = message.fields
                task_type = str(fields["type"])
                survey_cell = None
                if task_type == TaskType.SURVEY.value:
                    # the task_id encodes the cell index: "survey-<cell>-<counter>"
                    survey_cell = int(str(fields["task_id"]).split("-")[1])
                tasks[str(fields["task_id"])] = TaskRuntime(
                    tuple(fields["pos"]),
                    int(fields["n_req"]),
                    float(fields["duration"]),
                    task_type,
                    str(fields["sender"]),
                    survey_cell,
                )
            elif message.kind is MessageType.AWARD:
                runtime = tasks.get(str(message.fields["task_id"]))
                if runtime is not None:
                    runtime.winners = [str(w) for w in message.fields["winners"]]
                    runtime.award_at = now
            elif message.kind is MessageType.RELEASE:
                runtime = tasks.get(str(message.fields["task_id"]))
                if runtime is not None and runtime.active_at is None:
                    runtime.winners = None
                    runtime.award_at = None
            elif message.kind is MessageType.DONE:
                tasks.pop(str(message.fields["task_id"]), None)

        # --- spawn the physical prop for each event once its scheduled time arrives - the
        # object exists (and is visible in the 3D view) before anyone necessarily notices it,
        # same as a real cardboard cylinder placed in the arena ahead of detection ------------
        for event in events:
            if event.event_id in spawned_objects or event.release_at > now:
                continue
            node = spawn_task_object(supervisor, event.position, event.task_type.value, event.event_id)
            spawned_objects[event.event_id] = node  # None for types with no visual (e.g. survey)
            task_id_to_event_id[event.event_id.replace("event", "roi")] = event.event_id

        # --- RoI detection (mirrors World._update_observation()'s event-detection loop) ---
        radius = config.detection_radius
        for event in events:
            if event.event_id in detected_events or event.release_at > now:
                continue
            candidates = [
                robot_id for robot_id, xy in pose.items() if dist(xy, event.position) <= radius
            ]
            if not candidates:
                continue
            detector = min(candidates, key=lambda robot_id: (dist(pose[robot_id], event.position), robot_id))
            detected_events.add(event.event_id)
            p0 = {
                TaskType.PUSH: config.priority_push,
                TaskType.SURROUND: config.priority_surround,
                TaskType.GUARD: config.priority_guard,
            }[event.task_type]
            detect_payload = {
                "ctl": CTL_DETECT_ROI,
                "to": detector,
                "event_id": event.event_id,
                "task_type": event.task_type.value,
                "pos": list(event.position),
                "n_req": event.n_req,
                "duration": event.duration,
                "p0": p0,
            }
            emitter.send(json.dumps(detect_payload).encode("utf-8"))
            logger.event(now, "DETECTED", event.event_id, detector, {"event_created_at": event.release_at})
            if DEBUG:
                print(f"  t={now:6.1f}s DETECT_ROI  -> {detector}  event={event.event_id} type={event.task_type.value}")

        # --- survey staleness + hosting (mirrors _update_observation + _announce_surveys) -
        # Skipped entirely in TEST_SINGLE_OBJECT: the point is isolating the single SURROUND
        # object with zero competing tasks, and survey hosting would otherwise keep announcing
        # on its own schedule regardless of the RoI event list being trimmed to one entry.
        if TEST_SINGLE_OBJECT:
            survey_announcement_due = False
        else:
            for cell_index, centre in enumerate(survey_centres):
                if any(dist(xy, centre) <= radius for xy in pose.values()):
                    survey_last_seen[cell_index] = now
            survey_announcement_due = now >= next_survey_announcement
        if survey_announcement_due:
            next_survey_announcement = now + config.survey_announce_interval
            candidates: list[tuple[float, int]] = []
            for index, last_seen in enumerate(survey_last_seen):
                if index in active_survey_cells or survey_cooldown_until[index] > now:
                    continue
                dt = max(0.0, now - last_seen)
                priority = 1.0 - pow(2.718281828, -((config.lambda_model * dt) ** config.priority_k))
                if priority >= config.survey_priority_threshold:
                    candidates.append((priority, index))
            candidates.sort(key=lambda item: (-item[0], item[1]))
            for _, cell_index in candidates[: config.max_survey_announcements]:
                host = fleet_ids[cell_index % FLEET_SIZE]
                task_id = f"survey-{cell_index:02d}-{survey_counter:04d}"
                survey_counter += 1
                active_survey_cells[cell_index] = task_id
                emitter.send(
                    json.dumps(
                        {
                            "ctl": CTL_HOST_SURVEY,
                            "to": host,
                            "task_id": task_id,
                            "pos": list(survey_centres[cell_index]),
                            "last_seen": survey_last_seen[cell_index],
                        }
                    ).encode("utf-8")
                )
                if DEBUG:
                    print(f"  t={now:6.1f}s HOST_SURVEY -> {host}  cell={cell_index} task={task_id}")

        # --- n_req arrival + completion (mirrors World._update_execution()) ---------------
        # Same rule for every task type, PUSH included: n_req winners hold position for
        # `duration`, the object disappears, DONE - see OBJECT_APPEARANCE's comment for why
        # PUSH no longer tries to decide completion from the object's own physical position.
        for task_id, runtime in list(tasks.items()):
            if runtime.winners is None:
                continue
            if runtime.active_at is None:
                if runtime.award_at is not None and now - runtime.award_at > config.assignment_timeout:
                    emitter.send(json.dumps({"ctl": CTL_COORD_TIMEOUT, "task_id": task_id}).encode("utf-8"))
                    if DEBUG:
                        print(f"  t={now:6.1f}s COORD_TIMEOUT task={task_id} winners={runtime.winners}")
                    runtime.winners = None
                    runtime.award_at = None
                    continue
                # pose.get(w) is None if getPosition() failed for w this tick (see positions())
                # - treat a winner whose pose we don't have yet as "not arrived", not a crash.
                ready = all(
                    pose.get(w) is not None and dist(pose[w], runtime.position) <= config.arrival_tolerance
                    for w in runtime.winners
                )
                if ready and len(runtime.winners) == runtime.n_req:
                    runtime.active_at = now
                    emitter.send(json.dumps({"ctl": CTL_ACTIVE, "task_id": task_id}).encode("utf-8"))
                    if DEBUG:
                        print(f"  t={now:6.1f}s ACTIVE      task={task_id} winners={runtime.winners}")
            elif now - runtime.active_at + 1e-9 >= runtime.duration:
                complete_task(task_id, runtime, now)

        # --- explore-target assignment for currently-unassigned robots ---------------------
        assigned_robots = {w for runtime in tasks.values() if runtime.winners for w in runtime.winners}
        for index, robot_id in enumerate(fleet_ids):
            if robot_id in assigned_robots or robot_id not in pose:
                continue
            waypoints = explore_waypoints[index]
            target = waypoints[explore_indices[index] % len(waypoints)]
            if dist(pose[robot_id], target) <= config.arrival_tolerance:
                explore_indices[index] += 1
                target = waypoints[explore_indices[index] % len(waypoints)]
            emitter.send(
                json.dumps({"ctl": CTL_EXPLORE_TARGET, "to": robot_id, "pos": list(target)}).encode("utf-8")
            )

        if now >= config.duration:
            break

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    events_path, _ = logger.write(LOG_DIR)
    print(f"supervisor: wrote {len(logger.events)} events to {events_path}")


if __name__ == "__main__":
    main()
