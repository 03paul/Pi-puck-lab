"""Real Pi-Puck controller: one process per robot, running the unmodified
market/greedy/random allocation logic - the exact same `sim.robot.Robot`
and `allocation/` code already used unchanged in Tier 0 and Tier 1
(docs/webots_market_robot_controller.py), now driven by
backends/pipuck.PiPuckBackend (MQTT transport + the lab's own RCPS
tracking system for pose) instead of the abstract or Webots backend.

STATUS: written ahead of the first hardware pilot session, not yet run on
real Pi-Pucks. See docs/LAB_PILOT_CHECKLIST.md before the first run.

Run on each Pi-Puck as:
    python3 pipuck_market_robot_controller.py r00
    python3 pipuck_market_robot_controller.py r01
(the robot_id argument must match one of backends/pipuck.py's
TRACKING_ID_TO_ROBOT_ID values, i.e. the numeric id the RCPS dashboard
shows for that specific physical robot).
"""

from __future__ import annotations

import csv
import json
import sys
import time
import traceback
from dataclasses import replace
from pathlib import Path
from random import Random

import _py37_compat  # noqa: F401 - must run before any allocation/sim/backends import, see its docstring

# math.dist is one of the things _py37_compat patches in - must be imported
# after it too, same reason as the allocation/sim/backends imports below.
from math import dist

# Auto-detected, unlike docs/webots_market_robot_controller.py's hardcoded
# REPO_ROOT: Webots *copies* controller files into its own controllers/
# directory before running them, so __file__ points into the Webots
# project, not this repo - that trick doesn't apply here. This script runs
# directly out of the cloned repo on the Pi-Puck (git clone, then
# `python3 docs/pipuck_market_robot_controller.py r00`), so __file__
# genuinely lives inside the checkout and this resolves correctly with no
# per-machine editing needed.
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from allocation.bidding import BidWeights
from allocation.messages import Message, MessageType
from allocation.strategies import Strategy, StrategyName
from backends.pipuck import (
    ARRIVAL_TOLERANCE,
    CTL_ACTIVE,
    CTL_COORD_TIMEOUT,
    PiPuckBackend,
    _MotorDriver,
    _MqttTransport,
    task_from_detection_payload,
)
from metrics.logger import STATE_FIELDS, RunLogger
from sim.config import SimulationConfig
from sim.robot import Robot

TICK_HZ = 15.0  # control-loop rate; VERIFY this is achievable given the tracking system's own publish rate and motor I/O latency
CONFIG_PATH = REPO_ROOT / "parameters" / "default.json"
BEST_WEIGHTS_PATH = REPO_ROOT / "results" / "best_parameters.json"
LOG_DIR = Path("pipuck_logs")
DEBUG = True

# Pilot uses Market by default - per docs/LAB_PILOT_CHECKLIST.md, this is a
# risk-elimination run for the wire protocol and movement stack, not the
# strategy comparison (that already ran in Tier 0/1 with statistical
# power this single physical trial cannot match).
STRATEGY_NAME = StrategyName.MARKET

# Same rationale as Tier 1's PREEMPTION_COOLDOWN_OVERRIDE_S (see
# docs/webots_market_robot_controller.py) - real travel time is slower than
# Tier 0's abstract model assumes. With only one task and two robots in the
# pilot this should rarely matter, but costs nothing to keep consistent.
PREEMPTION_COOLDOWN_OVERRIDE_S = 90.0

# parameters/default.json's arena_width/arena_height (2.0 x 2.0) is shared
# with Tier 0's abstract experiments and Tier 1's Webots world - do NOT edit
# that file for this pilot, it would shift experiments/replay.py's plot
# scaling and docs/webots_market_supervisor_controller.py's task-spawn grid
# for those already-calibrated tiers. Override locally instead, same pattern
# as PREEMPTION_COOLDOWN_OVERRIDE_S above. VERIFY against the lab's actual
# taped-out field before each session - this matters beyond cosmetics: both
# used below via arena_diagonal() to normalise the MARKET strategy's bid
# distance term (sim/robot.py), so a wrong arena size skews bids relative to
# how results/best_parameters.json's weights were actually calibrated.
PILOT_ARENA_WIDTH_M = 2.0
PILOT_ARENA_HEIGHT_M = 1.0

# Hard physical-safety net (backends.pipuck.PiPuckBackend.field_min/field_max):
# on real hardware, overshoot past a waypoint or a bad tracked reading can
# drive a robot off the taped field before the navigation logic itself
# would ever notice something's wrong. 2026-08-20: robots repeatedly went
# over the edge - CHANGED FROM a small buffer OUTSIDE the tape to a 0.3m
# INSET, i.e. the robot now halts 0.3m before the real edge, not 0.15m past
# it. On the 2x1m field this leaves a 1.4m x 0.4m safe zone - narrow, but
# safety over coverage per explicit instruction.
FIELD_SAFETY_INSET_M = 0.3

# Idle robots need to actually search the field for the GUARD task instead of
# holding still - detection is purely proximity-based (docs/pipuck_task_simulator.py's
# DETECTION_RADIUS), so an idle robot that never moves may just never come
# close enough to trigger it. Mirrors what docs/webots_market_supervisor_controller.py
# does centrally via CTL_EXPLORE_TARGET, but simpler: no coordination needed
# for a single-task, two-robot pilot, so each robot just picks its own
# random point in the field and re-picks on arrival - see explore_target
# handling in _run_loop below.
# Kept 0.05m inside FIELD_SAFETY_INSET_M so a waypoint itself never sits
# right on the safety boundary - normal arrival slop there would trip the
# safety stop on every single exploration leg.
EXPLORE_MARGIN_M = FIELD_SAFETY_INSET_M + 0.05


def _random_explore_target(rng: Random, config: SimulationConfig) -> tuple[float, float]:
    x = rng.uniform(EXPLORE_MARGIN_M, max(EXPLORE_MARGIN_M, config.arena_width - EXPLORE_MARGIN_M))
    y = rng.uniform(EXPLORE_MARGIN_M, max(EXPLORE_MARGIN_M, config.arena_height - EXPLORE_MARGIN_M))
    return (x, y)


def _load_strategy() -> Strategy:
    weights = json.loads(BEST_WEIGHTS_PATH.read_text(encoding="utf-8"))["market_weights"]
    return Strategy(STRATEGY_NAME, BidWeights(**weights))


def main() -> None:
    if len(sys.argv) < 2:
        raise SystemExit("usage: python3 pipuck_market_robot_controller.py <robot_id, e.g. r00>")
    robot_id = sys.argv[1]

    config = SimulationConfig.from_json(CONFIG_PATH)
    config = replace(
        config,
        preemption_cooldown=PREEMPTION_COOLDOWN_OVERRIDE_S,
        arrival_tolerance=ARRIVAL_TOLERANCE,
        arena_width=PILOT_ARENA_WIDTH_M,
        arena_height=PILOT_ARENA_HEIGHT_M,
    )
    strategy = _load_strategy()

    transport = _MqttTransport()  # connects to MQTT_BROKER, subscribes to POS_TOPIC + WIRE_TOPIC - see backends/pipuck.py
    motors = _MotorDriver()  # VERIFY against docs/LAB_PILOT_CHECKLIST.md item 1 before running
    backend = PiPuckBackend(
        robot_id=robot_id,
        motors=motors,
        transport=transport,
        battery_idle_drain=config.battery_idle_drain,
        battery_move_drain_per_m=config.battery_move_drain_per_m,
        battery=config.initial_battery_max,
        field_min=(FIELD_SAFETY_INSET_M, FIELD_SAFETY_INSET_M),
        field_max=(PILOT_ARENA_WIDTH_M - FIELD_SAFETY_INSET_M, PILOT_ARENA_HEIGHT_M - FIELD_SAFETY_INSET_M),
    )

    print(f"  {robot_id}: waiting for first tracked pose on robot_pos/all...")
    while backend._last_pose is None:
        backend._drain()
        time.sleep(0.1)
    print(f"  {robot_id}: pose received, starting.")

    robot_index = int(robot_id[1:])
    robot = Robot(
        robot_id,
        backend.get_pose()[:2],
        config.initial_battery_max,
        strategy,
        Random(robot_index * 97 + 31),
        config,
        backend=backend,
    )

    # Separate stream from the robot's own bid-tiebreak Random(...) above -
    # unrelated concerns, no reason to couple their sequences.
    explore_rng = Random(robot_index * 97 + 31 + 500)
    explore_target = _random_explore_target(explore_rng, config)
    active_tasks: set[str] = set()
    logger = RunLogger()
    dt = 1.0 / TICK_HZ
    start = time.monotonic()

    print(f"  {robot_id}: STRATEGY_NAME={STRATEGY_NAME!r} module={__file__}")

    try:
        _run_loop(robot_id, robot, backend, config, active_tasks, explore_target, explore_rng, logger, dt, start)
    finally:
        # Always write whatever was logged, even on Ctrl+C or an exception -
        # a partial log from an aborted pilot run is still worth keeping.
        transport.close()
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        with (LOG_DIR / f"state_{robot_id}.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, STATE_FIELDS, lineterminator="\n")
            writer.writeheader()
            writer.writerows(logger.states)
        print(f"{robot_id}: wrote {len(logger.states)} state rows to {LOG_DIR / f'state_{robot_id}.csv'}")


def _run_loop(robot_id, robot, backend, config, active_tasks, explore_target, explore_rng, logger, dt, start) -> None:
    next_state_log = 0.0
    last_assignments: list[str] = []

    while True:
        loop_start = time.monotonic()
        now = loop_start - start
        backend.tick(dt)

        for payload in backend.pop_control_messages():
            if "event_id" in payload:  # CAM_DETECT payload
                task = task_from_detection_payload(payload, config.roi_aging_lambda, config.priority_k, now)
                backend.broadcast(robot.start_auction(task, now).to_bytes())
                if DEBUG:
                    print(f"  t={now:6.1f}s {robot_id} hosts auction for {task.task_id} ({task.task_type.value})")
            elif payload.get("ctl") == CTL_ACTIVE:
                active_tasks.add(payload["task_id"])
            elif payload.get("ctl") == CTL_COORD_TIMEOUT:
                task_id = payload["task_id"]
                active_tasks.discard(task_id)
                robot.abandon_task(task_id)
                if task_id in robot.owned_tasks:
                    backend.broadcast(robot.start_auction(robot.owned_tasks[task_id], now).to_bytes())

        incoming = backend.receive()
        for raw in incoming:
            message = Message.from_bytes(raw)
            if message.kind is MessageType.DONE:
                task_id = str(message.fields["task_id"])
                active_tasks.discard(task_id)
                if task_id in robot.assignments:
                    robot.complete_task(task_id)
                    if DEBUG:
                        print(f"  t={now:6.1f}s {robot_id} completed {task_id}")
        for message in robot.receive(incoming, now):
            backend.broadcast(message.to_bytes())

        outgoing, _awards, _failed = robot.process_auctions(now)
        for message in outgoing:
            backend.broadcast(message.to_bytes())

        for message in robot.advance(now, config.dt, active_tasks, explore_target):
            backend.broadcast(message.to_bytes())

        # Idle and reached the current waypoint - pick a new one so the
        # robot keeps covering the field instead of parking there. Only
        # relevant while unassigned; robot.advance() ignores explore_target
        # entirely once current_task_id is set.
        if robot.current_task_id is None and dist(robot.position, explore_target) <= config.arrival_tolerance:
            explore_target = _random_explore_target(explore_rng, config)
            if DEBUG:
                print(f"  t={now:6.1f}s {robot_id} exploring -> ({explore_target[0]:+.3f},{explore_target[1]:+.3f})")

        if DEBUG and robot.assignments != last_assignments:
            last_assignments = list(robot.assignments)
            print(
                f"  t={now:6.1f}s [assign] {robot_id} state={robot.state.value} "
                f"current_task={robot.current_task_id} assignments={robot.assignments} "
                f"pos=({robot.position[0]:+.3f},{robot.position[1]:+.3f})"
            )

        if now >= next_state_log:
            next_state_log = now + config.state_log_interval
            logger.state(now, robot.robot_id, robot.position[0], robot.position[1], robot.battery, robot.workload, robot.state.value)

        elapsed = time.monotonic() - loop_start
        time.sleep(max(0.0, dt - elapsed))


if __name__ == "__main__":
    # Was silently swallowing every non-KeyboardInterrupt exit path too -
    # three real pilot runs have now ended with just "wrote N state rows"
    # and no visible cause, and there's no way to tell from that alone
    # whether it was Ctrl-C, a real crash whose traceback scrolled off the
    # terminal, or something else. Make every exit path unambiguous and
    # -crucially- durable across a truncated copy-paste: real crashes now
    # also land in a file, not just stdout.
    try:
        main()
    except KeyboardInterrupt:
        print("\n  Interrupted via Ctrl-C.")
    except BaseException:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        _robot_id = sys.argv[1] if len(sys.argv) > 1 else "unknown"
        _crash_path = LOG_DIR / f"crash_{_robot_id}.log"
        _crash_path.write_text(traceback.format_exc(), encoding="utf-8")
        print(f"\n  CRASHED - full traceback saved to {_crash_path}")
        raise
