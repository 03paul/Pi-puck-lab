"""Webots controller template: one fleet robot running the unmodified market/greedy/random logic.

STATUS: untested starting point, not a verified controller - same caveat as
docs/webots_calibration_controller.py, and this one is substantially larger. This repository has
no Webots installation; nothing below has run inside the simulator.

Reuses, completely unmodified: allocation/ (bidding, auction, messages, strategies, tasks) and
sim.robot.Robot (via the backend hook added to sim/robot.py for this stage - Robot.advance()
runs exactly as it does in Tier 0, just delegating movement/battery to WebotsBackend instead of
point-kinematics). Only NEW code is the plumbing: reading detection/survey/explore-target/
active/timeout notices from docs/webots_market_supervisor_controller.py (the "overhead camera"
role) and translating them into the same allocation.Robot calls sim/world.py already makes.

Setup:
  1. Wizards > New > Robot Controller (Python)..., name exactly `webots_market_robot_controller`.
  2. Assign it to EVERY fleet robot's `controller` field (Webots runs one process per robot,
     all from this same source file - that's supported and expected).
  3. Name each robot node exactly "r00", "r01", ... "r{N-1}" (matches sim/world.py's robot_ids)
     so supervisor.getName() alone identifies which Tier-0 robot this process plays.
  4. Each robot needs: `supervisor TRUE` (self-pose reads only, see backends/webots.py's module
     docstring on why this isn't remote-control of other robots), left/right wheel motors, and
     Emitter + Receiver devices both named "emitter"/"receiver" on the SAME channel as every
     other fleet robot AND the supervisor controller.
  5. FLEET_SIZE below must equal parameters/default.json's robot_count AND the number of robot
     nodes actually placed in the world - all three out of sync is a silent-corruption risk
     (wrong sector waypoints), not a crash, so double check it.
"""

from __future__ import annotations

import csv
import json
import sys
from dataclasses import replace
from pathlib import Path
from random import Random

from controller import Supervisor  # type: ignore[import-not-found]

# EDIT ME: absolute path to the decentralized-mrta repo checkout on this machine. Can't be
# derived from __file__ - this file gets copy-pasted into Webots' own
# controllers/<name>/<name>.py, so __file__ points into the Webots project, not the repo.
# Needed both to import allocation/sim/backends/metrics (in case `pip install -e .` into
# Webots' own Python didn't take - see docs/WEBOTS_MARKET_DEMO.md) and to locate
# parameters/default.json and results/best_parameters.json below.
REPO_ROOT = Path(r"/Users/03paul_/Desktop/Studium/Praktikum Verteilte Robotiksysteme/decentralized-mrta")
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from allocation.bidding import BidWeights
from allocation.messages import Message, MessageType
from allocation.strategies import Strategy, StrategyName
from backends.webots import (
    ARRIVAL_TOLERANCE,
    CTL_ACTIVE,
    CTL_COORD_TIMEOUT,
    CTL_DETECT_ROI,
    CTL_EXPLORE_TARGET,
    CTL_HOST_SURVEY,
    WebotsBackend,
    task_from_detection_payload,
    task_from_host_survey_payload,
)
from metrics.logger import STATE_FIELDS, RunLogger
from sim.config import SimulationConfig
from sim.robot import Robot

TIME_STEP = 64  # ms
FLEET_SIZE = 7  # must match parameters/default.json's robot_count and the placed robot count

CONFIG_PATH = REPO_ROOT / "parameters" / "default.json"
BEST_WEIGHTS_PATH = REPO_ROOT / "results" / "best_parameters.json"
DEBUG = True  # console prints for every auction-lifecycle event; set False once runs are reliable

# Which allocation.strategies.StrategyName to run. Swap this (and re-place robots) to compare
# Market against the same baselines used in results/comparison_summary.md - the point of this
# stage is to check whether the Tier-0 ranking survives real Webots dynamics, not to run only
# Market.
STRATEGY_NAME = StrategyName.RANDOM

# Per-strategy subfolder, not a flat "webots_logs" - every run used to write the exact same
# filenames, so switching STRATEGY_NAME and re-running silently overwrote the previous strategy's
# logs with no warning (lost an entire Market run this way). Keyed off STRATEGY_NAME so Market/
# Greedy/Random runs coexist on disk automatically - nothing to remember to rename by hand.
LOG_DIR = Path("webots_logs") / STRATEGY_NAME.value  # relative to this controller's Webots working dir

# Tier-1-only calibration override, same spirit and same disclosure as
# docs/webots_market_supervisor_controller.py's ASSIGNMENT_TIMEOUT_OVERRIDE_S - NOT a change to
# allocation/ (preemption_cooldown is read only in sim/robot.py's Robot._on_announce(), never in
# allocation/), and parameters/default.json's canonical 15.0 (Tier-0 comparison_summary.md) is
# left untouched on disk. Diagnosis from a live run: with 0 ACTIVE events in a 300s run across
# every task type, the root cause turned out to be preemption, not speed or timeouts - a robot
# mid-NAVIGATE toward task A can have that assignment entirely discarded (RELEASE
# reason="preempted", the whole in-progress trip thrown away) for a higher-priority task B every
# time 15s elapses. At Tier 0's near-instantaneous abstract movement that's cheap; at Webots'
# real ~35-45s single-leg diagonal travel time it means a robot gets redirected 2-3x before ever
# arriving anywhere, near-guaranteeing nothing ever completes. Raised 6x to give a robot real
# odds of finishing a trip before being redirected again - preemption itself stays on (still
# core to the market design), just throttled to match Tier 1's much slower real clock.
PREEMPTION_COOLDOWN_OVERRIDE_S = 90.0


def _load_strategy() -> Strategy:
    weights = json.loads(BEST_WEIGHTS_PATH.read_text(encoding="utf-8"))["market_weights"]
    return Strategy(STRATEGY_NAME, BidWeights(**weights))


def main() -> None:
    supervisor = Supervisor()
    robot_id = supervisor.getName()
    config = SimulationConfig.from_json(CONFIG_PATH)
    # frozen dataclass - replace() rather than attribute assignment; see the constants' comments above
    # and backends.webots.ARRIVAL_TOLERANCE's comment for arrival_tolerance
    config = replace(config, preemption_cooldown=PREEMPTION_COOLDOWN_OVERRIDE_S, arrival_tolerance=ARRIVAL_TOLERANCE)
    strategy = _load_strategy()
    # Diagnostic: a run produced byte-identical logs to a RANDOM run while the file on disk said
    # GREEDY at the time - printing the resolved value directly at startup is the only way to
    # confirm what a given process is actually using, rather than inferring it indirectly from
    # bid costs after the fact. Remove once the mismatch is understood and no longer reproduces.
    print(f"  {robot_id}: STRATEGY_NAME (from file)={STRATEGY_NAME!r}  strategy.name (resolved)={strategy.name!r}  module={__file__}")

    left_motor = supervisor.getDevice("left wheel motor")
    right_motor = supervisor.getDevice("right wheel motor")
    for motor in (left_motor, right_motor):
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
    emitter = supervisor.getDevice("emitter")
    receiver = supervisor.getDevice("receiver")
    receiver.enable(TIME_STEP)

    backend = WebotsBackend(
        robot_id=robot_id,
        node=supervisor.getSelf(),
        left_motor=left_motor,
        right_motor=right_motor,
        emitter=emitter,
        receiver=receiver,
        clock=supervisor.getTime,
        battery_idle_drain=config.battery_idle_drain,
        battery_move_drain_per_m=config.battery_move_drain_per_m,
        battery=config.initial_battery_max,
    )

    # Deterministic per-robot RNG seed, same construction as sim/world.py's _make_robots so a
    # given robot index behaves identically to its Tier-0 counterpart wherever randomness
    # (e.g. RANDOM strategy's cost()) is involved.
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

    explore_target = robot.position  # until the supervisor sends a real one, hold position
    seen_event_ids: set[str] = set()
    active_tasks: set[str] = set()
    logger = RunLogger()
    next_state_log = 0.0
    last_assignments: list[str] = []  # diagnostic: print only when this actually changes

    while supervisor.step(TIME_STEP) != -1:
        now = supervisor.getTime()
        backend.tick(TIME_STEP / 1000.0)

        for payload in backend.pop_control_messages():
            ctl = payload["ctl"]
            if ctl == CTL_DETECT_ROI:
                if payload["event_id"] in seen_event_ids:
                    continue
                seen_event_ids.add(payload["event_id"])
                task = task_from_detection_payload(payload, config.roi_aging_lambda, config.priority_k, now)
                emitter.send(robot.start_auction(task, now).to_bytes())
                if DEBUG:
                    print(f"  t={now:6.1f}s {robot_id} hosts auction for {task.task_id} ({task.task_type.value})")
            elif ctl == CTL_HOST_SURVEY:
                task = task_from_host_survey_payload(payload, config.lambda_model, config.priority_k, now)
                emitter.send(robot.start_auction(task, now).to_bytes())
                if DEBUG:
                    print(f"  t={now:6.1f}s {robot_id} hosts survey auction for {task.task_id}")
            elif ctl == CTL_EXPLORE_TARGET:
                explore_target = tuple(payload["pos"])
            elif ctl == CTL_ACTIVE:
                active_tasks.add(payload["task_id"])
            elif ctl == CTL_COORD_TIMEOUT:
                task_id = payload["task_id"]
                active_tasks.discard(task_id)
                robot.abandon_task(task_id)
                if task_id in robot.owned_tasks:
                    emitter.send(robot.start_auction(robot.owned_tasks[task_id], now).to_bytes())

        incoming = backend.receive()
        for raw in incoming:
            message = Message.from_bytes(raw)
            if message.kind is MessageType.DONE:
                task_id = str(message.fields["task_id"])
                active_tasks.discard(task_id)
                # Mirrors sim/world.py's World._update_execution(): the world calls
                # complete_task() directly on each winner's Robot object (bookkeeping the
                # completed-task counters used for load_imbalance), then the DONE broadcast
                # separately lets everyone else clean up owned/hosted state via receive()
                # below. Same split here, just triggered by an overheard broadcast instead of
                # direct object access.
                if task_id in robot.assignments:
                    robot.complete_task(task_id)
                    if DEBUG:
                        print(f"  t={now:6.1f}s {robot_id} completed {task_id}")
        for message in robot.receive(incoming, now):
            emitter.send(message.to_bytes())

        outgoing, _awards, _failed = robot.process_auctions(now)
        for message in outgoing:
            emitter.send(message.to_bytes())

        for message in robot.advance(now, config.dt, active_tasks, explore_target):
            emitter.send(message.to_bytes())

        if DEBUG and robot.assignments != last_assignments:
            last_assignments = list(robot.assignments)
            print(
                f"  t={now:6.1f}s [assign] {robot_id} state={robot.state.value} "
                f"current_task={robot.current_task_id} assignments={robot.assignments} "
                f"workload={robot.workload} pos=({robot.position[0]:+.3f},{robot.position[1]:+.3f})"
            )

        if now >= next_state_log:
            next_state_log = now + config.state_log_interval
            logger.state(now, robot.robot_id, robot.position[0], robot.position[1], robot.battery, robot.workload, robot.state.value)

        if now >= config.duration:
            break

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    # One state file per robot process (can't share a single writer across processes) - see
    # docs/WEBOTS_MARKET_DEMO.md for how these get merged with the supervisor's events.csv
    # before running metrics/analysis.py.
    with (LOG_DIR / f"state_{robot_id}.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, STATE_FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(logger.states)
    print(f"{robot_id}: wrote {len(logger.states)} state rows")


if __name__ == "__main__":
    main()
