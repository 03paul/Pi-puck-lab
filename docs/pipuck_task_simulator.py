"""Task-detection simulator for the Tier-2 pilot: plays the cross-robot
bookkeeping half of the role docs/webots_market_supervisor_controller.py
plays in Webots ("localisation may be centralised, decision-making may
not") - but NOT the localisation half. The lab already runs an overhead
tracking system (RCPS dashboard) that publishes every robot's pose to the
MQTT topic `robot_pos/all`; this process only subscribes to that (never
runs a camera itself) and does two things with it:

1. Simulates on-board RoI detection: exactly as instructed - "über die
   Kamera simulieren wir, dass ein neuer Task über die robotereigenen
   Sensoren erfasst wurde, wenn dieser in die Range des Objekts kommt" -
   once a robot's tracked position comes within DETECTION_RADIUS of the
   one pilot task's known position, this process publishes a CAM_DETECT
   payload to that robot only, exactly as if its own on-board sensor had
   triggered.
2. Decides when the required number of winners have simultaneously
   arrived (n_req=1 for this pilot's single GUARD task) - by passively
   overhearing the fleet's own AWARD broadcasts on WIRE_TOPIC, the same
   way docs/webots_market_supervisor_controller.py does. It never decides
   *who* wins an auction - that stays fully decentralised in each robot's
   own process.

STATUS: written ahead of the first hardware pilot session, not yet run
against the lab's real MQTT broker - see docs/LAB_PILOT_CHECKLIST.md.

Scope, deliberately minimal per the project proposal's own stated pilot
goal: exactly ONE GUARD task (n_req=1), SURVEY disabled, PUSH excluded,
battery virtual. This is NOT the three-strategy comparison Tier 0/1
already ran - it exists to prove the wire protocol and movement stack
work on real hardware at all.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import _py37_compat  # noqa: F401 - must run before any allocation/sim/backends import, see its docstring

# Auto-detected - see docs/pipuck_market_robot_controller.py's REPO_ROOT
# comment for why this works here but not for Webots controllers.
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from allocation.messages import Message, MessageType, done_message
from backends.pipuck import (
    CAM_DETECT,
    CTL_ACTIVE,
    CTL_COORD_TIMEOUT,
    MQTT_BROKER,
    MQTT_PORT,
    PILOT_TASK_TRACKING_ID,
    POS_TOPIC,
    WIRE_TOPIC,
    _parse_robot_pos_payload,
    parse_tracked_positions,
)
from sim.config import SimulationConfig

CONFIG_PATH = REPO_ROOT / "parameters" / "default.json"
GUARD_DURATION_S = 15.0  # matches Tier 0/1's GUARD duration (parameters/default.json)
DETECTION_RADIUS = 0.3  # m -- matches Tier 0/1's config.detection_radius
ASSIGNMENT_TIMEOUT_S = 240.0  # generous first-run value, same rationale as Tier 1's override
TASK_ID = "roi-pilot-000"
EVENT_ID = "event-pilot-000"


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def main() -> None:
    import paho.mqtt.client as mqtt

    config = SimulationConfig.from_json(CONFIG_PATH)

    state = {
        "detected": False,
        "task_position": None,  # tracked live from PILOT_TASK_TRACKING_ID until detected, then frozen
        "winners": None,
        "award_at": None,
        "active_at": None,
        "poses": {},
        "start": time.monotonic(),
    }

    def now() -> float:
        return time.monotonic() - state["start"]

    def publish_wire(client, payload: dict) -> None:
        client.publish(WIRE_TOPIC, json.dumps(payload).encode("utf-8"))

    def on_connect(client, _userdata, _flags, rc) -> None:
        print(f"Connected with result code {rc}")
        client.subscribe(POS_TOPIC)
        client.subscribe(WIRE_TOPIC)

    def on_message(client, _userdata, msg) -> None:
        if msg.topic == POS_TOPIC:
            try:
                state["poses"] = _parse_robot_pos_payload(msg.payload)  # `robot_pos/all` carries every robot each message
                tracked = parse_tracked_positions(msg.payload)  # includes non-robot ids too, e.g. the task object
            except (json.JSONDecodeError, UnicodeDecodeError):
                return
            if not state["detected"]:
                # Keep following the object's live position (it may be placed
                # or nudged before detection) - freeze it the instant a robot
                # actually detects it, matching Task.position's immutability
                # everywhere else in this project (see allocation/tasks.py).
                object_pose = tracked.get(PILOT_TASK_TRACKING_ID)
                if object_pose is not None:
                    state["task_position"] = (object_pose[0], object_pose[1])
            _maybe_detect(client)
            _maybe_check_arrival(client)
            return

        # WIRE_TOPIC: passively overhear AWARD/DONE, exactly as
        # docs/webots_market_supervisor_controller.py does.
        try:
            message = Message.from_bytes(msg.payload)
        except (ValueError, KeyError, UnicodeDecodeError, json.JSONDecodeError):
            return
        if message.kind is MessageType.AWARD and str(message.fields["task_id"]) == TASK_ID:
            state["winners"] = [str(w) for w in message.fields["winners"]]
            state["award_at"] = now()
            print(f"AWARD winners={state['winners']}")
        elif message.kind is MessageType.DONE and str(message.fields["task_id"]) == TASK_ID:
            print("DONE observed - pilot task completed end-to-end.")
            client.disconnect()

    def _maybe_detect(client) -> None:
        task_position = state["task_position"]
        if state["detected"] or task_position is None:
            return  # object not yet seen by the tracker at all, or already handled
        candidates = [rid for rid, xy in state["poses"].items() if _dist(xy[:2], task_position) <= DETECTION_RADIUS]
        if not candidates:
            return
        detector_id = min(candidates, key=lambda rid: _dist(state["poses"][rid][:2], task_position))
        state["detected"] = True
        publish_wire(
            client,
            {
                "cam": CAM_DETECT,
                "to": detector_id,
                "event_id": EVENT_ID,
                "task_type": "guard",
                "pos": list(task_position),
                "n_req": 1,
                "duration": GUARD_DURATION_S,
                "p0": config.priority_guard,
            },
        )
        print(f"DETECT -> {detector_id} (object at {task_position})")

    def _maybe_check_arrival(client) -> None:
        winners = state["winners"]
        task_position = state["task_position"]
        if winners is None or state["active_at"] is not None or task_position is None:
            return
        award_at = state["award_at"]
        if award_at is not None and now() - award_at > ASSIGNMENT_TIMEOUT_S:
            publish_wire(client, {"ctl": CTL_COORD_TIMEOUT, "task_id": TASK_ID})
            print("COORD_TIMEOUT - winner never arrived within timeout")
            state["winners"], state["award_at"] = None, None
            return
        poses = state["poses"]
        ready = all(w in poses and _dist(poses[w][:2], task_position) <= config.arrival_tolerance for w in winners)
        if ready:
            state["active_at"] = now()
            publish_wire(client, {"ctl": CTL_ACTIVE, "task_id": TASK_ID})
            print(f"ACTIVE winners={winners} - waiting {GUARD_DURATION_S}s")

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)

    print("Task simulator running. Waiting for a robot to approach the pilot task...")
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
