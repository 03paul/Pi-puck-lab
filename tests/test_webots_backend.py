"""Tests the device-agnostic logic in backends/webots.py against fake Webots devices.

Real Webots devices (Node, Motor, Emitter, Receiver) aren't available outside the simulator,
so these fakes implement only the exact surface WebotsBackend calls - not a claim that Webots'
real physics behaves like this, only that the message-routing and phase-transition logic here
is self-consistent and does what it says.
"""

from __future__ import annotations

import json
import math
import unittest

from allocation.messages import announce_message
from allocation.tasks import PriorityFn, Task, TaskType
from backends.webots import (
    CTL_ACTIVE,
    CTL_COORD_TIMEOUT,
    CTL_DETECT_ROI,
    CTL_HOST_SURVEY,
    HEADING_OFFSET,
    WebotsBackend,
    find_node_by_name,
    task_from_detection_payload,
    task_from_host_survey_payload,
)


class FakeMotor:
    def __init__(self) -> None:
        self.velocity = 0.0

    def setVelocity(self, value: float) -> None:  # noqa: N802 - matches Webots API naming
        self.velocity = value


class FakeNode:
    """Reports a fixed pose; HEADING_OFFSET-compensated so heading==true_heading_rad."""

    def __init__(self, x: float, y: float, true_heading_rad: float) -> None:
        self.x, self.y = x, y
        # WebotsBackend computes atan2(matrix[3], matrix[0]) + HEADING_OFFSET as heading, so
        # encode a matrix whose atan2(matrix[3], matrix[0]) == true_heading_rad - HEADING_OFFSET
        # (imported live from backends.webots, not hardcoded - stays correct however that
        # constant gets recalibrated, see its own comment for why it moved from -pi/2 to 0).
        raw = true_heading_rad - HEADING_OFFSET
        self._matrix = [math.cos(raw), 0, 0, math.sin(raw), 0, 0, 0, 0, 1]

    def getPosition(self) -> tuple[float, float, float]:  # noqa: N802
        return (self.x, self.y, 0.0)

    def getOrientation(self) -> list[float]:  # noqa: N802
        return self._matrix


class FakeReceiver:
    def __init__(self) -> None:
        self.queue: list[bytes] = []

    def push(self, payload: bytes) -> None:
        self.queue.append(payload)

    def getQueueLength(self) -> int:  # noqa: N802
        return len(self.queue)

    def getBytes(self) -> bytes:  # noqa: N802
        return self.queue[0]

    def nextPacket(self) -> None:  # noqa: N802
        self.queue.pop(0)


class FakeEmitter:
    def __init__(self) -> None:
        self.sent: list[bytes] = []

    def send(self, payload: bytes) -> None:
        self.sent.append(payload)


class FakeClock:
    """A settable clock, so tests can fast-forward time without real delays."""

    def __init__(self, t: float = 0.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t


def make_backend(x: float = 0.0, y: float = 0.0, heading: float = 0.0, robot_id: str = "r00", clock=None) -> WebotsBackend:
    return WebotsBackend(
        robot_id=robot_id,
        node=FakeNode(x, y, heading),
        left_motor=FakeMotor(),
        right_motor=FakeMotor(),
        emitter=FakeEmitter(),
        receiver=FakeReceiver(),
        clock=clock if clock is not None else (lambda: 0.0),
    )


class WebotsBackendTests(unittest.TestCase):
    def test_get_pose_matches_encoded_heading(self) -> None:
        backend = make_backend(0.3, -0.1, heading=math.radians(37))
        x, y, heading = backend.get_pose()
        self.assertAlmostEqual(x, 0.3)
        self.assertAlmostEqual(y, -0.1)
        self.assertAlmostEqual(math.degrees(heading), 37, places=5)

    def test_is_at_target_true_when_no_target_set(self) -> None:
        backend = make_backend()
        self.assertTrue(backend.is_at_target())

    def test_drive_to_sets_rotate_phase_and_tracks_target(self) -> None:
        backend = make_backend()
        backend.drive_to(1.0, 1.0)
        self.assertEqual(backend.target, (1.0, 1.0))
        self.assertFalse(backend.is_at_target())

    def test_protocol_message_bytes_are_routed_to_receive(self) -> None:
        backend = make_backend()
        task = Task("roi-000", TaskType.GUARD, (0.5, 0.5), 1, PriorityFn(0.6, 0.01, 1.0, 0.0), 15.0, 0.0)
        wire = announce_message(task, deadline=3.0, sender="r01", now=0.0).to_bytes()
        backend.receiver.push(wire)

        received = backend.receive()

        self.assertEqual(received, [wire])
        self.assertEqual(backend.detect_rois(), [])
        self.assertEqual(backend.pop_control_messages(), [])

    def test_detect_roi_only_delivered_to_addressed_robot(self) -> None:
        backend = make_backend(robot_id="r00")
        payload_for_me = json.dumps(
            {"ctl": CTL_DETECT_ROI, "to": "r00", "event_id": "e1", "pos": [0.1, 0.2], "task_type": "guard"}
        ).encode("utf-8")
        payload_for_other = json.dumps(
            {"ctl": CTL_DETECT_ROI, "to": "r05", "event_id": "e2", "pos": [0.3, 0.4], "task_type": "push"}
        ).encode("utf-8")
        backend.receiver.push(payload_for_me)
        backend.receiver.push(payload_for_other)

        detections = backend.detect_rois()

        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].detection_id, "e1")
        self.assertEqual(detections[0].position, (0.1, 0.2))

    def test_active_and_timeout_control_messages_reach_every_robot(self) -> None:
        backend = make_backend(robot_id="r00")
        active = json.dumps({"ctl": CTL_ACTIVE, "task_id": "roi-000"}).encode()
        timeout = json.dumps({"ctl": CTL_COORD_TIMEOUT, "task_id": "roi-000"}).encode()
        backend.receiver.push(active)
        backend.receiver.push(timeout)

        control = backend.pop_control_messages()

        self.assertEqual([item["ctl"] for item in control], [CTL_ACTIVE, CTL_COORD_TIMEOUT])

    def test_host_survey_only_delivered_to_addressed_robot(self) -> None:
        backend = make_backend(robot_id="r00")
        mine = json.dumps({"ctl": CTL_HOST_SURVEY, "to": "r00", "cell_index": 3, "pos": [1.0, 1.0]}).encode()
        others = json.dumps({"ctl": CTL_HOST_SURVEY, "to": "r05", "cell_index": 4, "pos": [0.0, 0.0]}).encode()
        backend.receiver.push(mine)
        backend.receiver.push(others)

        control = backend.pop_control_messages()

        self.assertEqual(len(control), 1)
        self.assertEqual(control[0]["cell_index"], 3)

    def test_malformed_and_unknown_payloads_are_ignored_not_raised(self) -> None:
        backend = make_backend()
        backend.receiver.push(b"not json at all")
        backend.receiver.push(json.dumps({"ctl": "SOMETHING_FUTURE"}).encode())
        backend.receiver.push(json.dumps(["not", "a", "dict"]).encode())

        self.assertEqual(backend.receive(), [])
        self.assertEqual(backend.detect_rois(), [])
        self.assertEqual(backend.pop_control_messages(), [])

    def test_tick_stops_motors_once_arrived(self) -> None:
        backend = make_backend(1.0, 1.0)
        backend.drive_to(1.0, 1.0)  # already there
        backend.tick(dt=0.064)
        self.assertEqual(backend.left_motor.velocity, 0.0)
        self.assertEqual(backend.right_motor.velocity, 0.0)

    def test_tick_rotates_in_place_when_far_off_heading(self) -> None:
        backend = make_backend(0.0, 0.0, heading=0.0)
        backend.drive_to(0.0, 1.0)  # target is 90 degrees to the left of current heading
        backend.tick(dt=0.064)
        # rotating in place: equal magnitude, opposite sign
        self.assertGreater(abs(backend.left_motor.velocity), 0.0)
        self.assertAlmostEqual(backend.left_motor.velocity, -backend.right_motor.velocity, places=6)

    def test_battery_drains_with_idle_time_even_when_not_moving(self) -> None:
        backend = make_backend(1.0, 1.0)
        backend.drive_to(1.0, 1.0)
        start_battery = backend.battery
        backend.tick(dt=1.0)
        self.assertLess(backend.battery, start_battery)

    def test_stuck_robot_backs_off_instead_of_cruising_forever(self) -> None:
        # Node position never changes no matter what velocity is commanded - simulates being
        # physically jammed (e.g. against a wall), which is exactly the case tick() must detect.
        clock = FakeClock(0.0)
        backend = make_backend(0.0, 0.0, heading=0.0, clock=clock)
        backend.drive_to(1.0, 0.0)  # straight ahead - aligned immediately, goes to CRUISE

        backend.tick(dt=0.064)  # ROTATE -> CRUISE (already aligned)
        self.assertEqual(backend._phase, "cruise")

        clock.t = 6.0  # past STUCK_CHECK_INTERVAL_S with zero real progress
        backend.tick(dt=0.064)

        self.assertEqual(backend._phase, "backoff")
        self.assertLess(backend.left_motor.velocity, 0.0)
        self.assertLess(backend.right_motor.velocity, 0.0)

    def test_backoff_returns_to_rotate_after_its_duration(self) -> None:
        clock = FakeClock(0.0)
        backend = make_backend(0.0, 0.0, heading=0.0, clock=clock)
        backend.drive_to(1.0, 0.0)
        backend.tick(dt=0.064)
        clock.t = 6.0
        backend.tick(dt=0.064)
        self.assertEqual(backend._phase, "backoff")

        clock.t = 8.0  # past BACKOFF_DURATION_S
        backend.tick(dt=0.064)

        # Exits backoff into rotate-then-possibly-straight-to-cruise (this fake node's heading
        # happens to already point at the target, so it can pass through rotate in zero time -
        # the real assertion is just "no longer stuck reversing").
        self.assertNotEqual(backend._phase, "backoff")


class FakeSFStringField:
    def __init__(self, value: str) -> None:
        self._value = value

    def getSFString(self) -> str:  # noqa: N802
        return self._value


class FakeMFNodeField:
    def __init__(self, nodes: list["FakeSceneNode"]) -> None:
        self._nodes = nodes

    def getCount(self) -> int:  # noqa: N802
        return len(self._nodes)

    def getMFNode(self, index: int) -> "FakeSceneNode":  # noqa: N802
        return self._nodes[index]


class FakeSceneNode:
    """Stands in for a Webots Node: some nodes have a "name" field, some have "children"."""

    def __init__(self, name: str | None = None, children: list["FakeSceneNode"] | None = None) -> None:
        self._name = name
        self._children = children or []

    def getField(self, field_name: str):  # noqa: N802
        if field_name == "name" and self._name is not None:
            return FakeSFStringField(self._name)
        if field_name == "children":
            return FakeMFNodeField(self._children)
        return None


class FindNodeByNameTests(unittest.TestCase):
    def test_finds_a_direct_child_by_name(self) -> None:
        r00 = FakeSceneNode(name="r00")
        r01 = FakeSceneNode(name="r01")
        root = FakeSceneNode(children=[FakeSceneNode(), r00, r01])  # WorldInfo-like node has no name

        self.assertIs(find_node_by_name(root, "r01"), r01)

    def test_descends_into_nested_children_fields(self) -> None:
        r00 = FakeSceneNode(name="r00")
        group = FakeSceneNode(children=[FakeSceneNode(), r00])
        root = FakeSceneNode(children=[group])

        self.assertIs(find_node_by_name(root, "r00"), r00)

    def test_returns_none_when_not_found(self) -> None:
        root = FakeSceneNode(children=[FakeSceneNode(name="r00")])

        self.assertIsNone(find_node_by_name(root, "r99"))

    def test_returns_none_for_root_without_children_field(self) -> None:
        self.assertIsNone(find_node_by_name(None, "r00"))


class TaskFromPayloadTests(unittest.TestCase):
    def test_detection_payload_builds_a_valid_roi_task(self) -> None:
        payload = {
            "event_id": "event-004",
            "task_type": "guard",
            "pos": [0.5, 0.7],
            "n_req": 1,
            "duration": 15.0,
            "p0": 0.6,
        }
        task = task_from_detection_payload(payload, roi_aging_lambda=0.01, priority_k=1.0, now=12.5)

        self.assertEqual(task.task_id, "roi-004")
        self.assertEqual(task.task_type, TaskType.GUARD)
        self.assertEqual(task.position, (0.5, 0.7))
        self.assertEqual(task.n_req, 1)
        self.assertEqual(task.duration, 15.0)
        self.assertEqual(task.announced_at, 12.5)
        self.assertAlmostEqual(task.priority_fn.at(12.5), 0.6)  # p0 at t_ref

    def test_host_survey_payload_builds_a_valid_survey_task(self) -> None:
        payload = {"task_id": "survey-03-0001", "pos": [1.2, 0.4], "last_seen": 40.0}
        task = task_from_host_survey_payload(payload, lambda_model=0.00693, priority_k=1.0, now=90.0)

        self.assertEqual(task.task_id, "survey-03-0001")
        self.assertEqual(task.task_type, TaskType.SURVEY)
        self.assertEqual(task.n_req, 1)
        self.assertEqual(task.duration, 2.0)
        self.assertFalse(task.counts_toward_workload)
        # priority should have aged since last_seen=40 was 50s before now=90
        self.assertGreater(task.priority_fn.at(90.0), 0.0)


if __name__ == "__main__":
    unittest.main()
