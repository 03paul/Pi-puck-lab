"""Tests the device-agnostic logic in backends/pipuck.py against a fake MQTT
transport and a fake motor driver - same rationale as
tests/test_webots_backend.py: no real hardware, network, or MQTT broker
available here, so these fakes implement only the exact surface
PiPuckBackend calls, checking that wire-routing and the movement state
machine are self-consistent before the first live pilot.
"""

from __future__ import annotations

import json
import math
import unittest

from allocation.tasks import TaskType
from backends.pipuck import (
    CAM_DETECT,
    CTL_ACTIVE,
    CTL_COORD_TIMEOUT,
    PILOT_TASK_TRACKING_ID,
    POS_TOPIC,
    WIRE_TOPIC,
    PiPuckBackend,
    parse_tracked_positions,
    task_from_detection_payload,
)


class FakeMotorDriver:
    def __init__(self) -> None:
        self.left = 0.0
        self.right = 0.0
        self.calls: list[tuple[float, float]] = []

    def set_velocity(self, left: float, right: float) -> None:
        self.left, self.right = left, right
        self.calls.append((left, right))


class FakeTransport:
    """A queue-backed stand-in for _MqttTransport: push_pos()/push_wire()
    simulate a message having arrived on the respective topic; publish() is
    captured, not actually sent to a broker."""

    def __init__(self) -> None:
        self.queue: list[tuple[str, bytes]] = []
        self.published: list[bytes] = []

    def push_pos(self, payload: bytes) -> None:
        self.queue.append((POS_TOPIC, payload))

    def push_wire(self, payload: bytes) -> None:
        self.queue.append((WIRE_TOPIC, payload))

    def publish(self, payload: bytes) -> None:
        self.published.append(payload)

    def drain(self) -> list[tuple[str, bytes]]:
        items, self.queue = self.queue, []
        return items


class FakeClock:
    def __init__(self, t: float = 0.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t


def _pos_payload(tracking_id: int, x: float, y: float, angle_deg: float) -> bytes:
    return json.dumps({str(tracking_id): {"id": tracking_id, "position": [x, y], "angle": angle_deg}}).encode()


def make_backend(robot_id: str = "r00", clock: FakeClock | None = None) -> tuple[PiPuckBackend, FakeTransport, FakeMotorDriver]:
    transport = FakeTransport()
    motors = FakeMotorDriver()
    backend = PiPuckBackend(
        robot_id=robot_id, motors=motors, transport=transport, clock=clock if clock is not None else (lambda: 0.0)
    )
    return backend, transport, motors


class PiPuckBackendTests(unittest.TestCase):
    def test_pose_before_any_tracking_payload_raises(self) -> None:
        backend, _transport, _motors = make_backend()
        with self.assertRaises(RuntimeError):
            backend.get_pose()

    def test_pose_payload_is_routed_only_to_the_mapped_robot(self) -> None:
        # r00 is mapped to tracking id 22 (see TRACKING_ID_TO_ROBOT_ID)
        backend, transport, _motors = make_backend(robot_id="r00")
        transport.push_pos(_pos_payload(22, 1.0, 1.5, 90.0))  # 90 degrees -> pi/2 rad

        x, y, heading = backend.get_pose()

        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 1.5)
        self.assertAlmostEqual(heading, math.pi / 2)

    def test_pose_payload_for_a_different_robot_is_ignored(self) -> None:
        backend, transport, _motors = make_backend(robot_id="r00")
        transport.push_pos(_pos_payload(32, 0.1, 0.2, 0.0))  # tracking id 32 -> r01, not r00

        with self.assertRaises(RuntimeError):
            backend.get_pose()

    def test_stale_pose_still_returned_but_warns(self) -> None:
        clock = FakeClock(0.0)
        backend, transport, _motors = make_backend(robot_id="r00", clock=clock)
        transport.push_pos(_pos_payload(22, 0.0, 0.0, 0.0))
        backend.get_pose()
        clock.t = 5.0  # well past POSE_STALE_AFTER_S

        x, y, _heading = backend.get_pose()  # should not raise, only warn

        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 0.0)

    def test_detect_only_delivered_to_addressed_robot(self) -> None:
        backend, transport, _motors = make_backend(robot_id="r00")
        mine = json.dumps(
            {"cam": CAM_DETECT, "to": "r00", "event_id": "e1", "pos": [0.1, 0.2], "task_type": "guard"}
        ).encode()
        others = json.dumps(
            {"cam": CAM_DETECT, "to": "r05", "event_id": "e2", "pos": [0.3, 0.4], "task_type": "guard"}
        ).encode()
        transport.push_wire(mine)
        transport.push_wire(others)

        detections = backend.detect_rois()

        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].detection_id, "e1")

    def test_active_and_timeout_reach_every_robot(self) -> None:
        backend, transport, _motors = make_backend()
        transport.push_wire(json.dumps({"ctl": CTL_ACTIVE, "task_id": "roi-000"}).encode())
        transport.push_wire(json.dumps({"ctl": CTL_COORD_TIMEOUT, "task_id": "roi-000"}).encode())

        control = backend.pop_control_messages()

        self.assertEqual([item["ctl"] for item in control], [CTL_ACTIVE, CTL_COORD_TIMEOUT])

    def test_protocol_message_bytes_pass_through_unmodified(self) -> None:
        backend, transport, _motors = make_backend()
        wire = json.dumps({"kind": "ANNOUNCE", "task_id": "roi-000"}).encode()
        transport.push_wire(wire)

        self.assertEqual(backend.receive(), [wire])

    def test_broadcast_publishes_via_transport(self) -> None:
        backend, transport, _motors = make_backend()
        backend.broadcast(b"hello")
        self.assertEqual(transport.published, [b"hello"])

    def test_drive_to_sets_rotate_phase_and_tracks_target(self) -> None:
        backend, transport, _motors = make_backend()
        transport.push_pos(_pos_payload(22, 0.0, 0.0, 0.0))
        backend.drive_to(1.0, 1.0)
        self.assertEqual(backend.target, (1.0, 1.0))
        self.assertFalse(backend.is_at_target())

    def test_tick_rotates_in_place_when_far_off_heading(self) -> None:
        clock = FakeClock(0.0)
        backend, transport, motors = make_backend(robot_id="r00", clock=clock)
        transport.push_pos(_pos_payload(22, 0.0, 0.0, 180.0))  # facing -X
        backend.drive_to(1.0, 0.0)  # target is +X: 180 degrees off

        backend.tick(0.1)

        self.assertEqual(backend._phase, "rotate")
        self.assertNotEqual(motors.left, motors.right)  # turning in place, not driving straight

    def test_tick_cruises_once_aligned(self) -> None:
        clock = FakeClock(0.0)
        backend, transport, motors = make_backend(robot_id="r00", clock=clock)
        transport.push_pos(_pos_payload(22, 0.0, 0.0, 0.0))  # facing +X
        backend.drive_to(1.0, 0.0)  # already aligned

        backend.tick(0.1)

        self.assertEqual(backend._phase, "cruise")
        self.assertGreaterEqual(motors.left, 0.0)
        self.assertAlmostEqual(motors.left, motors.right)

    def test_stuck_robot_backs_off_instead_of_cruising_forever(self) -> None:
        clock = FakeClock(0.0)
        backend, transport, motors = make_backend(robot_id="r00", clock=clock)
        transport.push_pos(_pos_payload(22, 0.0, 0.0, 0.0))
        backend.drive_to(2.0, 0.0)
        backend.tick(0.1)  # ROTATE -> CRUISE
        self.assertEqual(backend._phase, "cruise")

        clock.t = 5.1  # past STUCK_CHECK_INTERVAL_S with (simulated) zero progress
        transport.push_pos(_pos_payload(22, 0.0, 0.0, 0.0))
        backend.tick(0.1)

        self.assertEqual(backend._phase, "backoff")
        self.assertLess(motors.left, 0.0)

    def test_battery_drains_with_time_and_distance(self) -> None:
        backend, _transport, _motors = make_backend()
        backend.battery = 1.0
        backend._drain_battery(1.0, 0.5)
        self.assertLess(backend.battery, 1.0)


class ParseTrackedPositionsTests(unittest.TestCase):
    def test_includes_entries_not_mapped_to_any_robot(self) -> None:
        # PILOT_TASK_TRACKING_ID (the cardboard-cylinder marker) is deliberately
        # NOT in TRACKING_ID_TO_ROBOT_ID - parse_tracked_positions() must still
        # return it (unlike _parse_robot_pos_payload(), which filters it out),
        # since docs/pipuck_task_simulator.py needs the object's live position.
        payload = _pos_payload(PILOT_TASK_TRACKING_ID, 0.7, 0.8, 0.0)

        tracked = parse_tracked_positions(payload)

        self.assertIn(PILOT_TASK_TRACKING_ID, tracked)
        self.assertAlmostEqual(tracked[PILOT_TASK_TRACKING_ID][0], 0.7)
        self.assertAlmostEqual(tracked[PILOT_TASK_TRACKING_ID][1], 0.8)


class TaskFromDetectionPayloadTests(unittest.TestCase):
    def test_builds_a_valid_roi_task(self) -> None:
        payload = {
            "event_id": "event-000",
            "task_type": "guard",
            "pos": [0.5, 0.5],
            "n_req": 1,
            "duration": 15.0,
            "p0": 0.6,
        }
        task = task_from_detection_payload(payload, roi_aging_lambda=0.01, priority_k=1.0, now=0.0)

        self.assertEqual(task.task_id, "roi-000")
        self.assertEqual(task.task_type, TaskType.GUARD)
        self.assertEqual(task.position, (0.5, 0.5))
        self.assertEqual(task.n_req, 1)


if __name__ == "__main__":
    unittest.main()
