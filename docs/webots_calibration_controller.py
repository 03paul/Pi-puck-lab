"""Webots controller template: point-to-point travel-time calibration for Tier 1.

STATUS: untested starting point, not a verified controller. This repository has no Webots
installation, so this file has never actually been run in the simulator. Before use:

  - Verify device names against your actual Pi-Puck/e-puck2 PROTO via `robot.getDeviceList()`
    (Webots' built-in e-puck PROTO uses "left wheel motor"/"right wheel motor"; a Pi-Puck
    extension may differ).
  - Verify WHEEL_RADIUS, AXLE_LENGTH, MAX_WHEEL_SPEED against the real datasheet.
  - The robot's Robot node needs `supervisor TRUE` so this controller can read ground-truth
    pose via `self.getSelf()` — that stands in for the overhead camera from the proposal
    (§2.1), not an on-board GPS.
  - Tune ROTATE_GAIN / CRUISE_SPEED / REALIGN_THRESHOLD_DEG for your PROTO's actual dynamics;
    values below are a reasonable starting point, not calibrated.

Drop this file into <webots_project>/controllers/calibrate_travel_model/calibrate_travel_model.py
(controller directory name must match the file name).

Output: appends one row per run to OUTPUT_CSV as `distance_m,elapsed_s`, in the format
`experiments.calibrate_travel_model` expects.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

from controller import Supervisor  # type: ignore[import-not-found]

TIME_STEP = 64  # ms
WHEEL_RADIUS = 0.0205  # m -- VERIFY against your PROTO
AXLE_LENGTH = 0.053  # m -- VERIFY against your PROTO
MAX_WHEEL_SPEED = 6.28  # rad/s -- VERIFY against your PROTO

ARRIVAL_TOLERANCE = 0.035  # m, must match parameters/default.json

# Rotate-then-cruise, repeated: never command forward and turn at the same time. Three
# combined-feedback designs were tried and all three spiraled despite being theoretically
# stable. A dedicated isolation test (docs/webots_diagnose_straight.py) proved the physics
# itself is NOT the cause: two wheels commanded to an identical, constant 3.0 rad/s drove
# dead straight (heading drift < 0.5 deg over 15s). The one thing that test did differently
# from the failing cruise phase was the speed itself - 3.0 rad/s there vs
# MAX_WHEEL_SPEED*0.8 ~= 5.0 rad/s here, commanded as a sudden jump from a standing stop
# after rotation. That's a much larger sudden acceleration demand, plausibly exceeding the
# wheels' traction and causing slip, which breaks the no-slip-kinematics assumption every
# go-to-goal formula above relied on. Fix: reuse the empirically-validated speed directly
# instead of a fraction of MAX_WHEEL_SPEED, and ramp up to it instead of jumping.
ROTATE_GAIN = 3.0  # rad/s per rad of heading error
ROTATE_TOLERANCE_DEG = 3.0  # rotation phase exits once heading is within this many degrees
# The settle pause (added below) provably changed nothing about the drift itself - this run's
# trajectory is identical to the pre-settle run, just shifted by exactly ROTATE_SETTLE_S. So
# whatever causes the ~3.5-4 deg heading drift during cruise is NOT residual momentum, and
# guessing further root causes stops here. Pragmatic fix instead of another theory: the drift
# plateaus around ~4 deg rather than growing, so it never crossed the old 8 deg realign
# threshold - the robot cruised in one large, gently curved arc for 89s without a single
# correction. Realigning far more often (5 deg instead of 8) forces frequent small corrections
# and keeps the actual path close to straight, regardless of what is physically causing the
# per-segment curve.
ROTATE_SETTLE_S = 0.4
CRUISE_SPEED = 3.0  # rad/s - the exact value proven to drive straight in the isolation test
CRUISE_RAMP_S = 1.0  # seconds to ramp from 0 to CRUISE_SPEED, both wheels together, no jump
REALIGN_THRESHOLD_DEG = 5.0  # cruise segment stops and re-rotates once bearing drifts past this

# If the robot drives in a circle instead of straight to the target, the PROTO's "forward"
# axis is not local +x as assumed below. Try 0, then +-PI_HALF, then PI_FULL until it drives
# straight in the 1-distance test from docs/WEBOTS_CALIBRATION.md step 8 - exactly one of
# these four values is correct for a planar differential-drive robot. See the troubleshooting
# section there for why. A *growing* spiral (not a steady circle) means the heading error's
# sign is inverted, not just offset - try the opposite sign next.
# -pi/2 confirmed empirically working for the built-in Webots e-puck PROTO (R2025a) - re-verify
# if you switch to a different/real Pi-Puck PROTO, the offset is PROTO-specific.
HEADING_OFFSET = -math.pi / 2  # rad

# Prints one telemetry line per drive_to() every this many simulated seconds while debugging.
# Set to 0 once runs are reliable, to keep the calibration console output down to the
# per-run distance/elapsed summary.
DEBUG_PRINT_INTERVAL_S = 1.0

# Safety cutoff so a wrong HEADING_OFFSET/gain fails fast instead of spinning for hours
# unnoticed. At the now-lower CRUISE_SPEED=3.0 rad/s (~0.0615 m/s), the 2.83 m diagonal alone
# takes ~46s; 90s leaves comfortable margin for rotation/realignment overhead on top of that.
MAX_DRIVE_TIME_S = 90.0

# 20 distances spread across the 2x2 m arena, up to its diagonal (~2.83 m). Repeat a couple
# of mid-range values if you have time, to feed the bootstrap CI more information.
DISTANCES_M = [
    0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 1.05, 1.20, 1.35, 1.50,
    1.65, 1.80, 1.95, 2.10, 2.30, 2.50, 2.65, 2.83, 0.90, 1.50,
]

OUTPUT_CSV = Path("calibration_runs.csv")


def reset_pose(supervisor: Supervisor) -> None:
    node = supervisor.getSelf()
    node.getField("translation").setSFVec3f([0.0, 0.0, 0.0])
    node.getField("rotation").setSFRotation([0.0, 0.0, 1.0, 0.0])
    supervisor.simulationResetPhysics()


def _read_heading(node) -> float:
    rotation_matrix = node.getOrientation()
    return math.atan2(rotation_matrix[3], rotation_matrix[0]) + HEADING_OFFSET


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def rotate_to_heading(
    supervisor: Supervisor,
    left_motor,
    right_motor,
    node,
    target_heading: float,
    start_time: float,
) -> None:
    """Phase 1: pure in-place rotation, no forward motion - cannot spiral by construction."""

    while True:
        supervisor.step(TIME_STEP)
        if supervisor.getTime() - start_time > MAX_DRIVE_TIME_S:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            raise RuntimeError(f"rotate_to_heading did not converge within {MAX_DRIVE_TIME_S:.0f}s")
        error = _wrap(target_heading - _read_heading(node))
        if abs(error) <= math.radians(ROTATE_TOLERANCE_DEG):
            break
        turn = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, ROTATE_GAIN * error))
        left_motor.setVelocity(-turn)
        right_motor.setVelocity(turn)

    # Settle: hold both wheels at 0 and just wait, instead of immediately handing off to the
    # cruise phase. Real motors don't stop instantly - this gives any residual spin from the
    # rotation time to actually decay before straight driving starts.
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    settle_start = supervisor.getTime()
    while supervisor.getTime() - settle_start < ROTATE_SETTLE_S:
        supervisor.step(TIME_STEP)
        if supervisor.getTime() - start_time > MAX_DRIVE_TIME_S:
            raise RuntimeError(f"rotate_to_heading did not converge within {MAX_DRIVE_TIME_S:.0f}s")


def drive_to(
    supervisor: Supervisor,
    left_motor,
    right_motor,
    target_x: float,
    target_y: float,
) -> float:
    """Rotate-then-cruise, repeated until arrival. Never commands forward and turn together.

    Returns elapsed simulated seconds, measured from the very first rotation - so the
    returned time includes every realignment, matching what a real point-to-point command
    would take.
    """

    start_time = supervisor.getTime()
    node = supervisor.getSelf()
    last_debug = -DEBUG_PRINT_INTERVAL_S  # force a print on the very first iteration

    while True:
        elapsed = supervisor.getTime() - start_time
        if elapsed > MAX_DRIVE_TIME_S:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            raise RuntimeError(
                f"drive_to did not converge within {MAX_DRIVE_TIME_S:.0f}s toward "
                f"({target_x:.2f}, {target_y:.2f})"
            )
        position = node.getPosition()
        dx, dy = target_x - position[0], target_y - position[1]
        distance = math.hypot(dx, dy)
        if distance <= ARRIVAL_TOLERANCE:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            return elapsed

        # Rotate phase: pure turning, zero forward command, always converges on its own.
        bearing = math.atan2(dy, dx)
        rotate_to_heading(supervisor, left_motor, right_motor, node, bearing, start_time)

        # Cruise phase: both wheels ALWAYS equal, zero turn command - open-loop straight,
        # nothing to destabilize. Ramped up over CRUISE_RAMP_S instead of jumping straight to
        # CRUISE_SPEED - a sudden large speed demand from a standing stop is the one thing that
        # differed from the isolation test that proved equal-wheel commands drive straight (see
        # the comment above CRUISE_SPEED). Runs until arrival, until bearing drifts past
        # REALIGN_THRESHOLD_DEG (back to the outer loop to rotate again), or the safety timeout.
        cruise_start = supervisor.getTime()
        while True:
            supervisor.step(TIME_STEP)
            elapsed = supervisor.getTime() - start_time
            if elapsed > MAX_DRIVE_TIME_S:
                left_motor.setVelocity(0.0)
                right_motor.setVelocity(0.0)
                raise RuntimeError(
                    f"drive_to did not converge within {MAX_DRIVE_TIME_S:.0f}s toward "
                    f"({target_x:.2f}, {target_y:.2f})"
                )
            ramp_fraction = min(1.0, (supervisor.getTime() - cruise_start) / CRUISE_RAMP_S)
            speed = CRUISE_SPEED * ramp_fraction
            left_motor.setVelocity(speed)
            right_motor.setVelocity(speed)

            position = node.getPosition()
            heading = _read_heading(node)
            dx, dy = target_x - position[0], target_y - position[1]
            distance = math.hypot(dx, dy)
            if distance <= ARRIVAL_TOLERANCE:
                left_motor.setVelocity(0.0)
                right_motor.setVelocity(0.0)
                return elapsed
            bearing = math.atan2(dy, dx)
            drift_deg = math.degrees(abs(_wrap(bearing - heading)))

            if DEBUG_PRINT_INTERVAL_S > 0 and elapsed - last_debug >= DEBUG_PRINT_INTERVAL_S:
                last_debug = elapsed
                print(
                    f"  t={elapsed:5.1f}s pos=({position[0]:+.3f},{position[1]:+.3f}) "
                    f"dist={distance:.3f} heading={math.degrees(heading):+6.1f}deg "
                    f"drift={drift_deg:5.1f}deg [cruise]"
                )

            if drift_deg > REALIGN_THRESHOLD_DEG:
                break  # back to the outer loop: stop cruising, rotate again


def main() -> None:
    supervisor = Supervisor()
    left_motor = supervisor.getDevice("left wheel motor")
    right_motor = supervisor.getDevice("right wheel motor")
    for motor in (left_motor, right_motor):
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)

    rows = []
    for distance in DISTANCES_M:
        reset_pose(supervisor)
        supervisor.step(TIME_STEP)  # let physics settle after the teleport
        elapsed = drive_to(supervisor, left_motor, right_motor, distance, 0.0)
        rows.append({"distance_m": distance, "elapsed_s": round(elapsed, 3)})
        print(f"distance={distance:.2f} m elapsed={elapsed:.3f} s")

    with OUTPUT_CSV.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["distance_m", "elapsed_s"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {len(rows)} runs to {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
