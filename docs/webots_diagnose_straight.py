"""Isolation test: do EQUAL, constant wheel commands actually drive straight in this world?

This has nothing to do with calibrate_travel_model's control logic - it commands both wheels
to the exact same constant velocity and does nothing else. If heading still drifts by more
than a degree or two over 15s, the cause is external to our controller: a physical/PROTO
asymmetry, a collision, or another active controller - not anything tunable in
calibrate_travel_model.py.

Setup:
  1. In Webots, create a NEW controller: Wizards > New > Robot Controller (Python)...,
     name it exactly `diagnose_straight`.
  2. Replace its generated content with this file's content.
  3. Temporarily set the E-puck's `controller` field to "diagnose_straight" (remember the
     current value is "calibrate_travel_model" - switch it back afterward).
  4. Ctrl+S, Revert/Ctrl+Shift+R, Play. Let it run the full 15s.
  5. Read the printed positions/headings.
"""

from __future__ import annotations

import math

from controller import Supervisor  # type: ignore[import-not-found]

TIME_STEP = 64
SPEED = 3.0  # rad/s - modest, well under MAX_WHEEL_SPEED, safe for this isolation test
RUN_SECONDS = 15.0

supervisor = Supervisor()
left_motor = supervisor.getDevice("left wheel motor")
right_motor = supervisor.getDevice("right wheel motor")
for motor in (left_motor, right_motor):
    motor.setPosition(float("inf"))
    motor.setVelocity(SPEED)  # identical on both wheels, set once, never touched again

node = supervisor.getSelf()
node.getField("translation").setSFVec3f([0.0, 0.0, 0.0])
node.getField("rotation").setSFRotation([0.0, 0.0, 1.0, 0.0])
supervisor.simulationResetPhysics()

print(f"Both wheels commanded to exactly {SPEED} rad/s, unchanged for the next {RUN_SECONDS:.0f}s.")
print("If raw_heading below moves by more than 1-2 degrees, something outside this script is")
print("turning the robot.")

t = 0.0
last_print = -1.0
while t < RUN_SECONDS:
    supervisor.step(TIME_STEP)
    t = supervisor.getTime()
    if t - last_print >= 1.0:
        last_print = t
        pos = node.getPosition()
        rot = node.getOrientation()
        raw_heading_deg = math.degrees(math.atan2(rot[3], rot[0]))
        print(f"t={t:5.1f}s pos=({pos[0]:+.3f},{pos[1]:+.3f}) raw_heading={raw_heading_deg:+6.1f}deg")

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
print("done - both wheels were commanded EXACTLY equal the whole time")
