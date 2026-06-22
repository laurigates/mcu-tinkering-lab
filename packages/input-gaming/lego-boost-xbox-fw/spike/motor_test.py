"""Phase 2 baseline: drive the Move Hub motors (Vernie 17101 port map).

Run on the hub over BLE:  just boost-fw::run motor_test

Vernie port map (from ../lego-boost-xbox/README.md):
  A = built-in track motor (left)   B = built-in track motor (right)
  C = color/distance sensor          D = external head motor

Pass gate: tracks spin forward then reverse, head sweeps, hub light cycles.
"""

from pybricks.hubs import MoveHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Color
from pybricks.tools import wait

hub = MoveHub()
hub.light.on(Color.GREEN)

left = Motor(Port.A)
right = Motor(Port.B)

# Head motor is optional depending on the build.
try:
    head = Motor(Port.D)
except OSError:
    head = None
    print("No motor on port D (head) — skipping head sweep")

print("Tracks forward")
left.run(400)
right.run(400)
wait(1500)

print("Tracks reverse")
left.run(-400)
right.run(-400)
wait(1500)

left.stop()
right.stop()

if head is not None:
    print("Head sweep")
    head.run_angle(300, 90)
    head.run_angle(300, -90)

hub.light.on(Color.BLUE)
print("motor_test done")
