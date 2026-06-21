"""Phase 3 target: connect an Xbox controller directly to the Move Hub and read it.

Run on a *custom* movehub build that has the Xbox feature enabled (see Phase 3):
    just boost-fw::run xbox_test

On stock movehub firmware the `XboxController` import fails (the feature is not
compiled in) — that is expected and is exactly what Phase 3 sets out to change.

Turn the controller on and hold the pair button until the Xbox light flashes
quickly. The most likely failure point is the BLE pairing/bonding handshake (the
Xbox HID service requires LE security that the LEGO Remote does not) — record
exactly where it stops in docs/spike-log.md.

Pass gate: connects, pairs, and prints live joystick/button values.
"""

from pybricks.iodevices import XboxController
from pybricks.tools import wait

print("Searching for an Xbox controller (hold the pair button)...")
controller = XboxController()
print("Connected.")

while True:
    # API per the Pybricks docs for your firmware version; adjust if names differ.
    lx, ly = controller.joystick_left()
    rx, ry = controller.joystick_right()
    pressed = controller.buttons.pressed()
    print("L=({:.2f},{:.2f}) R=({:.2f},{:.2f}) btn={}".format(lx, ly, rx, ry, pressed))
    wait(100)
