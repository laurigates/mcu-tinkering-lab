"""Phase 1 (optional) central-path confidence check — needs a LEGO Powered Up handset.

Run on stock Pybricks firmware (no custom build required):
    just boost-fw::run remote_test

This proves the Move Hub can act as a BLE *central*: scan for, connect to, and
read input from an external BLE peripheral. The LEGO Remote needs no pairing, so
this is the lower bound; the Xbox controller additionally requires LE
pairing/bonding (see xbox_test.py / Phase 3).

Pass gate: pressed-button names print live as you press the handset buttons.
Skip this phase if you don't own a Powered Up handset.
"""

from pybricks.pupdevices import Remote
from pybricks.tools import wait

print("Searching for a LEGO Powered Up remote (white blinking light = ready)...")
remote = Remote()
print("Connected:", remote.name())

while True:
    pressed = remote.buttons.pressed()
    if pressed:
        print(pressed)
    wait(50)
