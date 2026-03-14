#!/usr/bin/env bash
# Reset ESP32-S3 via USB-Serial-JTAG CDC control signals (DTR/RTS pulse)
# Usage: esp32s3-reset.sh <port>

set -euo pipefail

port="${1:?Usage: esp32s3-reset.sh <port>}"

uvx --from pyserial python3 -c "
import serial, time
s = serial.Serial('$port', 115200)
s.dtr = False; s.rts = True; time.sleep(0.1)
s.rts = False; s.dtr = True; time.sleep(0.1)
s.dtr = False; s.close()
"
echo "Reset OK"
