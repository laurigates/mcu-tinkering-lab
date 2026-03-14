#!/usr/bin/env bash
# Serial monitor for ESP32-S3 via USB-Serial-JTAG with auto-reset
# Uses pyserial — cat doesn't reliably read USB-Serial-JTAG CDC devices on macOS.
# Usage: esp32s3-monitor.sh <port> [baud]

set -euo pipefail

port="${1:?Usage: esp32s3-monitor.sh <port> [baud]}"
baud="${2:-115200}"

echo "Monitoring $port @ ${baud} baud — Ctrl-C to stop"
uvx --from pyserial python3 -c "
import serial, sys, signal, time
signal.signal(signal.SIGINT, lambda *a: sys.exit(0))
s = serial.Serial('$port', $baud, timeout=1)
# Reset board so we see boot messages
s.dtr = False; s.rts = True; time.sleep(0.1)
s.rts = False; s.dtr = True; time.sleep(0.1)
s.dtr = False
try:
    while True:
        data = s.read(4096)
        if data:
            sys.stdout.buffer.write(data)
            sys.stdout.buffer.flush()
except serial.SerialException:
    print('\nDevice disconnected', file=sys.stderr)
"
