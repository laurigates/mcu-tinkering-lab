#!/usr/bin/env bash
# Detect ESP32-S3 USB-Serial-JTAG port by Espressif VID (0x303a = 12346)
# Prints /dev/cu.* path or exits 1 if not found.
# Usage: detect-esp32s3-port.sh [--quiet]

set -euo pipefail

quiet=false
[[ "${1:-}" == "--quiet" ]] && quiet=true

port=$(ioreg -r -c IOUSBHostDevice -l 2>/dev/null \
    | grep -A 30 '"idVendor" = 12346' \
    | grep IODialinDevice \
    | head -1 \
    | sed 's/.*"\(\/dev\/[^"]*\)".*/\1/' \
    | sed 's/tty\./cu./' || true)

if [ -z "$port" ]; then
    if ! $quiet; then
        echo "ERROR: No ESP32-S3 USB-Serial-JTAG device found" >&2
        echo "  - Is the board plugged in?" >&2
        echo "  - Try: ioreg -p IOUSB -l | grep -B 5 -A 10 'USB JTAG'" >&2
        echo "  - Or set PORT/S3_PORT manually" >&2
    fi
    exit 1
fi

echo "$port"
