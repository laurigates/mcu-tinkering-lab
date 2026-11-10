#!/usr/bin/env bash
# List the Bluetooth devices the cutter driver can see, then stop. Nothing is
# sent to a cutter.
#
#   tools/labels/scan-cutter.sh            # Bluetooth LE
#   tools/labels/scan-cutter.sh bluetooth  # Bluetooth Classic (Linux, Windows)
#
# The list is every visible device, not only cutters. Find the Portrait 2 in
# it and export LABEL_CUTTER=ble, plus LABEL_CUTTER_NAME if the name shown does
# not contain PORTRAIT2. On macOS the first scan asks for Bluetooth permission
# for the terminal app.
set -euo pipefail

kind=${1:-ble}
case "$kind" in
    ble | bluetooth) ;;
    *)
        echo "usage: scan-cutter.sh [ble|bluetooth]" >&2
        exit 1
        ;;
esac

home=${SILHOUETTE_HOME:-$HOME/.cache/inkscape-silhouette}
python=$home/venv/bin/python
driver=$home/src/sendto_silhouette.py
if [ ! -x "$python" ] || [ ! -f "$driver" ]; then
    echo "error: no driver at $home; run: just labels::setup" >&2
    exit 1
fi

# The driver is an Inkscape extension and wants a document, even to scan.
svg=$(mktemp -t scan-cutter.XXXXXX)
trap 'rm -f "$svg"' EXIT
echo '<svg xmlns="http://www.w3.org/2000/svg" width="10mm" height="10mm"/>' >"$svg"

"$python" "$driver" --connection_type="$kind" --bluetooth_scan=True "$svg"
