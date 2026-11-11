#!/usr/bin/env bash
# Kiss-cut one printed sticker page on a Silhouette Portrait 2 (AutoBlade),
# headless, through inkscape-silhouette's sendto_silhouette.py.
#
#   tools/labels/cut.sh <cut-pN.svg> <plan.json>
#
# The registration-mark geometry is read from plan.json, the same numbers
# labels.typ printed, so the CLI flags cannot drift from the print.
#
# Environment (just labels::cut sets these from its arguments):
#   DRY_RUN=true    send nothing; write the command stream to <svg>.cmd and a log
#   DEPTH=1         AutoBlade depth 0-10 (sticker paper: 1; raise if it does not cut through the face)
#   PRESSURE=0      force 1-33, or 0 for the media default
#   SPEED=0         1-10, or 0 for the media default
#   MEDIA=134       inkscape-silhouette's "Sticker Sheet" preset
#   MAT=portrait_8x12
#   SILHOUETTE_HOME=~/.cache/inkscape-silhouette   (setup-cutter.sh installs there)
#   LABEL_MARK_TOP=20   mm from the machine origin, in the feed direction, to
#                       where the mark search starts. The sheet sits on the
#                       mat behind the mat's border, and from 0 the Portrait 2
#                       gave up before feeding in to the square; from 20 it
#                       found the marks and the cut landed on the sticker
#   LABEL_MARK_LEFT=0   the same across the sheet
#
# Connection (exported by the caller; a dry run ignores it and never connects):
#   LABEL_CUTTER=usb          usb (default), ble (Bluetooth LE) or bluetooth
#                             (Classic RFCOMM, which the driver has on Linux and
#                             Windows only)
#   LABEL_CUTTER_NAME=PORTRAIT2   ble: advertised name, exact or a substring of it
#   LABEL_CUTTER_ID=          ble: local identifier from scan-cutter.sh, used
#                             in place of the name (a per-computer UUID on macOS)
#   LABEL_CUTTER_ADDR=        bluetooth: the paired cutter's MAC address
set -euo pipefail

svg=${1:?usage: cut.sh <cut-pN.svg> <plan.json>}
plan=${2:?usage: cut.sh <cut-pN.svg> <plan.json>}
home=${SILHOUETTE_HOME:-$HOME/.cache/inkscape-silhouette}
python=$home/venv/bin/python
driver=$home/src/sendto_silhouette.py
if [ ! -x "$python" ] || [ ! -f "$driver" ]; then
    echo "error: no driver at $home; run: just labels::setup" >&2
    exit 1
fi

# x y w l of the marks, from the plan, and where the mark search starts.
# The driver starts the search at (regorigin - 10 mm) from the machine origin
# and places the cut at (offset - regorigin). Adding the same distance to both
# moves the search start and leaves the cut where it was.
mark_top=${LABEL_MARK_TOP:-20}
mark_left=${LABEL_MARK_LEFT:-0}
if ! reg=$(python3 -c '
import json, sys
m = json.load(open(sys.argv[1]))["regmarks"]
top, left = float(sys.argv[2]), float(sys.argv[3])
print(m["x"] + left, m["y"] + top, m["w"], m["l"], left, top)
' "$plan" "$mark_top" "$mark_left"); then
    echo "error: LABEL_MARK_TOP ('$mark_top') and LABEL_MARK_LEFT ('$mark_left') must be numbers, in mm" >&2
    exit 1
fi
read -r rx ry rw rl xoff yoff <<<"$reg"

args=(
    --preview=False
    --tool=autoblade
    --depth="${DEPTH:-1}"
    --media="${MEDIA:-134}"
    --pressure="${PRESSURE:-0}"
    --speed="${SPEED:-0}"
    --cuttingmat="${MAT:-portrait_8x12}"
    --regmark=True
    --regsearch=True
    --regoriginx="$rx" --regoriginy="$ry"
    --regwidth="$rw" --reglength="$rl"
    --x_off="$xoff" --y_off="$yoff"
    # The driver's default, "below", ends a job by feeding to the bottom of the
    # cut and making that the new origin. The next job on the same sheet then
    # searches for the marks that much further in, and the second or third
    # job fails with reply -1. "start" returns the mat to where the job began.
    --endposition=start
    --logfile="${svg%.svg}.log"
)
if [ "${DRY_RUN:-false}" = "true" ]; then
    # No machine: name the model so the driver builds the Portrait 2 command stream.
    args+=(--dry_run=True --force_hardware=Silhouette_Portrait2 --cmdfile="${svg%.svg}.cmd")
else
    args+=(--wait=True)
    # Over USB the driver knows the model from the product id. Over Bluetooth
    # it reads it from the firmware reply, and a reply it does not recognise
    # leaves it with no model, so name the Portrait 2 as the dry run does.
    case "${LABEL_CUTTER:-usb}" in
        usb) ;;
        ble)
            args+=(--connection_type=ble --force_hardware=Silhouette_Portrait2)
            if [ -n "${LABEL_CUTTER_ID:-}" ]; then
                args+=(--bluetooth_identifier="$LABEL_CUTTER_ID")
            else
                args+=(--bluetooth_name="${LABEL_CUTTER_NAME:-PORTRAIT2}")
            fi
            ;;
        bluetooth)
            if [ -z "${LABEL_CUTTER_ADDR:-}" ]; then
                echo "error: LABEL_CUTTER=bluetooth needs LABEL_CUTTER_ADDR, the paired cutter's MAC address" >&2
                exit 1
            fi
            args+=(--connection_type=bluetooth --force_hardware=Silhouette_Portrait2
                --bluetooth_addr="$LABEL_CUTTER_ADDR")
            ;;
        *)
            echo "error: LABEL_CUTTER is '${LABEL_CUTTER}'; use usb, ble or bluetooth" >&2
            exit 1
            ;;
    esac
fi

echo "regmarks: origin ($rx, $ry) mm, mark to mark $rw x $rl mm"
if [ "${DRY_RUN:-false}" = "true" ]; then
    # dry_cut.py answers the mark search, which a plain dry run waits on forever.
    "$python" "$(dirname "$0")/dry_cut.py" "$driver" "${args[@]}" "$svg"
    echo "dry run: commands in ${svg%.svg}.cmd, log in ${svg%.svg}.log"
else
    # autoblade.py sends the blade depth, which the driver leaves out for a Portrait 2.
    "$python" "$(dirname "$0")/autoblade.py" "$driver" "${args[@]}" "$svg"
fi
