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

# x y w l of the marks, from the plan.
read -r rx ry rw rl < <(python3 -c 'import json,sys; m=json.load(open(sys.argv[1]))["regmarks"]; print(m["x"], m["y"], m["w"], m["l"])' "$plan")

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
    --logfile="${svg%.svg}.log"
)
if [ "${DRY_RUN:-false}" = "true" ]; then
    # No machine: name the model so the driver builds the Portrait 2 command stream.
    args+=(--dry_run=True --force_hardware=Silhouette_Portrait2 --cmdfile="${svg%.svg}.cmd")
else
    args+=(--wait=True)
fi

echo "regmarks: origin ($rx, $ry) mm, mark to mark $rw x $rl mm"
if [ "${DRY_RUN:-false}" = "true" ]; then
    # dry_cut.py answers the mark search, which a plain dry run waits on forever.
    "$python" "$(dirname "$0")/dry_cut.py" "$driver" "${args[@]}" "$svg"
    echo "dry run: commands in ${svg%.svg}.cmd, log in ${svg%.svg}.log"
else
    "$python" "$driver" "${args[@]}" "$svg"
fi
