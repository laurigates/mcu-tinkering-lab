#!/usr/bin/env bash
# Does the app binary fit the partition table it was built with?
#
# Usage:
#   tools/check-app-partition-fit.sh <project-dir> [report-file]
#
# Must run inside the ESP-IDF container, after `idf.py build` — it needs the
# build's own flasher_args.json and $IDF_PATH's check_sizes.py.
#
# The limit is NOT a number this repo owns. `idf.py build` already fails when
# the app exceeds the SMALLEST app partition of the table it just generated
# (esptool_py's `app_check_size` target, attached to `app` since ESP-IDF 5.x),
# using the real partition-table.bin — so it is exact for every layout in
# this monorepo at once: 1.81 MB ota_0/ota_1 on the 4 MB robocars, 3.5 MB on
# the 8 MB XIAO, 2–3 MB factory-only tables, and the 1 MB single-app default
# of every project with no partitions.csv. A hardcoded "1.8 MB" gate was wrong
# for 14 of the 16 flasher-enabled projects, and even for the two it was
# written for: their ota_0 is 0x1D0000 = 1900544 bytes, not 1887436
# (issue #556).
#
# This script re-runs that same shipped check, verbatim, for two reasons:
#   1. Its verdict becomes a named, per-project step output and a report file
#      the workflows can put in the job summary, instead of one line deep in
#      a build log.
#   2. It is an independent assertion that the check ran at all. If a future
#      ESP-IDF or sdkconfig change detached app_check_size from the build,
#      a run that measured nothing fails here instead of passing by absence.
#
# Everything is read from build/flasher_args.json (app file, partition-table
# file, partition-table offset), which ESP-IDF writes from the same CMake
# variables the build used — no re-parse of partitions.csv, no assumption
# about the project name, no 0x8000 typed in by hand.
#
# Headroom floor. Fitting is not enough: it-troubleshooter fit with 0x4570
# bytes (2%) to spare and esp32-cam-webserver with 5%, and the only signal
# was ESP-IDF's "nearly full" line inside a green job (#582, #584). So the
# script also FAILS when free space in the smallest app partition is below
# APP_PARTITION_MIN_FREE_PCT percent (default 5, the same threshold ESP-IDF
# warns at). The fix it asks for is the one-line partition change, made in
# the PR that would otherwise have sailed through:
#     CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE=y   # or a partitions.csv
# Set APP_PARTITION_MIN_FREE_PCT=0 to disable the floor for one run.
#
# Exit: 0 fits with headroom, 1 does not fit / below the floor / the check
# could not run, 2 usage.

set -euo pipefail

usage() {
    echo "Usage: $0 <project-dir> [report-file]" >&2
    exit 2
}

project_dir="${1:-}"
[[ -n "$project_dir" ]] || usage
report="${2:-$project_dir/build/app-partition-fit.txt}"

min_free_pct="${APP_PARTITION_MIN_FREE_PCT:-5}"
if ! [[ "$min_free_pct" =~ ^[0-9]+$ ]] || (( min_free_pct > 100 )); then
    echo "APP_PARTITION_MIN_FREE_PCT must be an integer 0-100, got '$min_free_pct'" >&2
    exit 2
fi

if [[ -z "${IDF_PATH:-}" ]]; then
    echo "::error::IDF_PATH is not set — run this inside the ESP-IDF container, after idf.py build"
    exit 1
fi
checker="$IDF_PATH/components/partition_table/check_sizes.py"
if [[ ! -f "$checker" ]]; then
    echo "::error::$checker not found — ESP-IDF moved its partition size check; update this script"
    exit 1
fi

flasher_args="$project_dir/build/flasher_args.json"
if [[ ! -f "$flasher_args" ]]; then
    echo "::error::$flasher_args not found — did idf.py build run in $project_dir?"
    exit 1
fi

read -r app_file pt_file pt_offset < <(python3 - "$flasher_args" <<'PY'
import json
import sys

with open(sys.argv[1]) as f:
    args = json.load(f)
print(args["app"]["file"], args["partition-table"]["file"], args["partition-table"]["offset"])
PY
)

app_bin="$project_dir/build/$app_file"
pt_bin="$project_dir/build/$pt_file"
for f in "$app_bin" "$pt_bin"; do
    if [[ ! -f "$f" ]]; then
        echo "::error::$f named by flasher_args.json but not present in the build tree"
        exit 1
    fi
done

project_id=$(basename "$(cd "$project_dir" && pwd)")

# check_sizes.py prints one line per verdict and exits 1 (SystemExit) when
# every app partition is too small; a partial fit is a warning, by ESP-IDF's
# own rule. pipefail keeps its status through the tee into the report.
if ! python3 "$checker" --offset "$pt_offset" partition --type app "$pt_bin" "$app_bin" 2>&1 | tee "$report"; then
    echo "::error::$project_id: $app_file does not fit the app partition(s) of the table it was built with (see above)"
    exit 1
fi

# A run that measured nothing must not pass by absence. The two hex values
# on the verdict line are the floor's inputs, so parse them rather than
# grep for a phrase: a line that fails to parse is treated the same way.
read -r bin_hex part_hex < <(sed -n \
    's/.*binary size \(0x[0-9a-fA-F]*\) bytes\. Smallest app partition is \(0x[0-9a-fA-F]*\) bytes\..*/\1 \2/p' \
    "$report")
if [[ -z "${bin_hex:-}" || -z "${part_hex:-}" ]]; then
    echo "::error::$project_id: check_sizes.py produced no size verdict for $app_file — refusing to treat that as a pass"
    exit 1
fi

bin_size=$((bin_hex))
part_size=$((part_hex))
free=$((part_size - bin_size))
# Integer percent, truncated, so a 4.9% margin reads as 4 and fails a 5 floor.
free_pct=$((free * 100 / part_size))
if (( free * 100 < min_free_pct * part_size )); then
    printf '::error::%s: %s leaves %#x bytes (%d%%) of the %#x-byte app partition — below the %d%% headroom floor (APP_PARTITION_MIN_FREE_PCT). Enlarge the partition, e.g. CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE=y or a partitions.csv, rather than trimming the build to the byte.\n' \
        "$project_id" "$app_file" "$free" "$free_pct" "$part_size" "$min_free_pct"
    exit 1
fi
echo "headroom: $free_pct% free, floor $min_free_pct%" | tee -a "$report"

# ESP-IDF's own "nearly full" line, or a partial fit (some app partitions
# too small), surfaces as a warning when the floor above did not fail it.
if grep -q '^Warning:' "$report"; then
    echo "::warning::$project_id: $(grep -m1 '^Warning:' "$report")"
fi
