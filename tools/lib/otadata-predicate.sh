#!/usr/bin/env bash
# Shared predicate: does this project's partitions.csv declare a `data,ota` row?
#
# Sourced by both tools/generate-flasher-manifests.sh (to decide whether the
# manifest lists ota_data_initial.bin as a part) and build-firmware.yml's
# "Assemble firmware directory" step (to decide whether that file is required
# in the assembled firmware/<project>/ directory). Keeping one parser behind
# one function is what stops the two call sites from silently disagreeing —
# see issue #541.
#
# Requires: bash >= 4 (associative-array-free, but uses [[ ]] and local).

# Parse the OTA data partition offset from partitions.csv.
# Prints the decimal offset on stdout, or nothing if no `data,ota` row exists
# (including when the file itself is missing). Callers should treat non-empty
# stdout as "this project has an OTA data partition", not the exit status.
parse_otadata_offset() {
    local csv="$1"
    local offset=""
    if [[ -f "$csv" ]]; then
        offset=$(grep -v '^#' "$csv" \
            | awk -F',' '{ gsub(/[[:space:]]/, "", $2); gsub(/[[:space:]]/, "", $3) } $2 == "data" && $3 == "ota" { gsub(/[[:space:]]/, "", $4); print $4; exit }')
    fi
    if [[ -z "$offset" ]]; then
        echo ""
        return
    fi
    if [[ "$offset" == 0x* || "$offset" == 0X* ]]; then
        printf "%d\n" "$offset"
    else
        echo "$offset"
    fi
}

# Boolean wrapper: succeeds (exit 0) iff the project's partitions.csv declares
# a `data,ota` row, i.e. iff ota_data_initial.bin is a real build output for
# this project rather than a file that simply happens not to exist.
has_otadata_partition() {
    local csv="$1"
    [[ -n "$(parse_otadata_offset "$csv")" ]]
}
