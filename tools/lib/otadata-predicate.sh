#!/usr/bin/env bash
# Shared predicate: does this project's partition table declare a `data,ota` row?
#
# Sourced by both tools/generate-flasher-manifests.sh (to decide whether the
# manifest lists ota_data_initial.bin as a part) and build-firmware.yml's
# "Assemble firmware directory" step (to decide whether that file is required
# in the assembled firmware/<project>/ directory). Keeping one parser behind
# one function is what stops the two call sites from silently disagreeing —
# see issue #541.
#
# Requires: bash >= 4 (associative-array-free, but uses [[ ]] and local).

# Resolve the partition-table CSV that ESP-IDF builds for a project.
# ESP-IDF reads the table named by CONFIG_PARTITION_TABLE_CUSTOM_FILENAME
# (mirrored into CONFIG_PARTITION_TABLE_FILENAME), relative to the project
# directory, and that option defaults to "partitions.csv". Reading the name
# from sdkconfig.defaults instead of assuming it keeps the predicate on the
# file the build uses when a project renames its table (issue #560). Later
# assignments win, as in sdkconfig. Prints the path on stdout; the file may
# not exist, which parse_otadata_offset treats as "no otadata row".
resolve_partition_table() {
    local project_dir="$1"
    local defaults="${project_dir}/sdkconfig.defaults"
    local name=""
    if [[ -f "$defaults" ]]; then
        name=$(awk '
            /^[[:space:]]*#/ { next }
            {
                eq = index($0, "=")
                if (eq == 0) next
                key = substr($0, 1, eq - 1); val = substr($0, eq + 1)
                gsub(/[[:space:]]/, "", key)
                gsub(/^[[:space:]"]+|[[:space:]"]+$/, "", val)
                if (key == "CONFIG_PARTITION_TABLE_CUSTOM_FILENAME") custom = val
                else if (key == "CONFIG_PARTITION_TABLE_FILENAME") plain = val
            }
            END { print (custom != "") ? custom : plain }' "$defaults")
    fi
    name="${name:-partitions.csv}"
    if [[ "$name" == /* ]]; then
        echo "$name"
    else
        echo "${project_dir}/${name}"
    fi
}

# Parse the OTA data partition offset from a partition-table CSV (locate it
# with resolve_partition_table).
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

# Boolean wrapper: succeeds (exit 0) iff the given partition table declares
# a `data,ota` row, i.e. iff ota_data_initial.bin is a real build output for
# this project rather than a file that simply happens not to exist.
has_otadata_partition() {
    local csv="$1"
    [[ -n "$(parse_otadata_offset "$csv")" ]]
}
