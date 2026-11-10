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
# tools/check-flash-recipes.py asks it too, so the flash check reads the same
# table as the release path.
#
# Requires: bash >= 4 (associative-array-free, but uses [[ ]] and local).

# Where ESP-IDF's built-in tables live, resolved to explicit offsets by
# idf-partition-tables/regenerate.sh. Captured at source time, from this file.
_OTADATA_PREDICATE_TABLES="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/idf-partition-tables"

# Resolve the partition-table CSV that ESP-IDF builds for a project, from its
# sdkconfig.defaults, the way components/partition_table/Kconfig.projbuild does
# (issue #684). Later assignments win, as in sdkconfig.
#
# - CONFIG_PARTITION_TABLE_CUSTOM=y: the project's own CSV, named by
#   CONFIG_PARTITION_TABLE_CUSTOM_FILENAME (default "partitions.csv") relative
#   to the project directory (issue #560). CONFIG_PARTITION_TABLE_FILENAME in
#   sdkconfig.defaults is ignored, as kconfig ignores it: it has no prompt.
# - Any other choice, including none (PARTITION_TABLE_SINGLE_APP is the
#   default): the built-in preset ESP-IDF builds instead, and a project CSV is
#   never read. The preset's resolved copy is printed from idf-partition-tables/;
#   CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y picks its _coredump variant.
#
# Prints the path of a file that exists on stdout. Fails closed, with the
# reason on stderr and nothing on stdout, when the table cannot be named: a
# selected custom CSV that is missing, a preset with no table for its options,
# or a preset behind a moved CONFIG_PARTITION_TABLE_OFFSET (the resolved copies
# assume 0x8000, and gen_esp32part.py places every partition after the table).
resolve_partition_table() {
    local project_dir="$1"
    local defaults="${project_dir}/sdkconfig.defaults"
    local choice="" custom="" coredump="" offset=""
    if [[ -f "$defaults" ]]; then
        # Unit separator, not tab: tab is IFS whitespace, so empty fields
        # between tabs would collapse and shift the rest.
        IFS=$'\x1f' read -r choice custom coredump offset < <(awk '
            # "# CONFIG_X is not set" is an assignment of n.
            /^#[[:space:]]*CONFIG_[A-Z0-9_]+[[:space:]]+is not set/ {
                key = $0; sub(/^#[[:space:]]*/, "", key); sub(/[[:space:]].*$/, "", key)
                val = "n"
            }
            !/^[[:space:]]*#/ {
                eq = index($0, "=")
                if (eq == 0) next
                key = substr($0, 1, eq - 1); val = substr($0, eq + 1)
                gsub(/[[:space:]]/, "", key)
                gsub(/^[[:space:]"]+|[[:space:]"]+$/, "", val)
            }
            key ~ /^CONFIG_PARTITION_TABLE_(SINGLE_APP|SINGLE_APP_LARGE|TWO_OTA|TWO_OTA_LARGE|CUSTOM|SINGLE_APP_ENCRYPTED_NVS|SINGLE_APP_LARGE_ENC_NVS|TWO_OTA_ENCRYPTED_NVS)$/ {
                name = substr(key, length("CONFIG_PARTITION_TABLE_") + 1)
                if (val == "y") choice = name
                else if (choice == name) choice = ""
            }
            key == "CONFIG_PARTITION_TABLE_CUSTOM_FILENAME" { custom = val }
            key == "CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH" { coredump = val }
            key == "CONFIG_PARTITION_TABLE_OFFSET" { offset = val }
            { key = "" }
            END { printf "%s\037%s\037%s\037%s\n", choice, custom, coredump, offset }' "$defaults")
    fi
    choice="${choice:-SINGLE_APP}"

    if [[ "$choice" == CUSTOM ]]; then
        local name="${custom:-partitions.csv}"
        [[ "$name" == /* ]] || name="${project_dir}/${name}"
        if [[ ! -f "$name" ]]; then
            echo "ERROR: ${project_dir} selects CONFIG_PARTITION_TABLE_CUSTOM, but its table ${name} does not exist" >&2
            return 1
        fi
        echo "$name"
        return 0
    fi

    local suffix=""
    [[ "$coredump" == y ]] && suffix="_coredump"
    local file=""
    case "$choice" in
        SINGLE_APP) file="partitions_singleapp${suffix}.csv" ;;
        SINGLE_APP_LARGE) file="partitions_singleapp_large${suffix}.csv" ;;
        TWO_OTA) file="partitions_two_ota${suffix}.csv" ;;
        TWO_OTA_LARGE) [[ -z "$suffix" ]] && file="partitions_two_ota_large.csv" ;;
        SINGLE_APP_ENCRYPTED_NVS) file="partitions_singleapp_encr_nvs.csv" ;;
        SINGLE_APP_LARGE_ENC_NVS) file="partitions_singleapp_large_encr_nvs.csv" ;;
        TWO_OTA_ENCRYPTED_NVS) file="partitions_two_ota_encr_nvs.csv" ;;
    esac
    if [[ -z "$file" ]]; then
        echo "ERROR: ${project_dir}: ESP-IDF has no built-in table for CONFIG_PARTITION_TABLE_${choice} with CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=${coredump:-n}" >&2
        return 1
    fi
    if [[ -n "$offset" ]] && ! { [[ "$offset" =~ ^(0[xX][0-9a-fA-F]+|[0-9]+)$ ]] && ((offset == 0x8000)); }; then
        echo "ERROR: ${project_dir} moves CONFIG_PARTITION_TABLE_OFFSET to ${offset}, but the resolved built-in table ${file} assumes 0x8000; select a custom table with explicit offsets" >&2
        return 1
    fi
    if [[ ! -f "${_OTADATA_PREDICATE_TABLES}/${file}" ]]; then
        echo "ERROR: ${_OTADATA_PREDICATE_TABLES}/${file} is missing; run tools/lib/idf-partition-tables/regenerate.sh" >&2
        return 1
    fi
    echo "${_OTADATA_PREDICATE_TABLES}/${file}"
}

# Parse the OTA data partition offset from a partition-table CSV (locate it
# with resolve_partition_table). Prints one of three states on stdout:
#   <decimal offset>  the table has a `data,ota` row at that offset
#   auto              the table has a `data,ota` row whose Offset field is
#                     blank, left for gen_esp32part.py to place (issue #689)
#   (empty)           the table has no `data,ota` row
# Callers treat non-empty stdout as "this project has an OTA data partition",
# not the exit status. `auto` has no offset to write or check against: a
# caller that needs the number fails closed on it, never guesses. A missing
# file is an error (exit 1), not "no row": resolve_partition_table names only
# tables that exist, so a missing one means the caller passed a wrong path.
parse_otadata_offset() {
    local csv="$1"
    local offset=""
    if [[ ! -f "$csv" ]]; then
        echo "ERROR: partition table ${csv} does not exist" >&2
        return 1
    fi
    # A row is printed as "=<offset>" so a blank offset still prints. awk
    # drains its input, so no early-closing reader breaks the pipe.
    offset=$(grep -v '^#' "$csv" \
        | awk -F',' '{ gsub(/[[:space:]]/, "", $2); gsub(/[[:space:]]/, "", $3) } $2 == "data" && $3 == "ota" && !done { gsub(/[[:space:]]/, "", $4); print "=" $4; done = 1 }')
    case "$offset" in
        "") echo "" ;;
        "=") echo "auto" ;;
        =0x* | =0X*) printf "%d\n" "${offset#=}" ;;
        *) echo "${offset#=}" ;;
    esac
}

# Boolean wrapper: succeeds (exit 0) iff the given partition table declares
# a `data,ota` row, with or without an explicit offset, i.e. iff
# ota_data_initial.bin is a real build output for this project rather than a
# file that simply happens not to exist. Returns 2 when the table cannot be
# read; an `if` reads that as false, so resolve the table first with
# resolve_partition_table, which fails closed outside the `if`.
has_otadata_partition() {
    local csv="$1" offset
    offset=$(parse_otadata_offset "$csv") || return 2
    [[ -n "$offset" ]]
}
