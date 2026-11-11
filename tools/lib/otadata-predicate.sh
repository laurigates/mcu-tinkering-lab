#!/usr/bin/env bash
# Shared flash-layout predicate: which partition table a project builds, and
# where each build output belongs in flash — the bootloader, the partition
# table, otadata (and whether the table has a `data,ota` row at all) and the
# app.
#
# Sourced by tools/generate-flasher-manifests.sh (every offset in the
# manifest) and build-firmware.yml's "Assemble firmware directory" step
# (whether ota_data_initial.bin is required in the assembled
# firmware/<project>/ directory). tools/check-flash-recipes.py asks it too, so
# the flash check reads the same offsets as the release path. Keeping one
# parser behind these functions is what stops the call sites from silently
# disagreeing — see issues #541 and #1090.
#
# Every offset is printed in decimal on stdout. A function that cannot name an
# offset fails closed: the reason on stderr, nothing on stdout, exit 1.
#
# Requires: bash (associative-array-free; uses [[ ]] and local).

# Where ESP-IDF's built-in tables live, resolved to explicit offsets by
# idf-partition-tables/regenerate.sh. Captured at source time, from this file.
_OTADATA_PREDICATE_TABLES="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/idf-partition-tables"

# Read an integer field of a partition table as gen_esp32part.py's parse_int
# does: int(x, 0) after stripping K/M suffixes, each a factor of 1024 ("52K" is
# 53248, "1M" 1048576). Decimal and 0x-hex are accepted. Anything else fails
# closed, a blank field included; so does a leading-zero decimal ("0100"),
# which int(x, 0) rejects and bash arithmetic would read as octal.
partition_int() {
    local field="$1" value mult=1
    value="${field//[[:space:]]/}"
    while [[ "$value" == *[kKmM] ]]; do
        case "$value" in
            *[kK]) mult=$((mult * 1024)) ;;
            *) mult=$((mult * 1048576)) ;;
        esac
        value="${value%?}"
    done
    if [[ ! "$value" =~ ^(0[xX][0-9a-fA-F]+|0|[1-9][0-9]*)$ ]]; then
        echo "ERROR: '${field}' is not a partition-table integer gen_esp32part.py reads" >&2
        return 1
    fi
    echo $((value * mult))
}

# Where the ROM loads the second-stage bootloader from, for an ESP-IDF target
# name (esp32s3). Copied from ESP-IDF v5.5.6 components/bootloader/Kconfig.projbuild,
# BOOTLOADER_OFFSET_IN_FLASH ("not configurable in ESP-IDF"); the IDF-version
# tripwire in tools/test_check_flash_recipes.py fails when the builds move to
# another version. A target outside this table fails closed.
bootloader_offset() {
    case "$1" in
        esp32 | esp32s2) echo 4096 ;;
        esp32c5 | esp32p4 | esp32h4) echo 8192 ;;
        esp32s3 | esp32c2 | esp32c3 | esp32c6 | esp32c61 | esp32h2) echo 0 ;;
        *)
            echo "ERROR: target \"$1\" has no known bootloader offset; add it to bootloader_offset in tools/lib/otadata-predicate.sh from components/bootloader/Kconfig.projbuild" >&2
            return 1
            ;;
    esac
}

# The partition-table options a project's sdkconfig.defaults sets, as one line
# of four fields separated by the unit separator: the PARTITION_TABLE choice,
# CUSTOM_FILENAME, ESP_COREDUMP_ENABLE_TO_FLASH and PARTITION_TABLE_OFFSET.
# Later assignments win, as in sdkconfig. Only sdkconfig.defaults is read: no
# overlay sets one of these today, which a tripwire test holds.
_partition_options() {
    local defaults="$1/sdkconfig.defaults"
    if [[ ! -f "$defaults" ]]; then
        printf '\037\037\037\n'
        return 0
    fi
    awk '
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
        END { printf "%s\037%s\037%s\037%s\n", choice, custom, coredump, offset }' "$defaults"
}

# Where the partition table is flashed: CONFIG_PARTITION_TABLE_OFFSET from
# sdkconfig.defaults, else ESP-IDF's default 0x8000. The option is a Kconfig
# hex, which kconfiglib reads with int(value, 16), so "9000" is 0x9000. It
# ignores a value that is not hex with a warning and builds 0x8000; a warning
# in a green build is easy to miss, so that fails closed here instead.
partition_table_offset() {
    local choice custom coredump offset
    # Unit separator, not tab: tab is IFS whitespace, so empty fields between
    # tabs would collapse and shift the rest.
    IFS=$'\x1f' read -r choice custom coredump offset < <(_partition_options "$1")
    if [[ -z "$offset" ]]; then
        echo 32768
        return 0
    fi
    if [[ ! "$offset" =~ ^(0[xX])?[0-9a-fA-F]+$ ]]; then
        echo "ERROR: $1 sets CONFIG_PARTITION_TABLE_OFFSET=${offset}, which is not the hex value Kconfig takes" >&2
        return 1
    fi
    echo $((16#${offset#0[xX]}))
}

# Resolve the partition-table CSV that ESP-IDF builds for a project, from its
# sdkconfig.defaults, the way components/partition_table/Kconfig.projbuild does
# (issue #684).
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
    local choice custom coredump offset
    IFS=$'\x1f' read -r choice custom coredump offset < <(_partition_options "$project_dir")
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
    local table_offset
    table_offset=$(partition_table_offset "$project_dir") || return 1
    if ((table_offset != 0x8000)); then
        echo "ERROR: ${project_dir} moves CONFIG_PARTITION_TABLE_OFFSET to ${offset}, but the resolved built-in table ${file} assumes 0x8000; select a custom table with explicit offsets" >&2
        return 1
    fi
    if [[ ! -f "${_OTADATA_PREDICATE_TABLES}/${file}" ]]; then
        echo "ERROR: ${_OTADATA_PREDICATE_TABLES}/${file} is missing; run tools/lib/idf-partition-tables/regenerate.sh" >&2
        return 1
    fi
    echo "${_OTADATA_PREDICATE_TABLES}/${file}"
}

# A missing table is an error, not a table without rows: resolve_partition_table
# names only tables that exist, so a missing one means the caller passed a
# wrong path.
_require_table() {
    if [[ ! -f "$1" ]]; then
        echo "ERROR: partition table $1 does not exist" >&2
        return 1
    fi
}

# The first `data,ota` row's Offset field as "=<field>", so a blank field still
# prints; nothing when the table has no such row. Comment lines are skipped the
# way gen_esp32part.py skips them: after stripping, they start with "#".
_otadata_field() {
    awk -F',' '
        /^[[:space:]]*#/ { next }
        { for (i = 1; i <= 4; i++) gsub(/[[:space:]]/, "", $i) }
        $2 == "data" && $3 == "ota" && !done { print "=" $4; done = 1 }' "$1"
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
# table, or an Offset field partition_int cannot read, is an error (exit 1).
parse_otadata_offset() {
    local csv="$1" field
    _require_table "$csv" || return 1
    field=$(_otadata_field "$csv")
    case "$field" in
        "") echo "" ;;
        "=") echo "auto" ;;
        *)
            partition_int "${field#=}" || {
                echo "ERROR: ${csv}'s otadata row has an offset that cannot be read" >&2
                return 1
            }
            ;;
    esac
}

# Boolean wrapper: succeeds (exit 0) iff the given partition table declares
# a `data,ota` row, whatever its Offset field holds, i.e. iff
# ota_data_initial.bin is a real build output for this project rather than a
# file that simply happens not to exist. Returns 2 when the table cannot be
# read; an `if` reads that as false, so resolve the table first with
# resolve_partition_table, which fails closed outside the `if`.
has_otadata_partition() {
    _require_table "$1" || return 2
    [[ -n "$(_otadata_field "$1")" ]]
}

# The partition `idf.py flash` writes the app to, as "<decimal offset> <name>":
# the table's factory partition, else its lowest ota_N, which is parttool.py's
# --partition-boot-default search. Fails closed when the table has neither, or
# when that partition's Offset field is blank (left to gen_esp32part.py to
# place) or cannot be read.
boot_app_partition() {
    local csv="$1" picked name field offset
    _require_table "$csv" || return 1
    picked=$(awk -F',' '
        /^[[:space:]]*#/ { next }
        { for (i = 1; i <= 4; i++) gsub(/[[:space:]]/, "", $i) }
        $2 == "app" && !($3 in off) { off[$3] = $4 }
        END {
            if ("factory" in off) { print "factory=" off["factory"]; exit }
            for (n = 0; n < 16; n++)
                if (("ota_" n) in off) { print "ota_" n "=" off["ota_" n]; exit }
        }' "$csv")
    if [[ -z "$picked" ]]; then
        echo "ERROR: ${csv} has no factory or ota_N app partition for idf.py flash to write the app to" >&2
        return 1
    fi
    name="${picked%%=*}"
    field="${picked#*=}"
    if [[ -z "$field" ]]; then
        echo "ERROR: ${csv}'s ${name} partition has no explicit offset (gen_esp32part.py places it at build time); give it one so the app can be placed" >&2
        return 1
    fi
    offset=$(partition_int "$field") || {
        echo "ERROR: ${csv}'s ${name} partition has an offset that cannot be read" >&2
        return 1
    }
    echo "${offset} ${name}"
}

# The decimal offset alone, for callers that do not report the partition name.
boot_app_offset() {
    local picked
    picked=$(boot_app_partition "$1") || return 1
    echo "${picked%% *}"
}
