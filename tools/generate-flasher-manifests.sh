#!/usr/bin/env bash
# Generate ESP Web Tools manifests and projects.json from per-project flasher.json metadata.
#
# Usage:
#   ./tools/generate-flasher-manifests.sh <version> [firmware-dir]
#
# Arguments:
#   version      Release version string, e.g. "0.3.1"
#   firmware-dir Output directory for manifests (default: "firmware")
#
# For every packages/*/*/flasher.json found, this script:
#   1. Reads chipFamily and appBinaryName from flasher.json
#   2. Determines flash offsets (bootloader/partition-table/otadata/app) from
#      partitions.csv if present, otherwise uses chip-family defaults
#   3. Writes firmware/<project-id>/manifest.json (ESP Web Tools format)
#   4. Writes firmware/projects.json index consumed by the flasher page
#
# Requirements: bash >= 4, jq
#
# Chip-family bootloader offsets:
#   ESP32, ESP32-S2  ->  0x1000 (4096)
#   ESP32-S3, ESP32-C3, ESP32-C6, ESP32-H2  ->  0x0 (0)
#
# Default partition offsets (no partitions.csv):
#   partition-table  ->  0x8000  (32768)  -- same for all families
#   app              ->  0x10000 (65536)  -- same for all families
#   otadata          ->  none             -- only if found in partitions.csv

set -euo pipefail

VERSION="${1:?Usage: $0 <version> [firmware-dir]}"
FIRMWARE_DIR="${2:-firmware}"
PROJECTS_GLOB="packages/*/*/flasher.json"

# Associative array: chip family -> bootloader offset (decimal)
bootloader_offset() {
    case "$1" in
        "ESP32"|"ESP32-S2") echo 4096 ;;  # 0x1000
        *)                   echo 0    ;;  # 0x0  (S3, C3, C6, H2)
    esac
}

# Parse the first app partition offset from partitions.csv.
# Returns decimal offset, or 65536 (0x10000) if not found.
parse_app_offset() {
    local csv="$1"
    local offset=""
    if [[ -f "$csv" ]]; then
        offset=$(grep -v '^#' "$csv" \
            | awk -F',' '{ gsub(/[[:space:]]/, "", $2) } $2 == "app" { gsub(/[[:space:]]/, "", $4); print $4; exit }')
    fi
    if [[ -z "$offset" ]]; then
        echo 65536
        return
    fi
    # Convert hex (0x...) to decimal
    if [[ "$offset" == 0x* || "$offset" == 0X* ]]; then
        printf "%d\n" "$offset"
    else
        echo "$offset"
    fi
}

# Parse the OTA data partition offset from partitions.csv.
# Returns decimal offset, or empty string if no OTA data partition exists.
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

PROJECTS_JSON_ARRAY="[]"

echo "Scanning ${PROJECTS_GLOB} ..."

for flasher_json in ${PROJECTS_GLOB}; do
    project_dir=$(dirname "$flasher_json")
    project_id=$(basename "$project_dir")

    echo ""
    echo "  [$project_id]"

    # Read flasher.json fields
    name=$(jq -r '.name'                       "$flasher_json")
    description=$(jq -r '.description'         "$flasher_json")
    chip_family=$(jq -r '.chipFamily'          "$flasher_json")
    board=$(jq -r '.board'                     "$flasher_json")
    app_binary_name=$(jq -r '.appBinaryName'   "$flasher_json")
    category=$(jq -r '.category // "standalone"' "$flasher_json")
    flash_mode=$(jq -r '.flashMode // "uart"'  "$flasher_json")
    improv=$(jq -r '.improv // false'          "$flasher_json")

    # Determine offsets
    bl_offset=$(bootloader_offset "$chip_family")
    pt_offset=32768   # 0x8000  -- partition table always here
    app_offset=$(parse_app_offset "${project_dir}/partitions.csv")
    ota_offset=$(parse_otadata_offset "${project_dir}/partitions.csv")

    echo "    chip=$chip_family  bl=0x$(printf '%x' "$bl_offset")  pt=0x$(printf '%x' "$pt_offset")  app=0x$(printf '%x' "$app_offset")"
    [[ -n "$ota_offset" ]] && echo "    otadata=0x$(printf '%x' "$ota_offset")"

    # Build the parts array
    parts="["
    parts+=$(printf '{"path":"bootloader.bin","offset":%d}' "$bl_offset")
    parts+=","
    parts+=$(printf '{"path":"partition-table.bin","offset":%d}' "$pt_offset")
    if [[ -n "$ota_offset" ]]; then
        parts+=","
        parts+=$(printf '{"path":"ota_data_initial.bin","offset":%d}' "$ota_offset")
    fi
    parts+=","
    parts+=$(printf '{"path":"%s","offset":%d}' "$app_binary_name" "$app_offset")
    parts+="]"

    # Write manifest
    manifest_dir="${FIRMWARE_DIR}/${project_id}"
    mkdir -p "$manifest_dir"

    jq -n \
        --arg  name    "$name" \
        --arg  version "$VERSION" \
        --arg  chip    "$chip_family" \
        --argjson parts "$parts" \
        '{
            "name":    $name,
            "version": $version,
            "builds": [{"chipFamily": $chip, "parts": $parts}]
        }' > "${manifest_dir}/manifest.json"

    echo "    -> ${manifest_dir}/manifest.json"

    # Append project entry to projects index array
    PROJECTS_JSON_ARRAY=$(echo "$PROJECTS_JSON_ARRAY" | jq -c \
        --arg  id           "$project_id" \
        --arg  name         "$name" \
        --arg  description  "$description" \
        --arg  chip         "$chip_family" \
        --arg  board        "$board" \
        --arg  version      "$VERSION" \
        --arg  category     "$category" \
        --arg  flash_mode   "$flash_mode" \
        --argjson improv    "$improv" \
        --arg  manifest     "firmware/${project_id}/manifest.json" \
        '. + [{
            "id":           $id,
            "name":         $name,
            "description":  $description,
            "chipFamily":   $chip,
            "board":        $board,
            "version":      $version,
            "category":     $category,
            "flashMode":    $flash_mode,
            "improv":       $improv,
            "manifestPath": $manifest
        }]')
done

# Write top-level projects.json
GENERATED_AT=$(date -u +%Y-%m-%dT%H:%M:%SZ)

jq -n \
    --arg  version      "$VERSION" \
    --arg  generated_at "$GENERATED_AT" \
    --argjson projects  "$PROJECTS_JSON_ARRAY" \
    '{
        "version":     $version,
        "generatedAt": $generated_at,
        "projects":    $projects
    }' > "${FIRMWARE_DIR}/projects.json"

PROJECT_COUNT=$(echo "$PROJECTS_JSON_ARRAY" | jq 'length')
echo ""
echo "Done. Generated ${FIRMWARE_DIR}/projects.json with ${PROJECT_COUNT} projects."
