#!/usr/bin/env bash
# Generate ESP Web Tools manifests and projects.json from per-project flasher.json metadata.
#
# Usage:
#   ./tools/generate-flasher-manifests.sh <version> [firmware-dir]
#
# Arguments:
#   version      Release tag that triggered the build, e.g. "robocar-v<version>".
#                Used as the site-level version and as a fallback for projects
#                missing from .release-please-manifest.json.
#   firmware-dir Output directory for manifests (default: "firmware")
#
# Per-project versions come from .release-please-manifest.json — this is a
# release-please monorepo where every package carries its own version, so the
# triggering release's tag would be wrong for every other project's card.
#
# For every packages/*/*/flasher.json found, this script:
#   1. Reads chipFamily and appBinaryName from flasher.json
#   2. Determines flash offsets (bootloader/partition-table/otadata/app) with
#      lib/otadata-predicate.sh, the parser tools/check-flash-recipes.py and
#      build-firmware.yml's assemble step use (issue #1090), from the
#      partition table the build uses (resolve_partition_table): the
#      project's CSV under CONFIG_PARTITION_TABLE_CUSTOM=y, otherwise the
#      built-in preset's resolved copy in lib/idf-partition-tables/. An offset
#      the predicate cannot name stops the run (issues #684, #689, #1090)
#   3. Writes firmware/<project-id>/manifest.json (ESP Web Tools format),
#      plus a top-level "buildSha" ESP Web Tools ignores (see BUILD_SHA below)
#      and "notes": the version's Features / Bug Fixes / Performance entries
#      from the project's CHANGELOG.md as a JSON array of short strings
#      (tools/changelog-notes.py), [] when there is no section. ESP Web Tools
#      ignores it too; it is there for robocar to announce an update.
#   4. Writes firmware/projects.json index consumed by the flasher page
#
# Environment:
#   BUILD_SHA    Full 40-hex commit the firmware was built from. Defaults to
#                `git rev-parse HEAD` of the current checkout, which in
#                build-firmware.yml is the same tag checkout the build job
#                compiled. Written into every manifest as "buildSha": every
#                release rebuilds every project at the triggering commit, so a
#                project's version alone cannot say which commit its binary
#                came from. robocar's OTA compares it against the SHA
#                compiled into the running firmware (issue #627).
#
# Requirements: bash >= 4, jq, python3, git (unless BUILD_SHA is set)
#
# Offsets (lib/otadata-predicate.sh):
#   bootloader       ->  bootloader_offset of the chipFamily's ESP-IDF target
#                        ("ESP32-C5" -> esp32c5 -> 0x2000)
#   partition-table  ->  partition_table_offset: CONFIG_PARTITION_TABLE_OFFSET,
#                        default 0x8000
#   app              ->  boot_app_offset: the factory partition, else the
#                        lowest ota_N, where `idf.py flash` writes the app
#   otadata          ->  parse_otadata_offset, only if the table has a
#                        data,ota row; a row with a blank offset fails the run

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/otadata-predicate.sh
source "${SCRIPT_DIR}/lib/otadata-predicate.sh"

VERSION="${1:?Usage: $0 <version> [firmware-dir]}"
FIRMWARE_DIR="${2:-firmware}"
PROJECTS_GLOB="packages/*/*/flasher.json"

BUILD_SHA="${BUILD_SHA:-$(git rev-parse HEAD 2>/dev/null || true)}"
# A missing or abbreviated SHA must stop the release here, not publish
# manifests every OTA consumer would refuse.
if [[ ! "$BUILD_SHA" =~ ^[0-9a-f]{40}$ ]]; then
    echo "ERROR: build commit SHA '${BUILD_SHA}' is not 40 lowercase hex digits — set BUILD_SHA or run inside the checkout that was built" >&2
    exit 1
fi
echo "Build commit: ${BUILD_SHA}"
RELEASE_MANIFEST=".release-please-manifest.json"

# Look up a project's own version in the release-please manifest (keyed by the
# package path, e.g. "packages/audio/kids-audio-toy"). Falls back to $VERSION.
project_version() {
    local dir="$1" v=""
    if [[ -f "$RELEASE_MANIFEST" ]]; then
        v=$(jq -r --arg p "$dir" '.[$p] // empty' "$RELEASE_MANIFEST")
    fi
    echo "${v:-$VERSION}"
}

# The ESP-IDF target of an ESP Web Tools chipFamily: "ESP32-S3" -> "esp32s3".
chip_target() {
    echo "$1" | tr '[:upper:]' '[:lower:]' | tr -d '-'
}

# Every offset comes from lib/otadata-predicate.sh, shared with
# build-firmware.yml's assemble step and tools/check-flash-recipes.py so none
# of them parses a partition table differently. Each predicate call is its own
# assignment: under set -e a failing one stops the run, where inside a longer
# expression its status would be lost.

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
    version=$(project_version "$project_dir")
    echo "    version=$version"

    # Determine offsets
    target=$(chip_target "$chip_family")
    if ! bl_offset=$(bootloader_offset "$target"); then
        echo "ERROR: ${project_id}: no bootloader offset for chipFamily \"${chip_family}\"" >&2
        exit 1
    fi
    pt_offset=$(partition_table_offset "$project_dir")
    partition_table=$(resolve_partition_table "$project_dir")
    ota_offset=$(parse_otadata_offset "$partition_table")
    # A data,ota row with a blank offset is placed by gen_esp32part.py at build
    # time. Leaving the part out would publish a manifest that never writes
    # otadata, and guessing its offset could write it over another partition,
    # so stop the release instead (issue #689).
    if [[ "$ota_offset" == auto ]]; then
        echo "ERROR: ${project_id}: ${partition_table} has an otadata row with a blank offset; give it an explicit offset so the manifest can place ota_data_initial.bin" >&2
        exit 1
    fi
    # The same holds for the app: a blank offset used to become 0x10000
    # (issue #1090).
    app_offset=$(boot_app_offset "$partition_table")

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

    # What this version changes, for an OTA consumer to announce. Never fails
    # the release: a missing changelog or section gives [].
    notes=$(python3 "${SCRIPT_DIR}/changelog-notes.py" "${project_dir}/CHANGELOG.md" "$version")
    echo "    notes=$(echo "$notes" | jq 'length') entries"

    # Write manifest
    manifest_dir="${FIRMWARE_DIR}/${project_id}"
    mkdir -p "$manifest_dir"

    jq -n \
        --arg  name    "$name" \
        --arg  version "$version" \
        --arg  chip    "$chip_family" \
        --arg  sha     "$BUILD_SHA" \
        --argjson parts "$parts" \
        --argjson notes "$notes" \
        '{
            "name":     $name,
            "version":  $version,
            "buildSha": $sha,
            "notes":    $notes,
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
        --arg  version      "$version" \
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
