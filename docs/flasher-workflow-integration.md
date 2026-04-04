# Dynamic Web Flasher — Workflow Integration Guide

The web flasher (`docs/flasher/index.html`) uses dynamic project discovery.
On page load it fetches `projects.json` from the same origin and renders one
card per project. `projects.json` is generated at release time by CI using
`tools/generate-flasher-manifests.sh`, which scans every
`packages/esp32-projects/*/flasher.json` opt-in file.

## Required changes to `build-firmware.yml`

The workflow must be updated to:
1. Build all `flasher.json`-enabled projects
2. Run `tools/generate-flasher-manifests.sh` to produce manifests + `projects.json`
3. Copy app binaries and supporting blobs into the firmware directory
4. Expose `projects.json` at the Pages root

### 1. Build all flasher-enabled firmware

Replace the two hard-coded `Build robocar-*` steps:

```yaml
- name: Build all flasher-enabled firmware
  run: |
    for flasher_json in packages/esp32-projects/*/flasher.json; do
      project_dir=$(dirname "$flasher_json")
      project_id=$(basename "$project_dir")
      chip_family=$(jq -r '.chipFamily' "$flasher_json")
      case "$chip_family" in
        "ESP32-S3") target="esp32s3" ;;
        "ESP32-C3") target="esp32c3" ;;
        "ESP32-C6") target="esp32c6" ;;
        *)          target="esp32"   ;;
      esac
      echo "Building $project_id (target=$target)..."
      cd "$project_dir" && idf.py -D IDF_TARGET="$target" build && cd -
    done
```

Projects with external dependencies (e.g. `xbox-switch-bridge`) still need
their `fetch_deps` step before building.

### 2. Generate manifests and projects.json

```yaml
- name: Get version from tag
  id: version
  run: |
    TAG="${{ github.event.release.tag_name || inputs.release_tag }}"
    echo "version=${TAG#v}" >> $GITHUB_OUTPUT

- name: Generate flasher manifests and projects.json
  run: bash tools/generate-flasher-manifests.sh "${{ steps.version.outputs.version }}" firmware
```

### 3. Copy binaries into firmware directory

```yaml
- name: Assemble firmware directory
  run: |
    for flasher_json in packages/esp32-projects/*/flasher.json; do
      project_dir=$(dirname "$flasher_json")
      project_id=$(basename "$project_dir")
      app_bin=$(jq -r '.appBinaryName' "$flasher_json")
      dest="firmware/$project_id"
      mkdir -p "$dest"
      cp "$project_dir/build/$app_bin"                            "$dest/" 2>/dev/null || true
      cp "$project_dir/build/bootloader/bootloader.bin"           "$dest/" 2>/dev/null || true
      cp "$project_dir/build/partition_table/partition-table.bin" "$dest/" 2>/dev/null || true
      cp "$project_dir/build/ota_data_initial.bin"                "$dest/" 2>/dev/null || true
    done
```

### 4. Expose projects.json at Pages root

In the `deploy-web-flasher` job, update the `Assemble Pages site` step:

```yaml
- name: Assemble Pages site
  run: |
    cp docs/flasher/* _site/
    # Expose projects.json at the root so index.html can fetch it
    cp _site/firmware/projects.json _site/projects.json 2>/dev/null || true
```

## flasher.json schema

Each opt-in project places a `flasher.json` in its project root:

```json
{
  "name":          "Human-readable project name",
  "description":   "One-sentence description",
  "chipFamily":    "ESP32 | ESP32-S2 | ESP32-S3 | ESP32-C3 | ESP32-C6",
  "board":         "Board name",
  "appBinaryName": "cmake-project-name.bin",
  "category":      "robocar | tools | games | standalone",
  "flashMode":     "uart | gpio0 | usb"
}
```

`flashMode` values:
- `uart`  — standard UART programmer, no special boot step
- `gpio0` — ESP32-CAM: hold GPIO0 LOW during reset
- `usb`   — ESP32-S3 native USB: hold BOOT during plug-in or reset

## Offset resolution

`tools/generate-flasher-manifests.sh` determines flash offsets automatically:

| Scenario | Bootloader | Partition table | App |
|----------|-----------|-----------------|-----|
| ESP32 / ESP32-S2, no `partitions.csv` | 0x1000 | 0x8000 | 0x10000 |
| ESP32-S3 / C3 / C6, no `partitions.csv` | 0x0 | 0x8000 | 0x10000 |
| Any family with `partitions.csv` | family default | 0x8000 | read from CSV |

OTA data offset is read from `partitions.csv` when present. If absent, no
`ota_data_initial.bin` part is added to the manifest.
