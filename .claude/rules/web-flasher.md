# Web Flasher Development Rules

## Overview

The web flasher (`docs/flasher/index.html`) enables browser-based firmware flashing via ESP Web Tools. It is deployed to GitHub Pages on every firmware release.

## Flash Offset Reference

Both robocar targets use identical partition layouts on standard ESP32 (4MB flash):

| Binary | Hex Offset | Decimal Offset | Build path |
|--------|-----------|----------------|------------|
| bootloader.bin | 0x1000 | 4096 | `build/bootloader/bootloader.bin` |
| partition-table.bin | 0x8000 | 32768 | `build/partition_table/partition-table.bin` |
| ota_data_initial.bin | 0xF000 | 61440 | `build/ota_data_initial.bin` |
| app binary | 0x12000 | 73728 | `build/<project-name>.bin` |

**These offsets are derived from `partitions.csv` and must stay in sync.** If the partition table changes, update the manifest generation in `build-firmware.yml`.

## ESP Web Tools Manifest Format

ESP Web Tools requires decimal integer offsets (not hex). The manifests are generated dynamically in CI, not checked into the repo. The `chipFamily` must be `"ESP32"` for both targets.

## Key Constraints

- **CORS**: Firmware binaries must be served from the same origin as the HTML page (GitHub Pages). Do not reference GitHub Releases URLs in the ESP Web Tools manifest.
- **ESP32-CAM boot mode**: GPIO0 must be held LOW during reset to enter flash mode. This must be documented on the flasher page.
- **Binary size limit**: App binaries must be under 1.8MB (1887436 bytes) to fit the OTA partition. This is enforced in CI.
- **No credentials in CI builds**: Pre-built firmware does not include WiFi credentials. Post-flash provisioning (Improv WiFi) is required.

## File Locations

- `docs/flasher/index.html` — Flasher web page (source, deployed to Pages root)
- `.github/workflows/build-firmware.yml` — Builds firmware, generates manifests, deploys Pages
- `packages/*/*/partitions.csv` — Source of truth for flash offsets

## Deployment Pipeline

1. Release is published (or `workflow_dispatch` triggered)
2. `build-firmware` job builds both firmwares, extracts all binaries, generates ESP Web Tools manifests
3. `deploy-web-flasher` job assembles `docs/flasher/*` + firmware artifacts into a Pages site and deploys

## When Modifying

- **Changing partition layout**: Update offsets in the manifest generation step of `build-firmware.yml`
- **Adding a new firmware target**: Add a new manifest, card, and `<esp-web-install-button>` in `index.html`
- **Changing the flasher page**: Edit `docs/flasher/index.html`; it deploys on next release
