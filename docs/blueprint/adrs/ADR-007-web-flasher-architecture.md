# ADR-007: Browser-Based Web Flasher Architecture

## Status

Accepted

## Context

The robocar and other ESP32 projects in this monorepo require ESP-IDF, a USB cable, and command-line tooling to flash firmware. This creates a high barrier to entry for:

- Users who want to try the firmware without setting up a development environment
- Quick re-flashing during demos or workshops
- Android phone-based flashing via USB OTG in the field
- Contributors who want to test firmware on hardware without building locally

The project already has a CI pipeline (`build-firmware.yml`) that builds firmware binaries and uploads them to GitHub Releases on every release. The OTA system (ADR-004) handles updates for devices that are already running firmware, but there is no solution for initial flashing without a toolchain.

Requirements:
- Flash firmware from a browser with zero local toolchain setup
- Support both robocar boards (Heltec WiFi LoRa 32 V1 and ESP32-CAM)
- Work on desktop Chrome/Edge and Android Chrome with USB OTG
- Integrate with the existing release CI pipeline
- Eventually support all ESP32 projects in the monorepo via opt-in

## Decision

### ESP Web Tools for browser-based flashing

We use [ESP Web Tools](https://esphome.github.io/esp-web-tools/), an open-source web component by the ESPHome project that leverages the Web Serial API to flash ESP32 devices directly from a browser. It handles serial connection, chip detection, and binary writing without any browser extensions or native software.

### GitHub Pages for hosting

The flasher page and firmware binaries are hosted on GitHub Pages (same-origin), deployed via GitHub Actions (`actions/deploy-pages`). This avoids CORS issues that would occur if the HTML page on `*.github.io` tried to fetch binaries from `github.com/releases/download/` (different origin, redirect through `objects.githubusercontent.com`).

### Integrated into the release pipeline

Rather than a separate workflow, the web flasher deployment is a job (`deploy-web-flasher`) within the existing `build-firmware.yml` workflow. This ensures:
- The flasher always serves the same version that was just released
- No extra workflow to maintain or coordinate
- Firmware binaries flow directly from the build job to the Pages deployment via artifacts

### Full flash (not OTA-only)

The web flasher performs a complete flash including bootloader, partition table, OTA data, and application binary. This is necessary because:
- First-time users have no existing firmware to OTA from
- The partition layout must match the application binary
- OTA data initialization ensures the bootloader selects the correct partition

Flash offsets are derived from `partitions.csv` (identical for both boards):

| Binary | Offset | Source |
|--------|--------|--------|
| bootloader.bin | 0x1000 (4096) | `build/bootloader/bootloader.bin` |
| partition-table.bin | 0x8000 (32768) | `build/partition_table/partition-table.bin` |
| ota_data_initial.bin | 0xF000 (61440) | `build/ota_data_initial.bin` |
| app binary | 0x12000 (73728) | `build/<project>.bin` |

These are standard ESP32 offsets. ESP32-S3/C3/C6 projects would use 0x0 for the bootloader instead of 0x1000.

### Static HTML with no build step

The flasher page (`docs/flasher/index.html`) is plain HTML + CSS + the ESP Web Tools web component loaded from unpkg CDN (pinned to `@10`). No JavaScript framework, no bundler, no build step. The page renders board selection cards and delegates all flashing logic to ESP Web Tools.

### WiFi provisioning via Improv WiFi (planned)

Pre-built firmware from CI does not include WiFi credentials (they are compiled in via `credentials.h` for local builds). The planned solution is [Improv WiFi](https://www.improv-wifi.com/), which ESP Web Tools natively supports — after flashing, the UI automatically presents a WiFi configuration dialog if the firmware advertises Improv support. This is tracked in #151.

### Dynamic project discovery (planned)

Currently the flasher hardcodes two boards. The planned approach (#152) uses a per-project `flasher.json` opt-in file that CI discovers at build time, generating a `projects.json` index that the flasher page loads dynamically. This allows any project in the monorepo to join the web flasher by adding a single metadata file.

## Alternatives Considered

### 1. Host binaries on GitHub Releases, page on GitHub Pages

Simpler CI (no need to copy binaries to Pages), but GitHub Releases URLs redirect through `objects.githubusercontent.com` which historically causes CORS failures when fetched via the Fetch API from a different origin. Rejected for reliability.

### 2. Separate `gh-pages` branch

Traditional GitHub Pages approach using an orphan branch. Rejected because:
- Force-pushing generated content to a branch is messy in a monorepo
- Pollutes git history with binary commits
- Actions-based deployment (`actions/deploy-pages`) is idempotent and cleaner

### 3. Separate workflow for Pages deployment

A dedicated `deploy-flasher.yml` workflow. Rejected because:
- Would need to download release assets or share artifacts across workflows
- Adds coordination complexity
- The flasher deployment is inherently tied to firmware releases

### 4. ESPHome Web dashboard

Using the full ESPHome dashboard for flashing. Rejected because:
- Our projects use ESP-IDF, not ESPHome
- The ESPHome dashboard is designed for ESPHome configs, not arbitrary binaries
- ESP Web Tools (the underlying component) is the right layer of abstraction

### 5. Tasmota Web Installer pattern

Tasmota's web installer uses a similar approach but with a more complex frontend. We chose the simpler ESP Web Tools component directly since we don't need Tasmota's multi-variant selection UI.

## Consequences

### Positive
- Zero-setup firmware flashing for end users
- Works on Android with USB OTG — useful for field deployment
- Automatically updated on every release with no manual steps
- Foundation for supporting all monorepo projects via dynamic discovery
- Same-origin hosting eliminates CORS issues

### Negative
- GitHub Pages deployment adds ~30 seconds to release pipeline
- Firmware binaries are duplicated (GitHub Releases + GitHub Pages)
- Web Serial API only works in Chrome/Edge (no Firefox/Safari support)
- Pre-built firmware lacks WiFi credentials until Improv WiFi is implemented (#151)

### Risks
- `unpkg.com` CDN dependency for ESP Web Tools JS — could pin and self-host if needed
- Web Serial API is not a W3C standard yet (though Chrome/Edge support is stable)
- ESP32-CAM requires manual GPIO0 boot mode entry, which is error-prone for inexperienced users

## References

- ESP Web Tools: https://esphome.github.io/esp-web-tools/
- Improv WiFi: https://www.improv-wifi.com/
- Web Serial API: https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API
- ADR-004: OTA Update Architecture
- #150: Enable GitHub Pages
- #151: Improv WiFi provisioning
- #152: Dynamic project discovery
