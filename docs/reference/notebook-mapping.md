# NotebookLM Notebook Mapping

Canonical mapping of repo concerns to curated NotebookLM notebooks. This
file is the source of truth for:

- The scheduled monthly refresh reminder (`.github/workflows/notebook-refresh-reminder.yml`)
- The PR-triggered ADR/PRD reminder (`.github/workflows/notebook-doc-reminder.yml`)
- The `just notebooks-refresh` recipe

When you add a new ADR or PRD, update the matching notebook's sources in
the same PR (or within the next monthly refresh cycle).

## Active notebooks

| Notebook | ID | Primary repo touchpoints |
|---|---|---|
| AI vision backends for embedded (Claude / Ollama / Gemini) | `a755b5cd-22ec-4b5a-844e-ba7c1a258957` | ADR-003, ADR-008, ADR-016; PRD-002; `packages/robocar/camera/`, `packages/camera-vision/` |
| OTA updates & browser-based flashing | `c6868def-e128-4367-b70d-0efeb289d052` | ADR-004, ADR-012, ADR-015; `packages/components/ota-github/`, `docs/flasher/` |
| WiFi provisioning, mDNS & MQTT | `ada2b18e-6172-4fea-8176-36db2031e404` | ADR-010; `.claude/rules/mdns-hostname.md`; `packages/components/improv-wifi/` |
| USB gadget & HID on ESP32-S3 | `e91ebe8d-4b93-4321-9d7e-b97aa34956aa` | ADR-006, ADR-009; PRD-005; `packages/input-gaming/` |
| I2S audio & synthesis on ESP32 | `8cbdd8bb-e88e-4a26-a94f-d0126e68a09e` | ADR-011; PRD-008; `packages/audio/`, `packages/camera-vision/cam-i2s-audio/` |
| ESP32-S3 hardware families (XIAO Sense, Heltec, ESP32-CAM) | `3bb67ba2-58b4-4485-a306-6ab49ec38160` | `packages/robocar/main/`, `packages/robocar/unified/`, ESP32-CAM projects |
| Sensors & modules used in this lab | `c2d44a8c-cba1-436f-9c5c-0fe98f672909` | Module references used anywhere in `packages/` |
| ESP32 peripherals: I2C, UART, I2S, GPIO | `5601a768-acdb-4aeb-a2f0-455b3e493b1b` | ADR-007; `packages/components/i2c-protocol/` |
| ESP-IDF v5.4+ core reference (FreeRTOS, flashing, sdkconfig) | `f5af16f1-d307-4cc6-b7ab-39b7dbff0381` | `.claude/rules/esp-idf-sdkconfig.md`, `.claude/rules/containerized-builds.md` |
| ChameleonUltra: Advanced RFID and NFC Card Emulation Platform | `4831062f-84ab-4f67-89c0-61345941e3ea` | PRD-006; `packages/games/nfc-scavenger-hunt/` |
| Claude Code workflow & plugins | `ae5b0f8b-91fa-4e9e-9871-ee85d8ce24f9` | `CLAUDE.md`, `.claude/` |
| The Modern Soldering Equipment and Techniques Guide | `4e584364-0c55-408b-a7db-a69c177af71c` | (Lab hardware, independent of packages) |

## Path → notebook routing (for the PR reminder)

| Path pattern | Notebooks to consider updating |
|---|---|
| `docs/decisions/ADR-003-*`, `docs/decisions/ADR-008-*`, `docs/decisions/ADR-016-*`, `docs/requirements/PRD-002-*` | AI vision backends |
| `docs/decisions/ADR-004-*`, `docs/decisions/ADR-012-*`, `docs/decisions/ADR-015-*`, `packages/components/ota-github/**`, `docs/flasher/**` | OTA & web flasher |
| `docs/decisions/ADR-010-*`, `packages/components/improv-wifi/**`, `.claude/rules/mdns-hostname.md` | WiFi / mDNS / MQTT |
| `docs/decisions/ADR-006-*`, `docs/decisions/ADR-009-*`, `docs/requirements/PRD-005-*`, `packages/input-gaming/**` | USB / HID |
| `docs/decisions/ADR-011-*`, `docs/requirements/PRD-008-*`, `packages/audio/**` | I2S audio |
| `docs/decisions/ADR-007-*`, `packages/components/i2c-protocol/**` | ESP32 peripherals |
| `docs/requirements/PRD-006-*`, `packages/games/nfc-scavenger-hunt/**` | ChameleonUltra RFID/NFC |
| `CLAUDE.md`, `.claude/**` | Claude Code workflow |

## Maintenance cadence

- **Monthly**: regenerate the briefing-doc on each active notebook; note which answers changed materially; add primary references for any new ADR/PRD that landed.
- **Quarterly**: regenerate the mind-map and study-guide artifacts.
- **Per-PR**: when a PR touches `docs/decisions/` or `docs/requirements/`, update the matching notebook's sources.

## Conventions

- **Source cap**: ~30 per notebook. When adding, consider dropping a stale one.
- **No duplicates**: dedup by `(title, url)` — `notebooklm source list --notebook <id> --json` + manual pass.
- **Local-only auth**: `notebooklm` CLI needs `notebooklm login` on the machine running the refresh. It cannot run in CI.
