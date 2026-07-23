# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with
code in this repository.

## Project Overview

**facedancer-espdancer-fw** is an ESP32-S3 firmware that turns the board into
a *raw USB relay* for the [facedancer](https://github.com/greatscottgadgets/facedancer)
`espdancer` backend. The S3 enumerates as **whatever device the controlling PC
tells it to** — the host supplies the full descriptor set and endpoint topology
over a WiFi control channel; the firmware just relays USB primitives.

**Current status**: Milestone 0 done → Milestone 1 in progress. M0 produces a
flashable binary that boots, advertises the `espdancer-log` SoftAP, and answers
`HELLO` / `GET_VERSION` on TCP 4444 (no USB emulation). M1 vendors a DCD-only
TinyUSB port and stands up `components/raw_usb` — see `docs/raw-usb-design.md`.
Design is fully documented in `docs/architecture.md`, `docs/raw-usb-design.md`,
and ADRs 0001–0006; the open question from ADR-0002 is **resolved** by ADR-0006.

**Target hardware:** Waveshare ESP32-S3-Zero (same as xbox-switch-bridge; any
ESP32-S3 with native USB-OTG works).

## Relationship to other projects

- **xbox-switch-bridge** (`packages/input-gaming/xbox-switch-bridge`) —
  provenance; this scaffold lifts its `sdkconfig.defaults`, build-variant
  system, `components/status_led`, `components/log_udp`, justfile/Makefile, and
  the single-writer pattern, dropping all BLE/controller logic.
- **external/facedancer** — upstream facedancer clone; the Python backend
  `facedancer/backends/espdancer.py` (Milestone 3) lands there, not here.

## Build Commands

Builds run in Docker (`mcu-tinkering-lab/esp-idf:v5.4` via the shared
`docker-compose.yml`); flash/monitor run natively.

```bash
just build              # Production build (validated output; checks .bin exists)
just build-verbose      # Plain idf.py build (no filtering)
just build-debug-uart   # TinyUSB ON + UART logging (GPIO43/44 + CP2102)
just build-debug-jtag   # TinyUSB OFF + USB-Serial-JTAG logging (no USB emu)
just build-debug-usb   # TinyUSB ON + verbose WiFi UDP logging
just flash              # Flash to ESP32-S3 (auto-detects USB-Serial-JTAG port)
just monitor           # Serial monitor via USB-Serial-JTAG (debug-jtag build)
just log-listen         # Listen for UDP log broadcast (join 'espdancer-log' WiFi)
just menuconfig         # Interactive sdkconfig in Docker
just clean              # Remove build artifacts
just info               # Show project + port info
```

Override USB port: `just flash PORT=/dev/cu.usbmodem1101`.

**Switching variants:** delete the generated `sdkconfig` cache first:
`rm sdkconfig && just build-debug-uart` (failing to do so silently keeps the
old variant's values).

## Architecture

### Data flow (target, post-Milestone 1)

```
Controlling PC ──WiFi SoftAP──▶ ESP32-S3 ──USB-C──▶ Target host
  (espdancer.py)    TCP 4444       raw_usb relay      (enumerates as the
                     framed TLV     (DCD-level)         emulated device)
```

### Core split

- **Core 0**: `usb_rpc` control-channel task — TCP server, frame parser,
  dispatch table. Calls `raw_usb_*` primitives (Milestone 1+) or returns
  `ERROR_NOT_IMPLEMENTED` (Milestone 0).
- **Core 1**: `raw_usb` relay pump — single writer to TinyUSB DCD endpoints
  (ADR-0005). Drains the host→device command queue; frames device→host events.

### Components

| Component | Purpose | Status |
|-----------|---------|--------|
| `status_led` | WS2812 RMT (non-DMA) — emulator-state indicator | Lifted |
| `log_udp` | WiFi SoftAP "espdancer-log" + UDP log broadcast | Lifted (SSID renamed) |
| `usb_rpc` | TCP server + framed-TLV dispatcher (this firmware's host iface) | New (stub) |
| `raw_usb` | Own `dcd_event_handler` + relay pump + DCD primitives (M1 skeleton;
  DCD calls stubbed behind `RAW_USB_DCD_VENDORED`) | **In progress** |
| `raw_usb/tinyusb_port` | DCD-only vendored TinyUSB port (ADR-0006; vendoring pending) | TODO |

## Wire protocol

Authoritative spec: `docs/rpc-protocol.md`. The Python `espdancer.py` backend
must implement it exactly; `components/usb_rpc/include/usb_rpc.h` mirrors the
message IDs. Brief: framed `[0xF0 0xD0][u16 len_be][msg_id][payload][crc8]` over
TCP 4444 on the SoftAP.

## Hardware constraints

**USB PHY sharing (critical):** USB-Serial-JTAG and USB-OTG share one internal
PHY on the S3. Once TinyUSB takes the PHY, JTAG disconnects entirely. Use build
variants to choose which owns it (ADR-0004).

**Full Speed only** (internal OTG-FS PHY, 12 Mbps). High Speed needs an external
ULPI PHY — no common devkit has it. `raw_usb_connect()` must reject/downgrade
`DeviceSpeed.HIGH`/`LOW` (clone greatdancer's wording).

**Single OTG port ⇒ no USBProxy/MITM.** Facedancer's proxy needs the Facedancer
to act as *host* and *device* simultaneously; one port can't. Same limitation
MAX3421 boards carry; document it when adding to facedancer's README.

**RMT + TinyUSB DMA conflict:** `with_dma = true` on the RMT channel breaks
after TinyUSB init. `status_led` uses `with_dma = false` +
`mem_block_symbols = 48` (S3 has 48 symbols/block, not 64).

## Documentation

- `CHECKLIST.md` — working backlog, milestone-gated (start here)
- `docs/adrs/` — ADRs 0001–0005 (target, DCD-not-HID, WiFi control channel,
  build variants, single-writer pump)
- `docs/rpc-protocol.md` — host↔device wire contract

## Shared tooling

The `justfile` imports `../../../tools/esp32-idf.just` (→ `tools/esp32.just`),
which provides containerized `_idf-build`/`_idf-build-checked`/`_idf-clean`,
native `_s3-flash`/`_s3-reset`/`_s3-monitor`, and S3 port auto-detection. Do
not duplicate that logic here; compose the shared recipes.

## Dependencies

- `espressif/led_strip` ≥2.0.0 — WS2812 RMT driver (managed; declared in
  `components/status_led/idf_component.yml`).
- `espressif/esp_tinyusb` ≥1.4.0 — is the *source* (TinyUSB 0.19.0) of the
  vendored DCD files but is **NOT** pulled as a managed dependency (pulling it
  would compile its `usbd.c`, strongly defining `dcd_event_handler` and
  colliding with `raw_usb.c`). The DCD-only port lives in
  `components/raw_usb/tinyusb_port/` (ADR-0006).
- ESP-IDF core: `nvs_flash`, `esp_wifi`, `lwip`, `freertos`, `esp_timer`.