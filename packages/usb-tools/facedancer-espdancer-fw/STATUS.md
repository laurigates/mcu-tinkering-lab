# Status — facedancer-espdancer-fw

> A narrative snapshot of where the project stands, meant to be read once to
> get oriented. For the task-level backlog see [`CHECKLIST.md`](CHECKLIST.md);
> for design see [`docs/architecture.md`](docs/architecture.md),
> [`docs/raw-usb-design.md`](docs/raw-usb-design.md), and the
> [`docs/adrs/`](docs/adrs/); for the wire contract see
> [`docs/rpc-protocol.md`](docs/rpc-protocol.md).
>
> **Last updated:** after commit `2389ad8` (PR #415) — `just build-debug-usb` is
> green; no USB emulation exercised yet.

## One-paragraph summary

`facedancer-espdancer-fw` is ESP32-S3 firmware that turns the board into a
**raw USB device relay** for the facedancer `espdancer` backend. The
controlling PC decides what USB device the S3 enumerates as — descriptors,
endpoint topology, and every transfer come from the host over a WiFi SoftAP
control channel; the firmware holds no device-class knowledge and just relays
USB primitives between that RPC and the S3's DWC2 OTG-FS controller. The
project is scaffolded from `xbox-switch-bridge` (same board, same sdkconfig
knobs, same `status_led`/`log_udp` components), with all BLE/controller logic
dropped and a raw-DCD USB path added in its place.

## Milestone status

| M  | Title                | Status                                                                                                        |
|----|----------------------|---------------------------------------------------------------------------------------------------------------|
| 0  | Scaffold             | ✅ Done. Boots, advertises `espdancer-log` SoftAP, TCP 4444 control channel answers `HELLO`/`GET_VERSION`.   |
| 1  | Raw USB relay        | 🟡 In progress. **Vendored DCD-only TinyUSB port compiles & links** (`debug-usb` build green); no USB I/O yet. |
| 2  | RPC wiring           | ⬜ Queued. Dispatch table → `raw_usb_*`; EVENT frames to host; backpressure.                                   |
| 3  | Python backend       | ⬜ Queued. Lands in `external/facedancer/facedancer/backends/espdancer.py`.                                     |
| 4  | Hardening + tests     | ⬜ Queued. Endpoint-limit errors, framing unit tests, Linux/macOS smoke.                                       |

**Current proof point:** `just build-debug-usb` produces `facedancer-espdancer-fw.bin`
(63% app-partition free). That's a *compile/link* proof of the ADR-0006 thesis
(the bypass of TinyUSB's `usbd` is feasible); the *runtime* proof — `DCD_EVENT_BUS_RESET`
firing on a host cable-plug — is the next milestone item and has not been
exercised yet.

## What got fixed to make the build green (commit #415)

Three bugs stood between the scaffold and a green `debug-usb` build:

1. **`CMakeLists.txt` — `CMake_CURRENT_SOURCE_DIR` case typo.** CMake variables
   are case-sensitive; the lowercase spelling made `file(READ)` resolve to
   `/version.txt` and fail, but the `if(EXISTS)` guard used the correct
   `CMAKE_*` form so it silently fell through. Fixed at the source.

2. **`main/main.c` — `ESP_LOGI` with a ternary as the format string.**
   `ESP_LOGI(TAG, cond ? "A" : "B")` is a non-literal format arg; the log macro
   string-concatenates the literal at macro-expansion time, so a non-literal
   breaks it. Rewritten as `ESP_LOGI(TAG, "Host %s", cond ? "connected" :
   "disconnected")`. (Introduced in the original scaffold; good catch.)

3. **`components/raw_usb/raw_usb.c` — `usbd_spin_lock` / `usbd_spin_unlock` /
   `usbd_int_set`.** The vendored `dcd_dwc2.c` guards its register access with
   `usbd_spin_lock`/`usbd_spin_unlock`, which normally live in `usbd.c` — the
   file we deliberately exclude (ADR-0006). Supplied in `raw_usb.c` as a
   faithful port of TinyUSB 0.19.0: an `OSAL_SPINLOCK_DEF(_usbd_spin,
   usbd_int_set)` whose `usbd_int_set` toggles `dcd_int_enable`/`dcd_int_disable`
   on our single rhport; the lock/unlock delegate to `osal_spin_lock`/`_unlock`
   from `osal_none.h`. **This is exactly the risk ADR-0006 flagged** — *"we
   must supply an osal unless the DCD uses an OSAL primitive (spike: confirm
   during the port compile)."* Confirmed at compile time: it does, and not just
   an OSAL primitive but a `usbd.c` symbol.

### Standing maintenance obligation (from fix #3)

Any future refresh of the vendored TinyUSB DCD must re-verify that
`usbd_spin_lock`, `usbd_spin_unlock`, and `usbd_int_set` are **still defined
in `raw_usb.c`** and **still match the upstream signatures**. The vendored
`dcd_dwc2.c` will link-error on these three symbols if the port code expects
them and we haven't supplied them. Recorded in ADR-0006's "Risk → Resolved"
section so it isn't rediscovered the hard way on a TinyUSB version bump.

## Architecture (in one screen)

```
┌──────────────┐  WiFi SoftAP   ┌──────────────┐    USB-C      ┌──────────────┐
│ Controlling  │ ──────────────▶│  ESP32-S3    │──────────────▶│   Target     │
│   PC         │  TCP 4444      │  espdancer   │  Full-Speed   │   host       │
│ espdancer.py │  framed TLV    │  raw relay   │  enumerated   │  (sees the   │
└──────────────┘                └──────────────┘               └──────────────┘
   control plane                    |   data plane
                                  (single OTG port)
```

Two planes meeting at the relay pump:

- **Control plane** — `components/usb_rpc` (TCP 4444 + framed-TLV) +
  `components/log_udp` (SoftAP + UDP log sink). Carries `CONNECT`/`SEND`/`STALL`/…
  from the host and `EVENT_*` (BUS_RESET, SETUP, OUT_PACKET, SEND_COMPLETE, NAK)
  back.
- **Data plane** — `components/raw_usb` driving the DWC2 OTG-FS controller via
  the **vendored DCD-only TinyUSB port** (`components/raw_usb/tinyusb_port/`,
  ADR-0006). We bypass TinyUSB's `usbd` (which auto-handles all USB standard
  requests) and provide our own `dcd_event_handler`, so the *host PC* drives
  device behavior — the whole point of Facedancer.

Core split: `usb_rpc` on core 0, the `raw_usb` relay pump on core 1. The DCD
ISR only enqueues events (single-writer discipline, ADR-0005); the pump is the
sole writer to the DCD.

## Hard constraints (from the hardware)

- **Full Speed only** — internal OTG-FS PHY, 12 Mbps. `raw_usb_connect()`
  rejects/downgrades `HIGH`/`LOW`.
- **`DWC2_EP_MAX = 7` incl. EP0 → 6 non-control endpoints max.** Surfaced as a
  clear error from `espdancer.py` (M3), mirroring the MAX3421 caveat.
- **Single OTG port ⇒ no USBProxy/MITM** — the proxy mode needs the Facedancer
  to act as host *and* device simultaneously; one port can't.
- **USB-Serial-JTAG ↔ USB-OTG share one PHY** — exclusive. Debug builds use
  WiFi UDP or an external CP2102 UART (GPIO43/44) for logging.

## What's vendored, and from where

`components/raw_usb/tinyusb_port/` carries a **DCD-only** slice of TinyUSB
**0.19.0** (the version `espressif/esp_tinyusb` ≥1.4.0 pulls under ESP-IDF
v5.4) — 21 files, MIT license in `LICENSE_TINYUSB.txt`:

- `src/portable/synopsys/dwc2/` — `dcd_dwc2.c`, `dwc2_common.c`, `dwc2_*.h` (×10)
- `src/common/` — `tusb_fifo.c/h`, `tusb_types/compiler/verify/debug/private/common.h`, `tusb_mcu.h`
- `src/device/` — `dcd.h`, `usbd_pvt.h` (header only; **`usbd.c` NOT vendored**)
- `src/osal/` — `osal.h`, `osal_none.h`
- `src/tusb_option.h`

Tree preserved so `#include "common/..."`, `#include "device/..."`,
`#include "osal/..."`, and sibling `#include "dwc2_common.h"` all resolve through
the `src` include dir. **Pinned to 0.19.0** — upstream fixes don't flow
automatically; bump deliberately and re-test (and re-verify the
`usbd_spin_*`/`usbd_int_set` shim above).

The `espressif/esp_tinyusb` managed component is **NOT** pulled
(`main/idf_component.yml` has `dependencies: {}`): pulling it would compile
`usbd.c` and strongly define `dcd_event_handler`, colliding with ours.

## Build / flash

```bash
just build-debug-usb     # green as of #415; raw DCD owns PHY, logs over WiFi UDP
just flash               # auto-detects the S3 USB-Serial-JTAG port
just log-listen          # join 'espdancer-log' WiFi first, then UDP 4444
```

Variants (`rm sdkconfig` before switching):
`build` (prod), `build-debug-uart` (CP2102 logging), `build-debug-jtag`
(no USB emu), `build-debug-usb` (verbose WiFi logs). See ADR-0004.

## Next actionable (M1, application-level — no more port archaeology)

1. Flash `debug-usb`, plug the S3 into a host, confirm the `EVT bus_reset`
   log line over WiFi UDP — the runtime proof the port works.
2. Runtime descriptor table + `raw_usb_connect` → S3 enumerates on a Linux/macOS
   host as a **built-in trivial HID** (driven in-firmware, not host-supplied
   yet).
3. `raw_usb_configure_ep` (`dcd_edpt_open`) + `raw_usb_send` (`dcd_edpt_xfer`)
   → round-trip a HID input report the host reads.
4. Control-transfer stages: `ack_status` (ZLP), `stall`/`clear_halt`,
   `set_address` (+ `MANUAL_SET_ADDRESS` quirk; mirror moondancer `QuirkFlag`).

M1 exit gate: built-in trivial-HID descriptor lets the S3 enumerate and a
`raw_usb_send` pumps an input report the host PC reads. Host-supplied
descriptors + full EVENT→RPC wiring land in M2.

## Provenance

Scaffolded from `packages/input-gaming/xbox-switch-bridge` — same Waveshare
ESP32-S3-Zero target, same `sdkconfig.defaults` WiFi coexistence knobs (AMPDU
off, NVS off, HT20, country FI), same `status_led` (RMT non-DMA + 48
symbols/block workaround vs the TinyUSB DMA conflict), same `log_udp` SoftAP +
UDP log sink. BLE/controller logic dropped; raw-DCD USB path added in its
place. The single-OTG-port constraint, the RMT↔TinyUSB DMA workaround, and the
SoftAP invisibility fixes were all pre-solved by that project.

## Reference index

- [`CHECKLIST.md`](CHECKLIST.md) — milestone-gated task backlog (source of truth for what's done)
- [`CLAUDE.md`](CLAUDE.md) — working-here guide for Claude Code
- [`docs/architecture.md`](docs/architecture.md) — system-level design
- [`docs/raw-usb-design.md`](docs/raw-usb-design.md) — M1 component design
- [`docs/rpc-protocol.md`](docs/rpc-protocol.md) — host↔device wire contract
- [`docs/adrs/`](docs/adrs/) — ADRs 0001–0006 (0006 resolves 0002's open
  question; carries the `usbd_spin_*` maintenance obligation)
