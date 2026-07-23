# Architecture

This document is the system-level design overview for `facedancer-espdancer-fw`.
It ties together the ADRs and the wire protocol into one picture. Individual
decisions live in `docs/adrs/`; the host↔device contract lives in
`docs/rpc-protocol.md`; the working backlog in `CHECKLIST.md`.

## What it is

An ESP32-S3 firmware that turns the board into a **raw USB device relay** for
the [facedancer](https://github.com/greatscottgadgets/facedancer) `espdancer`
backend. The controlling PC (running `espdancer.py`) decides what USB device
the S3 enumerates as — it supplies the descriptors and endpoint topology and
drives every USB transfer. The S3 holds **no device-class knowledge**: it is a
dumb pipe between the host's RPC and the S3's USB OTG-FS PHY.

```
┌──────────────┐  WiFi SoftAP   ┌──────────────┐    USB-C      ┌──────────────┐
│ Controlling  │ ──────────────▶│  ESP32-S3    │──────────────▶│   Target     │
│   PC         │  TCP 4444      │  espdancer   │  Full-Speed   │   host       │
│  espdancer.py│  framed TLV    │  raw relay   │  enumerated   │  (sees the   │
└──────────────┘                └──────────────┘               └──────────────┘
                                  control plane   |   data plane
                                                    (a single OTG port)
```

## Two planes

The firmware has two independent planes that meet at the relay pump:

- **Control plane** (`components/usb_rpc` + `components/log_udp`) — TCP server
  on the `espdancer-log` SoftAP. Carries framed-TLV RPC (see
  `docs/rpc-protocol.md`): the host issues `CONNECT`/`CONFIGURE_EP`/`SEND`/
  `STALL`/… and the device replies `OK`/`ERROR` and emits unsolicited
  `EVENT_*` (BUS_RESET, SETUP, OUT_PACKET, SEND_COMPLETE, NAK).
- **Data plane** (`components/raw_usb`) — the actual USB endpoint traffic,
  driven through the Synopsys DWC2 OTG-FS controller. The relay pump is the
  sole writer to the DCD (ADR-0005).

## Core split

| Core | Responsibility | Priority |
|------|---------------|----------|
| 0 | `usb_rpc` TCP server: frame parse → dispatch → call `raw_usb_*` (or queue a work item for the pump) | 4 |
| 1 | `raw_usb` relay pump: sole writer to the DCD; drains the work queue; frames USB events back over `usb_rpc_send_event()` | 4–5 (TinyUSB ISR co-ranks) |

The USB interrupt fires `dcd_int_handler` (in the DWC2 DCD), which calls our
`dcd_event_handler` (in ISR); that handler **only enqueues** an event onto a
FreeRTOS queue — it never sends on an endpoint (ADR-0005). The pump task on
core 1 drains that queue and, for SETUP/OUT packets, relays them as
`EVENT_SETUP`/`EVENT_OUT_PACKET` to the host over the control plane.

## Why we bypass TinyUSB's `usbd`

TinyUSB's `usbd` (the device stack that normally sits between the DCD driver
and the application) **auto-handles every USB standard request** —
SET_ADDRESS, SET_CONFIGURATION, GET_DESCRIPTOR, GET/SET_INTERFACE, GET_STATUS,
SET/CLEAR_FEATURE — in a *static* `process_control_request()`, and strongly
defines `dcd_event_handler`. Facedancer's whole point is that the *host PC*
drives this, including deliberately-misbehaving devices (NAK'd SET_CONFIG,
malformed descriptors) for emulation/fuzzing. Letting `usbd` auto-respond
breaks that and double-handles transfers.

So `raw_usb` is its **own minimal device stack**: it vendors only the TinyUSB
**DCD layer** (`dcd_dwc2.c` + `dwc2_common.c` + `tusb_fifo.c` + headers +
`osal_none` stub) into `components/raw_usb/tinyusb_port/`, and provides
`dcd_event_handler` itself. Decision and trade-offs: ADR-0006 (resolves the
open question flagged in ADR-0002).

## The raw_usb primitive surface

`raw_usb` exposes exactly the operations the `FacedancerBackend` interface
implies, sized to the hardware (ADR-0007 / `docs/raw-usb-design.md`):

| `FacedancerBackend` (Python) | `raw_usb_*` (firmware) | Maps to |
|-------------------------------|------------------------|---------|
| `connect(speed, ep0_max, quirks)` | `raw_usb_connect` | set descriptor table, `dcd_init` + `dcd_connect` |
| `disconnect()` | `raw_usb_disconnect` | `dcd_disconnect` |
| `reset()` | `raw_usb_reset` | (re-arm controller) |
| `configured(configuration)` | `raw_usb_configure_ep` ×N | `dcd_edpt_open` per endpoint |
| `send_on_endpoint(ep, data)` | `raw_usb_send` | `dcd_edpt_xfer` (IN) |
| `read_from_endpoint(ep)` | (host receives via `EVENT_OUT_PACKET`) | `dcd_edpt_xfer` (OUT, primed) |
| `stall_endpoint(ep, dir)` | `raw_usb_stall` | `dcd_edpt_stall` |
| `clear_halt(ep, dir)` | `raw_usb_clear_halt` | `dcd_edpt_clear_stall` |
| `ack_status_stage(dir, ep)` | `raw_usb_ack_status` | ZLP xfer on EP0 opposite dir |
| `set_address(addr, defer)` | `raw_usb_set_address` | `dcd_set_address` (+ `manual_set_address` quirk) |

Descriptor set and endpoint topology arrive as a `CONNECT` payload (device +
config + string descriptors) and are placed into a **runtime, RAM-backed
descriptor table** the DCD serves GET_DESCRIPTOR from; the host otherwise
**handles all standard requests itself** by responding `SEND`/`STALL`/
`ACK_STATUS` to the `EVENT_SETUP` it receives.

## Hard constraints (from the hardware)

- **Full Speed only** — internal OTG-FS PHY, 12 Mbps. `raw_usb_connect()` must
  reject/downgrade `DeviceSpeed.HIGH`/`LOW` (clone greatdancer's wording).
- **`DWC2_EP_MAX = 7` endpoints incl. EP0** — so 6 non-control endpoints max.
  `espdancer.py` (M3) surfaces a clear error for oversized configs, mirroring
  the MAX3421 caveat in facedancer's README.
- **Single OTG port ⇒ no USBProxy/MITM** — the proxy mode needs the Facedancer
  to act as USB *host* and *device* simultaneously; one port can't. Same
  limitation the MAX3421 boards carry.
- **USB-Serial-JTAG ↔ USB-OTG share one PHY** — exclusive. Debug builds use
  WiFi UDP or an external CP2102 UART (GPIO43/44) for logging (ADR-0004).
- **RMT ↔ TinyUSB DMA conflict** — the WS2812 `status_led` uses
  `with_dma=false` + `mem_block_symbols=48`.

## Milestone progression

1. **M0 — Scaffold** ✅ boots, SoftAP, control channel answers HELLO. No USB.
2. **M1 — Raw USB relay** ⬅ current. Vendor the DCD-only TinyUSB port; stand
   up `raw_usb`; host can `CONNECT` with a trivial descriptor and see the S3
   enumerate + round-trip a report.
3. **M2 — RPC wiring** — dispatch every primitive; backpressure; resync.
4. **M3 — Python backend** in `external/facedancer/facedancer/backends/espdancer.py`.
5. **M4 — Hardening + tests.**

Out of scope (see CHECKLIST "Out of scope"): High-Speed (ULPI), USBProxy/MITM,
on-device MicroPython.

## References

- `docs/adrs/` — ADRs 0001–0006
- `docs/rpc-protocol.md` — wire contract
- `docs/raw-usb-design.md` — M1 component design (primitive surface, endpoint
  table, control-transfer state machine, event mapping)
- `CHECKLIST.md` — milestone-gated working backlog