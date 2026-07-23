# ADR-0002: Raw DCD Relay, Not the TinyUSB HID Class Driver

**Status**: Accepted
**Date**: 2026-07-22
**Confidence**: High

## Context

Both provenance projects (xbox-switch-bridge, switch-usb-proxy) emulate one
**fixed** device — the Nintendo Switch Pro Controller — using TinyUSB's HID
class driver: a compile-time `tusb_desc_device_t`, one `TUD_HID_INOUT`
descriptor, a static HID report descriptor, and `tud_hid_*` callbacks.

Facedancer's backend contract (`facedancer/backends/base.py`) is the opposite:
the **host PC supplies the entire descriptor set and endpoint topology at
runtime**, and the firmware must relay arbitrary control transfers, endpoint
data, stalls, and events back to the host Python library. Endpoint count,
type, max-packet-size, and alt-settings are all host-chosen per device being
emulated.

## Decision

Implement `components/raw_usb` against TinyUSB's **DCD layer**
(`dcd_edpt_open` / `dcd_edpt_xfer` / `dcd_edpt_stall` and the device-event
hooks), **not** via `tinyusb_driver_install()` + class drivers. Hold the
device + config + string descriptors in a **runtime, RAM-backed descriptor
table** that the host populates over the control channel at `CONNECT` time
(`raw_usb_connect()`).

The `usb_rpc` dispatch table maps each `FacedancerBackend` callback to a
`raw_usb_*` primitive; the firmware holds no device-class knowledge.

## Consequences

- The firmware is a *dumb raw-USB relay*; the host PC (via `espdancer.py`)
  owns all device semantics. Replay/swap costs a `CONNECT` frame, not a
  reflash.
- Endpoint count/type bounded by the OTG-FS block (~6–8 in the S3 TinyUSB
  config); `espdancer.py` must surface a clear error for oversized configs
  (mirror the MAX3421 caveat in facedancer README).
- ZLP, status-stage, `ack_status_stage`, `manual_set_address` quirks must all
  be handled in the relay — `tud_hid_*` helpers don't apply.

## Alternatives Considered

- **Stay on the HID class driver, emulate only HID devices.** Would cover HID
  fuzzing cheaply but not mass storage, CDC, or composite devices — far short
  of what a Facedancer backend promises.
- **Bypass TinyUSB entirely, drive OTG-FS registers directly.** More control,
  more bugs. Use TinyUSB's DCD port until we hit a wall.

## Open question → Resolved (see ADR-0006)

Investigation of the `espressif__tinyusb` 0.19 managed component showed that
`src/device/usbd.c` strongly defines `dcd_event_handler` and that its static
`process_control_request` auto-handles all USB standard requests — an
override-free, non-negotiable blocker for Facedancer's host-driven model. We
therefore **bypass `usbd` entirely** and vendor a DCD-only TinyUSB port
(`dcd_dwc2.c` + `dwc2_common.c` + `tusb_fifo.c` + headers) into
`components/raw_usb/tinyusb_port/`, providing `dcd_event_handler` ourselves.
Decision, de-risk findings, and alternatives: ADR-0006.