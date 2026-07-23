# ADR-0006: Vendor a DCD-Only TinyUSB Port (Bypass `usbd`)

**Status**: Accepted
**Date**: 2026-07-22
**Confidence**: High
**Resolves**: the open question in ADR-0002.

## Context

ADR-0002 chose to drive TinyUSB at the **DCD layer** rather than its HID class
driver, but left open *how*: whether ESP-IDF's bundled `esp_tinyusb` exposes
`dcd_*` cleanly from app code, or whether we need a local port.

Investigation of the `espressif__tinyusb` 0.19 managed component (the version
ESP-IDF v5.4 pulls) gives a hard answer:

- **`dcd_event_handler` is a strong symbol** in `src/device/usbd.c` (line 1227).
  Linking `usbd.c` means accepting *its* event processing — we cannot supply
  our own handler without a duplicate-symbol error.
- **`process_control_request` is `static`** in `usbd.c`. It auto-responds to
  *every* USB standard request — SET_ADDRESS (line 816), SET_CONFIGURATION
  (832), GET_DESCRIPTOR (875), GET/SET_INTERFACE (948–955), GET_STATUS
  (918/983), SET/CLEAR_FEATURE (879/989). There is no documented override.
- The managed component's `CMakeLists.txt` compiles a **fixed** `srcs` list
  that always includes `src/device/usbd.c` plus every class driver (CDC, HID,
  MSC, vendor, audio, video…). There is no Kconfig to drop `usbd.c`.

For Facedancer, where the **host PC** drives device behavior — including
deliberately misbehaving devices and fuzzing cases (NAK'd SET_CONFIG, malformed
descriptors) — `usbd`'s silent auto-handling is a correctness blocker, not
just a limitation: it would double-handle transfers and foreclose the
misbehavior Facedancer exists to produce.

## Findings about the DCD layer (De-risk)

The ESP32-S3 USB device controller is the Synopsys DWC2 OTG-FS. Its TinyUSB
DCD lives in `src/portable/synopsys/dwc2/`:

- `dcd_dwc2.c` — the device controller driver; defines `dcd_init`,
  `dcd_connect`/`dcd_disconnect`, `dcd_set_address`, `dcd_edpt_open`/`_close`/
  `_close_all`, `dcd_edpt_xfer`, `dcd_edpt_stall`/`clear_stall`,
  `dcd_int_enable`/`_disable`/`_handler`, and the `dcd_dcache_*` family (it
  defines these itself — no IDF cache dependency needed for our non-DMA path).
  It calls back into the stack via **`dcd_event_handler`** (which *we* provide).
- `dwc2_common.c` — shared DWC2 plumbing. Includes `host/hcd.h` + `host/usbh.h`
  but those paths are guarded by `TUSB_OPT_HOST_ENABLED`; we disable host mode.
- `dwc2_esp32.h` — the S3 board glue: `DWC2_FS_REG_BASE = 0x60080000`,
  **`DWC2_EP_MAX = 7`** (incl. EP0 → 6 non-control endpoints max). Relevant IDF
  deps: `esp_intr_alloc.h`, `soc/periph_defs.h`, `soc/usb_wrap_struct.h`,
  FreeRTOS.

So the DCD compile set is bounded and host-mode-free. This is the port we vendor.

## Decision

Build **`components/raw_usb/tinyusb_port/`** as a local ESP-IDF component that
vendors *only* the DWC2 DCD driver + supporting TinyUSB common sources — **not**
`usbd.c`, **not** the class drivers, **not** `tusb.c`'s task glue. Concretely:

```
components/raw_usb/tinyusb_port/
├── CMakeLists.txt        # idf_component_register with the bounded srcs set
├── tusb_config.h         # ours: CFG_TUSB_MCU=OPT_MCU_ESP32S3, NO host, NO class
├── tusb_option.h → (vendored)  # tinyusb option/config plumbing
├── src/
│   ├── portable/synopsys/dwc2/dcd_dwc2.c        (vendored)
│   ├── portable/synopsys/dwc2/dwc2_common.c     (vendored)
│   ├── portable/synopsys/dwc2/dwc2_*.h          (vendored headers)
│   ├── common/tusb_fifo.c                       (vendored)
│   └── common/*.h, device/dcd.h, device/usbd_pvt.h, osal/*  (vendored headers)
└── LICENSE_TINYUSB.txt   # carry TinyUSB MIT notice
```

`raw_usb.c` provides:
- **`dcd_event_handler`** (the `extern` symbol the DCD calls) — enqueues every
  event onto a FreeRTOS queue and returns (never touches endpoints from ISR;
  see ADR-0005).
- the relay pump task (sole DCD writer).
- the `raw_usb_*` primitive surface (`docs/raw-usb-design.md`).

We take the `esp_tinyusb` managed component **out** of the build (remove the
dep from `main/idf_component.yml`). We talk to the DWC2 registers via the
PSP-PERI lock + the DWC2 register base, or rely on `dcd_dwc2.c`'s own
interrupt allocator (it calls `esp_intr_alloc`).

## Consequences

- **Licensing:** TinyUSB is MIT; we carry its notice in the port dir. No
  redistribution issue.
- **Maintenance:** we carry a snapshot of the DCD driver (~3k lines incl.
  dwc2_common). Upstream TinyUSB fixes don't flow automatically; we pin to
  the v0.19.0 the IDF component would have used and document the pin.
- **Build surface:** smaller than the full `esp_tinyusb` (no usbd, no classes,
  no `tusb_tasks.c`). Faster builds, less IRAM.
- **We own:** SET_ADDRESS/SET_CONFIGURATION/GET_DESCRIPTOR handling entirely
  on the host side via `espdancer.py` + facedancer's `USBDevice` standard
  request handlers. The S3 just relays `EVENT_SETUP` and acts on the host's
  subsequent `SEND`/`STALL`/`ACK_STATUS`.
- **Hard limit surfaced:** `DWC2_EP_MAX = 7`. Documented in
  `docs/raw-usb-design.md` and surfaced as a clear error from `espdancer.py`
  (M3).

## Alternatives considered

- **Use `usbd` with no class drivers + `tud_event_hook_cb`.** Rejected: the
  hook only *observes* events after `usbd` auto-handled the standard ones; it
  can't *intercept*. Double-handling and the inability to emulate misbehaving
  devices follow.
- **Use `usbd` and accept its auto-handling for SET_ADDRESS/SET_CONFIG while
  the host drives the rest.** Rejected: the host would lose GET_DESCRIPTOR
  timing and any fuzzing of standard requests — the core Facedancer use case.
- **Vendor the *entire* tinyusb component and `#if 0` out `usbd.c`.** More
  fragile than a curated DCD-only set.
- **Drive the OTG-FS registers directly, no TinyUSB at all.** More control,
  more bugs; the DWC2 DCD is already battle-tested and MIT-licensed.

## Risk

- `dcd_dwc2.c` includes `device/usbd_pvt.h` (OSAL + tusb_fifo + tusb_private
  types). Vendoring `usbd_pvt.h` is fine (it's a header); we don't compile
  `usbd.c`. We must supply an `osal` — `osal_none.h` (no-op) unless the DCD
  uses an OSAL primitive (spike: confirm during the port compile).
- Confirm the DWC2 FS interrupt allocator path uses `esp_intr_alloc` from IDF
  (it does per `dwc2_esp32.h`); verify the ISR is installable without
  `tusb_init()`.

Confidence on the *design* is high; confidence on a clean first compile is
medium. Milestone 1's first sub-task is the vendoring + dry compile.