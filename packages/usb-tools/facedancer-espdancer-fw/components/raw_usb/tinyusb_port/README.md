# tinyusb_port — DCD-only TinyUSB port (ADR-0006)

This directory vendors **only the DWC2 device controller driver** + supporting
common sources from TinyUSB, **excluding** `usbd.c`, the class drivers, and
`tusb.c`'s task glue. We bypass TinyUSB's `usbd` entirely and provide
`dcd_event_handler` ourselves in `../raw_usb.c`. See ADR-0006 for the why.

> **Status: vendored.** All files listed below are copied in (TinyUSB 0.19.0,
> the version the `espressif/esp_tinyusb` ≥1.4.0 managed component pulls under
> ESP-IDF v5.4). The port's CMakeLists compiles the three DCD sources and the
> license notice is at `LICENSE_TINYUSB.txt`. `raw_usb.c` calls the real
> `dcd_*` (RAW_USB_DCD_VENDORED=1).
>
> Next actionable: `rm -f sdkconfig && just build` against the
> `mcu-tinkering-lab/esp-idf:v5.4` docker image and clear residual IDF
> include/PRIV_REQUIRES resolution issues. This scaffolding session couldn't
> build end-to-end (the Docker image wasn't available in this environment).

## What to vendor — from `espressif__tinyusb/` (v0.19.0)

The files were copied from a worktree's `managed_components/espressif__tinyusb/`
(v0.19.0) into the layout below, **preserving the `src/<sub>/<...>/` tree**
so every `#include "common/..."', `#include "device/..."`,
`#include "osal/..."`, and sibling `#include "dwc2_common.h"` reference
resolves through the `src` include dir (plus a private include of
`src/portable/synopsys/dwc2` for the flat sibling case). Originally-listed
manifest (for reference — already done):

Source files (compile):
- `src/portable/synopsys/dwc2/dcd_dwc2.c` → `src/dcd_dwc2.c`
- `src/portable/synopsys/dwc2/dwc2_common.c` → `src/dwc2_common.c`
- `src/common/tusb_fifo.c` → `src/tusb_fifo.c`

Headers (vendored, include-only):
- `src/portable/synopsys/dwc2/dwc2_common.h`, `dwc2_esp32.h`,
  `dwc2_at32.h`, `dwc2_efm32.h`, `dwc2_gd32.h`, `dwc2_xmc.h` → `src/`
- `src/common/tusb_fifo.h`, `tusb_types.h`, `tusb_compiler.h`, `tusb_verify.h`,
  `tusb_debug.h`, `tusb_private.h` → `src/`
- `src/device/dcd.h`, `src/device/usbd_pvt.h` → `src/`
- `src/osal/osal_none.h` → `src/`
- `src/tusb_option.h` → `src/`

Carry the license:
- copy `LICENSE` from the tinyusb repo → `LICENSE_TINYUSB.txt` here.

> **Do NOT vendor** `src/device/usbd.c`, `src/device/usbd_control.c`,
> `src/tusb.c`, or any `src/class/*` — that's the whole point of the bypass.

## tusb_config.h (ours)

`tusb_config.h` (in this dir) sets:
- `CFG_TUSB_MCU = OPT_MCU_ESP32S3`
- `CFG_TUSB_OS = OPT_OS_NONE`
- `CFG_TUSB_RHPORT0_MODE = OPT_MODE_DEVICE` (no host)
- `CFG_TUD_*` all 0 (no class drivers — we don't use `tud_*`)
- `CFG_TUD_ENDPOINT0_SIZE` = 64 (Full Speed)

## Build wiring

`tinyusb_port/CMakeLists.txt` registers exactly the three source files with
the right include paths (`src/`, the IDF HAL headers). `components/raw_usb/CMakeLists.txt`
adds `PRIV_REQUIRES tinyusb_port` and `raw_usb.c` flips `RAW_USB_DCD_VENDORED`
to 1 so its `dcd_*` externs resolve to the port's strong definitions.

## Pin

TinyUSB **0.19.0** (the version the `espressif/esp_tinyusb` ≥1.4.0 managed
component pulls under ESP-IDF v5.4). Pin recorded so upstream fixes don't
silently change the DCD behaviour; bump deliberately and re-test.