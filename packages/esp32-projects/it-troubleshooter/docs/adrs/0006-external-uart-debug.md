# ADR-0006: External USB-UART Adapter for Debug Monitoring

**Status**: Accepted
**Date**: 2026-03-09
**Confidence**: 10/10

## Context

ESP32-S3 has a built-in USB-Serial-JTAG peripheral that normally provides log output and a JTAG interface via the on-chip USB-C connector. However, the ESP32-S3 has a single internal USB PHY shared between USB-Serial-JTAG and USB-OTG.

## Decision

Use an **external USB-UART adapter** connected to GPIO43 (TX) / GPIO44 (RX) at 115200 baud for all debug monitoring.

## Rationale

Once `tinyusb_driver_install()` is called, the USB-OTG peripheral claims the shared PHY. USB-Serial-JTAG disconnects from the USB bus entirely — it is no longer visible as a USB device on the host.

Mitigations that do NOT work:
- `CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG=y` — does not redirect ESP_LOG output after TinyUSB takes the PHY
- `usb_serial_jtag_driver_install()` + `esp_log_set_vprintf()` — fails after TinyUSB init
- Redirecting console to USB-Serial-JTAG before TinyUSB — logs stop when TinyUSB initializes

The only reliable solution is a hardware UART on separate pins, connected to an external USB-UART adapter.

## Evidence

- `justfile`: `monitor` recipe uses `minicom -b 115200 -D /dev/cu.usbserial-*` (external UART adapter)
- `sdkconfig.defaults`: UART0 console remains enabled (default GPIO43/44 on ESP32-S3)
- Pattern confirmed identically in xbox-switch-bridge project

## Consequences

- Monitoring requires a USB-UART adapter (CH340, CP2102, FT232, etc.) connected to GPIO43/44
- USB-C connector on the device is exclusively for the composite USB device (keyboard + CDC)
- Development workflow: two USB connections — one for the device function, one for debug monitoring
- JTAG debugging is unavailable while TinyUSB is running (same PHY constraint)
