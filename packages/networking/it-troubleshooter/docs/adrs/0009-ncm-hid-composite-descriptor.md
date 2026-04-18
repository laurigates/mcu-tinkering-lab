# ADR-0009: NCM + HID Composite Descriptor Architecture

**Status**: Proposed
**Date**: 2026-03-11
**Confidence**: 8/10
**Supersedes**: Refines [ADR-0003](0003-cdc-acm-to-ncm-phased.md) with concrete implementation decisions

## Context

Phase 4 replaces CDC-ACM with CDC-NCM for USB Ethernet while retaining the HID boot keyboard. Research (2026-03-11) confirmed that TinyUSB has full NCM support and `espressif/esp_tinyusb` v2.x exposes it via `tinyusb_net_init()`, but **no HID + NCM composite example exists** in ESP-IDF, TinyUSB upstream, or community projects.

Several architectural decisions must be locked before implementation begins:
1. How to construct the composite USB descriptor
2. Whether to use the esp_tinyusb wrapper or raw TinyUSB
3. How to bridge WiFi traffic to USB
4. How the ESP32 itself gets an IP on the USB-side network (needed for DHCP server and HTTP server)
5. Windows compatibility requirements

## Decisions

### D1: Custom configuration descriptor with TinyUSB macros

Manually compose the configuration descriptor by concatenating `TUD_HID_DESCRIPTOR()` and `TUD_CDC_NCM_DESCRIPTOR()` macros. The esp_tinyusb default descriptor generator does not support HID, so we must provide the full descriptor via `tinyusb_config_t.configuration_descriptor`.

**Interface layout:**

| Interface | Class | Endpoints | Notes |
|-----------|-------|-----------|-------|
| 0 | HID Boot Keyboard | EP 0x81 IN (interrupt, 8B, 10ms) | Unchanged from Phase 1–3 |
| 1 | NCM Communication | EP 0x82 IN (interrupt, 8B) | IAD wraps if=1,2 |
| 2 | NCM Data | EP 0x03 OUT + EP 0x83 IN (bulk, 512B) | NTB max size = 2048 |

This preserves the Phase 1–3 endpoint numbering. The device descriptor stays `TUSB_CLASS_MISC / MISC_SUBCLASS_COMMON / MISC_PROTOCOL_IAD` (already correct).

### D2: Use esp_tinyusb v2.x wrapper, not raw TinyUSB

Upgrade `idf_component.yml` from `espressif/esp_tinyusb: "^1.0.0"` to `"^2.0.0"`. The v2.x wrapper provides `tinyusb_net_init()` with receive/transmit callbacks, which is the correct abstraction for the WiFi bridge use case.

**Migration impact:** The `tinyusb_config_t` struct changed between v1.x and v2.x. Fields move from flat to nested `.descriptor` sub-struct. All existing `usb_composite_init()` code must be updated.

### D3: Dual data path — WiFi bridge + local lwIP netif

Two distinct networking roles coexist:

1. **WiFi-to-USB bridge** (FR4.3): Raw Ethernet frame relay using `esp_wifi_internal_reg_rxcb()` / `esp_wifi_internal_tx()`. Frames pass through without touching lwIP. This is the pattern used in ESP-IDF's `tusb_ncm` example and is the only proven approach (ESP-IDF #15639 confirms lwIP bridge integration is broken).

2. **Local ESP32 IP on USB network** (FR4.2, FR4.4): The ESP32 needs its own IP (192.168.7.1) on the USB-side subnet for DHCP server and HTTP server. This requires a lwIP `netif` attached to the NCM data path.

**Approach:** Intercept frames at the `tud_network_recv_cb()` level. If the destination MAC matches the ESP32's own MAC or is broadcast, deliver to the local lwIP netif. Otherwise, forward to WiFi via `esp_wifi_internal_tx()`. Outbound: local lwIP traffic goes to `tinyusb_net_send_sync()`; WiFi RX callback also goes to `tinyusb_net_send_async()`.

```
[Host OS] ←── USB NCM ──→ [Frame Router]
                              │         │
                         local frame?   WiFi-bound?
                              │         │
                         [lwIP netif]   [esp_wifi_internal_tx()]
                         192.168.7.1    → WiFi STA → internet
                              │
                    DHCP server + HTTP server
```

### D4: MS OS 2.0 descriptors for Windows compatibility

Include MS OS 2.0 BOS descriptors so Windows 10/11 auto-loads the `UsbNcm.sys` driver. Without this, Windows 10 reports error code 28 (no driver). TinyUSB supports this via `TUD_BOS_MS_OS_20_DESCRIPTOR()`.

RNDIS is **not** used — Windows 11 deprecated RNDIS and has no inbox driver on newer builds.

### D5: Link state tracks WiFi STA events

NCM link up/down notifications are driven by WiFi STA connection/disconnection events, not by USB mount state. This is required for iOS compatibility (iOS only triggers DHCP on first link-up notification) and correctly reflects actual internet availability.

```
WiFi STA connected   → tud_network_link_state_cb(true)
WiFi STA disconnected → tud_network_link_state_cb(false)
```

## Consequences

- The `usb_composite` component requires a major refactor: new descriptor, v2.x API, NCM callbacks
- CDC-ACM serial API (`usb_cdc_write/read`) is removed — diagnostic loop in `main.c` must use the network path instead (or be temporarily removed until the HTTP/SSH server replaces it)
- Phase 3's Claude API integration continues to work over WiFi — the CDC path was only used for operator input relay
- Flash budget increases: lwIP + DHCP server + HTTP server + NCM. Must verify < 3.5 MB
- Frame router adds code complexity but avoids the broken lwIP bridge path
- Windows 10 support requires MS OS 2.0 descriptors (adds ~200 bytes to descriptor)
- Testing requires Linux, macOS, and Windows hosts to verify driver loading

## Alternatives Considered

### A1: Use RNDIS instead of NCM
Rejected. RNDIS is deprecated on Windows 11, has no inbox driver on newer builds, and is more complex to implement than NCM. NCM is the USB-IF standard going forward.

### A2: Raw TinyUSB without esp_tinyusb wrapper
Rejected. The wrapper saves significant boilerplate for driver init, task creation, and descriptor management. The custom descriptor can still be provided to the wrapper. Only revisit if wrapper limitations block NCM init (e.g., if `tinyusb_net_init()` conflicts with custom descriptors).

### A3: Pure bridge without local IP (no DHCP/HTTP on ESP32)
Would simplify implementation but blocks FR4.2 (DHCP server) and FR4.4 (HTTP bootstrap server). These are core Phase 4 requirements.

## References

- [TinyUSB NCM class driver](https://github.com/hathach/tinyusb/tree/master/src/class/net)
- [TinyUSB NCM rewrite PR #2227](https://github.com/hathach/tinyusb/pull/2227) — performance fix
- [TinyUSB MS OS 2.0 descriptor PR #2829](https://github.com/hathach/tinyusb/pull/2829) — Windows support
- [ESP-IDF tusb_ncm example](https://github.com/espressif/esp-idf/tree/v5.4/examples/peripherals/usb/device/tusb_ncm)
- [ESP-IDF #15639: lwIP bridge with USB-NCM broken](https://github.com/espressif/esp-idf/issues/15639)
- [ESP-IDF #18079: iOS NCM link state timing](https://github.com/espressif/esp-idf/issues/18079)
- [Windows NCM driver behavior differences](https://learn.microsoft.com/en-us/answers/questions/1531925/)
- [ADR-0003: CDC-ACM to CDC-NCM Phased](0003-cdc-acm-to-ncm-phased.md)
- [ADR-0007: Core Assignment Strategy](0007-core-assignment-strategy.md)
