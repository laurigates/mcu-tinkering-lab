# PRP: CDC-NCM USB Ethernet (Phase 4)

**Source**: [IT Troubleshooter PRD — Phase 4](../prds/it-troubleshooter.md#phase-4--cdc-ncm-usb-ethernet-planned)
**Confidence**: 8/10 (upgraded from 6/10 after research 2026-03-11)
**Status**: Planning

## Goal

Replace the CDC-ACM serial interface with CDC-NCM (USB Ethernet), enabling the device to bridge WiFi to USB and present itself as a network adapter to the host. This eliminates the need for HID keyboard injection for most workflows, while retaining HID for BIOS-level access.

## Requirements

From PRD Phase 4:
- FR4.1: CDC-NCM USB Ethernet interface visible to host OS as network adapter
- FR4.2: DHCP server assigning IP to host
- FR4.3: WiFi-to-USB bridge routing host traffic through device's WiFi
- FR4.4: HTTP server hosting self-bootstrapping agent script
- FR4.5: Retain HID keyboard interface alongside NCM

## Research Findings (2026-03-11)

### TinyUSB NCM Support — Confirmed

TinyUSB has a mature CDC-NCM class driver (`src/class/net/ncm_device.c`), rewritten in PR #2227 for correctness and ~2x bandwidth improvement. Effective throughput: 2–5 Mbps on embedded targets. ESP-IDF v5.4 ships this version.

### esp_tinyusb NCM Wrapper — Confirmed

`espressif/esp_tinyusb` v2.x provides:
- `tinyusb_net_init(tinyusb_net_config_t *)` — registers MAC, receive callback, TX free callback
- `tinyusb_net_send_sync()` / `tinyusb_net_send_async()` — transmit paths
- Default NCM descriptor generation in `usb_descriptors.c`
- Official example: `examples/peripherals/usb/device/tusb_ncm` (WiFi bridge)

### HID + NCM Composite — No Existing Example

No HID + NCM composite example exists anywhere (ESP-IDF, TinyUSB, community). The esp_tinyusb default descriptor generator supports NCM but **not HID**. A custom configuration descriptor must be provided manually via `tinyusb_config_t.configuration_descriptor`.

### Descriptor Layout — Structurally Clean Swap

NCM uses the same 2-interface + 3-endpoint layout as CDC-ACM:

| Interface | Phase 1–3 (ACM) | Phase 4 (NCM) |
|-----------|-----------------|---------------|
| 0 | HID Boot Keyboard | HID Boot Keyboard (unchanged) |
| 1 | CDC-ACM Communication (IAD) | NCM Communication (IAD) |
| 2 | CDC-ACM Data | NCM Data |

Endpoint map is identical: EP 0x81 (HID IN), EP 0x82 (notification IN), EP 0x03 (data OUT), EP 0x83 (data IN). Bulk endpoint size increases from 64B to 512B for NCM throughput.

### WiFi Bridge — esp_wifi_internal Callback Pattern

The proven approach (used in ESP-IDF's tusb_ncm example) uses `esp_wifi_internal_reg_rxcb()` and `esp_wifi_internal_tx()` for raw Ethernet frame relay, bypassing lwIP entirely. This works because lwIP bridge integration with USB-NCM is broken (ESP-IDF #15639).

For the ESP32's own IP on the USB network (DHCP server, HTTP server), a separate lwIP `netif` intercepts frames addressed to the ESP32's MAC or broadcast.

### Windows Compatibility — NCM with MS OS 2.0 Descriptors

- **Windows 11**: `UsbNcm.sys` is inbox, auto-loads with MS OS 2.0 descriptors
- **Windows 10**: `UsbNcm.sys` is inbox but requires MS OS 2.0 descriptors for auto-loading; has a ZLP bug on max-packet-size bulk transfers
- **RNDIS**: Deprecated on Windows 11, no inbox driver — do not use
- TinyUSB supports MS OS 2.0 via `TUD_BOS_MS_OS_20_DESCRIPTOR()` (PR #2829)

### iOS Compatibility — Link State Timing

iOS only triggers DHCP on first NCM link-up notification. Link state must be driven by WiFi STA events (connected → link up, disconnected → link down), not USB mount state.

## Architecture

See [ADR-0009: NCM + HID Composite Descriptor Architecture](../adrs/0009-ncm-hid-composite-descriptor.md) for full rationale.

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

### Component Changes

| Component | Change |
|-----------|--------|
| `usb_composite` | Major refactor: NCM descriptor, v2.x API, NCM callbacks, remove CDC-ACM |
| `wifi_manager` | Add link state notification (NCM up/down on WiFi events) |
| NEW: `usb_network` | Frame router, lwIP netif, DHCP server config |
| NEW: `http_server` | Bootstrap HTTP server on 192.168.7.1:80 |
| `main.c` | Replace CDC-based diagnostic loop with network-based flow |
| `state_machine` | Add `SM_PHASE_NCM_READY` state |

## Implementation Steps

### Step 1: esp_tinyusb v1→v2 Migration + NCM Descriptor

**Goal**: Device enumerates as HID + NCM composite (no networking yet).

1. Upgrade `idf_component.yml`: `espressif/esp_tinyusb: "^2.0.0"`
2. Migrate `usb_composite_init()` to v2.x `tinyusb_config_t` struct
3. Write custom config descriptor: `TUD_HID_DESCRIPTOR()` + `TUD_CDC_NCM_DESCRIPTOR()`
4. Add `TUD_BOS_MS_OS_20_DESCRIPTOR()` for Windows
5. Implement `tud_network_*` callbacks (stubs initially)
6. Verify: host sees network adapter + HID keyboard

**Test**: `lsusb -v` (Linux), System Information (macOS), Device Manager (Windows)

### Step 2: WiFi-to-USB Bridge

**Goal**: Host gets internet via USB → ESP32 → WiFi.

1. Create `usb_network` component
2. Implement frame relay: `esp_wifi_internal_reg_rxcb()` → `tinyusb_net_send_async()`
3. Implement frame relay: NCM recv callback → `esp_wifi_internal_tx()`
4. Wire WiFi STA events to NCM link state

**Test**: Host gets internet; `ping 8.8.8.8` works from host

### Step 3: Local IP + DHCP Server

**Goal**: ESP32 has 192.168.7.1 on USB network; host gets 192.168.7.2 via DHCP.

1. Add lwIP `netif` for ESP32-local traffic on the NCM interface
2. Implement frame router: broadcast/local → lwIP, else → WiFi bridge
3. Configure lwIP DHCP server on 192.168.7.0/24 subnet
4. Bind to 192.168.7.1

**Test**: `ping 192.168.7.1` from host; verify DHCP lease

### Step 4: HTTP Bootstrap Server

**Goal**: `http://192.168.7.1/bootstrap` serves self-bootstrapping script.

1. Create `http_server` component using `esp_http_server`
2. Serve bootstrap script at `/bootstrap`
3. Add `/status` endpoint for device state JSON

**Test**: `curl http://192.168.7.1/bootstrap` returns script

### Step 5: Integration + Cross-Platform Testing

1. Update `main.c` flow for NCM-based operation
2. Update `state_machine` with NCM states
3. Test on Linux, macOS, Windows 10, Windows 11
4. Verify HID keyboard still works alongside NCM
5. Flash budget check (must be < 3.5 MB)

## Key Risks

| Risk | Severity | Mitigation |
|------|----------|------------|
| No HID+NCM example exists | High | Descriptor construction is well-understood from research; TinyUSB macros handle encoding |
| lwIP bridge broken (#15639) | Medium | Use `esp_wifi_internal_*` callback relay (proven in tusb_ncm example) |
| Frame router complexity | Medium | Start with bridge-only (Step 2), add local IP later (Step 3) |
| Flash budget (~3.5 MB limit) | Medium | Monitor with `idf.py size`; strip unused lwIP features |
| Windows 10 ZLP bug | Low | Firmware-side workaround documented; test explicitly |
| iOS DHCP timing (#18079) | Low | Link state tracks WiFi events, not USB mount |

## Test Plan

1. **Enumeration**: Host shows network adapter (check on Linux, macOS, Windows)
2. **HID retention**: Open text editor, verify HID keyboard types correctly
3. **DHCP**: Host auto-assigned 192.168.7.x; verify `ipconfig`/`ifconfig`
4. **Ping**: `ping 192.168.7.1` from host succeeds
5. **Bridge**: `ping 8.8.8.8` from host via USB → WiFi succeeds
6. **HTTP**: `curl http://192.168.7.1/bootstrap` returns bootstrap script
7. **LED states**: Correct LED feedback through NCM init → WiFi → ready
8. **Flash size**: `idf.py size` reports < 3.5 MB

## Related

- [ADR-0003: CDC-ACM to CDC-NCM Phased](../adrs/0003-cdc-acm-to-ncm-phased.md)
- [ADR-0007: Core Assignment Strategy](../adrs/0007-core-assignment-strategy.md)
- [ADR-0009: NCM + HID Composite Descriptor Architecture](../adrs/0009-ncm-hid-composite-descriptor.md)
- [PRP: WiFi Connectivity](wifi-connectivity.md) — prerequisite (completed)
- [PRP: Claude API Integration](claude-api-integration.md) — predecessor (completed)
