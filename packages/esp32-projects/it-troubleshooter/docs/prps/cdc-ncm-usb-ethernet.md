# PRP: CDC-NCM USB Ethernet (Phase 4)

**Source**: [IT Troubleshooter PRD — Phase 4](../prds/it-troubleshooter.md#phase-4--cdc-ncm-usb-ethernet-planned)
**Confidence**: 6/10
**Status**: Planned

## Goal

Replace the CDC-ACM serial interface with CDC-NCM (USB Ethernet), enabling the device to bridge WiFi to USB and present itself as a network adapter to the host. This eliminates the need for HID keyboard injection for most workflows, while retaining HID for BIOS-level access.

## Requirements

From PRD Phase 4:
- FR4.1: CDC-NCM USB Ethernet interface visible to host OS as network adapter
- FR4.2: DHCP server assigning IP to host
- FR4.3: WiFi-to-USB bridge routing host traffic through device's WiFi
- FR4.4: HTTP server hosting self-bootstrapping agent script
- FR4.5: Retain HID keyboard interface alongside NCM

## Design Notes

### Composite Descriptor Refactor

Phase 1–3 use: `HID (if=0) + CDC-ACM (if=1,2)` — 3 interfaces total.

Phase 4 replaces with: `HID (if=0) + CDC-NCM (if=1,2)` — CDC-NCM also uses 2 interfaces (Control + Data).

No official ESP-IDF example exists for HID + CDC-NCM composite. The `usb_composite` component descriptor must be fully rewritten. Key references:
- TinyUSB `examples/device/cdc_msc_hid` for composite patterns
- USB CDC-NCM spec (USB IF)
- lwIP netif integration via `esp_netif`

### Network Stack

```
[Host OS] ←── USB NCM ───→ [ESP32 NCM netif] ←──→ [esp_netif WiFi STA]
                                   │
                            NAT/bridge routing
                            DHCP server (192.168.7.1/24)
                            HTTP server (:80)
```

- `esp_netif` for both STA (WiFi) and USB NCM interfaces
- lwIP for IP stack, DHCP server, NAT
- `esp_http_server` for bootstrap HTTP server

### Bootstrap HTTP Server

Serve a script at `http://192.168.7.1/bootstrap` that:
1. Installs an SSH key for passwordless access
2. Configures SSH reverse tunnel back to operator's machine
3. Runs initial diagnostic commands

### HID Retention

HID keyboard interface must remain functional in Phase 4 for:
- BIOS/UEFI access (before OS loads, NCM not available)
- Typing credentials on login screens
- Emergency recovery when network path fails

## Implementation Steps

1. Research TinyUSB CDC-NCM descriptor requirements
2. Rewrite `usb_composite` component descriptor for HID + NCM
3. Implement lwIP NAT between NCM netif and WiFi STA netif
4. Add DHCP server on NCM interface (192.168.7.1/24, assign 192.168.7.2 to host)
5. Add `esp_http_server` with bootstrap endpoint
6. Test enumeration on Linux, macOS, Windows

## Key Risks

- No official HID+NCM composite example — descriptor construction is complex
- NAT/routing between two `esp_netif` interfaces may require custom lwIP hooks
- Windows NCM driver may require RNDIS instead (test on target platforms)
- Flash budget: lwIP + HTTP server + WiFi + TinyUSB may approach 3.5 MB limit

## Test Plan

1. Verify NCM enumeration: host shows new network adapter after plug-in
2. Ping 192.168.7.1 from host; verify DHCP lease
3. Access `http://192.168.7.1/bootstrap`; verify script served
4. Browse internet from host via USB NCM → WiFi bridge
5. Verify HID keyboard still works (open text editor, type test string)
6. Test on Linux, macOS, Windows

## Related

- [ADR-0003: CDC-ACM to CDC-NCM Phased](../adrs/0003-cdc-acm-to-ncm-phased.md)
- [ADR-0007: Core Assignment Strategy](../adrs/0007-core-assignment-strategy.md)
- [PRP: WiFi Connectivity](wifi-connectivity.md) — prerequisite
