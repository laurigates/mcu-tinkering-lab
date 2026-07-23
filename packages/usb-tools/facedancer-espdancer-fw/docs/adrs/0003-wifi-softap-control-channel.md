# ADR-0003: WiFi SoftAP as the Control Channel (not UART)

**Status**: Accepted
**Date**: 2026-07-22
**Confidence**: High

## Context

The S3 has one OTG port and zero additional USB. To receive Facedancer
commands from the controlling PC we need a *non-USB* transport. Candidates:
on-board USB-UART bridge (921600–2 Mbaud, ~90–180 kB/s), or the S3's WiFi
(mbps, ~ms jitter). xbox-switch-bridge already runs a battle-tested SoftAP
("xbox-bridge-log") + UDP log sink (`components/log_udp`) on the S3-Zero, with
the AMPDU-off / NVS-off / HT20 / country-FI / 19.5 dBm workarounds that make
the AP actually visible (espressif/esp-idf#13508).

## Decision

Carry the control channel over **WiFi**, reusing the proven SoftAP bring-up.
Joining the AP needs no router/credentials; the controlling PC connects to
`espdancer-log` and opens a TCP socket to `192.168.4.1:4444` (the framed TLV
in `usb_rpc`). The same AP forwards ESP_LOG over UDP for debugging.

UART (GPIO43/44 + CP2102) remains the **debug-uart** build-variant logging
path and the fallback if WiFi adoption ever bites, but it's not the production
control channel.

## Consequences

- Throughput is comfortably bulk-class from day one — enough for mass storage
  over Full-Speed, not just HID/CDC. UART's ~180 kB/s ceiling is the binding
  constraint there; WiFi clears it.
- ~ms of RF jitter on the RPC path. Acceptable because USB Full-Speed polling
  is 1 ms anyway; the host-side `espdancer.py` drives pacing.
- Memory: WiFi static buffers trimmed (6 RX / 8 TX) per xbox-switch-bridge to
  coexist with TinyUSB in internal DRAM. With 2 MB PSRAM this is not binding.
- BLE coexistence is irrelevant here (we disable BT entirely), so the
  "C1: unstable" coexistence matrix caveat does not apply.

## Alternatives Considered

- **UART-only first** (switch-usb-proxy's documented topology). Initially
  attractive (~180 kB/s HID), but it caps bulk devices and adds a CP2102 to
  the BOM for production. WiFi avoids both at roughly equal bring-up cost now
  that the SoftAP workarounds are pre-solved.

## Reference

- `xbox-switch-bridge/docs/adrs/0007-wifi-udp-log-broadcasting.md`
- `xbox-switch-bridge/components/log_udp/log_udp.c`