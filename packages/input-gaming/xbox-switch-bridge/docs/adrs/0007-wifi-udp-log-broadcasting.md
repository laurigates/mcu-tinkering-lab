# ADR-0007: WiFi UDP Log Broadcasting

**Status**: Accepted
**Date**: 2026-03-12
**Confidence**: High

## Context

In production builds, TinyUSB owns the USB PHY, which disconnects USB-Serial-JTAG entirely. `CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG` does not redirect ESP_LOG output when TinyUSB is active. A USB-UART adapter on GPIO43/44 works but requires extra hardware and physical access to the board.

A wireless logging mechanism is needed for debugging production builds (especially the Switch USB handshake) without additional hardware.

## Decision

Implement **WiFi SoftAP + UDP broadcast logging** as the `log_udp` component.

Architecture:
1. ESP32-S3 starts a WiFi access point: SSID "xbox-bridge-log" (open, no password)
2. A UDP socket broadcasts to 192.168.4.255 on port 4444
3. `esp_log_set_vprintf()` hooks all ESP_LOG output to the UDP sink (in addition to UART)
4. Logs are only sent when a client is connected (to avoid unnecessary packet construction)

Usage:
```bash
# 1. Connect Mac/PC to "xbox-bridge-log" WiFi
# 2. Listen for logs:
just log-listen   # or: socat UDP-RECV:4444 STDOUT
```

## Consequences

- **BLE + SoftAP coexistence is unstable** — Espressif rates this combination "C1: unstable" on ESP32-S3. WiFi is pinned to core 1 (`CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_1`) to reduce contention with BTstack on core 0.
- **No credentials needed** — the AP is open. This is acceptable because it's a development/debug feature, the range is limited (WiFi AP on a small board), and no sensitive data is transmitted.
- **Memory overhead** — WiFi + UDP adds ~50KB RAM usage. The ESP32-S3 with 2MB PSRAM handles this comfortably.
- **Best-effort logging** — UDP packets may be lost. This is acceptable for debug output; critical state is tracked in the bridge state machine, not in log messages.
- **Both sinks active** — UART output continues alongside UDP, so a USB-UART adapter still works if connected.

## Alternatives Considered

- **USB-UART adapter only**: Requires extra hardware and physical access. Not always available during docked testing. Rejected as sole solution.
- **WiFi STA mode (connect to router)**: Requires credentials and a nearby router. SoftAP is self-contained. Rejected for debug use.
- **TCP instead of UDP**: TCP requires connection management and can block. UDP broadcast is fire-and-forget. Rejected.
- **Bluetooth Serial Port Profile (SPP)**: Would compete with BLE for radio time. Rejected.
