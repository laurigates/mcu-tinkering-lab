# ADR-010: MQTT Logging Architecture

**Status**: accepted
**Date**: 2025-08-13
**Source commit**: feat: implement comprehensive MQTT logging system (041fdca)
**Confidence**: 8/10

---

## Context

During robocar development, debugging required either a serial cable (impractical on a moving robot) or WiFi-based telemetry. The robot camera module (ESP32-CAM) generates continuous inference data, motor commands, and system events that need to be observable in real time. Standard ESP-IDF logging (`ESP_LOGx`) outputs only to serial; no wireless mechanism existed.

MQTT was already used as the inter-system communication layer (robocar-camera → robocar-main). Adding structured logging to the same MQTT infrastructure avoids introducing a second network protocol.

## Decision

Implement a dedicated MQTT logger (`mqtt_logger.h/c`) on the ESP32-CAM that:
- Formats log messages as JSON with timestamp, level, tag, and message fields
- Publishes to configurable MQTT topics (e.g., `robocar/logs/{level}`)
- Buffers messages offline when MQTT is disconnected, replaying on reconnection
- Auto-discovers the MQTT broker via mDNS (`_mqtt._tcp.local`)
- Supports configurable log levels and QoS settings per topic

## Consequences

**Positive:**
- Real-time wireless observability of a moving robot without serial cable
- Structured JSON logs are machine-parseable for dashboards (Grafana, Home Assistant)
- mDNS broker discovery avoids hardcoded IP addresses
- Offline buffering ensures no log loss during brief disconnections

**Negative:**
- Additional MQTT message traffic (bandwidth impact on constrained WiFi)
- Buffer memory consumption on ESP32 (bounded, configurable)
- MQTT broker required in the development environment

## Alternatives Considered

1. **UDP syslog** — No reliability guarantees; no offline buffering
2. **HTTP log endpoint** — Higher per-message overhead; polling vs. push
3. **WebSocket streaming** — Requires persistent connection; less resilient to network changes
4. **Serial only** — Impractical for mobile robot debugging

## Implementation

- `packages/robocar/camera/main/mqtt_logger.h/c`
- Broker discovery: `mdns_query_srv("_mqtt", "_tcp", ...)`
- Log topic pattern: `robocar/camera/logs/<LEVEL>`
- JSON payload: `{"ts": <ms>, "level": "I", "tag": "...", "msg": "..."}`
- Buffer: circular buffer, configurable depth (default 32 messages)

## Related

- ADR-003: Pluggable AI Backends (uses same MQTT infrastructure)
- `packages/robocar/docs/docs/MQTT_LOGGING.md`
