# ADR-014: ThinkPack ESP-NOW Mesh Communication

**Status**: proposed
**Date**: 2026-04-11
**Source**: PRD-010 ThinkPack Modular Toy Boxes

---

## Context

ThinkPack is a set of modular ESP32-S3 toy boxes that need to discover each other and coordinate behavior wirelessly. The primary use case is toddlers (2-4 years old) placing boxes near each other at home, outdoors, or in a car -- environments where WiFi infrastructure may not be available. The system requires:

- Peer-to-peer discovery without any router or app
- Low-latency coordination (sync pulses at 4 Hz for light/sound synchronization)
- Coexistence with WiFi STA on the Brainbox variant (which needs WiFi for LLM API calls)
- Support for 5-10 simultaneous peers
- Small payloads: beacons (~30 bytes), commands (~20-50 bytes), occasional large LLM responses (~500-2000 bytes)

Three protocols were evaluated: ESP-NOW, WiFi+MQTT, and BLE Mesh.

## Decision

Use **ESP-NOW** as the primary inter-box communication protocol for the ThinkPack system.

ESP-NOW is a connectionless, vendor-specific protocol by Espressif that operates on the WiFi PHY layer without requiring WiFi association. It supports broadcast and unicast to registered peers, with payloads up to 250 bytes per frame and delivery latency of approximately 1ms.

The packet format follows the established pattern from `robocar-i2c-protocol`: packed C structs with explicit field sizes, sequence numbers for ordering, and XOR checksums for integrity.

### Key Design Choices

**Broadcast for beacons, unicast for commands.** Beacons (presence announcements) use ESP-NOW broadcast so any nearby box can discover peers without pre-registration. Commands from leader to specific boxes use unicast to registered peers for reliability (ESP-NOW provides per-frame send callbacks with success/failure status for unicast).

**WiFi channel pinning.** ESP-NOW and WiFi STA share the same radio and must operate on the same channel. The Brainbox (which uses WiFi STA for LLM API calls) sets its WiFi channel, and all other boxes match it via the channel field in ESP-NOW beacons. Default: channel 1.

**Message fragmentation for LLM responses.** LLM responses can exceed the 250-byte ESP-NOW frame limit. Large messages are fragmented with a fragment header (fragment index, total fragments, message ID) and reassembled by the receiver. Most operational messages (beacons, commands, sync pulses) fit in a single frame.

**Leader-follower topology, not true mesh.** Boxes don't relay messages for each other. The leader communicates directly with all followers. This simplifies the protocol and avoids the complexity of mesh routing. Range is limited to ESP-NOW's ~200m line-of-sight, which is more than sufficient for toys in the same room.

## Consequences

**Positive:**
- Zero infrastructure required -- works anywhere two boxes are powered on
- ~1ms latency enables real-time synchronized light and sound effects
- Coexists with WiFi STA on Brainbox (ESP-IDF supports concurrent ESP-NOW + WiFi STA on the same interface)
- Simple API: `esp_now_send()` / `esp_now_register_recv_cb()` -- well-documented in ESP-IDF
- Low power: no association handshake, no TCP stack, no connection maintenance
- Up to 20 registered peers (ESP-IDF limit), sufficient for toy-scale deployments

**Negative:**
- New to this codebase -- no existing ESP-NOW code to build on (all other projects use WiFi, BLE, or UART)
- 250-byte frame limit requires fragmentation logic for large payloads
- No built-in encryption beyond optional ESP-NOW PMK/LMK (sufficient for toys, not for sensitive data)
- Channel must match across all boxes and Brainbox's WiFi AP -- if the home router uses a non-default channel, Brainbox must adapt and notify peers
- No delivery guarantees for broadcast messages (beacons) -- mitigated by high beacon rate (2 Hz)
- RSSI-based proximity (FR-T13 Hot-Cold game) is approximate and affected by obstacles/multipath

## Alternatives Considered

### WiFi + MQTT

The robocar-camera project already uses MQTT for telemetry (`mqtt_logger.c`). This would allow direct code reuse.

**Rejected because:**
- Requires a WiFi router or one box running SoftAP (complex for non-technical parents of toddlers)
- SoftAP mode on the coordinator consumes more power and adds connection management complexity
- MQTT adds latency (TCP handshake, broker routing) that degrades synchronized light/sound effects
- Toddler toys must "just work" when placed near each other with no setup

### BLE Mesh

The codebase has BLE experience via Bluepad32 (gamepad-synth, xbox-switch-bridge) for gamepad input.

**Rejected because:**
- BLE Mesh is significantly more complex to implement (provisioning, relay nodes, friendship, models)
- Higher latency than ESP-NOW (BLE advertising intervals are 20-10240ms)
- Bluepad32 uses the BLE stack for gamepad input -- running BLE Mesh alongside it would require careful coexistence
- ESP-NOW is simpler for the leader-follower topology ThinkPack needs (not a true mesh)

### Hybrid: ESP-NOW for discovery + WiFi for data

Use ESP-NOW only for initial discovery, then switch to WiFi for richer data exchange.

**Rejected because:**
- Adds complexity of managing two protocols simultaneously
- Most ThinkPack messages are small (commands, sync pulses) -- ESP-NOW is sufficient
- Only LLM responses are large, and those only flow from Brainbox to followers -- fragmentation handles this adequately
- WiFi association adds latency to the discovery experience (the "hello chime" moment should be instant)

## Files Changed

New files (no existing files modified):

- `packages/shared-libs/thinkpack-protocol/` -- packet definitions, checksums, capability flags
- `packages/shared-libs/thinkpack-mesh/` -- ESP-NOW init, beacon, peer management, leader election
- `packages/shared-libs/thinkpack-behaviors/` -- collective behavior engine, command dispatch

## References

- [ESP-IDF ESP-NOW Programming Guide](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/api-reference/network/esp_now.html)
- [PRD-010: ThinkPack Modular Toy Boxes](../prds/PRD-010-thinkpack-modular-toys.md)
- [ADR-007: Shared I2C Protocol Component](ADR-007-shared-i2c-protocol-component.md) -- packet format pattern reference
