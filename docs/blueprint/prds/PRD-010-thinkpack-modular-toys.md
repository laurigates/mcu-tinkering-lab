# PRD-010: ThinkPack Modular Toy Boxes

**Status**: draft
**Created**: 2026-04-11
**Confidence**: 6/10

---

## Overview

ThinkPack is a set of small, toddler-safe ESP32-S3 boxes that each have a distinct personality and capability. Each box works alone as an engaging tactile toy. When multiple boxes are brought near each other, they discover peers over ESP-NOW and negotiate collaborative play -- synchronized lights, cascading melodies, call-and-response sounds. When one box has LLM API access, the group gains an invisible "brain" that generates novel patterns, keeps interactions fresh, and adapts to the child's behavior.

## Problem

Toddler toys are static. A light-up toy always plays the same sequence. A musical toy always plays the same song. There's no emergent behavior from combining toys, and no way to keep the experience fresh over months of daily play. Meanwhile, ESP32 boards are cheap (~$4) and capable of sophisticated wireless coordination, but existing multi-device projects in this repo (robocar dual-ESP32) are tightly coupled point-to-point systems, not modular swarms.

## Goals

- 5 distinct box variants, each fun alone, richer together
- Infrastructure-free discovery (no WiFi router, no app, no setup)
- Automatic leader election with graceful degradation
- Optional LLM integration that generates variety without requiring speech or text
- Toddler-safe: no screens to stare at, instant tactile feedback, volume/brightness caps

## Users

- **Toddlers (2-4)**: Primary users. Interact through pressing, shaking, tilting, scanning, and physically moving boxes near each other
- **Parents**: Set up WiFi on Brainbox (via Improv WiFi in a browser), manage NFC tags, observe status on OLED
- **Makers/developers**: Build and extend the platform, design enclosures, add new box variants

## Features

### The 5 Box Variants

#### FR-T01: Chatterbox (Voice + Touch)

**Hardware**: ESP32-S3 SuperMini, MAX98357A I2S amplifier + speaker, INMP441 MEMS microphone, capacitive touch pad, WS2812 RGB LED

**Standalone behaviors**:
- Touch = random silly sound (procedurally generated)
- Hold touch = record 2 seconds of audio
- Touch again = pitch-shifted playback (funny voice effect)
- Shake (detected via ADC noise or second touch threshold) = random animal noise
- LED pulses with audio output

**Capability flags**: `CAP_AUDIO_OUT | CAP_AUDIO_IN | CAP_TOUCH`

#### FR-T02: Glowbug (Light + Motion)

**Hardware**: ESP32-S3 SuperMini, WS2812B LED ring (8-12 LEDs), MPU6050 IMU (I2C), LDR light sensor (ADC), tactile button

**Standalone behaviors**:
- Tilt = color hue shifts with orientation (gyro-mapped)
- Shake = sparkle burst animation
- Hold still = gentle breathing pattern
- Dark room (LDR) = warm amber nightlight
- Bright room = rainbow chase
- Button = cycle animation mode

**Capability flags**: `CAP_LED_RING | CAP_IMU | CAP_LIGHT_SENSE`

#### FR-T03: Brainbox (WiFi + LLM Gateway)

**Hardware**: ESP32-S3-WROOM N8R8 (8MB PSRAM), SSD1306 OLED 128x64 (I2C, parent/dev use only), WS2812 status LED, button, piezo buzzer

**Standalone behaviors**:
- Button press = piezo chime + LED color change (simple cause-and-effect)
- OLED shows WiFi status, connected peers, last LLM call timestamp (for parents)
- With WiFi, pre-caches structured content from LLM into NVS for offline use

**Group role**: Always becomes leader (highest election priority). Bridges between ESP-NOW mesh and LLM API over WiFi. Dispatches AI-generated commands to other boxes.

**Capability flags**: `CAP_WIFI | CAP_LLM | CAP_DISPLAY | CAP_STORAGE | CAP_AUDIO_OUT`

#### FR-T04: Boombox (Music + Rhythm)

**Hardware**: ESP32-S3 SuperMini, passive piezo on LEDC PWM, 2 potentiometers (ADC) for tempo + pitch, WS2812 LED, tactile button

**Standalone behaviors**:
- Pot 1 = tempo (BPM 60-200)
- Pot 2 = pitch (transpose -12 to +12 semitones)
- Button = cycle melody pattern (march, waltz, pentatonic random, call-and-response silence)
- LED blinks on beat
- Always making some kind of music when powered on

**Capability flags**: `CAP_AUDIO_OUT | CAP_POTS | CAP_RHYTHM`

#### FR-T05: Finderbox (NFC + Seek)

**Hardware**: ESP32-S3 SuperMini, RC522 NFC/RFID reader (SPI), WS2812 LED, piezo buzzer (LEDC), tactile button

**Standalone behaviors**:
- Scan NFC tag = unique chime + unique LED color per tag UID
- No tag present = gentle ambient LED pulse
- Button = replay last scanned tag's sound
- Can work as a simple "what's this?" identifier toy with labeled NFC stickers

**Capability flags**: `CAP_NFC | CAP_AUDIO_OUT`

### Discovery & Mesh

#### FR-T06: ESP-NOW Peer Discovery

Each box broadcasts a beacon at ~2 Hz containing: box type, capability bitmask, group state, battery level, and short name. When a new peer is detected, both boxes play a synchronized "hello chime" and flash a brief LED pulse. This discovery moment is the core magical interaction for toddlers -- "I put them together and they noticed each other!"

ESP-NOW chosen because it requires no WiFi infrastructure. Works at the park, in the car, at grandma's house.

#### FR-T07: Leader Election

Simplified bully algorithm. Each box computes a priority score:

| Capability | Priority |
|-----------|----------|
| `CAP_LLM` | +1000 |
| `CAP_WIFI` | +500 |
| `CAP_DISPLAY` | +200 |
| `CAP_AUDIO_OUT` | +100 |
| `CAP_AUDIO_IN` | +50 |
| Tiebreaker | Last 2 bytes of MAC |

Any box can initiate election by broadcasting a bid. After 2 seconds with no higher bid, the highest bidder claims leadership. Brainbox always wins when present.

Election triggers: new box joins, leader disappears (5s timeout), higher-priority box joins.

#### FR-T08: Capability Negotiation

After election, the leader queries all peers for full capability descriptors. Builds a group manifest that determines which collective behaviors are available with the current group composition.

Capability bitmask flags:

```
CAP_WIFI        (1 << 0)    CAP_LIGHT_SENSE (1 << 7)
CAP_LLM         (1 << 1)    CAP_NFC         (1 << 8)
CAP_AUDIO_OUT   (1 << 2)    CAP_POTS        (1 << 9)
CAP_AUDIO_IN    (1 << 3)    CAP_TOUCH       (1 << 10)
CAP_DISPLAY     (1 << 4)    CAP_RHYTHM      (1 << 11)
CAP_LED_RING    (1 << 5)    CAP_STORAGE     (1 << 12)
CAP_IMU         (1 << 6)
```

#### FR-T09: Graceful Degradation

If the leader disappears (no beacon for 5 seconds), all boxes fall back to standalone mode within one animation cycle. No frozen states, no silent failures. A new election triggers automatically if any peers remain.

### Collective Behaviors (No LLM)

These work with any 2+ boxes over ESP-NOW alone.

#### FR-T10: Synchronized Light Show

Leader sends sync pulses at 4 Hz. All boxes with LEDs synchronize their animation phase. Toddler sees: "I put them together and they glow the same!"

#### FR-T11: Pass the Sound

A tone starts on one box and "hops" to the next (ordered by join sequence). Each box adds a pitch shift. Creates a cascading melody moving around the room.

#### FR-T12: Musical Round

Boombox plays a melody. After N beats (synced via sync pulses), the next audio-capable box starts the same melody at an offset, creating a canon effect.

#### FR-T13: Hot-Cold Proximity

Finderbox beeps faster when ESP-NOW RSSI to a designated target box increases (closer). Target box's LED shifts from blue (cold) to red (hot). Physical hide-and-seek game -- no GPS needed.

#### FR-T14: Mood Mirror

One Glowbug's light sensor reading is broadcast to the group. All Glowbugs shift their color palette together. "The boxes know when you turn off the lights."

#### FR-T15: Echo Chamber

Chatterbox records a short clip. Playback is dispatched to ALL audio-capable boxes simultaneously, each with a different pitch shift. Surround silly sounds.

### Collective Behaviors (With LLM)

These require Brainbox with WiFi + API key. The LLM never talks directly to the toddler. It generates **variety** -- new patterns that keep the toys feeling fresh.

#### FR-T16: Pattern Generator

LLM generates novel melody sequences (JSON note arrays) dispatched to audio boxes. Every play session produces different music. Prompt includes group composition and recent interaction history.

#### FR-T17: Reactive Conductor

LLM receives group state ("Glowbug shaken 3 times, Boombox tempo at 120 BPM, Finderbox scanned dragon tag") and generates a coordinated multi-box response. The toddler's physical actions influence what happens across all boxes.

#### FR-T18: NFC Story Sounds

Finderbox scans a tag. Brainbox asks LLM for a sound effect sequence for that tag theme. Generates a tone sequence dispatched to audio boxes. Each tag triggers a unique "sound story" without words -- just evocative melodies and rhythms.

#### FR-T19: Adaptive Difficulty

For the proximity game (FR-T13), LLM adjusts difficulty based on how quickly the child found the last target. Tightens or loosens the RSSI threshold.

#### FR-T20: Surprise Mode

Periodically, the LLM generates an unexpected coordinated event across all boxes -- a sudden synchronized light + sound burst. Toddlers love surprise and anticipation.

### Safety & Power

#### FR-T21: Toddler Safety

- Audio volume hard-capped in firmware at child-safe levels
- LED brightness limited, no strobe effects (minimum animation period > 100ms)
- All enclosures: rounded corners, no exposed electronics, screw-secured battery compartment
- No small detachable parts
- 2-second button hold = factory reset to standalone mode

#### FR-T22: Power Management

- Adaptive ESP-NOW beacon rate: 2 Hz discovering, 0.5 Hz in stable group, 0 Hz after 5 minutes alone (deep sleep)
- Wake from deep sleep on button press or capacitive touch
- USB-C powered for development, LiPo battery for production

## Architecture

### Hardware

All variants use ESP32-S3 for toolchain consistency. ESP32-S3 SuperMini (~$4) for most boxes. Brainbox uses ESP32-S3-WROOM N8R8 (~$6) for PSRAM.

### Communication Stack

```
Toddler interaction (button / shake / tilt / NFC scan)
    |
    v
Local standalone behavior (immediate feedback < 50ms)
    |
    v
ESP-NOW mesh layer (beacon, discovery, sync pulses)
    |
    v
Leader (elected) coordinates collective behaviors
    |
    v (if Brainbox is leader + WiFi available)
LLM API (Claude / Gemini / Ollama) -- async, non-blocking
    |
    v
AI-generated commands dispatched back via ESP-NOW
```

### ESP-NOW Protocol

Packet format modeled on `i2c_protocol.h`:

```c
typedef struct __attribute__((packed)) {
    uint8_t magic[2];            // {0xTP, 0x01}
    uint8_t msg_type;            // See message types below
    uint8_t sequence_number;
    uint8_t src_mac[6];
    uint8_t data_length;
    uint8_t data[200];           // ESP-NOW max payload ~250 bytes
    uint8_t checksum;            // XOR checksum
} thinkpack_packet_t;
```

Message types: `BEACON`, `ELECTION_BID`, `LEADER_CLAIM`, `CAPABILITY`, `COMMAND`, `STATUS`, `LLM_REQUEST`, `LLM_RESPONSE`, `SYNC_PULSE`, `COLLECTIVE_TRIGGER`

### LLM Integration

Gateway pattern: only Brainbox talks to the API.

```
All boxes --ESP-NOW--> Brainbox (leader) --HTTPS--> Claude / Gemini / Ollama
            context up                                structured JSON response
            commands down
```

- Reuses `ai_backend_t` vtable (from robocar-camera), extended for text queries
- Reuses `ollama_discovery.c` for mDNS-based local Ollama detection
- Prompt builder follows `prompt_builder.c` pattern (from nfc-scavenger-hunt)
- Prompts encode group manifest + recent interactions, request structured JSON output
- All LLM calls are async -- immediate local feedback, LLM response arrives as bonus overlay

### Dual-Core Task Layout (per box)

Following the proven pattern from gamepad-synth:

| Core | Responsibility |
|------|---------------|
| Core 0 | ESP-NOW event loop, WiFi (Brainbox only), beacon processing |
| Core 1 | Standalone behavior, LED animations, audio engine, sensor polling |

### Shared Libraries

| Library | Location | Contents |
|---------|----------|----------|
| `thinkpack-protocol` | `packages/shared-libs/thinkpack-protocol/` | Packet format, message types, capability flags, checksum helpers |
| `thinkpack-mesh` | `packages/shared-libs/thinkpack-mesh/` | ESP-NOW init, beacon, peer management, leader election, group manager |
| `thinkpack-behaviors` | `packages/shared-libs/thinkpack-behaviors/` | Collective behavior engine, command dispatcher, command executor |

### Code Reuse from Existing Projects

| Component | Source | Target |
|-----------|--------|--------|
| Packed struct + XOR checksum | `shared-libs/robocar-i2c-protocol/include/i2c_protocol.h` | `thinkpack-protocol` |
| AI backend vtable | `robocar-camera/main/ai_backend.h` | Brainbox |
| Ollama mDNS discovery | `robocar-camera/main/ollama_discovery.c` | Brainbox |
| Credential loader (NVS/env/header) | `robocar-camera/main/credentials_loader.c` | Brainbox |
| Improv WiFi provisioning | `shared-libs/improv-wifi/` | Brainbox |
| Prompt builder pattern | `nfc-scavenger-hunt/main/prompt_builder.c` | Brainbox |
| LEDC tone generation | `gamepad-synth/main/main.c` | Boombox, Finderbox, Brainbox |
| NVS bitmask persistence | `nfc-scavenger-hunt/main/game_logic.c` | All boxes |
| WS2812 RMT driver | `xbox-switch-bridge/main/status_led.c` | All boxes |
| Dual-core task pinning | `gamepad-synth/main/main.c` | All boxes |
| RC522 NFC + SPI | `nfc-scavenger-hunt/` | Finderbox |
| WiFi manager | `robocar-main/main/wifi_manager.c` | Brainbox |

### Project Structure

```
packages/
  shared-libs/
    thinkpack-protocol/          # Packet format, message types, capabilities
    thinkpack-mesh/              # ESP-NOW, discovery, election, group manager
    thinkpack-behaviors/         # Collective behavior engine
  esp32-projects/
    thinkpack-chatterbox/        # Variant A: voice + touch
    thinkpack-glowbug/           # Variant B: light + motion
    thinkpack-brainbox/          # Variant C: WiFi + LLM gateway
    thinkpack-boombox/           # Variant D: music + rhythm
    thinkpack-finderbox/         # Variant E: NFC + proximity
```

## Implementation Phases

### Phase 1: Foundation
`thinkpack-protocol` and `thinkpack-mesh` shared libraries. ESP-NOW beacon, discovery, leader election. Validate with 2 bare ESP32-S3 boards logging to serial.

### Phase 2: First Boxes
Glowbug (LED ring + IMU) and Boombox (piezo + pots). Full standalone modes. Synchronized light+tone pulse when paired.

### Phase 3: LLM Gateway
Brainbox firmware. WiFi + AI backend integration. Prompt builder. Command dispatch to Glowbug + Boombox.

### Phase 4: Audio
Chatterbox firmware. I2S record/playback. Echo Chamber collective behavior.

### Phase 5: NFC
Finderbox firmware. RC522 scanning. Hot-Cold proximity game. NFC Story Sounds with LLM.

### Phase 6: Polish
3D-printable enclosure designs, LiPo power management, deep sleep, OTA relay via Brainbox.

## Related ADRs

- [ADR-014: ThinkPack ESP-NOW Mesh](../adrs/ADR-014-thinkpack-espnow-mesh.md)
- [ADR-007: Shared I2C Protocol Component](../adrs/ADR-007-shared-i2c-protocol-component.md) (pattern reference)
- [ADR-003: Pluggable AI Backends](../adrs/ADR-003-pluggable-ai-backends.md) (pattern reference)

## Status

Draft. Design phase -- no code yet.
