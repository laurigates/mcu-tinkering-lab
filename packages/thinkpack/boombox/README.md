# ThinkPack Boombox

Music + rhythm box for the [ThinkPack modular toy mesh](https://github.com/laurigates/mcu-tinkering-lab/issues/195).
Phase 2 of the ThinkPack project — pairs with Glowbug (light + colour) and future boxes.

## Hardware

**Board**: ESP32-S3 SuperMini

| Component        | Pin    | Notes                                  |
|------------------|--------|----------------------------------------|
| Passive piezo    | GPIO 5 | LEDC PWM channel 0, 50 % duty cap      |
| WS2812 LED       | GPIO 4 | 3.3 V data, 60 % brightness cap        |
| Pot 1 (tempo)    | GPIO 1 | ADC1_CH0, 10 kΩ linear, 0–3.3 V       |
| Pot 2 (pitch)    | GPIO 2 | ADC1_CH1, 10 kΩ linear, 0–3.3 V       |
| Button           | GPIO 9 | Internal pull-up, active LOW           |

See `WIRING.md` for the full ASCII diagram.

## Quick Start

```bash
# Build (Docker — no local ESP-IDF needed)
just thinkpack-boombox::build

# Flash (auto-detects ESP32-S3 USB-Serial-JTAG port)
just thinkpack-boombox::flash

# Monitor serial output
just thinkpack-boombox::monitor

# Flash + monitor in one step
just thinkpack-boombox::flash-monitor
```

Override the port if auto-detection fails:

```bash
PORT=/dev/ttyACM0 just thinkpack-boombox::flash
```

## Expected Standalone Behaviour

| Action | Result |
|--------|--------|
| Power on | Plays MARCH pattern at mid tempo |
| Turn Pot 1 (tempo) | BPM changes smoothly (60–200 BPM) |
| Turn Pot 2 (pitch) | All notes shift up/down by up to ±12 semitones |
| Short press button | Cycles pattern: MARCH → WALTZ → PENTATONIC → SILENCE → MARCH |
| 2-second hold button | Resets to MARCH at beat 0 (soft factory reset) |
| Downbeat | LED flashes white for 80 ms |

## Expected Group Behaviour

| Event | Result |
|-------|--------|
| Peer discovered | 3-note ascending chime (C5 E5 G5), LED flashes cyan |
| Sync pulse from leader | Beat index aligns to leader — creates musical-round effect |
| Becomes leader | Broadcasts quarter-beat sync pulses; LED turns teal |
| Becomes follower | Stops broadcasting; plays in sync with leader |
| PLAY_MELODY command (0x20) | Switches to commanded pattern for N beats, then restores |

## Safety Constraints (FR-T21 — Toddler Safe)

- Piezo duty cycle capped at ≤ 50 % (13-bit LEDC: max duty = 4096 of 8192)
- LED brightness capped at 60 % of requested values
- Tone frequency bounded to 100–3500 Hz (no sub-bass, no piercing highs)

## Troubleshooting

**No sound**: Check GPIO 5 piezo connection and polarity. Passive piezos are not polarised but the connection must be solid. Verify with `just thinkpack-boombox::monitor` — you should see `Tone engine initialized on GPIO5`.

**LED not lighting**: Verify WS2812 data pin on GPIO 4. The strip needs a 300–500 Ω series resistor on the data line for signal integrity.

**ADC reads stuck at 0 or 4095**: Check pot wiring — one end to 3.3 V, other end to GND, wiper to GPIO 1 or 2.

**Port not found**: Run `just detect-s3-port` from the repo root or set `PORT=/dev/ttyACMx` explicitly.

**Mesh not forming**: Both boards must be on the same WiFi channel (default: 1). Check logs for `peer discovered` messages. LED turns teal on the leader.
