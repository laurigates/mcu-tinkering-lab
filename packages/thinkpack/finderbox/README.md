# thinkpack-finderbox

NFC scan-to-chime box for the ThinkPack modular toy mesh. Runs on an
ESP32-S3 SuperMini. An MFRC522 reads NFC tags, the tag UID is looked up
in an NVS-persisted registry, and a matching behaviour fires (chime +
LED colour, solid colour, etc.).

Part of ThinkPack Phase 3 — partial [issue #198](https://github.com/laurigates/mcu-tinkering-lab/issues/198).

## Hardware

| Component  | Part                | Notes                                          |
|------------|---------------------|------------------------------------------------|
| MCU        | ESP32-S3 SuperMini  | USB-C, USB-Serial-JTAG built-in                |
| NFC reader | MFRC522 (SPI)       | 3.3 V; 5 MHz SPI mode 0                        |
| LED ring   | WS2812B × 12        | GPIO4 data; 5 V external power                 |
| Piezo      | PWM-driven buzzer   | GPIO2, LEDC low-speed timer 0                  |
| Button     | Momentary N.O.      | GPIO9 → GND (short press replays last chime)   |

See [WIRING.md](WIRING.md) for the full pinout and rationale.

## Behaviours

The tag registry maps each UID to a behaviour label carried by
`thinkpack-nfc`:

| Behaviour | Param           | Standalone effect                        |
|-----------|-----------------|------------------------------------------|
| CHIME     | 8-bit hue       | Short bleep + solid hue for ~600 ms      |
| COLOR     | 8-bit hue       | Set ring to solid hue (persistent)       |
| STORY     | track id        | Placeholder bleep (PR E wires the mesh)  |
| SEEK      | —               | No-op standalone (group-mode seek target)|
| NONE      | —               | Ignored                                  |

Registry entries are written via the mesh (PR E) and persisted to NVS.
The same UID re-scanned within 1.5 s is suppressed.

## Quick Start

```bash
# Build (Docker required — no local ESP-IDF needed)
just thinkpack-finderbox::build

# Flash (auto-detects ESP32-S3 USB-Serial-JTAG port)
just thinkpack-finderbox::flash

# Monitor serial output
just thinkpack-finderbox::monitor

# Override port manually
PORT=/dev/ttyACM0 just thinkpack-finderbox::flash
```

## Troubleshooting

**"RC522 not detected — check wiring"**
- Confirm 3.3 V on VCC and GND; do not feed 5 V to VCC.
- Check SPI pin mapping against [WIRING.md](WIRING.md).
- The driver logs the reported firmware version; `0x00` / `0xFF` means
  the chip is not responding at all.

**LEDs dark after a tag scan**
- Each CHIME ends by clearing the ring. If you want a persistent colour
  programme the tag as a COLOR behaviour instead.

**Silent piezo**
- Confirm GPIO2 is wired to the piezo's signal leg, other leg to GND.
- Check LEDC init logs in `finderbox/piezo`.
