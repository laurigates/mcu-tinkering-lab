# Wiring Diagram

## Overview

```
                    ┌─────────────────────────────┐
                    │   TTGO LoRa32 V2.0 ESP32    │
                    │   (with OLED & LoRa)        │
                    └─────────────────────────────┘
                           │
        ┌──────────────────┼──────────────────┬──────────────┐
        │                  │                  │              │
   ┌────▼─────┐      ┌────▼─────┐      ┌────▼─────┐   ┌────▼─────┐
   │ RC522    │      │  Green   │      │   Red    │   │  Tilt    │
   │  RFID    │      │  Button  │      │  Button  │   │  Switch  │
   │  Reader  │      │  (Play)  │      │ (Pause)  │   │  (Wake)  │
   └──────────┘      └──────────┘      └──────────┘   └──────────┘
```

## RC522 RFID Reader

The RC522 uses SPI communication protocol. **Important:** The TTGO LoRa32 V2.0 has an onboard LoRa SX1276 chip that uses the default VSPI pins (GPIO5/18/19/27), so we must use the **HSPI bus** instead to avoid conflicts.

```
RC522               TTGO LoRa32 V2.0
┌─────────┐        ┌─────────┐
│ SDA(CS) ├────────┤ GPIO15  │ SPI CS
│ SCK     ├────────┤ GPIO14  │ SPI Clock
│ MOSI    ├────────┤ GPIO13  │ SPI MOSI
│ MISO    ├────────┤ GPIO35  │ SPI MISO (input-only pin)
│ IRQ     │        │         │ Not connected
│ GND     ├────────┤ GND     │
│ RST     │        │         │ Not connected (uses power-on reset)
│ 3.3V    ├────────┤ 3.3V    │
└─────────┘        └─────────┘
```

**⚠️ Important:**
- The RC522 operates at **3.3V only**. Do not connect to 5V!
- **Do NOT use VSPI pins** (GPIO18/19/23) - they conflict with the onboard LoRa chip
- **Do NOT use GPIO12 for MISO** - it's a strapping pin that prevents flashing when pulled high
- GPIO35 is input-only, which is perfect for MISO (data from RC522 to ESP32)

## Buttons

Both buttons use the same wiring pattern with internal pull-up resistors:

### Green Button (Play)

```
     TTGO LoRa32 V2.0
    ┌────────────┐
    │  GPIO32    ├──────┬─── One side of button
    │            │      │
    │            │    ┌─┴─┐
    │            │    │   │  Green Button
    │            │    └─┬─┘
    │  GND       ├──────┴─── Other side of button
    └────────────┘
```

### Red Button (Pause)

```
     TTGO LoRa32 V2.0
    ┌────────────┐
    │  GPIO33    ├──────┬─── One side of button
    │            │      │
    │            │    ┌─┴─┐
    │            │    │   │  Red Button
    │            │    └─┬─┘
    │  GND       ├──────┴─── Other side of button
    └────────────┘
```

**Note:** When the button is pressed, it connects the GPIO pin to GND. The internal pull-up resistor keeps the pin HIGH when the button is not pressed.

## Tilt Switch (Power Management)

The tilt switch (SW-200D or similar ball tilt switch) wakes the device from deep sleep:

```
     TTGO LoRa32 V2.0
    ┌────────────┐
    │  GPIO27    ├──────┬─── One side of tilt switch
    │            │      │
    │            │    ┌─┴─┐
    │            │    │ ○ │  Ball Tilt Switch
    │            │    └─┬─┘
    │  GND       ├──────┴─── Other side of tilt switch
    └────────────┘
```

**How it works:**
- Device lying flat: Tilt switch OPEN → Deep sleep (~10µA power draw)
- Device picked up/tilted: Tilt switch CLOSES → Wakes from deep sleep
- After 2 minutes awake: Automatically enters deep sleep to save battery
- Battery life: ~29 days in sleep vs ~17 hours always-on

**Tilt Switch Types:**
- **SW-200D** - Basic ball tilt switch (most common)
- **SW-520D** - Alternative model
- Any normally-open tilt switch with similar specifications

## Piezo Buzzer (Audio Feedback)

The piezo buzzer provides pleasant musical tones for state indication:

```
     TTGO LoRa32 V2.0
    ┌────────────┐
    │  GPIO4     ├──────────── + Buzzer
    │            │
    │  GND       ├──────────── - Buzzer
    └────────────┘
```

**Note:** GPIO4 is used instead of GPIO25 because the TTGO LoRa32 has a built-in blue LED on GPIO25 which is used for status indication.

**Buzzer Types:**
- **Passive piezo buzzer** (recommended) - Allows playing different tones/melodies
- Active buzzers only produce one fixed frequency (not recommended)
- Typical voltage: 3.3V or 5V

**Tones played:**
- Boot: Gentle ascending arpeggio (C-D-E-G)
- Ready: Major chord chime (C-E-G)
- Tag scanned: Soft ascending fifth (G-C6)
- Error: Descending third (E-C)

## Vibration Motor (Haptic Feedback)

The vibration motor provides tactile feedback when a tag is scanned:

```
     TTGO LoRa32 V2.0              Motor Circuit
    ┌────────────┐                 ┌─────────────┐
    │  GPIO26    ├──── 1kΩ ────────┤ Base        │
    │            │                 │   NPN       │
    │            │      3.3V ──────┤ Motor+ ─────┤ Collector
    │            │                 │             │
    │  GND       ├─────────────────┤ Motor- ─────┤ Emitter
    └────────────┘                 └─────────────┘
```

**Components needed:**
- Coin vibration motor (3V, ~100mA)
- NPN transistor (2N2222, BC547, or similar)
- 1kΩ resistor (base current limiter)

**Why transistor?** The motor draws ~100mA, too much for GPIO pins (max 12mA). The transistor acts as a switch controlled by the GPIO.

**Alternative:** Use a motor driver module (L9110, DRV8833) for simpler wiring.


## Complete Pin Summary

| Component | Component Pin | TTGO Pin | GPIO | Notes |
|-----------|--------------|----------|------|-------|
| **RC522** | SDA (CS)     | GPIO15   | GPIO15 | SPI Chip Select |
| **RC522** | SCK          | GPIO14   | GPIO14 | SPI Clock |
| **RC522** | MOSI         | GPIO13   | GPIO13 | SPI Data Out |
| **RC522** | MISO         | GPIO35   | GPIO35 | SPI Data In (input-only pin) |
| **RC522** | IRQ          | -        | -      | Not used |
| **RC522** | GND          | GND      | -      | Ground |
| **RC522** | RST          | -        | -      | Not connected (power-on reset) |
| **RC522** | 3.3V         | 3.3V     | -      | **3.3V only!** |
| **Green Button** | One side | GPIO32 | GPIO32 | Play button |
| **Green Button** | Other side | GND  | -      | Ground |
| **Red Button** | One side   | GPIO33 | GPIO33 | Pause button |
| **Red Button** | Other side | GND    | -      | Ground |
| **Tilt Switch** | One side   | GPIO27 | GPIO27 | Wake from deep sleep (RTC-capable GPIO) |
| **Tilt Switch** | Other side | GND    | -      | Ground |
| **Buzzer** | +          | GPIO4    | GPIO4  | Passive piezo buzzer for tones |
| **Buzzer** | -          | GND      | -      | Ground |
| **Vibration Motor** | Control | GPIO26 | GPIO26 | Via NPN transistor (2N2222) |
| **Vibration Motor** | Power   | 3.3V   | -      | Motor positive |
| **Vibration Motor** | Ground  | GND    | -      | Via transistor emitter |
| **Status LED** | Built-in   | GPIO25 | GPIO25 | Built-in blue LED on TTGO |

## Power Considerations

- The TTGO LoRa32 V2.0 can be powered via:
  - **USB-C** (5V) - Easiest for development
  - **LiPo Battery** - JST connector for 3.7V LiPo battery
    - Built-in TP4054 charging circuit
    - Always-on runtime: ~17 hours (80mA average, depends on battery capacity)
    - With deep sleep: Extended battery life (mostly in 10µA sleep mode)
  - **5V pin** - External 5V power supply
  - **3.3V pin** - External 3.3V regulated supply

### Deep Sleep Power Management

The tilt switch enables automatic power management:
- **Awake mode:** Device boots, connects to WiFi, ready for RFID scans (~80mA)
- **Deep sleep:** Ultra-low power mode after 2 minutes of inactivity (~10µA)
- **Wake-up:** Tilt/shake device to wake from sleep (3-5 seconds to reconnect WiFi)

**Important:** During deep sleep, the device will appear "unavailable" in Home Assistant. This is normal and expected behavior.

- Built-in battery management (TP4054):
  - 500mA charging current
  - Over-charge protection
  - Battery voltage monitoring available

- Current requirements:
  - ESP32 idle: ~80mA
  - ESP32 WiFi/BLE active: ~240mA
  - RC522 idle: ~13mA
  - RC522 reading: ~26mA
  - **Total peak**: ~270mA

A standard USB power supply (500mA+) is sufficient for development. For portable use, a 1000-2000mAh LiPo battery provides several hours of operation with deep sleep extending this significantly.

## Optional: External LED Indicator

If you want an external status LED instead of the built-in blue LED:

```
     TTGO LoRa32 V2.0
    ┌────────────┐
    │  GPIO17    ├────┬── Anode (+) of LED
    │            │    │
    │            │  ┌─┴─┐
    │            │  │LED│ (+ resistor in series)
    │            │  └─┬─┘
    │  GND       ├────┴── Cathode (-) of LED
    └────────────┘

Resistor: 220Ω - 330Ω (for standard 3.3V LED)
```

**Note:** The TTGO LoRa32 V2.0 has an onboard 0.96" OLED display that could also be used for status indication instead of/in addition to LEDs.

## Breadboard Layout Suggestion

```
┌────────────────────────────────────────┐
│  ┌──────────┐      ┌──────────────┐   │
│  │  Green   │      │     Red      │   │
│  │  Button  │      │    Button    │   │
│  └──────────┘      └──────────────┘   │
│                                        │
│          ┌────────────────────┐        │
│          │  TTGO LoRa32 V2.0  │        │
│          │   [OLED Display]   │        │
│          │   [LoRa Antenna]   │        │
│          └────────────────────┘        │
│                                        │
│         ┌──────────────────┐           │
│         │   RC522 Module   │           │
│         │                  │           │
│         └──────────────────┘           │
│                                        │
│         [Place RFID card here]         │
└────────────────────────────────────────┘

Power via USB-C cable or LiPo battery via JST connector
```

## Testing Checklist

1. ✓ RC522 powered with 3.3V (NOT 5V)
2. ✓ All SPI connections secure (GPIO13, GPIO14, GPIO15, GPIO35)
3. ✓ Both buttons connect to GND when pressed
4. ✓ Tilt switch wired to GPIO27 and GND
5. ✓ Buzzer wired to GPIO4 and GND
6. ✓ Vibration motor wired via transistor to GPIO26
7. ✓ TTGO LoRa32 powered via USB-C or LiPo battery
8. ✓ Upload firmware and check logs
9. ✓ Test boot sequence (blue LED blinks fast, hears ascending tones)
10. ✓ Test ready state (blue LED pulses slowly, hears ready chime)
11. ✓ Test RFID reading (hears success tone, feels vibration)
12. ✓ Test buttons by pressing and checking logs
13. ✓ Test tilt switch by tilting device (should wake from sleep)
14. ✓ Verify deep sleep after 2 minutes of inactivity
