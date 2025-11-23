# Wiring Diagram

## Overview

```
                    ┌─────────────────────────────┐
                    │  WEMOS Battery ESP32        │
                    │  (WiFi & Bluetooth)         │
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

The RC522 uses SPI communication protocol:

```
RC522               WEMOS ESP32
┌─────────┐        ┌─────────┐
│ SDA(CS) ├────────┤ GPIO17  │ SPI CS
│ SCK     ├────────┤ GPIO18  │ SPI Clock
│ MOSI    ├────────┤ GPIO23  │ SPI MOSI
│ MISO    ├────────┤ GPIO19  │ SPI MISO
│ IRQ     │        │         │ Not connected
│ GND     ├────────┤ GND     │
│ RST     ├────────┤ RST     │ (or 3.3V for normal operation)
│ 3.3V    ├────────┤ 3.3V    │
└─────────┘        └─────────┘
```

**⚠️ Important:** The RC522 operates at **3.3V only**. Do not connect to 5V!

## Buttons

Both buttons use the same wiring pattern with internal pull-up resistors:

### Green Button (Play)

```
     WEMOS ESP32
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
     WEMOS ESP32
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
     WEMOS ESP32
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
     WEMOS ESP32
    ┌────────────┐
    │  GPIO25    ├──────────── + Buzzer
    │            │
    │  GND       ├──────────── - Buzzer
    └────────────┘
```

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
     WEMOS ESP32                    Motor Circuit
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

| Component | Component Pin | ESP32 Pin | GPIO | Notes |
|-----------|--------------|-----------|------|-------|
| **RC522** | SDA (CS)     | GPIO17    | GPIO17 | SPI Chip Select (changed from GPIO5 to avoid strapping pin) |
| **RC522** | SCK          | GPIO18    | GPIO18 | SPI Clock |
| **RC522** | MOSI         | GPIO23    | GPIO23 | SPI Data Out |
| **RC522** | MISO         | GPIO19    | GPIO19 | SPI Data In |
| **RC522** | IRQ          | -         | -      | Not used |
| **RC522** | GND          | GND       | -      | Ground |
| **RC522** | RST          | RST or 3.3V | -    | Reset |
| **RC522** | 3.3V         | 3.3V      | -      | **3.3V only!** |
| **Green Button** | One side | GPIO32  | GPIO32 | Play button |
| **Green Button** | Other side | GND   | -      | Ground |
| **Red Button** | One side   | GPIO33  | GPIO33 | Pause button |
| **Red Button** | Other side | GND     | -      | Ground |
| **Tilt Switch** | One side   | GPIO27  | GPIO27 | Wake from deep sleep (RTC-capable GPIO) |
| **Tilt Switch** | Other side | GND     | -      | Ground |
| **Buzzer** | +          | GPIO25  | GPIO25 | Passive piezo buzzer for tones |
| **Buzzer** | -          | GND     | -      | Ground |
| **Vibration Motor** | Control | GPIO26 | GPIO26 | Via NPN transistor (2N2222) |
| **Vibration Motor** | Power   | 3.3V    | -      | Motor positive |
| **Vibration Motor** | Ground  | GND     | -      | Via transistor emitter |
| **Status LED** | Built-in   | GPIO16  | GPIO16 | On-board LED |

## Power Considerations

- The WEMOS Battery ESP32 can be powered via:
  - **USB** (5V) - Easiest for development
  - **18650 Battery** - Built-in battery holder on back
    - Always-on runtime: ~17 hours (80mA average)
    - With deep sleep: ~29 days (mostly in 10µA sleep mode)
  - **5V pin** - External 5V power supply
  - **3.3V pin** - External 3.3V regulated supply

### Deep Sleep Power Management

The tilt switch enables automatic power management:
- **Awake mode:** Device boots, connects to WiFi, ready for RFID scans (~80mA)
- **Deep sleep:** Ultra-low power mode after 2 minutes of inactivity (~10µA)
- **Wake-up:** Tilt/shake device to wake from sleep (3-5 seconds to reconnect WiFi)

**Important:** During deep sleep, the device will appear "unavailable" in Home Assistant. This is normal and expected behavior.

- Built-in battery management:
  - 0.5A charging current
  - 1A output current
  - Over-charge protection
  - Over-discharge protection

- Current requirements:
  - ESP32 idle: ~80mA
  - ESP32 WiFi/BLE active: ~240mA
  - RC522 idle: ~13mA
  - RC522 reading: ~26mA
  - **Total peak**: ~270mA

A standard USB power supply (500mA+) is sufficient for development. For portable use, a 3000mAh 18650 battery provides approximately 17 hours of operation.

## Optional: External LED Indicator

If you want an external status LED instead of the built-in one:

```
     WEMOS ESP32
    ┌────────────┐
    │  GPIO16    ├────┬── Anode (+) of LED
    │            │    │
    │            │  ┌─┴─┐
    │            │  │LED│ (+ resistor in series)
    │            │  └─┬─┘
    │  GND       ├────┴── Cathode (-) of LED
    └────────────┘

Resistor: 220Ω - 330Ω (for standard 3.3V LED)
```

## Breadboard Layout Suggestion

```
┌────────────────────────────────────────┐
│  ┌──────────┐      ┌──────────────┐   │
│  │  Green   │      │     Red      │   │
│  │  Button  │      │    Button    │   │
│  └──────────┘      └──────────────┘   │
│                                        │
│          ┌────────────────────┐        │
│          │   WEMOS ESP32      │        │
│          │   (Battery)        │        │
│          └────────────────────┘        │
│                                        │
│         ┌──────────────────┐           │
│         │   RC522 Module   │           │
│         │                  │           │
│         └──────────────────┘           │
│                                        │
│         [Place RFID card here]         │
└────────────────────────────────────────┘

Power via USB cable or 18650 battery in rear holder
```

## Testing Checklist

1. ✓ RC522 powered with 3.3V (NOT 5V)
2. ✓ All SPI connections secure (GPIO18, GPIO19, GPIO23, GPIO17)
3. ✓ Both buttons connect to GND when pressed
4. ✓ Tilt switch wired to GPIO27 and GND
5. ✓ Buzzer wired to GPIO25 and GND
6. ✓ Vibration motor wired via transistor to GPIO26
7. ✓ WEMOS ESP32 powered via USB or 18650 battery
8. ✓ Upload firmware and check logs
9. ✓ Test boot sequence (LED blinks fast, hears ascending tones)
10. ✓ Test ready state (LED pulses slowly, hears ready chime)
11. ✓ Test RFID reading (hears success tone, feels vibration)
12. ✓ Test buttons by pressing and checking logs
13. ✓ Test tilt switch by tilting device (should wake from sleep)
14. ✓ Verify deep sleep after 2 minutes of inactivity
