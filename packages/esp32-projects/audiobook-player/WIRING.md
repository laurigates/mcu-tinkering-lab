# Wiring Diagram

## Overview

```
                    ┌─────────────────────────────┐
                    │  WEMOS Battery ESP32        │
                    │  (WiFi & Bluetooth)         │
                    └─────────────────────────────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
   ┌────▼─────┐      ┌────▼─────┐      ┌────▼─────┐
   │ RC522    │      │  Green   │      │   Red    │
   │  RFID    │      │  Button  │      │  Button  │
   │  Reader  │      │  (Play)  │      │ (Pause)  │
   └──────────┘      └──────────┘      └──────────┘
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
| **Status LED** | Built-in   | GPIO16  | GPIO16 | On-board LED |

## Power Considerations

- The WEMOS Battery ESP32 can be powered via:
  - **USB** (5V) - Easiest for development
  - **18650 Battery** - Built-in battery holder on back (3000mAh = ~17 hours runtime)
  - **5V pin** - External 5V power supply
  - **3.3V pin** - External 3.3V regulated supply

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
4. ✓ WEMOS ESP32 powered via USB or 18650 battery
5. ✓ Upload firmware and check logs
6. ✓ Test RFID reading by scanning a tag
7. ✓ Test buttons by pressing and checking logs
