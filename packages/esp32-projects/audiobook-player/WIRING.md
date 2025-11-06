# Wiring Diagram

## Overview

```
                    ┌─────────────────┐
                    │  Lolin D1 Mini  │
                    │    (ESP8266)    │
                    └─────────────────┘
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
RC522               D1 Mini
┌─────────┐        ┌─────────┐
│ SDA(CS) ├────────┤ D8      │ GPIO15
│ SCK     ├────────┤ D5      │ GPIO14
│ MOSI    ├────────┤ D7      │ GPIO13
│ MISO    ├────────┤ D6      │ GPIO12
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
     D1 Mini
    ┌────────┐
    │  D1    ├──────┬─── One side of button
    │ (GPIO5)│      │
    │        │    ┌─┴─┐
    │        │    │   │  Green Button
    │        │    └─┬─┘
    │  GND   ├──────┴─── Other side of button
    └────────┘
```

### Red Button (Pause)

```
     D1 Mini
    ┌────────┐
    │  D2    ├──────┬─── One side of button
    │ (GPIO4)│      │
    │        │    ┌─┴─┐
    │        │    │   │  Red Button
    │        │    └─┬─┘
    │  GND   ├──────┴─── Other side of button
    └────────┘
```

**Note:** When the button is pressed, it connects the GPIO pin to GND. The internal pull-up resistor keeps the pin HIGH when the button is not pressed.

## Complete Pin Summary

| Component | Component Pin | D1 Mini Pin | GPIO | Notes |
|-----------|--------------|-------------|------|-------|
| **RC522** | SDA (CS)     | D8          | GPIO15 | SPI Chip Select |
| **RC522** | SCK          | D5          | GPIO14 | SPI Clock |
| **RC522** | MOSI         | D7          | GPIO13 | SPI Data Out |
| **RC522** | MISO         | D6          | GPIO12 | SPI Data In |
| **RC522** | IRQ          | -           | -      | Not used |
| **RC522** | GND          | GND         | -      | Ground |
| **RC522** | RST          | RST or 3.3V | -      | Reset |
| **RC522** | 3.3V         | 3.3V        | -      | **3.3V only!** |
| **Green Button** | One side | D1      | GPIO5  | Play button |
| **Green Button** | Other side | GND   | -      | Ground |
| **Red Button** | One side   | D2      | GPIO4  | Pause button |
| **Red Button** | Other side | GND     | -      | Ground |
| **Status LED** | Built-in   | D4      | GPIO2  | On-board LED |

## Power Considerations

- The D1 Mini can be powered via:
  - **USB** (5V) - Easiest for development
  - **5V pin** - External 5V power supply
  - **3.3V pin** - External 3.3V regulated supply

- Current requirements:
  - D1 Mini idle: ~80mA
  - D1 Mini WiFi active: ~170mA
  - RC522 idle: ~13mA
  - RC522 reading: ~26mA
  - **Total peak**: ~200mA

A standard USB power supply (500mA+) is sufficient.

## Optional: External LED Indicator

If you want an external status LED instead of the built-in one:

```
     D1 Mini
    ┌────────┐
    │  D4    ├────┬── Anode (+) of LED
    │ (GPIO2)│    │
    │        │  ┌─┴─┐
    │        │  │LED│ (+ resistor in series)
    │        │  └─┬─┘
    │  GND   ├────┴── Cathode (-) of LED
    └────────┘

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
│              ┌──────────┐              │
│              │  D1 Mini │              │
│              │          │              │
│              └──────────┘              │
│                                        │
│         ┌──────────────────┐           │
│         │   RC522 Module   │           │
│         │                  │           │
│         └──────────────────┘           │
│                                        │
│         [Place RFID card here]         │
└────────────────────────────────────────┘

Power via USB cable connected to D1 Mini
```

## Testing Checklist

1. ✓ RC522 powered with 3.3V (NOT 5V)
2. ✓ All SPI connections secure (D5, D6, D7, D8)
3. ✓ Both buttons connect to GND when pressed
4. ✓ D1 Mini powered via USB
5. ✓ Upload firmware and check logs
6. ✓ Test RFID reading by scanning a tag
7. ✓ Test buttons by pressing and checking logs
