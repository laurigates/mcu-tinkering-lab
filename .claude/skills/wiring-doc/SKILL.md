---
name: wiring-doc
description: Generate a standardized WIRING.md for an ESP32 project by analyzing source code pin assignments
argument-hint: <project-path>
allowed-tools: Read, Write, Edit, Grep, Glob, Agent
---

## Task

Generate or update a `WIRING.md` file for the project at `$1` (relative path under `packages/<domain>/`).

## Process

1. **Read the project source code** to discover pin assignments, components, and peripherals:
   - `main/*.c` and `main/*.h` — look for `#define` pin assignments, `gpio_config()`, `i2c_config()`, `spi_bus_config()`, LEDC, UART, ADC usage
   - `main/CMakeLists.txt` — `REQUIRES` list reveals components (mdns, driver, esp_wifi, led_strip, etc.)
   - `sdkconfig.defaults` — enabled features, stack sizes, Bluetooth mode
   - `justfile` — target chip, build variants
   - `README.md` / `CLAUDE.md` — existing hardware description
   - `Kconfig.projbuild` — user-configurable pin assignments

2. **Identify the board** from sdkconfig.defaults (`CONFIG_IDF_TARGET`), CLAUDE.md, or justfile. Note any board-specific constraints (e.g., ESP32-CAM PSRAM conflicts, TTGO LoRa pin reservations, ESP32-S3 USB PHY sharing).

3. **Generate WIRING.md** following the template below. Include ONLY sections relevant to the project — omit empty sections rather than adding placeholder text.

## Template

Follow this exact structure. Every section shown is optional except **Board** and **Pin Assignments** — include a section only when the project uses it.

```markdown
# <Project Name> — Wiring Guide

## Board

<Board name and variant>. Note any special constraints (PSRAM pins, onboard peripherals, USB PHY sharing).

## Pin Assignments

| GPIO | Function | Direction | Component | Notes |
|------|----------|-----------|-----------|-------|

Group related pins together (SPI bus, I2C bus, UART, etc.) with a blank row or subheading between groups.

Direction values: `Output`, `Input`, `Bidirectional`, `—` (for power/ground).

## Wiring Diagram

ASCII art showing physical connections. Keep it compact — one diagram for the whole project if possible.

```
ESP32
 ________
|        |
|  GPIOx |──── Component
|    ... |
|________|
```

## Components

| Component | Specification | Notes |
|-----------|--------------|-------|

Include only components that require wiring. Wireless peripherals (Bluetooth controllers, WiFi) get a one-line mention in Notes, not a table row.

## Bus Configuration

### I2C / SPI / UART

Only include if the project uses these buses. Show address, speed, or baud rate. Note any bus contention issues (e.g., shared SPI with onboard LoRa).

## Power

- Power source (USB, battery, external supply)
- Voltage rails (3.3V, 5V) and which components use each
- Current requirements (peak and typical)
- Deep sleep current if applicable
- Battery management notes if applicable

**Do not include generic ESP32 power specs** — only project-specific information.

## LED / Status Indicators

| State | Color/Pattern | Meaning |
|-------|--------------|---------|

Only include if the project has status LEDs with defined states.

## Safety Notes

Bullet list of project-specific warnings. Examples:
- Voltage level mismatches (5V component on 3.3V GPIO → needs level shifter or voltage divider)
- Components that must NOT receive 5V
- Current limits that require transistor drivers
- Pin conflicts with boot strapping

**Do not include generic electronics safety advice.**

## Troubleshooting

Only include if there are non-obvious failure modes specific to this hardware configuration.
```

## Style Rules

- **Concise over comprehensive** — a 50-line WIRING.md that covers all connections beats a 300-line tutorial
- **No generic content** — omit GPIO reference tables, "safe pin" lists, breadboard ASCII art, expansion ideas, component substitutions, and enclosure advice unless directly relevant
- **Pin numbers are authoritative** — derive from source code `#define` values, not README prose. If they conflict, flag the discrepancy
- **ASCII diagrams** over Mermaid — renders everywhere without a renderer
- **Decimal GPIO numbers** — use `GPIO4` not `IO4` or `Pin 4`
- **One table for all pins** — don't split across multiple tables unless bus grouping genuinely aids readability
- **Notes column** — use for pull-up/down requirements, strapping pin warnings, "input-only" constraints, or alternative pin options
