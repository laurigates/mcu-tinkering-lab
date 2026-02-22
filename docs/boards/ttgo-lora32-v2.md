# TTGO LoRa32 V2.0

ESP32-based development board with integrated LoRa radio, OLED display, and LiPo battery management.

## Specifications

| Feature | Details |
|---------|---------|
| MCU | ESP32-PICO-D4 (dual-core 240MHz) |
| Flash | 4MB |
| PSRAM | None |
| LoRa | Semtech SX1276 (868/915MHz) |
| Display | 0.96" OLED SSD1306 (128x64, I2C) |
| USB | USB-C with CP2102 or CH9102 serial |
| Battery | JST 1.25mm connector, TP4054 charger |
| Antenna | U.FL/IPEX connector for LoRa |

## Pinout

```
                    ┌─────────────────────────────┐
                    │   TTGO LoRa32 V2.0 ESP32    │
                    │      ┌───────────┐          │
                    │      │   OLED    │          │
                    │      │  128x64   │          │
                    │      └───────────┘          │
                    └─────────────────────────────┘

Left Header                              Right Header
┌────────────────────────┐              ┌────────────────────────┐
│ OLED_SDA    GPIO21  21 │              │ 36  GPIO36  S_VP       │
│ U0_TXD      GPIO01 TXD │              │ 39  GPIO39  S_VN       │
│ U0_RXD      GPIO03 RXD │              │ RST                    │
│ OLED_SCL    GPIO22  22 │              │ 34  GPIO34             │
│ LoRa_MISO   GPIO19  19 │              │ 35  GPIO35             │
│             LoRa_DIO2  │              │ 32  GPIO32  XTAL32     │
│             LoRa_DIO1  │              │ 33  GPIO33  XTAL33     │
│ LoRa_DIO0   GPIO26  26 │              │ 25  GPIO25  DAC2       │
│             GPIO23  23 │              │ 27  GPIO27  LoRa_MOSI  │
│ LoRa_CS     GPIO18  18 │              │ 14  GPIO14             │
│ LoRa_SCK    GPIO05   5 │              │ 12  GPIO12             │
│             GPIO13  13 │              │ GND                    │
│ HSPI_CS0    GPIO15  15 │              │ 5V                     │
│             GPIO02   2 │              │                        │
│             GPIO04   4 │              │                        │
│             GPIO00   0 │              │                        │
│             3.3V       │              │                        │
│             GND        │              │                        │
└────────────────────────┘              └────────────────────────┘
```

## Reserved Pins (Do Not Use)

### LoRa SX1276 (Always Connected)

| Function | GPIO | Notes |
|----------|------|-------|
| SCK | GPIO5 | SPI clock |
| CS | GPIO18 | Chip select |
| MISO | GPIO19 | SPI data in |
| MOSI | GPIO27 | SPI data out |
| DIO0 | GPIO26 | Interrupt |
| RST | GPIO14 | Reset |

### OLED SSD1306 (I2C)

| Function | GPIO |
|----------|------|
| SDA | GPIO21 |
| SCL | GPIO22 |

### Other Reserved

| Function | GPIO | Notes |
|----------|------|-------|
| Built-in LED | GPIO25 | Blue LED, active LOW (inverted) |
| USB TX | GPIO1 | Serial communication |
| USB RX | GPIO3 | Serial communication |

## Available GPIOs

### Fully Available (Recommended)

| GPIO | Features | Notes |
|------|----------|-------|
| GPIO4 | ADC2_0, Touch0 | Safe for general use |
| GPIO13 | ADC2_4, Touch4, HSPI_MOSI | Safe for general use |
| GPIO32 | ADC1_4, Touch9, XTAL32 | RTC-capable (deep sleep wake) |
| GPIO33 | ADC1_5, Touch8, XTAL33 | RTC-capable (deep sleep wake) |
| GPIO34 | ADC1_6 | **Input only** |
| GPIO35 | ADC1_7 | **Input only** |
| GPIO36 | ADC1_0, S_VP | **Input only** |
| GPIO39 | ADC1_3, S_VN | **Input only** |

### Available with Caveats

| GPIO | Features | Caveats |
|------|----------|---------|
| GPIO2 | ADC2_2, Touch2 | Strapping pin - must be LOW or floating at boot |
| GPIO12 | ADC2_5, Touch5, HSPI_MISO | Strapping pin - must be LOW at boot (flash voltage) |
| GPIO15 | ADC2_3, Touch3, HSPI_CS0 | Strapping pin - controls boot messages |
| GPIO23 | V_SPI_D | Available but on VSPI bus |

### Not Exposed on Headers

These GPIOs exist on ESP32 but are **not broken out** on TTGO LoRa32 V2.0:

- GPIO6-11 (connected to internal flash)
- GPIO16, GPIO17

## SPI Bus Conflict

The onboard LoRa SX1276 chip occupies the **VSPI bus**. Even if you're not using LoRa, these pins have the chip physically connected and will cause bus contention.

### Problem

Using GPIO5, GPIO18, GPIO19, or GPIO27 for external SPI devices will conflict with the LoRa chip, causing communication failures.

### Solution

Use the **HSPI bus** for external SPI devices:

| HSPI Function | GPIO |
|---------------|------|
| CLK | GPIO14 |
| MISO | GPIO12 |
| MOSI | GPIO13 |
| CS | GPIO15 (or any available GPIO) |

### ESPHome SPI Configuration

```yaml
# Correct - uses HSPI, avoids LoRa conflict
spi:
  id: hspi_bus
  clk_pin: GPIO14
  miso_pin: GPIO12
  mosi_pin: GPIO13
  interface: hardware

my_spi_device:
  spi_id: hspi_bus
  cs_pin: GPIO15
```

```yaml
# WRONG - conflicts with onboard LoRa chip
spi:
  clk_pin: GPIO18  # LoRa CS!
  miso_pin: GPIO19  # LoRa MISO!
  mosi_pin: GPIO23
```

## Strapping Pins

These pins affect boot behavior if pulled HIGH or LOW externally:

| GPIO | Function | Safe State at Boot |
|------|----------|-------------------|
| GPIO0 | Boot mode | HIGH (or floating) for normal boot |
| GPIO2 | Boot mode | LOW or floating |
| GPIO5 | SDIO timing | HIGH (has internal pull-up) |
| GPIO12 | Flash voltage | **LOW** - HIGH selects 1.8V flash (will fail!) |
| GPIO15 | Boot messages | HIGH enables UART boot log |

**GPIO12 is critical** - if pulled HIGH at boot, ESP32 configures flash for 1.8V which causes boot failure on most boards.

## Deep Sleep

RTC-capable GPIOs that can wake from deep sleep:

- GPIO0, GPIO2, GPIO4
- GPIO12, GPIO13, GPIO14, GPIO15
- GPIO25, GPIO26, GPIO27
- GPIO32, GPIO33, GPIO34, GPIO35, GPIO36, GPIO39

Example wake on GPIO27 (tilt switch):
```yaml
deep_sleep:
  wakeup_pin: GPIO27
  wakeup_pin_mode: INVERT_WAKEUP  # Wake when pin goes LOW
```

## Power

### USB Power
- USB-C connector provides 5V
- Onboard regulator provides 3.3V to ESP32

### Battery Power
- JST 1.25mm 2-pin connector
- TP4054 charge controller (500mA charge current)
- Supports 3.7V single-cell LiPo
- Automatic switchover between USB and battery

### Current Consumption

| State | Current |
|-------|---------|
| Active (WiFi) | ~80-240mA |
| Light sleep | ~0.8mA |
| Deep sleep | ~10µA |

## Board Definition in ESPHome

```yaml
esp32:
  # Option 1: Specific board (may auto-init LoRa SPI)
  board: ttgo-lora32-v2

  # Option 2: Generic (recommended if not using LoRa)
  board: esp32dev

  framework:
    type: esp-idf  # or arduino
```

Use `esp32dev` if you experience SPI conflicts or "Device already registered" errors - the specific board definition may initialize the LoRa chip.

## Common Issues

### "Device already registered" on SPI

**Cause:** Board definition initializes LoRa SPI, conflicting with your device.

**Solution:** Use `board: esp32dev` and HSPI pins.

### Boot loops or failure to flash

**Cause:** GPIO12 pulled HIGH by external circuit.

**Solution:** Ensure GPIO12 is LOW or floating at boot. Disconnect external circuits during flashing.

### OLED not working

**Cause:** Wrong I2C address or pins.

**Solution:** OLED is at address `0x3C` on GPIO21 (SDA) / GPIO22 (SCL).

### LoRa not responding

**Cause:** Antenna not connected or wrong frequency variant.

**Solution:** Always connect antenna before powering on. Match frequency to your region (868MHz EU, 915MHz US).

## References

- [LilyGO TTGO LoRa32 GitHub](https://github.com/LilyGO/TTGO-LORA32)
- [ESP32 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf)
- [SX1276 Datasheet](https://www.semtech.com/products/wireless-rf/lora-connect/sx1276)
