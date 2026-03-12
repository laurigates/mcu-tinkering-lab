# ESP32-C3 Super Mini

Ultra-compact RISC-V development board with Wi-Fi and Bluetooth LE in a 22.5 x 18 mm package. One of the cheapest and smallest ESP32 boards available (~$2–3), ideal for space-constrained IoT sensor nodes where raw performance and GPIO count are not critical.

## Why ESP32-C3 Super Mini

- **Tiny and cheap** — 22.5 x 18 mm, around $2–3 per board. Easy to embed inside enclosures, behind wall plates, or into wearables.
- **Native USB** — Built-in USB-Serial-JTAG means no external USB-UART bridge chip (no CH340/CP2102). Fewer components, simpler design.
- **Low power** — ~43 µA deep sleep (datasheet spec). Good for battery-powered sensors that wake periodically.
- **RISC-V** — Single-core RISC-V at 160 MHz. Plenty for sensor reading, protocol conversion, and lightweight control logic.
- **Wi-Fi + BLE** — 802.11 b/g/n and Bluetooth 5.0 LE on one chip. Sufficient for most IoT tasks.

### When to pick something else

| Need | Better choice |
|------|---------------|
| Zigbee / Thread / Matter | [XIAO ESP32-C6](xiao-esp32c6.md) |
| Wi-Fi 6 | [XIAO ESP32-C6](xiao-esp32c6.md) |
| USB OTG / HID emulation | [ESP32-S3](esp32-s3.md) |
| AI/ML inference / camera | [ESP32-S3](esp32-s3.md) |
| More GPIOs (>11) | ESP32-S3 DevKitC or full-size ESP32 |
| LoRa radio | [TTGO LoRa32](ttgo-lora32-v2.md) |
| Classic Bluetooth (A2DP, SPP) | Original ESP32 |

## Specifications

| Feature | Details |
|---------|---------|
| MCU | ESP32-C3FN4 — RISC-V single-core @ 160 MHz |
| SRAM | 400 KB |
| Flash | 4 MB |
| PSRAM | None |
| Wi-Fi | 802.11 b/g/n (Wi-Fi 4, 2.4 GHz) |
| Bluetooth | Bluetooth 5.0 LE (no Classic BT) |
| USB | USB-C (USB-Serial-JTAG, **not** USB OTG) |
| GPIO | 11 usable pins |
| ADC | 2× 12-bit SAR ADC, 6 channels (A0–A5) |
| PWM | 6 channels (LEDC, any GPIO) |
| SPI | 1× user-accessible |
| I2C | 1× (SDA: GPIO8, SCL: GPIO9) |
| UART | 1× (TX: GPIO21, RX: GPIO20) |
| Antenna | PCB trace antenna (no U.FL connector) |
| Onboard LED | Blue LED on GPIO8 (active LOW / inverted) |
| Buttons | BOOT (GPIO9), EN (reset) |
| Dimensions | 22.5 x 18 mm |
| Operating temp | -40°C to 85°C |

## Pinout

```
           ┌───────────────────┐
           │  ESP32-C3         │
           │  Super Mini       │
           │                   │
           │  [EN]    [BOOT]   │
           ├──┬────────────┬───┤
   5V      │● │            │ ●│  GPIO0 / A0
   GND     │● │            │ ●│  GPIO1 / A1
   3V3     │● │            │ ●│  GPIO2 / A2 (strapping)
   GPIO10  │● │            │ ●│  GPIO3 / A3
   GPIO21 / TX │● │        │ ●│  GPIO4 / A4 (JTAG)
   GPIO20 / RX │● │        │ ●│  GPIO5 / A5 (JTAG)
   GPIO9 / SCL │● │        │ ●│  GPIO6 / MISO (JTAG)
   GPIO8 / SDA │● │        │ ●│  GPIO7 / MOSI (JTAG)
           ├──┴────────────┴───┤
           │     [USB-C]       │
           └───────────────────┘
```

### Pin Mapping

| Pin | GPIO | Default Function | Alternate Functions | Notes |
|-----|------|------------------|---------------------|-------|
| A0 | GPIO0 | Analog input | ADC1_CH0 | Safe for general use |
| A1 | GPIO1 | Analog input | ADC1_CH1 | Safe for general use |
| A2 | GPIO2 | Analog input | ADC1_CH2 | Strapping pin — avoid pulling low at boot |
| A3 | GPIO3 | Analog input | ADC1_CH3 | Safe for general use |
| A4 | GPIO4 | Analog / JTAG TMS | ADC1_CH4 | JTAG by default |
| A5 | GPIO5 | Analog / JTAG TDI | ADC2_CH0 | JTAG; ADC2 limited with Wi-Fi active |
| MISO | GPIO6 | SPI MISO / JTAG TCK | — | JTAG by default |
| MOSI | GPIO7 | SPI MOSI / JTAG TDO | — | JTAG by default |
| SDA | GPIO8 | I2C SDA | Onboard LED | Strapping pin; LED is active LOW |
| SCL | GPIO9 | I2C SCL | BOOT button | Strapping pin; pulled HIGH internally |
| — | GPIO10 | SPI SCK | — | Safe for general use |
| RX | GPIO20 | UART RX | — | Default serial console |
| TX | GPIO21 | UART TX | — | Default serial console |

### Safe Pins for General Use

GPIO0, GPIO1, GPIO3, GPIO10 — no boot sequence involvement, no JTAG, no onboard peripherals. Use these first.

### Strapping Pins (use with caution)

| GPIO | Boot Function | Required State |
|------|---------------|----------------|
| GPIO2 | Boot mode | Must be HIGH or floating for normal boot |
| GPIO8 | Flash voltage / boot mode | Must be HIGH for normal boot (pulled up by LED circuit) |
| GPIO9 | Boot mode | HIGH = normal boot, LOW = download mode (BOOT button) |

### JTAG Pins

GPIO4, GPIO5, GPIO6, GPIO7 are JTAG by default. They can be reclaimed for general use by disabling JTAG in software, but external peripherals on these pins may interfere with boot/debug.

### Shared Pin: GPIO8

GPIO8 serves triple duty: I2C SDA, onboard blue LED, and strapping pin. If using I2C on the default pins, the LED will flicker during data transfer. Consider remapping I2C to other pins if LED behavior is distracting, or desolder the LED.

## Power Consumption

| State | Current | Notes |
|-------|---------|-------|
| Active (Wi-Fi TX) | ~300 mA peak | Depends on TX power |
| Active (Wi-Fi RX) | ~90 mA | |
| Modem sleep | ~20 mA | CPU active, radio off |
| Light sleep | ~0.8 mA | |
| Deep sleep (with LED) | ~600 µA | Onboard power LED draws current |
| Deep sleep (LED removed) | ~43 µA | Datasheet spec achieved |

**Deep sleep caveat:** The onboard power LED stays on during deep sleep, drawing ~550 µA. For true low-power operation, desolder the power LED or cut its trace. The blue status LED on GPIO8 does not affect deep sleep since the GPIO is tristated.

## Known Issues and Quirks

### Wi-Fi Antenna Quality Varies

Some batches have a PCB antenna layout defect. Check the spacing of the capacitor near the antenna trace:
- **Good boards:** ~3.3 mm from processor corner to capacitor
- **Defective boards:** ~1.5 mm spacing — reduced Wi-Fi range

If Wi-Fi range is poor, try elevating the board 15+ mm above a breadboard using stacking headers — the metal breadboard rails can block the antenna.

### Upload Requires Boot Mode Entry

Some boards (especially early batches) require manual bootloader entry for each upload:
1. Hold **BOOT** button
2. Press and release **EN** (reset) while holding BOOT
3. Release **BOOT**
4. Upload firmware

Newer firmware/bootloader versions may auto-enter download mode, but if uploads fail, try this sequence.

### Serial Monitor After Deep Sleep

The USB-Serial-JTAG connection drops when the chip enters deep sleep. The serial monitor will not automatically reconnect on wake. Press the reset button or power-cycle to re-establish the connection.

### Board Selection in Arduino IDE

The board shows up under several names depending on the board package version:
- `ESP32C3 Dev Module` (most reliable)
- `Nologo ESP32C3 Super Mini`
- `XIAO_ESP32C3` (works but not the same board)

Enable **USB CDC on Boot** in the Arduino IDE Tools menu for serial output to work over the USB-C port.

## Security Features

Same silicon as all ESP32-C3 variants:
- **Secure Boot v2** — RSA-3072 or ECDSA signature verification
- **Flash Encryption** — AES-128/256-XTS
- **Digital Signature** — Hardware RSA/ECDSA acceleration
- **HMAC** — Hardware HMAC peripheral

## ESP-IDF Configuration

### sdkconfig.defaults for C3 Super Mini

```ini
# Target
CONFIG_IDF_TARGET="esp32c3"

# Use native USB for console
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y

# Low power (for battery projects)
CONFIG_PM_ENABLE=y
CONFIG_FREERTOS_USE_TICKLESS_IDLE=y
```

### CMake target

```bash
idf.py set-target esp32c3
```

## ESP32-C3 vs C6 vs S3 Quick Comparison

| Feature | C3 Super Mini | [XIAO C6](xiao-esp32c6.md) | [ESP32-S3](esp32-s3.md) |
|---------|---------------|------------|-----------|
| CPU | RISC-V 1-core 160 MHz | RISC-V 1+LP core 160 MHz | Xtensa LX7 2-core 240 MHz |
| SRAM | 400 KB | 512 KB | 512 KB |
| Flash | 4 MB | 4 MB | 4–32 MB |
| Wi-Fi | Wi-Fi 4 | Wi-Fi 6 | Wi-Fi 4 |
| Bluetooth | BLE 5.0 | BLE 5.3 | BLE 5.0 |
| Zigbee/Thread | No | Yes | No |
| USB | Serial-JTAG | Serial-JTAG | Full USB OTG |
| AI/ML accel | No | No | Vector instructions |
| PSRAM | None | None | Up to 16 MB |
| Deep sleep | ~43 µA | ~15 µA | ~7 µA |
| Size | 22.5 x 18 mm | 21 x 17.8 mm | Varies by board |
| Price | ~$2–3 | ~$5 | ~$8–14 |
| Best for | Cheap IoT nodes | Matter/Thread, Wi-Fi 6 | AI/ML, cameras, USB |

## Use Cases

- **Wi-Fi temperature/humidity sensor** — Read DHT22/BME280 via GPIO, deep sleep between readings, publish via MQTT
- **Smart plug/relay controller** — GPIO drives a relay module, controlled via MQTT or HTTP
- **BLE beacon/scanner** — Broadcast or scan BLE advertisements for presence detection
- **ESP-NOW mesh node** — Lightweight peer-to-peer communication without a router
- **LED strip controller** — Drive WS2812/SK6812 via RMT peripheral

## In This Repository

The ESP32-C3 Super Mini is not currently used in any project. It is a candidate for:
- Low-cost sensor nodes that report to the robocar's MQTT broker
- ESP-NOW mesh expansion for multi-room presence detection
- Disposable/embedable IoT endpoints where board cost matters

## References

- [ESP32-C3 Super Mini Pinout & Specs (espboards.dev)](https://www.espboards.dev/esp32/esp32-c3-super-mini/)
- [Getting Started with ESP32-C3 Super Mini (Random Nerd Tutorials)](https://randomnerdtutorials.com/getting-started-esp32-c3-super-mini/)
- [ESP32-C3 Series Datasheet (Espressif)](https://www.espressif.com/sites/default/files/documentation/esp32-c3_datasheet_en.pdf)
- [ESP32-C3 Super Mini Pinout Guide (Studio Pieters)](https://www.studiopieters.nl/esp32-c3-super-mini-pinout/)
- [ESP32-C3 Super Mini High-Resolution Pinout (Mischianti)](https://mischianti.org/esp32-c3-super-mini-high-resolution-pinout-datasheet-and-specs/)
