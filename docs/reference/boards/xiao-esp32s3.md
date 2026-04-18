# Seeed Studio XIAO ESP32-S3

Thumb-sized dual-core ESP32-S3 board with 8 MB octal PSRAM, 8 MB flash, native USB OTG, and Wi-Fi + BLE 5.0 in a 21 x 17.8 mm package. Packs the full ESP32-S3 feature set — vector instructions for AI/ML, camera DVP interface, USB HID — into the compact XIAO form factor at ~$8.

## Why XIAO ESP32-S3

- **Dual-core + AI acceleration** — Xtensa LX7 at 240 MHz with vector instructions for TensorFlow Lite Micro inference (keyword spotting, face detection) without an external accelerator.
- **8 MB octal PSRAM** — Enough for camera frame buffers, audio processing, or ML model weights. On a separate SPI bus from user-facing SPI, so no GPIO conflicts.
- **USB OTG** — Native USB 1.1 host/device via TinyUSB. HID keyboards, CDC-ACM serial, MSC storage — no external USB-UART bridge.
- **Ultra-compact** — 21 x 17.8 mm with castellated pads on the bottom. Fits inside enclosures, on carrier boards, and in wearables.
- **Battery-friendly** — 14 µA deep sleep, onboard LiPo charge management (50–100 mA), charge indicator LED.
- **~$8 USD** — One of the cheapest ways to get an ESP32-S3 with 8 MB PSRAM.

### When to pick something else

| Need | Better choice |
|------|---------------|
| Camera + microphone built in | [XIAO ESP32-S3 Sense](xiao-esp32s3-sense.md) (same board + expansion) |
| More GPIOs (>11) | [XIAO ESP32-S3 Plus](xiao-esp32s3-plus.md) (18 GPIOs) or ESP32-S3 DevKitC |
| Wi-Fi 6 / Thread / Zigbee / Matter | [XIAO ESP32-C6](xiao-esp32c6.md) |
| Classic Bluetooth (A2DP, SPP) | Original ESP32 |
| LoRa radio | [TTGO LoRa32](ttgo-lora32-v2.md) |
| Cheaper disposable IoT node | [ESP32-C3 Super Mini](esp32-c3-super-mini.md) (~$2–3) |
| PIO state machines / RISC-V | [XIAO RP2350](xiao-rp2350.md) |

## Specifications

| Feature | Details |
|---------|---------|
| MCU | ESP32-S3R8 — Xtensa LX7 dual-core @ 240 MHz |
| SRAM | 512 KB |
| Flash | 8 MB (quad SPI) |
| PSRAM | 8 MB (octal SPI, separate bus) |
| Wi-Fi | 802.11 b/g/n (Wi-Fi 4, 2.4 GHz) |
| Bluetooth | Bluetooth 5.0 LE (no Classic BT) |
| USB | USB-C (USB 1.1 OTG + USB-Serial-JTAG) |
| GPIO | 11 usable pins (D0–D10) |
| ADC | 9 channels (12-bit SAR) |
| Touch | 9 capacitive touch pins (A0–A5, A8–A10) |
| PWM | 11 channels (LEDC, any GPIO) |
| SPI | 1× user-accessible (SCK/MISO/MOSI on D8/D9/D10) |
| I2C | 1× (SDA: GPIO5/D4, SCL: GPIO6/D5) |
| UART | 1× (TX: GPIO43/D6, RX: GPIO44/D7) |
| Antenna | U.FL connector with detachable external antenna |
| Battery | 3.7V LiPo via bottom pads, 50–100 mA charge IC |
| Onboard LED | User LED on GPIO21 |
| Buttons | BOOT, RESET |
| Dimensions | 21 x 17.8 mm |
| Operating temp | -40°C to 65°C |

## Pinout

```
           ┌──────────────────────┐
           │  XIAO ESP32-S3       │
           │                      │
           │        [antenna]     │
           │                 [U.FL]│
           │  [RST]    [BOOT]     │
           ├──┬───────────────┬───┤
    D0/A0  │● │               │ ●│  5V
    D1/A1  │● │               │ ●│  GND
    D2/A2  │● │               │ ●│  3V3
    D3/A3  │● │               │ ●│  D10/MOSI
    D4/SDA │● │               │ ●│  D9/MISO
    D5/SCL │● │               │ ●│  D8/SCK
    D6/TX  │● │               │ ●│  D7/RX
           ├──┴───────────────┴───┤
           │      [USB-C]         │
           └──────────────────────┘
```

### Pin Mapping

| XIAO Pin | GPIO | Default Function | Alternate Functions |
|----------|------|------------------|---------------------|
| D0 | GPIO1 | Analog input | ADC1_CH0, Touch1 |
| D1 | GPIO2 | Analog input | ADC1_CH1, Touch2 |
| D2 | GPIO3 | Analog input | ADC1_CH2, Touch3 (strapping) |
| D3 | GPIO4 | Analog input | ADC1_CH3, Touch4 |
| D4 | GPIO5 | I2C SDA | ADC1_CH4, Touch5 |
| D5 | GPIO6 | I2C SCL | ADC1_CH5, Touch6 |
| D6 | GPIO43 | UART TX | — |
| D7 | GPIO44 | UART RX | — |
| D8 | GPIO7 | SPI SCK | ADC1_CH6, Touch7 |
| D9 | GPIO8 | SPI MISO | ADC1_CH7, Touch8 |
| D10 | GPIO9 | SPI MOSI | ADC1_CH8, Touch9 |

### Safe Pins for General Use

GPIO1, GPIO2, GPIO4, GPIO5, GPIO6, GPIO7, GPIO8, GPIO43 — no boot sequence involvement, no flash/PSRAM connections. Use these first for external peripherals.

### Pins to Use with Caution

| GPIO | Issue |
|------|-------|
| GPIO3 | Strapping pin — sampled at reset for JTAG interface selection |
| GPIO9 | Connected to external flash hold signal |
| GPIO10 | Connected to external flash chip select |
| GPIO20 | USB D+ — avoid if using USB OTG |

### USB Pin Sharing

The USB OTG peripheral uses GPIO19 (D-) and GPIO20 (D+). When USB OTG is active (TinyUSB for HID, CDC, etc.), the USB-Serial-JTAG debug interface is unavailable. Use UART on D6/D7 with an external USB-UART adapter for logging in USB OTG mode.

## Power Consumption

| State | Current | Notes |
|-------|---------|-------|
| Active (Wi-Fi TX) | ~310 mA peak | Depends on TX power level |
| Active (Wi-Fi RX) | ~100 mA | |
| Modem sleep | ~30 mA | CPU active, radio off |
| Light sleep | ~5 mA | Auto wake on timer/GPIO |
| Deep sleep | ~14 µA | RTC memory retained |

## Battery Charging

The board has onboard LiPo charge management accessible via solder pads on the bottom:

- Connect a 3.7V lithium battery to the positive/negative pads
- Charges via USB-C at 50–100 mA
- Orange LED indicates charging; LED off when complete
- Read battery voltage via ADC on one of the analog pins with a voltage divider

## Known Issues and Quirks

### USB-Serial-JTAG Reset Behavior

Same ESP32-S3 silicon quirk as other S3 boards: `esptool --after hard_reset` does not reliably trigger a chip reset over USB-Serial-JTAG. After flashing, the board may continue running old firmware until manually reset. See the [ESP32-S3 board doc](esp32-s3.md) for the pyserial reset workaround.

### Pins A11/A12 Lack ADC

GPIO41 and GPIO42 (accessible on the Sense expansion board bottom pads) are assigned as A11/A12 but do not support ADC functionality due to ESP32-S3 chip architecture. They work fine as digital I/O.

### Octal PSRAM Pins Reserved

GPIO33–37 are used internally for the octal PSRAM bus. These pins are not broken out and cannot be used for external peripherals.

## ESP-IDF Configuration

### sdkconfig.defaults for XIAO ESP32-S3

```ini
# Target
CONFIG_IDF_TARGET="esp32s3"

# PSRAM (8 MB octal)
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y

# USB OTG (for HID/CDC projects)
CONFIG_TINYUSB=y
CONFIG_TINYUSB_HID_ENABLED=y

# USB Serial JTAG console (when not using USB OTG)
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y

# CPU frequency
CONFIG_ESP32S3_DEFAULT_CPU_FREQ_240=y
```

### CMake target

```bash
idf.py set-target esp32s3
```

## XIAO ESP32-S3 Family Comparison

| Feature | XIAO S3 (this board) | [XIAO S3 Sense](xiao-esp32s3-sense.md) | [XIAO S3 Plus](xiao-esp32s3-plus.md) |
|---------|---------------------|--------------------------------------|--------------------------------------|
| MCU | ESP32-S3R8 dual-core 240 MHz | Same | Same |
| Flash | 8 MB | 8 MB | 16 MB |
| PSRAM | 8 MB octal | 8 MB octal | 8 MB octal |
| GPIO | 11 (D0–D10) | 11 + 2 bottom pads | 11 + 9 castellated (18 total) |
| Camera | No | OV2640/OV3660 via expansion board | No |
| Microphone | No | PDM digital mic via expansion board | No |
| SD card | No | MicroSD via expansion board | No |
| Size | 21 x 17.8 mm | 21 x 17.8 x 15 mm (with expansion) | 21 x 17.8 mm |
| Price | ~$8 | ~$14–18 | ~$9 |
| Best for | General USB/AI/IoT | Camera + audio projects | GPIO-heavy projects |

## In This Repository

The XIAO ESP32-S3 Sense variant is the target for the unified single-board robocar ([ADR-013](../blueprint/adrs/ADR-013-single-board-xiao-esp32s3.md)). The base XIAO ESP32-S3 is a candidate for:

- USB HID automation projects (alternative to Waveshare ESP32-S3-Zero)
- BLE-to-WiFi gateway nodes
- On-device ML inference (keyword spotting, gesture recognition)
- Compact IoT controllers needing more RAM than ESP32-C3/C6

## References

- [XIAO ESP32-S3 Getting Started (Seeed Studio Wiki)](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)
- [XIAO ESP32-S3 Pin Multiplexing (Seeed Studio Wiki)](https://wiki.seeedstudio.com/xiao_esp32s3_pin_multiplexing/)
- [XIAO ESP32-S3 Product Page](https://www.seeedstudio.com/XIAO-ESP32S3-p-5627.html)
- [ESP32-S3 Series Datasheet (Espressif)](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
- [XIAO ESP32-S3 Pinout & Specs (espboards.dev)](https://www.espboards.dev/esp32/xiao-esp32s3/)
- [XIAO ESP32-S3 Pinout, Datasheet & Guide (components101)](https://components101.com/development-boards/xiao-esp32-s3-sense-datasheet-pinout)
