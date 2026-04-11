# Seeed Studio XIAO ESP32-S3 Plus

An upgraded [XIAO ESP32-S3](xiao-esp32s3.md) with doubled flash (16 MB), 9 additional castellated GPIO pads on the bottom, and a second UART and SPI interface — all in the same 21 x 17.8 mm footprint. Designed for GPIO-constrained projects where the standard XIAO's 11 pins aren't enough but a full-size DevKitC is too big.

## Why XIAO ESP32-S3 Plus

- **18 usable GPIOs** — 11 through-hole pins on the side headers (same as standard XIAO) plus 9 castellated SMD pads on the bottom. Enough for I2C sensors + SPI display + UART peripheral + spare GPIOs simultaneously.
- **16 MB flash** — Double the standard XIAO's 8 MB. Room for large firmware binaries, OTA dual-partition schemes, SPIFFS/LittleFS file systems, or embedded web assets.
- **2× UART, 2× SPI** — The extra bottom pads expose a second hardware UART and second SPI bus, enabling concurrent communication with multiple peripherals without software multiplexing.
- **B2B connector** — Board-to-board connector for carrier board integration, enabling SMD soldering for production-scale assembly.
- **Same compact footprint** — Identical 21 x 17.8 mm dimensions and side-header pinout as the standard XIAO. Drop-in replacement in existing carrier boards; extra GPIOs are a bonus via bottom pads.
- **~$9 USD** — Only ~$1 more than the standard XIAO ESP32-S3 for double the flash and nearly double the GPIOs.

### When to pick something else

| Need | Better choice |
|------|---------------|
| Camera + microphone | [XIAO ESP32-S3 Sense](xiao-esp32s3-sense.md) |
| Fewest possible GPIOs, lowest cost | [XIAO ESP32-S3](xiao-esp32s3.md) (~$8) |
| Wi-Fi 6 / Thread / Zigbee | [XIAO ESP32-C6](xiao-esp32c6.md) |
| 26+ GPIOs | ESP32-S3 DevKitC (full-size) |
| PIO state machines / RISC-V | [XIAO RP2350](xiao-rp2350.md) |
| LoRa radio | [TTGO LoRa32](ttgo-lora32-v2.md) |

## Specifications

| Feature | Details |
|---------|---------|
| MCU | ESP32-S3R8 — Xtensa LX7 dual-core @ 240 MHz |
| SRAM | 512 KB |
| Flash | 16 MB (quad SPI) |
| PSRAM | 8 MB (octal SPI, separate bus) |
| Wi-Fi | 802.11 b/g/n (Wi-Fi 4, 2.4 GHz) |
| Bluetooth | Bluetooth 5.0 LE (no Classic BT) |
| USB | USB-C (USB 1.1 OTG + USB-Serial-JTAG) |
| GPIO | 18 usable (11 through-hole + 9 castellated bottom pads) |
| ADC | 9 channels (12-bit SAR) |
| Touch | 9 capacitive touch pins |
| PWM | 18 channels (LEDC, any GPIO) |
| SPI | 2× (1 on side headers, 1 on bottom pads) |
| I2C | 1× (SDA: GPIO5/D4, SCL: GPIO6/D5) |
| UART | 2× (UART0 on side headers, UART1 on bottom pads) |
| Antenna | U.FL connector with detachable external antenna |
| Battery | 3.7V LiPo via bottom pads, 50–100 mA charge IC |
| Onboard LED | User LED on GPIO21 |
| Buttons | BOOT, RESET |
| B2B connector | Board-to-board for carrier board integration |
| Dimensions | 21 x 17.8 mm |
| Operating temp | -40°C to 65°C |

## Pinout

### Side Headers (same as standard XIAO ESP32-S3)

```
           ┌──────────────────────┐
           │  XIAO ESP32-S3 Plus  │
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

### Side Header Pin Mapping

| XIAO Pin | GPIO | Default Function | Alternate Functions |
|----------|------|------------------|---------------------|
| D0 | GPIO1 | Analog input | ADC1_CH0, Touch1 |
| D1 | GPIO2 | Analog input | ADC1_CH1, Touch2 |
| D2 | GPIO3 | Analog input | ADC1_CH2, Touch3 (strapping) |
| D3 | GPIO4 | Analog input | ADC1_CH3, Touch4 |
| D4 | GPIO5 | I2C SDA | ADC1_CH4, Touch5 |
| D5 | GPIO6 | I2C SCL | ADC1_CH5, Touch6 |
| D6 | GPIO43 | UART0 TX | — |
| D7 | GPIO44 | UART0 RX | — |
| D8 | GPIO7 | SPI0 SCK | ADC1_CH6, Touch7 |
| D9 | GPIO8 | SPI0 MISO | ADC1_CH7, Touch8 |
| D10 | GPIO9 | SPI0 MOSI | ADC1_CH8, Touch9 |

### Bottom Castellated Pads (9 additional GPIOs)

The 9 castellated pads use 1.27 mm pitch and are designed for SMD soldering onto carrier boards.

These extra pads provide a second UART and second SPI bus, plus additional digital I/O. Specific GPIO assignments follow the ESP32-S3's secondary peripheral mapping. Consult the [Seeed Studio Wiki](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) for the definitive bottom-pad pin table.

## Power Consumption

Same ESP32-S3R8 silicon as the standard XIAO ESP32-S3:

| State | Current | Notes |
|-------|---------|-------|
| Active (Wi-Fi TX) | ~310 mA peak | Depends on TX power level |
| Active (Wi-Fi RX) | ~100 mA | |
| Modem sleep | ~30 mA | CPU active, radio off |
| Light sleep | ~5 mA | |
| Deep sleep | ~14 µA | RTC memory retained |

## Use Cases

The Plus variant is strongest when you need the compact XIAO footprint but run out of pins on the standard board:

- **I2C + SPI + UART simultaneously** — Connect an I2C sensor on D4/D5, SPI display on D8–D10, and UART GPS on D6/D7, with the second UART on bottom pads for a secondary serial peripheral.
- **Carrier board integration** — SMD-solder the XIAO onto a custom PCB via castellated holes. The bottom pads provide additional I/O without flying wires.
- **Large firmware projects** — 16 MB flash provides ample space for OTA with dual app partitions, embedded web servers with static assets, or LittleFS data storage.
- **Bluetooth gateway with expansion** — BLE scanning + WiFi MQTT forwarding, with bottom-pad GPIOs driving status LEDs, relays, or additional sensors.

## ESP-IDF Configuration

### sdkconfig.defaults for XIAO ESP32-S3 Plus

```ini
# Target
CONFIG_IDF_TARGET="esp32s3"

# PSRAM (8 MB octal)
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y

# 16 MB flash — adjust partition table accordingly
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y

# USB OTG (for HID/CDC projects)
CONFIG_TINYUSB=y

# CPU frequency
CONFIG_ESP32S3_DEFAULT_CPU_FREQ_240=y
```

### CMake target

```bash
idf.py set-target esp32s3
```

## In This Repository

The XIAO ESP32-S3 Plus is not currently used in any project. It is a candidate for:

- Projects that outgrow the standard XIAO's 11 GPIOs but don't need the Sense expansion board's camera/mic
- Custom carrier board designs for production prototypes
- Projects requiring large firmware binaries or embedded file systems (16 MB flash)

## References

- [XIAO ESP32-S3 Plus Product Page](https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32S3-Plus-p-6361.html)
- [XIAO Plus Series — CNX Software](https://www.cnx-software.com/2025/01/13/seeed-studio-xiao-plus-series-adds-more-gpio-castellated-holes/)
- [XIAO ESP32-S3 Getting Started (Seeed Studio Wiki)](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)
- [ESP32-S3 Series Datasheet (Espressif)](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
