# Seeed Studio XIAO ESP32-C6

Thumb-sized RISC-V development board with Wi-Fi 6, Bluetooth 5.3, Zigbee, and Thread on a single chip. The first XIAO family member with 802.15.4 radio support, making it a natural fit for Matter-compliant smart home projects and low-power sensor networks.

## Why XIAO ESP32-C6

- **Four wireless protocols** — Wi-Fi 6 (802.11ax), Bluetooth 5.3 LE, Zigbee 3.0, and Thread on one chip. A single board can bridge between all of them.
- **Matter-ready** — Thread + Wi-Fi means it can participate in Matter fabric as both a Thread border router and a Wi-Fi endpoint.
- **Ultra-compact** — 21 x 17.8 mm with single-sided components. Fits inside wall switches, sensor enclosures, and wearables.
- **Battery-friendly** — 15 µA deep sleep, onboard LiPo charge management, and a dedicated low-power RISC-V core for background tasks.
- **~$5 USD** — One of the cheapest ways to get Wi-Fi 6 and Thread into a project.

### When to pick something else

| Need | Better choice |
|------|---------------|
| USB OTG / HID keyboard emulation | [ESP32-S3](esp32-s3.md) (full USB OTG) |
| AI/ML on-device inference | [ESP32-S3](esp32-s3.md) (vector instructions, dual LX7 cores) |
| Camera / DVP interface | ESP32-S3 or ESP32-CAM |
| Classic Bluetooth (A2DP, SPP) | Original ESP32 |
| LoRa radio | [TTGO LoRa32](ttgo-lora32-v2.md) |
| Maximum GPIO count | ESP32-S3 DevKitC (up to 45 GPIOs) |

## Specifications

| Feature | Details |
|---------|---------|
| MCU | ESP32-C6 — RISC-V single-core (HP) @ 160 MHz + LP core @ 20 MHz |
| SRAM | 512 KB |
| Flash | 4 MB |
| Wi-Fi | 802.11ax (Wi-Fi 6, 2.4 GHz) |
| Bluetooth | Bluetooth 5.3 LE |
| 802.15.4 | Zigbee 3.0, Thread 1.3 |
| USB | USB-C (USB-Serial-JTAG, **not** USB OTG) |
| GPIO | 11 usable digital pins |
| ADC | 12-bit SAR, 7 channels |
| PWM | 11 channels (LEDC) |
| SPI | 1× user-accessible |
| I2C | 1× (SDA: GPIO22, SCL: GPIO23) |
| UART | 1× (TX: GPIO16, RX: GPIO17) |
| Antenna | Onboard ceramic + U.FL connector for external |
| Battery | 3.7V LiPo via pads, onboard charge IC |
| Dimensions | 21 x 17.8 mm |
| Operating temp | -40°C to 85°C |

## Pinout

```
           ┌──────────────────────┐
           │  XIAO ESP32-C6       │
           │  ┌──────────────┐    │
           │  │  Ceramic     │    │
           │  │  Antenna     │    │
           │  └──────────────┘    │
           │                 [U.FL]│
           │  [RST]    [BOOT]     │
           ├──┬───────────────┬───┤
    D0/A0  │● │               │ ●│  5V
    D1/A1  │● │               │ ●│  GND
    D2/A2  │● │               │ ●│  3V3
    D3     │● │               │ ●│  D10/MOSI
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
| D0 | GPIO0 | Analog input | LP_GPIO0, ADC |
| D1 | GPIO1 | Analog input | LP_GPIO1, ADC |
| D2 | GPIO2 | Analog input | LP_GPIO2, ADC |
| D3 | GPIO21 | Digital I/O | SDIO_DATA1 |
| D4 | GPIO22 | I2C SDA | SDIO_DATA2 |
| D5 | GPIO23 | I2C SCL | SDIO_DATA3 |
| D6 | GPIO16 | UART TX | — |
| D7 | GPIO17 | UART RX | — |
| D8 | GPIO19 | SPI SCK | — |
| D9 | GPIO20 | SPI MISO | — |
| D10 | GPIO18 | SPI MOSI | — |

### JTAG Pins (active by default, bottom pads)

| Pad | GPIO | Function |
|-----|------|----------|
| MTMS | GPIO4 | JTAG TMS, ADC |
| MTDI | GPIO5 | JTAG TDI, ADC |
| MTCK | GPIO6 | JTAG TCK, ADC |
| MTDO | GPIO7 | JTAG TDO |

### Safe Pins for General Use

GPIO0, GPIO1, GPIO2, GPIO18, GPIO20, GPIO21 — no boot sequence involvement, no flash/PSRAM connections. Use these first for external peripherals.

### Pins to Use with Caution

GPIO4–GPIO9 serve JTAG debugging and strapping functions. Driving them during boot can cause boot failures. If you need to use them, disable JTAG in software first.

### Antenna Switching

GPIO14 controls the RF switch between onboard ceramic antenna and U.FL connector. GPIO3 must be driven LOW to enable RF switch control. By default, the onboard antenna is active.

## USB Limitation

The ESP32-C6 has a **USB-Serial-JTAG controller** — a fixed-function peripheral hardwired for serial console and JTAG debugging only. It **cannot** be reprogrammed for USB OTG, HID, CDC-ACM, or any custom USB class. This is why we chose the ESP32-S3 over the C6 for the [IT Troubleshooter](../../packages/esp32-projects/it-troubleshooter/) project (see [ADR-005](../blueprint/adrs/ADR-005-it-troubleshooter-hardware.md)).

## Power Consumption

| State | Current | Notes |
|-------|---------|-------|
| Active (Wi-Fi TX) | ~280 mA peak | Depends on TX power level |
| Active (Wi-Fi RX) | ~90 mA | |
| Modem sleep | ~30 mA | CPU active, radio off |
| Light sleep | ~3 mA | LP core can run |
| Deep sleep | ~15 µA | RTC memory retained |

The LP (low-power) RISC-V core at 20 MHz can handle GPIO monitoring, timer wakeups, and simple logic while the HP core sleeps — ideal for battery-powered sensors that wake periodically to transmit.

## Battery Charging

The board has an onboard LiPo charge management IC accessible via solder pads on the bottom:

- Connect a 3.7V lithium battery (positive pad on right, negative on left when viewed from bottom)
- Charges via USB-C when connected
- Red LED indicates charging; LED off when complete
- No built-in battery voltage monitoring ADC — use one of the analog pins (D0–D2) with a voltage divider if you need to read battery level

## Smart Home and IoT Use Cases

The quad-protocol wireless stack makes the XIAO ESP32-C6 uniquely suited for:

- **Thread border router** — Bridge Thread mesh devices to Wi-Fi/IP network. Essential for Matter ecosystems.
- **Zigbee coordinator/end device** — Join existing Zigbee networks (Home Assistant ZHA, Zigbee2MQTT) directly.
- **BLE-to-Wi-Fi gateway** — Collect BLE sensor advertisements and forward via MQTT over Wi-Fi.
- **Low-power environmental sensor** — Deep sleep at 15 µA, wake on timer, read sensor via I2C, transmit over Wi-Fi, go back to sleep. Battery can last months.
- **Matter device** — Native Thread + Wi-Fi support means full Matter compatibility without additional hardware.

## Security Features

The ESP32-C6 includes hardware security capabilities:

- **Secure Boot v2** — Verifies firmware signature on every boot
- **Flash Encryption** — AES-XTS-128 encryption of flash contents
- **Digital Signature** — Hardware-accelerated RSA/ECDSA
- **HMAC** — Hardware HMAC peripheral
- **Trusted Execution Environment** — World controller for privilege separation

## ESP-IDF Configuration

### sdkconfig.defaults for XIAO ESP32-C6

```ini
# Target
CONFIG_IDF_TARGET="esp32c6"

# Wi-Fi 6
CONFIG_ESP_WIFI_11AX_SUPPORT=y

# Zigbee (if using Zigbee stack)
CONFIG_ZB_ENABLED=y

# Thread (if using Thread/OpenThread)
CONFIG_OPENTHREAD_ENABLED=y

# Low power
CONFIG_PM_ENABLE=y
CONFIG_FREERTOS_USE_TICKLESS_IDLE=y
```

### CMake target

```bash
idf.py set-target esp32c6
```

## ESP32-C6 vs ESP32-S3 Quick Comparison

| Feature | ESP32-C6 (this board) | ESP32-S3 |
|---------|----------------------|----------|
| CPU | RISC-V single-core 160 MHz | Xtensa LX7 dual-core 240 MHz |
| Wi-Fi | Wi-Fi 6 (802.11ax) | Wi-Fi 4 (802.11n) |
| Bluetooth | BLE 5.3 | BLE 5.0 |
| Zigbee/Thread | Yes (802.15.4) | No |
| USB | Serial-JTAG only | Full USB OTG |
| AI/ML acceleration | No | Vector instructions |
| PSRAM | Not on XIAO C6 | Up to 16 MB |
| Camera interface | No | DVP 8/16-bit |
| Best for | Smart home, IoT sensors, Matter | AI/ML, cameras, USB devices |

## In This Repository

The XIAO ESP32-C6 was originally considered for the IT Troubleshooter project but was rejected due to its lack of USB OTG support ([ADR-005](../blueprint/adrs/ADR-005-it-troubleshooter-hardware.md)). It remains a strong candidate for future projects that need:

- Thread/Zigbee connectivity (e.g., Matter bridge)
- Low-power battery-operated sensors
- Wi-Fi 6 for dense AP environments

## References

- [XIAO ESP32-C6 Getting Started (Seeed Studio Wiki)](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/)
- [XIAO ESP32-C6 Product Page](https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C6-p-5884.html)
- [ESP32-C6 Series Datasheet (Espressif)](https://www.espressif.com/sites/default/files/documentation/esp32-c6_datasheet_en.pdf)
- [XIAO ESP32-C6 Pinout Reference (espboards.dev)](https://www.espboards.dev/esp32/xiao-esp32c6/)
- [XIAO ESP32-C6 Zephyr Support](https://docs.zephyrproject.org/latest/boards/seeed/xiao_esp32c6/doc/index.html)
