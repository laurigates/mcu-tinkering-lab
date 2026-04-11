# Seeed Studio XIAO RP2350

Raspberry Pi RP2350-based board in the XIAO form factor (21 x 17.8 mm). Dual-architecture — boot either dual Cortex-M33 or dual RISC-V Hazard3 cores at 150 MHz. Features 19 GPIOs (11 through-hole + 8 bottom pads), PIO state machines for deterministic I/O, ARM TrustZone security, and an onboard RGB LED. The first non-ESP32 XIAO in this collection.

## Why XIAO RP2350

- **Dual-architecture CPU** — Switch between ARM Cortex-M33 and RISC-V Hazard3 cores at build time. Experiment with RISC-V without buying separate hardware.
- **PIO state machines** — Programmable I/O blocks that bit-bang any digital protocol (WS2812, custom UART, I2S, VGA, logic analyzer) at deterministic timing, independent of the CPU. No DMA/interrupt jitter.
- **19 GPIOs** — 11 on side headers + 8 on bottom castellated pads. More I/O than any XIAO ESP32 variant in the same footprint.
- **Hardware security** — ARM TrustZone, Secure Boot with signed firmware, OTP memory for keys, hardware SHA-256. Production-grade security on a $5 board.
- **Broad SDK support** — Raspberry Pi Pico SDK (C/C++), MicroPython, CircuitPython, Arduino, Zephyr, NuttX. The RP2350 ecosystem is mature.
- **~$5 USD** — Cheapest XIAO board. Hard to beat for the GPIO count and PIO capability.

### When to pick something else

| Need | Better choice |
|------|---------------|
| Wi-Fi / Bluetooth | [XIAO ESP32-S3](xiao-esp32s3.md) or [XIAO ESP32-C6](xiao-esp32c6.md) |
| Wi-Fi + camera + AI | [XIAO ESP32-S3 Sense](xiao-esp32s3-sense.md) |
| On-device ML inference | [XIAO ESP32-S3](xiao-esp32s3.md) (vector instructions, PSRAM) |
| LoRa radio | [TTGO LoRa32](ttgo-lora32-v2.md) |
| Linux userspace | [Raspberry Pi Zero 2W](raspberry-pi-zero-2w.md) or [Radxa ZERO 3W](radxa-zero-3w.md) |
| Wi-Fi + PIO + more GPIOs | [Raspberry Pi Pico 2 W](raspberry-pi-pico-2w.md) (26 GPIOs, Wi-Fi, $7) |

## Specifications

| Feature | Details |
|---------|---------|
| SoC | RP2350 — dual Cortex-M33 or dual RISC-V Hazard3 @ 150 MHz |
| SRAM | 520 KB (10 banks) |
| Flash | 2 MB external QSPI (XIP) |
| PSRAM | None |
| Wi-Fi | None |
| Bluetooth | None |
| USB | USB-C (USB 1.1 device/host) |
| GPIO | 19 multi-function (11 through-hole + 8 bottom pads) |
| ADC | 3 channels (12-bit) + internal temp sensor |
| PWM | 16 channels (8 slices x 2) |
| PIO | 3 PIO blocks (12 state machines total) |
| SPI | 2× (SPI0 on side headers, SPI1 on bottom pads) |
| I2C | 2× (I2C0 on bottom pads, I2C1 on side headers) |
| UART | 2× (UART0 on side headers, UART1 on bottom pads) |
| Onboard LEDs | User LED (GPIO25, yellow, active LOW), RGB LED (GPIO22/23) |
| Buttons | RESET (bottom, labeled "B"), BOOT |
| Battery | LiPo charge management (~370 mA), voltage sense on GPIO29 |
| Dimensions | 21 x 17.8 mm |
| Operating temp | -20°C to 85°C |

## Pinout

```
           ┌──────────────────────┐
           │  XIAO RP2350         │
           │                      │
           │        [RGB LED]     │
           │                      │
           │           [BOOT]     │
           ├──┬───────────────┬───┤
    D0/A0  │● │               │ ●│  5V
    D1/A1  │● │               │ ●│  GND
    D2/A2  │● │               │ ●│  3V3
    D3     │● │               │ ●│  D10/MOSI
    D4/SDA │● │               │ ●│  D9/MISO
    D5/SCL │● │               │ ●│  D8/SCK
    D6/TX  │● │               │ ●│  D7/RX
           ├──┴───────────────┴───┤
           │      [USB-C]  [RST]  │
           └──────────────────────┘
```

### Side Header Pin Mapping

| XIAO Pin | GPIO | Default Function | Alternate Functions |
|----------|------|------------------|---------------------|
| D0 | GPIO26 | Analog input | ADC0 |
| D1 | GPIO27 | Analog input | ADC1 |
| D2 | GPIO28 | Analog input | ADC2 |
| D3 | GPIO5 | Digital I/O | SPI0 CS |
| D4 | GPIO6 | I2C1 SDA | Digital I/O |
| D5 | GPIO7 | I2C1 SCL | Digital I/O |
| D6 | GPIO0 | UART0 TX | Digital I/O |
| D7 | GPIO1 | UART0 RX | Digital I/O |
| D8 | GPIO2 | SPI0 SCK | Digital I/O |
| D9 | GPIO4 | SPI0 MISO | Digital I/O |
| D10 | GPIO3 | SPI0 MOSI | Digital I/O |

### Bottom Castellated Pads

| XIAO Pin | GPIO | Default Function | Alternate Functions |
|----------|------|------------------|---------------------|
| D11 | GPIO21 | UART1 RX | Digital I/O |
| D12 | GPIO20 | UART1 TX | Digital I/O |
| D13 | GPIO17 | I2C0 SCL | Digital I/O |
| D14 | GPIO16 | I2C0 SDA | Digital I/O |
| D15 | GPIO11 | SPI1 MOSI | Digital I/O |
| D16 | GPIO12 | SPI1 MISO | Digital I/O |
| D17 | GPIO10 | SPI1 SCK | Digital I/O |
| D18 | GPIO9 | SPI1 SS | Digital I/O |

### Internal Pins

| GPIO | Function | Notes |
|------|----------|-------|
| GPIO19 | Battery ADC enable | Set HIGH to read battery voltage |
| GPIO22 | RGB LED data | WS2812-compatible |
| GPIO23 | RGB LED power | Set HIGH to enable RGB LED |
| GPIO25 | User LED (yellow) | Active LOW |
| GPIO29 | Battery voltage ADC | Read via ADC when GPIO19 is HIGH |

## Power Consumption

| State | Current | Notes |
|-------|---------|-------|
| Active (both cores) | ~25 mA | No wireless — much lower than ESP32 |
| Single core active | ~15 mA | Second core in WFI |
| Sleep | ~27–50 µA | RTC maintained |
| Dormant | ~5 µA | Wake on GPIO edge |

The absence of a wireless radio means significantly lower active power draw compared to any ESP32 board. For battery-powered projects that don't need Wi-Fi, this board lasts much longer.

## Battery Monitoring

Read battery voltage via the internal ADC:

```c
// Enable battery voltage reading
gpio_init(19);
gpio_set_dir(19, GPIO_OUT);
gpio_put(19, 1);

// Read battery voltage on GPIO29 (ADC3)
adc_init();
adc_gpio_init(29);
adc_select_input(3);
uint16_t raw = adc_read();
float voltage = raw * 3.3f / 4096.0f * 2.0f;  // Voltage divider
```

## PIO State Machines

The RP2350 has 12 PIO state machines (3 PIO blocks × 4 SMs each), up from the RP2040's 8. PIO programs execute at clock speed with deterministic timing, independent of the CPU:

- **WS2812/NeoPixel** — Drive LED strips without DMA or CPU intervention
- **Custom serial protocols** — Bit-bang any protocol at precise timing
- **Logic analyzer** — Sample up to 8 pins simultaneously at full clock speed
- **VGA/DVI output** — Generate video signals
- **Rotary encoder** — Hardware quadrature decoding

Each PIO program is written in a simple assembly language and loaded at runtime.

## Security Features

The RP2350 brings production-grade security to a $5 board:

- **ARM TrustZone** — Privilege separation between secure and non-secure worlds (Cortex-M33 only)
- **Secure Boot** — Boot from signed firmware only. OTP fuses lock down the boot chain.
- **OTP Memory** — One-time programmable storage for encryption keys and configuration
- **Hardware SHA-256** — Accelerated hash computation
- **Glitch detectors** — Hardware fault injection countermeasures

## SDK Options

| SDK | Language | Notes |
|-----|----------|-------|
| Pico SDK | C/C++ | Official, most complete |
| MicroPython | Python | Good for prototyping |
| CircuitPython | Python | Adafruit ecosystem, USB workflow |
| Arduino | C++ | Via arduino-pico core |
| Zephyr | C | RTOS, industrial-grade |
| NuttX | C | POSIX-compatible RTOS |

## XIAO RP2350 vs XIAO ESP32-S3 Comparison

| Feature | XIAO RP2350 (this board) | [XIAO ESP32-S3](xiao-esp32s3.md) |
|---------|--------------------------|----------------------------------|
| CPU | Cortex-M33 / Hazard3, dual 150 MHz | Xtensa LX7 dual-core 240 MHz |
| SRAM | 520 KB | 512 KB |
| Flash | 2 MB | 8 MB |
| PSRAM | None | 8 MB octal |
| Wi-Fi | None | 802.11 b/g/n |
| Bluetooth | None | BLE 5.0 |
| USB | USB 1.1 host/device | USB 1.1 OTG |
| GPIO | 19 (11 + 8 bottom) | 11 |
| PIO | 12 state machines | None |
| ADC | 3 channels | 9 channels |
| Security | TrustZone, Secure Boot, OTP | Secure Boot v2, Flash Encryption |
| Deep sleep | ~27 µA | ~14 µA |
| Price | ~$5 | ~$8 |
| Best for | PIO protocols, GPIO-heavy, no wireless | AI/ML, camera, wireless IoT |

## In This Repository

The XIAO RP2350 is not currently used in any project. It is a candidate for:

- PIO-driven LED strip controllers (WS2812 without DMA jitter)
- USB HID devices where Wi-Fi is not needed (cheaper than ESP32-S3)
- Logic analyzer / protocol decoder tools
- Carrier board designs needing compact form factor with many GPIOs
- RISC-V experimentation projects

## References

- [XIAO RP2350 Getting Started (Seeed Studio Wiki)](https://wiki.seeedstudio.com/getting-started-xiao-rp2350/)
- [XIAO RP2350 Product Page](https://www.seeedstudio.com/Seeed-XIAO-RP2350-p-5944.html)
- [XIAO RP2350 Pinout & Specs (Mischianti)](https://mischianti.org/seeed-studio-xiao-rp2350-pinout-datasheet-schema-and-specifications/)
- [XIAO RP2350 Zephyr Support](https://docs.zephyrproject.org/latest/boards/seeed/xiao_rp2350/doc/index.html)
- [XIAO RP2350 NuttX Support](https://nuttx.apache.org/docs/latest/platforms/arm/rp23xx/boards/xiao-rp2350/index.html)
- [RP2350 Datasheet (Raspberry Pi)](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [XIAO RP2350 OSHW Documentation (GitHub)](https://github.com/Seeed-Studio/OSHW-XIAO-Series/blob/main/document/SeeedStudio_XIAO_RP2350/XIAO-RP2350.md)
