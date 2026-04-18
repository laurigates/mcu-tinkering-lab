# Raspberry Pi Pico 2 W

RP2350-based microcontroller board with Wi-Fi and Bluetooth from the Raspberry Pi Foundation. The Pico 2 W upgrades the original Pico W with dual Cortex-M33 cores (or dual RISC-V Hazard3 cores — selectable at boot), doubled RAM, and improved security features, while keeping the same form factor and $7 price point.

## Why Pico 2 W

- **Dual-architecture CPU** — RP2350 can boot either dual Cortex-M33 @ 150 MHz or dual RISC-V Hazard3 @ 150 MHz. Choose per-build via the linker script. Useful for RISC-V experimentation without buying separate hardware.
- **520 KB SRAM** — Nearly double the original Pico W's 264 KB. Enough for larger buffers, TLS stacks, and more complex state machines.
- **Hardware security** — Secure boot (signed firmware), ARM TrustZone, OTP memory for keys, and hardware SHA-256. A big step up from the original Pico's zero security story.
- **PIO v2** — 12 PIO state machines (up from 8). PIO lets you bit-bang nearly any digital protocol at deterministic timing — WS2812 LEDs, VGA, custom UART, I2S, and more.
- **$7 USD** — Same price as the original Pico W, with meaningfully better specs.
- **Massive ecosystem** — Raspberry Pi's SDK, MicroPython, CircuitPython, Arduino core, and Zephyr all support it. Documentation and community are first-class.

### When to pick something else

| Need | Better choice |
|------|---------------|
| On-device AI/ML inference | [Radxa ZERO 3W](radxa-zero-3w.md) (NPU) or ESP32-S3 (vector instructions) |
| Linux userspace (Python, OpenCV, Docker) | [Raspberry Pi Zero 2W](raspberry-pi-zero-2w.md) or [Radxa ZERO 3W](radxa-zero-3w.md) |
| Camera with ISP pipeline | [Luckfox Pico Ultra](luckfox-pico-ultra.md) (hardware ISP + NPU) |
| Wi-Fi 6 or Bluetooth 5.3+ | [XIAO ESP32-C6](xiao-esp32c6.md) |
| Zigbee / Thread / Matter | [XIAO ESP32-C6](xiao-esp32c6.md) |
| LoRa radio | [TTGO LoRa32](ttgo-lora32-v2.md) |
| Absolute minimum cost | [ESP32-C3 Super Mini](esp32-c3-super-mini.md) (~$2–3) |

## Specifications

| Feature | Details |
|---------|---------|
| SoC | RP2350 — dual Cortex-M33 or dual RISC-V Hazard3 @ 150 MHz |
| SRAM | 520 KB |
| Flash | 4 MB QSPI (external) |
| PSRAM | None |
| Wi-Fi | 802.11 b/g/n (2.4 GHz), via Infineon CYW43439 |
| Bluetooth | Bluetooth 5.2 LE |
| USB | Micro-USB (USB 1.1 device/host) |
| GPIO | 26 multi-function pins |
| ADC | 3× 12-bit ADC channels + internal temp sensor |
| PWM | 16 channels (8 slices × 2 outputs) |
| PIO | 12 state machines (3 PIO blocks × 4 SM each) |
| SPI | 2× |
| I2C | 2× |
| UART | 2× |
| Antenna | Onboard PCB antenna |
| Dimensions | 51 x 21 mm |
| Operating temp | -20°C to 70°C (with wireless) |

## Pinout

```
                    ┌──────────────────────┐
             GP0  ●│1                   40│●  VBUS
             GP1  ●│2                   39│●  VSYS
             GND  ●│3                   38│●  GND
             GP2  ●│4                   37│●  3V3_EN
             GP3  ●│5                   36│●  3V3
             GP4  ●│6                   35│●  ADC_VREF
             GP5  ●│7                   34│●  GP28 / ADC2
             GND  ●│8                   33│●  GND / AGND
             GP6  ●│9                   32│●  GP27 / ADC1
             GP7  ●│10                  31│●  GP26 / ADC0
             GP8  ●│11                  30│●  RUN
             GP9  ●│12                  29│●  GP22
             GND  ●│13                  28│●  GND
            GP10  ●│14                  27│●  GP21
            GP11  ●│15                  26│●  GP20
            GP12  ●│16                  25│●  GP19
            GP13  ●│17                  24│●  GP18
             GND  ●│18                  23│●  GND
            GP14  ●│19                  22│●  GP17
            GP15  ●│20                  21│●  GP16
                    └──────────────────────┘
                         [micro-USB]

       Note: GP23, GP24, GP25, GP29 are used internally
       by the wireless module and are NOT available on pins.
```

### Pin Mapping

| GPIO | Default Function | Notes |
|------|------------------|-------|
| GP0–GP1 | UART0 TX/RX | Default serial |
| GP2–GP3 | General I/O | Safe for any use |
| GP4–GP5 | I2C0 SDA/SCL | Default I2C |
| GP6–GP9 | General I/O | Safe for any use |
| GP10–GP13 | SPI1 (SCK, TX, RX, CS) | Default SPI |
| GP14–GP15 | General I/O | Safe for any use |
| GP16–GP17 | SPI0 (RX, CS) | |
| GP18–GP19 | SPI0 (SCK, TX) | |
| GP20–GP21 | I2C0 or general I/O | |
| GP22 | General I/O | |
| GP23 | Wireless SPI CLK | **Not on header** |
| GP24 | Wireless SPI DATA | **Not on header** |
| GP25 | Wireless SPI CS | **Not on header** |
| GP26 | ADC0 | 12-bit analog input |
| GP27 | ADC1 | 12-bit analog input |
| GP28 | ADC2 | 12-bit analog input |
| GP29 | VSYS/3 ADC | **Not on header**, reads supply voltage |

### PIO — The Killer Feature

The RP2350's 12 PIO state machines can implement arbitrary digital protocols at cycle-accurate timing without CPU involvement. Each state machine has a 32-instruction program memory and runs independently. This makes the Pico 2 W excellent for:

- WS2812/NeoPixel LED strips (no special peripheral needed)
- VGA/DVI video output
- Custom serial protocols
- Parallel interfaces (8080-style LCDs)
- Logic analyzer
- Quadrature encoder reading

PIO programs are deterministic — they execute in exact clock cycles regardless of CPU load or interrupts. This is something no Linux-based SBC can match.

## Power Consumption

| State | Current | Notes |
|-------|---------|-------|
| Active (Wi-Fi TX) | ~300 mA peak | Depends on TX power |
| Active (Wi-Fi connected, idle) | ~40–50 mA | Station mode, DTIM=1 |
| Active (no Wi-Fi) | ~25 mA | Both cores running |
| Sleep (dormant mode) | ~1.3 mA | Wireless module still powered |
| Deep sleep (DORMANT) | ~0.8 mA | RTC wake possible |

**Note:** The CYW43439 wireless module draws significant standby current even when idle. True microamp-level sleep requires fully powering down the wireless module and is more complex than on ESP32 boards.

## Known Issues and Quirks

### Wi-Fi and GPIO Share an SPI Bus

The CYW43439 wireless chip communicates with the RP2350 over SPI using GP23–GP25 (internal, not exposed). The wireless driver holds a mutex during SPI transactions. High-frequency Wi-Fi activity can introduce jitter in time-sensitive GPIO operations if you're not using PIO.

### No Built-in USB-UART Bridge

Unlike ESP32 dev boards, the Pico 2 W uses native USB from the RP2350. The first time you connect it, you need to hold BOOTSEL and plug in USB to enter mass storage mode for flashing. After that, UF2 drag-and-drop or SWD works normally.

### 2.4 GHz Only

Wi-Fi is 2.4 GHz only (802.11n). No 5 GHz support. In crowded RF environments, this can be a limitation.

### MicroPython vs C SDK

MicroPython support is excellent but Wi-Fi + BLE + PIO simultaneously can exhaust the 520 KB SRAM in MicroPython. For complex projects, the C/C++ SDK gives much better memory efficiency.

## Security Features

The RP2350 has significantly better security than the RP2040:

- **Secure Boot** — Signed boot with SHA-256 + optional RSA/ECDSA verification
- **ARM TrustZone** — Secure/non-secure world separation (Cortex-M33 mode only)
- **OTP Memory** — One-time programmable storage for keys and configuration
- **Hardware SHA-256** — Accelerated hashing
- **Debug Lockdown** — SWD can be permanently disabled via OTP
- **Glitch Detection** — Basic voltage glitch detectors

## SDK and Toolchain

### Pico SDK (C/C++)

```bash
# Install
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk && git submodule update --init

# Set environment
export PICO_SDK_PATH=/path/to/pico-sdk

# Build a project
mkdir build && cd build
cmake -DPICO_BOARD=pico2_w ..
make -j$(nproc)
```

### Flash via UF2

1. Hold BOOTSEL button, plug in USB
2. Board appears as mass storage device
3. Drag `.uf2` file onto the drive
4. Board reboots and runs firmware

### MicroPython

```bash
# Flash MicroPython UF2 for Pico 2 W
# Download from micropython.org/download/RPI_PICO2_W/
# Then drag-and-drop the .uf2 file
```

## Pico 2 W vs ESP32 Comparison

| Feature | Pico 2 W | ESP32 (original) | [ESP32-S3](esp32-s3.md) |
|---------|----------|-------------------|-----------|
| CPU | Cortex-M33 or RISC-V, 2-core 150 MHz | Xtensa LX6 2-core 240 MHz | Xtensa LX7 2-core 240 MHz |
| SRAM | 520 KB | 520 KB | 512 KB |
| Flash | 4 MB (external) | 4–16 MB (external) | 4–32 MB |
| PSRAM | None | Up to 4 MB | Up to 16 MB |
| Wi-Fi | Wi-Fi 4 (2.4 GHz) | Wi-Fi 4 (2.4 GHz) | Wi-Fi 4 (2.4 GHz) |
| Bluetooth | BLE 5.2 | Classic BT + BLE 4.2 | BLE 5.0 |
| USB | USB 1.1 native | Requires bridge chip | USB OTG |
| GPIO | 26 | 34 | Up to 45 |
| PIO | 12 state machines | None | None |
| Camera | None | None | DVP 8/16-bit |
| AI acceleration | None | None | Vector instructions |
| Security | TrustZone, Secure Boot, OTP | Basic flash encryption | Similar |
| Deep sleep | ~0.8 mA (with wireless) | ~10 µA | ~7 µA |
| Price | ~$7 | ~$4–8 | ~$8–14 |

## Use Cases

- **Real-time motor control** — PIO provides jitter-free PWM and encoder reading that Linux SBCs cannot match. Good replacement for ESP32 as robocar main controller.
- **Wi-Fi sensor hub** — Collect data from I2C/SPI sensors, push to MQTT/HTTP over Wi-Fi.
- **Custom protocol bridge** — PIO can implement DMX512, 1-Wire, custom serial, parallel LCD, and more.
- **USB device** — Native USB lets it appear as HID keyboard, MIDI controller, or CDC serial device.
- **Education** — Excellent documentation, drag-and-drop flashing, MicroPython support.

## In This Repository

The Pico 2 W is not currently used in any project. It is a candidate for:
- Replacing the Heltec WiFi LoRa 32 as the robocar main controller (PIO for motor control + Wi-Fi for telemetry)
- Standalone sensor nodes using MicroPython for rapid prototyping
- USB HID projects as an alternative to ESP32-S3

## References

- [Raspberry Pi Pico 2 W Product Page](https://www.raspberrypi.com/products/raspberry-pi-pico-2/)
- [RP2350 Datasheet (Raspberry Pi)](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [Pico 2 W Datasheet](https://datasheets.raspberrypi.com/picow/pico-2-w-datasheet.pdf)
- [Raspberry Pi Pico SDK Documentation](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html)
- [MicroPython for Pico 2 W](https://micropython.org/download/RPI_PICO2_W/)
