# Raspberry Pi Zero 2 W

Credit-card-sized Linux single-board computer with quad-core 64-bit ARM and Wi-Fi/Bluetooth. The Zero 2 W packs a BCM2710A1 (same family as the Pi 3) into the original Zero form factor, making it the smallest Raspberry Pi capable of running a full Linux desktop, Docker containers, or Python-based computer vision — all for ~$15.

## Why Raspberry Pi Zero 2 W

- **Full Linux** — Runs Raspberry Pi OS, Ubuntu, DietPi, and other ARM64 distributions. Access to apt, Python, Node.js, Docker, OpenCV, and the entire Linux ecosystem.
- **Quad-core 64-bit** — BCM2710A1 with four Cortex-A53 cores at 1 GHz. Enough for lightweight ML inference (TFLite), basic computer vision, and multi-threaded workloads.
- **Tiny form factor** — 65 x 30 mm, same footprint as the original Pi Zero. Fits in compact enclosures and robotics platforms.
- **Camera support** — MIPI CSI-2 camera connector supports official Raspberry Pi cameras. Hardware-accelerated H.264 encode/decode via VideoCore IV GPU.
- **$15 USD** — Cheapest way to get a Linux SBC with camera support and Wi-Fi into a project.
- **Massive community** — More tutorials, libraries, and community support than any other SBC.

### When to pick something else

| Need | Better choice |
|------|---------------|
| On-device ML with NPU acceleration | [Radxa ZERO 3W](radxa-zero-3w.md) (0.8 TOPS NPU) |
| Real-time motor control / deterministic timing | [Pico 2 W](raspberry-pi-pico-2w.md) (PIO, bare metal) |
| More RAM (>512 MB) | [Radxa ZERO 3W](radxa-zero-3w.md) (up to 8 GB) |
| Wi-Fi 6 | [Radxa ZERO 3W](radxa-zero-3w.md) or [XIAO ESP32-C6](xiao-esp32c6.md) |
| Lowest power consumption | [Pico 2 W](raspberry-pi-pico-2w.md) or [ESP32-C3 Super Mini](esp32-c3-super-mini.md) |
| Better availability | [Orange Pi Zero 2W](orange-pi-zero-2w.md) (historically easier to buy) |
| USB gadget mode + Ethernet | Raspberry Pi Zero 2 W via USB OTG (this board can do it) |

## Specifications

| Feature | Details |
|---------|---------|
| SoC | BCM2710A1 — quad Cortex-A53 @ 1 GHz (64-bit ARMv8-A) |
| GPU | VideoCore IV @ 400 MHz (OpenGL ES 2.0, H.264 encode/decode) |
| RAM | 512 MB LPDDR2 (shared with GPU) |
| Storage | microSD card slot (no eMMC) |
| Wi-Fi | 802.11 b/g/n (2.4 GHz) |
| Bluetooth | Bluetooth 4.2 / BLE |
| USB | 1× Micro-USB OTG (data), 1× Micro-USB (power only) |
| GPIO | 40-pin header (unpopulated) — 26 GPIO |
| CSI | MIPI CSI-2 camera connector (22-pin, requires adapter cable) |
| Video output | Mini-HDMI (up to 1080p60) |
| Composite video | Via GPIO (active low on GPIO18) |
| Dimensions | 65 x 30 x 5 mm |
| Weight | ~10 g |
| Operating temp | -20°C to 70°C |

## Pinout

```
                 ┌──────────────────────────────────────────┐
                 │  Raspberry Pi Zero 2 W                   │
                 │                                          │
    [mini-HDMI]  │  ┌─────────────────────────────────┐     │ [CSI camera]
                 │  │ 40-pin GPIO header (unpopulated) │     │
                 │  └─────────────────────────────────┘     │
    [USB data]   │                                          │ [USB power]
                 └──────────────────────────────────────────┘

  40-Pin GPIO Header (standard Raspberry Pi layout):

     3V3  (1) (2)  5V
   GPIO2  (3) (4)  5V          I2C1 SDA
   GPIO3  (5) (6)  GND         I2C1 SCL
   GPIO4  (7) (8)  GPIO14      UART TX
     GND  (9) (10) GPIO15      UART RX
  GPIO17 (11) (12) GPIO18      PCM CLK
  GPIO27 (13) (14) GND
  GPIO22 (15) (16) GPIO23
     3V3 (17) (18) GPIO24
  GPIO10 (19) (20) GND         SPI0 MOSI
   GPIO9 (21) (22) GPIO25      SPI0 MISO
  GPIO11 (23) (24) GPIO8       SPI0 SCLK / SPI0 CE0
     GND (25) (26) GPIO7       SPI0 CE1
   GPIO0 (27) (28) GPIO1       ID EEPROM (reserved for HATs)
   GPIO5 (29) (30) GND
   GPIO6 (31) (32) GPIO12
  GPIO13 (33) (34) GND
  GPIO19 (35) (36) GPIO16
  GPIO26 (37) (38) GPIO20
     GND (39) (40) GPIO21
```

### Key Interfaces

| Interface | Pins | Notes |
|-----------|------|-------|
| I2C1 | GPIO2 (SDA), GPIO3 (SCL) | Default I2C bus, 3.3V logic |
| SPI0 | GPIO10 (MOSI), GPIO9 (MISO), GPIO11 (SCLK), GPIO8/7 (CE0/CE1) | |
| UART | GPIO14 (TX), GPIO15 (RX) | Serial console (enable in config.txt) |
| PWM | GPIO12, GPIO13, GPIO18, GPIO19 | Hardware PWM (2 channels, 2 outputs each) |
| 1-Wire | GPIO4 (default) | For DS18B20 temperature sensors |

**Important:** The 40-pin header comes unpopulated. You must solder a pin header yourself or use a solderless hammer header.

## Power Consumption

| State | Current | Notes |
|-------|---------|-------|
| Idle (desktop) | ~350 mA | CPU mostly idle, Wi-Fi connected |
| Active (4 cores loaded) | ~500 mA | Sustained CPU workload |
| Active (Wi-Fi TX) | ~550 mA peak | |
| Idle (headless, Wi-Fi off) | ~100 mA | Minimal services running |
| Shutdown (halt) | ~30 mA | No true "off" without cutting power |

**Note:** The Zero 2 W has no hardware power-off state. Even in `halt`, it draws ~30 mA. For battery-powered projects, use a physical switch or a MOSFET to cut power. This is a fundamentally different power profile from microcontrollers like ESP32 or Pico.

## Known Issues and Quirks

### 512 MB RAM Is Tight

With Raspberry Pi OS, about 150–200 MB is consumed by the OS and GPU split before your application starts. Running OpenCV + a camera stream + inference simultaneously can cause OOM kills. Mitigations:
- Use DietPi or a minimal Raspberry Pi OS Lite image
- Reduce GPU memory split to 16 MB (if not using HDMI): `gpu_mem=16` in `config.txt`
- Use swap (but it's slow on microSD)

### Stock Availability

The Zero 2 W has been notoriously difficult to buy since launch (2021). Raspberry Pi has improved supply since late 2023, but stock still varies by region. If you can't find one, the [Orange Pi Zero 2W](orange-pi-zero-2w.md) is a readily available alternative.

### Thermal Throttling

The BCM2710A1 throttles at 80°C. Under sustained quad-core load, it can reach this without a heatsink. A small adhesive heatsink is recommended for compute-heavy workloads.

### microSD Wear

Running Linux from microSD means constant writes (logs, swap, tmp). For long-running deployments:
- Use `log2ram` to keep logs in RAM
- Disable swap or use zram instead
- Set `/tmp` as tmpfs
- Use a high-endurance microSD card (Samsung PRO Endurance, SanDisk MAX Endurance)

### Boot Time

Typical boot to login: 20–30 seconds. Not suitable for applications that need instant-on behavior. If you need sub-second startup, use a microcontroller instead.

## Camera Support

The CSI connector accepts Raspberry Pi camera modules via a Zero-specific ribbon cable adapter (22-pin to 15-pin):

| Camera | Resolution | FPS | Notes |
|--------|-----------|-----|-------|
| Camera Module v1 (OV5647) | 5 MP | 30 fps @ 1080p | Cheap, well-supported |
| Camera Module v2 (IMX219) | 8 MP | 30 fps @ 1080p | Better low-light |
| Camera Module v3 (IMX708) | 12 MP | 30 fps @ 1080p | Autofocus, HDR |

Use `libcamera` (not the legacy `raspicam` stack) for camera access on current Raspberry Pi OS.

## Software Ecosystem

The Zero 2 W runs the same software as any Raspberry Pi:

- **Raspberry Pi OS** (Debian-based) — Default, best hardware support
- **Ubuntu Server** — 64-bit ARM, good for headless/server use
- **DietPi** — Minimal, optimized for SBCs
- **Home Assistant OS** — Smart home hub (tight on 512 MB RAM but works)

### ML/AI Capabilities

```bash
# TensorFlow Lite (Python)
pip install tflite-runtime

# OpenCV
sudo apt install python3-opencv

# Example: MobileNet-SSD object detection
# ~2-3 FPS on Zero 2 W (CPU only, no NPU)
```

ML inference is CPU-only — no NPU, no GPU compute. Expect ~2–3 FPS for MobileNet-SSD object detection. For faster inference, consider the [Radxa ZERO 3W](radxa-zero-3w.md) with its dedicated NPU.

## Zero 2 W vs Similar SBCs

| Feature | Zero 2 W | [Radxa ZERO 3W](radxa-zero-3w.md) | [Orange Pi Zero 2W](orange-pi-zero-2w.md) |
|---------|----------|------------|-----------------|
| CPU | 4× A53 @ 1 GHz | 4× A55 @ 1.8 GHz | 4× A53 @ 1.5 GHz |
| RAM | 512 MB | 1–8 GB | 1–4 GB |
| Wi-Fi | Wi-Fi 4 | Wi-Fi 6 | Wi-Fi 5 |
| BT | 4.2 | 5.4 | 5.0 |
| NPU | None | 0.8 TOPS | None |
| GPU | VideoCore IV | Mali-G52 | Mali-G31 |
| Camera | CSI-2 | MIPI CSI-2 | None |
| Price | ~$15 | ~$16–50 | ~$13–23 |
| Community | Excellent | Growing | Good |
| Availability | Inconsistent | Good | Good |

## Use Cases

- **Lightweight vision system** — Pi Camera + OpenCV for basic image processing, color tracking, or QR code reading. Ship inference to a cloud API for complex tasks.
- **Wi-Fi gateway / bridge** — Run Zigbee2MQTT, Bluetooth-to-MQTT bridge, or RTSP camera proxy.
- **Headless server** — MQTT broker, Node-RED, small web server, or Home Assistant.
- **Education** — Full Linux desktop for learning programming, electronics, and networking.
- **USB gadget** — Emulate Ethernet adapter, serial port, or mass storage over USB OTG.

## In This Repository

The Raspberry Pi Zero 2 W is not currently used in any project. It is a candidate for:
- Replacing the ESP32-CAM with a more capable vision system (Pi Camera + local OpenCV + API calls to Claude for complex inference)
- Running a local MQTT broker for the robocar system
- Serving as a development/debug terminal connected to the robocar via UART

## References

- [Raspberry Pi Zero 2 W Product Page](https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/)
- [Raspberry Pi Zero 2 W Datasheet](https://datasheets.raspberrypi.com/rpizero2/raspberry-pi-zero-2-w-product-brief.pdf)
- [BCM2710A1 (Broadcom)](https://www.raspberrypi.com/documentation/computers/processors.html)
- [Raspberry Pi GPIO Pinout](https://pinout.xyz/)
- [Raspberry Pi Camera Documentation](https://www.raspberrypi.com/documentation/accessories/camera.html)
