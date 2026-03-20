# Orange Pi Zero 2W

Budget Linux SBC with a quad-core Allwinner H618 processor, Wi-Fi 5, and Bluetooth 5.0 in a Raspberry Pi Zero-compatible form factor. Positioned as a more powerful and more available alternative to the Raspberry Pi Zero 2 W, with up to 4 GB RAM and dual-band Wi-Fi.

## Why Orange Pi Zero 2W

- **Actually in stock** — Unlike the Raspberry Pi Zero 2 W, the Orange Pi Zero 2W has consistent availability from AliExpress and other retailers.
- **Up to 4 GB RAM** — Available in 1/1.5/2/4 GB configurations. The 4 GB model comfortably runs Linux + OpenCV + Python applications.
- **Dual-band Wi-Fi 5** — 802.11ac on 2.4 and 5 GHz. Significantly better wireless than the Pi Zero 2 W's 2.4 GHz-only Wi-Fi 4.
- **Faster CPU clock** — Cortex-A53 at 1.5 GHz vs Pi Zero 2 W's 1.0 GHz. Same architecture, 50% higher clock speed.
- **H.265 hardware decode** — 4K@60 H.265 and 4K@30 H.264 decode in hardware via the Allwinner VPU.
- **~$13–23 USD** — Starting at ~$13 for 1 GB, ~$23 for 4 GB. Excellent value for the specs.

### When to pick something else

| Need | Better choice |
|------|---------------|
| NPU for ML inference | [Radxa ZERO 3W](radxa-zero-3w.md) (0.8 TOPS NPU) |
| Camera (CSI connector) | [Raspberry Pi Zero 2W](raspberry-pi-zero-2w.md) or [Radxa ZERO 3W](radxa-zero-3w.md) |
| Best community support | [Raspberry Pi Zero 2W](raspberry-pi-zero-2w.md) |
| Real-time / bare metal | [Pico 2 W](raspberry-pi-pico-2w.md) or ESP32 |
| Ultra-low power | [ESP32-C3 Super Mini](esp32-c3-super-mini.md) or [Pico 2 W](raspberry-pi-pico-2w.md) |
| Wi-Fi 6 | [Radxa ZERO 3W](radxa-zero-3w.md) |

## Specifications

| Feature | Details |
|---------|---------|
| SoC | Allwinner H618 |
| CPU | Quad Cortex-A53 @ 1.5 GHz (64-bit ARMv8-A) |
| GPU | Mali-G31 MP2 (OpenGL ES 3.2, Vulkan 1.1) |
| NPU | None |
| RAM | 1 / 1.5 / 2 / 4 GB LPDDR4 |
| Storage | microSD, 16 MB SPI NOR flash (for bootloader) |
| Wi-Fi | 802.11 a/b/g/n/ac (Wi-Fi 5, 2.4 + 5 GHz) |
| Bluetooth | Bluetooth 5.0 LE |
| USB | 1× USB-C (OTG), 1× USB 2.0 Type-A |
| GPIO | 40-pin header (Raspberry Pi compatible layout) |
| CSI | None |
| Video output | Mini-HDMI (up to 4K@60) |
| Audio | Via HDMI (no 3.5mm jack) |
| Dimensions | 65 x 30 mm |
| Operating temp | -10°C to 60°C |

## Pinout

```
                 ┌──────────────────────────────────────────┐
                 │  Orange Pi Zero 2W                       │
                 │                                          │
  [mini-HDMI]    │  ┌─────────────────────────────────┐     │
                 │  │ 40-pin GPIO header               │     │
                 │  └─────────────────────────────────┘     │
  [USB-C OTG]    │                     [USB-A 2.0]         │
                 └──────────────────────────────────────────┘

  40-Pin Header (Raspberry Pi compatible):

     3V3  (1) (2)  5V
   GPIO2  (3) (4)  5V
   GPIO3  (5) (6)  GND
   GPIO4  (7) (8)  GPIO14      UART TX
     GND  (9) (10) GPIO15      UART RX
  GPIO17 (11) (12) GPIO18
  GPIO27 (13) (14) GND
  GPIO22 (15) (16) GPIO23
     3V3 (17) (18) GPIO24
  GPIO10 (19) (20) GND
   GPIO9 (21) (22) GPIO25
  GPIO11 (23) (24) GPIO8
     GND (25) (26) GPIO7
   GPIO0 (27) (28) GPIO1
   GPIO5 (29) (30) GND
   GPIO6 (31) (32) GPIO12
  GPIO13 (33) (34) GND
  GPIO19 (35) (36) GPIO16
  GPIO26 (37) (38) GPIO20
     GND (39) (40) GPIO21

  Note: Uses Raspberry Pi-compatible numbering but actual
  Allwinner H618 GPIO bank references differ. Check Orange Pi
  documentation for the PH/PI/PC bank mappings.
```

### Key Interfaces

| Interface | Pins | Notes |
|-----------|------|-------|
| I2C | Pin 3 (SDA), Pin 5 (SCL) | 3.3V logic |
| SPI | Pin 19/21/23/24/26 | |
| UART | Pin 8 (TX), Pin 10 (RX) | Serial console |
| IR receiver | Pin 7 | Infrared remote input |

## Power Consumption

| State | Current | Notes |
|-------|---------|-------|
| Idle (headless, Wi-Fi) | ~300 mA | Minimal services |
| Active (4 cores loaded) | ~600 mA | Sustained CPU workload |
| Idle (Wi-Fi off) | ~200 mA | |
| Shutdown (halt) | ~30 mA | No hardware power-off |

## Known Issues and Quirks

### No Camera Connector

The Orange Pi Zero 2W lacks a CSI camera connector. For camera input, you must use a USB webcam, which consumes the USB-A port or requires a hub. This is a significant limitation compared to the Pi Zero 2 W and Radxa ZERO 3W.

### Allwinner BSP Kernel

Allwinner's mainline Linux support has historically lagged behind Rockchip and Broadcom. The Orange Pi ships with an Allwinner BSP kernel. Mainline kernel support exists via sunxi community efforts but may lack some hardware features (GPU acceleration, hardware video decode).

### GPIO Library Support

`RPi.GPIO` does not work. Use:
- `wiringOP` (Orange Pi's fork of wiringPi)
- `OPi.GPIO` (Python library)
- `libgpiod` (recommended for new projects)

### Thermal

The H618 runs warm under sustained load. A heatsink is recommended. The board doesn't have mounting holes for a fan, but adhesive heatsinks work well.

### Audio Output

No 3.5mm audio jack — audio is HDMI-only. For headless projects that need audio, use a USB audio adapter.

## Software

### Operating Systems

- **Orange Pi OS (Arch-based)** — Default, includes desktop environment
- **Orange Pi Debian/Ubuntu** — Server and desktop images
- **Armbian** — Community-maintained, generally recommended for headless use
- **DietPi** — Minimal, optimized

### ML/AI (CPU Only)

Without an NPU, ML inference runs on the CPU:

```bash
# TensorFlow Lite
pip install tflite-runtime

# Expect similar performance to Pi Zero 2W
# MobileNet-SSD: ~3-4 FPS (slightly faster than Pi Zero 2W due to higher clock)
```

## Orange Pi Zero 2W vs Similar SBCs

| Feature | Orange Pi Zero 2W | [Pi Zero 2W](raspberry-pi-zero-2w.md) | [Radxa ZERO 3W](radxa-zero-3w.md) |
|---------|------------------|-------------|------------|
| CPU | 4× A53 @ 1.5 GHz | 4× A53 @ 1 GHz | 4× A55 @ 1.8 GHz |
| RAM | 1–4 GB | 512 MB | 1–8 GB |
| Wi-Fi | Wi-Fi 5 (dual-band) | Wi-Fi 4 (2.4 GHz) | Wi-Fi 6 (dual-band) |
| BT | 5.0 | 4.2 | 5.4 |
| NPU | None | None | 0.8 TOPS |
| GPU | Mali-G31 | VideoCore IV | Mali-G52 |
| Camera | None (USB only) | CSI-2 | CSI-2 |
| USB ports | USB-C + USB-A | Micro-USB (OTG) | USB-C |
| Price | ~$13–23 | ~$15 | ~$16–50 |
| Availability | Good | Inconsistent | Good |
| Community | Good | Excellent | Growing |

## Use Cases

- **Headless Linux server** — MQTT broker, Node-RED, Home Assistant, small web server. The 4 GB RAM model is very comfortable for this.
- **Wi-Fi bridge / gateway** — Dual-band Wi-Fi + Bluetooth makes it a good protocol bridge (BLE-to-MQTT, Wi-Fi relay).
- **Media player** — 4K HDMI output + H.265 hardware decode for Kodi or similar media center applications.
- **Network monitoring** — Run lightweight monitoring tools, VPN endpoints, or Pi-hole DNS filtering.
- **USB webcam processing** — Connect a USB webcam and run basic OpenCV processing. Limited by USB bandwidth and CPU-only inference.

## In This Repository

The Orange Pi Zero 2W is not currently used in any project. It is a candidate for:
- An alternative to the Raspberry Pi Zero 2 W for projects where availability matters
- Running a headless MQTT broker or Node-RED server for the robocar system
- General-purpose Linux SBC tasks where an NPU is not needed

## References

- [Orange Pi Zero 2W Product Page](http://www.orangepi.org/html/hardWare/computerAndMicrocontrollers/details/Orange-Pi-Zero-2W.html)
- [Orange Pi Zero 2W User Manual](http://www.orangepi.org/html/hardWare/computerAndMicrocontrollers/service-and-support/Orange-Pi-Zero-2W.html)
- [Allwinner H618 Brief](https://www.allwinnertech.com/index.php?c=product&a=index&id=155)
- [Armbian for Orange Pi Zero 2W](https://www.armbian.com/orangepi-zero-2w/)
- [Orange Pi Zero 2W Pinout (espboards.dev)](https://www.espboards.dev/sbc/orange-pi-zero-2w/)
