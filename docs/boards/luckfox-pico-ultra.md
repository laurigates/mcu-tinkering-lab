# Luckfox Pico Ultra

Tiny Linux-capable board built around the Rockchip RV1106 — a single Cortex-A7 core with a 0.5 TOPS NPU and hardware ISP, designed specifically for camera/vision applications. At roughly the size of a Raspberry Pi Pico and ~$10–15, it's one of the smallest and cheapest ways to run local ML inference on a Linux board.

## Why Luckfox Pico Ultra

- **Up to 1 TOPS NPU** — The RV1106G3 (Ultra) has a 1 TOPS INT8 neural network accelerator; the RV1106G2 (Ultra B) has 0.5 TOPS. Handles MobileNet, YOLOv5-nano, and similar models well.
- **Hardware ISP** — Built-in image signal processor with 3A (auto-exposure, auto-white-balance, auto-focus) support. Camera frames go directly to the ISP pipeline without CPU involvement.
- **Tiny form factor** — Similar dimensions to a Raspberry Pi Pico. Fits inside cameras, drones, and other space-constrained enclosures.
- **~$18–30 USD** — ~$18–20 for the base Ultra, ~$26–30 for the Ultra W with Wi-Fi. Still affordable for a vision-focused NPU board.
- **Runs Linux** — Full Buildroot or Ubuntu-based Linux. Access to Python, OpenCV, and RKNN inference runtime.
- **Ethernet + PoE** — 100 Mbps Ethernet with 802.3af PoE support via header. Power and data over a single cable — ideal for camera deployments.
- **Audio** — Onboard microphone and speaker header, unusual for a board this small.

### When to pick something else

| Need | Better choice |
|------|---------------|
| Wi-Fi / Bluetooth | [Radxa ZERO 3W](radxa-zero-3w.md) or [Pico 2 W](raspberry-pi-pico-2w.md) |
| More CPU power (multi-core) | [Radxa ZERO 3W](radxa-zero-3w.md) (quad A55) |
| Higher NPU performance | [Radxa ZERO 3W](radxa-zero-3w.md) (0.8 TOPS) |
| Large community / ecosystem | [Raspberry Pi Zero 2W](raspberry-pi-zero-2w.md) |
| Real-time motor control | [Pico 2 W](raspberry-pi-pico-2w.md) or ESP32 |
| More GPIO / peripherals | [Raspberry Pi Zero 2W](raspberry-pi-zero-2w.md) or [Radxa ZERO 3W](radxa-zero-3w.md) |

## Specifications

| Feature | Details |
|---------|---------|
| SoC | Rockchip RV1106G3 (Ultra) / RV1106G2 (Ultra B) |
| CPU | Single Cortex-A7 @ 1.2 GHz (32-bit ARMv7) |
| NPU | 1 TOPS INT8 (RV1106G3 Ultra) / 0.5 TOPS INT8 (RV1106G2 Ultra B) |
| ISP | Hardware ISP, up to 5 MP input, 3A support |
| RAM | 256 MB DDR3 (integrated in SoC package) |
| Storage | 8 GB eMMC onboard + microSD |
| Ethernet | 100 Mbps (with PoE support via header, 802.3af) |
| Wi-Fi | Ultra W variant: Wi-Fi 6 (802.11ax, 2.4 GHz) + BT 5.2; base Ultra: none |
| USB | 1× USB 2.0 (Type-C, OTG) |
| GPIO | 2× 26-pin headers (52 pins total: UART, SPI, I2C, PWM, ADC) |
| Audio | Onboard microphone + speaker header (MX1.25, 8-ohm/1W) |
| CSI | MIPI CSI-2 (2-lane) |
| Video codec | H.264/H.265 encode (up to 2304×1296 @ 30fps) |
| Dimensions | 50 x 50 mm |
| Operating temp | -20°C to 85°C (industrial grade) |

## Pinout

```
           ┌──────────────────────────────────┐
           │  Luckfox Pico Ultra              │
           │                                  │
           │  [CSI camera connector]          │
           │                                  │
           ├──┬────────────────────────────┬──┤
    3V3    │● │                            │ ●│ VBUS (5V)
    GND    │● │                            │ ●│ GND
   GPIO_A  │● │                            │ ●│ UART2_TX
   GPIO_B  │● │                            │ ●│ UART2_RX
   SPI_CLK │● │                            │ ●│ I2C_SDA
   SPI_MOSI│● │                            │ ●│ I2C_SCL
   SPI_MISO│● │                            │ ●│ PWM
   SPI_CS  │● │                            │ ●│ ADC_IN
   GPIO_C  │● │                            │ ●│ GPIO_D
    GND    │● │                            │ ●│ 3V3
           ├──┴────────────────────────────┴──┤
           │  [USB-C]          [Ethernet RJ45]│
           └──────────────────────────────────┘

  Note: Exact pin names vary by Luckfox Pico variant
  (Pico, Pico Plus, Pico Pro, Pico Ultra). Check the
  specific datasheet for your model. The Ultra adds
  Ethernet that other variants lack.
```

## Power Consumption

| State | Current | Notes |
|-------|---------|-------|
| Active (CPU + NPU) | ~300–400 mA | Running inference with camera |
| Active (idle) | ~150 mA | Linux booted, no workload |
| NPU inference only | ~200 mA | CPU mostly idle |

**Note:** Much lower power than quad-core SBCs. The single A7 core and efficient RV1106 design keep power consumption manageable for embedded deployments.

## NPU and Vision Pipeline

The RV1106 is designed as a "smart camera" SoC. The hardware pipeline is:

```
Camera sensor → MIPI CSI → ISP (3A) → NPU (inference) → CPU (decision logic) → Output
```

The ISP handles raw image processing (demosaic, denoise, exposure, white balance) in hardware, and the NPU runs ML models — both without significant CPU involvement. This leaves the single A7 core free for application logic, networking, and I/O.

### NPU Performance Estimates

| Model | FPS (NPU) | Notes |
|-------|-----------|-------|
| MobileNet-v2 (classification) | ~20 | INT8 quantized |
| YOLOv5n (detection) | ~8–10 | 640×640 input |
| Face detection (custom) | ~25 | Optimized small model |

### Model Conversion

Same RKNN toolchain as other Rockchip boards:

```bash
# On host PC
pip install rknn-toolkit2
# Convert model to RKNN format targeting RV1106
# Then deploy .rknn file to the board

# On board: run with rknnlite2
from rknnlite.api import RKNNLite
rknn = RKNNLite()
rknn.load_rknn('model.rknn')
rknn.init_runtime()
```

## Known Issues and Quirks

### Single Core Is Limiting

The single Cortex-A7 at 1.2 GHz is adequate when the NPU handles inference, but it struggles with concurrent tasks. Don't expect to run a heavy Python application, serve a web interface, and capture camera frames simultaneously without carefully managing CPU time.

### 256 MB RAM

Tight for complex Linux workloads. The board runs best with:
- Buildroot (minimal Linux, ~30 MB RAM used by OS)
- A focused single-purpose application
- Avoid running Python if possible — use C/C++ for the main application loop

### Wi-Fi Only on Ultra W Variant

The base Ultra has Ethernet but no Wi-Fi. The Ultra W variant adds Wi-Fi 6 + BT 5.2 but costs more (~$26–30). If you need wireless on the base model, you'll need a USB Wi-Fi dongle.

### 32-bit ARM Only

The Cortex-A7 is ARMv7 (32-bit). Some modern software (e.g., certain Docker images, newer Python packages) is dropping 32-bit ARM support. Check compatibility before committing to this platform.

### Smaller Ecosystem

Luckfox provides SDK and documentation, but the community is smaller than Raspberry Pi or even Radxa. Most resources are in Chinese, though English documentation is improving.

## Software

### Operating Systems

- **Luckfox Buildroot** — Default, minimal Linux. Best performance and lowest RAM usage.
- **Luckfox Ubuntu** — Available but tight on 256 MB RAM.

### Development

```bash
# Cross-compile with Luckfox SDK
git clone https://github.com/LuckfoxTECH/luckfox-pico.git
# SDK includes toolchain, buildroot config, and sample applications

# Deploy via SCP over Ethernet (Ultra model)
scp app root@<board-ip>:/root/
```

## Luckfox Pico Variants

| Feature | Pico | Pico Plus | Pico Pro | Pico Ultra |
|---------|------|-----------|----------|------------|
| SoC | RV1103 | RV1103 | RV1106 | RV1106G3 |
| NPU | 0.5 TOPS | 0.5 TOPS | 0.5 TOPS | **1 TOPS** |
| RAM | 64 MB | 128 MB | 256 MB | 256 MB |
| Ethernet | No | No | No | **100 Mbps + PoE** |
| Storage | SPI NOR | SPI NAND | SPI NAND | **8 GB eMMC** |
| Wi-Fi | No | No | No | **Wi-Fi 6 (W variant)** |
| Audio | No | No | No | **Mic + speaker** |
| Price | ~$6 | ~$8 | ~$10 | ~$18–30 |

## Use Cases

- **Smart camera** — Attach a MIPI camera, run face/object detection on the NPU, stream H.264/H.265 over Ethernet. Purpose-built for this use case.
- **Industrial vision inspection** — Industrial temperature range, small size, dedicated ISP+NPU pipeline for quality inspection tasks.
- **Doorbell / security camera** — Detect people/faces locally, send alerts without cloud dependency.
- **Vision coprocessor** — Pair with a microcontroller (ESP32, Pico) — Luckfox handles camera + inference, MCU handles actuators and real-time I/O.

## In This Repository

The Luckfox Pico Ultra is not currently used in any project. It is a candidate for:
- A dedicated vision coprocessor for the robocar — handles camera capture + object detection via NPU, sends detection results to the main controller ESP32 via UART
- Replacing the ESP32-CAM for applications where on-device inference is preferred over cloud API calls, at the cost of losing Wi-Fi (would need Ethernet or an external wireless module)

## References

- [Luckfox Pico Product Page](https://www.luckfox.com/Luckfox-Pico)
- [Luckfox Pico Wiki](https://wiki.luckfox.com/Luckfox-Pico/Luckfox-Pico-quick-start)
- [Luckfox Pico SDK (GitHub)](https://github.com/LuckfoxTECH/luckfox-pico)
- [RV1106 Brief (Rockchip)](https://www.rock-chips.com/a/en/products/RK11_Series/index.html)
- [RKNN Toolkit2 (GitHub)](https://github.com/rockchip-linux/rknn-toolkit2)
