# Radxa ZERO 3W

Compact Linux SBC with a Rockchip RK3566 quad-core CPU, 0.8 TOPS NPU, and Wi-Fi 6 in a form factor similar to the Raspberry Pi Zero. The ZERO 3W stands out as one of the cheapest boards with a dedicated neural processing unit, making it a strong option for edge AI and computer vision projects where cloud API latency or cost is a concern.

## Why Radxa ZERO 3W

- **0.8 TOPS NPU** — Dedicated neural network accelerator runs YOLOv5-nano, MobileNet-SSD, and similar models at usable frame rates without touching the CPU. This is the key differentiator from Raspberry Pi alternatives.
- **Quad A55 @ 1.6 GHz** — Cortex-A55 cores are ~20% faster per-clock than the A53 cores in Pi Zero 2 W, and clocked 60% higher. Noticeable difference for CPU-bound tasks.
- **Up to 8 GB RAM** — Available in 1/2/4/8 GB configurations. 4 GB is comfortable for Linux + OpenCV + inference simultaneously.
- **Wi-Fi 6 + BT 5.4** — 802.11ax on both 2.4 and 5 GHz bands. Better throughput and lower latency in congested environments.
- **Mali-G52 GPU** — OpenGL ES 3.2, Vulkan 1.1. Useful for hardware-accelerated video decode and display compositing.
- **MIPI CSI-2** — Camera connector for direct camera module attachment.
- **~$16–50 USD** — Starting at ~$16 for 1 GB RAM, ~$33 for 4 GB. The 4 GB model offers excellent price/performance for edge AI.

### When to pick something else

| Need | Better choice |
|------|---------------|
| Massive community + ecosystem | [Raspberry Pi Zero 2W](raspberry-pi-zero-2w.md) (unmatched documentation) |
| Real-time motor control / bare metal | [Pico 2 W](raspberry-pi-pico-2w.md) or ESP32 |
| Ultra-low power (battery months) | [ESP32-C3 Super Mini](esp32-c3-super-mini.md) or [Pico 2 W](raspberry-pi-pico-2w.md) |
| Higher NPU performance (>2 TOPS) | Orange Pi 5 (RK3588S, 6 TOPS) |
| Zigbee / Thread / Matter | [XIAO ESP32-C6](xiao-esp32c6.md) |
| Tiny form factor + NPU | [Luckfox Pico Ultra](luckfox-pico-ultra.md) (smaller, cheaper, but weaker CPU) |

## Specifications

| Feature | Details |
|---------|---------|
| SoC | Rockchip RK3566 |
| CPU | Quad Cortex-A55 @ up to 1.8 GHz (64-bit ARMv8.2-A) |
| GPU | Mali-G52 2EE (OpenGL ES 3.2, Vulkan 1.1, OpenCL 2.0) |
| NPU | 0.8 TOPS (INT8), supports TensorFlow Lite, ONNX, Caffe, PyTorch models |
| RAM | 1 / 2 / 4 / 8 GB LPDDR4 |
| Storage | microSD, optional eMMC module |
| Wi-Fi | 802.11ax (Wi-Fi 6, 2.4 + 5 GHz) |
| Bluetooth | Bluetooth 5.4 LE |
| USB | 1× USB 3.0 HOST Type-C, 1× USB 2.0 OTG Type-C (power + data) |
| GPIO | 40-pin header (Raspberry Pi compatible layout) |
| CSI | MIPI CSI-2 (4-lane) |
| Video output | Micro-HDMI (up to 1080p@60) |
| Audio | 3.5mm headphone jack |
| Dimensions | 65 x 30 mm |
| Operating temp | 0°C to 50°C (wider range with active cooling) |

## Pinout

```
                 ┌──────────────────────────────────────────┐
                 │  Radxa ZERO 3W                           │
                 │                                          │
  [micro-HDMI]   │  ┌─────────────────────────────────┐     │ [CSI camera]
                 │  │ 40-pin GPIO header               │     │
                 │  └─────────────────────────────────┘     │
  [USB-C OTG]    │              [3.5mm audio]               │
                 └──────────────────────────────────────────┘

  40-Pin Header (Raspberry Pi compatible):

     3V3  (1) (2)  5V
   GPIO2  (3) (4)  5V          I2C SDA
   GPIO3  (5) (6)  GND         I2C SCL
   GPIO4  (7) (8)  GPIO14      UART TX
     GND  (9) (10) GPIO15      UART RX
  GPIO17 (11) (12) GPIO18
  GPIO27 (13) (14) GND
  GPIO22 (15) (16) GPIO23
     3V3 (17) (18) GPIO24
  GPIO10 (19) (20) GND         SPI MOSI
   GPIO9 (21) (22) GPIO25      SPI MISO
  GPIO11 (23) (24) GPIO8       SPI SCLK / SPI CE0
     GND (25) (26) GPIO7       SPI CE1
   GPIO0 (27) (28) GPIO1
   GPIO5 (29) (30) GND
   GPIO6 (31) (32) GPIO12
  GPIO13 (33) (34) GND
  GPIO19 (35) (36) GPIO16
  GPIO26 (37) (38) GPIO20
     GND (39) (40) GPIO21

  Note: Pin numbering follows Raspberry Pi convention.
  Actual RK3566 GPIO bank/pin numbers differ — check Radxa docs
  for the mapping to RK3566 GPIO bank references (e.g., GPIO3_C4).
```

### Key Interfaces

| Interface | Pins | Notes |
|-----------|------|-------|
| I2C | Pin 3 (SDA), Pin 5 (SCL) | 3.3V logic |
| SPI | Pin 19/21/23/24/26 | |
| UART | Pin 8 (TX), Pin 10 (RX) | Debug/serial console |
| PWM | Multiple pins support PWM | Via sysfs or libgpiod |

## Power Consumption

| State | Current | Notes |
|-------|---------|-------|
| Idle (headless) | ~400 mA | Wi-Fi connected, minimal services |
| Active (4 cores + NPU) | ~800 mA | CPU + NPU inference workload |
| Active (NPU only) | ~500 mA | CPU idle, NPU running inference |
| Idle (Wi-Fi off) | ~250 mA | |
| Shutdown (halt) | ~30 mA | No hardware power-off |

**Supply:** Use a 5V/3A USB-C power supply. Insufficient power causes random crashes and SD card corruption.

## NPU — The Key Feature

The RK3566's 0.8 TOPS neural processing unit is the primary reason to choose this board over a Raspberry Pi Zero 2 W. It offloads ML inference from the CPU, allowing the A55 cores to handle other tasks (camera capture, networking, display) while the NPU runs the model.

### Supported Frameworks

Models must be converted to RKNN format using the `rknn-toolkit2` on a host PC (x86, Python):

```bash
# On host PC: convert a TFLite model to RKNN
pip install rknn-toolkit2
python3 -c "
from rknn.api import RKNN
rknn = RKNN()
rknn.config(target_platform='rk3566')
rknn.load_tflite(model='mobilenet_v2.tflite')
rknn.build(do_quantization=True, dataset='calibration.txt')
rknn.export_rknn('mobilenet_v2.rknn')
"

# On the board: run inference
pip install rknnlite2
python3 -c "
from rknnlite.api import RKNNLite
rknn = RKNNLite()
rknn.load_rknn('mobilenet_v2.rknn')
rknn.init_runtime()
# ... run inference
"
```

### NPU Performance Estimates

| Model | FPS (NPU) | FPS (CPU only) | Notes |
|-------|-----------|----------------|-------|
| MobileNet-v2 (classification) | ~30 | ~5 | INT8 quantized |
| YOLOv5n (detection) | ~10–15 | ~1–2 | 640×640 input |
| MobileNet-SSD (detection) | ~20 | ~3 | 300×300 input |

These are approximate. Actual FPS depends on input resolution, quantization, and pre/post-processing overhead.

## Known Issues and Quirks

### RKNN Toolkit Complexity

Converting models to RKNN format is not as straightforward as TFLite. The `rknn-toolkit2` has specific version requirements, some operator limitations, and the documentation is mostly in Chinese with partial English translations.

### BSP vs Mainline Linux

Radxa provides both:
- **BSP kernel** (Rockchip-based) — Full hardware support including NPU, GPU acceleration, camera ISP
- **Mainline kernel** — Better long-term support but NPU and some hardware features may not work

For NPU projects, you must use the BSP kernel. This means you're tied to Radxa's kernel update schedule.

### GPIO Libraries

The Raspberry Pi-compatible header layout is helpful, but `RPi.GPIO` doesn't work directly. Use:
- `libgpiod` and `gpiod` Python bindings (recommended)
- Radxa's `mraa` library
- `sysfs` GPIO (deprecated but functional)

### Smaller Community

Compared to Raspberry Pi, there are fewer tutorials, forum posts, and Stack Overflow answers. The official Radxa wiki and forum are the primary support channels.

### Thermal Management

The RK3566 throttles under sustained load. A heatsink is recommended, and active cooling (small fan) is beneficial for continuous NPU inference workloads.

## Software

### Operating Systems

- **Radxa Debian** — Default OS, best hardware support (NPU, GPU, camera)
- **Armbian** — Community builds, good for headless/server use
- **Ubuntu** — Available via Radxa images
- **Dietpi** — Minimal image available

### Essential Packages

```bash
# NPU runtime
sudo apt install rknpu2-runtime

# Camera
sudo apt install libcamera-dev

# OpenCV
sudo apt install python3-opencv
```

## Radxa ZERO 3W vs Raspberry Pi Zero 2W

| Feature | Radxa ZERO 3W | [Pi Zero 2W](raspberry-pi-zero-2w.md) |
|---------|--------------|-------------|
| CPU | 4× A55 @ 1.6 GHz | 4× A53 @ 1 GHz |
| RAM | 1–8 GB LPDDR4 | 512 MB LPDDR2 |
| NPU | 0.8 TOPS | None |
| GPU | Mali-G52 (Vulkan 1.1) | VideoCore IV (GLES 2.0) |
| Wi-Fi | Wi-Fi 6 (2.4+5 GHz) | Wi-Fi 4 (2.4 GHz only) |
| Bluetooth | 5.4 | 4.2 |
| Storage | microSD + eMMC option | microSD only |
| Video out | 4K@60 HDMI | 1080p60 HDMI |
| Camera | MIPI CSI-2 | MIPI CSI-2 |
| Price | ~$16 (1 GB) – $50 (8 GB) | ~$15 |
| Community | Growing | Excellent |
| NPU ML perf | ~15 FPS YOLOv5n | ~2 FPS (CPU only) |

**Bottom line:** If you need local AI inference, the Radxa ZERO 3W is significantly more capable. If you need the best community support and don't need an NPU, the Pi Zero 2 W wins on ecosystem.

## Use Cases

- **Edge AI vision system** — Camera + NPU for real-time object detection without cloud API calls. Ideal for robotics, security cameras, or industrial inspection.
- **Local inference for robocar** — Run obstacle detection and navigation models on-device, send movement commands to a motor controller MCU over UART.
- **Smart home hub** — Run Home Assistant, Zigbee2MQTT, or other home automation software with enough RAM (4 GB+) for comfortable operation.
- **Media player** — 4K HDMI output + hardware video decode for Kodi or similar.
- **AI-powered camera** — Attach a MIPI camera, run detection models, stream annotated video or trigger actions.

## In This Repository

The Radxa ZERO 3W is not currently used in any project. It is a strong candidate for:
- Replacing the ESP32-CAM + cloud API architecture with on-device inference (NPU runs obstacle detection, UART sends commands to motor controller)
- A more capable vision system that eliminates Claude API latency and cost for real-time navigation
- Running local Ollama instances for the robocar's AI backend (with 4+ GB RAM model)

## References

- [Radxa ZERO 3W Product Page](https://radxa.com/products/zeros/zero3w)
- [Radxa ZERO 3W Wiki](https://docs.radxa.com/en/zero/zero3)
- [RK3566 Datasheet (Rockchip)](https://www.rock-chips.com/a/en/products/RK35_Series/2021/0113/1274.html)
- [RKNN Toolkit2 (GitHub)](https://github.com/rockchip-linux/rknn-toolkit2)
- [Radxa ZERO 3W GPIO Pinout](https://docs.radxa.com/en/zero/zero3/hardware-design/hardware-interface)
