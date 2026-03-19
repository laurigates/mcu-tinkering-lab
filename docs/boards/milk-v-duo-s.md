# Milk-V Duo S

RISC-V + ARM hybrid SBC built on the SOPHGO SG2000 SoC — a dual-architecture chip with a RISC-V C906 main core, an ARM Cortex-A53 core, and a 0.5 TOPS INT8 TPU (tensor processing unit). One of the most interesting experimental boards in this price range, combining RISC-V Linux with hardware ML acceleration for ~$10.

## Why Milk-V Duo S

- **RISC-V + ARM on one chip** — The SG2000 has both a RISC-V C906 @ 1 GHz and a Cortex-A53 @ 1 GHz. You can run Linux on either architecture, or use one core for Linux and the other for RTOS tasks.
- **0.5 TOPS TPU** — Dedicated INT8 tensor processing unit for ML inference. Supports ONNX, Caffe, TFLite, and PyTorch model conversion via SOPHGO's toolchain.
- **~$11–14 USD** — $11 without Wi-Fi, $14 with Wi-Fi module. Extremely cheap for a board with both RISC-V Linux capability and an NPU/TPU.
- **Dual OS architecture** — Run Linux on the main core and FreeRTOS on the secondary core simultaneously. The RTOS core can handle real-time I/O while Linux handles networking and inference.
- **Wi-Fi 6 + Ethernet** — Optional Wi-Fi 6/BT 5 module plus 100 Mbps Ethernet.
- **Dual cameras** — Two MIPI CSI-2 connectors for stereo vision or front/rear camera setups.
- **MIPI DSI output** — 4-lane display interface for connecting small LCD panels.
- **Compact** — Small form factor suitable for embedded applications.

### When to pick something else

| Need | Better choice |
|------|---------------|
| Mature ecosystem / community | [Raspberry Pi Zero 2W](raspberry-pi-zero-2w.md) or any ESP32 |
| Higher NPU performance | [Radxa ZERO 3W](radxa-zero-3w.md) (0.8 TOPS, better toolchain) |
| Production-ready product | [Radxa ZERO 3W](radxa-zero-3w.md) or [Raspberry Pi Zero 2W](raspberry-pi-zero-2w.md) |
| Wi-Fi 6 / Bluetooth 5.4 | [Radxa ZERO 3W](radxa-zero-3w.md) |
| More RAM | [Radxa ZERO 3W](radxa-zero-3w.md) (up to 8 GB) |
| Real-time motor control | [Pico 2 W](raspberry-pi-pico-2w.md) or ESP32 |

## Specifications

| Feature | Details |
|---------|---------|
| SoC | SOPHGO SG2000 |
| CPU (RISC-V) | C906 @ 1 GHz (64-bit, RV64GCV — includes vector extension) |
| CPU (ARM) | Cortex-A53 @ 1 GHz (64-bit ARMv8-A) |
| Secondary core | C906 @ 700 MHz (for FreeRTOS / RTOS tasks) |
| MCU | 8051 co-processor (6 KB SRAM) |
| ISP | 5 MP @ 30 FPS hardware image signal processor |
| TPU/NPU | 0.5 TOPS INT8, BF16/FP32 support |
| RAM | 512 MB DDR3 |
| Storage | microSD + 8 GB eMMC onboard |
| Wi-Fi | 802.11ax (Wi-Fi 6, 2.4 GHz) — optional module |
| Bluetooth | Bluetooth 5 LE — optional module |
| Ethernet | 100 Mbps |
| USB | 1× USB 2.0 Type-C (host/device) |
| GPIO | 26-pin header (I2C, SPI, UART, PWM, ADC, GPIO) |
| CSI | 2× MIPI CSI-2 (2-lane each) — dual cameras supported |
| Video output | MIPI DSI (4-lane) — no HDMI |
| Audio | I2S interface |
| Dimensions | ~43 x 43 mm |
| Operating temp | -40°C to 85°C (industrial grade) |

## Pinout

```
           ┌──────────────────────────────────┐
           │  Milk-V Duo S                    │
           │                                  │
           │  [CSI camera]                    │
           │                                  │
           ├──┬────────────────────────────┬──┤
    GP0    │● │                            │ ●│  VBUS (5V)
    GP1    │● │                            │ ●│  VSYS
    GND    │● │                            │ ●│  GND
    GP2    │● │                            │ ●│  3V3
    GP3    │● │                            │ ●│  GP22
    GP4    │● │                            │ ●│  GP21
    GP5    │● │                            │ ●│  GP20
    GND    │● │                            │ ●│  GP19
    GP6    │● │                            │ ●│  GP18
    GP7    │● │                            │ ●│  GND
    GP8    │● │                            │ ●│  GP17
    GP9    │● │                            │ ●│  GP16
    GND    │● │                            │ ●│  GP15
           ├──┴────────────────────────────┴──┤
           │  [USB-C]     [Ethernet RJ45]     │
           └──────────────────────────────────┘

  Note: Pin assignments are approximate. Check the official
  Milk-V Duo S pinout diagram for exact GPIO-to-function mapping.
  Many pins are multiplexed between UART, SPI, I2C, PWM, and ADC.
```

### Key Interfaces

| Interface | Notes |
|-----------|-------|
| I2C | Multiple I2C buses available |
| SPI | 1× user-accessible |
| UART | Multiple UART channels |
| PWM | Multiple PWM outputs |
| ADC | SARADC input channels |

## Power Consumption

| State | Current | Notes |
|-------|---------|-------|
| Active (CPU + TPU) | ~400 mA | Running inference |
| Active (idle) | ~200 mA | Linux booted, idle |
| TPU inference | ~250 mA | CPU mostly idle |

## Dual-Architecture Design

The SG2000's most interesting feature is its multi-core heterogeneous architecture:

```
┌─────────────────────────────────────────────┐
│  SG2000 SoC                                 │
│                                             │
│  ┌─────────────┐  ┌─────────────┐           │
│  │ C906 @ 1GHz │  │ A53 @ 1GHz  │  ← Pick  │
│  │ (RISC-V)    │  │ (ARM)       │    one    │
│  │ Main Linux  │  │ Main Linux  │    for    │
│  │ core        │  │ core        │    Linux  │
│  └─────────────┘  └─────────────┘           │
│                                             │
│  ┌─────────────┐  ┌─────────────┐           │
│  │ C906 @700MHz│  │ TPU 0.5TOPS │           │
│  │ (RISC-V)    │  │ (INT8)      │           │
│  │ FreeRTOS    │  │ ML inference│           │
│  └─────────────┘  └─────────────┘           │
└─────────────────────────────────────────────┘
```

You choose at build time whether Linux runs on the RISC-V or ARM core. The secondary RISC-V core always runs FreeRTOS and can handle real-time tasks like sensor polling, PWM generation, or protocol bit-banging — similar in concept to the ESP32's dual-core architecture but with a full Linux system on the main core.

### Inter-core Communication

The main Linux core and the FreeRTOS core communicate via shared memory mailbox. SOPHGO provides a messaging API for sending commands and data between the two worlds.

## TPU / ML Inference

The 0.5 TOPS INT8 TPU supports common ML model formats via SOPHGO's model conversion toolchain:

```bash
# On host PC: convert model with TPU-MLIR
# SOPHGO provides tpu-mlir toolchain for model compilation
git clone https://github.com/sophgo/tpu-mlir.git

# Convert ONNX model to cvimodel format
model_transform.py --model_name mobilenet \
  --model_def mobilenet.onnx \
  --input_shapes [[1,3,224,224]] \
  --pixel_format rgb \
  --output mobilenet.mlir

model_deploy.py --mlir mobilenet.mlir \
  --quantize INT8 \
  --chip cv181x \
  --output mobilenet.cvimodel
```

### TPU Performance Estimates

| Model | FPS (TPU) | Notes |
|-------|-----------|-------|
| MobileNet-v2 (classification) | ~15–20 | INT8 quantized |
| YOLOv5n (detection) | ~5–8 | 640×640 input |

## Known Issues and Quirks

### Immature Ecosystem

This is the biggest caveat. The Milk-V Duo S has:
- Limited English documentation (improving, but many resources are in Chinese)
- A small community compared to Raspberry Pi or even Radxa
- SDK and toolchain that are still evolving
- Fewer pre-built OS images and packages

It's an experimental/enthusiast board, not a production-ready platform (yet).

### RISC-V Software Compatibility

Running Linux on the RISC-V core means some software won't be available:
- No Docker (limited RISC-V container support)
- Some Python packages lack RISC-V wheels
- Pre-built binaries are scarce — expect to compile from source frequently

Running on the ARM core avoids these issues but defeats part of the board's appeal.

### No HDMI Output

The Duo S has a MIPI DSI 4-lane display output for small LCD panels, but no HDMI. For desktop use, you'd need a DSI-to-HDMI adapter or a compatible MIPI display. Primarily managed via SSH or serial console.

### 512 MB RAM

Adequate for focused embedded applications but tight for complex Linux workloads. Similar constraints to the Raspberry Pi Zero 2 W.

### TPU Toolchain Learning Curve

SOPHGO's TPU-MLIR toolchain is less mature and less documented than Rockchip's RKNN toolkit. Model conversion can require trial and error, especially for custom architectures.

## Software

### Operating Systems

- **Milk-V Buildroot** — Default, minimal Linux. Recommended for embedded use.
- **Milk-V Debian** — Available for both RISC-V and ARM boot modes.
- **Alpine Linux** — Community port available.

### Development

```bash
# Milk-V Duo SDK
git clone https://github.com/milkv-duo/duo-buildroot-sdk.git

# Build system image
./build.sh milkv-duos-sd
```

## Milk-V Duo S vs Similar Boards

| Feature | Milk-V Duo S | [Luckfox Pico Ultra](luckfox-pico-ultra.md) | [Radxa ZERO 3W](radxa-zero-3w.md) |
|---------|-------------|----------------|------------|
| CPU | C906/A53 1-core 1 GHz | A7 1-core 1.2 GHz | 4× A55 1.6 GHz |
| Architecture | RISC-V + ARM | ARM | ARM |
| RAM | 512 MB | 256 MB | 1–8 GB |
| NPU/TPU | 0.5 TOPS | 0.5 TOPS | 0.8 TOPS |
| Wi-Fi | Wi-Fi 6 (optional) | None | Wi-Fi 6 |
| Camera | CSI-2 | CSI-2 (with ISP) | CSI-2 |
| RTOS core | Yes (C906) | No | No |
| Video out | None | None | 4K HDMI |
| Price | ~$11–14 | ~$15 | ~$16–50 |
| Ecosystem | Immature | Small | Growing |

## Use Cases

- **RISC-V experimentation** — Learn RISC-V Linux development on real hardware with ML capability. Great for education and research.
- **Hybrid real-time + Linux** — FreeRTOS core handles time-critical I/O (sensors, actuators) while Linux core runs networking, inference, and application logic.
- **Ultra-cheap edge AI** — $10 for Linux + 0.5 TOPS NPU is remarkable. Good for disposable/high-volume IoT deployments.
- **Protocol bridge** — RTOS core bit-bangs custom protocols while Linux core handles Wi-Fi/Ethernet networking.

## In This Repository

The Milk-V Duo S is not currently used in any project. It is a candidate for:
- RISC-V experimentation and learning
- Evaluating the dual-core (Linux + RTOS) architecture as a potential single-board solution for the robocar (Linux handles vision/AI, RTOS core handles motor PWM and sensor reading)
- Prototyping ultra-cheap edge AI nodes

**Recommendation:** This board is best suited for experimentation and learning, not production. For a production robocar vision system, the [Radxa ZERO 3W](radxa-zero-3w.md) is a safer choice with better tooling and community support.

## References

- [Milk-V Duo S Product Page](https://milkv.io/duo-s)
- [Milk-V Duo S Documentation](https://milkv.io/docs/duo/getting-started/duos)
- [SOPHGO SG2000 Technical Reference Manual](https://github.com/sophgo/sophgo-doc)
- [Milk-V Duo SDK (GitHub)](https://github.com/milkv-duo/duo-buildroot-sdk)
- [TPU-MLIR (GitHub)](https://github.com/sophgo/tpu-mlir)
