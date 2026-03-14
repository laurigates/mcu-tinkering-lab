# ESP32-S3

Dual-core Wi-Fi + Bluetooth LE SoC with vector instructions for AI/ML acceleration, USB OTG, and up to 45 GPIOs. The ESP32-S3 is Espressif's most capable general-purpose chip for projects that need both wireless connectivity and on-device intelligence.

## Why ESP32-S3

The ESP32-S3 fills a gap that other ESP32 variants leave open:

- **Dual-core + AI acceleration** — The LX7 vector instructions accelerate TensorFlow Lite Micro inference (face detection, keyword spotting) without an external accelerator.
- **USB OTG** — Native USB 1.1 host/device (HID keyboards, CDC-ACM, MSC) without an external USB-UART bridge. This is the main reason we chose it for the [IT Troubleshooter](../../packages/esp32-projects/it-troubleshooter/) project.
- **Wi-Fi + BLE simultaneously** — Dual-core lets one core handle radio stacks while the other runs application logic uninterrupted. A single device can bridge BLE sensors to MQTT over Wi-Fi.
- **Large PSRAM options** — Up to 16MB octal PSRAM for camera frame buffers, audio processing, or ML model weights.
- **~$8 USD** — Boards like the Waveshare ESP32-S3-Zero start around $8.

### When to pick something else

| Need | Better choice |
|------|---------------|
| WiFi 6 / Thread / Zigbee | ESP32-C6 |
| Classic Bluetooth (A2DP, SPP) | Original ESP32 |
| Ultra-low-power sensor node | ESP32-C3 or ESP32-H2 |
| LoRa radio | Heltec LoRa32 (original ESP32) |

## Specifications

| Feature | Details |
|---------|---------|
| MCU | Xtensa LX7 dual-core @ 240 MHz |
| SRAM | 512 KB |
| ROM | 384 KB |
| Flash | 4–32 MB (module-dependent) |
| PSRAM | 0 / 2 / 8 / 16 MB (module-dependent) |
| Wi-Fi | 802.11 b/g/n (Wi-Fi 4, 2.4 GHz) |
| Bluetooth | Bluetooth 5.0 LE (no Classic BT) |
| USB | Full-speed USB 1.1 OTG + USB-Serial-JTAG |
| GPIOs | Up to 45 |
| ADC | 2× 12-bit SAR ADC, 20 channels |
| Touch | 14 capacitive touch GPIOs |
| SPI | 4× SPI (2 usable by user) |
| I2C | 2× I2C |
| I2S | 2× I2S |
| UART | 3× UART |
| PWM | 8-ch LED PWM, 2× MCPWM |
| Camera | DVP 8/16-bit camera interface |
| LCD | 8/16-bit parallel LCD interface |
| Operating temp | -40°C to 85°C |

## Common Module Variants

| Module | Flash | PSRAM | Notes |
|--------|-------|-------|-------|
| ESP32-S3-WROOM-1 (N4) | 4 MB | None | Lowest cost, no PSRAM |
| ESP32-S3-WROOM-1 (N8R2) | 8 MB | 2 MB (quad) | Good balance for most projects |
| ESP32-S3-WROOM-1 (N8R8) | 8 MB | 8 MB (octal) | Camera + ML workloads |
| ESP32-S3-WROOM-1 (N16R8) | 16 MB | 8 MB (octal) | Large firmware + ML models |
| ESP32-S3-WROOM-2 (N32R8V) | 32 MB | 8 MB (octal) | Maximum storage |

**Octal vs Quad PSRAM:** Octal PSRAM (8-bit bus) is faster but uses GPIO33–37 internally, making those pins unavailable. Quad PSRAM (4-bit bus) uses fewer pins but has lower bandwidth.

## Popular Dev Boards

| Board | Flash/PSRAM | Form Factor | USB | Price |
|-------|-------------|-------------|-----|-------|
| Waveshare ESP32-S3-Zero | 4 MB / 2 MB | Stamp (tiny) | USB-C | ~$8 |
| ESP32-S3-DevKitC-1 | 8–16 MB / 8–16 MB | Full-size | USB-C (dual) | ~$10 |
| Seeed XIAO ESP32S3 | 8 MB / 8 MB | Thumb-sized | USB-C | ~$8 |
| Seeed XIAO ESP32S3 Sense | 8 MB / 8 MB | Thumb-sized | USB-C | ~$14 (with camera + mic) |
| Freenove ESP32-S3-WROOM | 8–16 MB / 8 MB | Full-size | USB-C | ~$10 |

## GPIO Considerations

### Strapping Pins

| GPIO | Function | Safe State at Boot |
|------|----------|-------------------|
| GPIO0 | Boot mode | HIGH for normal boot, LOW for download mode |
| GPIO3 | JTAG signal source | Floating |
| GPIO45 | VDD_SPI voltage | LOW for 3.3V flash (most boards) |
| GPIO46 | Boot mode / ROM log | LOW or floating |

### Pins Reserved for Octal PSRAM (N8R8 / N16R8 / N32R8V modules)

GPIO33, GPIO34, GPIO35, GPIO36, GPIO37 — used internally for the octal SPI bus to PSRAM. **Do not use these pins** on modules with octal PSRAM.

### Pins Reserved for Flash

GPIO26–32 are connected to the in-package flash on WROOM modules. **Do not use** for external peripherals.

### USB Pin Sharing

The USB OTG peripheral uses GPIO19 (D-) and GPIO20 (D+). When USB OTG is active (e.g., TinyUSB for HID), the USB-Serial-JTAG debug interface is unavailable — use a separate UART adapter for logging. This constraint is documented in detail in [ADR-005](../blueprint/adrs/ADR-005-it-troubleshooter-hardware.md).

### USB-Serial-JTAG Reset Behavior

The ESP32-S3's USB-Serial-JTAG has unreliable reset behavior compared to traditional USB-UART bridges (CP2102, CH340):

- **`esptool --after hard-reset`** does not reliably trigger a chip reset over USB-Serial-JTAG. After flashing, the board may continue running old firmware until manually reset.
- **Unplugging/replugging USB does NOT reset the SoC.** Board power regulation holds charge, and USB-Serial-JTAG reconnection does not trigger a chip reset.
- **`cat` on macOS does not reliably read USB-Serial-JTAG CDC devices.** Use pyserial instead for serial monitoring.

**Reliable reset method:** Send DTR/RTS control signals via pyserial:

```python
import serial, time
s = serial.Serial('/dev/cu.usbmodemXXXX', 115200)
s.dtr = False; s.rts = True; time.sleep(0.1)
s.rts = False; s.dtr = True; time.sleep(0.1)
s.dtr = False; s.close()
```

All ESP32-S3 project justfiles in this repo include `just reset` and `just monitor` recipes using this method. The monitor recipe automatically resets the board on connect so boot messages are captured.

## AI/ML Capabilities

The ESP32-S3's vector instructions accelerate 8-bit and 16-bit integer math used in quantized neural networks. Practical on-device inference use cases:

- **Keyword spotting** — "Hey Siri"-style wake words via ESP-SR
- **Face detection/recognition** — Local processing without cloud API calls
- **Person detection** — TensorFlow Lite Micro with MobileNet
- **Audio classification** — Environmental sound recognition

With 8 MB PSRAM, you can buffer camera frames and run inference at a few FPS — enough for a smart doorbell or security camera that processes locally before deciding whether to upload.

## Smart Home Use Cases

The combination of dual wireless radios and AI makes the ESP32-S3 particularly effective for smart home projects:

- **BLE-to-WiFi bridge** — Collect data from BLE sensors (temperature, air quality) and publish via MQTT/HTTP over WiFi, all on one chip.
- **Local AI doorbell** — Camera + face detection without cloud dependency.
- **Voice-controlled device** — On-device wake word detection + WiFi for command execution.
- **USB automation** — HID keyboard emulation for network-isolated devices (see IT Troubleshooter project).

## Power Consumption

| State | Current | Notes |
|-------|---------|-------|
| Active (Wi-Fi TX) | ~310 mA peak | Varies with TX power |
| Active (Wi-Fi RX) | ~100 mA | |
| Active (BLE) | ~100 mA | |
| Modem sleep | ~12 mA | CPU active, radio off |
| Light sleep | ~0.3 mA | Auto wake on timer/GPIO |
| Deep sleep | ~7 µA | RTC memory retained |
| Hibernation | ~2.5 µA | Minimal state retained |

## ESP-IDF Configuration

### sdkconfig.defaults for S3

```ini
# Target
CONFIG_IDF_TARGET="esp32s3"

# PSRAM (for modules with PSRAM)
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y

# USB OTG (for HID/CDC projects)
CONFIG_TINYUSB=y
CONFIG_TINYUSB_HID_ENABLED=y

# AI/ML optimization
CONFIG_ESP32S3_DEFAULT_CPU_FREQ_240=y
```

### CMakeLists.txt target

```cmake
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
)
# Set target in project CMakeLists.txt or via:
# idf.py set-target esp32s3
```

## In This Repository

The ESP32-S3 is currently used in:

- **[IT Troubleshooter](../../packages/esp32-projects/it-troubleshooter/)** — USB HID keyboard emulator using the Waveshare ESP32-S3-Zero. Selected specifically for USB OTG support ([ADR-005](../blueprint/adrs/ADR-005-it-troubleshooter-hardware.md)).

Future candidates for ESP32-S3:
- Robocar camera upgrade (replace ESP32-CAM with S3-based camera board for on-device ML)
- Standalone voice command module

## References

- [ESP32-S3 Series Datasheet v2.0 (Espressif)](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
- [ESP32-S3-WROOM-1/1U Datasheet v1.7](https://documentation.espressif.com/esp32-s3-wroom-1_wroom-1u_datasheet_en.html)
- [Waveshare ESP32-S3-Zero Wiki](https://www.waveshare.com/wiki/ESP32-S3-Zero)
- [Why the ESP32-S3 is the best $8 you'll ever spend (XDA Developers)](https://www.xda-developers.com/why-the-esp32-s3-is-the-best-8-youll-ever-spend-on-your-smart-home/)
- [ESP32 Wikipedia](https://en.wikipedia.org/wiki/ESP32)
