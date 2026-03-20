# Datasheets

Reference index for all hardware components used in this project.

## Naming Scheme

Files use a flat `category--component-name.md` convention:

| Prefix | Category |
|--------|----------|
| `board--` | Development boards and modules |
| `mcu--` | Microcontroller/SoC silicon datasheets |
| `camera--` | Image sensors |
| `display--` | Display controllers |
| `driver--` | Motor drivers, PWM controllers, LED drivers |
| `actuator--` | Servos, motors |
| `power--` | Voltage regulators, converters |
| `radio--` | LoRa, WiFi, BLE transceivers |
| `led--` | Addressable/smart LEDs |

## Components

### Boards
- [Heltec WiFi LoRa 32 V1](board--heltec-wifi-lora-32-v1.md) — Main controller
- [ESP32-CAM AI Thinker](board--esp32-cam-ai-thinker.md) — Camera module
- [Waveshare ESP32-S3-Zero](board--waveshare-esp32-s3-zero.md) — IT troubleshooter / Xbox-Switch bridge

### MCUs
- [ESP32](mcu--esp32.md) — Dual-core LX6, WiFi + BT (Heltec, ESP32-CAM)
- [ESP32-S3](mcu--esp32-s3.md) — Dual-core LX7, WiFi + BLE 5 (Waveshare S3-Zero)

### Peripherals
- [OV2640](camera--ov2640.md) — 2MP CMOS image sensor
- [SSD1306](display--ssd1306.md) — 128x64 OLED controller
- [PCA9685](driver--pca9685.md) — 16-ch 12-bit PWM/servo driver
- [TB6612FNG](driver--tb6612fng.md) — Dual DC motor driver
- [SG90](actuator--sg90.md) — 9g micro servo
- [XL6009](power--xl6009.md) — DC-DC boost converter
- [SX1276](radio--sx1276.md) — LoRa transceiver
- [WS2812B](led--ws2812b.md) — Addressable RGB LED
