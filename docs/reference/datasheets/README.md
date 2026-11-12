# Datasheets

Reference index for all hardware components used in this project.

Choosing between servos (frame rate, analog vs digital, size compatibility):
see [servo selection](../servo-selection.md).

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
| `sensor--` | Distance, motion and environmental sensors |
| `io--` | GPIO / I/O expanders |

## Components

### Boards
- [Heltec WiFi LoRa 32 V1](board--heltec-wifi-lora-32-v1.md) — Main controller
- [ESP32-CAM AI Thinker](board--esp32-cam-ai-thinker.md) — Camera module
- [Waveshare ESP32-S3-Zero](board--waveshare-esp32-s3-zero.md) — IT troubleshooter / Xbox-Switch bridge
- [Seeed Studio XIAO RP2350](board--xiao-rp2350.md) — Balancebot

### MCUs
- [ESP32](mcu--esp32.md) — Dual-core LX6, WiFi + BT (Heltec, ESP32-CAM)
- [ESP32-S3](mcu--esp32-s3.md) — Dual-core LX7, WiFi + BLE 5 (Waveshare S3-Zero)
- [RP2350](mcu--rp2350.md) — Dual Cortex-M33/Hazard3, PIO, no radio (XIAO RP2350)
- [ESP8285](mcu--esp8285.md) — ESP8266 core with 1–2 MB flash in package, NRND (ESP_IR_TR IR transceiver)

### Peripherals
- [OV2640](camera--ov2640.md) — 2MP CMOS image sensor
- [OV3660](camera--ov3660.md) — 3MP CMOS image sensor with on-chip JPEG (robocar; XIAO ESP32-S3 Sense)
- [SSD1306](display--ssd1306.md) — 128x64 OLED controller; also the 0.91-inch 128x32 module on robocar
- [PCA9685](driver--pca9685.md) — 16-ch 12-bit PWM/servo driver
- [TB6612FNG](driver--tb6612fng.md) — Dual DC motor driver
- [SG90](actuator--sg90.md) — 9g micro servo
- [MG90S](actuator--mg90s.md) — Micro metal gear servo, digital (candidate SG90 replacement)
- [MG90D](actuator--mg90d.md) — Micro metal gear servo, digital, double ball bearing (candidate SG90 replacement)
- [MG996R](actuator--mg996r.md) — Standard-size metal gear servo (evaluated, not used; not SG90-compatible)
- [LM2596](power--lm2596.md) — DC-DC step-down (buck) converter
- [XL6009](power--xl6009.md) — DC-DC boost converter
- [SX1276](radio--sx1276.md) — LoRa transceiver
- [WS2812B](led--ws2812b.md) — Addressable RGB LED
- [MCP23017](io--mcp23017.md) — 16-bit I2C GPIO expander (robocar, optional; CJMCU-2317 breakout)
- [RCWL-1601](sensor--rcwl-1601.md) — Ultrasonic rangefinder, 3–5.5 V, HC-SR04 compatible (robocar obstacle reflex)
- [MSM261D](sensor--msm261d.md) — PDM MEMS microphone (robocar; XIAO ESP32-S3 Sense expansion board)
- [MPU-6050](sensor--mpu6050.md) — 6-axis IMU, obsolete (balancebot and glowbug; GY-521 breakout)
- [VL53L4CD](sensor--vl53l4cd.md) — Short-range ToF, 1–1300 mm, 18° (candidate robocar cliff sensor; Adafruit 5396)
- [VL53L8CX](sensor--vl53l8cx.md) — 8×8 multizone ToF, 45°×45° (candidate; Pololu 3419)
- [VL53L9](sensor--vl53l9.md) — 54×42 direct-ToF 3D lidar, CSI-2/I3C (evaluated; not practical on ESP32-S3)
