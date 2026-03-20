# Heltec WiFi LoRa 32 V1

**Role in project:** Main controller for the robocar (motor control, OLED, buzzer, PCA9685)

## Key Specs

| Parameter | Value |
|-----------|-------|
| MCU | ESP32 (dual-core LX6, 240 MHz) |
| Flash | 4 MB |
| LoRa | SX1276 (868/915 MHz) |
| Display | SSD1306 128x64 OLED (I2C) |
| WiFi | 802.11 b/g/n |
| Bluetooth | BT 4.2 + BLE |
| USB | Micro-USB (CP2102 bridge) |

## Pin Assignments (project-specific)

- **OLED:** SDA=GPIO4, SCL=GPIO15, RST=GPIO16
- **LoRa SPI:** SCK=GPIO5, MISO=GPIO19, MOSI=GPIO27, CS=GPIO18, RST=GPIO14, DIO0=GPIO26

## Datasheets & References

- **V1 Pinout PDF:** <https://resource.heltec.cn/download/WiFi_LoRa_32/WIFI_LoRa_32_V1.pdf>
- **Official docs hub:** <https://docs.heltec.org/en/node/esp32/wifi_lora_32/index.html>
- **Hardware revision log:** <https://docs.heltec.org/en/node/esp32/hardware_update_log.html>
