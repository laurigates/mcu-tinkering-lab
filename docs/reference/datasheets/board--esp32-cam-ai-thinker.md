# ESP32-CAM (AI Thinker)

**Role in project:** Camera module for robocar vision system (OV2640 capture, AI inference, MQTT)

## Key Specs

| Parameter | Value |
|-----------|-------|
| MCU | ESP32-S (dual-core LX6, 240 MHz) |
| Flash | 4 MB |
| PSRAM | 4 MB (IPUS) |
| Camera | OV2640 (2MP) |
| Storage | MicroSD slot |
| WiFi | 802.11 b/g/n |
| Bluetooth | BT 4.2 + BLE |
| USB | None (requires external FTDI for programming) |

## Pin Assignments (project-specific)

- **UART to main controller:** TX=GPIO14, RX=GPIO15 (chosen to avoid PSRAM conflicts)
- **Camera flash LED:** GPIO4
- **Red status LED:** GPIO33 (active low)
- **Camera power down:** GPIO32

## Safe GPIOs for General Use

GPIO2, GPIO16, GPIO17 (most flexible, no conflicts with camera/PSRAM)

## Datasheets & References

- **Pinout guide:** <https://randomnerdtutorials.com/esp32-cam-ai-thinker-pinout/>
- **Datasheet & specs:** <https://components101.com/modules/esp32-cam-camera-module>
- **Pin notes (detailed):** <https://github.com/raphaelbs/esp32-cam-ai-thinker/blob/master/docs/esp32cam-pin-notes.md>
- **Pinout reference:** <https://lastminuteengineers.com/esp32-cam-pinout-reference/>
