# ESP32-CAM (AI Thinker)

**Role in project:** the board of cam-webserver, cam-i2s-audio and llm-telegram.
The robocar design that used it as a camera module has been removed.

**Board page:** [esp32-cam-ai-thinker.md](../boards/esp32-cam-ai-thinker.md),
with the header pinout, the camera, microSD and PSRAM pins, and the sources

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

## On-board pins

- **Camera flash LED:** GPIO4 (also microSD DATA1)
- **Red status LED:** GPIO33 (active low)
- **Camera power switch:** GPIO32, low powers the camera. It drives a P-MOS
  ahead of the camera regulators, not the sensor's PWDN pin

## Header GPIOs

No header GPIO is free of another duty. GPIO2, GPIO4, GPIO12, GPIO13, GPIO14
and GPIO15 are the microSD bus, and GPIO16 is the PSRAM's chip select. GPIO17
is the PSRAM clock and is not on a header. The board page's tables give each
pin with its source.

## Datasheets & References

- **Pinout guide:** <https://randomnerdtutorials.com/esp32-cam-ai-thinker-pinout/>
- **Datasheet & specs:** <https://components101.com/modules/esp32-cam-camera-module>
- **Pin notes (detailed):** <https://github.com/raphaelbs/esp32-cam-ai-thinker/blob/master/docs/esp32cam-pin-notes.md>
- **Pinout reference:** <https://lastminuteengineers.com/esp32-cam-pinout-reference/>
