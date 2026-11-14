# Waveshare ESP32-S3-Zero

**Role in project:** it-troubleshooter, xbox-switch-bridge, switch-usb-proxy,
facedancer-espdancer-fw

**Board page:** [waveshare-esp32-s3-zero.md](../boards/waveshare-esp32-s3-zero.md),
with the pad order, every GPIO's wiring and the sources

## Key Specs

| Parameter | Value |
|-----------|-------|
| MCU | ESP32-S3FH4R2 (dual-core LX7, 240 MHz) |
| Flash | 4 MB |
| PSRAM | 2 MB |
| WiFi | 802.11 b/g/n |
| Bluetooth | BLE 5.0 |
| USB | Type-C (native USB, no bridge chip) |
| LED | WS2812 RGB on GPIO21 |
| Antenna | Onboard ceramic |
| Form factor | Castellated (stamp hole) |

## Pin Notes

- GPIO33–GPIO37 are **not wired to a pad**. Waveshare's wiki calls them
  reserved for Octal PSRAM; the ESP32-S3FH4R2's PSRAM is quad SPI and does
  not use them (see the board page)
- UART0: TX=GPIO43, RX=GPIO44

## Datasheets & References

- **Wiki:** <https://www.waveshare.com/wiki/ESP32-S3-Zero>
- **Schematic PDF:** <https://files.waveshare.com/wiki/ESP32-S3-Zero/ESP32-S3-Zero-Sch.pdf>
- **Product page:** <https://www.waveshare.com/esp32-s3-zero.htm>
- **Documentation platform:** <https://docs.waveshare.com/ESP32-S3-Zero>
