# Presence Detector

ESPHome firmware for a 24 GHz mmWave human-presence sensor built on a
Seeed Studio XIAO ESP32-C6 + Hi-Link LD2410. Reports presence, moving /
still target distance, per-gate energy levels, and acts as a Bluetooth
proxy for Home Assistant.

## Hardware

- **MCU:** [Seeed Studio XIAO ESP32-C6](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/) (4 MB flash)
- **Sensor:** Hi-Link [LD2410](https://esphome.io/components/sensor/ld2410.html) 24 GHz mmWave radar
- **Reference build:** [24 GHz mmWave for XIAO](https://wiki.seeedstudio.com/mmwave_for_xiao/)

## Wiring

| Signal       | XIAO pin |
|--------------|----------|
| LD2410 TX    | GPIO02   |
| LD2410 RX    | GPIO21   |
| UART baud    | 256000 8N1 |

## Quick start

```bash
just init           # install ESPHome + create secrets.yaml from template
$EDITOR secrets.yaml
just upload         # first USB flash
just wireless       # subsequent OTA updates
just logs           # tail device logs (auto-detect USB, fall back to OTA)
```

## Background reading

- [LD2410 component docs (ESPHome)](https://esphome.io/components/sensor/ld2410.html)
- [ESP32 platform docs (esphome-docs/components/esp32.rst)](https://github.com/esphome/esphome-docs/blob/current/components/esp32.rst)
- [DIY mmWave Presence Sensor walkthrough (digiblurDIY)](https://digiblur.com/2023/05/24/esphome-mmwave-presence-how-to-guide/)
- [ESP32-C6 ESPHome support tracker](https://github.com/esphome/feature-requests/issues/2176)
