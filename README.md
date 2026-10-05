# MCU Tinkering Lab

A monorepo of microcontroller projects, mostly ESP32. The largest is a robot car
that drives on camera input and an LLM's plan. Others cover audio toys, game
controller bridges, an ESP-NOW mesh of companion boxes, and a self-balancing
robot on the RP2350.

[![Build](https://github.com/laurigates/mcu-tinkering-lab/actions/workflows/build.yml/badge.svg)](https://github.com/laurigates/mcu-tinkering-lab/actions/workflows/build.yml)
[![Tests](https://github.com/laurigates/mcu-tinkering-lab/actions/workflows/test.yml/badge.svg)](https://github.com/laurigates/mcu-tinkering-lab/actions/workflows/test.yml)

## Flash from the browser

Released firmware can be flashed with the
[web flasher](https://laurigates.github.io/mcu-tinkering-lab/) in Chrome or
Edge, with no toolchain installed. See the
[web flasher guide](packages/robocar/docs/docs/WEB_FLASHER.md).

## Build from source

ESP-IDF builds run in a container, so Docker and
[just](https://github.com/casey/just) are the only requirements.

```bash
git clone https://github.com/laurigates/mcu-tinkering-lab.git
cd mcu-tinkering-lab
just setup-all                   # Docker images, dev tools, pre-commit hooks
just list-projects               # every project module
just robocar-unified::build      # build one
```

Build, test and CI details are in [CONTRIBUTING.md](CONTRIBUTING.md).

## Projects

| Domain | Projects |
|---|---|
| [`robocar/`](packages/robocar/docs/README.md) | AI robot car: [single-board firmware](packages/robocar/unified/README.md) (XIAO ESP32-S3 Sense), its bring-up self-test, the earlier dual-board design (Heltec + ESP32-CAM), and a Pymunk simulation |
| `thinkpack/` | ESP-NOW mesh of companion boxes: brainbox (LLM coordinator), boombox, chatterbox, finderbox, glowbug, mesh-demo |
| `camera-vision/` | MJPEG webserver, camera + I2S audio, LLM Telegram bot, Gemini object detection |
| `audio/` | BLE gamepad synth, kids' audio toy, melody detector, RFID audiobook player (ESPHome) |
| `input-gaming/` | Xbox → Switch bridge, Switch USB proxy, LEGO Boost with an Xbox controller |
| `networking/` | IT troubleshooter, WiFi AP test, WireGuard + Home Assistant (ESPHome) |
| `sensors/` | 24 GHz mmWave presence sensor (ESPHome) |
| `robotics/` | Self-balancing robot (XIAO RP2350, Pico SDK) |
| `usb-tools/` | ESP32-S3 raw USB relay for facedancer |
| `games/` | NFC scavenger hunt |
| `components/` | Shared ESP-IDF components: Improv WiFi provisioning, GitHub Releases OTA, ThinkPack libraries |

Most projects have a README in their directory.

## Documentation

- [docs/](docs/README.md): architecture decisions, requirements, board
  references, schematics
- [CONTRIBUTING.md](CONTRIBUTING.md): development setup, build commands,
  testing, CI, adding a project

## License

MIT. See [LICENSE](LICENSE).
