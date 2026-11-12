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
Edge, with no toolchain installed. Board-specific boot-mode notes are on the
page itself ([source](docs/flasher/index.html)).

The [robocar board story](https://laurigates.github.io/mcu-tinkering-lab/robocar/)
follows the robot car from a 2018 kit to a generated, factory-made carrier
board ([source](docs/site/robocar/index.html)).

## Build from source

ESP-IDF builds run in a container, so Docker and
[just](https://github.com/casey/just) are the only requirements.

```bash
git clone https://github.com/laurigates/mcu-tinkering-lab.git
cd mcu-tinkering-lab
just setup-all                   # Docker images, dev tools, pre-commit hooks
just list-projects               # every project module
just robocar::build              # build one
```

Build, test and CI details are in [CONTRIBUTING.md](CONTRIBUTING.md).

## Projects

| Domain | Projects |
|---|---|
| `robotics/` | [AI robot car](packages/robotics/robocar/README.md): single-board firmware (XIAO ESP32-S3 Sense), its [integrated carrier](packages/robotics/robocar/integrated/README.md), [bring-up self-test](packages/robotics/robocar/bringup/README.md), a [Pymunk simulation](packages/robotics/robocar/simulation/README.md) and a [behaviour twin](packages/robotics/robocar/twin/README.md). [Self-balancing robot](packages/robotics/balancebot/README.md) (XIAO RP2350, Pico SDK) |
| `thinkpack/` | ESP-NOW mesh of companion boxes: brainbox (LLM coordinator), boombox, chatterbox, finderbox, glowbug, mesh-demo |
| `camera-vision/` | MJPEG webserver, camera + I2S audio, LLM Telegram bot, Gemini object detection |
| `audio/` | BLE gamepad synth, kids' audio toy, melody detector, RFID audiobook player (ESPHome) |
| `input-gaming/` | Xbox → Switch bridge, Switch USB proxy, LEGO Boost with an Xbox controller |
| `networking/` | IT troubleshooter, WiFi AP test, WireGuard + Home Assistant (ESPHome) |
| `sensors/` | 24 GHz mmWave presence sensor (ESPHome) |
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

MIT ([LICENSE](LICENSE)), except the two vendored KiCad libraries under
`tools/kicad/lib/`, which keep their upstream licences:
[`kicad-stock/`](tools/kicad/lib/kicad-stock/) is a subset of the KiCad
libraries under CC-BY-SA 4.0 with KiCad's design exception
([LICENSE.md](tools/kicad/lib/kicad-stock/LICENSE.md)), and
[`seeed-xiao/`](tools/kicad/lib/seeed-xiao/) is Seeed Studio's, under their MIT
licence ([LICENSE](tools/kicad/lib/seeed-xiao/LICENSE)).
