# robocar-unified

Single-board robocar firmware consolidating the dual-ESP32 design (`robocar-main` + `robocar-camera`) onto a **XIAO ESP32-S3 Sense**. Camera capture, AI inference, motor control, and peripherals all run on one module with core-affinity isolation.

## Hardware

Everything runs on one **XIAO ESP32-S3 Sense** (ESP32-S3 + 8 MB PSRAM, with the
Sense expansion board's OV3660 camera and PDM microphone). All I2C peripherals
hang off a TCA9548A multiplexer: a PCA9685 drives the motors, pan/tilt servos
and status LEDs, with an SSD1306 OLED and an optional MCP23017 expander on their
own channels. Off the headers directly: a TB6612FNG motor driver's enable line,
a piezo buzzer, a 3.3 V ultrasonic rangefinder, and I2S to a MAX98357A amplifier
for the robot's voice.

Pin assignments, channel maps, power and the wiring schematic are in
**[WIRING.md](WIRING.md)**; `main/pin_config.h` is authoritative for all of them.

## Printable guides

| Document | Scope |
|---|---|
| [`docs/build-guide.typ`](docs/build-guide.typ) → [PDF](docs/build-guide.pdf) | Whole build: bill of materials, wiring reference, power, assembly, flashing, WiFi provisioning, functional checkout, troubleshooting |
| [`docs/wiring-card-motors.typ`](docs/wiring-card-motors.typ) → [PDF](docs/wiring-card-motors.pdf) | One bench card for the PCA9685 → TB6612FNG motor wiring, with both board layouts drawn from the vendors' Eagle files |

Regenerate every PDF after editing any of them — channel numbers come from
`main/pin_config.h` via `docs/auto/pin_defs.typ`, so a pin moved in the header
changes what these print:

```bash
just robocar-unified::build-guide
```

The recipe pins the Typst version and the two determinism flags that
`.github/workflows/build-guide-check.yml` recompiles with, so a hand-run
`typst compile` with different flags will read as drift in CI.

## Architecture

Implements a hierarchical AI controller: a **slow planner** (Core 1) that calls Gemini Robotics-ER to emit structured goals, and a **fast reactive executor** (~30 Hz, Core 0) that drives the robot smoothly toward those goals. See [ADR-016](../../docs/decisions/ADR-016-hierarchical-ai-controller.md) for the detailed design.

- **Core 0**: reactive executor (visual servo, heading hold, motor PWM), motor control, peripheral I/O, command console, obstacle reflex via ultrasonic sensor
- **Core 1**: planner task (Gemini calls), camera capture, audio playback and voice turns, WiFi / MQTT / OTA

**The planner does not poll on a fixed schedule.** The board boots *dormant* and
makes a request only when something happened — the view changed, the room made a
noise, the rangefinder moved, the robot is driving, or somebody typed at the
console. With no evidence the interval grows `PLANNER_LOOP_PERIOD_MS` (15 s) →
30 → 60 → 120 → 300 s and then stops. A per-boot ceiling on requests and tokens
sits behind that as a hard fuse. See [ADR-022](../../docs/decisions/ADR-022-planner-dormancy-and-spend-ceiling.md),
and the `plan` console command to inspect or retune it live.

## Build & flash

This project builds in a container — no local ESP-IDF install required.

```bash
# From repo root
just robocar-unified::build
PORT=/dev/cu.usbmodem* just robocar-unified::flash
just robocar-unified::monitor
# or: just robocar-unified::flash-monitor
```

The XIAO ESP32-S3 uses native USB-Serial-JTAG — no external USB-serial adapter needed. Port auto-detection looks for VID `0x303a`.

## WiFi provisioning

No credentials are compiled in. When the firmware boots without a WiFi connection it starts **Improv WiFi Serial** on the USB console — open the board in [ESP Web Tools](https://esphome.github.io/esp-web-tools/) (or any Improv Serial provisioner) in Chrome and send the WiFi credentials. They are written to NVS only after they are proven to connect, so a typo cannot overwrite a working network. For local builds with hardcoded credentials, copy `main/credentials.h.example` to `main/credentials.h`.

Note this is Improv **Serial**, over the same USB port used for flashing — not the BLE variant, so nothing needs to be paired.

After connection, the device is reachable as `robocar-unified.local` via mDNS.

## OTA updates

OTA is enabled (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`) and configured to pull releases from `laurigates/mcu-tinkering-lab` GitHub. `version.txt` is the single source of truth (managed by release-please).

## Key files

- `main/main.c` — FreeRTOS task setup and core affinity
- `main/pin_config.h` — all GPIO / PCA9685 channel assignments
- `main/planner_task.c/.h` — Gemini Robotics-ER calls, goal state writes (cadence set by `plan_activity`)
- `main/plan_activity.c/.h` — the evidence ladder deciding whether a request is made at all (ADR-022)
- `main/plan_budget.c/.h` — per-boot request/token fuse at the `gemini_backend_plan()` choke point
- `main/reactive_controller.c/.h` — visual servo, heading hold, motor output (~30 Hz)
- `main/ultrasonic.c/.h` — distance measurement and reflex (~20 Hz sampling)
- `main/goal_state.c/.h` — shared planner-executor state (mutex-protected)
- `main/motor_controller.c/.h` — low-level motor PWM (called only by executor)
- `main/audio_player.c/.h`, `main/speech_queue.c/.h` — TTS playback ring and the speech path (ADR-019)
- `main/mic_pdm.c/.h`, `main/ambient_audio.c/.h` — onboard microphone and the ambient speech gate (ADR-020)
- `sdkconfig.defaults` — PSRAM, camera core pinning, OTA, mDNS config
- `partitions.csv` — OTA-capable partition table for 8MB flash
