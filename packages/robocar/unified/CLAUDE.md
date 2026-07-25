# CLAUDE.md - robocar-unified

Project-specific guidance for Claude Code. See repo-root `CLAUDE.md` for monorepo-wide conventions.

## What this project is

Single-board consolidation of the dual-ESP32 robocar onto a **XIAO ESP32-S3 Sense**. Replaces the separate `robocar-main` (Heltec) + `robocar-camera` (ESP32-CAM) boards — camera, AI, motors, peripherals, WiFi, MQTT, and OTA all run on one module.

## Architecture

Implements a hierarchical AI controller pattern: a **slow planner** (Core 1, every `PLANNER_LOOP_PERIOD_MS` — 15 s by default, bounded by the Gemini free-tier quota) that emits structured goals, and a **fast reactive executor** (Core 0, ~30 Hz) that drives the robot toward those goals. See [ADR-016](../../docs/decisions/ADR-016-hierarchical-ai-controller.md) for the canonical design.

Core affinity is load-bearing — do not change without understanding the trade-offs:

- **Core 0** (motor-critical, timing-sensitive, ~30 Hz):
  - `reactive_controller` — reads `goal_state`, implements visual servo and heading hold, owns all motor PWM output
  - `motor_task` — low-level PWM driver for motors and servos
  - `peripheral_task` — I2C devices (OLED, LEDs), buzzer
  - `command_task` — serial console command dispatch
  - `ultrasonic` driver — distance reflex at ~20 Hz sampling

- **Core 1** (bursty, I/O-bound):
  - `planner_task` — captures frames on a schedule, calls Gemini Robotics-ER, writes `goal_state`
  - `camera_task` — OV2640 frame capture (DMA pinned to Core 1 via `CONFIG_CAMERA_CORE1=y`)
  - `network_task` — WiFi, MQTT, credentials
  - OTA manager

Camera DMA is pinned to Core 1 so motor PWM jitter on Core 0 isn't degraded by frame captures.

Tasks communicate via FreeRTOS queues and the shared `goal_state` struct (mutex-protected).

## I2C topology

All I2C devices hang off a **TCA9548A multiplexer** on GPIO5/6. Don't talk to devices directly on the primary bus — always select the channel first via `i2c_bus.c`. Current channel map in `pin_config.h`:

- ch0: PCA9685 (motors, servos, LEDs)
- ch1: SSD1306 OLED
- ch2: MCP23017 GPIO expander (optional — 16 generic GPIOs via `gpio_expander.c`, no roles assigned yet; firmware boots fine without the board fitted)
- ch3-7: reserved

The MCP23017 (1953W breakout, address 0x20) is exercised from the serial console with `gpio`, `gpio mode <pin> in|up|out`, `gpio set <pin> 0|1`, `gpio get <pin>`.

## Serial console commands

| Command | Effect |
|---------|--------|
| `F` `B` `L` `R` `C` `W` `S` | Manual drive / rotate / stop — a ~1 s lease via `reactive_controller_manual()`, still subject to the obstacle reflex |
| `gpio …` | MCP23017 expander (see above) |
| `voice …` | Persona / TTS voice switching and auditioning; `voice vary` shows a drawn variation directive |
| `sound beep\|melody\|alert` | Buzzer |
| `servo pan\|tilt <deg>` | Pan/tilt servos |
| `led <r> <g> <b>` | Both RGB LEDs |

The `sound`/`servo`/`led` commands are the only producers for `peripheral_task`'s queue — without them the task and every `PERIPH_CMD_*` case are unreachable.

## PCA9685 channel layout

All motor direction, motor PWM, servo, and LED outputs go through the PCA9685 — the ESP32-S3 only drives STBY (GPIO1), the buzzer (GPIO2), and I2C. GPIO budget on XIAO headers is tight (11 pins). See the channel table in `main/pin_config.h` before reassigning.

Motor direction uses PCA9685 "full-on" (4096) / "full-off" (0) values on IN1/IN2 channels.

## AI planner

**Gemini Robotics-ER 1.6 only.** The planner calls Gemini to emit function-call goals:

- `drive(heading_deg, distance_cm, speed_pct)` — absolute heading + distance
- `track(box_2d, max_speed_pct)` — visual servo toward a bounding box
- `rotate(angle_deg)` — spin in place
- `stop()` — hold position

- `speak(text)` — say one short sentence aloud, emitted *in addition to* a movement call

The planner runs every `PLANNER_LOOP_PERIOD_MS` (15 s default, set by the Gemini free-tier quota rather than by control preference) on Core 1; the executor drives the goal at ~30 Hz on Core 0. No on-demand inference or blocking on responses — the planner is a background task that constantly updates `goal_state`, and the executor always has something to do.

## Voice (MAX98357A)

The robot speaks through a MAX98357A I2S amplifier on GPIO7/8/9. See [ADR-019](../../docs/decisions/ADR-019-robocar-voice-gemini-tts.md) for the design.

**Speech is a queue, not a goal.** `speak` is deliberately *not* a `goal_kind_t` — it travels via `speech_queue` alongside `goal_state`. Goals are last-write-wins with a TTL that collapses to STOP; applying those semantics to speech would truncate sentences mid-word and make talking and driving mutually exclusive. If you find yourself adding `GOAL_KIND_SPEAK`, read the header comment in `speech_queue.h` first.

The pipeline is two Gemini calls: Robotics-ER decides *what* to say (riding the existing planner call — no extra vision inference), then `gemini-3.1-flash-tts-preview` renders it. Audio is 24 kHz mono PCM, base64 inline, no WAV header — decoded **incrementally** into a PSRAM ring by `base64_stream_feed()` while the player task drains it into I2S, so playback starts before the download finishes. A few seconds of audio is hundreds of kB; it can never be buffered whole.

**What the robot says varies per utterance; what it *sounds like* does not.** The persona (`voice_persona.c`) fixes the language, voice and register — the parts that must hold for every line. Anything phrase-shaped lives in that persona's `dialogue_pool_t` pools instead, and `dialogue_style.c` draws one opener and one sentence-shape per generated line, appends them to the prompt, and adds the openings of the last few spoken lines as a "do not begin with these" list. This is prompt-side only: no extra API call, no extra latency, a few hundred bytes of rodata.

The trap it exists to prevent: a phrase named in `text_brief` gets used *every single time*. Naming the period construction "Asianlaita on oikeastaan niin, että ..." there made it the opening of practically every spoken line. Put such a phrase in the `openers` pool, where it turns up on its share of lines and the pool can also say "no opener at all". `voice vary` on the console prints a freshly drawn directive so pools can be tuned without waiting out a 15 s planner period per sample.

Two hardware consequences worth knowing before touching this:

- **The microSD slot is gone.** GPIO7/8/9 are the Sense expansion board's SPI bus. No alternative pins exist — I2S needs a real peripheral, so it cannot move behind the PCA9685 or MCP23017.
- **The GPIO budget is fully allocated.** Further digital I/O goes through the MCP23017 on TCA9548A ch2.

Claude and Ollama backends have been removed from this project. If an alternative planner becomes necessary in the future, it should be designed as a clean abstraction, not a resurrection of deleted code. See ADR-016 for the rationale.

## WiFi provisioning

Uses **Improv WiFi Serial** provisioning by default — no credentials compiled in. `CMakeLists.txt` auto-generates a stub `credentials.h` at configure time if one doesn't exist, so CI and the web flasher don't need credentials. For local dev with hardcoded creds, copy `main/credentials.h.example` to `main/credentials.h` (gitignored).

It is the **Serial** variant of Improv (the `improv-wifi` component implements the Improv Serial spec over UART0), not BLE. `init_network()` starts it whenever the board has no WiFi connection — including when a stored SSID has gone away — and `command_task` feeds every console byte to the parser, which ignores anything that is not a well-formed Improv packet. Credentials reach NVS only after `wifi_connect()` succeeds with them.

Because the planner is non-fatal, a board with no credentials still boots fully: the executor holds STOP, and the console, provisioning and self-report stay reachable. Do not reintroduce an `ESP_ERROR_CHECK` around the AI init phase — that is exactly what made the documented no-credentials build boot-loop.

NVS stores runtime-provisioned credentials — don't wipe NVS unless you want to re-provision.

## OTA

`ota_manager.c` pulls releases from the `laurigates/mcu-tinkering-lab` GitHub repo. `version.txt` is the single source of truth for the running version (read at CMake-configure time into `PROJECT_VER`). Release-please manages version bumps — do not edit `version.txt` manually.

Partition layout is OTA-capable (see `partitions.csv`) with app rollback enabled.

## Build & flash

Containerized ESP-IDF v5.4 via `just robocar-unified::*`. XIAO uses native USB-Serial-JTAG — `just` auto-detects VID `0x303a`. If flashing fails, hold BOOT and tap RESET to enter download mode.

```bash
just robocar-unified::build
just robocar-unified::flash-monitor
```

Do not invoke `idf.py` directly on the host — there's no local ESP-IDF install. Use `just robocar-unified::shell` for an interactive container.

## sdkconfig rules

See repo `.claude/rules/esp-idf-sdkconfig.md`. If you change `sdkconfig.defaults`, delete the generated `sdkconfig` and run `just robocar-unified::clean` before rebuilding — ESP-IDF preserves existing `sdkconfig` values and silently ignores new defaults otherwise.

Key settings that matter:
- `CONFIG_SPIRAM_MODE_OCT=y` — XIAO ESP32-S3 Sense has **octal** PSRAM (not quad); wrong mode = boot loop
- `CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192` — bumped from default 3584 for WiFi + BLE + camera init
- `CONFIG_ESP_BROWNOUT_DET=n` — disabled; motor inrush was tripping it
- mDNS (`robocar-unified.local`) needs **no** Kconfig switch — it comes from the `espressif/mdns` managed component. There is no `CONFIG_MDNS_ENABLED` symbol; a line setting one is reported as an unknown symbol and silently ignored

## Don't

- Don't call `motor_controller.c` directly from anywhere except `reactive_controller.c` — the executor owns motor output. Console/manual movement goes through `reactive_controller_manual()`, which takes a short lease the executor applies *after* the obstacle reflex; a second task writing the PCA9685 directly both fought the 30 Hz executor and bypassed the reflex
- Don't add goal sources outside `planner_task.c` — structured goals keep the two layers decoupled. If a new goal source is needed, it should write `goal_state` the same way the planner does
- Don't fold speech into `goal_t` — see the Voice section above and `speech_queue.h`
- Don't put a specific phrase, opener or filler word in a persona's `text_brief` — everything named there is said every time. Phrase-shaped flavour goes in the `openers`/`shapes` pools; see `dialogue_style.h`
- Don't "normalise" the audio path to 16 kHz to match the ThinkPack projects — 24 kHz is Gemini TTS's native rate, and matching it avoids a resampling stage entirely
- Don't buffer the TTS response whole — it's hundreds of kB and the response buffer is 16 kB. The streaming decoder exists for this reason
- Don't add direct GPIO motor control — everything goes through PCA9685 via `motor_controller.c`
- Don't bypass the TCA9548A — devices on different channels can share addresses (e.g. PCA9685 and OLED would conflict without it)
- Don't commit `main/credentials.h` — it's gitignored; use the `.example` as template
- Don't hand-edit `version.txt` — release-please owns it
- Don't change camera core pinning without re-verifying motor PWM jitter
- Don't resurrect `claude_*` or `ollama_*` source files. If a second backend becomes necessary, design an abstraction; old code paths are not a foundation
