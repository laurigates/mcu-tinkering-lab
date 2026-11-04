# CLAUDE.md - robocar-unified

Project-specific guidance for Claude Code. See repo-root `CLAUDE.md` for monorepo-wide conventions.

## What this project is

Single-board consolidation of the dual-ESP32 robocar onto a **XIAO ESP32-S3 Sense**. Replaces the separate `robocar-main` (Heltec) + `robocar-camera` (ESP32-CAM) boards — camera, AI, motors, peripherals, WiFi, MQTT, and OTA all run on one module.

## Architecture

Implements a hierarchical AI controller pattern: a **slow planner** (Core 1, every `PLANNER_LOOP_PERIOD_MS` — 15 s by default, bounded by the Gemini free-tier quota) that emits structured goals, and a **fast reactive executor** (Core 0, ~30 Hz) that drives the robot toward those goals. See [ADR-016](../../../docs/decisions/ADR-016-hierarchical-ai-controller.md) for the canonical design.

Core affinity is load-bearing — do not change without understanding the trade-offs:

- **Core 0** (motor-critical, timing-sensitive, ~30 Hz):
  - `reactive_controller` — reads `goal_state`, implements visual servo and heading hold, owns all motor PWM output and the pan/tilt head
  - `motor_task` — low-level PWM driver for motors and servos
  - `peripheral_task` — I2C devices (OLED, LEDs), buzzer
  - `command_task` — serial console command dispatch
  - `ultrasonic` driver — front and rear RCWL-1601 over I2C, polled by the
    executor; a reading completes every ~165 ms (`ULTRASONIC_CONVERSION_MS` plus
    a tick to read and a tick to re-trigger). The front one drives the reflex,
    which trips on a single reading below `STOP_THRESHOLD_CM` and releases on the
    running mean, so the worst case from an obstacle appearing to the brake is
    `(2·ceil(ULTRASONIC_CONVERSION_MS / 33) + 1) · 33` ms = 297 ms
    (`REACTIVE_REFLEX_WORST_CASE_MS`, pinned by `test_reflex_latency.c`). That is
    still above the GPIO driver's ~100 ms until the conversion wait is measured
    on the bench and shortened (issue #791)

- **Core 1** (bursty, I/O-bound):
  - `planner_task` — captures frames on a schedule, calls Gemini Robotics-ER, writes `goal_state`
  - `camera_task` — OV3660 frame capture (DMA pinned to Core 1 via `CONFIG_CAMERA_CORE1=y`)
  - `network_task` — WiFi, MQTT, credentials
  - OTA manager

Camera DMA is pinned to Core 1 so motor PWM jitter on Core 0 isn't degraded by frame captures.

Tasks communicate via FreeRTOS queues and the shared `goal_state` struct (mutex-protected).

## I2C topology

All I2C devices sit behind a **TCA9548A multiplexer** on GPIO5/6. Always select the channel through `i2c_bus.c` first: ch0 PCA9685 (motors, servos, LEDs), ch1 SSD1306 OLED, ch2 MCP23017 (optional; GPA7/GPB7 output-only), ch3/ch4 RCWL-1601 front/rear (0x57), ch5–7 reserved. Every peripheral is non-fatal, and the self-report names any that did not initialise. Details: [docs/firmware/i2c-and-motors.md](docs/firmware/i2c-and-motors.md).

## Subsystem docs

Each subsystem's design notes, measurements and the reasoning behind the rules below are in `docs/firmware/`. Read the matching doc before changing that subsystem.

| Doc | Covers |
|---|---|
| [i2c-and-motors.md](docs/firmware/i2c-and-motors.md) | Mux channel map, non-fatal peripherals and the `i2c_peripherals=` fact, PCA9685 channel layout, coast vs brake, bus clock and PWM phase, the managed I2C drivers and the `pca9685` version floor |
| [console-and-mqtt.md](docs/firmware/console-and-mqtt.md) | Serial console command table, MQTT remote commands and read-only mode, `servo exercise`, the boot wheel exercise |
| [head-aiming.md](docs/firmware/head-aiming.md) | How the executor satisfies `track` with the head, the wheels or both; head leases |
| [activity-trace.md](docs/firmware/activity-trace.md) | LED colour meanings, `trace` counters, the I2C bus counter |
| [planner.md](docs/firmware/planner.md) | Planner tools, dormancy ladder (`plan_activity`), spend fuse (`plan_budget`), reading the planner log |
| [camera-and-log.md](docs/firmware/camera-and-log.md) | Frame stats and `snap` dumps, the UART0 log mirror on J15, why the sensor is an OV3660 |
| [voice.md](docs/firmware/voice.md) | TTS streaming and preroll, personas and dialogue pools, speech gates (scene, ambient audio, budget), voice FX, push-to-talk and VAD voice turns, voice budget, memory peak |
| [wifi-and-ota.md](docs/firmware/wifi-and-ota.md) | Improv Serial provisioning over USB-Serial-JTAG, OTA manifest polling, the build-SHA install decision |

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
- `CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y` — mbedTLS buffers go to PSRAM. On the internal allocator the planner, TTS stream and voice turn each hold a TLS connection with a 16 kB RX buffer, and overlapping handshakes ran internal RAM down to 259 bytes and failed with `-0x7F00` / `-0x2880` / `-0x0010` (`*_ALLOC_FAILED`). It presented as a robot that does not hear: the voice turn recorded fine and never reached Gemini. `gemini_http.c` logs internal free/largest/min-ever on every failed request so a regression is visible at the failure point
- `CONFIG_CAMERA_JPEG_MODE_FRAME_SIZE=65536` (with `_AUTO=n`, `_CUSTOM=y`) — the AUTO default computes `width*height/5`, a hard **15360-byte** ceiling at QVGA. Past it the driver aborts accumulation, queues the truncated frame, finds no EOI marker and retries until a 4 s timeout — surfacing as `Camera capture failed` and a forced STOP, not as a bad image. Any detailed or bright scene exceeds it at quality 15, and raising the AGC gain ceiling makes it worse because noise inflates JPEG size
- mDNS (`robocar-unified.local`) needs **no** Kconfig switch — it comes from the `espressif/mdns` managed component. There is no `CONFIG_MDNS_ENABLED` symbol; a line setting one is reported as an unknown symbol and silently ignored

## Don't

One line per rule. Each group's doc has the full reasoning and the tests that hold it.

**Motors, I2C, PCA9685** ([why](docs/firmware/i2c-and-motors.md#dont))

- Don't call `motor_controller.c` from anywhere but `reactive_controller.c`. Console and manual movement go through `reactive_controller_manual()`, a lease applied after the obstacle reflex.
- Don't add direct GPIO motor control or bypass the TCA9548A; everything goes through the PCA9685 behind the mux.
- Don't drop `set_motors()`'s idle re-write suppression, or its `MOTOR_REFRESH_INTERVAL_MS` expiry.
- Don't put `ESP_ERROR_CHECK`/`ESP_RETURN_ON_ERROR` back into the hardware phase; `init_hardware()` returns `void` (#500).
- Don't drop a peripheral's `*_is_initialized()` accessor, or let `self_report_collect()` read a cached boot result.
- Don't unify `motor_stop()` (coast, low/low) with `motor_brake()` (brake, high/high); only the reflex brakes.
- Don't let a driver's `init_desc()` pick the bus clock; call `pin_bus_clock()` after every `init_desc()`.
- Don't change the PCA9685 frequency except through `servo_set_pwm_frequency()`.
- Don't give a full-on/full-off channel a phase offset in `pca9685_phase.c`; those are the motor direction pins.
- Don't patch anything in `managed_components/`, and don't lower the `esp-idf-lib/pca9685` floor below `'^1.0.8'`.
- Don't reorder the motor channel `#define`s; they follow the TB6612FNG header and describe soldered wires.

**Console and MQTT** ([why](docs/firmware/console-and-mqtt.md#dont))

- Don't give `mqtt_command.c` its own idea of what a command does; it classifies a line and calls `dispatch_movement()` or `execute_console_line()`.
- Don't make the MQTT read-only allow-list match prefixes, or add a command to it without reading its handler in `main.c` end to end (#626).
- Don't make `mqtt` reachable over MQTT: keep it out of `execute_console_line()` and out of `mqtt_command.c`'s prefixes.
- Don't skip `mqtt_command_extract_line()`'s fragmentation check.

**Planner** ([why](docs/firmware/planner.md#dont))

- Don't make the planner request unconditional again, or turn the 15 s → 300 s ladder into an on/off switch (ADR-022).
- Don't feed `ambient_audio_novel()` to the dormancy gate; it uses `ambient_audio_event()`.
- Don't point `plan_activity`'s scene reference at `scene_change`'s, or merge the `scene:`/`still:` log fields.
- Don't charge zero tokens for an unreadable `usageMetadata`; charge `PLAN_BUDGET_ASSUMED_TOKENS`.
- Don't give the planner or voice-turn budget fuse a rolling window or an auto-reset.
- Don't add goal sources outside `planner_task.c`.
- Don't resurrect `claude_*`/`ollama_*` sources; a second backend needs a designed abstraction.

**Pan/tilt head** ([why](docs/firmware/head-aiming.md#dont))

- Don't write the servos from anywhere but `reactive_controller.c` and the console lease path (`reactive_controller_head_hold()`, `reactive_controller_head_external()`).
- Don't add a head-aiming planner tool; `track` is the only aiming primitive (ADR-025).
- Don't recompute a track bearing from its box every tick; latch it per `goal_state_read_seq()` write.
- Don't drop `goal_t.head_pan_deg`/`head_tilt_deg`, or fill them anywhere but at frame capture in `planner_task`.

**Voice** ([why](docs/firmware/voice.md#dont))

- Don't fold speech into `goal_t`; it is a queue (`speech_queue.h`).
- Don't ration hands-free voice turns with `speech_budget`; `voice_budget` counts requests (#623).
- Don't name a phrase, opener or filler word in a persona's `text_brief`; use the `openers`/`shapes` pools.
- Don't add a band-limit, ring modulator, bitcrusher or drone to `voice_fx_core.c`, or drop the comb's `(1-g)` input trim.
- Don't name an era in `tts_style`; name the audible traits.
- Don't fix repetition with prompt wording; withhold `speak` and re-check against the recall ring.
- Don't compare scene fingerprints in absolute luma or threshold absolute mic level; both are relative (frame mean, adaptive floor).
- Don't allocate the PDM RX channel in the same `i2s_new_channel()` call as the TX channel.
- Don't open `speak` on audio evidence while the prompt still says "only if this frame shows something"; name the evidence.
- Don't let an audio clause assert or invite a guess at what made the sound (#618), or feed `speech_evidence_clause()` a bare `novel()`; use `speech_sense_from_gate()` with `*_compared()` (#631).
- Don't compare the instantaneous audio frame instead of the latched peak, or let a failed mic read reach the gate.
- Don't call `mic_pdm_flush()` at the start of a VAD turn, or offer a frame to the pre-roll after releasing the mic lock (#616).
- Don't send the clip as `audio/pcm`; it must be `audio/wav`.
- Don't hold PCM, WAV and base64 at once, or build the request with `cJSON_AddStringToObject`/`cJSON_PrintUnformatted`.
- Don't run voice turns on the planner's Robotics-ER model.
- Don't screen voice-turn replies, `voice say` or the self-report through `dialogue_style_is_repetitive()`, or call `scene_change_mark_spoken()` after a voice turn.
- Don't log a Gemini error body at DEBUG.
- Don't record while the amp sounds or inside `AMBIENT_PLAYBACK_HANGOVER_MS` after it; start-cue frames go through `ambient_gate_accepts()`.
- Don't point the scene reference at the previous frame, or let an undecodable frame update scene state; keep `!s_current.valid` ahead of the first-impression branch (#690).
- Don't grow `GEMINI_SYSTEM_PROMPT_MAX`, a persona brief or `DIALOGUE_RECALL_*` without checking `PLANNER_TASK_STACK_SIZE`.
- Don't move delivery tags into a `dialogue_pool_t`, or drop the `speech_tags.h` allow-list filter.
- Don't switch TTS back to `:generateContent`, resample to 16 kHz, or buffer the response whole.
- Don't start playback before the `AUDIO_PREROLL_BYTES` gate, or feed `audio_player_write()` unbatched quartets.
- Don't tally `s_written_total` from the caller's `bytes`; count each successful `ring_send()`.

**Camera and log mirror** ([why](docs/firmware/camera-and-log.md#dont))

- Don't set `fb_count = 1` on the camera.
- Don't re-add `set_aec_value()`/`set_agc_gain()` beside auto exposure/gain, or call `set_gainceiling()` with OV2640 values; this sensor is an OV3660.
- Don't read sensor registers without branching on the PID, or flip `set_aec2()` without an A/B against measured luma.
- Don't change camera core pinning without re-verifying motor PWM jitter.
- Don't call `ESP_LOG` inside `log_mirror.c`'s hook or drain task, or make the hook wait on the UART.
- Don't move the console to UART0.

**WiFi, OTA, repo hygiene** ([why](docs/firmware/wifi-and-ota.md#dont))

- Don't commit `main/credentials.h`, and don't hand-edit `version.txt`.
- Don't let a build SHA start an OTA install (only a strictly newer version installs), and don't drop `-c safe.directory=*` from the `CMakeLists.txt` git lookup.
- Don't call `improv_wifi_send_*()` without a writer set, or install the usb_serial_jtag driver for Improv replies.
