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

All I2C devices sit behind a **TCA9548A multiplexer** on GPIO5/6. Always select the channel through `i2c_bus.c` first: ch0 PCA9685 (motors, servos, LEDs), ch1 SSD1306 OLED, ch2 MCP23017 (optional; GPA7/GPB7 output-only; GPA0/GPA1 the front/rear floor sensors), ch3/ch4 RCWL-1601 front/rear (0x57), ch5–7 reserved. Every peripheral is non-fatal, and the self-report names any that did not initialise. Details: [docs/firmware/i2c-and-motors.md](docs/firmware/i2c-and-motors.md).

## Subsystem docs

Each subsystem's design notes, measurements and the reasoning behind the rules below are in `docs/firmware/`. Read the matching doc before changing that subsystem.

| Doc | Covers |
|---|---|
| [i2c-and-motors.md](docs/firmware/i2c-and-motors.md) | Mux channel map, non-fatal peripherals and the `i2c_peripherals=` fact, PCA9685 channel layout, coast vs brake, bus clock and PWM phase, the managed I2C drivers and the `pca9685` version floor |
| [console-and-mqtt.md](docs/firmware/console-and-mqtt.md) | Serial console command table (including `mqtt auth` and `gemini auth`), MQTT remote commands and read-only mode, `servo exercise`, the boot wheel exercise |
| [error-reporting.md](docs/firmware/error-reporting.md) | GlitchTip error events on `robocar/events`: setup, the development loop, reset events and the RTC breadcrumb ring, core-dump crash events, planner notes, `report` |
| [head-aiming.md](docs/firmware/head-aiming.md) | How the executor satisfies `track` with the head, the wheels or both; head leases |
| [activity-trace.md](docs/firmware/activity-trace.md) | LED colour meanings, `trace` counters, the I2C bus counter |
| [planner.md](docs/firmware/planner.md) | Planner tools, dormancy ladder (`plan_activity`), spend fuse (`plan_budget`), planner notes to GlitchTip (`note_gate`), reading the planner log |
| [camera-and-log.md](docs/firmware/camera-and-log.md) | Frame stats and `snap` dumps, the UART0 log mirror on J15, why the sensor is an OV3660 |
| [voice.md](docs/firmware/voice.md) | TTS streaming and preroll, personas and dialogue pools, speech gates (scene, ambient audio, budget), voice FX, sound clips (`[horn]`, the `sound` tool, the reflex horn), push-to-talk and VAD voice turns, voice budget, memory peak |
| [cliff-sensors.md](docs/firmware/cliff-sensors.md) | Front and rear floor sensors on the MCP23017, the cliff reflex (brake, back off, refuse motion toward an edge), wiring, `cliff on\|off`, bench checks |
| [wifi-and-ota.md](docs/firmware/wifi-and-ota.md) | Improv Serial provisioning over USB-Serial-JTAG, the network supervisor (nothing waits on WiFi), where the Gemini key comes from, OTA manifest polling, the build-SHA install decision |

## Build & flash

Containerized ESP-IDF (the version the root `Dockerfile` pins) via `just robocar-unified::*`. XIAO uses native USB-Serial-JTAG — `just` auto-detects VID `0x303a`. If flashing fails, hold BOOT and tap RESET to enter download mode.

```bash
just robocar-unified::build
just robocar-unified::flash-monitor
```

`flash` builds first and refuses unless the image embeds the expected build SHA; `flash <commit|branch|version tag>` writes that version instead, built in `tmp/flash/<sha>` or downloaded from the release (`tools/flash-source.py`). When a bench result looks wrong, read the boot banner's `build <sha>` before debugging the code.

Do not invoke `idf.py` directly on the host — there's no local ESP-IDF install. Use `just robocar-unified::shell` for an interactive container.

## sdkconfig rules

See repo `.claude/rules/esp-idf-sdkconfig.md`. If you change `sdkconfig.defaults`, delete the generated `sdkconfig` and run `just robocar-unified::clean` before rebuilding — ESP-IDF preserves existing `sdkconfig` values and silently ignores new defaults otherwise.

Key settings that matter:

- `CONFIG_SPIRAM_MODE_OCT=y` — XIAO ESP32-S3 Sense has **octal** PSRAM (not quad); wrong mode = boot loop
- `CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192` — bumped from default 3584 for WiFi + BLE + camera init
- `CONFIG_ESP_BROWNOUT_DET=n` — disabled; motor inrush was tripping it. The chip still resets on a deep enough dip, and reports it: the boot banner reads `reset BROWNOUT` and the ROM prints `rst:0xf (BROWNOUT_RST)` on J15 (20 times on battery on 2026-11-07, each at a motor start; issue #991)
- `CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y` — mbedTLS buffers go to PSRAM. On the internal allocator the planner, TTS stream and voice turn each hold a TLS connection with a 16 kB RX buffer, and overlapping handshakes ran internal RAM down to 259 bytes and failed with `-0x7F00` / `-0x2880` / `-0x0010` (`*_ALLOC_FAILED`). It presented as a robot that does not hear: the voice turn recorded fine and never reached Gemini. `gemini_http.c` logs internal free/largest/min-ever on every failed request so a regression is visible at the failure point
- `CONFIG_CAMERA_JPEG_MODE_FRAME_SIZE=65536` (with `_AUTO=n`, `_CUSTOM=y`) — the AUTO default computes `width*height/5`, a hard **15360-byte** ceiling at QVGA. Past it the driver aborts accumulation, queues the truncated frame, finds no EOI marker and retries until a 4 s timeout — surfacing as `Camera capture failed` and a forced STOP, not as a bad image. Any detailed or bright scene exceeds it at quality 15, and raising the AGC gain ceiling makes it worse because noise inflates JPEG size
- `CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y` (ELF, SHA256) — a panic writes an ELF core dump to the 128 kB `coredump` partition, and the next boot sends its summary to GlitchTip (ADR-033). The partition table only changes on a USB or web-flasher flash, never over OTA. `CONFIG_ESP_TASK_WDT_PANIC` stays off: it would make a stuck task reboot the robot
- mDNS (`robocar-unified.local`) needs **no** Kconfig switch — it comes from the `espressif/mdns` managed component. There is no `CONFIG_MDNS_ENABLED` symbol; a line setting one is reported as an unknown symbol and silently ignored

## Rules

One line per rule. Each group's doc has the full reasoning and the tests that hold it, under its `## Don't` heading.

**Motors, I2C, PCA9685** ([why](docs/firmware/i2c-and-motors.md#dont))

- Motor output goes through `reactive_controller.c` alone. Console and manual movement use `reactive_controller_manual()`, a lease applied after the obstacle and cliff reflexes.
- Every motor, servo and LED output goes through the PCA9685, reached by selecting its TCA9548A channel in `i2c_bus.c`.
- `set_motors()` skips identical idle re-writes and re-sends after `MOTOR_REFRESH_INTERVAL_MS`; keep both halves.
- `init_hardware()` returns `void` and logs each peripheral failure, so the boot always reaches the console (#500).
- Each peripheral keeps its `*_is_initialized()` accessor, and `self_report_collect()` reads it live.
- `motor_stop()` coasts (low/low) and `motor_brake()` brakes (high/high). Keep them separate; only the obstacle and cliff reflexes brake, from `apply_wheels()`.
- The rangefinders hold only motion toward what they see: the front reflex brakes forward motion, the rear reading brakes a reverse, turns in place run unless both are blocked (`apply_wheels()`, #1026).
- Call `pin_bus_clock()` after every driver's `init_desc()`, including for devices added later.
- Change the PCA9685 frequency through `servo_set_pwm_frequency()`, which re-sends both servo pulses.
- Full-on and full-off channels (the motor direction pins) keep phase 0 in `pca9685_phase.c`.
- Driver behaviour we add lives in our own sources, such as `pca9685_phase.c`; `managed_components/` is refetched as-is. Keep the `esp-idf-lib/pca9685` floor at `'^1.0.8'`.
- The motor channel `#define`s follow the TB6612FNG header order and describe soldered wires; keep that order.

**Cliff reflex** ([why](docs/firmware/cliff-sensors.md#dont))

- The cliff state starts as "no floor" and a failed read counts as no floor (fail closed); arming is not an edge.
- `apply_wheels()` is the one wheel arbiter; `goal_motion()` mirrors `execute_goal()`'s branches and changes with it.

**Console and MQTT** ([why](docs/firmware/console-and-mqtt.md#dont))

- `mqtt_command.c` classifies a line and calls `dispatch_movement()` or `execute_console_line()`, the functions the serial console uses. New behaviour goes into those.
- The MQTT read-only allow-list holds exact whole lines. Add a command only after reading its handler in `main.c` end to end (#626).
- `mqtt` is serial-only: `command_task()` handles it, outside `execute_console_line()` and `mqtt_command.c`'s prefixes.
- Every MQTT payload passes through `mqtt_command_extract_line()`, fragmentation check included.

**Error reporting** ([why](docs/firmware/error-reporting.md#dont))

- `event_report_breadcrumb()` runs inside the log hook: no logging calls, no waiting. `event_report_boot_capture()` runs before the first ESP_LOG line in `app_main` (ADR-033).
- The core dump is erased only after the broker acknowledges its crash event, on `event_report`'s task, never the MQTT one; `EVENT_REPORT_CRASH_COMMAND` stays 0 in commits.

**Planner** ([why](docs/firmware/planner.md#dont))

- The planner requests only on evidence, backing off 15 s → 300 s before going dormant. Keep the gate and the ladder (ADR-022).
- The dormancy gate reads `ambient_audio_event()`.
- `plan_activity` keeps its own scene references (the frame last planned on), separate from `scene_change`'s (last spoken about), with separate `scene:` and `still:` log fields.
- Both gates keep one reference per dwell pose and neither compare nor adopt a frame taken off one (ADR-029 §2).
- A response whose `usageMetadata` cannot be read is charged `PLAN_BUDGET_ASSUMED_TOKENS`.
- The planner and voice-turn budget fuses stay tripped until `plan resume`/`voice resume` or a reboot.
- `planner_task.c` is the only goal source; a new source writes `goal_state` the same way.
- A second AI backend is a newly designed abstraction; the removed `claude_*`/`ollama_*` code stays removed.
- Per-cycle decision logic goes in `planner_cycle_core.c`, not `planner_task.c`, so the behaviour twin (`packages/robocar/twin`, ADR-030) runs it.
- The `note` tool is declared only when `note_gate` offers a trigger (`planner_cycle_note_offer()`); the gate's per-boot count stays spent until `plan resume` or a reboot (ADR-033).

**Pan/tilt head** ([why](docs/firmware/head-aiming.md#dont))

- Only `reactive_controller.c` and the console lease path (`reactive_controller_head_hold()`, `reactive_controller_head_external()`) write the servos.
- `track` is the planner's only aiming primitive; the executor chooses head, wheels or both (ADR-025).
- Latch a track goal's bearing once per `goal_state_read_seq()` write.
- `planner_task` fills `goal_t.head_pan_deg`/`head_tilt_deg` at frame capture; keep both fields.
- With no fresh `track` goal or lease, the head runs the idle gaze (or re-centres with `idle gaze off`), moving only between `scene_pose_t` dwell poses (ADR-029).
- A frame gets a dwell pose only while the executor reports `head_settled`.

**Voice** ([why](docs/firmware/voice.md#dont))

- Speech travels through `speech_queue`, separate from `goal_t`.
- Hands-free voice turns are rationed by `voice_budget`, which counts requests (#623).
- Phrase-shaped flavour (openers, fillers) goes in the persona's `openers`/`shapes` pools; `text_brief` describes the register.
- `voice_fx_core.c` is a full-bandwidth comb plus saturation, and the comb keeps its `(1-g)` input trim.
- `tts_style` names audible traits (enunciation, tempo, held geminates), not an era.
- To stop repetition, withhold the `speak` tool and re-check the reply against the recall ring.
- Scene fingerprints are relative to the frame mean, and mic level is measured against the adaptive floor.
- The PDM RX channel gets its own `i2s_new_channel()` call.
- When audio evidence opens `speak`, the prompt names that evidence.
- Audio clauses say only that the sound changed and its source is unknown (#618), built from `speech_sense_from_gate()` with each gate's `*_compared()` (#631).
- The audio gate compares the latched peak and receives only successful mic reads.
- A VAD turn reads on from the DMA without flushing, and the listener offers frames to the pre-roll while it holds the mic lock (#616).
- The voice clip is WAV-wrapped and sent as `audio/wav`.
- Each audio stage is freed once the next exists, and the request is built from string references with a presized `cJSON_PrintBuffered`.
- Voice turns run on a flash model; Robotics-ER belongs to the planner.
- Voice-turn replies, `voice say` and the self-report bypass `dialogue_style_is_repetitive()`, and a voice turn leaves the scene reference where it is.
- A voice turn consumes the audio latch whether it answers or ignores, and a planner line is dropped if a voice turn ran or anything was spoken during its request.
- Gemini error bodies are logged at ERROR.
- Record only once `ambient_capture_allowed()` clears the amp and its `AMBIENT_PLAYBACK_HANGOVER_MS` tail; start-cue frames pass through `ambient_gate_accepts()`, and frames within `AMBIENT_SERVO_HANGOVER_MS_DEFAULT` of a head movement (`servo_last_move_ms()`, changed angles only) fail `ambient_head_quiet()`.
- The scene reference is the frame last spoken about at the same dwell pose. Only valid on-pose fingerprints update it, and `!s_current.valid` and the off-pose check come before the first-impression branch (#690).
- Check `PLANNER_TASK_STACK_SIZE` whenever `GEMINI_SYSTEM_PROMPT_MAX`, a persona brief or `DIALOGUE_RECALL_*` grows.
- The model places delivery tags, and the `speech_tags.h` allow-list filters every one.
- Sound tags (`[horn]`) leave the line in `sound_clips_split()` before TTS; the `sound` tool is declared only beside `speak`, and the reflex requests its horn with `sound_clips_request()`, never `speech_queue_post()`.
- TTS uses `:streamGenerateContent?alt=sse` at 24 kHz, decoded incrementally into the ring.
- Playback waits for `AUDIO_PREROLL_BYTES` or the end of the fetch, and PCM is batched (`TTS_PCM_BATCH_BYTES`) before `audio_player_write()`.
- `s_written_total` counts each successful `ring_send()`.

**Camera and log mirror** ([why](docs/firmware/camera-and-log.md#dont))

- The camera keeps `fb_count` at 2 or more.
- Exposure and gain are left to the sensor's own loops (`set_exposure_ctrl(1)`, `set_gain_ctrl(1)`). The sensor is an OV3660, so `set_gainceiling()` takes a raw 10-bit ceiling.
- Branch on the logged PID before reading sensor registers, and A/B `set_aec2()` against measured luma before changing it.
- Re-verify motor PWM jitter whenever camera core pinning changes.
- `log_mirror.c`'s hook and drain task make no logging calls, and the hook drops and counts a line rather than waiting on the UART.
- The console stays on USB-Serial-JTAG.

**WiFi, OTA, repo hygiene** ([why](docs/firmware/wifi-and-ota.md#dont))

- `main/credentials.h` stays local (gitignored; start from the `.example`), and release-please owns `version.txt`.
- OTA installs only a strictly newer version; the build SHA only qualifies an equal version or refuses. Keep `-c safe.directory=*` in the `CMakeLists.txt` git lookup.
- Before an install the robot announces, keeps a postpone window, parks (`ota_park.c`) and holds still (`reactive_controller_set_hold()`); a failed install releases the hold. Keep the manifest's `notes` under the old 2048-byte manifest buffer (`changelog-notes.py --max-bytes`).
- Set the Improv writer (`improv_console_write()`) before any `improv_wifi_send_*()` call; Improv replies go through the VFS console, without the usb_serial_jtag driver.
- Console lines are built by `console_line_feed()`, which keeps every byte `improv_wifi_process_byte()` claims out of the line: a credentials packet holds the Wi-Fi password, and lines are logged.
- Improv is armed and disarmed only by bumping `s_improv_session` (odd while armed), which `console_line_feed()` reads per byte, with the read time, to finish a packet in flight across an edge and to drop the rest of a packet's burst (#1087).
- The Improv announcement is `improv_wifi_announce()`, and a failed attempt ends with `improv_wifi_fail_provisioning()`.
- The MQTT client is created at boot before the network supervisor and connected only on link-up, so OTA's subscription on the first link-up finds it (ADR-028 §2, #1054).
