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
| `snap [n]` | Dump the next `n` planner frames over the console as base64 JPEG — see "Seeing what the camera sends" |
| `cam …` | Read live sensor gain/exposure; `cam gainceiling 0-6`, `cam ae -2..2`, `cam brightness -2..2` tune exposure without a reflash |
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

## Seeing what the camera actually sends

Every claim about image quality here was inference until `frame_dump.c` existed — the sensor registers, the JPEG size and the frames themselves had never been read off a device. Two instruments, and the pairing is the diagnosis:

- **`frame_stats_log()` runs on every planner frame.** It decodes the JPEG at 1/8 scale and logs `luma mean/p5/p50/p95` next to the sensor's **live** `gain`/`exp`/`ceil` registers. Read them together: high gain + maxed exposure + low luma means the sensor is *starved* and the gain ceiling needs raising; low gain + low exposure + low luma means auto-exposure is not converging; low mean with a **high p95** is backlit, not unlit. The cached `sensor_t` status struct cannot answer this — it holds what was last *written*, not what the AEC/AGC loops have since chosen, which is why `camera_read_exposure()` reads the registers over SCCB instead.
- **`frame_dump_maybe()` emits the JPEG itself**, base64-framed with a length and CRC32, for the first `FRAME_DUMP_BOOT_FRAMES` frames and on `snap [n]`. These are the exact bytes handed to `gemini_backend_plan()`, dumped while the planner still holds the buffer — no copy, no lifetime question, no chance of showing a different frame from the one the model saw.

```
just robocar-unified::monitor | tee /tmp/robocar.log
uv run tools/decode-frame-dump.py /tmp/robocar.log /tmp/frames
```

The monitor is **read-only** and resets the board on attach (`tools/esp32s3-monitor.sh` never writes to the port), so you cannot type `snap` through it — that is why the first frames auto-dump at boot. Use `screen`/`picocom` if you want the command.

The length+CRC are not decoration: the USB-Serial-JTAG console **silently drops** TX bytes once the host stops reading for 50 ms (`TX_FLUSH_TIMEOUT_US`) and reports success anyway. Without the checksum a host hiccup yields a corrupt JPEG that looks exactly like a broken camera.

Exposure is a **runtime knob** (`cam gainceiling 0-6`), not a compile-time constant, because whether a frame is "too dark" is a judgement only a human looking at the room can make and a reflash per trial is far too slow a loop. `CAMERA_DEFAULT_GAINCEILING` is deliberately still 2x — the esp32-camera driver's own forced default for every OV2640, and the lowest of seven — so the first measurements describe the camera as it has been behaving. Pin the winning value there once the luma numbers say what it should be.

## Voice (MAX98357A)

The robot speaks through a MAX98357A I2S amplifier on GPIO7/8/9. See [ADR-019](../../docs/decisions/ADR-019-robocar-voice-gemini-tts.md) for the design.

**Speech is a queue, not a goal.** `speak` is deliberately *not* a `goal_kind_t` — it travels via `speech_queue` alongside `goal_state`. Goals are last-write-wins with a TTL that collapses to STOP; applying those semantics to speech would truncate sentences mid-word and make talking and driving mutually exclusive. If you find yourself adding `GOAL_KIND_SPEAK`, read the header comment in `speech_queue.h` first.

The pipeline is two Gemini calls: Robotics-ER decides *what* to say (riding the existing planner call — no extra vision inference), then `gemini-3.1-flash-tts-preview` renders it. Audio is 24 kHz mono PCM, base64 inline, no WAV header — decoded **incrementally** into a PSRAM ring by `base64_stream_feed()` while the player task drains it into I2S, so playback starts before the download finishes. A few seconds of audio is hundreds of kB; it can never be buffered whole.

**Playback does not start at the first byte — it waits for `AUDIO_PREROLL_BYTES`, or for the fetch to finish.** Gemini's TTS stream is frequently *slower* than real time: measured across seven live captures (2026-07), real-time factors were 0.55 / 0.79 / 0.84 / 1.88 / 2.70 / 3.08 / 3.81, and the three below 1.0 were all the same *short* line — a fixed 0.9–2.7 s time-to-first-byte amortised over less audio. Draining on arrival therefore ran the ring dry mid-sentence 12–18 times per utterance. That is not merely a gap: ESP-IDF's I2S TX ISR keeps a `desc_num - 1` deep free-buffer queue and **drops the oldest entry** when it overflows, so after a stall `i2s_channel_write()` is handed the descriptor the DMA is about to transmit and its `memcpy` lands in a buffer being read out — torn samples, i.e. broadband noise. The gate's OR-condition is the important half: a short utterance completes before the threshold and plays from a *complete* buffer, so it cannot underrun at all.

Each utterance logs `first=`/`total=`/`rtf=`. **`rtf` below 1.00 means Gemini generated slower than playback consumes** — expected sometimes, and exactly what the preroll gate absorbs. `first=` drifting up toward `total=` means something reverted to whole-response synthesis. `ring full … dropping` or `i2s_channel_write failed` in the log means the gate is not holding and the ring or preroll needs re-sizing.

The decoder itself is **proven correct against real data**: `main/base64.c` was replayed over seven captured SSE bodies at twelve chunk sizes and produced byte-identical PCM to a reference JSON+base64 decode every time, and the literal `"data"` never occurs outside an audio payload (1096/1096 payloads). If the voice sounds wrong, it is not the decode — look at the ring, the gate, or the amp.

**The TTS call goes to `:streamGenerateContent?alt=sse`, not `:generateContent`.** The non-streaming endpoint synthesises the whole utterance before sending a byte, which left the ring buffer with almost nothing to overlap: measured 6.42 s to first byte versus **1.16 s** streaming, same body and model. The streamed body is a sequence of SSE events each carrying its own `"data"` payload, so the decoder seeks past each closing quote and concatenates all of them — `base64_stream_done()` means "between payloads", not "stream ended". Each utterance logs `first=`/`total=` ms; `first=` drifting up toward `total=` means something reverted to whole-response synthesis.

**Delivery tags are placed by the model, not drawn from a pool.** Gemini performs `[sighs]`, `[laughs]`, `[whispers]`, `[excited]`, `[bored]`, `[gasp]` inline rather than reading them aloud. Unlike openers and shapes — interchangeable by construction, hence random — a tag has to fit the sentence, so the persona's `tag_brief` invites the *generating* model to place them. Everything coming back is filtered against the allow-list in `speech_tags.h` at the single choke point in `speech_queue_post()`: an unrecognised bracketed run is liable to be **spoken aloud**, so the robot would announce its own stage directions. The self-report path deliberately gets no `tag_brief` — half its lines name a dead subsystem.

Full API surface, including the multi-speaker config that exists but is unused and why custom/cloned voices are not available here: [`.claude/skills/gemini-tts-voice/`](../../../.claude/skills/gemini-tts-voice/SKILL.md).

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
- `CONFIG_CAMERA_JPEG_MODE_FRAME_SIZE=65536` (with `_AUTO=n`, `_CUSTOM=y`) — the AUTO default computes `width*height/5`, a hard **15360-byte** ceiling at QVGA. Past it the driver aborts accumulation, queues the truncated frame, finds no EOI marker and retries until a 4 s timeout — surfacing as `Camera capture failed` and a forced STOP, not as a bad image. Any detailed or bright scene exceeds it at quality 15, and raising the AGC gain ceiling makes it worse because noise inflates JPEG size
- mDNS (`robocar-unified.local`) needs **no** Kconfig switch — it comes from the `espressif/mdns` managed component. There is no `CONFIG_MDNS_ENABLED` symbol; a line setting one is reported as an unknown symbol and silently ignored

## Don't

- Don't call `motor_controller.c` directly from anywhere except `reactive_controller.c` — the executor owns motor output. Console/manual movement goes through `reactive_controller_manual()`, which takes a short lease the executor applies *after* the obstacle reflex; a second task writing the PCA9685 directly both fought the 30 Hz executor and bypassed the reflex
- Don't add goal sources outside `planner_task.c` — structured goals keep the two layers decoupled. If a new goal source is needed, it should write `goal_state` the same way the planner does
- Don't fold speech into `goal_t` — see the Voice section above and `speech_queue.h`
- Don't put a specific phrase, opener or filler word in a persona's `text_brief` — everything named there is said every time. Phrase-shaped flavour goes in the `openers`/`shapes` pools; see `dialogue_style.h`
- Don't name an *era* in `tts_style` and expect the delivery to follow — "1950s Finnish film" produced flat contemporary Finnish. Name the audible traits instead (enunciation, tempo, `yleiskieli` forms, tapped /r/, held geminates). Note the ceiling: the era's *recording chain* (~100 Hz–5 kHz) is a filter, not a speaking style, and no prompt adds it
- Don't move delivery tags into a `dialogue_pool_t` — they're model-placed because they must fit the sentence, and the random pools exist for things that don't. Don't drop the allow-list filter either; unknown tags get read out loud
- Don't switch the TTS call back to `:generateContent` — it costs ~5 s of perceived latency (see the Voice section)
- Don't "normalise" the audio path to 16 kHz to match the ThinkPack projects — 24 kHz is Gemini TTS's native rate, and matching it avoids a resampling stage entirely
- Don't buffer the TTS response whole — it's hundreds of kB and the response buffer is 16 kB. The streaming decoder exists for this reason
- Don't start playback on the first decoded byte "to cut latency" — that is the bug the preroll gate fixes, and the measured cost was up to 2855 ms of ring dry-out per sentence. If latency needs cutting, lower `AUDIO_PREROLL_BYTES` and watch `rtf=`; don't remove the gate
- Don't feed `audio_player_write()` the decoder's raw 3-byte quartets — every ring send yields to the higher-priority player, so it preempted the fetch task ~25 000 times a second exactly when it needed to get ahead of real time. Batch first (`TTS_PCM_BATCH_BYTES`)
- Don't set `fb_count = 1` on the camera. Capture halts while the app holds the only buffer, so the planner's frame is one full period (15 s) old — the robot plans motion from a stale view, and `grab_mode` is inert below 2 buffers
- Don't re-add `set_aec_value()` / `set_agc_gain()` alongside `set_exposure_ctrl(1)` / `set_gain_ctrl(1)` — the sensor's own loops rewrite those registers every frame, so the calls do nothing but read like deliberate tuning. `set_agc_gain(s, 0)` is worse than nothing: it writes the *minimum* gain
- Don't flip `set_aec2()` on the strength of web folklore — the vendored driver's setter writes the inverse of its argument while its getter reads the raw bit, so even the direction of the change is unresolved in the source. A/B it against measured luma or leave it
- Don't add direct GPIO motor control — everything goes through PCA9685 via `motor_controller.c`
- Don't bypass the TCA9548A — devices on different channels can share addresses (e.g. PCA9685 and OLED would conflict without it)
- Don't commit `main/credentials.h` — it's gitignored; use the `.example` as template
- Don't hand-edit `version.txt` — release-please owns it
- Don't change camera core pinning without re-verifying motor PWM jitter
- Don't resurrect `claude_*` or `ollama_*` source files. If a second backend becomes necessary, design an abstraction; old code paths are not a foundation
