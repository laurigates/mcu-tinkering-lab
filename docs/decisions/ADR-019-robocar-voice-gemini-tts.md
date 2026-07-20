# ADR-019: Robocar Voice via Gemini TTS over I2S (MAX98357A)

**Status**: accepted
**Date**: 2026-07-20
**Source**: conversation 2026-07-20
**Confidence**: 7/10

---

## Context

`robocar-unified` had no audio output beyond a piezo buzzer on GPIO2. Adding a
MAX98357A I2S class-D amplifier gives the robot a voice, letting the planner
narrate what it sees rather than only acting on it.

The planner already calls Gemini Robotics-ER 1.6 once per second with a camera
frame (ADR-016). That call is the natural place to decide *what* to say, since
the model already has the scene in front of it — a separate "what should I say"
inference would duplicate the upload and double the cost.

Three constraints shaped the design:

1. **The GPIO budget was nearly exhausted.** The XIAO ESP32-S3 Sense exposes 11
   header pins. After motors, buzzer, I2C, ultrasonic, and USB serial, exactly
   three remained — D8/D10, which are also the Sense expansion board's microSD
   SPI bus.
2. **Gemini TTS returns 24 kHz 16-bit mono PCM**, base64-encoded inline, with no
   WAV header. At ~48 kB/s of PCM, a four-second utterance is ~192 kB of audio
   and ~256 kB of base64 — 16× the 16 kB HTTP response buffer the planner
   backend uses.
3. **`goal_state` is state, not events.** It is last-write-wins with a 1.5 s TTL
   and deliberately collapses to `GOAL_KIND_STOP` when stale.

## Decision

### Speech is a queue, not a goal

`speak` is **not** a `goal_kind_t`. It travels through a separate
`speech_queue` (a FreeRTOS queue) that runs alongside `goal_state`.

Modelling speech as a goal was the obvious first design and it is wrong on
three counts:

- **Truncation.** Last-write-wins means the next motion goal (~1 s later) would
  overwrite the utterance, cutting sentences mid-word.
- **Meaningless expiry.** TTL-expiry forcing `STOP` has no sensible
  interpretation for audio that is already playing.
- **False exclusivity.** A union member makes talking and driving mutually
  exclusive, when the robot should do both concurrently.

The two therefore have different lifetimes and different consumers. The planner
writes to both from the same response; the reactive executor reads the goal, and
the TTS task reads the queue.

Queue depth is 2 with a **drop-newest** overflow policy: if the robot is still
speaking, the in-flight utterance wins. Dropping the older line instead would
reintroduce exactly the stutter this design avoids.

### Two Gemini calls, one round trip for the decision

- The **Robotics-ER** call gains a `speak(text)` function declaration, emitted
  *in addition to* a motion call. The parser recovers both from the same
  response, so deciding what to say costs no extra request.
- A **separate TTS call** (`gemini-3.1-flash-tts-preview`) renders the text.
  These are different models and cannot be merged.

`gemini_parse` previously read only `parts[0]`. Since parallel function calls
land in successive parts, it now scans all parts: the first non-`speak` call
becomes the motion goal, and a `speak` call is extracted separately regardless
of position.

A response carrying `speak` but no motion call still returns `ESP_FAIL` with
`GOAL_KIND_STOP` — preserving the existing fail-safe contract — while the
utterance is still delivered. The robot holds position and talks.

### Streaming decode, not buffering

Audio is decoded **incrementally** from the HTTP body straight into a 96 kB
PSRAM ring buffer, which a separate player task drains into I2S concurrently.
Playback therefore starts before the download finishes, putting perceived
latency near time-to-first-byte rather than full-download time.

Two tasks are required because `esp_http_client_perform()` blocks for the whole
transfer: a fetch task on Core 1 fills the ring, a player task on Core 1 drains
it. Both stay off Core 0 to preserve the motor-PWM timing guarantee ADR-016
pins the reactive controller to.

The decoder (`base64_stream_feed`) locates the payload by scanning for the
`"data"` key and decodes only the string that follows, tolerating quartets and
the key itself straddling chunk boundaries. A full streaming JSON parser was
rejected as disproportionate for a response whose shape is fixed and known.

Back-pressure is deliberate: when the ring is full the sink blocks, stalling the
HTTP event handler, which stops draining the socket and lets TCP throttle the
download to real-time playback speed. Without it, a long utterance would need
unbounded PSRAM.

### Rate

I2S runs natively at **24 kHz** to match the TTS output, avoiding a resampling
stage. This differs from the 16 kHz used by the ThinkPack audio projects.

## Consequences

**Positive**

- Unbounded vocabulary, reactive to what the camera actually sees.
- No extra vision inference — `speak` rides the existing planner call.
- Audio begins playing mid-download; a slow link degrades gracefully.
- Voice failure is non-fatal: if any of the queue, player, or TTS task fails to
  start, the robot logs a warning and drives on mute.

**Negative**

- **The microSD slot is gone.** D8/D10 are the Sense expansion board's SPI bus.
  This is irreversible without giving up audio; no alternative pins exist.
- **The GPIO budget is now fully allocated.** Further digital I/O must go
  through the MCP23017 on TCA9548A channel 2.
- **Speech requires WiFi** and adds per-utterance API cost. The prompt asks the
  model to speak sparingly, but this is guidance, not a hard cap.
- **Power risk.** ~1 A transient draw on a rail where brownout detection is
  already disabled for motor inrush. Mitigated by decoupling and supply
  guidance in `WIRING.md`, but not by anything in firmware.
- Latency from decision to first audio is roughly 1–2 s (TTS synthesis plus
  transfer), so remarks lag the scene that prompted them.

**Neutral**

- The piezo buzzer on GPIO2 stays for boot chirps and reflex alerts, which must
  be instant and must not depend on the network.
- The ThinkPack `thinkpack-audio` component was evaluated and **not** reused:
  its volume cap is a toddler-safety constant, its ring buffer is explicitly
  single-task with no synchronisation (unusable across the HTTP-callback →
  player-task boundary, where FreeRTOS `Ringbuf` is the right primitive), and
  its pitch-shift and record/playback state machine are not needed here.

## Alternatives considered

**Pre-baked WAV clips in the existing 960 kB SPIFFS partition.** Offline,
instant, zero marginal cost, and testable without a network. Rejected because
the vocabulary is fixed at build time (~30 s of audio total), which cannot
respond to an arbitrary scene. Still the better choice if offline operation
later becomes a requirement.

**Synthesised R2D2-style chirps.** No assets, no network, trivial code, and
expressive of mood. Rejected as too close to what the buzzer already conveys —
it would not have justified the hardware. Remains a good fallback for when WiFi
is unavailable.

**`speak` as a `goal_kind_t`.** Rejected for the three reasons above; recorded
here because it is the intuitive design and will be proposed again otherwise.

## Related

- [ADR-016: Hierarchical AI Controller](ADR-016-hierarchical-ai-controller.md) — the planner/executor split and core affinity this builds on
- [ADR-013: Single-board XIAO ESP32-S3](ADR-013-single-board-xiao-esp32s3.md) — the GPIO budget being consumed here
- [ADR-011: Gamepad Synth I2S Audio](ADR-011-gamepad-synth-i2s-audio.md) — prior MAX98357A usage in this monorepo
