# ADR-024: Conversational Entity — Multi-Turn Multimodal Interaction, Autonomous VAD, and Expressive Personality

**Status**: proposed
**Date**: 2026-09-13
**Source**: conversation 2026-09-13
**Confidence**: 8/10

---

## Context

The `robocar-unified` voice architecture previously evolved in narrow slices:
- **ADR-019**: MAX98357A I2S amplifier added so the 15 s Gemini Robotics-ER planner could narrate observations via `speak(text)`.
- **ADR-020**: PDM microphone added as an uncalibrated amplitude excursion / spectral evidence gate (`ambient_audio.c`) on the planner's `speak` declaration.
- **ADR-022**: Planner dormancy ladder and spend ceiling to prevent runaway API spend in static, silent rooms.
- **PR #454 (Slice B)**: Console-only push-to-talk `listen [ms]` turn using `gemini-flash-latest`.

In practice, this created an experience where Robocar feels like an appliance resetting between isolated monologues:
1. At boot, it speaks a formal greeting and self-report ("Hyvää päivää, minä olen Robocar...").
2. The planner makes one unprompted remark about the scene.
3. The speech budget enforces a 60-second quiet gap (`SPEECH_BUDGET_MIN_GAP_MS = 60000`) and the dormancy ladder puts it to sleep.
4. If a sound occurs, the planner prompt is told *"The room sounds different; remark on that, not what you see"*, causing the model to make generic guesses about loudness ("Kuuluipa kova ääni") without ever receiving or transcribing the user's speech.
5. In `voice_turn.c`, `audio_clip_normalise()` only subtracts DC bias without digital gain, leaving the raw MEMS mic audio at ~-45 dBFS (peak 100–300 out of 32767), largely unintelligible to Gemini.
6. Expressive delivery tags (`[whispers]`, `[sighs]`, `[laughs]`) were banned in `voice_turn.c` ("no stage directions") and omitted in `self_report.c` to prevent "inappropriate" reactions to hardware faults.
7. The microSD card slot on the XIAO Sense expansion board is permanently unavailable because its SPI pins (GPIO 7, 8, 9 / D8, D9, D10) were reassigned to the I2S amplifier.

The user's goal is for Robocar to be an **expressive, endearing entity** with personality, capable of hearing, understanding, and having multi-turn conversations, chiming in when appropriate, and reacting dramatically to its physical and diagnostic state.

## Decision

### 1. Elevate Robocar from Monologue Narrator to Conversational Entity: "Teuvo"
- Give Robocar the character name **Teuvo** (*"Teuvo, maanteiden kuningas"*), embracing the irony of a small autonomous wheeled robot cruising the floorboards with 1950s gentlemanly decorum and grand automotive confidence.
- Enable expressive delivery tags everywhere: Teuvo should be allowed to sigh `[sighs]` or chuckle wryly `[laughs]` over his own hardware faults or obstacles, and whisper `[whispers]` in quiet or dim settings.

### 2. Audio Processing: Software Digital Gain & Peak Normalisation
- Update `audio_clip.c` to apply digital gain and peak normalisation to the recorded PDM audio before WAV header framing and base64 encoding.
- Ensure conversational speech at ~1 m distance reaches -6 dBFS to -3 dBFS peak without clipping, ensuring clear comprehension by Gemini.

### 3. Multi-Turn Multimodal Dialogue (`voice_turn.c`)
- Maintain a rolling conversational history buffer (last 4–6 turns) in PSRAM.
- Maintain dialogue turns in the Gemini `contents` list (`user` → `model` → `user`), preserving the context of ongoing conversations across turns.
- Attach the current camera framebuffer (JPEG) to the conversational turn so Robocar can see what the user is asking about ("What are you looking at?", "Can you see my keys?").
- Inject recent physical telemetry (ultrasonic obstacle distance, drivetrain state, recent movement) into the system instruction so Robocar can explain its physical actions.

### 4. Hands-Free Conversational Engagement: Cloud VAD & Name Detection
- Connect `ambient_listener.c`'s speech energy detection to automatically trigger a `voice_turn_request()`.
- Instruct Gemini Flash to act as intent & wake-name classifier:
  - If addressed directly (e.g. by name, "Robocar", or question directed at it), reply in character.
  - If people are talking in the room and Robocar is idle, allow occasional impromptu chime-ins with dry, witty remarks.
  - If background chatter is uninteresting or irrelevant, reply with `__IGNORE__` (which produces no speech).
- Once spoken to, enter an **Active Conversation Window** (5–8 s): pulse status LEDs, hold the speech budget lockout open, and listen for immediate follow-up turns before signing off.

## Consequences

**Positive**
- Robocar becomes a genuinely interactive character rather than a silent appliance with periodic status announcements.
- Speech comprehension improves immediately through digital gain normalisation.
- Robocar can converse about what it sees and does (multimodal vision + movement telemetry).
- Hands-free operation removes the need for typing `listen` in the serial console.

**Negative / Constraints**
- Cloud VAD adds API requests on `gemini-flash-latest` when the room is noisy (mitigated by prompt-based `__IGNORE__` filtering and ambient noise floor adaptation).
- Keeping conversational history in PSRAM consumes memory (bounded by limiting history to text/prompts and releasing raw WAV audio after each turn).
- MicroSD card slot remains unusable due to hardware pin sharing with the MAX98357A I2S amplifier.

## Related
- [ADR-019](ADR-019-robocar-voice-gemini-tts.md) — Voice output path via I2S amp
- [ADR-020](ADR-020-ambient-audio-speech-gate.md) — Ambient audio gate
- [ADR-022](ADR-022-planner-dormancy-and-spend-ceiling.md) — Planner dormancy & spend ceiling
- `.claude/skills/gemini-tts-voice/` — Delivery tags and TTS streaming specification
