# ADR-032: Real-Time Conversation over the Gemini Live API

**Status**: proposed
**Date**: 2026-11-07
**Confidence**: 5/10 (the model, latency and output format are measured from a
cloud container, not the robot's WiFi; nothing here has run on the XIAO, and the echo and
memory figures below are estimates until the phase-1 bench run)
**Issue**: [#619](https://github.com/laurigates/mcu-tinkering-lab/issues/619)
**Extends**: [ADR-019](ADR-019-robocar-voice-gemini-tts.md),
[ADR-024](ADR-024-conversational-entity-and-multimodal-voice.md)
**Related**: [#948](https://github.com/laurigates/mcu-tinkering-lab/issues/948)
(voice-turn latency), [#947](https://github.com/laurigates/mcu-tinkering-lab/issues/947)
(promised actions), [#950](https://github.com/laurigates/mcu-tinkering-lab/issues/950)
(spoken reasoning)

---

## Context

A voice turn today is record → `generateContent` (`gemini-flash-latest`, clip
as WAV plus a JPEG) → `gemini-3.8-flash-tts` (`voice_turn.c`). On 2026-11-05
six answered turns took 8.2–12.8 s from clip to reply text, plus about 2.3 s
to the first TTS audio (#948): a person waits 11–15 s and often repeats
themselves, which starts another turn.

The Live API probe (`packages/robocar/unified/tools/live-probe/`, README §
Measured) ran on 2026-11-07:

| Fact | Measured |
|---|---|
| Conversational Live models the key reaches | 7; `gemini-3.8-live` chosen |
| End of speech → first audio | 1.1–1.4 s (3 runs), server VAD ending the turn |
| Output | `audio/pcm;rate=24000`, arriving 4–8× faster than real time |
| Persona | Finnish, in the persona's register, from a system instruction built from `voice_persona.c` |
| Input tokens | 255 for a 3.44 s clip; post-VAD silence not counted |
| `gemini-robotics-er-2-streaming-preview` | refuses an `AUDIO` response (close code 1007) |

So the Live API removes roughly 10 s per turn, and it also removes the second
model call: Live returns speech itself, so no TTS request follows.

The documented limits that shape the design (README § Facts from the
documentation): audio-only sessions last at most 15 min without context
compression, audio+video 2 min; a connection lasts about 10 min and ends with
`GoAway`; native-audio models take no `language_code`, so Finnish comes from
the system instruction.

## Decision

Add a Live conversation path beside the existing voice turn, built in two
phases. The existing path stays as the fallback and for `listen`.

### 1. Transport: `esp_websocket_client` over TLS, JSON messages, no SDK

- Endpoint `wss://generativelanguage.googleapis.com/ws/google.ai.generativelanguage.v1beta.GenerativeService.BidiGenerateContent`,
  the URI google-genai 2.34 builds (`live.py`). The key goes in the
  `x-goog-api-key` header of the upgrade request, as the SDK sends it, never in
  the URL, which reaches the log.
- The managed component `espressif/esp_websocket_client` with
  `crt_bundle_attach`, so the TLS setup matches `gemini_http.c`. mbedTLS
  buffers are already in PSRAM (`CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y`).
- Messages are JSON: `setup` first, then `realtimeInput.audio` with base64
  `audio/pcm;rate=16000` chunks, `serverContent` back. Building and parsing
  them is pure C beside `voice_history.c` and `gemini_parse.c`, host-tested
  against messages captured by the probe (it already decodes every field this
  needs).
- 100 ms chunks, as the probe sends: 3.2 kB of PCM, about 4.3 kB as base64
  JSON, 10 messages per second upstream.

### 2. Downstream audio: the TTS ring and player, unchanged

The reply is 24 kHz s16 mono, the format `base64_stream_feed()`, the PSRAM ring
and `audio_player` already take. Each `serverContent.modelTurn.parts[].inlineData.data`
is fed to the same incremental decoder. The preroll gate stays: at the
measured arrival rate it fills in a fraction of a second, and it still covers
a slow chunk.

`serverContent.interrupted` (phase 2) flushes the ring.

### 3. Echo: half-duplex first, acoustic echo cancellation second

The speaker and the PDM mic sit centimetres apart. Sending the robot's own
voice upstream would make the server VAD hear speech, interrupt the reply and
answer itself.

- **Phase 1, half-duplex.** While the amp plays and for
  `AMBIENT_PLAYBACK_HANGOVER_MS` after, nothing is sent upstream: the existing
  `ambient_capture_allowed()` gate, the same one that keeps playback out of
  the ambient gate and the pre-roll. No barge-in. The person waits for the
  robot to finish, which at a 5–9 s reply is the cost of this phase.
- **Phase 2, barge-in with AEC.** ESP-SR's AFE with acoustic echo
  cancellation, fed the I2S TX samples as the reference, and the cleaned
  signal sent upstream during playback. To measure before committing: CPU on
  core 1 beside the camera and planner, the AFE's PSRAM, whether the library
  fits the 3.5 MB `ota_0` partition, and whether the PDM mic's clock domain
  needs drift compensation against the I2S TX clock.

### 4. Session lifecycle: open on speech, close on silence

- **Open** on the local speech trigger (`speech_trigger.c`, #617), the same
  event that starts a hands-free turn now. The pre-roll ring (#616) is sent
  first, at once, then the live mic at real-time pace, so the words that
  triggered the session reach the model.
- **Close** after an idle window with no speech either way. The existing 7 s
  conversation follow-up window is the starting value. Also close on
  `GoAway`, on a session-length cap well under the 15 min limit, and on any
  socket error. Nothing reconnects silently; the next trigger opens a new
  session.
- **Ration** in `voice_budget`: a session counts as a request against the
  hands-free ration and the per-boot ceiling, and session seconds are logged
  per session, because Live billing grows with the active context.
- **Ignore talk not addressed to the robot.** The system instruction keeps
  the `__IGNORE__` contract only if the model can reply in text, and a Live
  session replies in audio. Phase 1 relies on the trigger and the idle close;
  whether a native-audio model can be told to stay silent is a probe item.

### 5. Coexistence with the planner

- Robotics-ER stays the planner. The streaming ER model cannot produce audio,
  so one model cannot do both.
- While a session is open, the planner's `speak` is withheld (the existing
  `voice_turn_is_busy()` rule widened to "session open"), and the planner's
  request is not suppressed otherwise: it still steers.
- TLS: during a session the voice turn and TTS hold no connection, so at most
  two TLS contexts are open (planner and Live), one fewer than today's worst
  case.
- Camera: phase 1 sends no video, keeping the 15 min audio-only limit. Phase
  2 may send one JPEG at session start as `realtimeInput.video`, then measure
  whether that switches the session to the 2 min audio+video limit.

### 6. Model and persona

- `gemini-3.8-live`, a constant with the same rule as every other model id in
  this firmware (`.claude/rules/gemini-api.md` §1). `just robocar-unified::live-probe-models`
  is the check when it 404s.
- The system instruction is built from `voice_persona.c`, as the probe builds
  it (name, `text_brief`, `tts_style`, "answer in Finnish, briefly"), and the
  voice is the persona's prebuilt voice. The 3.1 preview ignored "briefly";
  3.8 kept to 5–9 s.
- No tools in phase 1, so the model cannot claim to move (#947). Movement by
  function call is a later decision.

## Phases

| Phase | Scope | Exit check |
|---|---|---|
| 0 | Probe (done, 2026-11-07) | model, latency, format, persona measured |
| 1 | Half-duplex Live session behind a console switch (`voice live on`), TTS ring reuse, open on trigger, close on idle | on the bench: first audio within 2 s of end of speech, no self-interruption over 20 turns, PSRAM low-water mark logged |
| 2 | AEC and barge-in, `interrupted` handling, optional start-of-session frame | interrupting a reply stops it within 300 ms; no self-interruption with the amp at normal volume |

## Consequences

- One request per conversation instead of two per turn, and about 10 s less
  wait per turn.
- A long-lived TLS socket on core 1 for the session's length. The planner's
  handshake can still take seconds on CPU (#948 notes a >5 s software ECDHE);
  the Live handshake happens once per session rather than once per turn.
- Two voice paths to keep working until the old one can go. The old one stays
  for `listen` and as the fallback when the Live endpoint fails.
- The `__IGNORE__` filter does not carry over as is (§4).

## Open questions (to measure, not to argue)

1. Live latency and handshake time from the XIAO over its WiFi link; the probe
   ran from a cloud container.
2. Free-tier quota and session concurrency for `gemini-3.8-live` on the
   owner's key (AI Studio publishes it; the docs do not).
3. Whether a native-audio model can stay silent on talk not addressed to it.
4. ESP-SR AEC cost and fit (§3).
5. Whether the 1950s delivery carries over, by ear, on `reply.wav`.
