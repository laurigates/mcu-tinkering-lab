# Gemini Live API probe (issue #619)

A workstation-only probe of the Gemini Live API (`BidiGenerateContent` over a
WebSocket), run before any firmware is written for real-time conversation. It
opens one session as Teuvo, streams one spoken question at real-time pace, and
records what comes back. No board is involved.

```bash
just robocar::live-probe-models   # Live models the key can reach (1 ListModels call)
just robocar::live-probe          # synthesise the question, run one session on gemini-3.8-live
just robocar::live-probe clip=/path/to/any-16bit.wav
just robocar::live-probe model=gemini-3.1-flash-live-preview
just robocar::live-probe-test     # offline tests, no key or network
```

Outputs land in the repo's `tmp/live-probe/`, uncommitted like the other voice
recipes' output: `question.wav` (the rendered
prompt, reused on later runs), `report.json`, and `reply.wav` — the last only
when every reply chunk declared the same rate, since a WAV at a guessed rate
plays back pitch-shifted.

## Interactive play

`play.py` turns the probe into a play session. Teuvo watches through a
workstation camera, listens on its mic and talks through its speaker while a
child plays. Every session leaves a transcript to tune the game against. No
board is involved, and tool calls are answered by a stub and logged.

```bash
just robocar::live-play-devices                       # camera and audio indices
just robocar::live-play                               # building game, built-in camera
just robocar::live-play moomins 2                     # Moomin game, Desk View camera
just robocar::live-play building 0 gemini-3.8-live --barge-in   # headphones: never mute the mic
```

The session ends on Ctrl-C, on `--max-minutes` (default 10), or on the server's
`GoAway`. Each run writes `tmp/live-probe/play/<UTC stamp>-<game>/`:

| File | Contents |
|---|---|
| `transcript.md` | Who said what and when, heartbeats, tool calls. Read this first |
| `instruction.txt` | The exact system instruction sent |
| `session.json` | Duration, end reason, heartbeat, turn and tool-call counts, last usage |
| `events.jsonl` | Every logged event, for scripts |
| `frames/` | Each JPEG sent, numbered as in the transcript |
| `teuvo.wav`, `mic.wav` | Teuvo's audio (24 kHz) and what the mic sent (16 kHz) |

### How a session works

- **Instruction**: the persona from `main/voice_persona.c` (as in the latency
  probe), then `games/register.fi.txt`, then the game's `prompt`. The register
  covers how Teuvo plays in every game: short lines, the child leads, he
  comments only on what is visible, and he calls `wait` when nothing is new.
- **Games** are TOML files in `games/` with `title`, `prompt`, `heartbeat` and
  `opening`. A new game is a new file.
- **Frames** go in at 1 per second, the documented video rate. A frame alone
  never starts a reply. The game's `heartbeat` text goes out at most every
  `--heartbeat-s` (8 s), and only after `--quiet-s` (2.5 s) of silence from
  both sides. It is never sent while Teuvo is talking, since a prompt during
  generation cancels it.
- **Silence is a tool call.** `wait` lets Teuvo answer a heartbeat by saying
  nothing. With proactive audio and "stay silent" in the prompt alone, a
  30-second run on 2026-11-16 with nothing in view drew a spoken reply to 4 of
  4 heartbeats, one of them about a tower that was not there. With `wait`, a
  45-second run gave the greeting and then answered 4 of 4 heartbeats with
  `wait`.
- **Tools**: `wait`, `look(pan_deg, tilt_deg)` and `sound(name)`. The sound
  names are extracted from `main/sound_clips_core.c`. All three are
  `NON_BLOCKING`, and the stub's answers go back with `SILENT` scheduling so
  Teuvo does not talk about them.
- **Echo**: the mic is muted from Teuvo's first reply sample until
  `--hangover-ms` (400 ms) after the last one, so laptop speakers do not make
  him interrupt himself. `--barge-in` turns that off.
- **Session length**: context-window compression is on. Without it an
  audio+video session ends after 2 minutes.
- **Not set**: `enable_affective_dialog`. The docs list it for
  `gemini-3.8-live`, but on 2026-11-16 it closed the session with 1007
  "invalid argument" on both v1beta and v1alpha. A bisect that kept every other
  field showed it alone causes the close.

## What it measures

| `report.json` field | Meaning |
|---|---|
| `model` | The Live model used, checked against ListModels — see below |
| `connect_ms` | Connect start → the SDK's `connect()` returning, which it does after `setupComplete`. `connect_to_setup_complete_ms` stays `None` on a live run, because the SDK consumes that message itself |
| `first_audio_after_speech_end_ms` | Last clip sample sent → first reply audio byte. The number a listener feels |
| `first_audio_after_stream_end_ms` | Same, from `audio_stream_end` (after the trailing silence) |
| `turn_complete_after_speech_end_ms` | End of speech → `turnComplete` |
| `output_mime_types`, `output_rate_hz` | As sent by the server, never assumed. `None` if chunks disagree |
| `arrival_rtf` | Reply audio seconds ÷ wall-clock seconds it took to arrive. Below 1.0 the device would need a preroll gate, as the TTS path has |
| `input_transcription`, `output_transcription` | What the model heard, and what it said |
| `usage_last` | The session's last `usageMetadata` (token counts). The probe keeps reading for 1.5 s after `turnComplete`, because the docs do not promise usage arrives before it |
| `go_away_time_left` | Set if the server announced a connection end |

Three choices make the latency number mean something:

- **The clip is streamed in 100 ms chunks at wall-clock pace**, then 800 ms of
  silence (`--tail-silence-ms`) and `audio_stream_end`. That is the shape the
  firmware would send from the microphone, so the server's voice-activity
  detector sees what it would see from the robot.
- **Latency is measured from the end of the speech**, not the end of the stream.
  The silence is for the server's VAD; the listener has been waiting since they
  stopped talking.
- **Input is resampled to 16 kHz mono 16-bit PCM** (`audio/pcm;rate=16000`). A
  16 kHz `mic dump` capture (via `tools/decode-mic-dump.py`) passes through
  unchanged.

**The model id is checked, never assumed.** The probe calls ListModels, keeps
the models that support `bidiGenerateContent`, drops the single-purpose ones
(translate, transcribe), and refuses to guess when more than one conversational
model remains. Seven remained on 2026-11-07, so the recipe passes
`model=gemini-3.8-live`; a retired id fails with the list instead of an opaque
connect error. Rule: `.claude/rules/gemini-api.md` §1.

**The persona is extracted, not retyped**: name, voice, `text_brief` and
`tts_style` of `[VOICE_PERSONA_FI_1950]` come out of `main/voice_persona.c`
with the style sweep's C-literal reader, and become the session's system
instruction. Native-audio Live models pick the output language themselves and
take no `language_code`, so the instruction also says to answer in Finnish.
Whether the 1950s delivery survives that move is a listening judgement on
`reply.wav`.

## Facts from the documentation (2026-10-04)

From the Gemini API Live guides, read through the `gemini-api-docs` MCP server:

- Audio is raw little-endian 16-bit PCM both ways. Input is natively 16 kHz;
  **output is always 24 kHz** — the same rate as the TTS path and its ring.
- Without context-window compression, audio-only sessions are limited to
  **15 minutes** and audio+video sessions to **2 minutes**. A connection lasts
  about **10 minutes** and is preceded by a `GoAway` message; session
  resumption carries a session across connections.
- Context window: 128k tokens for native-audio models. Audio accumulates at
  about 25 tokens per second, and billing compounds over the active context,
  which is why the firmware should open a session per conversation and close
  it when idle.
- The guides' current conversational model is `gemini-3.1-flash-live-preview`
  (deprecations table: no shutdown date announced). Affective dialog and
  proactive audio are documented as unsupported on it. Thinking defaults to
  `minimal` for latency.
- Free-tier quotas are not published in the docs; the rate-limits page points
  to AI Studio (`aistudio.google.com/rate-limit`), per project and tier.

## Measured (2026-11-07)

Run from a Claude Code cloud container, not the workstation, so `connect_ms`
and the latencies include that network path. The clip is `question.fi.json`
rendered by `gemini-3.8-flash-tts` (voice Kore, 3.44 s); the session voice is
the persona's own, Schedar. Reports were kept in `tmp/live-probe/` and are
not committed.

| Model | Tail silence | First audio after speech end | After `audio_stream_end` | Reply | `arrival_rtf` | Thinking tokens |
|---|---|---|---|---|---|---|
| `gemini-3.8-live` | 800 ms | 1112 ms | 204 ms | 9.29 s | 6.17 | 95 |
| `gemini-3.8-live` | 800 ms (recipe) | 1199 ms | 292 ms | 5.18 s | 7.87 | 105 |
| `gemini-3.8-live` | 3000 ms | 1432 ms | −1705 ms | 5.64 s | 6.75 | 256 |
| `gemini-3.1-flash-live-preview` | 3000 ms | 1301 ms | −1835 ms | 22.43 s | 4.03 | — |

- **Model.** ListModels offered seven conversational Live models:
  `gemini-3.8-live`, `gemini-3.8-live-extended-thinking`,
  `gemini-3.1-flash-live-preview`, three `gemini-2.5-flash-native-audio-*`, and
  `gemini-robotics-er-2-streaming-preview`. The last refuses an `AUDIO`
  response (close code 1007, "The requested combination of response modalities
  (AUDIO) is not supported"), so it cannot be the voice; it is a candidate for
  a streaming planner. `gemini-3.8-live` kept to "lyhyesti" (5–9 s replies);
  the 3.1 preview answered for 22 s and did not think.
- **Latency.** The server's own VAD ends the turn: with 3 s of trailing
  silence the first audio arrived 1.7 s *before* `audio_stream_end` was sent.
  End of speech to first audio is 1.1–1.4 s on both models, against 11–15 s on
  the current record → `generateContent` → TTS path (#948). `connect_ms` was
  504 ms.
- **Format.** `audio/pcm;rate=24000` in every chunk: the TTS ring's rate. It
  arrives 4–8× faster than it plays, so the device's preroll gate is not on the
  critical path.
- **Tokens.** `gemini-3.8-live`: 630 text tokens for the system instruction,
  255 audio tokens for the 3.44 s clip (the same with 0.8 s or 3 s of trailing
  silence, so post-VAD silence was not counted), and about 25.6 output audio
  tokens per second (238 for 9.29 s). The 3.1 preview counted 456 and 349.
  No `GoAway` arrived in these short sessions.
- **Persona.** Every 3.8 reply was Finnish, in the persona's register
  ("automobiili", "palveluksessanne", "herrasmiesmäinen", formal "Teidät"),
  for example: "Minä olen Teuvo, mekaanisella älyllä varustettu automobiili ja
  palveluksessanne maanteiden pitkillä taipaleilla." The input transcription
  rendered "Hei" as "Hey". Whether the delivery (enunciation, tempo) carried
  over is a listening judgement on `reply.wav`, not yet made.
- **Quota.** Not measured: the runs used a temporary key whose tier is not
  known here.
