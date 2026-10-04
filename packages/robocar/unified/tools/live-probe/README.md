# Gemini Live API probe (issue #619)

A workstation-only probe of the Gemini Live API (`BidiGenerateContent` over a
WebSocket), run before any firmware is written for real-time conversation. It
opens one session as Teuvo, streams one spoken question at real-time pace, and
records what comes back. No board is involved.

```bash
just robocar-unified::live-probe-models   # Live models the key can reach (1 ListModels call)
just robocar-unified::live-probe          # synthesise the question, run one session
just robocar-unified::live-probe clip=/path/to/any-16bit.wav
just robocar-unified::live-probe-test     # offline tests, no key or network
```

Outputs land in the repo's `tmp/live-probe/`, uncommitted like the other voice
recipes' output: `question.wav` (the rendered
prompt, reused on later runs), `reply.wav`, and `report.json`.

## What it measures

| `report.json` field | Meaning |
|---|---|
| `model` | The Live model used, chosen from ListModels — see below |
| `first_audio_after_speech_end_ms` | Last clip sample sent → first reply audio byte. The number a listener feels |
| `first_audio_after_stream_end_ms` | Same, from `audio_stream_end` (after the trailing silence) |
| `turn_complete_after_speech_end_ms` | End of speech → `turnComplete` |
| `output_mime_types`, `output_rate_hz` | As sent by the server, never assumed. `None` if chunks disagree |
| `arrival_rtf` | Reply audio seconds ÷ wall-clock seconds it took to arrive. Below 1.0 the device would need a preroll gate, as the TTS path has |
| `input_transcription`, `output_transcription` | What the model heard, and what it said |
| `usage_last` | The session's last `usageMetadata` (token counts) |
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

**The model id is never hardcoded.** The probe calls ListModels, keeps the
models that support `bidiGenerateContent`, drops the single-purpose ones
(translate, transcribe), and refuses to guess when more than one conversational
model remains — pass `--model` then. Rule: `.claude/rules/gemini-api.md` §1.

**The persona is extracted, not retyped**: name, voice, `text_brief` and
`tts_style` of `[VOICE_PERSONA_FI_1950]` come out of `main/voice_persona.c`
with the style sweep's C-literal reader, and become the session's system
instruction. Native-audio Live models pick the output language themselves and
take no `language_code`, so the instruction also says to answer in Finnish.
Whether the 1950s delivery survives that move is a listening judgement on
`reply.wav`.

## Facts from the documentation (2026-10-04, not yet confirmed by a run)

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

## Measured

Not yet run. The first live run's `report.json` numbers belong here and on
issue #619: model, time to first audio, output format and rate, `arrival_rtf`,
token usage, and a verdict on whether the Finnish persona carried over.
