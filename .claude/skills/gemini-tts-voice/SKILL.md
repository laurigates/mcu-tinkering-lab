---
name: gemini-tts-voice
description: The Gemini TTS request surface as this repo uses it — the streaming (SSE) endpoint, inline delivery tags, prebuilt voices, multi-speaker, and what is NOT available (custom/cloned voices). Use when changing how the robot speaks, tuning a persona, adding a voice feature, or debugging a TTS request.
user-invocable: true
argument-hint: "[topic: streaming / tags / voices / persona / multi-speaker]"
allowed-tools: Bash, Read, Grep, Glob, Edit, WebFetch
---

# Gemini TTS: What the API Offers and What This Repo Uses

Everything below was **verified against the live API** (2026-07) rather than
read off the docs page, because the docs page and the REST endpoint disagree —
see [The docs describe a different surface](#the-docs-describe-a-different-surface).
When a fact here stops holding, re-probe; don't reason from this file's memory
of the API any more than from your own.

Consumers in this repo: `packages/robocar/unified/main/gemini_tts.c` (the
request), `voice_persona.c` (the table), `dialogue_style.c` (per-line
variation), `speech_tags.c` (the tag allow-list), `audio_player.c` (playback).

## Always use the streaming endpoint

`:generateContent` synthesises the **entire** utterance before sending a byte.
Measured on one Finnish line, same body, same model:

| Endpoint | Time to first byte | Total | Response |
|---|---|---|---|
| `:generateContent` | 6.42 s | 7.42 s | 1 object |
| `:streamGenerateContent?alt=sse` | **1.16 s** | 3.78 s | 233 SSE events |

Time-to-first-byte is what a listener experiences as the robot's delay, so this
is a ~5 s latency difference for free. The firmware logs it per utterance:

```
gemini_tts: spoke 411648 bytes (8580 ms audio) first=1180 ms total=3910 ms: "…"
```

A `first=` climbing back toward `total=` means something reverted to
whole-response synthesis.

**Response shape.** `alt=sse` frames each event as `data: <json>\n\n`, and every
event is a *complete* object carrying its own chunk:

```
data: {"candidates":[{"content":{"parts":[{"inlineData":{"mimeType":"audio/l16; rate=24000; channels=1","data":"<b64>"}}]}}], …}
```

So the body holds **many** `"data"` payloads, not one. `base64_stream_feed()`
handles this: it resumes seeking after each closing quote and concatenates every
payload. The SSE framing keyword is a bare `data:` with no quotes, so it cannot
satisfy the quote-delimited `"data"` key match — the two coexist safely, and
`test_sse_event_stream` pins that.

The decoder's `base64_stream_done()` means "between payloads", **not**
"stream ended". Only the transport knows when the response is over.

## Inline delivery tags

The model performs bracketed tags inside the transcript rather than reading them
aloud. Verified working: `[whispers]`, `[laughs]`, `[sighs]`, `[excited]`,
`[bored]`, `[gasp]`.

**Tags are placed by the generating model, not drawn at random.** A tag has to
fit what is being said — `[laughs]` on a fault report is worse than no tag — and
only whoever writes the sentence knows that. This is the opposite of the
`openers`/`shapes` pools in `dialogue_style.h`, which *are* drawn randomly
precisely because their entries are interchangeable by construction. Don't
"simplify" tags into a pool; that trade was considered and rejected.

The mechanism:

1. `voice_persona_t.tag_brief` names the allowed tags in the persona's language.
2. `gemini_backend.c` appends it to the planner's `speak` prompt. It is
   **deliberately absent** from the self-report/narrate prompt — that path
   announces subsystem faults.
3. Whatever comes back is filtered in `speech_queue_post()` against
   `SPEECH_TAG_ALLOWED`, capped at `SPEECH_TAG_MAX_PER_LINE`.

**Why the filter is not optional:** an unrecognised bracketed run is *not*
silently ignored by the TTS model — it is liable to be read out as words, so the
robot announces its own stage directions. A model told it may write `[sighs]`
will eventually write `[pauses dramatically]`.

Tags are also stripped before `dialogue_style_note_spoken()` records a line's
opening words, or the avoid-list would start policing the tag instead of the
word.

## Prebuilt voices

30 of them, language-agnostic: Zephyr, Puck, Charon, Kore, Fenrir, Leda, Orus,
Aoede, Callirrhoe, Autonoe, Enceladus, Iapetus, Umbriel, Algieba, Despina,
Erinome, Algenib, Rasalgethi, Laomedeia, Achernar, Alnilam, Schedar, Gacrux,
Pulcherrima, Achird, Zubenelgenubi, Vindemiatrix, Sadachbia, Sadaltager, Sulafat.

The locale-specific `fi-FI-Chirp3-HD-*` names are **Cloud TTS**, a different
product, and 404 on `:generateContent`. Do not put one in the persona table.

Audition by ear at runtime rather than reflashing per candidate — that is what
`voice_persona_set_voice()` and the `voice` console command exist for. To sweep
all 30 offline, the sweep script pattern is: render one representative line per
voice, wrap the raw PCM in a WAV header, write `NN-Voice.wav` so they sort in a
listening order, and pace ~5 s between calls for the free-tier RPM.

## Models

- `gemini-3.1-flash-tts-preview` — current; the **only** one that streams.
- `gemini-2.5-flash-preview-tts`, `gemini-2.5-pro-preview-tts` — non-streaming.

Switching to a 2.5 model for timbre means giving up the 5 s latency win. Weigh
it deliberately.

## Multi-speaker (available, unused)

Verified 200 with the nested REST shape:

```json
"speechConfig": {
  "multiSpeakerVoiceConfig": {
    "speakerVoiceConfigs": [
      {"speaker": "Joe",  "voiceConfig": {"prebuiltVoiceConfig": {"voiceName": "Kore"}}},
      {"speaker": "Jane", "voiceConfig": {"prebuiltVoiceConfig": {"voiceName": "Puck"}}}
    ]
  }
}
```

Two named speakers, two voices, one call and one audio stream. The transcript
labels the turns (`Joe: … Jane: …`).

Not wired up — a single robot has one voice. Plausible uses if it ever earns its
place: a self-report where a second voice reads the fault list back, or a
persona that argues with itself. The cost is not the API call but the pipeline:
`speech_queue` carries one flat string per utterance, so speaker turns would
need a representation, and `voice_persona_t` holds one voice per persona.
Don't start unless the effect is worth that.

## What is NOT available here

**Custom or cloned voices.** Not on this endpoint. Google's instant voice
cloning is **Chirp 3** on Cloud TTS: allowlist-gated through sales, ~10 s consent
recording plus ~10 s reference audio, and its documented cross-language transfer
covers de/es/fr/pt — **no Finnish**. ElevenLabs Instant Voice Cloning is the
practical alternative and its `pcm_24000` output is byte-identical to what
`audio_player` already consumes (and needs no base64 layer at all), but it is a
different vendor, key, and bill.

## Persona style: prompt what is audible, not the label

`tts_style` is a natural-language delivery directive prefixed as
`"<style>: <text>"`. It is **interpreted, not spoken** — a ~40-word directive
(12+ s if read aloud) added 0.36 s to the rendered audio.

Naming an *era* does not produce that era. "1950s Finnish film" yielded correct,
flat, contemporary-sounding Finnish, because the model has no reason to infer a
delivery from a date. What reads as period is a bundle of separable, concretely
nameable traits — ask for those individually:

- theatre-derived enunciation, every syllable articulated
- full `yleiskieli` forms, no colloquial contractions
- a distinctly tapped /r/, geminates held long
- unhurried tempo with real pauses at clause boundaries
- the declamatory rise-and-fall of a radio announcer

**The ceiling on this approach:** a good part of what makes an old recording
*sound* old is the recording chain, not the speaking. Optical and early magnetic
sound ran roughly 100 Hz–5 kHz, thinning the bass and pushing the midrange
forward. TTS returns clean full-band audio, so that character is simply absent
and no amount of prompting adds it. Closing that gap means filtering the PCM on
the way to I2S — a bandpass plus light compression in `audio_player`'s drain
loop, applied to int16 samples before the I2S write. Cheap in CPU, and it would
apply uniformly rather than per-utterance. Not implemented; note it as a lever
if "sounds modern" survives the prompt work.

Everything phrase-shaped stays out of `tts_style` and `text_brief` — see
`dialogue_style.h`. A phrase named in a brief is said *every single time*.

## The docs describe a different surface

<https://ai.google.dev/gemini-api/docs/speech-generation> presents a flat
`speech_config: [{voice: …}]` array with `input` / `response_format` /
`stream: true`. That is the **OpenAI-compatibility** surface. This repo calls
`v1beta/…:streamGenerateContent` with the nested form:

```json
"speechConfig": {
  "languageCode": "fi-FI",
  "voiceConfig": {"prebuiltVoiceConfig": {"voiceName": "Charon"}}
}
```

which returns 200. The docs also say language is auto-detected with no
`languageCode` field; ours is *accepted*, but whether it is honoured or ignored
is untested — an ear test, not a code change.

**Do not "fix" the working nested shape to match the docs page.** If in doubt,
probe: a 400 names no field, so log the response body at ERROR (see
`.claude/rules/gemini-api.md`).

## Probing recipe

Write a script rather than inlining the key — the secret-protection hook blocks
`$GEMINI_API_KEY` in a Bash command line. Source `~/.api_tokens`, POST, and print
only status plus payload size:

```
code=$(curl -s -o /tmp/p.json -w '%{http_code}' -H "x-goog-api-key: ${GEMINI_API_KEY}" \
  -H 'content-type: application/json' -X POST "$URL" -d "$BODY")
```

For latency questions use `-w 'ttfb=%{time_starttransfer} total=%{time_total}'`;
the TTFB gap is the whole reason the streaming endpoint matters.

## Related

- `.claude/rules/gemini-api.md` — model-ID / `thinkingLevel` / token-budget drift on the *planner* side
- `.claude/skills/audio-static-debugging/` — when the audio plays but sounds wrong
- `packages/robocar/unified/CLAUDE.md` — the speech-is-a-queue-not-a-goal rule
- ADR-019 — the original voice design
