# TTS Clip Generator

Generates short voicing-announcement clips via the Google Gemini TTS API. The
firmware embeds the resulting raw 16-bit LE PCM (24 kHz mono) via
`EMBED_FILES` and linearly upsamples to 44.1 kHz at playback.

## Usage

```bash
# From packages/audio/gamepad-synth/ (via justfile):
GEMINI_API_KEY=... just tts-generate

# Or directly:
cd tools/tts
uv sync
GEMINI_API_KEY=... uv run python generate.py ../../data
```

The API key is read from `GEMINI_API_KEY` in the environment; per repo
convention it lives in `~/.api_tokens` (sourced by mise).

Edit `voices.json` to change wording or the prebuilt voice. Clips are committed
to git under `../../data/tts_<clip>.pcm`; regeneration is an explicit developer
action, not part of the firmware build.

## Verifying a clip

Raw PCM cannot be autoplayed by most tools. To audition:

```bash
ffplay -autoexit -f s16le -ar 24000 -ac 1 ../../data/tts_continuous.pcm
```

## Format

- 16-bit signed little-endian
- 24 kHz mono
- No container (raw samples only) — ESP-IDF `EMBED_FILES` exposes
  `_binary_tts_<clip>_pcm_start` / `_end` symbol pairs that bracket the buffer
