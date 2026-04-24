"""Generate voicing-announcement PCM clips via the Gemini TTS API.

Reads voices.json (array of {clip, text, voice} entries), calls
gemini-2.5-flash-preview-tts, and writes the returned raw 16-bit little-endian
PCM at 24 kHz mono to ``<out_dir>/tts_<clip>.pcm``.

The embedded firmware (tts_player.c) linearly upsamples 24 kHz → 44.1 kHz at
playback, so no sample-rate conversion is needed here.

Usage:
    GEMINI_API_KEY=... uv run python generate.py ../../data
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

from google import genai
from google.genai import types

MODEL = "gemini-2.5-flash-preview-tts"
RETRIES = 3
RETRY_DELAY_S = 10.0
PER_CALL_DELAY_S = 2.0


def load_voices(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def synthesize(client: genai.Client, text: str, voice: str) -> bytes:
    last_err: Exception | None = None
    for attempt in range(1, RETRIES + 1):
        response = client.models.generate_content(
            model=MODEL,
            contents=text,
            config=types.GenerateContentConfig(
                response_modalities=["AUDIO"],
                speech_config=types.SpeechConfig(
                    voice_config=types.VoiceConfig(
                        prebuilt_voice_config=types.PrebuiltVoiceConfig(
                            voice_name=voice
                        ),
                    ),
                ),
            ),
        )
        try:
            candidate = response.candidates[0]
            part = candidate.content.parts[0]
            data = part.inline_data.data
        except (AttributeError, IndexError, TypeError) as err:
            last_err = err
            print(
                f"  attempt {attempt}/{RETRIES}: empty response, retrying in {RETRY_DELAY_S}s",
                file=sys.stderr,
            )
            time.sleep(RETRY_DELAY_S)
            continue
        if data:
            return data
        last_err = RuntimeError("inline_data.data was falsy")
        time.sleep(RETRY_DELAY_S)
    raise RuntimeError(f"Failed after {RETRIES} attempts: {last_err}")


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: generate.py <out_dir>", file=sys.stderr)
        return 2
    out_dir = Path(sys.argv[1]).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    voices_path = Path(__file__).with_name("voices.json")
    entries = load_voices(voices_path)

    client = genai.Client()

    for i, entry in enumerate(entries):
        clip = entry["clip"]
        text = entry["text"]
        voice = entry["voice"]
        out_path = out_dir / f"tts_{clip}.pcm"
        print(f"Synthesizing {clip!r}: {text!r} ({voice}) → {out_path}")
        pcm = synthesize(client, text, voice)
        out_path.write_bytes(pcm)
        print(f"  wrote {len(pcm)} bytes ({len(pcm) / 2 / 24000.0:.2f} s @ 24kHz mono)")
        if i < len(entries) - 1:
            time.sleep(PER_CALL_DELAY_S)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
