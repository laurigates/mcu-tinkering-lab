"""Generate voicing-announcement PCM clips via the Gemini TTS API.

Reads voices.json (array of {clip, text, voice} entries), calls
gemini-2.5-flash-preview-tts, and writes the returned raw 16-bit little-endian
PCM at 24 kHz mono to ``<out_dir>/tts_<clip>.pcm``.

The embedded firmware (tts_player.c) linearly upsamples 24 kHz → 44.1 kHz at
playback, so no sample-rate conversion is needed here.

Usage:
    GEMINI_API_KEY=... uv run python generate.py <out_dir> [voices.json]

The voices file defaults to the one beside this script (gamepad-synth's own
vocabulary). robocar-bringup passes its own — one generator, two vocabularies,
no copy to drift.
"""

from __future__ import annotations

import array
import json
import sys
import time
from pathlib import Path

from google import genai
from google.genai import types

MODEL = "gemini-3.1-flash-tts-preview"
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


def trim_silence(pcm: bytes, threshold: int = 400, pad_ms: int = 40) -> bytes:
    """Strip leading and trailing near-silence from 24 kHz mono s16 PCM.

    Gemini returns roughly 0.3 s of silence at each end of every clip. For a
    one-word announcement that is more silence than speech, and a sweep that
    speaks thirteen of them pays it thirteen times. Opt-in (--trim) because
    trimming changes when an overlaid clip starts, which matters to a caller
    that mixes rather than plays one-shot.

    `pad_ms` is kept at each end so a plosive onset is not clipped.
    """
    samples = array.array("h")
    samples.frombytes(pcm)
    n = len(samples)
    first = next((i for i, v in enumerate(samples) if abs(v) > threshold), None)
    if first is None:
        return pcm  # all silence; hand it back rather than returning nothing
    last = next(i for i in range(n - 1, -1, -1) if abs(samples[i]) > threshold)

    pad = int(24000 * pad_ms / 1000)
    start = max(0, first - pad)
    end = min(n, last + 1 + pad)
    return samples[start:end].tobytes()


def main() -> int:
    argv = [a for a in sys.argv[1:] if a != "--trim"]
    trim = "--trim" in sys.argv
    if len(argv) not in (1, 2):
        print("usage: generate.py <out_dir> [voices.json] [--trim]", file=sys.stderr)
        return 2
    out_dir = Path(argv[0]).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    # The voices file is a parameter so a second project can drive the same
    # generator with its own vocabulary instead of copying this script — see
    # packages/robocar/bringup/tools/voices.json. Defaults to the sibling file,
    # so gamepad-synth's own `just tts-generate` is unchanged.
    voices_path = Path(argv[1]).resolve() if len(argv) == 2 else Path(__file__).with_name(
        "voices.json"
    )
    entries = load_voices(voices_path)
    print(f"Reading {len(entries)} entries from {voices_path}")

    client = genai.Client()

    for i, entry in enumerate(entries):
        clip = entry["clip"]
        text = entry["text"]
        voice = entry["voice"]
        out_path = out_dir / f"tts_{clip}.pcm"
        print(f"Synthesizing {clip!r}: {text!r} ({voice}) → {out_path}")
        pcm = synthesize(client, text, voice)
        if trim:
            before = len(pcm)
            pcm = trim_silence(pcm)
            print(f"  trimmed {(before - len(pcm)) / 2 / 24000.0:.2f} s of silence")
        out_path.write_bytes(pcm)
        print(f"  wrote {len(pcm)} bytes ({len(pcm) / 2 / 24000.0:.2f} s @ 24kHz mono)")
        if i < len(entries) - 1:
            time.sleep(PER_CALL_DELAY_S)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
