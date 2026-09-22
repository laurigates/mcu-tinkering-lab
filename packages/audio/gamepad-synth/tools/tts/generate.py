"""Generate voicing-announcement PCM clips via the Gemini TTS API.

Reads voices.json (array of {clip, text, voice} entries), calls
gemini-2.5-flash-preview-tts, and writes the returned raw 16-bit little-endian
PCM at 24 kHz mono to ``<out_dir>/tts_<clip>.pcm``.

The embedded firmware (tts_player.c) linearly upsamples 24 kHz → 44.1 kHz at
playback, so no sample-rate conversion is needed here.

Usage:
    GEMINI_API_KEY=... uv run python generate.py <out_dir> [voices.json] [--trim] [--wav]

The voices file defaults to the one beside this script (gamepad-synth's own
vocabulary). robocar-bringup passes its own — one generator, two vocabularies,
no copy to drift.

Two optional per-entry keys, both absent from the firmware vocabularies and
therefore inert for them:

  ``style``          Delivery directive, prefixed as "<style>: <text>" — the
                     documented Gemini form. It is *interpreted*, not spoken.
                     This is how robocar-unified's `voice_persona_t.tts_style`
                     reaches the model, so a sweep over candidate directives
                     here is auditioning the exact mechanism the firmware uses.
  ``language_code``  BCP-47 for speech_config.language_code (e.g. "fi-FI").

``--wav`` wraps the output in a 24 kHz mono 16-bit WAV header and writes
``<clip>.wav`` instead of ``tts_<clip>.pcm``. For listening on a workstation;
the firmware paths want the bare PCM and must not pass it.
"""

from __future__ import annotations

import array
import json
import os
import sys
import time
from pathlib import Path

from google import genai
from google.genai import types

MODEL = "gemini-3.1-flash-tts-preview"
RETRIES = 3
RETRY_DELAY_S = 10.0
# Free-tier quota is per-model requests-per-minute, so the right pacing depends
# on how many entries the vocabulary has. Three clips can go at 2 s; a 30-voice
# sweep at 2 s is 30 rpm and will start collecting 429s partway through. Env
# override rather than a new positional argument, so the firmware callers keep
# their existing command lines unchanged.
PER_CALL_DELAY_S = float(os.environ.get("TTS_CALL_DELAY_S", "2.0"))


def load_voices(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def synthesize(
    client: genai.Client,
    text: str,
    voice: str,
    style: str = "",
    language_code: str = "",
) -> bytes:
    # "<style>: <text>" is the documented delivery-directive form and the same
    # one gemini_tts.c builds. An empty style must send the bare text rather
    # than a leading ": ", which the model would otherwise have to interpret.
    prompt = f"{style}: {text}" if style else text
    speech_config = types.SpeechConfig(
        voice_config=types.VoiceConfig(
            prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name=voice),
        ),
        **({"language_code": language_code} if language_code else {}),
    )
    last_err: Exception | None = None
    for attempt in range(1, RETRIES + 1):
        try:
            response = client.models.generate_content(
                model=MODEL,
                contents=prompt,
                config=types.GenerateContentConfig(
                    response_modalities=["AUDIO"],
                    speech_config=speech_config,
                ),
            )
        except Exception as err:  # noqa: BLE001 - SDK raises several unrelated types
            # A rate-limit or transient server error used to propagate and kill
            # the whole run. That costs every remaining entry, and on a 30-entry
            # sweep the failure lands after twenty-odd successful calls have
            # already been paid for. Back off and retry instead; a genuinely
            # permanent error still exits after RETRIES.
            last_err = err
            wait = RETRY_DELAY_S * attempt
            print(
                f"  attempt {attempt}/{RETRIES}: {type(err).__name__}: {err}; "
                f"retrying in {wait:.0f}s",
                file=sys.stderr,
            )
            time.sleep(wait)
            continue
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


def wav_bytes(pcm: bytes, rate: int = 24000, channels: int = 1) -> bytes:
    """Wrap 16-bit LE PCM in a canonical 44-byte WAV header.

    Written by hand rather than via `wave` so the whole tool stays a single
    file with one dependency. The firmware never sees this — it consumes the
    bare PCM, which is why --wav is opt-in.
    """
    bits = 16
    block_align = channels * bits // 8
    header = b"".join(
        (
            b"RIFF",
            (36 + len(pcm)).to_bytes(4, "little"),
            b"WAVEfmt ",
            (16).to_bytes(4, "little"),  # PCM fmt chunk size
            (1).to_bytes(2, "little"),  # format 1 = PCM
            channels.to_bytes(2, "little"),
            rate.to_bytes(4, "little"),
            (rate * block_align).to_bytes(4, "little"),  # byte rate
            block_align.to_bytes(2, "little"),
            bits.to_bytes(2, "little"),
            b"data",
            len(pcm).to_bytes(4, "little"),
        )
    )
    return header + pcm


def main() -> int:
    flags = {"--trim", "--wav"}
    argv = [a for a in sys.argv[1:] if a not in flags]
    trim = "--trim" in sys.argv
    as_wav = "--wav" in sys.argv
    if len(argv) not in (1, 2):
        print(
            "usage: generate.py <out_dir> [voices.json] [--trim] [--wav]",
            file=sys.stderr,
        )
        return 2
    out_dir = Path(argv[0]).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    # The voices file is a parameter so a second project can drive the same
    # generator with its own vocabulary instead of copying this script — see
    # packages/robocar/bringup/tools/voices.json. Defaults to the sibling file,
    # so gamepad-synth's own `just tts-generate` is unchanged.
    voices_path = (
        Path(argv[1]).resolve()
        if len(argv) == 2
        else Path(__file__).with_name("voices.json")
    )
    entries = load_voices(voices_path)
    print(f"Reading {len(entries)} entries from {voices_path}")

    client = genai.Client()

    for i, entry in enumerate(entries):
        clip = entry["clip"]
        text = entry["text"]
        voice = entry["voice"]
        style = entry.get("style", "")
        language_code = entry.get("language_code", "")
        out_path = out_dir / (f"{clip}.wav" if as_wav else f"tts_{clip}.pcm")
        print(f"Synthesizing {clip!r}: {text!r} ({voice}) → {out_path}")
        if style:
            print(f"  style: {style}")
        pcm = synthesize(client, text, voice, style, language_code)
        if trim:
            before = len(pcm)
            pcm = trim_silence(pcm)
            print(f"  trimmed {(before - len(pcm)) / 2 / 24000.0:.2f} s of silence")
        out_path.write_bytes(wav_bytes(pcm) if as_wav else pcm)
        print(f"  wrote {len(pcm)} bytes ({len(pcm) / 2 / 24000.0:.2f} s @ 24kHz mono)")
        if i < len(entries) - 1:
            time.sleep(PER_CALL_DELAY_S)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
