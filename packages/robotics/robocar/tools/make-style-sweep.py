"""Emit a TTS sweep vocabulary for auditioning Teuvo delivery directives.

The point of the sweep is to find out whether more prompt work can still move
the voice, *before* anyone writes DSP for it. So the arms differ in exactly one
thing — `tts_style` — and share one line, one voice (Charon) and one language.
Changing the voice too would make it a two-variable experiment and neither
answer would be attributable.

Two of the arms are controls rather than candidates:

  00-bare       no directive at all, so the shipped one can be judged against
                what the model does unprompted. Without this the sweep cannot
                tell "this directive is good" from "Charon sounds like that".
  01-shipped    the directive the firmware actually sends, EXTRACTED from
                voice_persona.c rather than retyped. A retyped control is not a
                control: a transcription slip reads exactly like a real finding,
                and this text is six concatenated C literals.
  02-era-only   reproduces a known-negative from the gemini-tts-voice skill
                ("naming the era alone yielded flat contemporary Finnish"). If
                this arm comes back sounding period, the harness is measuring
                something other than the directive and the rest is suspect.

Usage:
    python3 make-style-sweep.py > styles.teuvo.json
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

PERSONA_C = Path(__file__).resolve().parent.parent / "main" / "voice_persona.c"

# The table holds one .tts_style per persona and Robocar's comes first, so an
# unscoped search for ".tts_style =" silently returns "Say in a friendly,
# natural tone" — a real directive, in the wrong language, for the wrong
# character. It extracts cleanly and reads like a successful control. Anchor on
# the designated initialiser and search only after it.
PERSONA_MARKER = "[VOICE_PERSONA_FI_1950] ="

# The 30 prebuilt voice names live in the skill, which records them as verified
# against the live API. Parsed rather than copied here for the same reason the
# directive is extracted rather than retyped: a silent typo in one of thirty
# proper nouns costs an API error in the middle of a sweep, and "Rasalgethi"
# is not a word anyone proofreads correctly.
SKILL_MD = (
    Path(__file__).resolve().parents[4]
    / ".claude"
    / "skills"
    / "gemini-tts-voice"
    / "SKILL.md"
)


def extract_voices() -> list[str]:
    s = SKILL_MD.read_text(encoding="utf-8")
    m = re.search(r"(\d+) of them, language-agnostic:(.*?)\.\s*\n", s, re.S)
    if not m:
        raise SystemExit(f"{SKILL_MD}: could not find the prebuilt-voice list")
    names = [n.strip() for n in m.group(2).replace("\n", " ").split(",")]
    # The prose states its own count; disagreeing with it means the list was
    # edited and the parse drifted. Fail rather than sweep a truncated set,
    # which would look exactly like a complete sweep.
    claimed = int(m.group(1))
    if len(names) != claimed:
        raise SystemExit(
            f"{SKILL_MD}: prose says {claimed} voices, parsed {len(names)}"
        )
    bad = [n for n in names if not re.fullmatch(r"[A-Za-z]+", n)]
    if bad:
        raise SystemExit(f"{SKILL_MD}: not voice names: {bad}")
    return names


# The one spoken line, shared by every arm. Picked for trait coverage rather
# than taste: it carries two tapped /r/ ("raportoi", "järjestelmät"), a geminate
# ("kunnossa"), and a colon that invites the clause pause the directive asks
# for. Extracted from the shipped fallback pool so the sweep judges a sentence
# Teuvo genuinely says.
LINE_MARKER = "No niin. Teuvo raportoi:"


def c_string_after(source: str, marker: str) -> str:
    """Concatenate the adjacent C string literals following `marker`.

    Stops at the first comma outside a literal, i.e. the end of the struct
    field. Handles backslash escapes so a field containing \" does not end the
    scan early.
    """
    i = source.index(marker) + len(marker)
    out: list[str] = []
    while i < len(source):
        ch = source[i]
        if ch == ",":
            break
        if ch != '"':
            i += 1
            continue
        i += 1
        while i < len(source) and source[i] != '"':
            if source[i] == "\\":
                out.append(source[i + 1])
                i += 2
                continue
            out.append(source[i])
            i += 1
        i += 1
    return "".join(out)


def extract_line(source: str) -> str:
    """Pull the shared sweep line out of the shipped fallback pool."""
    for literal in re.findall(r'"((?:[^"\\]|\\.)*)"', source):
        if literal.startswith(LINE_MARKER):
            return literal
    raise SystemExit(
        f"{PERSONA_C.name}: no fallback line starting {LINE_MARKER!r}. "
        "The pool was edited; re-point LINE_MARKER at a line that still exists "
        "rather than hardcoding one here."
    )


def main() -> int:
    voices_mode = "--voices" in sys.argv
    source = PERSONA_C.read_text(encoding="utf-8")
    if PERSONA_MARKER not in source:
        raise SystemExit(
            f"{PERSONA_C.name}: no {PERSONA_MARKER!r}. The persona enum was "
            "renamed; re-point PERSONA_MARKER rather than dropping the scope, "
            "which would silently extract another persona's directive."
        )
    persona = source[source.index(PERSONA_MARKER) :]
    shipped = c_string_after(persona, ".tts_style =")
    # Whole source, not the persona slice: the s_fi_* pools are declared above
    # the table, so scoping this the same way would find nothing.
    line = extract_line(source)
    if not shipped:
        raise SystemExit(f"{PERSONA_C.name}: .tts_style extracted empty")

    # Each candidate states its whole directive rather than appending to the
    # shipped one, because "shipped + extra clause" makes a long directive
    # longer and the model weights the tail differently. One self-contained
    # instruction per arm keeps the comparison readable.
    arms: list[tuple[str, str]] = [
        ("00-bare", ""),
        ("01-shipped", shipped),
        ("02-era-only", "Puhu kuin 1950-luvun suomalaisessa elokuvassa"),
        (
            # Tempo pushed well past the shipped "rauhallinen, hieman verkkainen".
            "03-slower",
            "Puhu hyvin hitaasti ja juhlallisesti, painota jokaista sanaa erikseen "
            "ja pidä selvä tauko jokaisen lauseenosan jälkeen, kuin lausuisit "
            "runoa näyttämöltä",
        ),
        (
            # Drops the film-actor framing entirely: news-reader, not character.
            "04-radio",
            "Puhu kuin Yleisradion uutistenlukija vuonna 1955: virallinen, "
            "hillitty ja asiallinen, huoliteltua yleiskieltä, tasainen "
            "julistava sävelkulku, ei lainkaan tunteenomaista värähtelyä",
        ),
        (
            # Timbre axis. Orthogonal to tempo and register, so a win here
            # combines with a win from 03 or 04 rather than competing.
            "05-gravel",
            "Puhu kuin iäkkäämpi herrasmies: matala, hieman karhea ja kulunut "
            "ääni, arvokas ja verkkainen, mutta selkeästi artikuloiden",
        ),
        (
            # Falsifiable test of the skill's claim that the RECORDING CHAIN
            # cannot be prompted — it says the model returns clean full-band
            # audio no matter what. If this arm actually sounds band-limited,
            # the planned bandpass DSP is unnecessary and the skill needs
            # correcting. Expect it to fail; the value is in knowing.
            "06-oldrec",
            "Puhu kuin vanhassa 1950-luvun radiolähetyksessä tai "
            "elokuvateatterin kaiuttimessa: kapea ja ohut äänialue, vähän "
            "bassoa, keskiäänet korostuneina, kuin nauhalta toistettuna",
        ),
    ]

    if voices_mode:
        # One arm per prebuilt voice, every one carrying the SHIPPED directive.
        # Holding the directive fixed is what makes this a voice comparison
        # rather than a second style sweep with the voice confounded in.
        # Numbered so the rendered files sort into a listening order.
        entries = [
            {
                "clip": f"{i:02d}-{v}",
                "text": line,
                "voice": v,
                "language_code": "fi-FI",
                "style": shipped,
            }
            for i, v in enumerate(extract_voices())
        ]
    else:
        entries = [
            {
                "clip": clip,
                "text": line,
                "voice": "Charon",
                "language_code": "fi-FI",
                **({"style": style} if style else {}),
            }
            for clip, style in arms
        ]
    json.dump(entries, sys.stdout, ensure_ascii=False, indent=2)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
