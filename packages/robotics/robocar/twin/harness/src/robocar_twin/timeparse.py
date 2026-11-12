"""Duration strings used in scenario files: "15.6s", "900ms", "2m", "1m30s", "1h"."""

from __future__ import annotations

import re

_UNIT_MS = {"ms": 1.0, "s": 1000.0, "m": 60_000.0, "h": 3_600_000.0}
_PART = re.compile(r"(\d+(?:\.\d+)?)(ms|s|m|h)")


class TimeParseError(ValueError):
    """A duration string that does not parse."""


def parse_ms(value: object, *, field: str = "time") -> int:
    """Return a duration in whole milliseconds.

    Accepts a string of one or more number+unit parts (``ms``, ``s``, ``m``,
    ``h``), e.g. ``"1m30s"``, or the integer ``0``. A bare number other than 0
    is refused: ``15`` could mean seconds or milliseconds, and guessing wrong
    shifts a scenario by a factor of 1000 without any error.
    """
    if isinstance(value, bool):
        raise TimeParseError(f"{field}: expected a duration like '15s', got {value!r}")
    if isinstance(value, int) and value == 0:
        return 0
    if not isinstance(value, str):
        raise TimeParseError(
            f"{field}: expected a duration string with a unit (ms, s, m, h), got {value!r}"
        )
    text = value.strip().replace(" ", "")
    if not text:
        raise TimeParseError(f"{field}: empty duration")
    pos = 0
    total = 0.0
    for match in _PART.finditer(text):
        if match.start() != pos:
            break
        total += float(match.group(1)) * _UNIT_MS[match.group(2)]
        pos = match.end()
    if pos != len(text):
        raise TimeParseError(
            f"{field}: cannot parse duration {value!r} (use e.g. '900ms', '15.6s', '1m30s')"
        )
    ms = round(total)
    if abs(ms - total) > 1e-6:
        raise TimeParseError(
            f"{field}: {value!r} is not a whole number of milliseconds"
        )
    return ms


def format_ms(ms: int) -> str:
    """Render milliseconds as seconds for messages, e.g. 15600 -> '15.6s'."""
    return f"{ms / 1000:g}s"
