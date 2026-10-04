"""Project-roles layer: `#define`s from C headers, and which of them are pins.

A pin assignment is a *role* (`I2S_BCLK_PIN`) bound to a GPIO. How a project
spells that binding is its pin-macro convention, and the repo has three:

    robocar        #define I2S_BCLK_PIN GPIO_NUM_7    in main/pin_config.h
    balancebot     #define PIN_IMU_SDA 6              in src/pin_config.h
    gamepad-synth  the robocar shape, but in main.c   (no pin_config.h at all)

Only `robocar` is implemented. The other two are named here so the next
project to adopt a hardware.toml extends this table rather than writing a
parser of its own (ADR-021: one join, many emitters).
"""

from __future__ import annotations

import re
from collections.abc import Iterable
from pathlib import Path

from .errors import HardwareError

# A `#define NAME value` line, with any trailing `//` comment dropped. The
# generated pin_defs.typ is drift-guarded byte for byte, so a change here is a
# change to that file's content. The separators are `[ \t]`, not `\s`: `\s`
# crosses newlines, so a valueless `#define PIN_CONFIG_H` used to take the next
# line as its value — harmless while that line is an `#include`, but a `#define`
# in that position would have vanished from the result.
_DEFINE = re.compile(
    r"^[ \t]*#define[ \t]+(\w+)[ \t]+(.+?)(?:[ \t]*//.*)?$", re.MULTILINE
)

# Convention name -> regex whose group 1 is the GPIO number, matched against
# the whole macro value. A macro whose value does not match is not a pin role
# (a frequency, an address, a PCA9685 channel).
CONVENTIONS: dict[str, re.Pattern[str]] = {
    "robocar": re.compile(r"GPIO_NUM_\(?(\d+)\)?"),
}


def parse_header(path: Path) -> dict[str, str]:
    """Return `#define` name -> raw value string for one header."""
    text = path.read_text(encoding="utf-8")
    return {m.group(1): m.group(2).strip() for m in _DEFINE.finditer(text)}


def parse_defines(paths: Iterable[Path]) -> dict[str, str]:
    """Merge the `#define`s of several headers, in order.

    A name defined in two inputs with different values would make every
    consumer depend on argument order, so that is an error rather than a
    silent pick.
    """
    paths = list(paths)
    merged: dict[str, str] = {}
    for path in paths:
        for name, value in parse_header(path).items():
            if name in merged and merged[name] != value:
                raise HardwareError(
                    f"{name} is defined as both {merged[name]!r} and {value!r} "
                    f"across {[str(p) for p in paths]}"
                )
            merged[name] = value
    return merged


def roles_from_defines(defines: dict[str, str], convention: str) -> dict[str, int]:
    """Return role -> GPIO for every macro that the convention reads as a pin."""
    pattern = CONVENTIONS.get(convention)
    if pattern is None:
        raise HardwareError(
            f"unknown pin-macro convention {convention!r}; implemented: "
            f"{sorted(CONVENTIONS)} (see tools/hardware/header.py for the others)"
        )
    roles: dict[str, int] = {}
    for name, value in defines.items():
        m = pattern.fullmatch(value)
        if m:
            roles[name] = int(m.group(1))
    return roles
