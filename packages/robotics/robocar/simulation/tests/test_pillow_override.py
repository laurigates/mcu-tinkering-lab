"""The Pillow override in pyproject.toml must match the Pillow pin.

uv's override replaces every Pillow requirement, the project's own pin
included, so a Renovate bump of the ``Pillow==`` line alone would not change
the resolved version. This test fails until both lines agree.
"""

import re
import tomllib
from pathlib import Path

PYPROJECT = Path(__file__).resolve().parent.parent / "pyproject.toml"


def _pin(requirements: list[str]) -> str | None:
    for req in requirements:
        m = re.fullmatch(r"(?i)pillow==(\S+)", req.strip())
        if m:
            return m.group(1)
    return None


def test_pillow_override_matches_pin():
    data = tomllib.loads(PYPROJECT.read_text())
    pin = _pin(data["project"]["dependencies"])
    override = _pin(data["tool"]["uv"]["override-dependencies"])
    assert pin is not None, "no Pillow== pin in [project] dependencies"
    assert override == pin, f"override pillow=={override} != pin Pillow=={pin}"
