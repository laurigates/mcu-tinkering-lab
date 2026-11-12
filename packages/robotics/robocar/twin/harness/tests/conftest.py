from __future__ import annotations

import sys
import textwrap
from pathlib import Path

import pytest

FAKE_TWIN = Path(__file__).with_name("fake_twin.py")


@pytest.fixture
def fake_twin_cmd() -> list[str]:
    return [sys.executable, str(FAKE_TWIN)]


@pytest.fixture
def write_scenario(tmp_path: Path):
    """Write YAML to <tmp>/twin/scenarios/<id>.yaml and return the path."""

    def _write(scenario_id: str, body: str) -> Path:
        d = tmp_path / "twin" / "scenarios"
        d.mkdir(parents=True, exist_ok=True)
        p = d / f"{scenario_id}.yaml"
        p.write_text(f"id: {scenario_id}\n" + textwrap.dedent(body), encoding="utf-8")
        return p

    return _write
