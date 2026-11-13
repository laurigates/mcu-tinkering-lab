from __future__ import annotations

import sys
import textwrap
from pathlib import Path

import pytest

FAKE_TWIN = Path(__file__).with_name("fake_twin.py")
FAKE_PLANT = Path(__file__).with_name("fake_plant.py")


@pytest.fixture
def fake_twin_cmd() -> list[str]:
    return [sys.executable, str(FAKE_TWIN)]


@pytest.fixture
def fake_plant_cmd() -> list[str]:
    return [sys.executable, str(FAKE_PLANT)]


@pytest.fixture
def write_arena(tmp_path: Path):
    """Write YAML to <tmp>/twin/arenas/<name>.yaml; returns the path a scenario
    in <tmp>/twin/scenarios names it by."""

    def _write(name: str, body: str) -> str:
        d = tmp_path / "twin" / "arenas"
        d.mkdir(parents=True, exist_ok=True)
        (d / f"{name}.yaml").write_text(textwrap.dedent(body), encoding="utf-8")
        return f"arenas/{name}.yaml"

    return _write


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
