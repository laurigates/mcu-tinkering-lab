"""Every scenarios/*.yaml: validated always, run through the twin binary.

The binary is $ROBOCAR_TWIN_BIN or <twin dir>/build/robocar-twin. A missing
binary fails the run unless ROBOCAR_TWIN_ALLOW_MISSING=1, so a CI job that
forgot to build the twin cannot pass by skipping every scenario.

A scenario whose ``plant:`` is ``physics`` starts the plant process
(packages/robotics/robocar/simulation), so its run carries the ``physics``
marker: ``just robocar-twin::test`` deselects it and ``test-physics`` selects
it. Validation is unmarked, so the light suite validates every scenario file,
physics ones included; under ``-m physics`` a scenario is validated when its
run loads it.
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest
import yaml
from robocar_twin.assertions import evaluate
from robocar_twin.runner import default_twin_dir, resolve_twin_bin, run_scenario
from robocar_twin.scenario import load_scenario

SCENARIO_DIR = default_twin_dir() / "scenarios"
SCENARIOS = sorted(SCENARIO_DIR.glob("*.yaml"))


def names_physics_plant(path: Path) -> bool:
    """Whether the file's top-level ``plant:`` key names the physics plant.

    Read from the raw YAML, not through the loader: this runs at collection
    time, and a file that does not validate must still be collected so that
    test_scenario_validates reports it.
    """
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError):
        return False
    plant = raw.get("plant") if isinstance(raw, dict) else None
    return isinstance(plant, dict) and "physics" in plant


RUNS = [
    pytest.param(
        p,
        id=p.stem,
        marks=[pytest.mark.physics] if names_physics_plant(p) else [],
    )
    for p in SCENARIOS
]


def test_scenarios_exist():
    assert SCENARIOS, f"no scenarios in {SCENARIO_DIR}"


@pytest.mark.parametrize("path", SCENARIOS, ids=lambda p: p.stem)
def test_scenario_validates(path):
    load_scenario(path)


@pytest.mark.parametrize("path", RUNS)
def test_scenario_passes(path):
    scenario = load_scenario(path)
    binary = resolve_twin_bin(scenario.twin_dir)
    if not binary.is_file():
        msg = (
            f"twin binary not found at {binary} (set ROBOCAR_TWIN_BIN or build the twin; "
            "ROBOCAR_TWIN_ALLOW_MISSING=1 skips instead)"
        )
        if os.environ.get("ROBOCAR_TWIN_ALLOW_MISSING") == "1":
            pytest.skip(msg)
        pytest.fail(msg)
    result = run_scenario(scenario, [str(binary)])
    failed = [o for o in evaluate(scenario, result.trace) if not o.passed]
    assert not failed, "\n\n".join(f"{o.name} {o.spec}:\n{o.message}" for o in failed)


def test_physics_marker_follows_the_plant_key(tmp_path):
    cases = {
        "plant:\n  physics: {arena: a.yaml, start: {}}\n": True,
        "plant: {physics: null}\n": True,
        "plant: scripted\n": False,
        "seed: 1\n": False,
        "- a list\n": False,
        "plant: [unclosed\n": False,
    }
    for i, (text, want) in enumerate(cases.items()):
        p = tmp_path / f"s{i}.yaml"
        p.write_text(text)
        assert names_physics_plant(p) is want, text
    assert names_physics_plant(tmp_path / "missing.yaml") is False


def test_runs_are_marked_physics_exactly_when_the_scenario_names_the_plant():
    for param in RUNS:
        (path,) = param.values
        marked = any(m.name == "physics" for m in param.marks)
        assert marked == (load_scenario(path).plant is not None), path.name
