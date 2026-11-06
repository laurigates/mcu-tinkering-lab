"""Every scenarios/*.yaml: validated always, run through the twin binary.

The binary is $ROBOCAR_TWIN_BIN or <twin dir>/build/robocar-twin. A missing
binary fails the run unless ROBOCAR_TWIN_ALLOW_MISSING=1, so a CI job that
forgot to build the twin cannot pass by skipping every scenario.
"""

from __future__ import annotations

import os

import pytest

from robocar_twin.assertions import evaluate
from robocar_twin.runner import default_twin_dir, resolve_twin_bin, run_scenario
from robocar_twin.scenario import load_scenario

SCENARIO_DIR = default_twin_dir() / "scenarios"
SCENARIOS = sorted(SCENARIO_DIR.glob("*.yaml"))


def test_scenarios_exist():
    assert SCENARIOS, f"no scenarios in {SCENARIO_DIR}"


@pytest.mark.parametrize("path", SCENARIOS, ids=lambda p: p.stem)
def test_scenario_validates(path):
    load_scenario(path)


@pytest.mark.parametrize("path", SCENARIOS, ids=lambda p: p.stem)
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
