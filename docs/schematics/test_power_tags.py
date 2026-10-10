"""robocar_unified's power-tag rails come from hardware.toml [[rails]] (#679).

The circuit keeps the drawing (where each tag sits, which way it leads); the
join supplies the fact (which rail the pin is on). A pin moved to another rail
in hardware.toml must therefore move its tag label, and a rail load the
drawing leaves untagged must fail the render.
"""

from __future__ import annotations

import dataclasses
import sys
from pathlib import Path as FsPath

import pytest
import schemdraw.elements as elm

sys.path.insert(0, str(FsPath(__file__).parent))
sys.path.insert(0, str(FsPath(__file__).resolve().parents[2] / "tools"))

from hardware import Endpoint, HardwareError, join
from hardware_nets import JoinedNets

UNIFIED = FsPath(__file__).resolve().parents[2] / "packages/robocar/unified"


@pytest.fixture(scope="module")
def model():
    return join(UNIFIED)


@pytest.fixture(scope="module")
def circuit():
    from render import ROOT, load_circuit

    return load_circuit(ROOT / "circuits" / "robocar_unified.py")


def _move_rail(model, endpoint: Endpoint, to: str):
    """``model`` with ``endpoint`` taken off whatever rail loads it and put on ``to``."""
    rails = []
    for rail in model.rails:
        loads = tuple(e for e in rail.loads if e != endpoint)
        if rail.name == to:
            loads = (*loads, endpoint)
        rails.append(dataclasses.replace(rail, loads=loads))
    return dataclasses.replace(model, rails=tuple(rails))


def _power_labels(drawing) -> dict[tuple[float, float], str]:
    """Every power symbol's label, keyed by where it is drawn."""
    return {
        (round(e.start.x, 4), round(e.start.y, 4)): e._userlabels[0].label
        for e in drawing.elements
        if isinstance(e, elm.Vdd)
    }


def test_a_pin_moved_to_another_rail_moves_its_tag_label(model, circuit):
    before = _power_labels(circuit.draw(model))
    moved = _move_rail(model, Endpoint("pwm", "VCC"), "5V")
    after = _power_labels(circuit.draw(moved))
    assert before.keys() == after.keys()
    changed = {k: (before[k], after[k]) for k in before if before[k] != after[k]}
    assert list(changed.values()) == [("+3V3", "+5V")]


def test_a_rail_load_with_no_tag_fails_the_render(model, circuit):
    # The amplifier is drawn, and has no tag on GAIN.
    extra = _move_rail(model, Endpoint("amp", "GAIN"), "3V3")
    with pytest.raises(HardwareError, match="amp.GAIN"):
        circuit.draw(extra)


def test_rail_of_a_pin_on_no_rail_is_an_error(model):
    nets = JoinedNets(model, None, {})
    with pytest.raises(HardwareError, match="BCLK"):
        nets.rail("amp", "BCLK")


def _cap_mismatches(model, caps) -> list[str]:
    """Each suggested cap whose rail label disagrees with the join's."""
    nets = JoinedNets(model, None, {})
    by_name = {p.name: pid for pid, p in model.parts.items()}
    bad = []
    for cap in caps:
        name, pin = cap.part_pin.rsplit(" ", 1)
        rail = nets.rail(by_name[name], pin)
        if rail != cap.rail:
            bad.append(f"{cap.ref} {cap.part_pin}: drawn {cap.rail}, join {rail}")
    return bad


def test_every_suggested_cap_sits_on_the_rail_the_join_gives_its_pin(model, circuit):
    assert _cap_mismatches(model, circuit.SUGGESTED_CAPS) == []


def test_a_suggested_cap_on_a_moved_pin_is_reported(model, circuit):
    moved = _move_rail(model, Endpoint("pwm", "VCC"), "5V")
    assert _cap_mismatches(moved, circuit.SUGGESTED_CAPS) == [
        "C4 PCA9685 VCC: drawn +3V3, join +5V"
    ]
