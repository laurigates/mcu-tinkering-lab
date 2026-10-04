"""Tests for hardware_nets — a circuit's MCU nets read from the hardware join (#462).

The join (tools/hardware, ADR-021) says which part pin each firmware pin role
is wired to. These tests pin the two things a circuit gains from reading it
rather than transcribing it: a renumbered role moves the wire, and a net the
join declares cannot be left out of the drawing without the render failing.

What is deliberately *not* here is a test that the drawn labels match
pin_config.h. After this change that is true by construction; a test of it
could not fail (#460, ADR-021 § Alternatives considered).
"""

from __future__ import annotations

import dataclasses
import sys
from pathlib import Path as FsPath

import pytest
import schemdraw

sys.path.insert(0, str(FsPath(__file__).parent))

from components import gpio_anchor, max98357a, xiao_esp32s3_sense  # noqa: E402
from hardware import HardwareError, Net, join  # noqa: E402
from hardware_nets import JoinedNets  # noqa: E402

UNIFIED = FsPath(__file__).resolve().parents[2] / "packages/robocar/unified"


@pytest.fixture(scope="module")
def model():
    return join(UNIFIED)


@pytest.fixture
def drawn():
    """A placed XIAO and amplifier, the two ends of robocar-unified's I2S nets."""
    d = schemdraw.Drawing(show=False)
    xiao = d.add(xiao_esp32s3_sense(layout="physical"))
    amp = d.add(max98357a(layout="physical").at((10, 0)))
    return xiao, amp


def _only_i2s(model, **changes):
    """``model`` reduced to its three I2S nets, with ``changes`` applied."""
    nets = tuple(n for n in model.nets if n.part == "amp")
    assert len(nets) == 3, "robocar-unified's join should wire three amp pins"
    return dataclasses.replace(model, nets=nets, **changes)


def test_ends_are_the_pad_the_role_lands_on_and_the_part_pin_the_join_names(
    model, drawn
):
    xiao, amp = drawn
    nets = JoinedNets(_only_i2s(model), xiao, {"amp": amp})
    mcu, part = nets.ends("I2S_BCLK_PIN")
    assert mcu == xiao.absanchors[gpio_anchor(model.roles["I2S_BCLK_PIN"])]
    assert part == amp.absanchors["BCLK"]


def test_a_renumbered_role_moves_the_mcu_end_of_its_wire(model, drawn):
    # The payoff of reading the join: pin_config.h moves BCLK to D0 and the
    # wire follows, with nothing in the circuit edited.
    xiao, amp = drawn
    roles = {**model.roles, "I2S_BCLK_PIN": 1}
    nets = JoinedNets(_only_i2s(model, roles=roles), xiao, {"amp": amp})
    mcu, _ = nets.ends("I2S_BCLK_PIN")
    assert mcu == xiao.absanchors["GPIO1"]
    assert mcu != xiao.absanchors[gpio_anchor(model.roles["I2S_BCLK_PIN"])]


def test_a_role_on_no_header_pad_cannot_be_wired(model, drawn):
    # GPIO42 is the Sense board's mic clock: real, but not on the header.
    xiao, amp = drawn
    roles = {**model.roles, "I2S_BCLK_PIN": 42}
    nets = JoinedNets(_only_i2s(model, roles=roles), xiao, {"amp": amp})
    with pytest.raises(HardwareError, match="not on a header pad"):
        nets.ends("I2S_BCLK_PIN")


def test_a_net_to_a_pad_the_drawn_part_lacks_is_an_error(model, drawn):
    xiao, amp = drawn
    bad = Net(role="I2S_BCLK_PIN", part="amp", pin="BCK", note="")
    others = tuple(n for n in model.nets if n.part == "amp" and n.role != bad.role)
    nets = JoinedNets(
        dataclasses.replace(model, nets=(bad, *others)), xiao, {"amp": amp}
    )
    with pytest.raises(HardwareError, match=r"amp\.BCK"):
        nets.ends("I2S_BCLK_PIN")


def test_a_net_to_an_undrawn_part_is_an_error(model, drawn):
    xiao, _ = drawn
    nets = JoinedNets(_only_i2s(model), xiao, {})
    with pytest.raises(HardwareError, match="'amp'"):
        nets.ends("I2S_BCLK_PIN")


def test_a_role_on_no_net_is_an_error(model, drawn):
    xiao, amp = drawn
    nets = JoinedNets(_only_i2s(model), xiao, {"amp": amp})
    with pytest.raises(HardwareError, match="I2C_SDA_PIN"):
        nets.ends("I2C_SDA_PIN")


def test_drawing_a_net_twice_is_an_error(model, drawn):
    xiao, amp = drawn
    nets = JoinedNets(_only_i2s(model), xiao, {"amp": amp})
    nets.ends("I2S_DIN_PIN")
    with pytest.raises(HardwareError, match="twice"):
        nets.ends("I2S_DIN_PIN")


def test_check_names_every_net_the_drawing_left_out(model, drawn):
    xiao, amp = drawn
    nets = JoinedNets(_only_i2s(model), xiao, {"amp": amp})
    nets.ends("I2S_BCLK_PIN")
    with pytest.raises(HardwareError) as err:
        nets.check_all_drawn()
    assert "I2S_LRCLK_PIN" in str(err.value) and "I2S_DIN_PIN" in str(err.value)
    assert "I2S_BCLK_PIN" not in str(err.value)


def test_check_passes_once_every_net_is_drawn(model, drawn):
    xiao, amp = drawn
    nets = JoinedNets(_only_i2s(model), xiao, {"amp": amp})
    nets.ends("I2S_BCLK_PIN")
    nets.ends("I2S_LRCLK_PIN")
    nets.start("I2S_DIN_PIN")  # hand-drawn far end counts as drawn
    nets.check_all_drawn()


def test_robocar_unified_fails_to_render_a_net_it_does_not_draw(model):
    # A [[nets]] entry added to hardware.toml with no wire in the circuit must
    # stop the render, not ship a drawing that silently lacks it. UART0 TX is
    # a real header pad (D6) that the circuit draws nothing to.
    from render import ROOT, load_circuit

    mod = load_circuit(ROOT / "circuits" / "robocar_unified.py")
    extra = Net(role="UART0_TX_PIN", part="amp", pin="GAIN", note="")
    with pytest.raises(HardwareError, match="UART0_TX_PIN"):
        mod.draw(dataclasses.replace(model, nets=(*model.nets, extra)))
