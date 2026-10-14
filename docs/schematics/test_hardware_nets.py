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
    return dataclasses.replace(model, nets=nets, bus_nets=(), **changes)


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


def test_a_pad_for_an_unknown_role_is_a_hardware_error(model, drawn):
    # The circuit calls pad() directly for placement, before any net lookup,
    # so a mistyped role must fail with the module's error type, not KeyError.
    xiao, amp = drawn
    nets = JoinedNets(_only_i2s(model), xiao, {"amp": amp})
    with pytest.raises(HardwareError, match="I2S_BLCK_PIN: not a pin role"):
        nets.pad("I2S_BLCK_PIN")


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


# --- Fan-out and driver outputs: balancebot (#738) ------------------------------
# balancebot's shared nENABLE is one role wired to three part pins, and its
# coil wires are [[outputs]] pin lists rather than MCU nets. Both used to be
# transcribed into the circuit.

BALANCEBOT = FsPath(__file__).resolve().parents[2] / "packages/robotics/balancebot"


@pytest.fixture(scope="module")
def bb_model():
    return join(BALANCEBOT)


@pytest.fixture
def bb_drawn():
    """A placed XIAO RP2350, both drivers, both steppers and the pull-up."""
    from components import drv8825, stepper_nema17, xiao_rp2350

    d = schemdraw.Drawing(show=False)
    xiao = d.add(xiao_rp2350(layout="physical"))
    pullup = d.add(schemdraw.elements.Resistor().at((5, 0)))
    pullup.absanchors["B"] = pullup.absanchors["start"]
    pullup.absanchors["A"] = pullup.absanchors["end"]
    parts = {
        "drv_left": d.add(drv8825().at((10, 0))),
        "drv_right": d.add(drv8825().at((10, -10))),
        "stepper_left": d.add(stepper_nema17().at((20, 0))),
        "stepper_right": d.add(stepper_nema17().at((20, -10))),
        "pullup": pullup,
    }
    return xiao, parts


def test_fanout_gives_the_pad_and_every_part_pin_of_the_role(bb_model, bb_drawn):
    xiao, parts = bb_drawn
    nets = JoinedNets(bb_model, xiao, parts)
    mcu, ends = nets.fanout("PIN_MOTOR_NENABLE")
    assert mcu == xiao.absanchors[gpio_anchor(bb_model.roles["PIN_MOTOR_NENABLE"])]
    assert ends == {
        "drv_left.nENABLE": parts["drv_left"].absanchors["nENABLE"],
        "drv_right.nENABLE": parts["drv_right"].absanchors["nENABLE"],
        "pullup.B": parts["pullup"].absanchors["B"],
    }


def test_ends_refuses_a_fanned_out_role_and_names_fanout(bb_model, bb_drawn):
    xiao, parts = bb_drawn
    nets = JoinedNets(bb_model, xiao, parts)
    with pytest.raises(HardwareError, match="fanout"):
        nets.ends("PIN_MOTOR_NENABLE")


def test_fanout_to_an_undrawn_part_is_an_error(bb_model, bb_drawn):
    xiao, parts = bb_drawn
    del parts["pullup"]
    nets = JoinedNets(bb_model, xiao, parts)
    with pytest.raises(HardwareError, match="'pullup'"):
        nets.fanout("PIN_MOTOR_NENABLE")


def test_a_fanout_counts_as_drawn_and_cannot_be_drawn_twice(bb_model, bb_drawn):
    xiao, parts = bb_drawn
    nets = JoinedNets(bb_model, xiao, parts)
    nets.fanout("PIN_MOTOR_NENABLE")
    with pytest.raises(HardwareError, match="twice"):
        nets.fanout("PIN_MOTOR_NENABLE")
    with pytest.raises(HardwareError) as err:
        nets.check_all_drawn()
    assert "PIN_MOTOR_NENABLE" not in str(err.value)


def test_output_ends_pairs_each_driver_pin_with_the_same_pin_on_its_load(
    bb_model, bb_drawn
):
    xiao, parts = bb_drawn
    nets = JoinedNets(bb_model, xiao, parts)
    pairs = nets.output_ends()
    assert len(pairs) == 8, "two drivers x four coil pins"
    assert (
        parts["drv_left"].absanchors["A1"],
        parts["stepper_left"].absanchors["A1"],
    ) in pairs
    assert (
        parts["drv_right"].absanchors["B2"],
        parts["stepper_right"].absanchors["B2"],
    ) in pairs


def test_output_ends_to_an_undrawn_load_is_an_error(bb_model, bb_drawn):
    xiao, parts = bb_drawn
    del parts["stepper_right"]
    nets = JoinedNets(bb_model, xiao, parts)
    with pytest.raises(HardwareError, match="'stepper_right'"):
        nets.output_ends()


def test_balancebot_fails_to_render_a_net_it_does_not_draw(bb_model):
    from render import ROOT, load_circuit

    mod = load_circuit(ROOT / "circuits" / "balancebot.py")
    # D1 (GPIO27) is a real header pad balancebot leaves free.
    roles = {**bb_model.roles, "PIN_SPARE": 27}
    extra = Net(role="PIN_SPARE", part="imu", pin="SDA", note="")
    with pytest.raises(HardwareError, match="PIN_SPARE"):
        mod.draw(
            dataclasses.replace(bb_model, roles=roles, nets=(*bb_model.nets, extra))
        )


def test_a_renumbered_balancebot_role_moves_its_wire(bb_model):
    # DIR_RIGHT moved from D6 to D1 in pin_config.h: the render follows the
    # header with nothing in the circuit edited.
    from render import ROOT, load_circuit
    from routing import Path as Wire

    mod = load_circuit(ROOT / "circuits" / "balancebot.py")

    def wire_ends(model):
        d = mod.draw(model)
        return {
            (round(e.polyline[0][0], 3), round(e.polyline[0][1], 3))
            for e in d.elements
            if isinstance(e, Wire)
        }

    moved = wire_ends(
        dataclasses.replace(bb_model, roles={**bb_model.roles, "PIN_DIR_RIGHT": 27})
    )
    assert moved != wire_ends(bb_model)


def test_balancebot_tags_the_driver_straps_from_the_rails(bb_model):
    # nSLEEP/nRESET/M0/M1 on 3V3 per hardware.toml (#738: the old drawing
    # wired a VDD pin the Pololu-layout carrier does not have). Taking M1 off
    # every rail must fail the render rather than leave a stale tag.
    from hardware import Endpoint
    from render import ROOT, load_circuit

    mod = load_circuit(ROOT / "circuits" / "balancebot.py")
    gone = Endpoint("drv_left", "M1")
    rails = tuple(
        dataclasses.replace(r, loads=tuple(e for e in r.loads if e != gone))
        for r in bb_model.rails
    )
    with pytest.raises(HardwareError, match=r"drv_left\.M1"):
        mod.draw(dataclasses.replace(bb_model, rails=rails))


# --- Mux channel buses: robocar-unified (#768) ---------------------------------
# The TCA9548A's downstream SDn/SCn pairs used to be hand-drawn by channel macro;
# [[bus_nets]] now says which part each channel reaches.


@pytest.fixture
def mux_drawn():
    """A placed XIAO, the TCA9548A, the PCA9685 and the OLED."""
    from components import pca9685, ssd1306_oled, tca9548a

    d = schemdraw.Drawing(show=False)
    xiao = d.add(xiao_esp32s3_sense(layout="physical"))
    parts = {
        "mux": d.add(tca9548a(layout="physical").at((10, 0))),
        "pwm": d.add(pca9685(layout="physical").at((30, 0))),
        "oled": d.add(ssd1306_oled().at((10, -20))),
    }
    return xiao, parts


def test_bus_ends_pair_each_channel_pad_with_the_loads_sda_and_scl(model, mux_drawn):
    xiao, parts = mux_drawn
    nets = JoinedNets(model, xiao, parts)
    n = model.bus_channels["I2C_BUS_CHANNEL_OLED"]
    mux, oled = parts["mux"].absanchors, parts["oled"].absanchors
    assert nets.bus_ends("I2C_BUS_CHANNEL_OLED") == [
        (mux[f"SD{n}"], oled["SDA"]),
        (mux[f"SC{n}"], oled["SCL"]),
    ]


def test_a_renumbered_mux_channel_moves_the_mux_end(model, mux_drawn):
    xiao, parts = mux_drawn
    buses = tuple(
        dataclasses.replace(b, channel=5) if b.role == "I2C_BUS_CHANNEL_OLED" else b
        for b in model.bus_nets
    )
    nets = JoinedNets(dataclasses.replace(model, bus_nets=buses), xiao, parts)
    (sda, _), (scl, _) = nets.bus_ends("I2C_BUS_CHANNEL_OLED")
    assert (sda, scl) == (
        parts["mux"].absanchors["SD5"],
        parts["mux"].absanchors["SC5"],
    )


def test_bus_ends_takes_the_drawings_anchor_for_a_repeated_pad(model, mux_drawn):
    # The PCA9685 has SDA on both side headers; the drawing picks the right one.
    xiao, parts = mux_drawn
    nets = JoinedNets(model, xiao, parts)
    anchors = {"pwm.SDA": "SDA.R4", "pwm.SCL": "SCL.R3"}
    (_, sda), (_, scl) = nets.bus_ends("I2C_BUS_CHANNEL_PCA9685", anchors)
    assert (sda, scl) == (
        parts["pwm"].absanchors["SDA.R4"],
        parts["pwm"].absanchors["SCL.R3"],
    )


def test_an_anchor_on_another_pin_is_an_error(model, mux_drawn):
    # SCL.R3 for the SDA line would wire channel 0's data to the clock pad.
    xiao, parts = mux_drawn
    nets = JoinedNets(model, xiao, parts)
    with pytest.raises(HardwareError, match=r"SCL\.R3.*pwm\.SDA"):
        nets.bus_ends(
            "I2C_BUS_CHANNEL_PCA9685", {"pwm.SDA": "SCL.R3", "pwm.SCL": "SCL.R3"}
        )


def test_an_anchor_for_a_pin_off_the_bus_is_an_error(model, mux_drawn):
    xiao, parts = mux_drawn
    nets = JoinedNets(model, xiao, parts)
    with pytest.raises(HardwareError, match=r"oled\.SDA"):
        nets.bus_ends("I2C_BUS_CHANNEL_PCA9685", {"oled.SDA": "SDA"})


def test_a_bus_role_with_no_entry_is_an_error(model, mux_drawn):
    xiao, parts = mux_drawn
    nets = JoinedNets(model, xiao, parts)
    with pytest.raises(HardwareError, match="I2C_BUS_CHANNEL_CAMERA.*bus_nets"):
        nets.bus_ends("I2C_BUS_CHANNEL_CAMERA")


def test_a_bus_drawn_twice_is_an_error(model, mux_drawn):
    xiao, parts = mux_drawn
    nets = JoinedNets(model, xiao, parts)
    nets.bus_ends("I2C_BUS_CHANNEL_OLED")
    with pytest.raises(HardwareError, match="twice"):
        nets.bus_ends("I2C_BUS_CHANNEL_OLED")


def test_check_names_a_bus_the_drawing_left_out(model, mux_drawn):
    xiao, parts = mux_drawn
    only = tuple(b for b in model.bus_nets if b.part in ("oled", "pwm"))
    nets = JoinedNets(dataclasses.replace(model, nets=(), bus_nets=only), xiao, parts)
    nets.bus_ends("I2C_BUS_CHANNEL_OLED")
    with pytest.raises(HardwareError) as err:
        nets.check_all_drawn()
    assert "I2C_BUS_CHANNEL_PCA9685 -> pwm" in str(err.value)
    assert "OLED" not in str(err.value)


def test_robocar_unified_fails_to_render_a_bus_it_does_not_draw(model):
    # A [[bus_nets]] entry for a channel the circuit never draws must stop the
    # render, as an MCU net does.
    from hardware import BusNet
    from render import ROOT, load_circuit

    mod = load_circuit(ROOT / "circuits" / "robocar_unified.py")
    extra = BusNet("I2C_BUS_CHANNEL_SPARE", 3, "mux", "oled", "")
    with pytest.raises(HardwareError, match="I2C_BUS_CHANNEL_SPARE"):
        mod.draw(dataclasses.replace(model, bus_nets=(*model.bus_nets, extra)))
