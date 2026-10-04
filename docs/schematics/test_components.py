"""Tests for component factories whose drawing carries a claim about hardware.

Most factories are pin lists the routing suite already exercises. The PDM
microphone is different: it is drawn to say "this is on the module, solder
nothing", and that claim lives in its styling, so the styling is pinned here
along with its presence in the circuit that ships it (issue #486).

The physical symbols (#495) claim the board's own pad order, so each is
checked against the board reference it reads.
"""

import re
import sys
from pathlib import Path as FsPath

import pytest
import schemdraw
import schemdraw.elements as elm

sys.path.insert(0, str(FsPath(__file__).parent))

from components import (  # noqa: E402
    PITCH,
    max98357a,
    pca9685,
    pdm_microphone,
    tb6612fng,
    tca9548a,
    xiao_esp32s3_sense,
)
from hardware import board_layout, join  # noqa: E402

UNIFIED = FsPath(__file__).resolve().parents[2] / "packages/robocar/unified"


def _mic(clk: str = "GPIO42", data: str = "GPIO41"):
    return pdm_microphone(clk=clk, data=data)


def _svg_text(svg: bytes) -> list[str]:
    return re.findall(r"<tspan[^>]*>([^<]*)</tspan>", svg.decode())


def test_pdm_microphone_exposes_clk_and_data_by_name():
    names = {p.name for p in _mic()._userparams["pins"]}
    assert names == {"CLK", "DATA"}


def test_pdm_microphone_prints_the_pins_it_is_given():
    # The GPIOs come from the hardware join in the circuit (#462); the factory
    # only places them. Distinct values catch a CLK/DATA swap.
    pins = {p.name: p.pin for p in _mic("CLKPAD", "DATAPAD")._userparams["pins"]}
    assert pins == {"CLK": "CLKPAD", "DATA": "DATAPAD"}


def test_pdm_microphone_is_dashed_so_it_reads_as_on_module():
    # A solid outline is what every breakout the builder solders looks like.
    assert _mic()._userparams.get("ls") == "--"


def test_robocar_unified_draws_the_onboard_microphone(real_circuit):
    circuit = real_circuit("robocar_unified")
    mics = [
        e
        for e in circuit.drawing.elements
        if isinstance(e, elm.Ic) and {"CLK", "DATA"} <= set(e.anchors)
    ]
    assert len(mics) == 1, "robocar_unified must draw exactly one PDM microphone"
    mic = mics[0]
    assert mic._userparams.get("ls") == "--"

    # CLK must sit above DATA as drawn, or the two leads into the XIAO cross.
    # Checked on the rendered geometry, not on the factory's pin-list order,
    # which only implies it through schemdraw's side="R" layout.
    clk, data = mic.absanchors["CLK"], mic.absanchors["DATA"]
    assert clk[1] > data[1], "CLK must be drawn above DATA"

    # The leads carry the "nothing to wire" claim as much as the outline does:
    # a solid black lead reads as a net to route.
    leads = [
        e
        for e in circuit.drawing.elements
        if isinstance(e, elm.Wire)
        and tuple(e.absanchors["start"]) in {tuple(clk), tuple(data)}
    ]
    assert len(leads) == 2, "expected one lead from each mic pin"
    for lead in leads:
        assert lead._userparams.get("ls") == "--"
        assert lead._userparams.get("color") == "gray"

    # Each pin is labelled with its role's GPIO from the join. The GPIO value
    # is read, not restated, but which role feeds which pin is still chosen by
    # hand in the circuit: the mic roles are [[undrawn]], so no [[nets]] entry
    # binds them and check_all_drawn() cannot see a CLK/DATA role swap. This
    # pins that choice, and fails on exactly that swap.
    roles = join(UNIFIED).roles
    labels = {p.name: p.pin for p in mic._userparams["pins"]}
    assert labels == {
        "CLK": f"GPIO{roles['MIC_PDM_CLK_PIN']}",
        "DATA": f"GPIO{roles['MIC_PDM_DATA_PIN']}",
    }

    text = _svg_text(circuit.svg)
    assert set(labels.values()) <= set(text)
    # The block must say it needs no wiring, or the dashes are left to guesswork.
    assert any("no wiring" in t for t in text)


# --- Physical module layouts (ADR-023 stage 6, #495) ------------------------

# Factory -> the board reference its physical symbol must be read from.
PHYSICAL = {
    "xiao_esp32s3_sense": (xiao_esp32s3_sense, "xiao-esp32s3"),
    "tb6612fng": (tb6612fng, "sparkfun-tb6612fng"),
    "pca9685": (pca9685, "adafruit-pca9685"),
    "tca9548a": (tca9548a, "adafruit-tca9548a"),
    "max98357a": (max98357a, "adafruit-max98357a"),
}


def _placed(ic):
    """Add ``ic`` to a throwaway drawing so its absolute anchors exist."""
    schemdraw.Drawing(show=False).add(ic)
    return ic


def _anchor_for(factory_name):
    # The XIAO wires by firmware name (GPIO5); every breakout by the layout's
    # own anchor (SDA, GND.L3).
    if factory_name == "xiao_esp32s3_sense":
        return lambda p: f"GPIO{p.gpio}" if p.gpio is not None else p.name
    return lambda p: p.anchor


@pytest.mark.parametrize("factory_name", sorted(PHYSICAL))
def test_physical_symbol_draws_every_pad_on_its_edge_in_board_order(factory_name):
    # The expectation is the board reference itself, so this checks the
    # drawing against the reference; the reference is checked against the
    # vendor's board file by the page that cites it.
    factory, slug = PHYSICAL[factory_name]
    layout = board_layout(slug)
    ic = _placed(factory(layout="physical"))
    anchor = _anchor_for(factory_name)

    points = {pad: ic.absanchors[anchor(pad)] for pad in layout.pads}
    xs = [x for x, _ in points.values()]
    ys = [y for _, y in points.values()]
    edge = {"L": min(xs), "R": max(xs), "T": max(ys), "B": min(ys)}

    for side in "LRTB":
        pads = layout.side(side)
        if not pads:
            continue
        coords = [points[p] for p in pads]
        if side in "LR":
            assert all(abs(x - edge[side]) < 1e-9 for x, _ in coords), side
            along = [-y for _, y in coords]  # Pos 1 at the top
        else:
            assert all(abs(y - edge[side]) < 1e-9 for _, y in coords), side
            along = [x for x, _ in coords]  # Pos 1 at the left
        steps = [b - a for a, b in zip(along, along[1:])]
        assert all(abs(s - PITCH) < 1e-9 for s in steps), (side, steps)

    # Nothing drawn that the board does not have.
    assert len(ic._userparams["pins"]) == len(layout.pads)


def test_physical_xiao_is_fourteen_pads_with_power_on_the_right():
    # The issue's done-when, stated directly: 7 per side, power at the top of
    # the right edge beside USB-C, D6/D7 present.
    ic = _placed(xiao_esp32s3_sense(layout="physical"))
    pins = ic._userparams["pins"]
    left = [p for p in pins if p.side == "L"]
    right = [p for p in pins if p.side == "R"]
    assert (len(left), len(right)) == (7, 7)

    def top_to_bottom(side_pins):
        return [
            p.name
            for p in sorted(side_pins, key=lambda p: -ic.absanchors[p.anchorname][1])
        ]

    assert top_to_bottom(right)[:3] == ["5V", "GND", "3V3"]
    assert top_to_bottom(left)[0] == "GPIO1"  # D0, nearest USB-C
    assert {p.pin for p in pins if p.pin} >= {"D6", "D7"}
    assert {"GPIO43", "GPIO44"} <= set(ic.absanchors)


def test_repeated_pad_names_get_distinct_anchors():
    # Three GND pads on the TB6612FNG: one shared anchor would wire all three
    # to whichever was drawn last.
    ic = _placed(tb6612fng(layout="physical"))
    gnds = {tuple(ic.absanchors[a]) for a in ("GND.L3", "GND.L8", "GND.R8")}
    assert len(gnds) == 3
    assert "GND" not in ic.absanchors


@pytest.mark.parametrize("factory_name", sorted(PHYSICAL))
def test_unknown_layout_is_rejected(factory_name):
    factory, _ = PHYSICAL[factory_name]
    with pytest.raises(ValueError, match="layout"):
        factory(layout="pretty")


def test_default_layout_is_still_the_schematic_symbol():
    # Circuits that have not opted in (gamepad_synth's amplifier) must render
    # exactly as before.
    names = {p.name for p in max98357a()._userparams["pins"]}
    assert names == {"DIN", "LRC", "BCLK", "GAIN", "SD", "GND", "VIN", "OUT-", "OUT+"}


def test_robocar_unified_draws_every_known_board_physically(real_circuit):
    circuit = real_circuit("robocar_unified")
    ics = [e for e in circuit.drawing.elements if isinstance(e, elm.Ic)]
    for factory_name, (_, slug) in PHYSICAL.items():
        layout = board_layout(slug)
        anchor = _anchor_for(factory_name)
        wanted = {anchor(p) for p in layout.pads}
        matches = [ic for ic in ics if wanted <= set(ic.anchors)]
        assert len(matches) == 1, (
            f"{slug}: expected one physical symbol, found {len(matches)}"
        )
        assert len(matches[0]._userparams["pins"]) == len(layout.pads), slug
