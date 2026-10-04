"""Tests for component factories whose drawing carries a claim about hardware.

Most factories are pin lists the routing suite already exercises. The PDM
microphone is different: it is drawn to say "this is on the module, solder
nothing", and that claim lives in its styling, so the styling is pinned here
along with its presence in the circuit that ships it (issue #486).
"""

import re
import sys
from pathlib import Path as FsPath

import schemdraw.elements as elm

sys.path.insert(0, str(FsPath(__file__).parent))

from components import pdm_microphone  # noqa: E402

PIN_CONFIG = (
    FsPath(__file__).resolve().parents[2] / "packages/robocar/unified/main/pin_config.h"
)


def _firmware_gpio(macro: str) -> str:
    """Read ``#define <macro> GPIO_NUM_<n>`` from robocar-unified's pin_config.h."""
    match = re.search(
        rf"^#define\s+{macro}\s+GPIO_NUM_(\d+)\b", PIN_CONFIG.read_text(), re.M
    )
    assert match, f"{macro} not found in {PIN_CONFIG}"
    return f"GPIO{match.group(1)}"


def _svg_text(svg: bytes) -> list[str]:
    return re.findall(r"<tspan[^>]*>([^<]*)</tspan>", svg.decode())


def test_pdm_microphone_exposes_clk_and_data_by_name():
    names = {p.name for p in pdm_microphone()._userparams["pins"]}
    assert names == {"CLK", "DATA"}


def test_pdm_microphone_pins_carry_the_firmware_gpio_numbers():
    # Read from pin_config.h rather than restated, so a renumbered mic pin in
    # the firmware fails here instead of leaving the schematic quietly wrong.
    pins = {p.name: p.pin for p in pdm_microphone()._userparams["pins"]}
    assert pins == {
        "CLK": _firmware_gpio("MIC_PDM_CLK_PIN"),
        "DATA": _firmware_gpio("MIC_PDM_DATA_PIN"),
    }


def test_pdm_microphone_is_dashed_so_it_reads_as_on_module():
    # A solid outline is what every breakout the builder solders looks like.
    assert pdm_microphone()._userparams.get("ls") == "--"


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

    text = _svg_text(circuit.svg)
    assert "GPIO42" in text and "GPIO41" in text
    # The block must say it needs no wiring, or the dashes are left to guesswork.
    assert any("no wiring" in t for t in text)
