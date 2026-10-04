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


def _svg_text(svg: bytes) -> list[str]:
    return re.findall(r"<tspan[^>]*>([^<]*)</tspan>", svg.decode())


def test_pdm_microphone_exposes_clk_and_data_by_name():
    # Bottom-to-top: CLK is drawn above DATA so the leads into the XIAO nest.
    names = [p.name for p in pdm_microphone()._userparams["pins"]]
    assert names == ["DATA", "CLK"]


def test_pdm_microphone_pins_carry_the_firmware_gpio_numbers():
    # pin_config.h: MIC_PDM_CLK_PIN GPIO_NUM_42, MIC_PDM_DATA_PIN GPIO_NUM_41.
    pins = {p.name: p.pin for p in pdm_microphone()._userparams["pins"]}
    assert pins == {"CLK": "GPIO42", "DATA": "GPIO41"}


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
    assert mics[0]._userparams.get("ls") == "--"

    text = _svg_text(circuit.svg)
    assert "GPIO42" in text and "GPIO41" in text
    # The block must say it needs no wiring, or the dashes are left to guesswork.
    assert any("no wiring" in t for t in text)
