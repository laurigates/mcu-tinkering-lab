"""One render end to end; skipped where VTK cannot open an off-screen window."""

from pathlib import Path

import pytest

pv = pytest.importorskip("pyvista")

DESIGN = """
from build123d import Box, Pos, Rot
from mechanism.check import Body

def printed_parts():
    return {"arm": Pos(15, 0, 0) * Box(30, 4, 4)}

def assembly(pan, tilt):
    arm = Rot(0, 0, pan) * printed_parts()["arm"]
    return [Body("arm", arm, "moving", printed=True), Body("post", Pos(0, 25, 0) * Box(4, 4, 10), "base")]
"""


def test_every_view_is_written(tmp_path: Path):
    try:
        p = pv.Plotter(off_screen=True, window_size=(64, 64))
        p.screenshot(str(tmp_path / "probe.png"))
        p.close()
    except Exception as e:  # no libOSMesa / EGL
        pytest.skip(f"no off-screen VTK: {e}")
    from mechanism import showcase

    (tmp_path / "design.py").write_text(DESIGN)
    assert showcase.main([str(tmp_path)]) == 0
    names = {f.name for f in (tmp_path / "output" / "showcase").iterdir()}
    assert {"assembled-front-left.png", "assembled-corner-pan60-tilt30.png"} <= names
    assert {"exploded-front-left.png", "exploded-back-right.png", "parts.png"} <= names
