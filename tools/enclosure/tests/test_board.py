"""Reading a committed carrier board: placement transform and STEP heights."""

import shutil

import pytest
from enclosure import board
from kicad.paths import REPO_ROOT

BALANCEBOT = REPO_ROOT / "packages" / "robotics" / "balancebot"


def test_rotate_matches_kicad_convention():
    # KiCad (y down): a pad at local (1, 0) on a footprint at 90 degrees
    # ends up at (0, -1), i.e. above the anchor on screen.
    x, y = board._rotate(1.0, 0.0, 90.0)
    assert (round(x, 9), round(y, 9)) == (0.0, -1.0)
    x, y = board._rotate(0.0, 2.0, 180.0)
    assert (round(x, 9), round(y, 9)) == (0.0, -2.0)


def test_balancebot_holes_match_board_toml():
    _, holes, thickness = board.read_board(BALANCEBOT)
    assert thickness == pytest.approx(1.6)
    got = sorted((h.ref, h.x, h.y, h.drill) for h in holes)
    assert got == [
        ("H1", 4.0, 4.0, 3.2),
        ("H2", 86.0, 4.0, 3.2),
        ("H3", 4.0, 58.0, 3.2),
        ("H4", 86.0, 58.0, 3.2),
    ]


def test_every_courtyard_is_on_the_board():
    parts, _, _ = board.read_board(BALANCEBOT)
    for p in parts:
        c = p.courtyard
        assert -1 <= c.x0 < c.x1 <= 91 and -1 <= c.y0 < c.y1 <= 63, p.ref


@pytest.mark.skipif(not board.KICAD_CLI.exists(), reason="needs KiCad's kicad-cli")
def test_every_step_model_lands_on_a_courtyard(tmp_path):
    """The placement transform agrees with KiCad's own: step_heights raises if
    any 3D model's centre falls outside every courtyard."""
    step = board.export_step(BALANCEBOT, tmp_path / "b.step")
    parts, _, _ = board.read_board(BALANCEBOT)
    board.step_heights(step, parts)
    j5 = next(p for p in parts if p.ref == "J5")
    assert j5.above == pytest.approx(7.09, abs=0.2)  # JST XH vertical
    assert j5.below > 1.0  # its pins come through the board


@pytest.mark.skipif(not board.KICAD_CLI.exists(), reason="needs KiCad's kicad-cli")
def test_missing_height_is_refused(tmp_path):
    proj = tmp_path / "proj"
    shutil.copytree(BALANCEBOT / "hardware", proj / "hardware")
    toml = proj / "hardware" / "enclosure" / "enclosure.toml"
    text = toml.read_text().replace("A1 = {", "#A1 = {")
    toml.write_text(text)
    step = board.export_step(BALANCEBOT, tmp_path / "b.step")
    with pytest.raises(board.EnclosureError, match=r"no height for \['A1'\]"):
        board.load_pieces(proj, step)
