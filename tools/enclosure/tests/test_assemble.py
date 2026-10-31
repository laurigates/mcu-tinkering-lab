"""Assembly gates on synthetic pieces: can the board go in, can the screws be
driven. The box of 2026-10-23 is the control: printed, it could not take its
board, because the lid bosses stood over all four corners."""

import pytest
from build123d import Align, Box, Cylinder, Pos
from enclosure import assemble, case
from test_case import frame, piece

A = (Align.MIN,) * 3
C = (Align.CENTER, Align.CENTER, Align.MIN)


def slab(f, part_h: float = 10.0):
    """The piece's board with U1 as a solid block on it."""
    r = f.piece.region
    out = Pos(f.off, f.off, f.z_board) * Box(
        r.width, r.height, f.piece.thickness, align=A
    )
    for q in f.piece.parts:
        x, y, w, h = f.rect(q.courtyard)
        out += Pos(x, y, f.z_top) * Box(w, h, part_h, align=A)
    return out


def collider(fixed, moving) -> assemble.Collider:
    return assemble.Collider(assemble.mesh_of(fixed), assemble.mesh_of(moving))


def test_voxel_volume_matches_the_solid():
    cube = Pos(1.3, 2.7, 0.4) * Box(10, 6, 4, align=A)
    g = assemble.voxelize(assemble.mesh_of(cube))
    assert g.occ.sum() * g.pitch**3 == pytest.approx(240, rel=0.03)


def test_a_point_inside_hits_and_one_outside_does_not():
    g = assemble.voxelize(assemble.mesh_of(Box(10, 10, 10, align=A)))
    assert g.hits([[5, 5, 5], [11, 5, 5], [-1, 5, 5]]).tolist() == [True, False, False]


def test_board_lifts_straight_out_of_the_tray():
    f = frame(piece())
    name, p = assemble.removal(collider(case.tray(f), slab(f)))
    assert p.found and name == "lift"


def test_the_printed_box_cannot_take_its_board():
    # Control: the box as printed on 2026-10-23. Every strategy must fail.
    f = frame(piece())
    base, _ = case.box(f)
    name, p = assemble.removal(collider(base, slab(f)))
    assert not p.found


def _one_end_box(f, slot: float):
    """Walls, corner pads under the board, and lid bosses at the west end only,
    starting `slot` mm above the board."""
    wall_top = f.z_top + 15
    body = case._shell(f, wall_top)
    r = case.LID_BOSS / 2
    c = f.p.wall + r - 0.5
    for y in (c, f.width - c):
        body += Pos(c, y, f.z_top + slot) * Cylinder(
            r, wall_top - f.z_top - slot, align=C
        )
    for x in (f.off + 6, f.length - f.off - 6):
        for y in (f.off + 6, f.width - f.off - 6):
            body += Pos(x, y, 0) * Cylinder(2.25, f.z_board, align=C)
    return body


def test_a_board_that_needs_a_tilt_passes():
    # The board's west end tucks under the remaining bosses: no straight lift,
    # but a tilt about y while lifting frees it, and the fine check agrees.
    f = frame(piece(parts=[], holes=[]))
    col = collider(_one_end_box(f, 3.1), slab(f))
    assert not assemble.plan(col, axes=assemble.STRATEGIES["lift"]).found
    name, p = assemble.removal(col)
    assert p.found and name == "tilt-about-y"
    assert any(abs(q[4]) > 10 for q in p.path)


def test_no_tilt_gets_under_a_boss_half_a_millimetre_up():
    f = frame(piece(parts=[], holes=[]))
    _, p = assemble.removal(collider(_one_end_box(f, 0.5), slab(f)))
    assert not p.found


def test_lid_bosses_block_the_board_screws_under_them():
    f = frame(piece())
    base, _ = case.box(f)
    assert case.blocked_screws(f, base) == ["H1", "H2"]
    assert case.blocked_screws(f, case.tray(f)) == []
