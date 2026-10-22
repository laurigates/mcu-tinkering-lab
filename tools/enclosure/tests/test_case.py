"""Case geometry on synthetic pieces: no KiCad, no board files."""

from pathlib import Path

import pytest
from build123d import Align, Box, Pos, Vector
from enclosure import case
from enclosure.board import Hole, Part, Piece, Rect

A = (Align.MIN,) * 3


def piece(parts=None, openings=(), holes=None, lid_windows=()) -> Piece:
    return Piece(
        project=Path("synthetic"),
        name="p",
        region=Rect(0, 0, 60, 40),
        quantity=1,
        thickness=1.6,
        holes=holes
        if holes is not None
        else [Hole("H1", 4, 4, 3.2), Hole("H2", 56, 36, 3.2)],
        parts=parts
        if parts is not None
        else [
            Part(
                "U1",
                "fp",
                "top",
                Rect(20, 10, 40, 30),
                above=10.0,
                height_source="test",
            )
        ],
        openings=list(openings),
        lid_windows=list(lid_windows),
    )


def frame(p: Piece) -> case.Frame:
    return case.Frame(p, case.Params())


def test_tray_outline_is_region_plus_wall_and_gap():
    f = frame(piece())
    bb = case.tray(f).bounding_box()
    assert bb.size.X == pytest.approx(60 + 2 * (2.0 + 0.5))
    assert bb.size.Y == pytest.approx(40 + 2 * (2.0 + 0.5))
    assert bb.size.Z == pytest.approx(f.z_top + 3.0)


def test_board_y_is_flipped_into_case_y():
    f = frame(piece())
    # Board (4, 4) is near the top-left corner; in case coordinates (y up)
    # that is near the top of the case, not the bottom.
    x, y = f.xy(4, 4)
    assert x == pytest.approx(4 + 2.5)
    assert y == pytest.approx(40 - 4 + 2.5)


def test_bolt_bore_goes_through_each_standoff():
    f = frame(piece())
    tray = case.tray(f)
    for h in f.piece.holes:
        x, y = f.xy(h.x, h.y)
        assert not tray.is_inside(Vector(x, y, 1.0)), "bore missing through the floor"
        # The boss wall beside the bore is solid.
        assert tray.is_inside(Vector(x + 2.5, y, f.z_board - 0.5))


def test_standoff_clears_the_deepest_back_side_part():
    deep = Part("J2", "fp", "bottom", Rect(10, 30, 20, 38), above=0.0, below=6.0)
    f = frame(piece(parts=[deep], openings=["J2"]))
    # header 6 mm + mated plug 6 mm + 1.5 mm clearance
    assert f.standoff == pytest.approx(13.5)


def test_opening_lands_on_the_nearest_edge():
    edge = Part(
        "J1", "fp", "top", Rect(55, 15, 59.5, 25), above=8.0, height_source="test"
    )
    f = frame(piece(parts=[edge], openings=["J1"]))
    tray = case.tray(f)
    _, y = f.xy(0, 20)
    east_wall_x = f.length - 1.0
    west_wall_x = 1.0
    z = f.z_top + 1.0
    assert not tray.is_inside(Vector(east_wall_x, y, z)), "no opening in the east wall"
    assert tray.is_inside(Vector(west_wall_x, y, z)), "opening cut in the wrong wall"


def test_lid_pillar_stops_above_a_part_under_it():
    # A tall part whose courtyard covers H1: the pillar must not reach it.
    tall = Part(
        "U1",
        "fp",
        "top",
        Rect(0, 0, 12, 12),
        above=15.0,
        height_source="enclosure.toml: test",
    )
    f = frame(piece(parts=[tall]))
    _, lid = case.box(f)
    x, y = f.xy(4, 4)
    assert not lid.is_inside(Vector(x + 2.0, y, f.z_top + 5.0))


def test_lid_window_opens_over_the_part():
    f = frame(piece(lid_windows=["U1"]))
    _, lid = case.box(f)
    x, y = f.xy(30, 20)
    wall_top = f.z_top + f.headroom + f.p.lid_clearance
    assert not lid.is_inside(Vector(x, y, wall_top + 1.0))


def test_interference_detects_a_part_through_the_wall():
    """Control for the clash check: a part poking through the wall must clash."""
    outside = Part(
        "J9",
        "fp",
        "top",
        Rect(58, 10, 64, 20),
        above=5.0,
        height_source="enclosure.toml: test",
    )
    f = frame(piece(parts=[outside]))
    keep = case.keepout(f, None)
    whole = Pos(-10, -10, -10) * Box(200, 200, 200, align=A)
    assert case.interference(case.tray(f), keep, whole) > 1.0


def test_no_interference_for_a_part_inside():
    f = frame(piece())
    keep = case.keepout(
        case.Frame(
            piece(
                parts=[
                    Part(
                        "U1",
                        "fp",
                        "top",
                        Rect(20, 10, 40, 30),
                        above=10.0,
                        height_source="enclosure.toml: test",
                    )
                ]
            ),
            case.Params(),
        ),
        None,
    )
    whole = Pos(-10, -10, -10) * Box(200, 200, 200, align=A)
    base, lid = case.box(f)
    assert case.interference(base, keep, whole) == 0.0
    assert case.interference(lid, keep, whole) == 0.0


def test_unknown_case_key_is_refused():
    with pytest.raises(Exception, match="unknown key"):
        case.Params.from_toml({"wal": 2.0})


def test_every_boss_and_the_lid_lip_keep_the_minimum_wall():
    for _, clear, boss in case.BOLTS.values():
        assert (boss - clear) / 2 >= case.MIN_WALL
    with pytest.raises(Exception, match="under"):
        case.Params.from_toml({"wall": 1.2})


def test_a_boss_tangent_to_the_lip_is_grown_so_the_lid_meshes():
    """H2-like hole whose pillar would touch the lip's inner face exactly."""
    from build123d import Mesher

    f = frame(piece())
    inner = f.p.wall + 0.3 + case.MIN_WALL
    boss = case.BOLTS[3.2][2] / 2
    # Board x that puts the pillar's outer surface on the lip's inner face.
    x_case = f.length - inner - boss
    hole_x = x_case - f.off + f.piece.region.x0
    f = frame(piece(holes=[Hole("H2", hole_x, 4, 3.2)]))
    _, lid = case.box(f)
    Mesher().add_shape(lid)  # raises "3mf mesh is invalid" when tangent


def test_thickness_probe_reads_a_known_plate():
    """Control for the wall gate: a 0.5 mm plate must read as 0.5 mm."""
    from enclosure import thickness

    plate = Box(20, 20, 0.5, align=A)
    assert thickness.thinnest(plate).mm == pytest.approx(0.5, abs=0.02)


def test_a_lid_window_over_a_pillar_keeps_the_minimum_wall():
    """Head 1's camera window once cut a pillar's edge down to 0.15 mm."""
    from enclosure import thickness

    # Head 1's case: the window (courtyard - 1 mm margin) ends 0.15 mm past the
    # pillar's bore. H5 at board (4, 4) -> case x 6.5, bore edge 7.7, so a
    # courtyard from board x 6.35 puts the window edge at case x 7.85.
    tall = Part(
        "U1", "fp", "top", Rect(6.35, 2, 30, 30), above=15.0, height_source="test"
    )
    f = frame(piece(parts=[tall], holes=[Hole("H5", 4, 4, 2.2)], lid_windows=["U1"]))
    _, lid = case.box(f)
    assert thickness.thinnest(lid).mm >= case.MIN_WALL - 0.05
