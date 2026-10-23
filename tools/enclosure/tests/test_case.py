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
    assert bb.size.X == pytest.approx(60 + 2 * (2.0 + 0.5) + 2 * f.p.tab)  # tabs
    assert bb.size.Y == pytest.approx(40 + 2 * (2.0 + 0.5))
    assert bb.size.Z == pytest.approx(f.z_top + 3.0)


def test_board_y_is_flipped_into_case_y():
    f = frame(piece())
    # Board (4, 4) is near the top-left corner; in case coordinates (y up)
    # that is near the top of the case, not the bottom.
    x, y = f.xy(4, 4)
    assert x == pytest.approx(4 + 2.5)
    assert y == pytest.approx(40 - 4 + 2.5)


def test_each_standoff_has_a_blind_pilot_for_its_screw():
    f = frame(piece())
    tray = case.tray(f)
    for h in f.piece.holes:
        x, y = f.xy(h.x, h.y)
        assert not tray.is_inside(Vector(x, y, f.z_board - 0.5)), "no pilot hole"
        assert tray.is_inside(Vector(x, y, case.SKIN / 2)), (
            "pilot runs through the floor"
        )
        assert tray.is_inside(Vector(x + 2.5, y, f.z_board - 0.5)), "boss wall missing"


def test_chassis_tabs_with_holes_unless_the_piece_opts_out():
    f = frame(piece())
    tray = case.tray(f)
    bb = tray.bounding_box()
    assert bb.size.X == pytest.approx(f.length + 2 * f.p.tab)
    assert not tray.is_inside(Vector(-f.p.tab / 2, f.width / 2, f.p.floor / 2))
    assert tray.is_inside(Vector(-f.p.tab / 2 + 4, f.width / 2, f.p.floor / 2))
    p = piece()
    p.mount_tabs = False
    assert case.tray(frame(p)).bounding_box().size.X == pytest.approx(f.length)


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


def test_the_lid_has_nothing_reaching_down_to_the_board():
    f = frame(piece())
    _, lid = case.box(f)
    wall_top = f.z_top + f.headroom + f.p.lid_clearance
    assert lid.bounding_box().min.Z >= wall_top - 3.0 - 1e-6  # the lip only


def test_lid_screws_go_into_corner_bosses_clear_of_the_board_screws():
    f = frame(piece())
    base, _ = case.box(f)
    # H1 (4, 4) and H2 (56, 36) are under two of the four corner bosses.
    screwed = 0
    for x, y, z in case._lid_bosses(f):
        wall_top = f.z_top + f.headroom + f.p.lid_clearance
        assert base.is_inside(Vector(x + 2.0, y, wall_top - 1.0)), "boss missing"
        assert not base.is_inside(Vector(x, y, wall_top - 1.0)), "pilot missing"
        bx, by = x - f.off, 40 - (y - f.off)
        if any(abs(h.x - bx) < 6 and abs(h.y - by) < 6 for h in f.piece.holes):
            screwed += 1
            assert z >= f.z_top + case.SCREW_HEAD[1], "boss sits on a board screw head"
    assert screwed == 2


def test_box_window_is_closed_above_the_plug():
    edge = Part(
        "J1", "fp", "top", Rect(55, 15, 59.5, 25), above=8.0, height_source="test"
    )
    f = frame(piece(parts=[edge], openings=["J1"]))
    base, _ = case.box(f)
    _, y = f.xy(0, 20)
    x = f.length - 1.0
    wall_top = f.z_top + f.headroom + f.p.lid_clearance
    assert not base.is_inside(Vector(x, y, f.z_top + 8.0)), "window too low"
    assert base.is_inside(Vector(x, y, wall_top - 1.0)), "window open to the top"


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
    for _, pilot, boss in case.SCREWS.values():
        assert (boss - pilot) / 2 >= case.MIN_WALL - 1e-9
    assert (case.LID_BOSS - case.LID_PILOT) / 2 >= case.MIN_WALL - 1e-9
    with pytest.raises(Exception, match="under"):
        case.Params.from_toml({"wall": 1.2})


def test_a_standoff_tangent_to_the_wall_is_grown_so_the_base_meshes():
    """A standoff whose surface would touch the wall's inner face exactly."""
    from build123d import Mesher

    f = frame(piece())
    boss = case.SCREWS[3.2][2] / 2
    hole_x = f.p.wall + boss - f.off  # case x = wall + r: tangent to the west wall
    f = frame(piece(holes=[Hole("H1", hole_x, 20, 3.2)]))
    Mesher().add_shape(case.tray(f))  # raises "3mf mesh is invalid" when tangent


def test_thickness_probe_reads_a_known_plate():
    """Control for the wall gate: a 0.5 mm plate must read as 0.5 mm."""
    from enclosure import thickness

    plate = Box(20, 20, 0.5, align=A)
    assert thickness.thinnest(plate).mm == pytest.approx(0.5, abs=0.02)


def test_a_lid_window_reaching_a_corner_keeps_the_minimum_wall():
    """Head 1's camera window once left a 0.15 mm fin; a window running into a
    corner must leave MIN_WALL round the lid screw and inside the lip."""
    from enclosure import thickness

    tall = Part("U1", "fp", "top", Rect(0, 0, 30, 30), above=15.0, height_source="test")
    f = frame(piece(parts=[tall], lid_windows=["U1"]))
    _, lid = case.box(f)
    assert thickness.thinnest(lid).mm >= case.MIN_WALL - 0.05


def test_a_corner_boss_too_short_for_a_screw_is_dropped():
    """A part filling a corner up to just under the lid leaves no room for a
    boss there; the lid keeps the other three."""
    corner = Part(
        "U1", "fp", "top", Rect(0, 0, 14, 14), above=20.0, height_source="test"
    )
    low = Part(
        "U2", "fp", "top", Rect(20, 10, 40, 30), above=19.0, height_source="test"
    )
    f = frame(piece(parts=[corner, low], holes=[]))
    _, lid = case.box(f)
    wall_top = f.z_top + f.headroom + f.p.lid_clearance
    drilled = [
        (x, y)
        for x, y, _ in case._lid_bosses(f)
        if not lid.is_inside(Vector(x, y, wall_top + f.p.lid / 2))
    ]
    assert len(drilled) == 3, drilled
