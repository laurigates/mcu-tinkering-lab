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
    assert pytest.approx(60 + 2 * (2.0 + 0.5) + 2 * f.p.tab) == bb.size.X  # tabs
    assert pytest.approx(40 + 2 * (2.0 + 0.5)) == bb.size.Y
    assert pytest.approx(f.z_top + 3.0) == bb.size.Z


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
    assert pytest.approx(f.length + 2 * f.p.tab) == bb.size.X
    assert not tray.is_inside(Vector(-f.p.tab / 2, f.width / 2, f.p.floor / 2))
    assert tray.is_inside(Vector(-f.p.tab / 2 + 4, f.width / 2, f.p.floor / 2))
    p = piece()
    p.mount_tabs = False
    assert pytest.approx(f.length) == case.tray(frame(p)).bounding_box().size.X


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


def wall_top(f) -> float:
    return case.lid_tabs(f)[0]


def test_nothing_in_the_base_stands_over_the_board():
    """The 2026-10-23 boxes had a lid boss over each board corner: the board
    could not get past them, nor a screwdriver to its screws."""
    f = frame(piece())
    base, _ = case.box(f)
    r = f.piece.region
    over = Pos(f.off, f.off, f.z_top + 0.1) * Box(
        r.width, r.height, wall_top(f), align=A
    )
    hit = base & over
    assert not hit or sum(s.volume for s in hit.solids()) < 1e-6


def test_two_lid_tabs_on_each_long_side_one_per_half():
    f = frame(piece())  # 60 x 40: the long sides are north and south
    _, tabs = case.lid_tabs(f)
    for side in ("north", "south"):
        us = sorted(t.u for t in tabs if t.side == side)
        assert len(us) == 2, (side, us)
        assert us[0] < 30 < us[1]
    assert {t.side for t in tabs} == {"north", "south"}


def test_each_tab_pilot_lines_up_with_a_wall_hole():
    f = frame(piece())
    base, lid = case.box(f)
    for t in case.lid_tabs(f)[1]:
        nx, ny = t.normal
        ux, uy = -ny, nx  # along the wall
        wall_pt = Vector(
            t.x + nx * (case.TAB_T / 2 + 0.3 + f.p.wall / 2),
            t.y + ny * (case.TAB_T / 2 + 0.3 + f.p.wall / 2),
            t.z,
        )
        assert not base.is_inside(wall_pt), "no screw hole in the wall"
        above = wall_pt + Vector(0, 0, case.LID_CLEARANCE / 2 + 1.0)
        assert base.is_inside(above), "the wall above the hole is gone"
        assert not lid.is_inside(Vector(t.x, t.y, t.z)), "no pilot in the tab"
        beside = Vector(t.x + ux * 2.6, t.y + uy * 2.6, t.z)
        assert lid.is_inside(beside), "tab missing round its pilot"


def test_the_lid_reaches_down_only_by_its_tabs():
    f = frame(piece())
    _, lid = case.box(f)
    assert pytest.approx(wall_top(f) - case.TAB_DROP) == lid.bounding_box().min.Z


def test_a_tab_avoids_a_tall_part_under_its_wall():
    tall = Part("U9", "fp", "top", Rect(2, 0, 26, 6), above=10.0, height_source="test")
    f = frame(piece(parts=[tall], holes=[]))
    top, tabs = case.lid_tabs(f)
    for t in tabs:
        if t.side == "north":
            assert not (2 - case.TAB_W / 2 < t.u < 26 + case.TAB_W / 2), t
    assert top == pytest.approx(f.z_top + f.headroom + f.p.lid_clearance)


def test_the_box_grows_when_no_tab_position_clears_the_parts():
    wide = Part("U9", "fp", "top", Rect(0, 0, 60, 6), above=10.0, height_source="test")
    f = frame(piece(parts=[wide], holes=[]))
    top, _ = case.lid_tabs(f)
    assert top >= f.z_top + 10.0 + 0.5 + case.TAB_DROP - 1e-6
    keep = case.keepout(f, None)
    whole = Pos(-10, -10, -10) * Box(200, 200, 200, align=A)
    _, lid = case.box(f)
    assert case.interference(lid, keep, whole) == 0.0


def test_tabs_keep_off_cable_windows():
    jack = Part("J1", "fp", "top", Rect(5, 0, 25, 5), above=5.0, height_source="test")
    f = frame(piece(parts=[jack], openings=["J1"], holes=[]))
    m = f.p.opening_margin
    for t in case.lid_tabs(f)[1]:
        if t.side == "north":
            lo, hi = t.u - case.TAB_W / 2, t.u + case.TAB_W / 2
            assert hi <= 5 - m or lo >= 25 + m, t


def test_a_box_with_no_wall_pair_for_its_tabs_is_refused():
    # Jacks along the north edge and down the west edge, 1 mm in from the
    # corners so each one's nearest wall is the intended one; their windows
    # leave no 8 mm gap on either wall, so neither pair can take two tabs.
    north = [
        Part(f"N{i}", "fp", "top", Rect(x, 0, x + w, 5), above=5.0, height_source="t")
        for i, (x, w) in enumerate(((1, 17), (21, 18), (42, 17)))
    ]
    west = [
        Part(f"W{i}", "fp", "top", Rect(0, y, 5, y + h), above=5.0, height_source="t")
        for i, (y, h) in enumerate(((7, 6), (14, 12), (28, 11)))
    ]
    jacks = north + west
    f = frame(piece(parts=jacks, openings=[j.ref for j in jacks], holes=[]))
    with pytest.raises(case.EnclosureError, match=r"lid tabs on the north wall.*west"):
        case.lid_tabs(f)


def test_box_window_is_closed_above_the_plug():
    edge = Part(
        "J1", "fp", "top", Rect(55, 15, 59.5, 25), above=8.0, height_source="test"
    )
    f = frame(piece(parts=[edge], openings=["J1"]))
    base, _ = case.box(f)
    _, y = f.xy(0, 20)
    x = f.length - 1.0
    assert not base.is_inside(Vector(x, y, f.z_top + 8.0)), "window too low"
    assert base.is_inside(Vector(x, y, wall_top(f) - 1.0)), "window open to the top"


def test_lid_window_opens_over_the_part():
    f = frame(piece(lid_windows=["U1"]))
    _, lid = case.box(f)
    x, y = f.xy(30, 20)
    assert not lid.is_inside(Vector(x, y, wall_top(f) + 1.0))


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


def test_every_boss_and_lid_tab_keep_the_minimum_wall():
    for _, pilot, boss in case.SCREWS.values():
        assert (boss - pilot) / 2 >= case.MIN_WALL - 1e-9
    assert (case.TAB_W - case.LID_PILOT) / 2 >= case.MIN_WALL - 1e-9
    assert case.TAB_T >= case.MIN_THREAD
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
    corner must leave MIN_WALL round the lid tabs and inside the lip."""
    from enclosure import thickness

    tall = Part("U1", "fp", "top", Rect(0, 0, 30, 30), above=15.0, height_source="test")
    f = frame(piece(parts=[tall], lid_windows=["U1"]))
    _, lid = case.box(f)
    assert thickness.thinnest(lid).mm >= case.MIN_WALL - 0.05


def test_the_plain_lid_keeps_the_minimum_wall():
    from enclosure import thickness

    _, lid = case.box(frame(piece()))
    assert thickness.thinnest(lid).mm >= case.MIN_WALL - 0.05


def test_tabs_move_to_the_short_sides_when_windows_fill_the_long_ones():
    # A square head (robocar head 1): its camera and header windows take the
    # north wall, so the tabs go west and east.
    cam = Part("U1", "fp", "top", Rect(6, 0, 24, 21), above=26.0, height_source="t")
    hdr = Part("J2", "fp", "top", Rect(1, 0, 5, 4), above=3.0, height_source="t")
    hdr2 = Part("J3", "fp", "top", Rect(25, 0, 29, 4), above=3.0, height_source="t")
    p = piece(parts=[cam, hdr, hdr2], openings=["U1", "J2", "J3"], holes=[])
    p.region = Rect(0, 0, 30, 30)
    _, tabs = case.lid_tabs(frame(p))
    assert {t.side for t in tabs} == {"west", "east"}


def test_tab_ends_keep_the_slide_fit_from_the_perpendicular_walls():
    # Jacks over both quarter points push the tabs to the ends of the wall
    # (review of #909: they sat flush against the west and east walls).
    j1 = Part("J1", "fp", "top", Rect(11, 0, 21, 5), above=5.0, height_source="t")
    j2 = Part("J2", "fp", "top", Rect(39, 0, 49, 5), above=5.0, height_source="t")
    f = frame(piece(parts=[j1, j2], openings=["J1", "J2"], holes=[]))
    for t in case.lid_tabs(f)[1]:
        if t.side in ("north", "south"):
            lo, hi = t.x - case.TAB_W / 2, t.x + case.TAB_W / 2
            assert lo >= f.p.wall + case.LIP_FIT - 1e-9, t
            assert hi <= f.length - f.p.wall - case.LIP_FIT + 1e-9, t


def test_windows_below_the_board_do_not_push_the_tabs_away():
    under = [
        Part(
            f"B{i}", "fp", "bottom", Rect(x, 0, x + w, 5), above=0.0, height_source="t"
        )
        for i, (x, w) in enumerate(((1, 17), (21, 18), (42, 17)))
    ]
    top = Part("U1", "fp", "top", Rect(20, 10, 40, 30), above=10.0, height_source="t")
    f = frame(piece(parts=[*under, top], openings=[q.ref for q in under], holes=[]))
    assert {t.side for t in case.lid_tabs(f)[1]} == {"north", "south"}


def test_the_clip_keeps_an_overhang_and_drops_the_other_pieces():
    f = frame(piece())
    clip = case.piece_clip(f, [f.piece.region, Rect(60, 0, 90, 40)])
    x, y = f.xy(30, -3.0)  # 3 mm past the north edge: through a window
    assert clip.is_inside(Vector(x, y, f.z_top + 1))
    x, y = f.xy(61.0, 20)  # just inside the neighbouring piece
    assert not clip.is_inside(Vector(x, y, f.z_top + 1))
    x, y = f.xy(59.0, 20)  # still this piece
    assert clip.is_inside(Vector(x, y, f.z_top + 1))
