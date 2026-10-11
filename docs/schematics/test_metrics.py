"""Tests for the routing-quality metrics in metrics.py.

Every metric is pinned on small hand-made wires and boxes whose answer can be
worked out on paper, so a later change to the router that moves a number in
``metrics.py``'s report is known to be a change in the drawing, not in the
ruler. The last test runs the harness over the real circuits only to prove it
loads them the way ``render.py`` does; it deliberately asserts no values.
"""

import sys
from pathlib import Path as FsPath

import pytest

sys.path.insert(0, str(FsPath(__file__).parent))

from metrics import (  # noqa: E402
    Box,
    CircuitMetrics,
    Label,
    Tag,
    collinear_gap_sites,
    collinear_gaps,
    collinear_overlaps,
    crossings,
    format_table,
    junctions,
    length_inside_boxes,
    length_over_labels,
    length_over_tags,
    measure,
    measure_circuits,
    near_parallel_length,
    shared_vertex_sites,
    shared_vertices,
    tight_parallel_pairs,
    total_length,
)

GRID = 0.25


# -- length -------------------------------------------------------------------


def test_total_length_sums_every_segment_of_every_wire():
    wires = [[(0, 0), (3, 0), (3, 2)], [(10, 10), (10, 11)]]
    assert total_length(wires) == pytest.approx(6.0)


def test_length_inside_counts_only_the_strict_interior():
    box = Box(0, 0, 4, 2)
    # Crosses the box horizontally through its middle: 4 units inside.
    through = [(-1, 1), (5, 1)]
    # Runs exactly along the top edge: on the boundary, never inside.
    along_edge = [(-1, 2), (5, 2)]
    # Starts inside and leaves downward: 0.5 units inside (y in 0..0.5).
    leaving = [(3, 0.5), (3, -3)]
    assert length_inside_boxes([through], [box]) == pytest.approx(4.0)
    assert length_inside_boxes([along_edge], [box]) == pytest.approx(0.0)
    assert length_inside_boxes([leaving], [box]) == pytest.approx(0.5)


def test_overlapping_boxes_are_counted_once_as_a_union():
    # Two boxes overlapping on x in 2..3; a wire across both is inside
    # 0..5 = 5 units, not 3 + 3 = 6.
    boxes = [Box(0, 0, 3, 2), Box(2, 0, 5, 2)]
    assert length_inside_boxes([[(-1, 1), (6, 1)]], boxes) == pytest.approx(5.0)


def test_foreign_body_length_skips_the_boxes_a_wire_terminates_on():
    own = Box(0, 0, 2, 2)  # the wire starts on its right edge...
    other = Box(5, -1, 7, 1)  # ...and cuts through this one on the way out
    wire = [(2, 0.5), (10, 0.5)]
    assert length_inside_boxes([wire], [own, other]) == pytest.approx(2.0)
    assert length_inside_boxes(
        [wire], [own, other], exclude_terminal=True
    ) == pytest.approx(2.0)

    # Reversed, so the wire leaves its pin straight back across its own
    # body: that length counts towards inside_any but not inside_foreign.
    back = [(2, 0.5), (-1, 0.5)]
    assert length_inside_boxes([back], [own, other]) == pytest.approx(2.0)
    assert length_inside_boxes(
        [back], [own, other], exclude_terminal=True
    ) == pytest.approx(0.0)


def test_length_over_tags_counts_only_tags_the_wire_is_not_wired_to():
    # A ground tag whose stub hangs off a pin at (-1, 1): terminal at (0, 1),
    # body below it. #591's shape: a net passing through the symbol.
    tag = Tag(Box(-0.25, 0.36, 0.25, 1.0), ((0.0, 1.0), (-1.0, 1.0)))
    across = [(-3, 0.5), (3, 0.5)]  # straight through the body: 0.5 wide
    down = [(0, 3), (0, -3)]  # down its centre line: the full 0.64 height
    # Down the right edge: unlike a component body this counts, because a
    # ground tag's box is exactly its top bar's width — the run touches the
    # bar's end. robocar_unified's STBY net did that once moved off-centre.
    along_edge = [(0.25, 3), (0.25, -3)]
    beside = [(0.5, 3), (0.5, -3)]  # one grid step clear: nothing
    own_pin = [(-1.0, 1.0), (-1.0, 3.0)]  # ends on the pin the stub leads to
    own_terminal = [(0.0, 0.5), (3.0, 0.5)]  # starts inside the tag itself
    assert length_over_tags([across], [tag]) == pytest.approx(0.5)
    assert length_over_tags([down], [tag]) == pytest.approx(0.64)
    assert length_over_tags([along_edge], [tag]) == pytest.approx(0.64)
    assert length_over_tags([beside], [tag]) == pytest.approx(0.0)
    assert length_over_tags([own_pin], [tag]) == pytest.approx(0.0)
    assert length_over_tags([own_terminal], [tag]) == pytest.approx(0.0)
    # A wire of the tag's own net is still charged for every *other* tag.
    other = Tag(Box(5.0, 0.0, 5.5, 1.0), ((5.25, 0.0),))
    own_then_other = [(-1.0, 1.0), (-1.0, 0.5), (8.0, 0.5)]
    assert length_over_tags([own_then_other], [tag, other]) == pytest.approx(0.5)


def test_measure_reports_over_tags():
    tag = Tag(Box(-0.25, 0.36, 0.25, 1.0), ((0.0, 1.0),))
    m = measure("demo", [[(-3, 0.5), (3, 0.5)]], [], GRID, [tag])
    assert m.over_tags == pytest.approx(0.5)
    assert measure("demo", [[(-3, 0.5), (3, 0.5)]], [], GRID).over_tags == 0.0


def test_length_over_labels_counts_every_label_but_those_of_its_own_tags():
    # A "+3V3" tag whose stub hangs off a pin at (-1, 0), its text box above
    # the terminal, and a chip's name label off to the right (#641).
    tag = Tag(Box(-0.25, 0.0, 0.25, 0.5), ((0.0, 0.0), (-1.0, 0.0)))
    tag_label = Label(Box(-0.4, 0.5, 0.4, 0.8), "+3V3", tag)
    chip_label = Label(Box(2.0, 0.0, 4.0, 0.3), "ESP32-S3-Zero")
    labels = [tag_label, chip_label]
    across = [(-3, 0.65), (1, 0.65)]  # through the tag's text: 0.8 wide
    down = [(3, 3), (3, -3)]  # down through the chip label: 0.3 tall
    # Along the text box's top edge: counts, as for a tag body, because the
    # box is only schemdraw's estimate of the text and a substituted font
    # draws past it.
    along_edge = [(-3, 0.8), (1, 0.8)]
    beside = [(-3, 1.0), (1, 1.0)]  # clear of it: nothing
    # Ends on the pin the tag hangs off, so the tag's own label is exempt,
    # exactly as its body is in over_tags...
    own = [(-1.0, 0.0), (-1.0, 0.65), (0.6, 0.65)]
    # ...but no other label is: a chip's name crossed by one of its own nets
    # is no easier to read (gamepad_synth's GPIO8/GPIO9 did that).
    own_then_chip = [(-1.0, 0.0), (-1.0, 0.15), (5.0, 0.15)]
    assert length_over_labels([across], labels) == pytest.approx(0.8)
    assert length_over_labels([down], labels) == pytest.approx(0.3)
    assert length_over_labels([along_edge], labels) == pytest.approx(0.8)
    assert length_over_labels([beside], labels) == pytest.approx(0.0)
    assert length_over_labels([own], labels) == pytest.approx(0.0)
    assert length_over_labels([own_then_chip], labels) == pytest.approx(2.0)


def test_measure_reports_over_labels():
    label = Label(Box(-0.4, 0.5, 0.4, 0.8), "+3V3")
    m = measure("demo", [[(-3, 0.65), (3, 0.65)]], [], GRID, labels=[label])
    assert m.over_labels == pytest.approx(0.8)
    assert measure("demo", [[(-3, 0.65), (3, 0.65)]], [], GRID).over_labels == 0.0


# -- crossings ----------------------------------------------------------------


def test_perpendicular_interiors_cross_once():
    h = [(0, 0), (4, 0)]
    v = [(2, -2), (2, 2)]
    assert crossings([h, v]) == 1


def test_touching_at_an_endpoint_is_not_a_crossing():
    h = [(0, 0), (4, 0)]
    t_junction = [(2, 0), (2, 2)]  # ends on h's interior: a T, not a crossing
    corner = [(4, 0), (4, 3)]  # shares h's endpoint
    assert crossings([h, t_junction]) == 0
    assert crossings([h, corner]) == 0


def test_a_wire_does_not_cross_itself_for_this_metric():
    # A self-intersecting path is a router bug of a different kind; the
    # metric is about distinct wires meeting.
    loop = [(0, 0), (4, 0), (4, 2), (2, 2), (2, -2)]
    assert crossings([loop]) == 0


def test_crossings_count_every_intersecting_segment_pair():
    h1 = [(0, 0), (10, 0)]
    h2 = [(0, 1), (10, 1)]
    v = [(5, -1), (5, 2)]
    assert crossings([h1, h2, v]) == 2


# -- parallel runs --------------------------------------------------------------


def test_tight_parallel_within_one_grid_step_with_overlap():
    a = [(0, 0), (4, 0)]
    near = [(2, GRID), (6, GRID)]  # one grid step away, overlaps 2..4
    far = [(2, 2 * GRID), (6, 2 * GRID)]  # two steps away from a
    assert tight_parallel_pairs([a, near], GRID) == 1
    assert tight_parallel_pairs([a, far], GRID) == 0


def test_parallel_segments_that_do_not_overlap_are_not_tight():
    a = [(0, 0), (4, 0)]
    after = [(4, GRID), (8, GRID)]  # touch at x=4 only: zero-length overlap
    assert tight_parallel_pairs([a, after], GRID) == 0


def test_vertical_parallels_are_counted_too():
    a = [(0, 0), (0, 4)]
    b = [(0.1, 1), (0.1, 3)]
    assert tight_parallel_pairs([a, b], GRID) == 1


def test_collinear_overlap_is_counted_separately_from_tight_parallel():
    a = [(0, 0), (4, 0)]
    on_top = [(3, 0), (6, 0)]
    assert collinear_overlaps([a, on_top]) == 1
    assert tight_parallel_pairs([a, on_top], GRID) == 0
    # And a tight-but-separate pair is not a collinear overlap.
    assert collinear_overlaps([a, [(0, GRID), (4, GRID)]]) == 0


def test_segments_of_the_same_wire_are_never_paired():
    zigzag = [(0, 0), (4, 0), (4, GRID), (0, GRID)]
    assert tight_parallel_pairs([zigzag], GRID) == 0
    assert collinear_overlaps([zigzag]) == 0


# -- near-parallel runs (#494) ----------------------------------------------------


def test_near_parallel_sums_the_overlap_of_runs_two_rows_apart():
    a = [(0, 0), (4, 0)]
    two_rows = [(1, 2 * GRID), (6, 2 * GRID)]  # overlaps 1..4: 3 units
    assert near_parallel_length([a, two_rows], GRID) == pytest.approx(3.0)
    # Vertical runs count the same way.
    v = [(0, 0), (0, 4)]
    v_two = [(2 * GRID, 1), (2 * GRID, 2.5)]
    assert near_parallel_length([v, v_two], GRID) == pytest.approx(1.5)


def test_near_parallel_takes_any_separation_up_to_two_rows():
    # Off-lattice pin rows land between rows: 1.5 grid steps is near.
    a = [(0, 0), (4, 0)]
    between = [(0, 1.5 * GRID), (2, 1.5 * GRID)]
    assert near_parallel_length([a, between], GRID) == pytest.approx(2.0)


def test_one_row_and_three_rows_are_not_near_parallel():
    a = [(0, 0), (4, 0)]
    one_row = [(0, GRID), (4, GRID)]  # tight_parallel's, not this ruler's
    three_rows = [(0, 3 * GRID), (4, 3 * GRID)]
    on_top = [(1, 0), (3, 0)]  # collinear
    assert near_parallel_length([a, one_row], GRID) == 0
    assert near_parallel_length([a, three_rows], GRID) == 0
    assert near_parallel_length([a, on_top], GRID) == 0
    # Control: the same wires two rows out do count, so the zeros above are
    # the separation test speaking, not a ruler that never fires.
    assert near_parallel_length([a, [(0, 2 * GRID), (4, 2 * GRID)]], GRID) > 0


def test_near_parallel_needs_a_positive_overlap_and_two_wires():
    a = [(0, 0), (4, 0)]
    end_to_end = [(4, 2 * GRID), (8, 2 * GRID)]  # touch at x=4 only
    assert near_parallel_length([a, end_to_end], GRID) == 0
    u_turn = [(0, 0), (4, 0), (4, 2 * GRID), (0, 2 * GRID)]
    assert near_parallel_length([u_turn], GRID) == 0


# -- shared vertices (#494) -------------------------------------------------------


def test_corner_on_a_corner_is_a_shared_vertex():
    # robocar_unified's SDA/SCL kiss at main: both wires turn at one point,
    # drawing a "+" with no dot and no hop.
    sda = [(4, 0), (0, 0), (0, -1), (-1, -1)]
    scl = [(4, -1), (3.5, -1), (3.5, 0.5), (0, 0.5), (0, 0), (-1, 0)]
    assert shared_vertex_sites([sda, scl]) == [(0, 1, (0.0, 0.0))]
    assert shared_vertices([sda, scl]) == 1


def test_corner_on_a_straight_run_is_a_shared_vertex():
    # Each of a corner's two segments is on one axis, so a corner on another
    # wire's run always has one of them along that run: this wire joins the
    # run at (0, 0), follows it, and leaves at (1, 0), two corners on it.
    run = [(-2, 0), (2, 0)]
    touching = [(0, 2), (0, 0), (1, 0), (1, -1)]
    assert shared_vertex_sites([run, touching]) == [
        (0, 1, (0.0, 0.0)),
        (0, 1, (1.0, 0.0)),
    ]


def test_a_tee_and_a_shared_pin_end_are_not_shared_vertices():
    run = [(-2, 0), (2, 0)]
    tee = [(0, 2), (0, 0)]  # ends on the run's interior: finish() dots it
    pin_a = [(5, 0), (5, 2), (7, 2)]
    pin_b = [(5, 0), (3, 0)]  # both end on the same pin anchor
    assert shared_vertices([run, tee]) == 0
    assert shared_vertices([pin_a, pin_b]) == 0
    # An end landing on another wire's corner is a T at a bend: dotted too.
    bend = [(-2, 0), (0, 0), (0, -2)]
    assert shared_vertices([bend, [(0, 0), (0, 3)]]) == 0
    # Control: carry the tee on past the run so it turns there instead of
    # ending, and the same ruler counts its corner.
    assert shared_vertices([run, [(0, 2), (0, 0), (-3, 0)]]) == 1


def test_a_plain_crossing_is_not_a_shared_vertex():
    assert shared_vertices([[(-2, 0), (2, 0)], [(0, -2), (0, 2)]]) == 0


def test_a_shared_vertex_is_counted_once_per_pair_and_point():
    # Two corners of each wire meet at one point: still one (pair, point).
    a = [(-1, 0), (0, 0), (0, 1), (1, 1), (1, 0), (0, 0), (0, -1)]
    b = [(-1, 1), (0, 1), (0, 0), (2, 0)]
    sites = shared_vertex_sites([a, b])
    assert len(sites) == len(set(sites))
    assert all(i == 0 and j == 1 for i, j, _ in sites)


def test_measure_reports_near_parallel_and_shared_vertices():
    a = [(0, 0), (4, 0)]
    near = [(0, 2 * GRID), (4, 2 * GRID)]
    kiss = [[(4, 0), (0, 0), (0, -1)], [(0, 1), (0, 0), (-1, 0)]]
    m = measure("demo", [a, near], [], GRID)
    assert m.near_parallel == pytest.approx(4.0)
    assert m.shared_vertices == 0
    assert measure("demo", kiss, [], GRID).shared_vertices == 1
    assert "near_par" in format_table([m]) and "shared_vtx" in format_table([m])


# -- collinear gaps (#494) ---------------------------------------------------------


def test_two_nets_stopping_a_row_short_on_one_line_are_a_collinear_gap():
    # robocar_unified's SDA run, once stepped off the kiss, ended 0.25 short
    # of SCL's turn on the same row: one line with a hairline break.
    sda = [(4, 0), (0.25, 0), (0.25, -1)]
    scl = [(0, 1), (0, 0), (-1, 0)]
    assert collinear_gap_sites([sda, scl], GRID) == [(0, 1, "H", 0.0, 0.25)]
    # Vertical runs count the same way, and a gap of exactly one row counts.
    up = [(2, 0), (2, 3)]
    on = [(2, 3 + GRID), (2, 5)]
    assert collinear_gap_sites([up, on], GRID) == [(0, 1, "V", 2.0, GRID)]


def test_touching_far_overlapping_or_offset_runs_are_not_collinear_gaps():
    a = [(0, 0), (4, 0)]
    touching = [(4, 0), (8, 0)]  # a shared end: a point, not a gap
    two_rows = [(4 + 2 * GRID, 0), (8, 0)]  # far enough to read as two lines
    overlapping = [(3, 0), (8, 0)]  # collinear_overlaps', not this ruler's
    offset = [(4.25, GRID), (8, GRID)]  # a row up: a different line
    for other in (touching, two_rows, overlapping, offset):
        assert collinear_gaps([a, other], GRID) == 0, other
    # One wire's own runs never pair with each other.
    assert collinear_gaps([[(0, 0), (1, 0), (1, 1), (2, 1), (2, 0), (3, 0)]], GRID) == 0
    # Control: the same pair a row short of each other does count, so the
    # zeros above are the gap test speaking, not a ruler that never fires.
    assert collinear_gaps([a, [(4 + GRID, 0), (8, 0)]], GRID) == 1


def test_measure_reports_collinear_gaps():
    m = measure("demo", [[(0, 0), (4, 0)], [(4.25, 0), (8, 0)]], [], GRID)
    assert m.collinear_gaps == 1
    assert "coll_gap" in format_table([m])


# -- junctions ----------------------------------------------------------------


def test_junction_needs_three_wire_ends_at_one_point():
    star = [
        [(0, 0), (2, 0)],
        [(0, 0), (0, 2)],
        [(0, 0), (-2, 0)],
    ]
    pair = [[(5, 5), (7, 5)], [(5, 5), (5, 7)]]
    assert junctions(star) == 1
    assert junctions(pair) == 0
    assert junctions(star + pair) == 1


def test_junction_ends_are_matched_through_float_noise():
    star = [
        [(0.1 + 0.2, 0), (2, 0)],  # 0.30000000000000004
        [(0.3, 0), (0.3, 2)],
        [(0.3, 0), (0.3, -2)],
    ]
    assert junctions(star) == 1


# -- aggregation / output -----------------------------------------------------


def test_measure_combines_every_metric():
    wires = [[(-1, 1), (5, 1)], [(2, -1), (2, 3)]]
    boxes = [Box(0, 0, 4, 2)]
    m = measure("demo", wires, boxes, GRID)
    assert m == CircuitMetrics(
        name="demo",
        wires=2,
        total_length=pytest.approx(10.0),
        inside_foreign=pytest.approx(6.0),
        inside_any=pytest.approx(6.0),
        crossings=1,
        tight_parallel=0,
        collinear_overlaps=0,
        junctions=0,
    )


def test_table_is_sorted_by_circuit_name():
    rows = [
        measure("zeta", [[(0, 0), (1, 0)]], [], GRID),
        measure("alpha", [[(0, 0), (1, 0)]], [], GRID),
    ]
    table = format_table(rows)
    assert table.index("alpha") < table.index("zeta")


def test_real_circuits_load_and_measure():
    results = measure_circuits()
    names = [m.name for m in results]
    assert names == sorted(names)
    assert "robocar_unified" in names
    for m in results:
        assert m.wires > 0
        assert m.total_length > 0


# -- drawn marks ----------------------------------------------------------------


def test_measure_drawing_counts_the_hops_and_dots_actually_drawn():
    # ``junctions`` counts routed-wire ends only; ``hops`` and ``dots`` count
    # the marks in the drawing, hand-drawn leads included, so the report
    # matches what the image shows.
    import schemdraw
    import schemdraw.elements as elm
    from metrics import measure_drawing
    from routing import Router

    d = schemdraw.Drawing(show=False)
    router = Router(d)
    router.wire((0.0, 0.0), (4.0, 0.0), net="i2c")
    router.wire((2.0, -2.0), (2.0, 2.0), net="pwm")
    d.add(elm.Wire("-").at((3.0, 2.0)).to((3.0, 0.0)))  # a T onto the i2c wire
    router.finish()
    m = measure_drawing("demo", d)
    assert (m.hops, m.dots, m.junctions) == (1, 1, 0)
