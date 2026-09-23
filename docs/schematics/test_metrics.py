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
    collinear_overlaps,
    crossings,
    format_table,
    junctions,
    length_inside_boxes,
    measure,
    measure_circuits,
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
