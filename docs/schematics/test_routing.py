"""Tests for the Manhattan auto-router in routing.py.

Covers the router's core invariants directly (orthogonality, obstacle
avoidance, exact endpoints) and then re-checks every wire actually drawn by
the real circuits, so a future circuit edit that breaks routing fails here
instead of only showing up as a visual glitch in the rendered SVG.
"""

import math
import sys
from pathlib import Path as FsPath

import pytest
import schemdraw
import schemdraw.elements as elm
from schemdraw.segments import SegmentText

sys.path.insert(0, str(FsPath(__file__).parent))
sys.path.insert(0, str(FsPath(__file__).parent / "circuits"))

from components import esp32_s3_zero, max98357a  # noqa: E402
from routing import Path, Router, _BBox  # noqa: E402


def _segments_are_orthogonal(points):
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        assert ax == ax and by == by  # sanity: real numbers, not NaN
        assert ax == bx or ay == by, (
            f"diagonal segment {(ax, ay)} -> {(bx, by)} is not axis-aligned"
        )


def _segment_hits_box(p1, p2, box: _BBox) -> bool:
    """True if the axis-aligned segment p1->p2 passes through box's interior."""
    (ax, ay), (bx, by) = p1, p2
    xmin, xmax = sorted((ax, bx))
    ymin, ymax = sorted((ay, by))
    # Shrink by a hair so touching an edge (normal for a pin lead) doesn't count.
    eps = 1e-6
    return not (
        xmax - eps <= box.xmin
        or xmin + eps >= box.xmax
        or ymax - eps <= box.ymin
        or ymin + eps >= box.ymax
    )


# Free space between the two chips' facing edges in _build_two_chip_drawing().
_CHIP_GAP = 4.5


def _box_of(el) -> _BBox:
    return _BBox(*el.get_bbox(transform=True, includetext=False))


def _gap_centre_x(left, right) -> float:
    """x midway between left's right body edge and right's left body edge."""
    return (_box_of(left).xmax + _box_of(right).xmin) / 2


def _build_two_chip_drawing():
    """Two chips with a fixed 4.5-unit gap of free space between their facing edges.

    The amp is anchored by its LRC pin, ``_CHIP_GAP`` to the right of the
    ESP32's GPIO6 pin, so the gap does not change when either symbol is
    resized. (It used to be a fixed offset from the ESP32's centre, which
    made the gap depend on both parts' widths: widening the amp from 4 to 6
    needed the offset to go from 9 to 10.) Tests that place an obstacle in
    the gap take its x from ``_gap_centre_x``, which reads the two bounding
    boxes, rather than hardcoding it. Each such test must also assert that
    its obstacle really blocks the straight path, or a drifted obstacle
    makes the test pass vacuously.
    """
    d = schemdraw.Drawing(show=False)
    d.config(unit=2.0, fontsize=12)
    esp = d.add(esp32_s3_zero().label("ESP32-S3-Zero", loc="bot", ofst=0.4))
    amp = d.add(
        max98357a()
        .at((esp.GPIO6[0] + _CHIP_GAP, esp.GPIO6[1]))
        .anchor("LRC")
        .label("MAX98357A", loc="top", ofst=0.4)
    )
    return d, esp, amp


def test_simple_wire_is_orthogonal_and_exact():
    d, esp, amp = _build_two_chip_drawing()
    router = Router(d)
    handle = router.wire(esp.GPIO5, amp.BCLK, net="i2s")
    router.finish()
    # The drawn Path, not just the recorded polyline: the exact-endpoint
    # claim is about what reaches the SVG.
    abspoints = _abspoints(handle.element)

    assert abspoints[0] == (float(esp.GPIO5[0]), float(esp.GPIO5[1]))
    assert abspoints[-1] == (float(amp.BCLK[0]), float(amp.BCLK[1]))
    _segments_are_orthogonal(abspoints)


def test_wire_avoids_obstacle_placed_between_pins():
    d, esp, amp = _build_two_chip_drawing()
    # A small blocker sitting in the middle of the GPIO6 -> LRC straight-line
    # path, but clear of either pin's own stub lead, so there's a genuine
    # detour to find (as opposed to an obstacle swallowing an endpoint,
    # which makes the goal unreachable — see test_unreachable_goal_fails_fast).
    d.add(
        elm.Ic(pins=[elm.IcPin(name="X", side="L")], size=(1, 1))
        .at((_gap_centre_x(esp, amp), esp.GPIO6[1] + 0.2))
        .anchor("center")
    )
    blocker_box = _BBox(*d.elements[-1].get_bbox(transform=True, includetext=False))
    # The detour assertions below only mean something if the naive straight
    # path is actually blocked.
    assert _segment_hits_box(esp.GPIO6, amp.LRC, blocker_box)

    router = Router(d)
    abspoints = router.wire(esp.GPIO6, amp.LRC, net="i2s").points

    _segments_are_orthogonal(abspoints)
    for p1, p2 in zip(abspoints, abspoints[1:]):
        assert not _segment_hits_box(p1, p2, blocker_box), (
            f"routed segment {p1} -> {p2} cuts through the obstacle {blocker_box}"
        )


def test_unreachable_goal_fails_fast():
    # Regression: a malformed layout where an obstacle swallows a pin's own
    # stub lead makes the goal unreachable. Without a bounded search region,
    # A* sweeps across open free space forever looking for it. It must
    # instead raise promptly.
    import time

    d, esp, amp = _build_two_chip_drawing()
    d.add(
        elm.Ic(pins=[elm.IcPin(name="X", side="L")], size=(2, 2))
        .at((_gap_centre_x(esp, amp) + 0.25, esp.center.y + 0.2))
        .anchor("center")
    )
    router = Router(d)
    t0 = time.monotonic()
    try:
        router.wire(esp.GPIO6, amp.LRC, net="i2s")
        raise AssertionError("expected RuntimeError for an unreachable goal")
    except RuntimeError:
        pass
    assert time.monotonic() - t0 < 5.0, "unreachable-goal search did not fail fast"


def test_own_component_is_not_treated_as_an_obstacle():
    # A pin sits exactly on its own chip's boundary; routing from it must not
    # raise just because the owning component's (inflated) box contains it.
    d, esp, amp = _build_two_chip_drawing()
    router = Router(d)
    router.wire(esp.GPIO5, amp.BCLK)
    router.wire(esp.GPIO6, amp.LRC)
    router.wire(esp.GPIO7, amp.DIN)


def _abspoints(el):
    return list(el.polyline)


def test_wire_does_not_cut_through_its_own_destination_chip():
    # Regression (#490): wire() used to drop every box containing either
    # endpoint from the obstacle list for the *whole* search, so a net whose
    # pin sits on the far side of its destination chip took the short way —
    # straight in one side of the body and out the other. GAIN is on the
    # amp's right edge, facing away from the ESP32; the only honest route
    # goes around the amp.
    d, esp, amp = _build_two_chip_drawing()
    abspoints = Router(d).wire(esp.GPIO5, amp.GAIN).points
    amp_box = _box_of(amp)

    _segments_are_orthogonal(abspoints)
    for p1, p2 in zip(abspoints, abspoints[1:]):
        assert not _segment_hits_box(p1, p2, amp_box), (
            f"routed segment {p1} -> {p2} cuts through its own chip {amp_box}"
        )


def test_overlapping_box_around_a_pin_and_its_stub_does_not_block_it():
    # The legitimate reason wire() drops boxes at all: two components can
    # overlap a little, and a second box that contains the pin *and* the
    # stub point the search starts from would otherwise make that end
    # unreachable outright. Narrowing the exclusion to the stub (#490) must
    # keep this case routable.
    d, esp, amp = _build_two_chip_drawing()
    bx, by = float(amp.BCLK[0]), float(amp.BCLK[1])
    d.add(
        elm.Ic(pins=[elm.IcPin(name="X", side="L")], size=(1.5, 1))
        .at((bx - 0.5, by))
        .anchor("center")
    )
    overlap = _box_of(d.elements[-1])
    assert overlap.contains(bx, by) and overlap.contains(bx - 0.75, by)

    abspoints = Router(d).wire(esp.GPIO5, amp.BCLK).points
    assert abspoints[-1] == (bx, by)
    _segments_are_orthogonal(abspoints)


def test_router_wire_defaults_to_visible_stroke():
    # Regression: passing color=None straight through to Path used to omit
    # the SVG stroke attribute entirely, which CSS defaults to `stroke: none`
    # -> an invisible wire (this is how OUT-/OUT+ in gamepad_synth.py were
    # called before net classes: no color= argument). An unclassified wire
    # must still be drawn visibly.
    d, esp, amp = _build_two_chip_drawing()
    router = Router(d)
    handle = router.wire(esp.GPIO5, amp.BCLK)
    router.finish()
    assert handle.element.params.get("color") is not None


def _all_real_circuit_paths(real_circuits):
    from routing import Path, Router

    for circuit in real_circuits:
        d = circuit.drawing
        obstacles = [
            _BBox(*el.get_bbox(transform=True, includetext=False))
            for el in d.elements
            if not isinstance(el, Router._NON_OBSTACLES)
        ]
        for el in d.elements:
            if not isinstance(el, Path):
                continue
            yield circuit.name, list(el.polyline), obstacles


def _contains_endpoint(box: _BBox, point, eps=1e-6) -> bool:
    x, y = point
    return (
        box.xmin - eps <= x <= box.xmax + eps and box.ymin - eps <= y <= box.ymax + eps
    )


def _trim_ends(points, length):
    """``points`` with ``length`` of polyline cut off each end.

    Returns an empty list when the wire is too short to have a middle.
    """

    def trim_front(pts, remaining):
        pts = list(pts)
        while len(pts) >= 2:
            (ax, ay), (bx, by) = pts[0], pts[1]
            seg = abs(bx - ax) + abs(by - ay)  # axis-aligned: L1 == length
            if seg > remaining:
                f = remaining / seg
                pts[0] = (ax + (bx - ax) * f, ay + (by - ay) * f)
                return pts
            remaining -= seg
            pts.pop(0)
        return []

    front = trim_front(points, length)
    return list(reversed(trim_front(list(reversed(front)), length)))


def test_all_real_circuits_route_orthogonally_without_crossing_components(
    real_circuits,
):
    # Only the pin's stub lead may pass through a box: it runs from the pin,
    # which sits on its own chip's edge (and possibly inside a neighbour
    # that overlaps it), out to the stub point where the A* search starts.
    # Past the stub, a wire must clear every body — including the chips it
    # terminates on (#490). So trim a stub's worth off each end, then exempt
    # only the boxes the *trimmed* ends still sit in: the overlapping-box
    # case wire() exists to keep routable.
    #
    # This used to exempt every box containing the raw endpoints — exactly
    # the set wire() dropped from its search — so it could never observe a
    # wire tunnelling through its own chip. The trim is the stub plus half a
    # grid step because the stub point is snapped to the grid.
    probe = Router(schemdraw.Drawing(show=False))
    trim = probe.stub + probe.grid / 2
    checked = 0
    for name, abspts, obstacles in _all_real_circuit_paths(real_circuits):
        _segments_are_orthogonal(abspts)
        middle = _trim_ends(abspts, trim)
        if not middle:
            continue
        relevant = [
            box
            for box in obstacles
            if not _contains_endpoint(box, middle[0])
            and not _contains_endpoint(box, middle[-1])
        ]
        for p1, p2 in zip(middle, middle[1:]):
            for box in relevant:
                assert not _segment_hits_box(p1, p2, box), (
                    f"{name}: routed segment {p1} -> {p2} cuts through {box}"
                )
        checked += 1
    assert checked > 0, "no circuits were found to check"


def test_robocar_unified_wire_stays_out_of_component_bodies(real_circuit):
    # The whole-drawing figure behind #490: with every terminal box dropped
    # for the entire search, robocar_unified carried ~26 units of wire
    # inside component bodies — nets entering their own chip on one side
    # and leaving on the other. Once only a stub may cross a body the
    # figure is 0.00 here; the issue's own target was ~3 units, the length
    # a few stub leads through boxes overlapping their pins would add. The
    # bound admits that legitimate case, not a tunnel coming back.
    m = real_circuit("robocar_unified").metrics
    assert m.inside_any < 3.5, (
        f"robocar_unified routes {m.inside_any:.2f} units of wire inside "
        "component bodies"
    )


def _route_circuit_with_overlap_penalty(monkeypatch, name, penalty):
    """``name``'s routed wires with every ``Router`` built at ``penalty``.

    Circuits construct ``Router(d)`` with defaults, so the penalty is swapped
    in at the keyword default rather than by editing a circuit file.
    """
    from metrics import drawing_wires
    from render import circuit_files, draw_circuit, load_circuit

    monkeypatch.setitem(Router.__init__.__kwdefaults__, "overlap_penalty", penalty)
    (path,) = circuit_files([name])
    return drawing_wires(draw_circuit(load_circuit(path)))


def test_overlap_penalty_is_load_bearing(monkeypatch):
    # overlap_penalty is the only thing keeping parallel nets apart, and
    # the rendered SVG was byte-identical at 6, 50 and 1000 (#491). It was
    # not strictly dead — 0 and 6 routed differently — but it saturated at
    # once: it charged only a wire running exactly on top of another, the
    # default already removed every such run, and the wires one lattice row
    # apart that ADR-023 complains about cost nothing at any setting. A
    # parameter that changes nothing when multiplied tenfold is not one, so
    # a real circuit with parallel neighbours must route differently at 6
    # than at 60.
    #
    # The circuit is balancebot, not robocar_unified: since #495 draws the
    # robocar's boards physically, that drawing routes with no tight pair
    # even in authored order, so it has no neighbour for the penalty to push
    # and routes identically at 6 and 60 (it still differs between 0 and 6).
    # balancebot keeps two tight pairs and responds. A circuit edit can move
    # this again — re-check which drawing has parallel runs (#463).
    low = _route_circuit_with_overlap_penalty(monkeypatch, "balancebot", 6.0)
    high = _route_circuit_with_overlap_penalty(monkeypatch, "balancebot", 60.0)
    assert len(low) == len(high)
    assert low != high, "overlap_penalty 6 and 60 routed identical geometry"


def test_parallel_neighbour_is_pushed_a_lattice_row_further_away():
    # The mechanism behind the test above, pinned without a real circuit:
    # a net whose ends sit one lattice row beside an already-routed wire
    # must not run its whole length in that neighbouring row. Its ends sit
    # a little off the lattice (row 1 is y = 0.25), as a stub point
    # generally does across its lead, so it also exercises the search
    # starting and finishing at the nearest lattice point.
    router = Router(schemdraw.Drawing(show=False))
    g = router.grid
    router.wire((0.0, 0.0), (20 * g, 0.0))
    ys = [y for _, y in router.wire((0.0, 1.1 * g), (20 * g, 1.1 * g)).points]
    assert max(ys) >= 2 * g - 1e-6, f"second wire at y={ys} hugs the first"


# -- record / finish split (#492) ------------------------------------------------
#
# Hops, nudging and net ordering are properties of the *finished* set of wires,
# so wire() only routes and records; finish() is what puts Paths in the drawing.


def test_wire_records_without_drawing_until_finish():
    d, esp, amp = _build_two_chip_drawing()
    router = Router(d)
    before = len(d.elements)
    handle = router.wire(esp.GPIO5, amp.BCLK, net="i2s")
    assert len(d.elements) == before, "wire() drew instead of recording"
    assert handle.element is None
    assert handle.points[0] == (float(esp.GPIO5[0]), float(esp.GPIO5[1]))
    assert handle.points[-1] == (float(amp.BCLK[0]), float(amp.BCLK[1]))

    router.finish()
    assert len(d.elements) == before + 1
    assert handle.element is d.elements[-1]
    assert _abspoints(handle.element) == handle.points


def test_finish_draws_in_recorded_order_once():
    d, esp, amp = _build_two_chip_drawing()
    router = Router(d)
    handles = [
        router.wire(esp.GPIO5, amp.BCLK),
        router.wire(esp.GPIO6, amp.LRC),
        router.wire(esp.GPIO7, amp.DIN),
    ]
    before = len(d.elements)
    drawn = router.finish()
    assert [h.element for h in handles] == drawn == d.elements[before:]
    # A second finish() has nothing left to draw and must not duplicate.
    assert router.finish() == []
    assert len(d.elements) == before + 3


def test_finish_applies_name_to_the_drawn_path():
    d, esp, amp = _build_two_chip_drawing()
    router = Router(d)
    handle = router.wire(esp.GPIO5, amp.BCLK, name="bclk")
    router.finish()
    assert handle.element.name == "bclk"


def test_deferring_the_draw_does_not_change_routing():
    # Occupancy is marked at wire() time and drawn Paths are never obstacles,
    # so drawing each wire immediately or all at the end must route the same.
    def route(finish_each: bool):
        d, esp, amp = _build_two_chip_drawing()
        router = Router(d)
        points = []
        for a, b in (
            (esp.GPIO5, amp.BCLK),
            (esp.GPIO6, amp.LRC),
            (esp.GPIO7, amp.GAIN),
        ):
            points.append(router.wire(a, b).points)
            if finish_each:
                router.finish()
        router.finish()
        return points

    assert route(True) == route(False)


def test_render_harness_rejects_a_forgotten_finish():
    # render.py loads circuits dynamically: a draw() that never calls
    # finish() would otherwise render a drawing with no wires at all.
    from types import SimpleNamespace

    from render import draw_circuit

    def draw(finish: bool):
        d, esp, amp = _build_two_chip_drawing()
        router = Router(d)
        router.wire(esp.GPIO5, amp.BCLK)
        if finish:
            router.finish()
        return d

    try:
        draw_circuit(SimpleNamespace(__name__="forgetful", draw=lambda: draw(False)))
        raise AssertionError("an unfinished router rendered without error")
    except RuntimeError as exc:
        assert "finish()" in str(exc)
    assert draw_circuit(SimpleNamespace(__name__="ok", draw=lambda: draw(True)))


# -- crossings, junctions and net colour (#493) ---------------------------------
#
# Until these existed a crossing and a connection were the same mark: two wires
# that merely cross looked exactly like two that join. finish() now bridges
# every crossing with a hop, dots every junction, and colours each wire by its
# net class. The rules are KiCad's (ShouldHopOver): the horizontal wire hops,
# never at a wire's own endpoint, never where more than two wires meet.


def _free_router(**router_kwargs):
    """A router on an empty drawing: no boxes, so no stubs and straight runs."""
    d = schemdraw.Drawing(show=False)
    d.config(unit=2.0)
    return d, Router(d, **router_kwargs)


def _curve_path(el):
    """The drawn command list of a routed Path: strings and points."""
    from schemdraw.segments import SegmentPath

    (seg,) = el.segments[:1]
    assert isinstance(seg, SegmentPath), f"wire drawn as {type(seg).__name__}"
    return seg.path


def test_a_crossing_gets_one_hop_on_the_horizontal_wire():
    from metrics import crossings

    d, router = _free_router()
    h = router.wire((0.0, 0.0), (4.0, 0.0), net="i2c")
    v = router.wire((2.0, -2.0), (2.0, 2.0), net="pwm")
    router.finish()

    assert crossings([h.points, v.points]) == 1
    assert h.hops == [(2.0, 0.0)]
    assert v.hops == [], "the vertical wire hopped too; exactly one may"
    # Two quarter-circle cubics, never the SVG arc command: schemdraw's
    # matplotlib backend turns "A" into a MOVETO and breaks the path.
    cmds = [c for c in _curve_path(h.element) if isinstance(c, str)]
    assert cmds.count("C") == 2 and "A" not in cmds
    assert "C" not in [c for c in _curve_path(v.element) if isinstance(c, str)]
    # And the logical polyline the metrics read is untouched by the drawing.
    assert h.element.polyline == h.points


def test_no_hop_at_a_shared_endpoint_or_a_tee():
    from routing import _hop_sites

    corner = [[(0.0, 0.0), (2.0, 0.0)], [(2.0, 0.0), (2.0, 2.0)]]
    tee = [[(0.0, 0.0), (4.0, 0.0)], [(2.0, 0.0), (2.0, 2.0)]]
    assert _hop_sites(corner) == [[], []]
    assert _hop_sites(tee) == [[], []]


def test_no_hop_where_more_than_two_wires_meet():
    # A third wire through the crossing point makes it a junction, not a
    # crossing — and hopping one wire over two others would draw a bridge
    # across a connection.
    from routing import _hop_sites

    wires = [
        [(0.0, 0.0), (4.0, 0.0)],
        [(2.0, -2.0), (2.0, 2.0)],
        [(2.0, 1.0), (2.0, -1.0)],
    ]
    assert _hop_sites(wires) == [[], [], []]


def test_hop_bulges_the_same_side_whichever_way_the_wire_runs():
    # The segment angle is normalised into [0, pi) before choosing the side,
    # so a wire routed right-to-left does not flip its hop upside down.
    from routing import HOP_RADIUS, Path

    for pts in ([(0.0, 0.0), (4.0, 0.0)], [(4.0, 0.0), (0.0, 0.0)]):
        ys = [
            p[1]
            for p in _curve_path(Path(pts, hops=[(2.0, 0.0)]))
            if not isinstance(p, str)
        ]
        assert min(ys) >= -1e-9
        assert abs(max(ys) - HOP_RADIUS) < 1e-9


def test_hops_closer_than_a_diameter_merge_into_one_bridge():
    # Adjacent verticals sit one grid step (0.25) apart, closer than two hop
    # radii: two separate arcs would overlap and double back on themselves.
    from routing import Path

    xs = [
        p[0]
        for p in _curve_path(
            Path([(0.0, 0.0), (4.0, 0.0)], hops=[(2.0, 0.0), (2.25, 0.0)])
        )
        if not isinstance(p, str)
    ]
    assert xs == sorted(xs), f"bridge doubles back: {xs}"


def test_hop_near_a_segment_end_stays_inside_the_segment():
    from routing import Path

    el = Path([(0.0, 0.0), (2.05, 0.0), (2.05, 3.0)], hops=[(2.0, 0.0)])
    xs = [p[0] for p in _curve_path(el) if not isinstance(p, str)]
    assert max(xs) <= 2.05 + 1e-9


def test_junction_dot_where_three_wire_ends_meet():
    d, router = _free_router()
    router.wire((2.0, 0.0), (0.0, 0.0), net="i2c")
    router.wire((2.0, 0.0), (4.0, 0.0), net="i2c")
    router.wire((2.0, 0.0), (2.0, 2.0), net="i2c")
    router.finish()
    dots = [el for el in d.elements if isinstance(el, elm.Dot)]
    assert router.junctions == [(2.0, 0.0)]
    assert len(dots) == 1


def test_junction_dot_where_a_wire_ends_on_another_wires_interior():
    from routing import _junction_points

    tee = [[(0.0, 0.0), (4.0, 0.0)], [(2.0, 0.0), (2.0, 2.0)]]
    on_bend = [[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)], [(2.0, 0.0), (4.0, 0.0)]]
    corner = [[(0.0, 0.0), (2.0, 0.0)], [(2.0, 0.0), (2.0, 2.0)]]
    crossing = [[(0.0, 0.0), (4.0, 0.0)], [(2.0, -2.0), (2.0, 2.0)]]
    assert _junction_points(tee) == [(2.0, 0.0)]
    assert _junction_points(on_bend) == [(2.0, 0.0)]
    assert _junction_points(corner) == [], "two ends meeting is a bend, not a join"
    assert _junction_points(crossing) == [], "a crossing is not a join"


def test_wire_net_class_sets_colour_and_rejects_unknown_classes():
    from routing import NET_COLORS

    d, router = _free_router()
    w = router.wire((0.0, 0.0), (4.0, 0.0), net="i2s")
    router.finish()
    assert w.net == "i2s"
    assert w.element.params["color"] == NET_COLORS["i2s"]
    try:
        router.wire((0.0, 1.0), (4.0, 1.0), net="steelblue")
        raise AssertionError("an unknown net class was accepted")
    except ValueError as exc:
        assert "i2c" in str(exc), "the error should list the valid classes"


def _real_circuit_drawings(real_circuits):
    return [(c.name, c.drawing) for c in real_circuits]


def test_shared_drawing_fingerprint_sees_every_kind_of_mutation():
    # Negative control for conftest.real_circuits' read-only guard: if the
    # fingerprint missed a mutation, a test changing a shared drawing would
    # silently pass and couple every test after it (#594). The in-place edit
    # is the case a plain SVG hash misses — schemdraw caches the figure.
    from conftest import drawing_fingerprint

    def routed():
        d, router = _free_router()
        router.wire((0.0, 0.0), (4.0, 0.0), net="i2c")
        router.wire((2.0, -2.0), (2.0, 2.0), net="pwm")
        router.finish()
        d.get_imagedata("svg")  # populate the figure cache
        return d, router

    d, _ = routed()
    before = drawing_fingerprint(d)
    assert drawing_fingerprint(d) == before, "reading is not a mutation"

    d, router = routed()
    path = router._wires[0].element
    path.polyline[-1] = (5.0, 0.0)
    path.set_hops(path.hops)  # redraw the Path in place
    assert drawing_fingerprint(d) != before, "in-place element edit missed"

    d, router = routed()
    hops = router._wires[0].hops  # the horizontal wire takes the hop
    assert hops, "the control needs a hop to remove"
    hops.clear()  # router state only: the drawn Path keeps its own copy
    assert drawing_fingerprint(d) != before, "router state edit missed"

    d, _ = routed()
    d.add(elm.Dot().at((8.0, 8.0)))
    assert drawing_fingerprint(d) != before, "added element missed"


def test_shared_circuit_guard_blames_each_mutation_once():
    # The comparison real_circuits runs at teardown: a changed drawing or a
    # changed metrics field (CircuitMetrics is a plain, mutable dataclass) is
    # reported, and only by the check that first sees it (#594).
    from conftest import RenderedCircuit, circuit_fingerprint, mutated_circuits
    from metrics import measure_drawing

    d, router = _free_router()
    router.wire((0.0, 0.0), (4.0, 0.0), net="i2c")
    router.wire((2.0, -2.0), (2.0, 2.0), net="pwm")
    router.finish()
    c = RenderedCircuit("probe", d, measure_drawing("probe", d), b"")
    fingerprints = {c.name: circuit_fingerprint(c)}

    assert mutated_circuits([c], fingerprints) == [], "reading is not a mutation"
    c.metrics.crossings += 1
    assert mutated_circuits([c], fingerprints) == ["probe"], "metrics edit missed"
    assert mutated_circuits([c], fingerprints) == [], "a mutation is blamed once"
    d.add(elm.Dot().at((8.0, 8.0)))
    assert mutated_circuits([c], fingerprints) == ["probe"], "drawing edit missed"


def test_every_real_wire_has_a_net_class(real_circuits):
    from routing import NET_COLORS

    seen = 0
    for name, d in _real_circuit_drawings(real_circuits):
        for router in d._routers:
            for w in router._wires:
                assert w.net in NET_COLORS, f"{name}: {w.points[0]} has no net class"
                assert w.element.params["color"] != "steelblue"
                seen += 1
    assert seen > 0


def test_every_real_crossing_carries_a_hop(real_circuits):
    # Hops between two routed wires, against the metric that counts exactly
    # those crossings. Hops over hand-drawn leads are checked further down.
    from metrics import crossings, drawing_wires
    from routing import _on_wire

    for name, d in _real_circuit_drawings(real_circuits):
        routed = [w.points for r in d._routers for w in r._wires]
        hops = [
            p
            for r in d._routers
            for w in r._wires
            for p in w.hops
            if sum(_on_wire(p, o) for o in routed) == 2
        ]
        assert len(hops) == crossings(drawing_wires(d)), name


def test_robocar_unified_renders_byte_identically_twice(real_circuit):
    # Two independent routes: the session's shared render (its SVG as first
    # drawn) and a fresh load-and-route here.
    from render import circuit_files, draw_circuit, load_circuit

    (path,) = circuit_files(["robocar_unified"])
    first = real_circuit("robocar_unified").svg
    second = draw_circuit(load_circuit(path)).get_imagedata("svg")
    assert first == second
    assert b" C " in first, "no hop was drawn in the densest circuit"


# Hand-drawn leads (elm.Wire / elm.Line: bus trunks, power stubs) are part of
# the rendered output too. finish() cannot redraw them, so a routed wire takes
# the hop across a lead whichever axis it runs on, and a lead ending on a
# routed wire is a junction like any other.


def test_routed_wire_hops_a_hand_drawn_lead_on_either_axis():
    d, router = _free_router()
    d.add(elm.Wire("-").at((2.0, -2.0)).to((2.0, 2.0)))
    d.add(elm.Line().right(4.0).at((10.0, 0.0)))
    h = router.wire((0.0, 0.0), (4.0, 0.0), net="i2c")
    v = router.wire((12.0, -2.0), (12.0, 2.0), net="i2c")
    router.finish()
    assert h.hops == [(2.0, 0.0)]
    assert v.hops == [(12.0, 0.0)], "a lead cannot hop, so the routed wire must"


def test_hand_drawn_lead_ending_on_a_routed_wire_gets_a_dot():
    d, router = _free_router()
    router.wire((0.0, 0.0), (4.0, 0.0), net="signal")
    d.add(elm.Wire("-").at((2.0, 2.0)).to((2.0, 0.0)))
    router.finish()
    assert router.junctions == [(2.0, 0.0)]


def _lead_polylines(d):
    from routing import Path, _simplify

    return [
        _simplify([tuple(el.transform.transform(p)) for p in el.segments[0].path])
        for el in d.elements
        if isinstance(el, (elm.Wire, elm.Line)) and not isinstance(el, Path)
    ]


def test_every_crossing_with_a_routed_wire_is_hopped_in_real_circuits(real_circuits):
    # Over the *final* drawing, so a lead added after finish() that crosses
    # a routed wire — and so was never seen by it — fails here too.
    from routing import _axis_segments, _on_wire

    for name, d in _real_circuit_drawings(real_circuits):
        routed = [w for r in d._routers for w in r._wires]
        everything = [w.points for w in routed] + _lead_polylines(d)
        hopped = {p for w in routed for p in w.hops}
        for i, w in enumerate(routed):
            for _, axis, fixed, lo, hi in _axis_segments(w.points):
                for j, other in enumerate(everything):
                    if j == i:
                        continue
                    for _, o_axis, o_fixed, o_lo, o_hi in _axis_segments(other):
                        if o_axis == axis:
                            continue
                        if not (lo + 1e-6 < o_fixed < hi - 1e-6):
                            continue
                        if not (o_lo + 1e-6 < fixed < o_hi - 1e-6):
                            continue
                        p = (o_fixed, fixed) if axis == "H" else (fixed, o_fixed)
                        if sum(_on_wire(p, e) for e in everything) > 2:
                            continue
                        assert any(
                            abs(p[0] - q[0]) < 1e-6 and abs(p[1] - q[1]) < 1e-6
                            for q in hopped
                        ), f"{name}: crossing at {p} has no hop"


def test_no_dot_where_a_routed_wire_runs_through_a_power_or_ground_tag():
    # A lead ending in a Ground/Vdd tag ends *on the tag*, not free. A routed
    # wire passing through that point (tags are not obstacles) would
    # otherwise read as a T and be dotted — drawing a connection to ground
    # that does not exist. robocar_unified's STBY net ran through the
    # TCA9548A's GND tag and got exactly that dot.
    # The tag cost (#591) is switched off so the wire still runs through the
    # tag: this pins the dot rule for the case the cost only discourages.
    from metrics import length_over_tags

    d, router = _free_router(tag_penalty=0.0)
    d.add(elm.Line().right(2.0).at((0.0, 0.0)))
    d.add(elm.Ground())
    w = router.wire((2.0, -2.0), (2.0, 2.0), net="signal")
    router.finish()
    assert length_over_tags([w.points], router._tags()) > 0, "precondition"
    assert router.junctions == []
    assert not [el for el in d.elements if isinstance(el, elm.Dot)]


# -- power and ground tags (#591) -------------------------------------------------
#
# Tags are not obstacles (see Router._NON_OBSTACLES): a chip's own GND tag sits
# beside its other pins and would make them unreachable. But a wire drawn
# through a tag reads as a connection to the rail, so the search charges a
# penalty per lattice cell inside a tag the wire is not wired to.


def _tag_drawing(label: str = "", loc: str = "top"):
    """A ground tag on a 1-unit stub from a "pin" at (0, 0): terminal (1, 0).

    ``label`` gives the tag a text label at ``loc`` (#641).
    """
    d = schemdraw.Drawing(show=False)
    d.config(unit=2.0)
    d.add(elm.Line().right(1.0).at((0.0, 0.0)))
    tag = elm.Ground()
    d.add(tag.label(label, loc=loc) if label else tag)
    return d


def test_tags_know_the_pin_their_stub_leads_to():
    d = _tag_drawing()
    (tag,) = Router(d)._tags()
    assert tag.owned_by((0.0, 0.0)), "the pin at the far end of the stub"
    assert tag.owned_by((1.0, 0.0)), "the tag's own terminal"
    assert not tag.owned_by((0.0, -3.0))


def test_a_wire_detours_round_a_foreign_tag():
    from metrics import length_over_tags

    def route(penalty):
        d = _tag_drawing()
        router = Router(d, tag_penalty=penalty)
        w = router.wire((-3.0, -0.25), (5.0, -0.25), net="signal")
        return w.points, router._tags()

    straight, tags = route(0.0)
    assert length_over_tags([straight], tags) > 0, "precondition: runs through"
    detoured, tags = route(Router.__init__.__kwdefaults__["tag_penalty"])
    assert length_over_tags([detoured], tags) == 0, f"crossed the tag: {detoured}"


def test_a_wire_to_the_tags_own_pin_is_not_charged():
    # Leaving the pin, the cheapest way to (2, -0.5) is down and straight
    # across the tag's body. The tag is this net's own, so that is allowed:
    # charging it would bend the wire round its own rail symbol.
    from metrics import Tag, length_over_tags

    def route(penalty):
        d = _tag_drawing()
        router = Router(d, tag_penalty=penalty)
        return router.wire((0.0, 0.0), (2.0, -0.5)).points, router._tags()

    free, (tag,) = route(0.0)
    unowned = Tag(tag.box, ())
    assert length_over_tags([free], [unowned]) > 0, "precondition: crosses it"
    charged, _ = route(Router.__init__.__kwdefaults__["tag_penalty"])
    assert charged == free


# -- labels (#641) ------------------------------------------------------------
#
# The obstacle and tag boxes are taken without text, so nothing stopped a wire
# being drawn straight through a label. Each text segment is its own label box,
# charged per lattice point like a tag; a tag's label is exempt for the nets
# wired to that tag, as the tag itself is.


def test_labels_are_read_per_text_segment_and_know_their_tag():
    d = _tag_drawing("GND")
    d.add(elm.Label().at((4.0, 4.0)).label("NOTE"))
    labels = Router(d)._labels()
    assert [lbl.text for lbl in labels] == ["GND", "NOTE"]
    gnd, note = labels
    assert gnd.owned_by((0.0, 0.0)), "the pin its tag's stub leads to"
    assert not gnd.owned_by((0.0, -3.0))
    assert not note.owned_by((4.0, 4.0)), "a plain label belongs to no net"
    assert note.box.contains(4.0, 4.0)


def test_a_wire_detours_round_a_foreign_label():
    from metrics import length_over_labels

    def route(penalty):
        d = schemdraw.Drawing(show=False)
        d.config(unit=2.0)
        d.add(elm.Label().at((1.0, 0.0)).label("NOTE"))
        router = Router(d, label_penalty=penalty)
        w = router.wire((-3.0, 0.0), (5.0, 0.0), net="signal")
        return w.points, router._labels()

    straight, labels = route(0.0)
    assert length_over_labels([straight], labels) > 0, "precondition: runs through"
    detoured, labels = route(Router.__init__.__kwdefaults__["label_penalty"])
    assert length_over_labels([detoured], labels) == 0, f"crossed it: {detoured}"


def test_a_wire_to_the_tags_own_pin_is_not_charged_for_its_label():
    # The tag's own net may cross the tag's label, as it may cross its body:
    # the label sits beside the very pin the wire ends on.
    def route(penalty):
        d = _tag_drawing("GND", loc="bot")
        router = Router(d, label_penalty=penalty, tag_penalty=0.0)
        return router.wire((0.0, 0.0), (2.0, -1.0)).points, router._labels()

    from metrics import Label, length_over_labels

    free, (label,) = route(0.0)
    unowned = Label(label.box, label.text)
    assert length_over_labels([free], [unowned]) > 0, "precondition: crosses it"
    charged, _ = route(Router.__init__.__kwdefaults__["label_penalty"])
    assert charged == free


def test_no_real_wire_runs_through_a_label(real_circuits):
    # #641: robocar_unified's SC1 -> OLED SDA net ran down through the OLED's
    # +3V3 label, balancebot's two MPU6050 nets through both +3V3 labels and
    # the pull-up's "10 kΩ", and gamepad_synth's two piezo nets left their
    # pins straight through the chip's own name.
    for c in real_circuits:
        assert c.metrics.over_labels == 0, (
            f"{c.name}: {c.metrics.over_labels:.2f} units of wire through labels"
        )


def test_no_real_wire_runs_over_a_foreign_power_or_ground_tag(real_circuits):
    # #591: robocar_unified's STBY net ran down through the TCA9548A GND tag
    # (0.64 units, the tag's full height) and balancebot's GPIO0 net across
    # the right DRV8825's (0.50). Correctly undotted, but both read as a
    # connection to ground at a glance.
    for c in real_circuits:
        assert c.metrics.over_tags == 0, (
            f"{c.name}: {c.metrics.over_tags:.2f} units of wire over foreign tags"
        )


def test_every_real_circuit_places_its_tags_before_its_first_net(monkeypatch):
    # The router charges tag_penalty only for tags already in the drawing when
    # wire() runs (#591), so a tag added after the nets is invisible to the
    # search: the test above can only catch a wire over it after the fact
    # (#649, gamepad_synth). Every net must see the circuit's final tag set.
    from render import circuit_files, draw_circuit, load_circuit

    def tag_count(d):
        return sum(isinstance(el, (elm.Vdd, elm.Ground)) for el in d.elements)

    seen: list[tuple[object, int]] = []
    real_wire = Router.wire

    def counting_wire(self, *args, **kwargs):
        seen.append((self.d, tag_count(self.d)))
        return real_wire(self, *args, **kwargs)

    monkeypatch.setattr(Router, "wire", counting_wire)
    checked = 0
    for path in circuit_files([]):
        mod = load_circuit(path)
        if mod is None:
            continue
        seen.clear()
        d = draw_circuit(mod)
        final = tag_count(d)
        nets = [n for drawing, n in seen if drawing is d]
        late = [n for n in nets if n != final]
        assert not late, (
            f"{path.stem}: {len(late)} of {len(nets)} nets routed before all "
            f"{final} power/ground tags were placed"
        )
        checked += 1
    assert checked > 0, "no circuits were found to check"


# -- legibility of the marks ------------------------------------------------------
#
# A hop or a dot the tests can count is worth nothing if the drawing hides it.
# balancebot's GPIO4 -> DIR net ran 0.07 below the nENABLE lead, so its hop
# over the trunk sat 0.07 from the trunk's junction dot: the dot covered the
# arc and the row read as a T into the bus — a connection that does not exist,
# drawn by the very commit meant to tell the two apart.


def _dot_points(d):
    """Centre of every junction dot drawn in ``d``: Dots and dotted Paths."""
    from routing import Path

    points = []
    for el in d.elements:
        if isinstance(el, elm.Dot):
            x, y = el.absanchors["start"]
            points.append((float(x), float(y)))
        elif isinstance(el, Path) and any(
            type(s).__name__ == "SegmentCircle" for s in el.segments
        ):
            points.append(el.polyline[-1])
    return points


def test_router_keeps_off_a_hand_drawn_lead():
    # The balancebot shape in miniature: a net whose ends sit a sliver off a
    # lead's row. The search runs on the lattice row nearest its ends — the
    # lead's own row — so unless the lead counts as occupied the net is drawn
    # alongside it for its whole length, too close to read as two wires.
    d, router = _free_router()
    g = router.grid
    d.add(elm.Wire("-").at((2 * g, 0.0)).to((18 * g, 0.0)))
    w = router.wire((0.0, -0.3 * g), (20 * g, -0.3 * g), net="signal")
    for (ax, ay), (bx, by) in zip(w.points, w.points[1:]):
        if abs(ay - by) > 1e-6:
            continue  # vertical: at most a crossing
        overlap = min(max(ax, bx), 18 * g) - max(min(ax, bx), 2 * g)
        assert not (abs(ay) < g - 1e-6 and overlap > 1e-6), (
            f"wire runs at y={ay} beside the lead at y=0"
        )


def test_every_hop_in_real_circuits_is_drawn_clear_and_full_size(real_circuits):
    # A hop must be visible: clear of every junction dot (a dot over an arc
    # turns a crossing into a connection), clear of every other wire's end or
    # bend (the arc would merge into the corner), and far enough from its own
    # segment's ends not to shrink into an unreadable bump.
    import math

    from routing import HOP_RADIUS, JUNCTION_RADIUS, _on_wire

    for name, d in _real_circuit_drawings(real_circuits):
        routed = [w for r in d._routers for w in r._wires]
        everything = [w.points for w in routed] + _lead_polylines(d)
        dots = _dot_points(d)
        for w in routed:
            for h in w.hops:
                for p in dots:
                    gap = math.dist(h, p)
                    assert gap >= HOP_RADIUS + JUNCTION_RADIUS - 1e-6, (
                        f"{name}: hop at {h} is {gap:.3f} from the dot at {p}"
                    )
                for a, b in zip(w.points, w.points[1:]):
                    if _on_wire(h, [a, b]):
                        room = min(math.dist(h, a), math.dist(h, b))
                        assert room >= HOP_RADIUS - 1e-6, (
                            f"{name}: hop at {h} shrinks to r={room:.3f}"
                        )
                for other in everything:
                    if other is w.points:
                        continue
                    for p in other:
                        gap = math.dist(h, p)
                        assert not 1e-6 < gap < HOP_RADIUS - 1e-6, (
                            f"{name}: hop at {h} is {gap:.3f} from a corner at {p}"
                        )


def test_no_two_hand_drawn_leads_cross_in_real_circuits(real_circuits):
    # finish() hops every crossing that involves a routed wire; a crossing
    # between two leads it cannot redraw would carry no hop at all. None
    # exists today — this keeps "every crossing carries a hop" true.
    from metrics import crossings

    for name, d in _real_circuit_drawings(real_circuits):
        assert crossings(_lead_polylines(d)) == 0, name


def _wire_lead_contacts(wires, leads, grid):
    from metrics import collinear_overlaps, tight_parallel_pairs, wire_lead_pairs

    return (
        wire_lead_pairs(collinear_overlaps, wires, leads),
        wire_lead_pairs(lambda ws: tight_parallel_pairs(ws, grid), wires, leads),
    )


def test_wire_lead_contacts_are_counted():
    # Negative control for the helper below: a wire laid over a lead and one
    # a single row beside it must both register, or its zeros mean nothing.
    lead = [(0.0, 0.0), (4.0, 0.0)]
    assert _wire_lead_contacts([[(1.0, 0.0), (3.0, 0.0)]], [lead], 0.25) == (1, 0)
    assert _wire_lead_contacts([[(1.0, 0.25), (3.0, 0.25)]], [lead], 0.25) == (0, 1)


def test_no_routed_wire_runs_on_or_beside_a_lead_in_real_circuits(real_circuits):
    # finish()'s ordering score counts wire-lead pairs from the leads drawn
    # before it runs (#593); the lead counts there equal the final drawing's
    # (16/16, 8/8, 34/34). Pin the measured zeros on the final drawing.
    from metrics import drawing_wires

    for name, d in _real_circuit_drawings(real_circuits):
        grid = d._routers[0].grid
        contacts = _wire_lead_contacts(drawing_wires(d), _lead_polylines(d), grid)
        assert contacts == (0, 0), f"{name}: wire-lead (collinear, tight) {contacts}"


def test_every_junction_in_real_circuits_joins_one_net_class(real_circuits):
    # A dot takes one colour; wires of two classes meeting at it means a lead
    # was left uncoloured or a net mis-classed — a drawing error, so fail.
    from routing import _on_wire

    for name, d in _real_circuit_drawings(real_circuits):
        coloured = [(w.points, w.color) for r in d._routers for w in r._wires] + [
            (pts, c) for r in d._routers for pts, c in r._leads()
        ]
        for p in _dot_points(d):
            colours = sorted({c for pts, c in coloured if _on_wire(p, pts)})
            assert len(colours) == 1, f"{name}: junction at {p} joins {colours}"


def test_a_later_finish_rehops_a_wire_drawn_by_an_earlier_one():
    # finish() may run more than once. A wire drawn by the first call that a
    # later wire crosses is the one that must hop (it is horizontal), so its
    # Path is redrawn in place — same element, same paint position.
    d, router = _free_router()
    h = router.wire((0.0, 0.0), (4.0, 0.0), net="i2c")
    router.finish()
    first = h.element
    assert "C" not in _curve_path(first)
    router.wire((2.0, -2.0), (2.0, 2.0), net="pwm")
    router.finish()
    assert h.element is first
    assert h.hops == [(2.0, 0.0)]
    assert "C" in _curve_path(first)
    assert sum(isinstance(el, type(first)) for el in d.elements) == 2


def test_a_real_t_at_a_tag_point_keeps_its_dot():
    # Only the tag-terminated lead's end is discounted, not the whole point:
    # a routed wire ending on another that runs through the tag point is a
    # genuine T there and must still be dotted. The tag cost (#591) is off so
    # the first wire still runs through the tag point.
    d, router = _free_router(tag_penalty=0.0)
    d.add(elm.Line().right(2.0).at((0.0, 0.0)))
    d.add(elm.Ground())
    router.wire((2.0, -2.0), (2.0, 2.0), net="ground")
    router.wire((4.0, 0.0), (2.0, 0.0), net="ground")
    router.finish()
    assert router.junctions == [(2.0, 0.0)]


def test_a_lead_with_float_noise_is_marked_on_its_own_axis():
    # A lead's points come through a schemdraw transform, so a horizontal
    # stub can end at y = 1.0000000000000002. Classified by exact equality it
    # marked as a *vertical* run, and balancebot's GPIO3 net paid a phantom
    # overlap charge to cross the left DRV8825's GND stub.
    from routing import _mark_cells

    occupied = {}
    _mark_cells(occupied, [(12.25, 1.0), (11.25, 1.0000000000000002)], 0.25)
    assert occupied and all(axes == ("H",) for axes in occupied.values())


# -- net ordering and occupancy order (#494) ------------------------------------
#
# The router is greedy and first-come, so the order nets are routed in decides
# which of them detours. finish() re-routes the recorded batch under a fixed
# list of candidate orderings and keeps the best-scoring finished set; every
# part of that choice must be a pure function of the circuit, because CI
# compares the SVG bytes.


def test_occupancy_records_axes_as_sorted_tuples():
    # A set's iteration order depends on string hashing, which is salted per
    # process; a value that is never iterated today is one refactor away from
    # being iterated into the output. Tuples, sorted, so there is no order to
    # depend on.
    _, router = _free_router()
    router.wire((0.0, 1.0), (2.0, 1.0))
    router.wire((1.0, 0.0), (1.0, 2.0))
    router.finish()
    values = list(router._occupied.values())
    assert values and all(isinstance(v, tuple) for v in values)
    assert all(list(v) == sorted(set(v)) for v in values)
    assert router._occupied[(4, 4)] == ("H", "V")  # the crossing cell


def _l_net_then_straight_net(router):
    # Authored order routes the L-shaped net first, along the row the second
    # net needs for its straight run, so the straight net has to detour round
    # it. Routing the straight net first leaves the L net a free path.
    first = router.wire((4.0, 1.0), (1.0, 3.5))
    second = router.wire((5.0, 1.0), (1.0, 1.0))
    return first, second


def test_finish_reroutes_in_a_better_order_than_authored():
    from metrics import crossings, total_length

    d, router = _free_router()
    first, second = _l_net_then_straight_net(router)
    greedy = [first.points, second.points]
    drawn = router.finish()

    assert router.ordering not in (None, "authored")
    assert second.points == [(5.0, 1.0), (1.0, 1.0)], "straight net still detours"
    finished = [first.points, second.points]
    assert (crossings(finished), total_length(finished)) < (
        crossings(greedy),
        total_length(greedy),
    )
    # Only the routing order moves: the paint order stays as authored. (The
    # L net starts on the straight one, so a junction dot follows them.)
    assert drawn == [first.element, second.element]
    assert d.elements.index(first.element) < d.elements.index(second.element)
    assert _abspoints(first.element) == first.points
    assert _abspoints(second.element) == second.points


def test_finish_keeps_authored_order_when_no_candidate_is_better():
    # Two nets that never meet score the same in every order, and the tie
    # goes to the first candidate: the circuit exactly as its author wrote it.
    _, router = _free_router()
    a = router.wire((0.0, 0.0), (3.0, 0.0))
    b = router.wire((0.0, 5.0), (3.0, 5.0))
    before = [a.points, b.points]
    router.finish()
    assert router.ordering == "authored"
    assert [a.points, b.points] == before


def test_a_net_wired_after_finish_avoids_the_rerouted_batch():
    # finish() replaces the occupancy with the chosen ordering's, so a net
    # routed after it avoids where the batch finally lies, not where the
    # greedy pass first put it. The L net's greedy route ran up x = 1; the
    # chosen ordering moves it to x = 4. A later net wanting x = 4 must see
    # it there — with the greedy occupancy kept it runs straight on top.
    from metrics import collinear_overlaps

    _, router = _free_router()
    first, second = _l_net_then_straight_net(router)
    router.finish()
    assert first.points == [(4.0, 1.0), (4.0, 3.5), (1.0, 3.5)]
    later = router.wire((4.0, 4.5), (4.0, 2.0))
    assert later.points != [(4.0, 4.5), (4.0, 2.0)], "ran over the moved L net"
    assert collinear_overlaps([first.points, second.points, later.points]) == 0


def test_a_wire_without_a_plan_is_routed_round_and_kept_in_occupancy():
    # No code path records a wire without a _Plan today, but finish() swaps
    # in the chosen ordering's occupancy wholesale: a plan-less undrawn wire
    # left out of the candidates' starting occupancy would vanish from it,
    # and every later net would route straight over it.
    from routing import RoutedWire, _mark_cells

    _, router = _free_router()
    _l_net_then_straight_net(router)
    fixed = RoutedWire([(0.0, 2.0), (6.0, 2.0)])
    router._wires.append(fixed)
    router.finish()
    cells = {}
    _mark_cells(cells, fixed.points, router.grid)
    for cell, axes in cells.items():
        assert set(axes) <= set(router._occupied.get(cell, ())), cell


def test_ordering_choice_is_reproducible_across_hash_seeds():
    # "Across runs and machines": the one input that differs between two
    # otherwise identical runs is the string-hash salt, so route the densest
    # circuit under two salts in fresh interpreters and require the same
    # ordering, the same geometry and the same SVG bytes, hop and dot markup
    # included. A forward guard rather than a
    # regression proof: the pre-change router never iterated a set into its
    # output either, so only the "authored" assertion depends on #494. It is
    # here so a future candidate or tie-break keyed on a set or a hash cannot
    # land without failing.
    import os
    import subprocess

    script = (
        "import sys, contextlib, json; sys.path.insert(0, '.');"
        "from render import circuit_files, load_circuit;"
        "f = contextlib.redirect_stdout(sys.stderr); f.__enter__();"
        "d = load_circuit(circuit_files(['robocar_unified'])[0]).draw();"
        "f.__exit__(None, None, None);"
        "import hashlib;"
        "svg = hashlib.sha256(d.get_imagedata('svg')).hexdigest();"
        "print(json.dumps([svg, [[r.ordering, [w.points for w in r._wires]]"
        " for r in d._routers]]))"
    )
    here = str(FsPath(__file__).parent)
    # Both interpreters run at once: each is an independent process, so
    # overlapping them halves the wall time without sharing any state (#594).
    runs = [
        subprocess.Popen(
            [sys.executable, "-c", script],
            cwd=here,
            env={**os.environ, "PYTHONHASHSEED": seed},
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        for seed in ("0", "12345")
    ]
    # Collect both before asserting, so a failure never leaves one running.
    results = [(run, *run.communicate()) for run in runs]
    outputs = []
    for run, stdout, stderr in results:
        assert run.returncode == 0, stderr
        outputs.append(stdout)
    assert outputs[0] == outputs[1]
    assert '"authored"' not in outputs[0], "robocar_unified no longer reorders"


def test_ordering_score_sees_a_tight_pair_against_a_lead(monkeypatch):
    # finish() draws no lead of its own, but a circuit's hand-drawn leads are
    # already in the drawing when it chooses an ordering, and a wire a row
    # beside one reads as a tight pair just like a wire-wire one (#593). Two
    # candidate orderings are the whole search here: authored routes ``b``
    # one row from the lead, reversed routes it clear.
    import routing
    from metrics import tight_parallel_pairs, wire_lead_pairs

    by_name = dict(routing.ORDERINGS)
    monkeypatch.setattr(
        routing,
        "ORDERINGS",
        (("authored", by_name["authored"]), ("reversed", by_name["reversed"])),
    )
    scored = []
    real_score = routing._order_score

    def spy(wires, grid, *args, **kwargs):
        scored.append(wires)
        return real_score(wires, grid, *args, **kwargs)

    monkeypatch.setattr(routing, "_order_score", spy)

    d, router = _free_router()
    lead = [(0.0, 0.0), (6.0, 0.0)]
    d.add(elm.Wire("-").at(lead[0]).to(lead[1]))
    a = router.wire((0.5, 1.0), (1.0, 1.5), net="signal")
    b = router.wire((0.5, 0.75), (4.5, -0.25), net="signal")
    router.finish()

    # Precondition: the two candidates tie on wire-wire score alone, so only
    # the lead term can separate them and the assertion below is not vacuous.
    wire_only = [real_score(w, router.grid) for w in scored]
    assert len(wire_only) == 2 and wire_only[0] == wire_only[1]

    wires = [a.points, b.points]
    grid = router.grid
    tight = wire_lead_pairs(lambda ws: tight_parallel_pairs(ws, grid), wires, [lead])
    assert tight == 0, f"{router.ordering}: {tight} wire-lead tight pair(s)"
    assert router.ordering == "reversed"


def test_ordering_score_sees_a_crossing_against_a_lead(monkeypatch):
    # The crossing term of the same lead score (#593): the candidates tie on
    # wire-wire score, and authored crosses the vertical lead where reversed
    # does not.
    import routing
    from metrics import crossings, wire_lead_pairs

    by_name = dict(routing.ORDERINGS)
    monkeypatch.setattr(
        routing,
        "ORDERINGS",
        (("authored", by_name["authored"]), ("reversed", by_name["reversed"])),
    )
    scored = []
    real_score = routing._order_score

    def spy(wires, grid, *args, **kwargs):
        scored.append(wires)
        return real_score(wires, grid, *args, **kwargs)

    monkeypatch.setattr(routing, "_order_score", spy)

    d, router = _free_router()
    lead = [(1.0, -2.0), (1.0, 3.0)]
    d.add(elm.Wire("-").at(lead[0]).to(lead[1]))
    a = router.wire((2.0, 3.0), (3.0, 1.0), net="signal")
    b = router.wire((2.0, 1.5), (0.5, 3.0), net="signal")
    router.finish()

    wire_only = [real_score(w, router.grid) for w in scored]
    assert len(wire_only) == 2 and wire_only[0] == wire_only[1]

    crossed = wire_lead_pairs(crossings, [a.points, b.points], [lead])
    assert crossed == 0, f"{router.ordering}: {crossed} wire-lead crossing(s)"
    assert router.ordering == "reversed"


def test_robocar_unified_tight_parallel_pairs_drop_below_baseline(real_circuit):
    # The #494 baseline, measured by metrics.py with ordering search off
    # (ORDERINGS cut to "authored"): robocar_unified 0 tight parallel pairs,
    # 15 crossings, 298.83 units of wire. The chosen ordering must not lose
    # to it on tight pairs, overlaps, crossings or length.
    # These numbers are a property of robocar_unified.py as it stood, not of
    # the router: an edit to that circuit can trip or loosen this pin, so
    # re-measure the baseline whenever the circuit changes (#463). Last
    # re-measured for #495, which draws the XIAO, TCA9548A, PCA9685,
    # TB6612FNG and MAX98357A physically: the boards are larger and the six
    # motor-driver lines are drawn individually instead of as one trunk, so
    # the wire is longer (217.14 before) while the crossings fell from 23.
    # The chosen ordering (shortest-first) crosses 11 times.
    m = real_circuit("robocar_unified").metrics
    assert m.tight_parallel == 0, f"{m.tight_parallel} tight pairs (baseline 0)"
    assert m.collinear_overlaps == 0
    assert m.crossings <= 15
    assert m.total_length <= 298.8345 + 1e-6


# Label boxes (#681). schemdraw's SegmentText.get_bbox ignores rotation and
# treats align=None as bottom-left, while the SVG backend draws align=None
# centred and rotates about the anchor. The reference below is read off the
# <text> element text_tosvg emits (anchor, dominant-baseline, transform), i.e.
# where the glyphs are drawn. It is not the backend's test-mode <rect>: that
# rectangle sits a line (valign top) or half a line (centre) away from the
# glyphs, so a box matched to it would be offset from the rendered text.

_PT_TO_UNITS = 2 / 72


def _rendered_glyph_box(seg):
    """Where the glyphs of ``seg`` (already transformed) are drawn, in drawing units."""
    import re

    from schemdraw.backends import svgtext

    halign, valign = seg.align or ("center", "center")
    rotation = seg.rotation or 0
    elem = svgtext.text_tosvg(
        seg.text,
        0.0,
        0.0,
        size=seg.fontsize,
        halign=halign,
        valign=valign,
        rotation=rotation,
        rotation_mode=seg.rotation_mode or "anchor",
    )
    w, h, _ = svgtext.text_approx_size(seg.text, size=seg.fontsize)
    size = seg.fontsize
    x, y_attr = float(elem.get("x")), float(elem.get("y"))
    # The first tspan is shifted down one line (dy=size) from the text's y and
    # later ones one line each, so the block spans h from the first line's top.
    baseline_y = y_attr + size
    left = {"start": x, "middle": x - w / 2, "end": x - w}[elem.get("text-anchor")]
    top = {
        "hanging": baseline_y,
        "central": baseline_y - size / 2,
        "ideographic": baseline_y - size,
    }[elem.get("dominant-baseline")]
    corners = [(left, top), (left + w, top), (left, top + h), (left + w, top + h)]
    dx = dy = 0.0
    angle = 0.0
    transform = elem.get("transform")
    if transform:
        shift = re.search(r"translate\(([-\d.e]+) ([-\d.e]+)\)", transform)
        if shift:
            dx, dy = float(shift.group(1)), float(shift.group(2))
        angle = math.radians(
            float(re.search(r"rotate\(([-\d.e]+)", transform).group(1))
        )
    cos, sin = math.cos(angle), math.sin(angle)
    # SVG rotate(a) about the origin (the anchor), then translate; y points down.
    pts = [(px * cos - py * sin + dx, px * sin + py * cos + dy) for px, py in corners]
    x0, y0 = seg.xy
    xs = [x0 + px * _PT_TO_UNITS for px, _ in pts]
    ys = [y0 - py * _PT_TO_UNITS for _, py in pts]
    return min(xs), min(ys), max(xs), max(ys)


def _assert_box_matches_rect(box, rect):
    got = (box.xmin, box.ymin, box.xmax, box.ymax)
    assert all(abs(g - r) < 1e-6 for g, r in zip(got, rect)), f"{got} != {rect}"


def _label_box_of_segment(**kwargs):
    class _Text(elm.Element):
        def __init__(self):
            super().__init__()
            self.segments.append(SegmentText((3.0, 2.0), "GND", fontsize=10, **kwargs))

    d = schemdraw.Drawing(show=False)
    d.add(_Text())
    (label,) = Router(d)._labels()
    el = d.elements[0]
    (seg,) = [s for s in el.segments if isinstance(s, SegmentText)]
    return label.box, seg.xform(el.transform)


def test_svg_text_mode_is_text_so_the_reference_rect_is_valid():
    # With ziamath installed schemdraw draws text as paths and sizes it
    # differently; text_tosvg's rect would then stop describing the render.
    assert schemdraw.svgconfig.text == "text"


def test_rotated_default_mode_label_box_is_the_rendered_rect():
    box, seg = _label_box_of_segment(
        rotation=90, rotation_mode="default", align=("center", "top")
    )
    assert box.ymax - box.ymin > box.xmax - box.xmin, "90 degrees makes it tall"
    _assert_box_matches_rect(box, _rendered_glyph_box(seg))


def test_rotated_anchor_mode_label_box_is_the_rendered_rect():
    box, seg = _label_box_of_segment(
        rotation=90, rotation_mode="anchor", align=("left", "bottom")
    )
    assert box.ymax - box.ymin > box.xmax - box.xmin
    _assert_box_matches_rect(box, _rendered_glyph_box(seg))


def test_unaligned_label_box_is_centred_like_the_render():
    # The backend draws align=None as ('center', 'center'); get_bbox reads it as
    # bottom-left.
    box, seg = _label_box_of_segment(align=None)
    _assert_box_matches_rect(box, _rendered_glyph_box(seg))
    assert box.contains(3.0, 2.0)


@pytest.mark.parametrize("valign", ["top", "center", "bottom"])
@pytest.mark.parametrize("halign", ["left", "center", "right"])
@pytest.mark.parametrize(
    ("rotation", "mode"),
    [(0, "anchor"), (90, "default"), (90, "anchor"), (45, "default")],
)
def test_label_box_is_where_the_glyphs_are_drawn(halign, valign, rotation, mode):
    box, seg = _label_box_of_segment(
        rotation=rotation, rotation_mode=mode, align=(halign, valign)
    )
    _assert_box_matches_rect(box, _rendered_glyph_box(seg))


def test_unaligned_rotated_label_box_is_the_rendered_rect():
    box, seg = _label_box_of_segment(align=None, rotation=90, rotation_mode="default")
    assert box.ymax - box.ymin > box.xmax - box.xmin
    _assert_box_matches_rect(box, _rendered_glyph_box(seg))


def test_every_real_circuit_label_box_is_the_rendered_rect(real_circuits):
    checked = rotated = 0
    unaligned = []
    for circuit in real_circuits:
        # Reuse the router the circuit was measured with: a fresh Router would
        # register itself on the shared drawing and trip the read-only guard.
        router = circuit.drawing._routers[0]
        texts = [
            s.xform(el.transform)
            for el in circuit.drawing.elements
            if not isinstance(el, Path)
            for s in el.segments
            if isinstance(s, SegmentText) and s.text.strip()
        ]
        labels = router._labels()
        assert len(labels) == len(texts), circuit.name
        for label, seg in zip(labels, texts):
            _assert_box_matches_rect(label.box, _rendered_glyph_box(seg))
            checked += 1
            rotated += bool(seg.rotation)
            if seg.align is None:
                unaligned.append(label.box)
                assert label.box.contains(*seg.xy), f"{circuit.name} {seg.text!r}"
                mid = (
                    (label.box.xmin + label.box.xmax) / 2,
                    (label.box.ymin + label.box.ymax) / 2,
                )
                assert max(abs(mid[0] - seg.xy[0]), abs(mid[1] - seg.xy[1])) < 1e-6
    assert rotated >= 11, f"expected the rotated pin labels, saw {rotated}"
    assert checked > rotated
    assert len(unaligned) >= 3, "the Capacitor '+' marks have align=None"


def test_a_pin_on_one_boxs_edge_is_owned_by_that_box_not_an_equal_one_it_is_inside():
    # #592: two boxes of equal area overlap and pin P lies on B's left edge
    # but strictly inside A. An area-only tie-break took A (first of the
    # equals), so P's stub left through B's body.
    a = _BBox(0.0, 0.0, 2.0, 2.0)
    b = _BBox(1.0, 0.0, 3.0, 2.0)
    pin = (1.0, 1.0)
    router = Router(schemdraw.Drawing())
    for boxes in ([a, b], [b, a]):
        owner = router._owning_box(pin, boxes)
        assert owner is b
        assert router._exit_direction(pin, owner) == (-1.0, 0.0)


def test_every_real_circuit_routes_no_wire_inside_a_component_body(real_circuits):
    # #674: gamepad_synth's GPIO9 stub left through Piezo B's body (1.48 units).
    for c in real_circuits:
        assert c.metrics.inside_any == 0, (
            f"{c.name}: {c.metrics.inside_any:.2f} units of wire inside bodies"
        )


def test_no_two_labels_overlap_in_real_circuits(real_circuits):
    # #674: gamepad_synth's bottom-pin labels collided with each other and
    # with the GND / GPIO7 side labels.
    import itertools

    for c in real_circuits:
        # The router the circuit was measured with: a fresh one would register
        # on the shared drawing and trip the read-only guard.
        labels = c.drawing._routers[0]._labels()
        clashes = [
            (a.text, b.text)
            for a, b in itertools.combinations(labels, 2)
            if min(a.box.xmax, b.box.xmax) - max(a.box.xmin, b.box.xmin) > 1e-6
            and min(a.box.ymax, b.box.ymax) - max(a.box.ymin, b.box.ymin) > 1e-6
        ]
        assert not clashes, f"{c.name}: overlapping labels {clashes}"
