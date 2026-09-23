"""Tests for the Manhattan auto-router in routing.py.

Covers the router's core invariants directly (orthogonality, obstacle
avoidance, exact endpoints) and then re-checks every wire actually drawn by
the real circuits, so a future circuit edit that breaks routing fails here
instead of only showing up as a visual glitch in the rendered SVG.
"""

import sys
from pathlib import Path as FsPath

import schemdraw
import schemdraw.elements as elm

sys.path.insert(0, str(FsPath(__file__).parent))
sys.path.insert(0, str(FsPath(__file__).parent / "circuits"))

from components import esp32_s3_zero, max98357a  # noqa: E402
from routing import Router, _BBox  # noqa: E402


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


def _build_two_chip_drawing():
    """Two chips with a ~7-unit gap of free space between their facing edges.

    The obstacle coordinates in the tests below are absolute, so they assume
    that gap. The offset is therefore tied to max98357a()'s body width — when
    the part was widened from 4 to 6, this had to go from 9 to 10 to keep the
    amp's left edge at the same x. If a test starts failing because a blocker
    unexpectedly swallows a pin stub, check that coupling first.
    """
    d = schemdraw.Drawing(show=False)
    d.config(unit=2.0, fontsize=12)
    esp = d.add(esp32_s3_zero().label("ESP32-S3-Zero", loc="bot", ofst=0.4))
    amp = d.add(
        max98357a()
        .at((esp.center.x + 10, esp.center.y))
        .anchor("center")
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
        .at((5.75, esp.GPIO6[1] + 0.2))
        .anchor("center")
    )
    blocker_box = _BBox(*d.elements[-1].get_bbox(transform=True, includetext=False))

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
        .at((esp.center.x + 4.5, esp.center.y + 0.2))
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


def _box_of(el) -> _BBox:
    return _BBox(*el.get_bbox(transform=True, includetext=False))


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


def _all_real_circuit_paths():
    import importlib.util

    from render import draw_circuit

    circuits_dir = FsPath(__file__).parent / "circuits"
    for py in sorted(circuits_dir.glob("*.py")):
        spec = importlib.util.spec_from_file_location(py.stem, py)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        if not hasattr(mod, "draw"):
            continue
        d = draw_circuit(mod)
        from routing import Path, Router

        obstacles = [
            _BBox(*el.get_bbox(transform=True, includetext=False))
            for el in d.elements
            if not isinstance(el, Router._NON_OBSTACLES)
        ]
        for el in d.elements:
            if not isinstance(el, Path):
                continue
            yield py.stem, list(el.polyline), obstacles


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


def test_all_real_circuits_route_orthogonally_without_crossing_components():
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
    for name, abspts, obstacles in _all_real_circuit_paths():
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


def test_robocar_unified_wire_stays_out_of_component_bodies():
    # The whole-drawing figure behind #490: with every terminal box dropped
    # for the entire search, robocar_unified carried ~26 units of wire
    # inside component bodies — nets entering their own chip on one side
    # and leaving on the other. Once only a stub may cross a body the
    # figure is 0.00 here; the issue's own target was ~3 units, the length
    # a few stub leads through boxes overlapping their pins would add. The
    # bound admits that legitimate case, not a tunnel coming back.
    from metrics import measure_circuits

    (m,) = measure_circuits(["robocar_unified"])
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
    # robocar_unified, the densest drawing, must route differently at 6
    # than at 60.
    low = _route_circuit_with_overlap_penalty(monkeypatch, "robocar_unified", 6.0)
    high = _route_circuit_with_overlap_penalty(monkeypatch, "robocar_unified", 60.0)
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


def _free_router():
    """A router on an empty drawing: no boxes, so no stubs and straight runs."""
    d = schemdraw.Drawing(show=False)
    d.config(unit=2.0)
    return d, Router(d)


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


def _real_circuit_drawings():
    from render import circuit_files, draw_circuit, load_circuit

    for path in circuit_files([]):
        mod = load_circuit(path)
        if mod is not None:
            yield path.stem, draw_circuit(mod)


def test_every_real_wire_has_a_net_class():
    from routing import NET_COLORS

    seen = 0
    for name, d in _real_circuit_drawings():
        for router in d._routers:
            for w in router._wires:
                assert w.net in NET_COLORS, f"{name}: {w.points[0]} has no net class"
                assert w.element.params["color"] != "steelblue"
                seen += 1
    assert seen > 0


def test_every_real_crossing_carries_a_hop():
    # Hops between two routed wires, against the metric that counts exactly
    # those crossings. Hops over hand-drawn leads are checked further down.
    from metrics import crossings, drawing_wires
    from routing import _on_wire

    for name, d in _real_circuit_drawings():
        routed = [w.points for r in d._routers for w in r._wires]
        hops = [
            p
            for r in d._routers
            for w in r._wires
            for p in w.hops
            if sum(_on_wire(p, o) for o in routed) == 2
        ]
        assert len(hops) == crossings(drawing_wires(d)), name


def test_robocar_unified_renders_byte_identically_twice():
    from render import circuit_files, draw_circuit, load_circuit

    (path,) = circuit_files(["robocar_unified"])
    first = draw_circuit(load_circuit(path)).get_imagedata("svg")
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


def test_every_crossing_with_a_routed_wire_is_hopped_in_real_circuits():
    # Over the *final* drawing, so a lead added after finish() that crosses
    # a routed wire — and so was never seen by it — fails here too.
    from routing import _axis_segments, _on_wire

    for name, d in _real_circuit_drawings():
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
    d, router = _free_router()
    d.add(elm.Line().right(2.0).at((0.0, 0.0)))
    d.add(elm.Ground())
    router.wire((2.0, -2.0), (2.0, 2.0), net="signal")
    router.finish()
    assert router.junctions == []
    assert not [el for el in d.elements if isinstance(el, elm.Dot)]


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


def test_every_hop_in_real_circuits_is_drawn_clear_and_full_size():
    # A hop must be visible: clear of every junction dot (a dot over an arc
    # turns a crossing into a connection), clear of every other wire's end or
    # bend (the arc would merge into the corner), and far enough from its own
    # segment's ends not to shrink into an unreadable bump.
    import math

    from routing import HOP_RADIUS, JUNCTION_RADIUS, _on_wire

    for name, d in _real_circuit_drawings():
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


def test_no_two_hand_drawn_leads_cross_in_real_circuits():
    # finish() hops every crossing that involves a routed wire; a crossing
    # between two leads it cannot redraw would carry no hop at all. None
    # exists today — this keeps "every crossing carries a hop" true.
    from metrics import crossings

    for name, d in _real_circuit_drawings():
        assert crossings(_lead_polylines(d)) == 0, name


def _wire_lead_contacts(wires, leads, grid):
    # Pairs with one routed wire and one lead only: measure the mixed set
    # and take away what each side scores on its own.
    from metrics import collinear_overlaps, tight_parallel_pairs

    def mixed(count):
        return count(wires + leads) - count(wires) - count(leads)

    return (
        mixed(collinear_overlaps),
        mixed(lambda ws: tight_parallel_pairs(ws, grid)),
    )


def test_wire_lead_contacts_are_counted():
    # Negative control for the helper below: a wire laid over a lead and one
    # a single row beside it must both register, or its zeros mean nothing.
    lead = [(0.0, 0.0), (4.0, 0.0)]
    assert _wire_lead_contacts([[(1.0, 0.0), (3.0, 0.0)]], [lead], 0.25) == (1, 0)
    assert _wire_lead_contacts([[(1.0, 0.25), (3.0, 0.25)]], [lead], 0.25) == (0, 1)


def test_no_routed_wire_runs_on_or_beside_a_lead_in_real_circuits():
    # finish()'s ordering score sees routed wires only, and two circuits
    # draw leads after routing, so neither the plan's lead snapshot nor the
    # score knows about them. A candidate ordering could therefore trade a
    # wire-wire pair for a wire-lead one unseen; pin the measured zeros.
    from metrics import drawing_wires

    for name, d in _real_circuit_drawings():
        grid = d._routers[0].grid
        contacts = _wire_lead_contacts(drawing_wires(d), _lead_polylines(d), grid)
        assert contacts == (0, 0), f"{name}: wire-lead (collinear, tight) {contacts}"


def test_every_junction_in_real_circuits_joins_one_net_class():
    # A dot takes one colour; wires of two classes meeting at it means a lead
    # was left uncoloured or a net mis-classed — a drawing error, so fail.
    from routing import _on_wire

    for name, d in _real_circuit_drawings():
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
    # genuine T there and must still be dotted.
    d, router = _free_router()
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
    outputs = []
    for seed in ("0", "12345"):
        env = {**os.environ, "PYTHONHASHSEED": seed}
        run = subprocess.run(
            [sys.executable, "-c", script],
            cwd=here,
            env=env,
            capture_output=True,
            text=True,
            check=True,
        )
        outputs.append(run.stdout)
    assert outputs[0] == outputs[1]
    assert '"authored"' not in outputs[0], "robocar_unified no longer reorders"


def test_robocar_unified_tight_parallel_pairs_drop_below_baseline():
    # The #494 baseline, measured by metrics.py on the tree before ordering
    # search: robocar_unified 2 tight parallel pairs, 24 crossings, 206.39
    # units of wire. The chosen ordering must beat it on tight pairs without
    # paying for it in overlaps, crossings or length. These numbers are a
    # property of robocar_unified.py as it stood, not of the router: an
    # edit to that circuit can trip or loosen this pin, so re-measure the
    # baseline (ordering search off) whenever the circuit changes (#463).
    from metrics import measure_circuits

    (m,) = measure_circuits(["robocar_unified"])
    assert m.tight_parallel <= 1, f"{m.tight_parallel} tight pairs (baseline 2)"
    assert m.collinear_overlaps == 0
    assert m.crossings <= 24
    assert m.total_length <= 206.39 + 1e-6
