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
    el = router.wire(esp.GPIO5, amp.BCLK, color="steelblue")
    points = el.segments[0].path
    at = el._userparams["at"]
    abspoints = [(at[0] + x, at[1] + y) for x, y in points]

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
    el = router.wire(esp.GPIO6, amp.LRC, color="steelblue")
    points = el.segments[0].path
    at = el._userparams["at"]
    abspoints = [(at[0] + x, at[1] + y) for x, y in points]

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
        router.wire(esp.GPIO6, amp.LRC, color="steelblue")
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
    at = el._userparams["at"]
    return [(at[0] + x, at[1] + y) for x, y in el.segments[0].path]


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
    el = Router(d).wire(esp.GPIO5, amp.GAIN)
    abspoints = _abspoints(el)
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

    el = Router(d).wire(esp.GPIO5, amp.BCLK)
    abspoints = _abspoints(el)
    assert abspoints[-1] == (bx, by)
    _segments_are_orthogonal(abspoints)


def test_router_wire_defaults_to_visible_stroke():
    # Regression: passing color=None straight through to Path used to omit
    # the SVG stroke attribute entirely, which CSS defaults to `stroke: none`
    # -> an invisible wire (this is how OUT-/OUT+ in gamepad_synth.py are
    # called: no color= argument).
    d, esp, amp = _build_two_chip_drawing()
    router = Router(d)
    el = router.wire(esp.GPIO5, amp.BCLK)
    assert el.params.get("color") is not None


def _all_real_circuit_paths():
    import importlib.util

    circuits_dir = FsPath(__file__).parent / "circuits"
    for py in sorted(circuits_dir.glob("*.py")):
        spec = importlib.util.spec_from_file_location(py.stem, py)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        if not hasattr(mod, "draw"):
            continue
        d = mod.draw()
        from routing import Path, Router

        obstacles = [
            _BBox(*el.get_bbox(transform=True, includetext=False))
            for el in d.elements
            if not isinstance(el, Router._NON_OBSTACLES)
        ]
        for el in d.elements:
            if not isinstance(el, Path):
                continue
            pts = el.segments[0].path
            at = el._userparams["at"]
            abspts = [(at[0] + x, at[1] + y) for x, y in pts]
            yield py.stem, abspts, obstacles


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
