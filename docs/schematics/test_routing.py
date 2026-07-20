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


def test_all_real_circuits_route_orthogonally_without_crossing_components():
    # A wire legitimately touches the boundary of its own source/destination
    # component (that's where the pin lives) — exclude those two boxes per
    # wire, same as Router.wire() does internally, so only genuine
    # cut-through-an-unrelated-component cases fail this check.
    checked = 0
    for name, abspts, obstacles in _all_real_circuit_paths():
        _segments_are_orthogonal(abspts)
        relevant = [
            box
            for box in obstacles
            if not _contains_endpoint(box, abspts[0])
            and not _contains_endpoint(box, abspts[-1])
        ]
        for p1, p2 in zip(abspts, abspts[1:]):
            for box in relevant:
                assert not _segment_hits_box(p1, p2, box), (
                    f"{name}: routed segment {p1} -> {p2} cuts through {box}"
                )
        checked += 1
    assert checked > 0, "no circuits were found to check"
