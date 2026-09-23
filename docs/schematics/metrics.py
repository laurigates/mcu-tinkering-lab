"""Measure routing quality of every circuit in ``circuits/``.

The router's defects (issue #489, ADR-023) are about how the finished drawing
reads — wires through chip bodies, crossings that look like joins, parallel
runs too close to tell apart — and none of that shows up as a test failure or
an SVG diff until someone looks. This harness turns each of them into a
number, so a router change can state its effect as a before/after instead of
"looks better".

Run with ``just schematics::metrics`` or ``uv run python metrics.py [name ...]
[--json]``. Circuits are discovered and imported by ``render.py``'s own
loader, so the set measured is exactly the set rendered.

What is measured
----------------

A *wire* is the absolute polyline of one :class:`routing.Path` — one call to
``Router.wire()``. Hand-drawn leads (``elm.Line``/``elm.Wire`` power tags and
stubs) are not wires here: the router did not place them. A *box* is a
component body exactly as ``Router._component_boxes()`` computes it — raw,
without the ``clearance`` inflation the search adds. Every segment is
axis-aligned (``test_routing.py`` enforces that); a diagonal is an error.

``total_length``
    Sum of every segment length of every wire.
``inside_any``
    Length of wire lying strictly inside the *interior* of any box. A
    segment running along a box edge is outside; overlapping boxes count
    once (the union of the covered intervals per segment, not the sum).
``inside_foreign``
    As ``inside_any``, but for each wire ignoring the boxes it terminates on
    — every box whose closed extent contains the wire's first or last point,
    the same set ``Router.wire()`` drops from its obstacle list. Wire running
    through its *own* chip is ``inside_any - inside_foreign``.
``crossings``
    Pairs of segments from two distinct wires, one horizontal and one
    vertical, whose intersection lies strictly inside both. A T (one
    segment ending on the other) or a shared endpoint is not a crossing, and
    a wire never crosses itself for this count.
``tight_parallel``
    Pairs of same-axis segments from two distinct wires whose perpendicular
    separation is greater than zero and at most one router grid step, and
    whose extents along the axis overlap by a positive length.
``collinear_overlaps``
    Pairs of same-axis segments from two distinct wires on the same line
    (separation zero) whose extents overlap by a positive length — drawn on
    top of each other, which is worse than tight and so counted apart.
``junctions``
    Points where three or more wire *ends* coincide. Only ends: a wire
    ending on another wire's interior (a T) is not counted.

All coordinate comparisons use a ``1e-6`` tolerance, the same as
``routing._EPS``, because pin and grid coordinates reach the polyline along
independent float paths.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import sys
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path as FsPath

ROOT = FsPath(__file__).parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from render import circuit_files, draw_circuit, load_circuit  # noqa: E402
from routing import _EPS, Path, Router, _BBox, assert_finished  # noqa: E402

Coord = tuple[float, float]
Wire = list[Coord]
# The router's own box type, so ``Box.contains`` is the very predicate
# ``Router.wire()`` uses to decide which boxes a wire terminates on.
Box = _BBox


@dataclass(frozen=True)
class _Seg:
    """An axis-aligned segment: ``axis`` "H" runs along x at y=``fixed``."""

    axis: str
    fixed: float
    lo: float
    hi: float


def _segments(wire: Wire) -> list[_Seg]:
    segs = []
    for (ax, ay), (bx, by) in zip(wire, wire[1:]):
        if abs(ay - by) <= _EPS and abs(ax - bx) <= _EPS:
            continue  # zero-length: contributes nothing to any metric
        if abs(ay - by) <= _EPS:
            segs.append(_Seg("H", ay, min(ax, bx), max(ax, bx)))
        elif abs(ax - bx) <= _EPS:
            segs.append(_Seg("V", ax, min(ay, by), max(ay, by)))
        else:
            raise ValueError(f"diagonal segment {(ax, ay)} -> {(bx, by)}")
    return segs


# -- individual metrics ---------------------------------------------------------


def total_length(wires: list[Wire]) -> float:
    return sum(s.hi - s.lo for w in wires for s in _segments(w))


def _covered(seg: _Seg, boxes: list[Box]) -> float:
    """Length of ``seg`` inside the union of the boxes' open interiors."""
    intervals = []
    for b in boxes:
        if seg.axis == "H":
            f_lo, f_hi, a_lo, a_hi = b.ymin, b.ymax, b.xmin, b.xmax
        else:
            f_lo, f_hi, a_lo, a_hi = b.xmin, b.xmax, b.ymin, b.ymax
        if not (f_lo + _EPS < seg.fixed < f_hi - _EPS):
            continue  # on an edge or beside the box: never inside
        lo, hi = max(seg.lo, a_lo), min(seg.hi, a_hi)
        if hi - lo > _EPS:
            intervals.append((lo, hi))
    total = 0.0
    cur_lo = cur_hi = None
    for lo, hi in sorted(intervals):
        if cur_hi is None or lo > cur_hi:
            if cur_hi is not None:
                total += cur_hi - cur_lo
            cur_lo, cur_hi = lo, hi
        else:
            cur_hi = max(cur_hi, hi)
    if cur_hi is not None:
        total += cur_hi - cur_lo
    return total


def length_inside_boxes(
    wires: list[Wire], boxes: list[Box], *, exclude_terminal: bool = False
) -> float:
    """``inside_any``, or ``inside_foreign`` with ``exclude_terminal=True``."""
    total = 0.0
    for w in wires:
        relevant = boxes
        if exclude_terminal:
            relevant = [
                b for b in boxes if not b.contains(*w[0]) and not b.contains(*w[-1])
            ]
        total += sum(_covered(s, relevant) for s in _segments(w))
    return total


def _wire_pairs(wires: list[Wire]):
    """Every segment pair drawn from two distinct wires, in input order."""
    segs = [_segments(w) for w in wires]
    for i in range(len(segs)):
        for j in range(i + 1, len(segs)):
            for a in segs[i]:
                for b in segs[j]:
                    yield a, b


def crossings(wires: list[Wire]) -> int:
    count = 0
    for a, b in _wire_pairs(wires):
        if a.axis == b.axis:
            continue
        if a.lo + _EPS < b.fixed < a.hi - _EPS and b.lo + _EPS < a.fixed < b.hi - _EPS:
            count += 1
    return count


def _parallel_overlap(a: _Seg, b: _Seg) -> float:
    return min(a.hi, b.hi) - max(a.lo, b.lo)


def tight_parallel_pairs(wires: list[Wire], grid: float) -> int:
    count = 0
    for a, b in _wire_pairs(wires):
        if a.axis != b.axis:
            continue
        sep = abs(a.fixed - b.fixed)
        if _EPS < sep <= grid + _EPS and _parallel_overlap(a, b) > _EPS:
            count += 1
    return count


def collinear_overlaps(wires: list[Wire]) -> int:
    count = 0
    for a, b in _wire_pairs(wires):
        if a.axis != b.axis:
            continue
        if abs(a.fixed - b.fixed) <= _EPS and _parallel_overlap(a, b) > _EPS:
            count += 1
    return count


def junctions(wires: list[Wire]) -> int:
    # Rounding to 6 places merges ends that differ only by float noise
    # (0.1 + 0.2 vs 0.3); drawing coordinates are never that close on purpose.
    ends = Counter(
        (round(x, 6) + 0.0, round(y, 6) + 0.0)  # + 0.0 folds -0.0 into 0.0
        for w in wires
        if w
        for x, y in (w[0], w[-1])
    )
    return sum(1 for n in ends.values() if n >= 3)


# -- per circuit ----------------------------------------------------------------


@dataclass
class CircuitMetrics:
    name: str
    wires: int
    total_length: float
    inside_foreign: float
    inside_any: float
    crossings: int
    tight_parallel: int
    collinear_overlaps: int
    junctions: int


def measure(name: str, wires: list[Wire], boxes: list[Box], grid: float):
    return CircuitMetrics(
        name=name,
        wires=len(wires),
        total_length=total_length(wires),
        inside_foreign=length_inside_boxes(wires, boxes, exclude_terminal=True),
        inside_any=length_inside_boxes(wires, boxes),
        crossings=crossings(wires),
        tight_parallel=tight_parallel_pairs(wires, grid),
        collinear_overlaps=collinear_overlaps(wires),
        junctions=junctions(wires),
    )


def drawing_wires(d) -> list[Wire]:
    """Absolute polylines of every routed ``Path`` in ``d``, in drawing order."""
    wires = []
    for el in d.elements:
        if not isinstance(el, Path):
            continue
        ox, oy = el._userparams["at"]
        wires.append([(ox + x, oy + y) for x, y in el.segments[0].path])
    return wires


def measure_drawing(name: str, d, *, grid: float | None = None) -> CircuitMetrics:
    """Measure a finished drawing.

    The boxes come from a throwaway ``Router(d)`` so they are computed by the
    router's own code over the final element set; constructing a Router adds
    nothing to ``d``. ``grid`` defaults to the router's default step, which
    every circuit currently uses — pass it explicitly if a circuit ever
    routes on a different grid.

    Refuses a drawing whose routers still hold undrawn wires: the metrics
    read drawn Paths, so they would silently measure a circuit with its
    nets missing.
    """
    assert_finished(d)
    probe = Router(d)
    return measure(
        name,
        drawing_wires(d),
        probe._component_boxes(),
        probe.grid if grid is None else grid,
    )


def measure_circuits(names: list[str] | None = None) -> list[CircuitMetrics]:
    results = []
    for path in circuit_files(list(names or [])):
        # load_circuit reports skipped files on stdout, which is render.py's
        # progress channel; here stdout is the report (possibly --json).
        with contextlib.redirect_stdout(sys.stderr):
            mod = load_circuit(path)
        if mod is None:
            continue
        results.append(measure_drawing(path.stem, draw_circuit(mod)))
    return sorted(results, key=lambda m: m.name)


# -- output ---------------------------------------------------------------------

_COLUMNS = (
    ("circuit", "name"),
    ("wires", "wires"),
    ("length", "total_length"),
    ("in_foreign", "inside_foreign"),
    ("in_any", "inside_any"),
    ("crossings", "crossings"),
    ("tight_par", "tight_parallel"),
    ("collinear", "collinear_overlaps"),
    ("junctions", "junctions"),
)


def _cell(value) -> str:
    return f"{value:.2f}" if isinstance(value, float) else str(value)


def format_table(results: list[CircuitMetrics]) -> str:
    """A fixed-width table, one row per circuit sorted by name."""
    rows = [[h for h, _ in _COLUMNS]]
    for m in sorted(results, key=lambda m: m.name):
        rows.append([_cell(getattr(m, attr)) for _, attr in _COLUMNS])
    widths = [max(len(r[i]) for r in rows) for i in range(len(_COLUMNS))]
    lines = []
    for r in rows:
        cells = [r[0].ljust(widths[0])]
        cells += [c.rjust(w) for c, w in zip(r[1:], widths[1:])]
        lines.append("  ".join(cells).rstrip())
    return "\n".join(lines)


def to_json(results: list[CircuitMetrics]) -> str:
    rows = []
    for m in sorted(results, key=lambda m: m.name):
        row = asdict(m)
        for k, v in row.items():
            if isinstance(v, float):
                row[k] = round(v, 6)
        rows.append(row)
    return json.dumps(rows, indent=2, sort_keys=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("names", nargs="*", help="circuit names (default: all)")
    parser.add_argument("--json", action="store_true", help="emit JSON")
    args = parser.parse_args(argv)
    try:
        results = measure_circuits(args.names)
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        return 1
    print(to_json(results) if args.json else format_table(results))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
