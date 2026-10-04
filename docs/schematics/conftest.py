"""Shared fixtures for the schematics test suites.

Routing a real circuit is the expensive step in this suite: ``finish()``
routes every batch under each candidate in ``ORDERINGS`` (#494), and eleven
read-only tests used to load and route the same circuits independently
(#594). ``real_circuits`` routes each one once per session and hands every
test the same finished drawing.

Sharing a mutable ``Drawing`` couples every test that reads it, so the
function-scoped fixture fingerprints each drawing (and its metrics) after
the test that used it and fails that test if anything changed. Tests that must route afresh —
determinism, hash seeds, parameter comparisons — still do so themselves.
"""

from __future__ import annotations

import contextlib
import hashlib
import sys
from dataclasses import dataclass
from pathlib import Path as FsPath

import pytest

sys.path.insert(0, str(FsPath(__file__).parent))

from metrics import CircuitMetrics, measure_drawing  # noqa: E402
from render import circuit_files, draw_circuit, load_circuit  # noqa: E402


@dataclass(frozen=True)
class RenderedCircuit:
    """One real circuit, loaded, routed and measured once for the session.

    ``svg`` is the image as first rendered, so a test comparing a fresh
    render against it compares two independent routes of the circuit.
    ``drawing`` and ``metrics`` are shared: read them, never change them.
    """

    name: str
    drawing: object
    metrics: CircuitMetrics
    svg: bytes


def drawing_fingerprint(d) -> str:
    """A digest of everything a test can read off a finished drawing.

    The SVG alone is not enough: schemdraw caches the rendered figure, so
    an element changed in place would still serialise to the old bytes.
    Dropping the cache forces a re-render from the current elements (cheap,
    ~10 ms for the densest circuit, and byte-identical when nothing moved).
    The router state covers what tests read without going through the
    image — recorded points, hops, net classes, junctions, chosen ordering.
    """
    d.fig = None
    digest = hashlib.sha256(d.get_imagedata("svg"))
    digest.update(str(len(d.elements)).encode())
    for r in getattr(d, "_routers", ()):
        digest.update(repr((r.ordering, r.grid, r.junctions)).encode())
        for w in r._wires:
            digest.update(repr(w).encode())
    return digest.hexdigest()


def circuit_fingerprint(c: RenderedCircuit) -> str:
    """``drawing_fingerprint`` plus the shared metrics, which are mutable too."""
    digest = hashlib.sha256(drawing_fingerprint(c.drawing).encode())
    digest.update(repr(c.metrics).encode())
    return digest.hexdigest()


def mutated_circuits(circuits, fingerprints: dict[str, str]) -> list[str]:
    """Names of the circuits whose fingerprint moved since ``fingerprints``.

    Re-baselines ``fingerprints`` as it goes, so a mutation is blamed on the
    test that made it only; the run is red either way, and every later test
    would otherwise report the same mutation as its own.
    """
    now = {c.name: circuit_fingerprint(c) for c in circuits}
    changed = [name for name, digest in now.items() if digest != fingerprints[name]]
    fingerprints.update(now)
    return changed


@pytest.fixture(scope="session")
def _rendered_real_circuits():
    circuits = []
    for path in circuit_files([]):
        # load_circuit reports skipped files on stdout, as in metrics.py.
        with contextlib.redirect_stdout(sys.stderr):
            mod = load_circuit(path)
        if mod is None:
            continue
        d = draw_circuit(mod)
        # measure_drawing() registers a probe Router on ``d``, so measure
        # before taking the fingerprint the tests are checked against.
        metrics = measure_drawing(path.stem, d)
        circuits.append(RenderedCircuit(path.stem, d, metrics, d.get_imagedata("svg")))
    assert circuits, "no real circuits were found"
    return tuple(circuits), {c.name: circuit_fingerprint(c) for c in circuits}


@pytest.fixture
def real_circuits(_rendered_real_circuits):
    """Every real circuit, routed once per session; fails a test that mutates one."""
    circuits, fingerprints = _rendered_real_circuits
    yield circuits
    changed = mutated_circuits(circuits, fingerprints)
    if changed:
        pytest.fail(
            f"test mutated the shared circuit(s) {changed}; real_circuits is "
            "read-only — route a fresh copy with draw_circuit(load_circuit(...))",
            pytrace=False,
        )


@pytest.fixture
def real_circuit(real_circuits):
    """Look up one shared real circuit by name."""
    by_name = {c.name: c for c in real_circuits}
    return by_name.__getitem__
