"""Render circuits in ``circuits/`` to SVG + PNG in ``images/``.

Each file in ``circuits/`` that defines a ``draw() -> Drawing`` function is
rendered. Run with ``just schematics::render`` (all circuits) or
``just schematics::render-one <name>`` (single circuit), or directly via
``python render.py [name ...]``.

A full render (no names) also deletes any image in ``images/`` that no
circuit produces, so removing a circuit removes its SVG and PNG.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).parent
CIRCUITS = ROOT / "circuits"
IMAGES = ROOT / "images"
PNG_WIDTH = 1400


def _load_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[path.stem] = mod
    spec.loader.exec_module(mod)
    return mod


def circuit_files(names: list[str]) -> list[Path]:
    """Resolve circuit names to their ``circuits/<name>.py`` files.

    No names means every circuit, in sorted order so output and error
    ordering never depend on the filesystem. Raises ``FileNotFoundError``
    naming every missing file (or the empty directory). Shared with
    ``metrics.py`` so both tools see exactly the same set of circuits.
    """
    if names:
        files = [CIRCUITS / f"{name}.py" for name in names]
        missing = [str(p) for p in files if not p.is_file()]
        if missing:
            raise FileNotFoundError(f"no such circuit(s): {', '.join(missing)}")
        return files
    files = sorted(CIRCUITS.glob("*.py"))
    if not files:
        raise FileNotFoundError(f"no circuits found in {CIRCUITS}")
    return files


def load_circuit(path: Path):
    """Import ``path`` and return its module, or ``None`` if it is not a circuit.

    ``_``-prefixed files are helpers, and a module without ``draw()`` is not
    renderable; both yield ``None`` (the latter after saying so, as it always
    has).
    """
    if path.stem.startswith("_"):
        return None
    # Circuits import ``components``/``routing`` as top-level modules.
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    mod = _load_module(path)
    if not hasattr(mod, "draw"):
        print(f"skip {path.name}: no draw() function")
        return None
    return mod


def draw_circuit(mod):
    """Call ``mod.draw()`` and return the drawing, refusing one left unfinished.

    ``Router.wire()`` only records a net; ``Router.finish()`` draws it
    (#492). A circuit that forgets ``finish()`` still returns a valid
    drawing — one with every routed wire missing — so every harness that
    renders or measures a circuit goes through here and fails instead.
    """
    from routing import assert_finished

    d = mod.draw()
    try:
        assert_finished(d)
    except RuntimeError as exc:
        raise RuntimeError(f"{mod.__name__}: {exc}") from None
    return d


def remove_orphans(produced: set[str], images: Path = IMAGES) -> list[Path]:
    """Delete every ``images/*.svg`` and ``*.png`` no circuit in ``produced`` wrote.

    A circuit removed from ``circuits/`` would otherwise leave its images
    behind, and the freshness check could not tell them from current ones
    (#721). Returns the deleted paths, sorted.
    """
    if not images.is_dir():
        return []
    orphans = sorted(
        p
        for pattern in ("*.svg", "*.png")
        for p in images.glob(pattern)
        if p.stem not in produced
    )
    for path in orphans:
        path.unlink()
    return orphans


def main(argv: list[str] | None = None) -> int:
    # Imported here rather than at module level so metrics.py (and its tests)
    # can reuse the loader above without needing libcairo present.
    import cairosvg

    names = list(argv) if argv is not None else sys.argv[1:]
    try:
        files = circuit_files(names)
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        return 1
    IMAGES.mkdir(exist_ok=True)

    produced: set[str] = set()
    for py in files:
        mod = load_circuit(py)
        if mod is None:
            continue
        produced.add(py.stem)
        svg_path = IMAGES / f"{py.stem}.svg"
        png_path = IMAGES / f"{py.stem}.png"
        draw_circuit(mod).save(str(svg_path))
        # schemdraw writes SVG without a trailing newline, which trips the
        # repo's end-of-file-fixer pre-commit hook. Normalize here.
        svg_text = svg_path.read_text()
        if not svg_text.endswith("\n"):
            svg_path.write_text(svg_text + "\n")
        cairosvg.svg2png(
            url=str(svg_path), write_to=str(png_path), output_width=PNG_WIDTH
        )
        print(f"rendered {py.stem}  ->  {svg_path.name}, {png_path.name}")
    # Only a full render knows every image a circuit produces; render-one
    # renders a subset, so the images it skipped are not orphans.
    if not names:
        for path in remove_orphans(produced, IMAGES):
            print(f"removed {path.name}  (no circuit produces it)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
