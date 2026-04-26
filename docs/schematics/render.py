"""Render circuits in ``circuits/`` to SVG + PNG in ``images/``.

Each file in ``circuits/`` that defines a ``draw() -> Drawing`` function is
rendered. Run with ``just schematics::render`` (all circuits) or
``just schematics::render-one <name>`` (single circuit), or directly via
``python render.py [name ...]``.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import cairosvg

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


def main(argv: list[str] | None = None) -> int:
    names = list(argv) if argv is not None else sys.argv[1:]
    sys.path.insert(0, str(ROOT))
    IMAGES.mkdir(exist_ok=True)

    if names:
        circuit_files = [CIRCUITS / f"{name}.py" for name in names]
        missing = [str(p) for p in circuit_files if not p.is_file()]
        if missing:
            print(f"no such circuit(s): {', '.join(missing)}", file=sys.stderr)
            return 1
    else:
        circuit_files = sorted(CIRCUITS.glob("*.py"))
        if not circuit_files:
            print(f"no circuits found in {CIRCUITS}", file=sys.stderr)
            return 1

    for py in circuit_files:
        if py.stem.startswith("_"):
            continue
        mod = _load_module(py)
        if not hasattr(mod, "draw"):
            print(f"skip {py.name}: no draw() function")
            continue
        svg_path = IMAGES / f"{py.stem}.svg"
        png_path = IMAGES / f"{py.stem}.png"
        mod.draw().save(str(svg_path))
        # schemdraw writes SVG without a trailing newline, which trips the
        # repo's end-of-file-fixer pre-commit hook. Normalize here.
        svg_text = svg_path.read_text()
        if not svg_text.endswith("\n"):
            svg_path.write_text(svg_text + "\n")
        cairosvg.svg2png(
            url=str(svg_path), write_to=str(png_path), output_width=PNG_WIDTH
        )
        print(f"rendered {py.stem}  ->  {svg_path.name}, {png_path.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
