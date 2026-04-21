"""Render every circuit in ``circuits/`` to SVG + PNG in ``images/``.

Each file in ``circuits/`` that defines a ``draw() -> Drawing`` function is
rendered. Run with ``just schematics::render`` or directly via
``python render.py``.
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


def main() -> int:
    sys.path.insert(0, str(ROOT))
    IMAGES.mkdir(exist_ok=True)

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
        cairosvg.svg2png(
            url=str(svg_path), write_to=str(png_path), output_width=PNG_WIDTH
        )
        print(f"rendered {py.stem}  ->  {svg_path.name}, {png_path.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
