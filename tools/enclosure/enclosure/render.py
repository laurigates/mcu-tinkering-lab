"""Isometric line drawings of generated cases, for review without a CAD viewer.

    python -m enclosure.render <file.step> [<file.step> ...]

Writes <file>.svg beside each STEP: visible edges solid, hidden edges dashed.
"""

from __future__ import annotations

import sys
from pathlib import Path

from build123d import ExportSVG, LineType, import_step


def render(step: Path, view=(-1.0, -1.4, 1.1)) -> Path:
    part = import_step(step)
    bb = part.bounding_box()
    size = max(*bb.size)
    # project_to_viewport takes a camera position, not a direction: stand
    # well back from the centre along `view`.
    c = bb.center()
    centre = (c.X, c.Y, c.Z)
    eye = tuple(centre[i] + 4 * size * view[i] for i in range(3))
    visible, hidden = part.project_to_viewport(eye, look_at=centre)
    svg = ExportSVG(scale=200 / size)
    svg.add_layer("hidden", line_color=(160, 160, 160), line_type=LineType.ISO_DOT)
    svg.add_layer("visible", line_color=(20, 20, 20))
    svg.add_shape(hidden, layer="hidden")
    svg.add_shape(visible, layer="visible")
    out = step.with_suffix(".svg")
    svg.write(str(out))
    return out


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        print(render(Path(arg)))
