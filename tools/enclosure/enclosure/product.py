"""Product renders: a carrier, its breakouts and its cases, captioned.

    python -m enclosure.product job <project> [<project> ...]
    python -m enclosure.product finish <project> [<project> ...]

`job` writes hardware/enclosure/output/product/product.json: one view per
piece and state (bare carrier, breakouts fitted, each case empty and with the
board in it), its caption, and the instruction an image-edit model gets to
turn the render into a product photo. blender/render_product.py renders each
view to <id>.png (RGBA, transparent background).

`finish` turns those into, per view:

    <id>-plate.png      flattened on a light backdrop, no text: the edit
                        model's source image (text in it would be "kept")
    <id>-captioned.png  the plate with its caption below, for people
    captions.md         every caption and instruction, beside contact-sheet.png

Run after `python -m enclosure.assembly`, whose scene JSON each view points at.
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
import textwrap
import tomllib
from collections import Counter
from pathlib import Path

from kicad.paths import REPO_ROOT

from . import board

SIZE = (1600, 1200)  # 4:3; Fibo 2.0 picks its ~1 MP size from the aspect
SAMPLES = 96
BACKDROP = "#f2f2f0"
CAPTION_CHARS = 100  # 30 pt across the 1600 px plate, 60 px margins
FONTS = (
    "/System/Library/Fonts/Supplemental/Arial.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)

# (id suffix, caption, scene flags). Order is the contact sheet's.
VIEWS = (
    ("carrier", "bare carrier board, sockets fitted", {"board": True, "modules": False}),
    ("board", "carrier board with its breakouts fitted", {"board": True}),
    ("tray", "open tray, empty", {"case": "tray"}),
    ("tray-board", "board seated in the open tray", {"case": "tray", "board": True}),
    ("box-open", "box base and lid, empty", {"case": "box", "lid": "lifted"}),
    ("box-board", "board in the box base, lid lifted", {"case": "box", "board": True, "lid": "lifted"}),
    ("box", "closed box", {"case": "box", "lid": "closed"}),
)  # fmt: skip

# What stays fixed is spelled out: an edit model given only "make it a photo"
# also restyles the parts, the colours and the layout.
KEEP = (
    "Keep the object exactly as it is: the same shape, proportions, layout, "
    "camera angle, part positions and colours. "
)
PHOTO = {
    "case": "Render the case as a matte white 3D-printed resin part with a fine, "
    "even surface texture. ",
    "board": "Render the circuit board as real FR4 with glossy solder mask, crisp "
    "white silkscreen and bright solder joints, and every module, chip and "
    "connector as real electronic parts. ",
}
STYLES = {
    "studio": "Turn this 3D render into a professional studio product photograph on "
    "a seamless light grey backdrop, soft diffused lighting, a gentle contact "
    "shadow, sharp focus.",
    "workbench": "Turn this 3D render into a product photograph on a light oak "
    "workbench, soft window light from the left, shallow depth of field with a "
    "blurred workshop behind.",
    "dark": "Turn this 3D render into a premium product photograph on a dark slate "
    "surface, dramatic low-key lighting with a soft rim light, a subtle reflection.",
}


def _out(project: Path) -> Path:
    return project / "hardware" / "enclosure" / "output"


def _report(project: Path) -> dict:
    rows = json.loads((_out(project) / "report.json").read_text())
    return {(r["piece"], r["style"]): r for r in rows}


def _mm(size: list[float]) -> str:
    return " × ".join(f"{v:g}" for v in size) + " mm"


def job(project: Path) -> Path:
    cfg = tomllib.loads(
        (project / "hardware" / "enclosure" / board.ENCLOSURE_TOML).read_text()
    )
    titles = {p["name"]: p.get("title", p["name"]) for p in cfg["pieces"]}
    report = _report(project)
    views = []
    for scene_path in sorted(_out(project).glob("*-assembly.json")):
        scene = json.loads(scene_path.read_text())
        piece = scene["piece"]
        counts = Counter(m["label"] for m in scene["modules"] if "pcb" in m)
        fitted = [f"{n}× {k}" if n > 1 else k for k, n in sorted(counts.items())]
        for suffix, what, flags in VIEWS:
            has_case = "case" in flags
            facts = []
            if has_case:
                r = report[(piece, flags["case"])]
                facts.append(f"{flags['case']} {_mm(r['size_mm'])}, resin print")
            if flags.get("board") and flags.get("modules", True) and fitted:
                facts.append(", ".join(fitted))
            instruction = KEEP
            if has_case:
                instruction += PHOTO["case"]
            if flags.get("board"):
                instruction += PHOTO["board"]
            views.append(
                {
                    "id": f"{project.name}-{piece}-{suffix}",
                    "scene": str(scene_path),
                    **flags,
                    "title": f"{titles[piece]}: {what}",
                    "caption": facts,  # one line each, wrapped by finish
                    "instructions": {k: instruction + v for k, v in STYLES.items()},
                }
            )
    out = _out(project) / "product"
    out.mkdir(parents=True, exist_ok=True)
    path = out / "product.json"
    path.write_text(
        json.dumps(
            {"out": str(out), "size": SIZE, "samples": SAMPLES, "views": views},
            indent=2,
            ensure_ascii=False,
        )
        + "\n"
    )
    return path


def _magick(*args: str) -> None:
    subprocess.run(["magick", *args], check=True)


def finish(project: Path) -> Path:
    out = _out(project) / "product"
    spec = json.loads((out / "product.json").read_text())
    font = next((f for f in FONTS if Path(f).exists()), None)
    font_args = ["-font", font] if font else []
    captioned, lines = [], [f"# {project.name}: product renders\n"]
    for v in spec["views"]:
        src = out / f"{v['id']}.png"
        if not src.exists():
            raise SystemExit(f"{src}: not rendered; run blender/render_product.py")
        plate = out / f"{v['id']}-plate.png"
        _magick(str(src), "-background", BACKDROP, "-alpha", "remove",
                "-alpha", "off", str(plate))  # fmt: skip
        cap = out / f"{v['id']}-captioned.png"
        w = spec["size"][0]
        rows = [r for f in v["caption"] for r in textwrap.wrap(f, CAPTION_CHARS)]
        sub = []
        for i, row in enumerate(rows):
            sub += ["-annotate", f"+60+{100 + 42 * i}", row]
        _magick(
            str(plate), "(", "-size", f"{w}x{130 + 42 * len(rows)}", "xc:white",
            *font_args, "-fill", "#1d1d1f", "-pointsize", "46",
            "-gravity", "northwest", "-annotate", "+60+34", v["title"],
            "-fill", "#6e6e73", "-pointsize", "30", *sub, ")",
            "-background", "white", "-append", "+repage", str(cap),
        )  # fmt: skip
        captioned.append(str(cap))
        lines += [
            f"## {v['title']}\n",
            f"`{plate.name}`" + "".join(f"; {f}" for f in v["caption"]) + "\n",
            *[f"- **{k}**: {t}" for k, t in v["instructions"].items()],
            "",
        ]
    _magick("montage", *captioned, "-tile", "4x", "-geometry", "800x+12+12",
            "-background", "#d8d8d8", str(out / "contact-sheet.png"))  # fmt: skip
    (out / "captions.md").write_text("\n".join(lines))
    return out


if __name__ == "__main__":
    if shutil.which("magick") is None and sys.argv[1] == "finish":
        raise SystemExit("finish needs ImageMagick 7 (`magick`)")
    step = {"job": job, "finish": finish}[sys.argv[1]]
    for arg in sys.argv[2:]:
        print(step((REPO_ROOT / arg).resolve()).relative_to(REPO_ROOT))
