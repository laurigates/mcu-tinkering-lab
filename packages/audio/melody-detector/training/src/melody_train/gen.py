"""Synthetic ROI generator for the melody-detector classifier.

Produces 32×32 grayscale patches for each of the five ROI classes —
``empty``, ``quarter``, ``half``, ``rest``, ``other`` — plus a
JSON manifest for reproducibility. All randomness flows through an
explicitly-passed ``numpy.random.Generator`` so two runs with the same
seed produce byte-identical output.

Class semantics:
- ``empty``    — a clean cell of paper with no ink (the dominant class
                 on most kid drawings; CV pre-filter usually catches
                 these before inference).
- ``quarter``  — filled notehead drawn close to the cell centre. Plays
                 as a short note at the row's pitch.
- ``half``     — hollow notehead (open circle outline). Plays as a long
                 note at the row's pitch.
- ``rest``     — squiggle / scribble that the user drew to mean
                 "silence here". Holds the beat without sound.
- ``other``    — non-notehead ink: smudges, fingerprints, partial
                 erasures, paper artifacts. Treated as silent in v1
                 (noise-rejection class).
"""

from __future__ import annotations

import hashlib
import json
import pathlib
import sys
from typing import Any

import click
import numpy as np
from PIL import Image, ImageDraw, ImageFilter

CLASSES: tuple[str, ...] = ("empty", "quarter", "half", "rest", "other")
SCHEMA_VERSION = "0.1.0"

# Convention throughout this module:
#   uint8 grayscale, 255 = paper white, 0 = ink black.

_PAPER_BASE = 245  # very slightly off-white so paper texture has room to vary


# ----------------------------------------------------------------- paper ----


def _paper_background(rng: np.random.Generator, size: int) -> np.ndarray:
    """Off-white background with low-frequency texture variation."""
    base = rng.integers(_PAPER_BASE - 8, _PAPER_BASE + 6, dtype=np.int16)
    img = np.full((size, size), base, dtype=np.int16)
    # Sparse speckle noise — paper fibres / printer dust
    speckle_count = int(rng.integers(0, max(2, size // 8)))
    if speckle_count > 0:
        ys = rng.integers(0, size, size=speckle_count)
        xs = rng.integers(0, size, size=speckle_count)
        magnitudes = rng.integers(-15, 5, size=speckle_count)
        for y, x, m in zip(ys, xs, magnitudes, strict=True):
            img[y, x] = np.clip(int(img[y, x]) + int(m), 0, 255)
    return np.clip(img, 0, 255).astype(np.uint8)


def _ink_intensity(rng: np.random.Generator) -> int:
    """How dark the user's pencil/pen is for this sample."""
    # Pencil is rarely fully black; allow a wide range
    return int(rng.integers(15, 75))


def _maybe_blur(img: Image.Image, rng: np.random.Generator) -> Image.Image:
    """Apply mild blur ~30% of the time (camera defocus / motion blur)."""
    if rng.random() < 0.30:
        radius = float(rng.uniform(0.4, 1.1))
        return img.filter(ImageFilter.GaussianBlur(radius=radius))
    return img


def _add_paper_artifacts(arr: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """Optional faint creases / smudges that aren't ink."""
    if rng.random() < 0.15:
        # Faint diagonal crease — straight darkening line
        size = arr.shape[0]
        y0 = rng.integers(0, size)
        y1 = rng.integers(0, size)
        x0 = rng.integers(0, size)
        x1 = rng.integers(0, size)
        crease = Image.fromarray(arr)
        d = ImageDraw.Draw(crease)
        d.line(((x0, y0), (x1, y1)), fill=int(rng.integers(180, 220)), width=1)
        arr = np.asarray(crease)
    return arr


# --------------------------------------------------------------- classes ----


def _render_empty(rng: np.random.Generator, size: int) -> np.ndarray:
    arr = _paper_background(rng, size)
    arr = _add_paper_artifacts(arr, rng)
    img = Image.fromarray(arr)
    img = _maybe_blur(img, rng)
    return np.asarray(img)


def _render_quarter(rng: np.random.Generator, size: int) -> np.ndarray:
    """Filled notehead — solid dark ellipse near the centre."""
    arr = _paper_background(rng, size)
    img = Image.fromarray(arr)
    cx = size / 2 + rng.uniform(-3, 3)
    cy = size / 2 + rng.uniform(-3, 3)
    rx = rng.uniform(size * 0.20, size * 0.32)
    ry = rng.uniform(size * 0.16, size * 0.26)
    # Mild rotation — kids don't draw axis-aligned ovals
    angle = float(rng.uniform(-25.0, 25.0))
    ink = _ink_intensity(rng)
    overlay = Image.new("L", (size, size), 255)
    od = ImageDraw.Draw(overlay)
    od.ellipse((cx - rx, cy - ry, cx + rx, cy + ry), fill=ink)
    overlay = overlay.rotate(angle, resample=Image.Resampling.BILINEAR, fillcolor=255)
    img = Image.fromarray(np.minimum(np.asarray(img), np.asarray(overlay)))
    img = _maybe_blur(img, rng)
    arr = _add_paper_artifacts(np.asarray(img), rng)
    return arr


def _render_half(rng: np.random.Generator, size: int) -> np.ndarray:
    """Hollow notehead — ellipse outline, hand-drawn imperfections."""
    arr = _paper_background(rng, size)
    img = Image.fromarray(arr)
    cx = size / 2 + rng.uniform(-3, 3)
    cy = size / 2 + rng.uniform(-3, 3)
    rx = rng.uniform(size * 0.22, size * 0.34)
    ry = rng.uniform(size * 0.18, size * 0.28)
    angle = float(rng.uniform(-25.0, 25.0))
    ink = _ink_intensity(rng)
    width = int(rng.integers(1, 3))
    overlay = Image.new("L", (size, size), 255)
    od = ImageDraw.Draw(overlay)
    od.ellipse((cx - rx, cy - ry, cx + rx, cy + ry), outline=ink, width=width)
    overlay = overlay.rotate(angle, resample=Image.Resampling.BILINEAR, fillcolor=255)
    img = Image.fromarray(np.minimum(np.asarray(img), np.asarray(overlay)))
    img = _maybe_blur(img, rng)
    arr = _add_paper_artifacts(np.asarray(img), rng)
    return arr


def _render_rest(rng: np.random.Generator, size: int) -> np.ndarray:
    """Hand-drawn squiggle. A few connected strokes that aren't a notehead."""
    arr = _paper_background(rng, size)
    img = Image.fromarray(arr)
    d = ImageDraw.Draw(img)
    ink = _ink_intensity(rng)
    width = int(rng.integers(1, 3))
    n_segments = int(rng.integers(3, 7))
    margin = size // 5
    pts = [
        (
            int(rng.integers(margin, size - margin)),
            int(rng.integers(margin, size - margin)),
        )
        for _ in range(n_segments + 1)
    ]
    for a, b in zip(pts[:-1], pts[1:], strict=True):
        d.line((a, b), fill=ink, width=width)
    img = _maybe_blur(img, rng)
    arr = _add_paper_artifacts(np.asarray(img), rng)
    return arr


def _render_other(rng: np.random.Generator, size: int) -> np.ndarray:
    """Non-notehead ink: smudge, fingerprint, partial drawing, stray dot.

    The "other" class exists to absorb things we explicitly don't want
    interpreted as music — it's a noise-rejection bucket. The rendered
    sample must not look like a recognizable notehead or rest squiggle.
    """
    arr = _paper_background(rng, size)
    img = Image.fromarray(arr)
    d = ImageDraw.Draw(img)
    ink = _ink_intensity(rng)
    variant = int(rng.integers(0, 4))

    if variant == 0:
        # Stray small dots (eraser remnants, pencil tip touches)
        n = int(rng.integers(1, 5))
        for _ in range(n):
            cx = int(rng.integers(0, size))
            cy = int(rng.integers(0, size))
            r = int(rng.integers(1, 3))
            d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=ink)
    elif variant == 1:
        # Smudge — soft low-contrast blob, much lighter than ink
        cx = int(rng.integers(size // 4, 3 * size // 4))
        cy = int(rng.integers(size // 4, 3 * size // 4))
        r = int(rng.integers(size // 5, size // 3))
        smudge_overlay = Image.new("L", (size, size), 255)
        od = ImageDraw.Draw(smudge_overlay)
        smudge_intensity = int(rng.integers(150, 210))
        od.ellipse((cx - r, cy - r, cx + r, cy + r), fill=smudge_intensity)
        smudge_overlay = smudge_overlay.filter(ImageFilter.GaussianBlur(radius=2.5))
        img = Image.fromarray(np.minimum(np.asarray(img), np.asarray(smudge_overlay)))
        d = ImageDraw.Draw(img)
    elif variant == 2:
        # Partial / aborted notehead — arc, not a closed shape
        cx = size / 2 + rng.uniform(-4, 4)
        cy = size / 2 + rng.uniform(-4, 4)
        rx = rng.uniform(size * 0.15, size * 0.30)
        ry = rng.uniform(size * 0.12, size * 0.22)
        start = int(rng.integers(0, 360))
        end = start + int(rng.integers(40, 180))  # less than full circle
        d.arc((cx - rx, cy - ry, cx + rx, cy + ry), start, end, fill=ink, width=1)
    else:
        # Stray straight line — paper fold, ruler mark
        x0 = int(rng.integers(0, size))
        y0 = int(rng.integers(0, size))
        x1 = int(rng.integers(0, size))
        y1 = int(rng.integers(0, size))
        d.line(((x0, y0), (x1, y1)), fill=ink, width=int(rng.integers(1, 2)))

    img = _maybe_blur(img, rng)
    arr = _add_paper_artifacts(np.asarray(img), rng)
    return arr


_RENDERERS = {
    "empty": _render_empty,
    "quarter": _render_quarter,
    "half": _render_half,
    "rest": _render_rest,
    "other": _render_other,
}


# -------------------------------------------------------------- public API ----


def render_class_sample(
    cls: str,
    rng: np.random.Generator,
    *,
    image_size: int = 32,
) -> np.ndarray:
    """Render a single grayscale ROI for ``cls`` using ``rng`` for all randomness.

    Returns ``ndarray`` of shape ``(image_size, image_size)``, dtype ``uint8``.
    Convention: 255 = paper white, 0 = ink black.

    Raises ``ValueError`` if ``cls`` is not in :data:`CLASSES`.
    """
    if cls not in _RENDERERS:
        raise ValueError(f"Unknown class {cls!r}; expected one of {CLASSES}")
    if image_size < 4:
        raise ValueError(f"image_size must be >= 4, got {image_size}")
    return _RENDERERS[cls](rng, image_size)


def generate_dataset(
    *,
    seed: int,
    out_dir: pathlib.Path,
    per_class: int = 5000,
    image_size: int = 32,
) -> dict[str, int]:
    """Generate a balanced synthetic dataset under ``out_dir``.

    Writes:
      - ``out_dir/<class>/NNNNNN.png`` (6-digit zero-padded index) for each class
      - ``out_dir/manifest.json`` with seed, per_class, image_size, classes,
        counts, and schema version

    Returns a dict mapping each class name to the number of files written.
    """
    out_dir = pathlib.Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    counts: dict[str, int] = {}
    # Each class gets its own RNG stream derived deterministically from the
    # top-level seed so adding a class doesn't perturb existing classes.
    seed_seq = np.random.SeedSequence(seed)
    class_seeds = seed_seq.spawn(len(CLASSES))

    for cls, sub_seed in zip(CLASSES, class_seeds, strict=True):
        cls_dir = out_dir / cls
        cls_dir.mkdir(parents=True, exist_ok=True)
        cls_rng = np.random.default_rng(sub_seed)
        for i in range(per_class):
            arr = render_class_sample(cls, cls_rng, image_size=image_size)
            Image.fromarray(arr, mode="L").save(cls_dir / f"{i:06d}.png", optimize=True)
        counts[cls] = per_class

    manifest: dict[str, Any] = {
        "version": SCHEMA_VERSION,
        "seed": int(seed),
        "per_class": int(per_class),
        "image_size": int(image_size),
        "classes": list(CLASSES),
        "counts": counts,
    }
    manifest_path = out_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    return counts


# --------------------------------------------------------- CLI entry-point ----


@click.command()
@click.option("--seed", type=int, required=True, help="Top-level RNG seed.")
@click.option(
    "--out",
    "out_dir",
    type=click.Path(file_okay=False, path_type=pathlib.Path),
    required=True,
    help="Output directory; created if missing.",
)
@click.option(
    "--per-class",
    type=int,
    default=5000,
    show_default=True,
    help="Samples to generate per class.",
)
@click.option(
    "--image-size",
    type=int,
    default=32,
    show_default=True,
    help="Square ROI side length in pixels.",
)
def _cli(seed: int, out_dir: pathlib.Path, per_class: int, image_size: int) -> None:
    counts = generate_dataset(
        seed=seed,
        out_dir=out_dir,
        per_class=per_class,
        image_size=image_size,
    )
    total = sum(counts.values())
    click.echo(f"Wrote {total} ROIs across {len(counts)} classes -> {out_dir}")
    for cls, n in counts.items():
        click.echo(f"  {cls:<8} {n}")


def main(argv: list[str] | None = None) -> int:
    """CLI entry point. Returns process exit code."""
    try:
        _cli.main(args=argv, standalone_mode=False)
    except click.exceptions.UsageError as exc:
        click.echo(f"Error: {exc.format_message()}", err=True)
        return 2
    except click.exceptions.Abort:
        return 130
    except SystemExit as exc:
        return int(exc.code) if isinstance(exc.code, int) else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())


# ----------------------------------------------------------- module helper ---


def manifest_sha256(manifest_path: pathlib.Path) -> str:
    """Return SHA-256 of the manifest file contents (helper for tests)."""
    return hashlib.sha256(manifest_path.read_bytes()).hexdigest()
