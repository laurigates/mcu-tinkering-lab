"""Turn kicad-cli BOM and position exports into JLCPCB assembly files.

    PYTHONPATH=tools python3 -m kicad.jlc <bom.csv> <pos.csv> <out_dir>

`kicad.fab` calls `write()` after its own exports; the CLI is for re-running
it on a pair already on disk.

Only parts whose symbol carries an LCSC number are kept. The modules, sockets and
connectors have none, so they stay off both files and are hand-soldered.
Writes ``<out_dir>/bom-jlc.csv`` (Comment, Designator, Footprint, LCSC Part #)
and ``<out_dir>/cpl-jlc.csv`` (Designator, Mid X, Mid Y, Layer, Rotation).

JLCPCB places a part as LCSC's footprint draws it, so each CPL rotation is
KiCad's plus the part's offset from ``docs/reference/jlcpcb-rotations.toml``
(``load_rotations``). A placed LCSC number with no entry there, or with an
entry checked for another footprint, fails: an unchecked offset is a chip
soldered at 90 or 180 degrees.
"""

from __future__ import annotations

import csv
import os
import re
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

from .errors import KicadError
from .paths import REPO_ROOT

ROTATIONS = REPO_ROOT / "docs/reference/jlcpcb-rotations.toml"
ROTATION_KEYS = frozenset(
    {"footprint", "rotation", "lcsc_footprint", "max_err_mm", "checked"}
)
_LCSC = re.compile(r"C[0-9]+")


@dataclass(frozen=True)
class Rotation:
    footprint: str  # the KiCad footprint the offset was checked against
    rotation: int  # degrees counter-clockwise, added to KiCad's


def load_rotations(path: Path = ROTATIONS) -> dict[str, Rotation]:
    """The rotation table: LCSC number -> its checked offset."""
    try:
        data = tomllib.loads(path.read_text(encoding="utf-8"))
    except (OSError, tomllib.TOMLDecodeError) as e:
        raise KicadError(f"{path}: {e}") from None
    table: dict[str, Rotation] = {}
    for lcsc, entry in data.items():
        where = f"{path.name} [{lcsc}]"
        if not _LCSC.fullmatch(lcsc) or not isinstance(entry, dict):
            raise KicadError(f"{where}: not an LCSC number table")
        extra = set(entry) - ROTATION_KEYS
        if extra:
            raise KicadError(f"{where}: unknown keys {sorted(extra)}")
        footprint, rotation = entry.get("footprint"), entry.get("rotation")
        if not isinstance(footprint, str) or ":" not in footprint:
            raise KicadError(f"{where}: footprint must be a Lib:Name string")
        if rotation not in (0, 90, 180, 270) or isinstance(rotation, bool):
            raise KicadError(f"{where}: rotation {rotation!r}, not 0/90/180/270")
        table[lcsc] = Rotation(footprint, rotation)
    return table


def _offsets(bom: list[dict], rotations: dict[str, Rotation]) -> dict[str, int]:
    """Designator -> its LCSC part's rotation offset."""
    offsets: dict[str, int] = {}
    for r in bom:
        refs = r["Designator"].split(",")
        what = f"{r['LCSC']} ({','.join(refs)})"
        entry = rotations.get(r["LCSC"])
        if entry is None:
            raise KicadError(
                f"no checked JLCPCB rotation for {what}, footprint {r['Footprint']}: "
                f"derive one with `python3 -m kicad.lcsc_rotation {r['LCSC']} "
                f"{r['Footprint']}` and add it to {ROTATIONS.relative_to(REPO_ROOT)}"
            )
        if entry.footprint != r["Footprint"]:
            raise KicadError(
                f"{what}: JLCPCB rotation checked for {entry.footprint}, "
                f"placed with {r['Footprint']}"
            )
        offsets.update((ref, entry.rotation) for ref in refs)
    return offsets


def write(
    bom_path, pos_path, out_dir, rotations: dict[str, Rotation] | None = None
) -> tuple[int, int]:
    """Write both files; (BOM lines, placements). `rotations` defaults to the
    repo's table."""
    with open(bom_path, newline="") as f:
        bom = [r for r in csv.DictReader(f) if r["LCSC"]]
    if not bom:
        raise KicadError(f"no part in {bom_path} has an LCSC number")
    refs = {ref for r in bom for ref in r["Designator"].split(",")}
    offsets = _offsets(bom, load_rotations() if rotations is None else rotations)

    with open(pos_path, newline="") as f:
        pos = [r for r in csv.DictReader(f) if r["Ref"] in refs]
    missing = refs - {r["Ref"] for r in pos}
    if missing:
        raise KicadError(f"LCSC parts with no SMD placement: {sorted(missing)}")

    with open(os.path.join(out_dir, "bom-jlc.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
        for r in bom:
            w.writerow([r["Comment"], r["Designator"], r["Footprint"], r["LCSC"]])

    with open(os.path.join(out_dir, "cpl-jlc.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for r in pos:
            layer = {"top": "Top", "bottom": "Bottom"}[r["Side"]]
            w.writerow(
                [
                    r["Ref"],
                    f"{float(r['PosX']):.4f}mm",
                    f"{float(r['PosY']):.4f}mm",
                    layer,
                    f"{(float(r['Rot']) + offsets[r['Ref']]) % 360:g}",
                ]
            )
    print(f"wrote {len(bom)} BOM lines, {len(pos)} placements to {out_dir}")
    return len(bom), len(pos)


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 3:
        print(f"FAIL: expected <bom.csv> <pos.csv> <out_dir>\n{__doc__}")
        return 1
    try:
        write(*args)
    except KeyError as e:
        print(f"FAIL: column {e} missing from the BOM or position CSV")
        return 1
    except (KicadError, OSError) as e:
        print(f"FAIL: {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
