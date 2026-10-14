"""Turn kicad-cli BOM and position exports into JLCPCB assembly files.

    PYTHONPATH=tools python3 -m kicad.jlc <bom.csv> <pos.csv> <out_dir>

`kicad.fab` calls `write()` after its own exports; the CLI is for re-running
it on a pair already on disk.

Only parts whose symbol carries an LCSC number are kept. The modules, sockets and
connectors have none, so they stay off both files and are hand-soldered.
Writes ``<out_dir>/bom-jlc.csv`` (Comment, Designator, Footprint, LCSC Part #)
and ``<out_dir>/cpl-jlc.csv`` (Designator, Mid X, Mid Y, Layer, Rotation).
"""

from __future__ import annotations

import csv
import os
import sys

from .errors import KicadError


def write(bom_path, pos_path, out_dir) -> tuple[int, int]:
    """Write both files; (BOM lines, placements)."""
    with open(bom_path, newline="") as f:
        bom = [r for r in csv.DictReader(f) if r["LCSC"]]
    if not bom:
        raise KicadError(f"no part in {bom_path} has an LCSC number")
    refs = {ref for r in bom for ref in r["Designator"].split(",")}

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
                    f"{float(r['Rot']):g}",
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
