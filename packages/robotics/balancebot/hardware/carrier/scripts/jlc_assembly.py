"""Turn kicad-cli BOM and position exports into JLCPCB assembly files.

    python3 jlc_assembly.py <bom.csv> <pos.csv> <out_dir>

Only parts whose symbol carries an LCSC number are kept. The modules, sockets and
connectors have none, so they stay off both files and are hand-soldered.
Writes ``<out_dir>/bom-jlc.csv`` (Comment, Designator, Footprint, LCSC Part #)
and ``<out_dir>/cpl-jlc.csv`` (Designator, Mid X, Mid Y, Layer, Rotation).
"""

import csv
import os
import sys


def main(bom_path, pos_path, out_dir):
    with open(bom_path, newline="") as f:
        bom = [r for r in csv.DictReader(f) if r["LCSC"]]
    if not bom:
        sys.exit(f"no part in {bom_path} has an LCSC number")
    refs = {ref for r in bom for ref in r["Designator"].split(",")}

    with open(pos_path, newline="") as f:
        pos = [r for r in csv.DictReader(f) if r["Ref"] in refs]
    missing = refs - {r["Ref"] for r in pos}
    if missing:
        sys.exit(f"LCSC parts with no SMD placement: {sorted(missing)}")

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


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(*sys.argv[1:])
