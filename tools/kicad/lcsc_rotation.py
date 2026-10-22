"""Derive a part's JLCPCB rotation offset from LCSC's own footprint.

    PYTHONPATH=tools python3 -m kicad.lcsc_rotation <C123> <Lib:Footprint> [--pads 1,2,3]
    PYTHONPATH=tools python3 -m kicad.lcsc_rotation --check

JLCPCB places each part as LCSC's (EasyEDA) footprint draws it. This fetches
that footprint, turns it by 0, 90, 180 and 270 degrees about its pads'
centre, and keeps the turn that lands every numbered pad of the vendored KiCad
footprint on the LCSC pad of the same number: the offset kicad.jlc adds to
KiCad's rotation. It fails when no turn lands every pad within `TOLERANCE_MM`,
which means the two footprints are not the same package (or number their pins
differently), and the part needs a look by hand.

`--pads` matches only the listed KiCad pad numbers, for a package whose two
footprints number one pad differently: KiCad's PowerDI3333-8 draws the drain
pad and its fingers as one pad 5, centred on the drain, where LCSC numbers the
fingers 5-8, so LCSC's pad 5 is a corner finger. The entry records `pads`, and
`--check` matches the same pads. Choose pads that fix the turn (the pins of
two sides, not one symmetric pair).

The first form prints an entry for docs/reference/jlcpcb-rotations.toml;
`--check` re-derives every entry there and reports any that disagree. Both
need the network (easyeda.com); nothing in CI runs them. Pad numbers prove pin
1 lands on pin 1, not which end of a polarised part LCSC marks "+": read the
LCSC footprint's silkscreen for those (the table's header says how).
"""

from __future__ import annotations

import json
import math
import sys
import urllib.request
from datetime import date

from . import footprints, jlc, library
from .errors import KicadError

EASYEDA = "https://easyeda.com/api/products/{lcsc}/components?version=6.4.19.5"
TOLERANCE_MM = 0.5  # hand-solder pads run up to ~0.45 mm longer than LCSC's
MIL10_MM = 0.254  # EasyEDA's footprint unit is 10 mil

Pads = dict[str, tuple[float, float]]


def fetch(lcsc: str) -> dict:
    """The EasyEDA component record for an LCSC number."""
    # Python's default User-Agent is refused (403); curl's is served.
    req = urllib.request.Request(
        EASYEDA.format(lcsc=lcsc), headers={"User-Agent": "curl/8.5.0"}
    )
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            record = json.load(resp)
    except (OSError, ValueError) as e:
        raise KicadError(f"{lcsc}: EasyEDA lookup failed: {e}") from None
    if not record.get("success") or not record.get("result"):
        raise KicadError(f"{lcsc}: EasyEDA has no component for it")
    return record


def easyeda_package(record: dict) -> str:
    return record["result"]["packageDetail"]["dataStr"]["head"]["c_para"]["package"]


def easyeda_pads(record: dict) -> Pads:
    """Numbered pad centres in mm, y down; the first pad of each number."""
    pads: Pads = {}
    for shape in record["result"]["packageDetail"]["dataStr"]["shape"]:
        if shape.startswith("PAD~"):
            f = shape.split("~")
            if f[8]:
                pads.setdefault(f[8], (float(f[2]) * MIL10_MM, float(f[3]) * MIL10_MM))
    if not pads:
        raise KicadError("EasyEDA footprint has no numbered pads")
    return pads


def kicad_pads(fpid: str) -> Pads:
    fp = footprints.parse(library.footprint_file(fpid))
    return {n: (p.x, p.y) for n, p in fp.pads.items()}


def _centred(pads: Pads, numbers: list[str]) -> dict[str, tuple[float, float]]:
    """Pads about their centroid, y up (KiCad's rotation counts that way)."""
    cx = sum(pads[n][0] for n in numbers) / len(numbers)
    cy = sum(pads[n][1] for n in numbers) / len(numbers)
    return {n: (pads[n][0] - cx, cy - pads[n][1]) for n in numbers}


def derive(
    lcsc_pads: Pads, kicad: Pads, pads: tuple[str, ...] | None = None
) -> tuple[int, float]:
    """(offset, worst pad error mm): LCSC's footprint turned counter-clockwise
    by `offset` lies on KiCad's at rotation 0. `pads`, if given, are the KiCad
    pad numbers to match; the rest are left out."""
    if pads is not None:
        unknown = sorted(set(pads) - set(kicad))
        if unknown:
            raise KicadError(f"pads {unknown} are not pads of the KiCad footprint")
        kicad = {n: kicad[n] for n in pads}
    missing = sorted(set(kicad) - set(lcsc_pads))
    if missing:
        raise KicadError(f"KiCad pads {missing} have no LCSC pad of that number")
    numbers = sorted(kicad)
    k, e = _centred(kicad, numbers), _centred(lcsc_pads, numbers)
    best: tuple[int, float] | None = None
    for turn in (0, 90, 180, 270):
        c, s = round(math.cos(math.radians(turn))), round(math.sin(math.radians(turn)))
        err = max(
            math.hypot(ex * c - ey * s - k[n][0], ex * s + ey * c - k[n][1])
            for n, (ex, ey) in e.items()
        )
        if best is None or err < best[1]:
            best = (turn, err)
    assert best is not None
    if best[1] > TOLERANCE_MM:
        raise KicadError(
            f"no turn lands every pad within {TOLERANCE_MM} mm "
            f"(best {best[0]} degrees, {best[1]:.2f} mm): not the same package?"
        )
    return best


def entry(lcsc: str, fpid: str, pads: tuple[str, ...] | None = None) -> str:
    """A TOML entry for the rotation table."""
    record = fetch(lcsc)
    turn, err = derive(easyeda_pads(record), kicad_pads(fpid), pads)
    listed = "" if pads is None else f"pads = {json.dumps(list(pads))}\n"
    return (
        f"[{lcsc}] # {record['result'].get('title', '')}\n"
        f'footprint = "{fpid}"\n'
        f'lcsc_footprint = "{easyeda_package(record)}"\n'
        f"rotation = {turn}\n"
        f"{listed}"
        f"max_err_mm = {round(err, 2)}\n"
        f'checked = "{date.today().isoformat()}"\n'
    )


def check() -> list[str]:
    """Every table entry whose re-derived offset differs."""
    problems = []
    for lcsc, rot in sorted(jlc.load_rotations().items()):
        try:
            record = fetch(lcsc)
            turn, _ = derive(
                easyeda_pads(record), kicad_pads(rot.footprint), rot.pads
            )
        except KicadError as e:
            problems.append(f"{lcsc}: {e}")
            continue
        if turn != rot.rotation:
            problems.append(
                f"{lcsc}: table says {rot.rotation}, LCSC's footprint {turn}"
            )
    return problems


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    try:
        if args == ["--check"]:
            problems = check()
            for p in problems:
                print(f"FAIL: {p}")
            if not problems:
                print("every JLCPCB rotation matches LCSC's footprint")
            return 1 if problems else 0
        pads = None
        if len(args) == 4 and args[2] == "--pads":
            pads = tuple(n for n in args[3].split(",") if n)
            args = args[:2]
        if len(args) != 2 or pads == ():
            print(
                "FAIL: expected <C123> <Lib:Footprint> [--pads 1,2,3] or --check"
                f"\n{__doc__}"
            )
            return 1
        print(entry(args[0], args[1], pads), end="")
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
