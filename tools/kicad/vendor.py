"""Pin, compare and rebuild the vendored KiCad library subset.

    PYTHONPATH=tools python3 -m kicad.vendor --check            # CI: KiCad-free
    PYTHONPATH=tools python3 -m kicad.vendor --check-installed  # local: drift report
    PYTHONPATH=tools python3 -m kicad.vendor --update           # local: rebuild

`tools/kicad/lib/kicad-stock/` holds exactly the KiCad 10.0.6 symbols and
footprints the carriers reference (CC-BY-SA 4.0, see its LICENSE.md). Symbol
libraries are extracted subsets: the source library's header, then each used
top-level symbol block unchanged, then `)`. Footprints are byte copies.

`--check` recomputes every file's sha256 against MANIFEST.tsv and fails on a
mismatch, a file the manifest does not list, a listed file that is missing,
a derived (`extends`) symbol, or any difference between the vendored set and
the referenced set. The project library tables point KiCad at these same
files, so a hand edit here would agree with everything generated from it;
the manifest is what catches it.

`--check-installed` compares the subset with the installed KiCad, byte for
byte. It is informational (a newer KiCad is drift to review, not an error in
the repo), and it is the step to run before `--update`.
"""

from __future__ import annotations

import argparse
import hashlib
import re
import shutil
import subprocess
import sys
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path

from . import library, symbols
from .errors import KicadError
from .paths import REPO_ROOT

TAG = "10.0.6"
STOCK_REL = Path("tools/kicad/lib/kicad-stock")
MANIFEST = "MANIFEST.tsv"
LICENSE = "LICENSE.md"
README = "README.md"
# Hand-written, not vendored: present, but neither hashed nor counted as orphans.
UNPINNED = frozenset({MANIFEST, README})

# What the carriers reference from the KiCad library, and so what is vendored.
# The parts registry (docs/reference/parts/, PR B wave 3) becomes the source of
# this set: each entry's `symbol`, `footprint` and `[perfboard] footprint`, plus
# the schematic writer's power flag. Until it lands, the set is spelled here.
REFERENCED_SYMBOLS = frozenset(
    {
        "Connector_Generic:Conn_01x02",
        "Connector_Generic:Conn_01x04",
        "Connector_Generic:Conn_01x08",
        "Device:C",
        "Device:C_Polarized",
        "Device:D_Schottky",
        "Device:R",
        "Driver_Motor:Pololu_Breakout_DRV8825",
        "Mechanical:MountingHole",
        "power:PWR_FLAG",
    }
)
REFERENCED_FOOTPRINTS = frozenset(
    {
        "Module:Pololu_Breakout-16_15.2x20.3mm",
        "Connector_AMASS:AMASS_XT30PW-M_1x02_P2.50mm_Horizontal",
        "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical",
        "Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical",
        "Capacitor_SMD:CP_Elec_6.3x7.7",
        "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder",
        "Diode_SMD:D_SOD-123",
        "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
        "MountingHole:MountingHole_3.2mm_M3",
        # Perfboard through-hole stand-ins.
        "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
        "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical",
        "Capacitor_THT:CP_Radial_D8.0mm_P2.50mm",
        "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm",
        "Diode_THT:D_DO-41_SOD81_P2.54mm_Vertical_AnodeUp",
        "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical",
    }
)


@dataclass(frozen=True)
class Referenced:
    symbols: frozenset[str]
    footprints: frozenset[str]


def referenced(repo_root: Path = REPO_ROOT) -> Referenced:
    """The stock (non-project) library ids the carriers use."""
    del repo_root  # read from the parts registry once it exists (see above)
    return Referenced(
        frozenset(i for i in REFERENCED_SYMBOLS if _stock(i)),
        frozenset(i for i in REFERENCED_FOOTPRINTS if _stock(i)),
    )


def _stock(lib_id: str) -> bool:
    return library.split_id(lib_id)[0] not in library.NICKNAME_DIRS


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def subtree_files(subtree: Path) -> list[str]:
    """Every file under `subtree`, as sorted posix paths relative to it."""
    return sorted(
        p.relative_to(subtree).as_posix()
        for p in subtree.rglob("*")
        if p.is_file() and p.name != ".DS_Store"
    )


def render_manifest(subtree: Path, tag: str = TAG) -> str:
    lines = [
        f"{rel}\t{sha256(subtree / rel)}\t{tag}"
        for rel in subtree_files(subtree)
        if rel not in UNPINNED
    ]
    return "".join(line + "\n" for line in lines)


def read_manifest(path: Path) -> dict[str, tuple[str, str]]:
    """rel path -> (sha256, tag). Raises on a malformed or unsorted manifest."""
    try:
        text = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        raise KicadError(f"{path}: missing; run `just kicad::vendor`") from None
    out: dict[str, tuple[str, str]] = {}
    rels = []
    for n, line in enumerate(text.splitlines(), 1):
        fields = line.split("\t")
        if len(fields) != 3 or not re.fullmatch(r"[0-9a-f]{64}", fields[1]):
            raise KicadError(
                f"{path}:{n}: expected 'path<TAB>sha256<TAB>tag', got {line!r}"
            )
        rel, digest, tag = fields
        if rel in out:
            raise KicadError(f"{path}:{n}: {rel} listed twice")
        out[rel] = (digest, tag)
        rels.append(rel)
    if rels != sorted(rels):
        raise KicadError(f"{path}: entries are not sorted by path")
    return out


def vendored_ids(subtree: Path) -> Referenced:
    """The symbol and footprint ids present in a kicad-stock style subtree."""
    syms = set()
    for lib in sorted((subtree / "symbols").glob("*.kicad_sym")):
        for name in symbols.blocks(lib.read_text(encoding="utf-8")):
            syms.add(f"{lib.stem}:{name}")
    fps = {
        f"{p.parent.name.removesuffix('.pretty')}:{p.stem}"
        for p in (subtree / "footprints").glob("*.pretty/*.kicad_mod")
    }
    return Referenced(frozenset(syms), frozenset(fps))


def check(repo_root: Path = REPO_ROOT, want: Referenced | None = None) -> list[str]:
    """Problems with the vendored subtree; [] when it is pinned and complete."""
    subtree = repo_root / STOCK_REL
    if not subtree.is_dir():
        return [f"{subtree}: no vendored library; run `just kicad::vendor`"]
    errors = []
    for name in (LICENSE, README):
        if not (subtree / name).is_file():
            errors.append(f"{subtree / name}: missing")
    try:
        manifest = read_manifest(subtree / MANIFEST)
    except KicadError as e:
        return [*errors, str(e)]
    on_disk = [rel for rel in subtree_files(subtree) if rel not in UNPINNED]
    for rel in on_disk:
        if rel not in manifest:
            errors.append(f"{STOCK_REL / rel}: not in {MANIFEST} (orphan file)")
    for rel, (digest, tag) in manifest.items():
        path = subtree / rel
        if not path.is_file():
            errors.append(f"{STOCK_REL / rel}: listed in {MANIFEST} but missing")
            continue
        if tag != TAG:
            errors.append(
                f"{STOCK_REL / rel}: {MANIFEST} tag {tag!r}, expected {TAG!r}"
            )
        actual = sha256(path)
        if actual != digest:
            errors.append(
                f"{STOCK_REL / rel}: sha256 {actual} does not match {MANIFEST} ({digest}); "
                "vendored files are never hand-edited: run `just kicad::vendor`"
            )
    for lib in sorted((subtree / "symbols").glob("*.kicad_sym")):
        text = lib.read_text(encoding="utf-8")
        try:
            symbols.header(text)
            for name in symbols.blocks(text):
                symbols.find(text, name, str(lib.relative_to(repo_root)))
        except KicadError as e:
            errors.append(str(e))
    want = referenced(repo_root) if want is None else want
    have = vendored_ids(subtree)
    for kind in ("symbols", "footprints"):
        w, h = getattr(want, kind), getattr(have, kind)
        errors += [
            f"{kind[:-1]} {i}: referenced but not vendored" for i in sorted(w - h)
        ]
        errors += [
            f"{kind[:-1]} {i}: vendored but not referenced" for i in sorted(h - w)
        ]
    return errors


def kicad_cli() -> str | None:
    found = shutil.which("kicad-cli")
    mac = library.MAC_ROOT.parent / "MacOS" / "kicad-cli"
    return found or (str(mac) if mac.is_file() else None)


def installed_version() -> str | None:
    cli = kicad_cli()
    if cli is None:
        return None
    out = subprocess.run([cli, "version"], capture_output=True, text=True)
    if out.returncode != 0:
        return None
    return out.stdout.strip() or None


def check_installed(
    repo_root: Path = REPO_ROOT,
    symbol_roots: Iterable[Path] | None = None,
    footprint_roots: Iterable[Path] | None = None,
) -> list[str]:
    """Differences between the vendored subset and the installed KiCad."""
    subtree = repo_root / STOCK_REL
    have = vendored_ids(subtree)
    drift = []
    sym_roots = None if symbol_roots is None else list(symbol_roots)
    fp_roots = None if footprint_roots is None else list(footprint_roots)
    for lib in sorted((subtree / "symbols").glob("*.kicad_sym")):
        ours = lib.read_text(encoding="utf-8")
        try:
            src = library.installed_symbol_lib(lib.stem, sym_roots)
        except KicadError as e:
            drift.append(str(e))
            continue
        theirs = src.read_text(encoding="utf-8")
        if symbols.header(ours) != symbols.header(theirs):
            drift.append(f"symbols/{lib.name}: header differs from {src}")
        installed = symbols.blocks(theirs)
        for name, block in symbols.blocks(ours).items():
            if name not in installed:
                drift.append(f"symbol {lib.stem}:{name}: not in {src}")
            elif installed[name] != block:
                drift.append(f"symbol {lib.stem}:{name}: differs from {src}")
    for fpid in sorted(have.footprints):
        ours = library.vendored_footprint_file(fpid, repo_root / "tools/kicad/lib")
        try:
            src = library.installed_footprint_file(fpid, fp_roots)
        except KicadError as e:
            drift.append(str(e))
            continue
        if ours.read_bytes() != src.read_bytes():
            drift.append(f"footprint {fpid}: differs from {src}")
    return drift


def update(
    repo_root: Path = REPO_ROOT,
    symbol_roots: Iterable[Path] | None = None,
    footprint_roots: Iterable[Path] | None = None,
    want: Referenced | None = None,
) -> list[str]:
    """Rebuild symbols/, footprints/ and MANIFEST.tsv from the installed KiCad."""
    subtree = repo_root / STOCK_REL
    if not (subtree / LICENSE).is_file():
        raise KicadError(
            f"{subtree / LICENSE}: missing; fetch kicad-footprints {TAG}'s LICENSE.md first"
        )
    want = referenced(repo_root) if want is None else want
    sym_roots = None if symbol_roots is None else list(symbol_roots)
    fp_roots = None if footprint_roots is None else list(footprint_roots)
    libs: dict[str, list[str]] = {}
    for lib_id in sorted(want.symbols):
        nick, name = library.split_id(lib_id)
        libs.setdefault(nick, []).append(name)
    sym_out = {}
    for nick, names in libs.items():
        src = library.installed_symbol_lib(nick, sym_roots)
        text = src.read_text(encoding="utf-8")
        picked = [symbols.find(text, n, str(src)) for n in names]
        sym_out[nick] = symbols.header(text) + "\n".join(picked) + "\n)\n"
    fp_src = {
        f: library.installed_footprint_file(f, fp_roots)
        for f in sorted(want.footprints)
    }
    for sub in ("symbols", "footprints"):
        shutil.rmtree(subtree / sub, ignore_errors=True)
    (subtree / "symbols").mkdir(parents=True)
    for nick, text in sym_out.items():
        (subtree / "symbols" / f"{nick}.kicad_sym").write_text(text, encoding="utf-8")
    for fpid, src in fp_src.items():
        dst = library.vendored_footprint_file(fpid, repo_root / "tools/kicad/lib")
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, dst)
    (subtree / MANIFEST).write_text(render_manifest(subtree), encoding="utf-8")
    return [
        f"wrote {len(sym_out)} symbol libraries ({len(want.symbols)} symbols), "
        f"{len(fp_src)} footprints and {MANIFEST} under {STOCK_REL}"
    ]


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.vendor", description=__doc__.split("\n")[0]
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--check", action="store_true", help="KiCad-free: hashes and sets"
    )
    mode.add_argument(
        "--check-installed",
        action="store_true",
        help="compare with the installed KiCad",
    )
    mode.add_argument(
        "--update", action="store_true", help="rebuild from the installed KiCad"
    )
    args = parser.parse_args(argv)
    try:
        if args.check:
            errors = check(repo_root)
            for e in errors:
                print(f"FAIL: {e}")
            if errors:
                return 1
            have = vendored_ids(repo_root / STOCK_REL)
            n = len(read_manifest(repo_root / STOCK_REL / MANIFEST))
            print(
                f"OK: {STOCK_REL}: {len(have.symbols)} symbols, {len(have.footprints)} "
                f"footprints, {n} files pinned at KiCad {TAG}"
            )
            return 0
        version = installed_version()
        if args.check_installed:
            drift = check_installed(repo_root)
            if version is not None and version != TAG:
                drift.insert(0, f"installed KiCad is {version}; vendored is {TAG}")
            for d in drift:
                print(f"DRIFT: {d}")
            if drift:
                return 1
            shown = version or "version unknown: no kicad-cli"
            print(f"OK: {STOCK_REL} matches the installed KiCad ({shown})")
            return 0
        if version != TAG:
            raise KicadError(
                f"installed KiCad is {version or 'unknown (no kicad-cli)'}, vendor.TAG is "
                f"{TAG}: bump TAG and {LICENSE} together, then --update"
            )
        for line in update(repo_root):
            print(line)
        return 0
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
