"""Pin, compare and rebuild the vendored KiCad library subset.

    PYTHONPATH=tools python3 -m kicad.vendor --check            # CI: KiCad-free
    PYTHONPATH=tools python3 -m kicad.vendor --check-installed  # local: drift report
    PYTHONPATH=tools python3 -m kicad.vendor --update           # local: rebuild

`tools/kicad/lib/kicad-stock/` holds exactly the KiCad 10.0.6 symbols and
footprints the carriers reference (CC-BY-SA 4.0, see its LICENSE.md). Symbol
libraries are extracted subsets: the source library's header, then each used
top-level symbol block unchanged, then `)`. Footprints are byte copies.

`tools/kicad/lib/seeed-xiao/` holds the Seeed Studio XIAO parts (MIT, see its
LICENSE): the vendor DIP footprint and symbol at OSHW-XIAO-Series `cc7f3bb`, and
the socket parts `kicad.derive_xiao` derives from them. It has its own
MANIFEST.tsv, which `derive_xiao` rewrites when it regenerates.

`--check` recomputes every file's sha256 against its subtree's MANIFEST.tsv
and fails on a mismatch, a file the manifest does not list, a listed file that is missing,
a derived (`extends`) symbol, or any difference between the vendored set and
the referenced set: what the parts registry (docs/reference/parts/) names,
plus the schematic writer's power flag. The project library tables point KiCad at these same
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

from . import library, registry, symbols
from .errors import KicadError
from .paths import REPO_ROOT

TAG = "10.0.6"
STOCK_REL = Path("tools/kicad/lib/kicad-stock")
SEEED_TAG = "cc7f3bb"
SEEED_REL = Path("tools/kicad/lib/seeed-xiao")
MANIFEST = "MANIFEST.tsv"
LICENSE = "LICENSE.md"
README = "README.md"
# Hand-written, not vendored: present, but neither hashed nor counted as orphans.
UNPINNED = frozenset({MANIFEST, README})


@dataclass(frozen=True)
class Subtree:
    """A vendored directory pinned by its own MANIFEST.tsv.

    `tag` is the upstream revision every file in it comes from (or, for a
    derived file, is derived from); `license` is the upstream licence file;
    `regen` is the command that rewrites its files and manifest.
    """

    rel: Path
    tag: str
    license: str
    unpinned: frozenset[str]
    regen: str


STOCK_TREE = Subtree(STOCK_REL, TAG, LICENSE, UNPINNED, "just kicad::vendor")
SEEED_TREE = Subtree(
    SEEED_REL,
    SEEED_TAG,
    "LICENSE",
    UNPINNED | {"derive.toml"},
    "PYTHONPATH=tools python3 -m kicad.derive_xiao",
)
SUBTREES = (STOCK_TREE, SEEED_TREE)

# The schematic writer's power flag (spec §4.4): referenced by no registry entry.
WRITER_SYMBOLS = frozenset({"power:PWR_FLAG"})


@dataclass(frozen=True)
class Referenced:
    symbols: frozenset[str]
    footprints: frozenset[str]


def referenced(repo_root: Path = REPO_ROOT) -> Referenced:
    """The stock (non-project) library ids the carriers use.

    Every parts-registry entry's `symbol`, `footprint` and `[perfboard]
    footprint`, plus WRITER_SYMBOLS.
    """
    specs = registry.load_all(repo_root)
    syms = {s.symbol for s in specs} | WRITER_SYMBOLS
    fps = {s.footprint for s in specs} | {s.perfboard.footprint for s in specs}
    return Referenced(
        frozenset(i for i in syms if _stock(i)),
        frozenset(i for i in fps if _stock(i)),
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


def render_manifest(
    subtree: Path, tag: str = TAG, unpinned: frozenset[str] = UNPINNED
) -> str:
    lines = [
        f"{rel}\t{sha256(subtree / rel)}\t{tag}"
        for rel in subtree_files(subtree)
        if rel not in unpinned
    ]
    return "".join(line + "\n" for line in lines)


def subtree_at(path: Path, repo_root: Path = REPO_ROOT) -> Subtree:
    """The pinned subtree whose directory is `path`."""
    for tree in SUBTREES:
        if (repo_root / tree.rel).resolve() == path.resolve():
            return tree
    raise KicadError(
        f"{path}: not a vendored subtree ({[str(t.rel) for t in SUBTREES]})"
    )


def write_manifest(repo_root: Path, tree: Subtree) -> str:
    """Rewrite `tree`'s MANIFEST.tsv from the files on disk; returns a summary line."""
    subtree = repo_root / tree.rel
    text = render_manifest(subtree, tree.tag, tree.unpinned)
    (subtree / MANIFEST).write_text(text, encoding="utf-8")
    return (
        f"wrote {tree.rel / MANIFEST}: {text.count(chr(10))} files pinned at {tree.tag}"
    )


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


def check_pinned(repo_root: Path, tree: Subtree) -> tuple[list[str], bool]:
    """(problems, manifest readable) for one subtree's licence, README and hashes."""
    subtree = repo_root / tree.rel
    if not subtree.is_dir():
        return [f"{subtree}: no vendored library; run `{tree.regen}`"], False
    errors = []
    for name in (tree.license, README):
        if not (subtree / name).is_file():
            errors.append(f"{subtree / name}: missing")
    if not (subtree / MANIFEST).is_file():
        return [*errors, f"{subtree / MANIFEST}: missing; run `{tree.regen}`"], False
    try:
        manifest = read_manifest(subtree / MANIFEST)
    except KicadError as e:
        return [*errors, str(e)], False
    on_disk = [rel for rel in subtree_files(subtree) if rel not in tree.unpinned]
    for rel in on_disk:
        if rel not in manifest:
            errors.append(f"{tree.rel / rel}: not in {MANIFEST} (orphan file)")
    for rel, (digest, tag) in manifest.items():
        path = subtree / rel
        if not path.is_file():
            errors.append(f"{tree.rel / rel}: listed in {MANIFEST} but missing")
            continue
        if tag != tree.tag:
            errors.append(
                f"{tree.rel / rel}: {MANIFEST} tag {tag!r}, expected {tree.tag!r}"
            )
        actual = sha256(path)
        if actual != digest:
            errors.append(
                f"{tree.rel / rel}: sha256 {actual} does not match {MANIFEST} ({digest}); "
                f"vendored files are never hand-edited: run `{tree.regen}`"
            )
    for lib in sorted(subtree.rglob("*.kicad_sym")):
        text = lib.read_text(encoding="utf-8")
        try:
            symbols.header(text)
            for name in symbols.blocks(text):
                symbols.find(text, name, str(lib.relative_to(repo_root)))
        except KicadError as e:
            errors.append(str(e))
    return errors, True


def check(repo_root: Path = REPO_ROOT, want: Referenced | None = None) -> list[str]:
    """Problems with the vendored subtrees; [] when they are pinned and complete."""
    errors = []
    stock_ok = False
    for tree in SUBTREES:
        tree_errors, readable = check_pinned(repo_root, tree)
        errors += tree_errors
        if tree is STOCK_TREE:
            stock_ok = readable
    if not stock_ok:
        return errors
    subtree = repo_root / STOCK_REL
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
            n = len(read_manifest(repo_root / SEEED_REL / MANIFEST))
            print(f"OK: {SEEED_REL}: {n} files pinned at OSHW-XIAO-Series {SEEED_TAG}")
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
