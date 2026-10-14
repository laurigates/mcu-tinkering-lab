"""Derive socket-mount XIAO footprints and symbols from Seeed's DIP ones.

    PYTHONPATH=tools python3 -m kicad.derive_xiao           # regenerate
    PYTHONPATH=tools python3 -m kicad.derive_xiao --check   # CI: fail if stale
    PYTHONPATH=tools python3 -m kicad.derive_xiao --update-vendor  # after re-vendoring

Seeed's DIP footprints are for soldering a XIAO flat onto a host board: they
carry an Edge.Cuts cutout under the module and plated holes for the bottom pads
along that cutout. On a carrier the XIAO sits in two female headers, so the
cutout would mill a hole in the board and the bottom pads have nothing to meet.

Each `[[socket]]` table of a vendored library's `derive.toml` names a vendor
footprint and symbol, the derived name, and the pad range to keep. The derived
footprint keeps the vendor geometry for the through-hole pads in that range and
drops every Edge.Cuts graphic, every other pad, and the SMD castellations (the
socket pins only need the through-holes). The derived symbol gets the matching
cut, so schematic parity has no pins without pads, and its Footprint property
names the derived footprint. The vendor footprint and symbol stay in the
library unmodified, as the source.

The symbol library is rewritten as its header, every vendor symbol in file
order, then the derived symbols in `derive.toml` order. Regenerating also
rewrites the subtree's MANIFEST.tsv (`kicad.vendor`), so `vendor --check` pins
the derived bytes; `--check` regenerates in memory and byte-compares, writing
nothing. Every library read is from the vendored copy (`vendored_only`).

Regenerating refuses, and writes nothing, when a vendor file (the licence, a
vendor footprint, a vendor symbol block) differs from its MANIFEST.tsv pin:
rewriting the manifest from disk would pin that edit, and `vendor --check`
would stop seeing it. `--update-vendor` re-pins them; it is for after
re-vendoring from upstream, and for pinning a newly vendored part.
"""

from __future__ import annotations

import argparse
import hashlib
import re
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

from . import library, sexpr, symbols, vendor
from .errors import KicadError
from .paths import REPO_ROOT

DERIVE_TOML = "derive.toml"
UPDATE_VENDOR = "--update-vendor"
SOCKET_KEYS = frozenset({"vendor_footprint", "vendor_symbol", "derived", "pads"})
_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*")


@dataclass(frozen=True)
class Socket:
    """One `[[socket]]` table: keep `vendor_footprint`'s pads first..last."""

    vendor_footprint: str
    vendor_symbol: str
    derived: str
    first: int
    last: int

    @property
    def kept(self) -> list[int]:
        return list(range(self.first, self.last + 1))

    @property
    def span(self) -> str:
        return f"{self.first}-{self.last}"


def load_config(path: Path) -> list[Socket]:
    """The `[[socket]]` tables of a derive.toml, validated."""
    try:
        data = tomllib.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise KicadError(f"{path}: missing") from None
    except tomllib.TOMLDecodeError as e:
        raise KicadError(f"{path}: {e}") from None
    unknown = sorted(set(data) - {"socket"})
    if unknown:
        raise KicadError(
            f"{path}: unknown key(s) {unknown}; expected [[socket]] tables"
        )
    tables = data.get("socket")
    if not isinstance(tables, list) or not tables:
        raise KicadError(f"{path}: no [[socket]] tables")
    out = []
    for n, t in enumerate(tables, 1):
        where = f"{path}: [[socket]] #{n}"
        unknown = sorted(set(t) - SOCKET_KEYS)
        missing = sorted(SOCKET_KEYS - set(t))
        if unknown or missing:
            raise KicadError(
                f"{where}: unknown key(s) {unknown}, missing key(s) {missing}; "
                f"expected {sorted(SOCKET_KEYS)}"
            )
        for key in ("vendor_footprint", "vendor_symbol", "derived"):
            if not isinstance(t[key], str) or not _NAME.fullmatch(t[key]):
                raise KicadError(f"{where}: {key} must be a part name, got {t[key]!r}")
        pads = t["pads"]
        if (
            not isinstance(pads, list)
            or len(pads) != 2
            or not all(type(p) is int for p in pads)
            or not 1 <= pads[0] <= pads[1]
        ):
            raise KicadError(
                f"{where}: pads must be [first, last] with 1 <= first <= last, got {pads!r}"
            )
        if t["derived"] in (t["vendor_footprint"], t["vendor_symbol"]):
            raise KicadError(
                f"{where}: derived {t['derived']!r} would overwrite its source"
            )
        out.append(
            Socket(t["vendor_footprint"], t["vendor_symbol"], t["derived"], *pads)
        )
    names = [s.derived for s in out]
    dupes = sorted({x for x in names if names.count(x) > 1})
    if dupes:
        raise KicadError(f"{path}: derived {dupes} listed more than once")
    return out


def _keep_footprint_item(item: str, kept: list[int]) -> bool:
    if '(layer "Edge.Cuts")' in item:
        return False
    m = re.match(r'\(pad "(\d+)" (\w+)', item)
    return not m or (int(m.group(1)) in kept and m.group(2) == "thru_hole")


def derive_footprint(vendor_text: str, socket: Socket, where: str = "footprint") -> str:
    """The derived `.kicad_mod` text for `socket`, from its vendor footprint."""
    body = sexpr.inner(vendor_text)
    kept = [
        body[a:b]
        for a, b in sexpr.spans(body)
        if _keep_footprint_item(body[a:b], socket.kept)
    ]
    pads = sorted(
        int(re.match(r'\(pad "(\d+)"', k).group(1))
        for k in kept
        if k.startswith("(pad ")
    )
    if pads != socket.kept:
        vendor_pads = sorted(
            {int(n) for n in re.findall(r'\(pad "(\d+)" thru_hole', vendor_text)}
        )
        raise KicadError(
            f"{where}: expected one through-hole pad for each of {socket.span}, got {pads} "
            f"(the vendor footprint's through-hole pads: {vendor_pads})"
        )
    return f'(footprint "{socket.derived}"\n\t' + "\n\t".join(kept) + "\n)\n"


def strip_pins(block: str, kept: list[int]) -> str:
    """`block` without the `(pin ...)` blocks numbered outside `kept`, at any depth."""
    out, i = [], 0
    for m in re.finditer(r"\(pin ", block):
        if m.start() < i:
            continue
        a = m.start()
        b = sexpr.block_at(block, a)
        number = re.search(r'\(number "(\d+)"', block[a:b])
        if number and int(number.group(1)) not in kept:
            out.append(block[i:a].rstrip(" \t"))
            i = b
    out.append(block[i:])
    return "".join(out)


def derive_symbol(
    vendor_block: str, socket: Socket, nick: str, where: str = "symbol"
) -> str:
    """The derived symbol block (indent included) for `socket`."""
    derived = strip_pins(vendor_block, socket.kept)
    derived = derived.replace(f'"{socket.vendor_symbol}', f'"{socket.derived}')
    derived = re.sub(
        r'(\(property "Footprint" )"[^"]*"',
        rf'\1"{nick}:{socket.derived}"',
        derived,
        count=1,
    )
    numbers = sorted(int(n) for n in re.findall(r'\(number "(\d+)"', derived))
    if numbers != socket.kept:
        raise KicadError(
            f"{where}: derived symbol pins {numbers}, expected {socket.span}"
        )
    return derived


def derive_library(lib_text: str, sockets: list[Socket], nick: str, where: str) -> str:
    """The symbol library with every derived symbol regenerated from its vendor one."""
    existing = symbols.blocks(lib_text)
    derived_names = {s.derived for s in sockets}
    vendor_blocks = {n: b for n, b in existing.items() if n not in derived_names}
    out = list(vendor_blocks.values())
    for s in sockets:
        if s.vendor_symbol not in vendor_blocks:
            raise KicadError(f"{where}: no vendor symbol {s.vendor_symbol!r}")
        out.append(derive_symbol(vendor_blocks[s.vendor_symbol], s, nick, where))
    return symbols.header(lib_text) + "\n".join(out) + "\n)\n"


def _subtree(nick: str, vendored: Path) -> Path:
    """Library `nick`'s vendored directory, under `vendored` (a test's temp tree)."""
    return vendored / library.NICKNAME_DIRS[nick].relative_to(library.VENDORED)


def libraries(vendored: Path = library.VENDORED) -> list[tuple[str, Path]]:
    """(nickname, subtree) of every vendored project library with a derive.toml."""
    return [
        (nick, _subtree(nick, vendored))
        for nick in sorted(library.NICKNAME_DIRS)
        if (_subtree(nick, vendored) / DERIVE_TOML).is_file()
    ]


def render(nick: str, vendored: Path = library.VENDORED) -> dict[Path, str]:
    """Output path -> derived text, for library `nick`'s derive.toml."""
    sockets = load_config(_subtree(nick, vendored) / DERIVE_TOML)
    out: dict[Path, str] = {}
    for s in sockets:
        src = library.footprint_file(f"{nick}:{s.vendor_footprint}", vendored=vendored)
        dst = library.vendored_footprint_file(f"{nick}:{s.derived}", vendored)
        out[dst] = derive_footprint(src.read_text(encoding="utf-8"), s, str(src))
    lib = library.symbol_lib(nick, vendored=vendored)
    out[lib] = derive_library(lib.read_text(encoding="utf-8"), sockets, nick, str(lib))
    return out


def file_kinds(nick: str, subtree: Path) -> dict[str, str]:
    """Subtree-relative path -> "vendor", "derived" or "mixed", from derive.toml.

    "vendor" files are upstream bytes (the licence, each vendor footprint);
    "derived" files are written only by this module; the symbol library is
    "mixed": its vendor blocks are upstream bytes, its derived blocks are ours.
    """
    sockets = load_config(subtree / DERIVE_TOML)
    kinds = {vendor.SEEED_TREE.license: "vendor", f"{nick}.kicad_sym": "mixed"}
    for s in sockets:
        kinds[f"{nick}.pretty/{s.vendor_footprint}.kicad_mod"] = "vendor"
        kinds[f"{nick}.pretty/{s.derived}.kicad_mod"] = "derived"
    return kinds


def mismatch_hint(rel: str, subtree: Path) -> str:
    """What to do when `rel` (subtree-relative) is off its MANIFEST.tsv pin."""
    regen = "PYTHONPATH=tools python3 -m kicad.derive_xiao"
    restore = (
        f"restore it to the upstream bytes ({vendor.SEEED_TAG}); only after "
        f"re-vendoring it from upstream, re-pin with `{regen} {UPDATE_VENDOR}`"
    )
    nick = next(
        (n for n, d in library.NICKNAME_DIRS.items() if d.name == subtree.name), None
    )
    try:
        kind = file_kinds(nick, subtree).get(rel) if nick else None
    except KicadError:
        kind = None
    if kind == "derived":
        return f"a derived file, never hand-edited: run `{regen}`"
    if kind == "vendor":
        return f"a vendor file, never hand-edited: {restore}"
    if kind == "mixed":
        return (
            f"holds vendor and derived symbols, never hand-edited: `{regen}` "
            f"regenerates the derived ones; for a vendor symbol, {restore}"
        )
    return f"never hand-edited: run `{regen}`, or {restore}"


def vendor_pin_errors(nick: str, subtree: Path, rendered: dict[Path, str]) -> list[str]:
    """Why regenerating would re-pin edited vendor bytes; [] when it is safe.

    Regenerating rewrites MANIFEST.tsv from the files on disk, so it must not
    run over a vendor file that no longer matches its pin: that would pin the
    edit, and `vendor --check` would stop seeing it. A vendor file must equal its
    pin. The mixed symbol library may differ only in its derived blocks, which
    is the case when the regenerated library (`rendered`) equals the pin again.
    """
    try:
        manifest = vendor.read_manifest(subtree / vendor.MANIFEST)
    except KicadError as e:
        return [f"{e}; a first pin needs {UPDATE_VENDOR}"]
    errors = []
    for rel, kind in sorted(file_kinds(nick, subtree).items()):
        if kind == "derived":
            continue
        path = subtree / rel
        pin = manifest.get(rel, (None, None))[0]
        if pin is None:
            errors.append(
                f"{path}: not in {vendor.MANIFEST}; pin a newly vendored file with "
                f"{UPDATE_VENDOR}"
            )
            continue
        if path.is_file() and vendor.sha256(path) == pin:
            continue
        regenerated = rendered.get(path)
        if (
            kind == "mixed"
            and regenerated is not None
            and hashlib.sha256(regenerated.encode("utf-8")).hexdigest() == pin
        ):
            continue  # only a derived block was off; regenerating restores the pin
        errors.append(
            f"{path}: differs from its {vendor.MANIFEST} pin, and regenerating would "
            f"pin the edit; {mismatch_hint(rel, subtree)}"
        )
    return errors


def _rel(path: Path, repo_root: Path) -> str:
    try:
        return path.relative_to(repo_root).as_posix()
    except ValueError:
        return str(path)


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.derive_xiao", description=__doc__.split("\n")[0]
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--check",
        action="store_true",
        help="fail if a derived file differs from its regeneration; write nothing",
    )
    mode.add_argument(
        UPDATE_VENDOR,
        action="store_true",
        help="also re-pin vendor files that differ from MANIFEST.tsv (after re-vendoring)",
    )
    args = parser.parse_args(argv)
    vendored = repo_root / library.VENDORED.relative_to(REPO_ROOT)
    try:
        libs = libraries(vendored)
        if not libs:
            raise KicadError(
                f"no vendored library under {vendored} has a {DERIVE_TOML}"
            )
        rendered = {nick: render(nick, vendored) for nick, _ in libs}
        if not args.check and not args.update_vendor:
            errors = [
                e
                for nick, subtree in libs
                for e in vendor_pin_errors(nick, subtree, rendered[nick])
            ]
            if errors:
                for e in errors:
                    print(f"FAIL: {e}")
                print("FAIL: wrote nothing")
                return 1
        stale = []
        for nick, subtree in libs:
            for path, text in rendered[nick].items():
                rel = _rel(path, repo_root)
                current = path.read_text(encoding="utf-8") if path.is_file() else None
                if current == text:
                    print(f"ok    {rel}")
                elif args.check:
                    stale.append(rel)
                else:
                    path.write_text(text, encoding="utf-8")
                    print(f"wrote {rel}")
            if not args.check:
                print(
                    vendor.write_manifest(
                        repo_root, vendor.subtree_at(subtree, repo_root)
                    )
                )
        for rel in stale:
            print(
                f"FAIL: {rel}: stale; run `PYTHONPATH=tools python3 -m kicad.derive_xiao`"
            )
        return 1 if stale else 0
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
