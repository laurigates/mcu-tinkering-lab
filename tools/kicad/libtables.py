"""A carrier project's `sym-lib-table` and `fp-lib-table`, generated.

Every library nickname the project uses gets an entry, the stock KiCad ones
included, each a `${KIPRJMOD}`-relative path to the vendored copy in
`tools/kicad/lib/`. A project-table entry wins over KiCad's global table of
the same nickname (spec §3.3), so local ERC and DRC read the same bytes
MANIFEST.tsv pins and CI checks, and an update of the installed KiCad library
cannot turn `carrier-check` red. The cost: inside these projects the GUI's
chooser lists only the vendored subset for those nicknames.

Entries are sorted by nickname. `kicad.schematic` writes both files and
`--check` compares them.
"""

from __future__ import annotations

import os
from collections.abc import Iterable
from pathlib import Path

from . import library
from .errors import KicadError
from .vendor import SEEED_TAG, TAG

TABLE_VERSION = 7
STOCK_DESCR = (
    "KiCad {tag} {nick} library, vendored subset "
    "(CC-BY-SA 4.0, tools/kicad/lib/kicad-stock/LICENSE.md)"
)
# Project libraries (library.NICKNAME_DIRS): their own source and licence.
PROJECT_DESCR = {
    "Seeed_XIAO": f"Seeed XIAO RP2350, from Seeed-Studio/OSHW-XIAO-Series @ {SEEED_TAG} (MIT)",
}


def uri(kicad_dir: Path, target: Path) -> str:
    """`target` as a `${KIPRJMOD}`-relative URI from the KiCad project directory."""
    rel = Path(os.path.relpath(os.path.abspath(target), os.path.abspath(kicad_dir)))
    return "${KIPRJMOD}/" + rel.as_posix()


def descr(nick: str) -> str:
    if nick in library.NICKNAME_DIRS:
        if nick not in PROJECT_DESCR:
            raise KicadError(
                f"library {nick!r}: no source/licence description in libtables"
            )
        return PROJECT_DESCR[nick]
    return STOCK_DESCR.format(tag=TAG, nick=nick)


def _table(kind: str, entries: dict[str, Path], kicad_dir: Path) -> str:
    lines = [f"({kind}", f"  (version {TABLE_VERSION})"]
    for nick in sorted(entries):
        lines.append(
            f'  (lib (name "{nick}")(type "KiCad")(uri "{uri(kicad_dir, entries[nick])}")'
            f'(options "")(descr "{descr(nick)}"))'
        )
    lines += [")", ""]
    return "\n".join(lines)


def render_sym_lib_table(
    kicad_dir: Path, lib_ids: Iterable[str], vendored: Path = library.VENDORED
) -> str:
    """The sym-lib-table for symbols `lib_ids` ("Lib:Name"); each must be vendored."""
    entries: dict[str, Path] = {}
    for lib_id in lib_ids:
        nick, _ = library.split_id(lib_id)
        entries[nick] = library.symbol_lib(nick, vendored=vendored)
    return _table("sym_lib_table", entries, kicad_dir)


def render_fp_lib_table(
    kicad_dir: Path, fpids: Iterable[str], vendored: Path = library.VENDORED
) -> str:
    """The fp-lib-table for footprints `fpids`; each must be vendored."""
    entries: dict[str, Path] = {}
    for fpid in fpids:
        nick, _ = library.split_id(fpid)
        pretty = library.footprint_file(fpid, vendored=vendored).parent
        if entries.setdefault(nick, pretty) != pretty:
            raise KicadError(f"footprint library {nick!r}: two directories")
        entries[nick] = pretty
    return _table("fp_lib_table", entries, kicad_dir)
