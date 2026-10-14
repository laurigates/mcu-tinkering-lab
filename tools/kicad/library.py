"""Where a symbol library or a footprint file comes from.

Lookup order:

1. The vendored copy in `tools/kicad/lib/`: a project library named in
   NICKNAME_DIRS, else the KiCad 10.0.6 subset in `kicad-stock/`.
2. The environment: KICAD_SYMBOL_DIR / KICAD_FOOTPRINT_DIR.
3. The macOS bundle's SharedSupport directory.
4. Linux's /usr/share/kicad (the `kicad/kicad` container's path;
   KICAD10_FOOTPRINT_DIR is not set there, so it is not consulted).

Every generator and check passes `vendored_only=True` (the default), so its
output depends only on bytes the repo pins. Steps 2-4 serve `vendor` alone:
rebuilding the subset and reporting drift against the installed KiCad.
"""

from __future__ import annotations

import os
from collections.abc import Iterable, Mapping
from pathlib import Path

from .errors import KicadError
from .paths import REPO_ROOT

VENDORED = REPO_ROOT / "tools/kicad/lib"
STOCK = VENDORED / "kicad-stock"
# Project-specific libraries, each in its own vendored subtree.
NICKNAME_DIRS = {"Seeed_XIAO": VENDORED / "seeed-xiao"}

MAC_ROOT = Path("/Applications/KiCad/KiCad.app/Contents/SharedSupport")
LINUX_ROOT = Path("/usr/share/kicad")
ENV = {"symbols": "KICAD_SYMBOL_DIR", "footprints": "KICAD_FOOTPRINT_DIR"}
VENDOR_HINT = "run `just kicad::vendor` to add it to tools/kicad/lib/"


def split_id(lib_id: str) -> tuple[str, str]:
    """`"Device:R"` -> ("Device", "R")."""
    nick, sep, name = lib_id.partition(":")
    if not sep or not nick or not name or ":" in name:
        raise KicadError(f"{lib_id!r}: expected a library id 'Nickname:Name'")
    return nick, name


def installed_roots(
    kind: str,
    env: Mapping[str, str] | None = None,
    bases: Iterable[Path] = (MAC_ROOT, LINUX_ROOT),
) -> list[Path]:
    """Existing installed-KiCad directories of `kind` ("symbols" or "footprints").

    Steps 2-4 of the lookup order, and the only place they are spelled. A
    `symbols` directory holds `<nick>.kicad_sym`; a `footprints` directory holds
    `<nick>.pretty/`. `env` and `bases` are parameters so tests stay hermetic.
    """
    if kind not in ENV:
        raise KicadError(
            f"unknown library kind {kind!r}; expected one of {sorted(ENV)}"
        )
    env = os.environ if env is None else env
    candidates = []
    if env.get(ENV[kind]):
        candidates.append(Path(env[ENV[kind]]))
    candidates.extend(Path(b) / kind for b in bases)
    out = []
    for c in candidates:
        if c.is_dir() and c not in out:
            out.append(c)
    return out


def vendored_symbol_lib(nick: str, vendored: Path = VENDORED) -> Path:
    if nick in NICKNAME_DIRS:
        return _rebase(NICKNAME_DIRS[nick], vendored) / f"{nick}.kicad_sym"
    return vendored / "kicad-stock" / "symbols" / f"{nick}.kicad_sym"


def vendored_footprint_file(fpid: str, vendored: Path = VENDORED) -> Path:
    nick, name = split_id(fpid)
    if nick in NICKNAME_DIRS:
        base = _rebase(NICKNAME_DIRS[nick], vendored)
    else:
        base = vendored / "kicad-stock" / "footprints"
    return base / f"{nick}.pretty" / f"{name}.kicad_mod"


def installed_symbol_lib(nick: str, search_roots: Iterable[Path] | None = None) -> Path:
    roots = installed_roots("symbols") if search_roots is None else list(search_roots)
    for root in roots:
        path = Path(root) / f"{nick}.kicad_sym"
        if path.is_file():
            return path
    raise KicadError(
        f"symbol library {nick!r}: not in any installed KiCad "
        f"({', '.join(str(r) for r in roots) or 'none found'})"
    )


def installed_footprint_file(
    fpid: str, search_roots: Iterable[Path] | None = None
) -> Path:
    nick, name = split_id(fpid)
    roots = (
        installed_roots("footprints") if search_roots is None else list(search_roots)
    )
    for root in roots:
        path = Path(root) / f"{nick}.pretty" / f"{name}.kicad_mod"
        if path.is_file():
            return path
    raise KicadError(
        f"footprint {fpid!r}: not in any installed KiCad "
        f"({', '.join(str(r) for r in roots) or 'none found'})"
    )


def symbol_lib(
    nick: str,
    vendored_only: bool = True,
    search_roots: Iterable[Path] | None = None,
    vendored: Path = VENDORED,
) -> Path:
    """The `.kicad_sym` file for library nickname `nick`."""
    path = vendored_symbol_lib(nick, vendored)
    if path.is_file():
        return path
    if vendored_only:
        raise KicadError(
            f"symbol library {nick!r}: not vendored ({path}); {VENDOR_HINT}"
        )
    return installed_symbol_lib(nick, search_roots)


def footprint_file(
    fpid: str,
    vendored_only: bool = True,
    search_roots: Iterable[Path] | None = None,
    vendored: Path = VENDORED,
) -> Path:
    """The `.kicad_mod` file for footprint id `"Nickname:Name"`."""
    path = vendored_footprint_file(fpid, vendored)
    if path.is_file():
        return path
    if vendored_only:
        raise KicadError(f"footprint {fpid!r}: not vendored ({path}); {VENDOR_HINT}")
    return installed_footprint_file(fpid, search_roots)


def _rebase(path: Path, vendored: Path) -> Path:
    """`path` under VENDORED, moved under `vendored` (a test's temp tree)."""
    return vendored / path.relative_to(VENDORED)
