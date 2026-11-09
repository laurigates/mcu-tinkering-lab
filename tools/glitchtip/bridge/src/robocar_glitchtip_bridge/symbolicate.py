"""Resolve instruction addresses to function, file and line from an ELF's DWARF.

pyelftools reads DWARF in pure Python, so the bridge needs no Xtensa
toolchain. `load_line_table` is the only function that reads a file; `lookup`
is pure and works on any `LineTable`, which the tests build by hand.

ESP-IDF's backtrace already points caller frames at the call instruction
(it subtracts 3 from each return address), so addresses are looked up as-is.
"""

from __future__ import annotations

import bisect
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

from .event import Frame


@dataclass(frozen=True)
class LineTable:
    """Sorted line rows and function ranges for one ELF."""

    # (start address, end address, file, line), sorted by start; end is exclusive
    rows: list[tuple[int, int, str, int]] = field(default_factory=list)
    # (low pc, high pc, name), sorted by low pc; high is exclusive
    functions: list[tuple[int, int, str]] = field(default_factory=list)


def _containing(ranges: list, addr: int):
    i = bisect.bisect_right([r[0] for r in ranges], addr) - 1
    if i >= 0 and ranges[i][0] <= addr < ranges[i][1]:
        return ranges[i]
    return None


def lookup(table: LineTable, addr: int) -> Frame | None:
    """The frame for `addr`, or None if no row or function covers it."""
    row = _containing(table.rows, addr)
    func = _containing(table.functions, addr)
    if row is None and func is None:
        return None
    return Frame(
        function=func[2] if func else None,
        filename=row[2] if row else None,
        lineno=row[3] if row else None,
    )


def _file_name(lineprog, file_index: int) -> str:
    v5 = lineprog.header.version >= 5
    entries = lineprog["file_entry"]
    # DWARF 5 indexes files and directories from 0; earlier versions from 1,
    # with directory 0 meaning the compilation directory (not in the list)
    idx = file_index if v5 else file_index - 1
    if not 0 <= idx < len(entries):
        return "?"
    entry = entries[idx]
    name = entry.name.decode(errors="replace")
    dirs = lineprog["include_directory"]
    d_idx = entry.dir_index if v5 else entry.dir_index - 1
    if name.startswith("/") or not 0 <= d_idx < len(dirs):
        return name
    return f"{dirs[d_idx].decode(errors='replace')}/{name}"


def _line_rows(dwarf) -> list[tuple[int, int, str, int]]:
    rows: list[tuple[int, int, str, int]] = []
    for cu in dwarf.iter_CUs():
        lineprog = dwarf.line_program_for_CU(cu)
        if lineprog is None:
            continue
        prev = None
        for entry in lineprog.get_entries():
            state = entry.state
            if state is None:
                continue
            if prev is not None and state.address > prev.address:
                rows.append(
                    (
                        prev.address,
                        state.address,
                        _file_name(lineprog, prev.file),
                        prev.line,
                    )
                )
            prev = None if state.end_sequence else state
    rows.sort()
    return rows


def _functions(dwarf) -> list[tuple[int, int, str]]:
    funcs: list[tuple[int, int, str]] = []
    for cu in dwarf.iter_CUs():
        for die in cu.iter_DIEs():
            if die.tag != "DW_TAG_subprogram":
                continue
            attrs = die.attributes
            if "DW_AT_low_pc" not in attrs or "DW_AT_high_pc" not in attrs:
                continue
            low = attrs["DW_AT_low_pc"].value
            high_attr = attrs["DW_AT_high_pc"]
            # DWARF 4+: an offset from low_pc unless the form is an address
            high = (
                high_attr.value
                if high_attr.form == "DW_FORM_addr"
                else low + high_attr.value
            )
            name_attr = attrs.get("DW_AT_name") or attrs.get("DW_AT_linkage_name")
            name = name_attr.value.decode(errors="replace") if name_attr else "?"
            funcs.append((low, high, name))
    funcs.sort()
    return funcs


def load_line_table(path: Path) -> LineTable:
    """Read an ELF's DWARF into a LineTable. Raises if the file has no DWARF."""
    from elftools.elf.elffile import ELFFile

    with path.open("rb") as fh:
        elf = ELFFile(fh)
        if not elf.has_dwarf_info():
            raise ValueError(f"{path} has no DWARF info")
        dwarf = elf.get_dwarf_info()
        return LineTable(rows=_line_rows(dwarf), functions=_functions(dwarf))


Fetcher = Callable[[str, "str | None"], "Path | None"]


class ElfStore:
    """LineTables by build SHA from `<dir>/<sha>.elf`, each parsed once.

    With a `fetch` callable, a SHA with no local ELF is fetched once per
    bridge run (release_elf.fetch_release_elf); a failed fetch is not retried
    until restart, so a burst of crashes from a local build costs one request.
    """

    def __init__(self, directory: Path, fetch: Fetcher | None = None) -> None:
        self.directory = directory
        self._fetch = fetch
        self._tried: set[str] = set()
        self._cache: dict[str, LineTable] = {}

    def table_for(self, sha: str, version: str | None = None) -> LineTable | None:
        """None when no ELF is kept (or fetched) for `sha`."""
        if sha not in self._cache:
            path = self.directory / f"{sha}.elf"
            if not path.is_file():
                if self._fetch is None or sha in self._tried:
                    return None
                self._tried.add(sha)
                fetched = self._fetch(sha, version)
                if fetched is None:
                    return None
                path = fetched
            self._cache[sha] = load_line_table(path)
        return self._cache[sha]
