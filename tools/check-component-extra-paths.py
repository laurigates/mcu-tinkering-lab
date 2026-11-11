#!/usr/bin/env python3
"""Assert that CI builds every registered project, and every component each one compiles.

Two checks over `.github/project-matrix.json`, the list `build.yml` builds:

1. Every project the root justfile registers under packages/ (`mod name
   'packages/...'`) has a matrix entry, or an exclusion in NOT_IN_MATRIX below
   with a reason. A registered firmware project with no entry is never built in
   CI, on its own changes or on a shared-component change, and nothing
   reports it (issues #697, #709). An exclusion that names an unregistered
   path, or a project that has since gained an entry, is stale and fails too.
2. Every shared component an ESP-IDF entry compiles is in its build triggers,
   described below.

`.github/workflows/build.yml` decides which projects a push or PR rebuilds from
the changed file paths. A project is selected when a changed file sits under its
own `path`, or under one of the directories in its `extra_paths`
(`.github/project-matrix.json`). A component that lives OUTSIDE the project's
directory, such as `packages/components/improv-wifi` or
`packages/robocar/components/i2c-protocol`, is compiled into the project but is
not under its `path`. Unless `extra_paths` names it, a PR that changes only that
component builds none of the projects that compile it, and the matrix stays green
(issue #670; #529 is the same failure for robocar-bringup).

This check derives, from each ESP-IDF project's own CMakeLists.txt, the set of
out-of-tree component directories it compiles, and fails when `extra_paths`
does not cover one of them.

What a project compiles:

* `EXTRA_COMPONENT_DIRS` (`set(...)` and `list(APPEND ...)`) names directories.
  Each entry is either a component (it has a CMakeLists.txt) or a directory of
  components (its immediate subdirectories that have one).
* Unless the project trims its build (`MINIMAL_BUILD` or `set(COMPONENTS ...)`),
  ESP-IDF builds EVERY component it finds there, required or not. Pointing
  `EXTRA_COMPONENT_DIRS` at `packages/components` therefore compiles all of
  `packages/components/*`; a robocar-main build's `project_description.json`
  lists thinkpack-mesh among its build components although nothing requires it.
* When the build is trimmed, only the `REQUIRES` / `PRIV_REQUIRES` closure is
  built, starting from the project's own components (main/ and any component
  inside the project directory).

An `extra_paths` entry covers a component directory when it equals it or is an
ancestor of it, which is the same prefix test build.yml's jq filter applies.

Run: python3 tools/check-component-extra-paths.py [--verbose]
Exit: 0 clean; 1 on any finding (a registered project with no entry or
exclusion, a stale or reasonless exclusion, a missed component, an
unresolvable `EXTRA_COMPONENT_DIRS` entry, or an out-of-tree entry that does
not exist).
Tests: uv run --no-project --with pytest==9.1.1 python -m pytest -q
       tools/test_check_component_extra_paths.py
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
MATRIX = Path(".github/project-matrix.json")
JUSTFILE = Path("justfile")

# Projects the root justfile registers under packages/ that build.yml does not
# build, keyed by path relative to packages/ (the matrix's `path`). Each needs
# a reason; remove the entry when the project gains a matrix entry.
NOT_IN_MATRIX: dict[str, str] = {
    "components": (
        "shared ESP-IDF components, not a project; the projects that compile "
        "them build them, and the module only runs their host suites "
        "(`just components::test`, the `components` entry of python-suites.json)"
    ),
    "robocar/unified-integrated": (
        "carrier PCB only; it runs packages/robocar/unified's firmware, and "
        "carrier-check.yml checks the board"
    ),
    "robocar/twin": (
        "host build of the planner C, not firmware; robocar-twin.yml builds "
        "and tests it"
    ),
    "robocar/simulation": (
        "Python simulation, not firmware; test.yml and "
        "build-python-simulation.yml cover it"
    ),
    "audio/melody-detector/training": (
        "Python training pipeline, not firmware; test-melody-train.yml covers it"
    ),
    "input-gaming/lego-boost-xbox-fw": (
        "Pybricks firmware for the LEGO Move Hub, built from an upstream clone "
        "that `just boost-fw::setup` fetches into a git-ignored external/; no "
        "ESP-IDF, ESPHome or Pico build applies"
    ),
    "sensors/presence-detector": (
        "does not build with the ESPHome CI uses: it pins esp-idf 5.2.2 / "
        "platform 6.7.0, which ESPHome 2026.10.2 cannot download (404 from "
        "esphome-libs/esp-idf). Without the pin it builds on esp-idf 5.5.6, an "
        "IDF move that waits for the v5.5 migration (#734) and a bench check"
    ),
}

_MOD_LINE = re.compile(
    r"""^\s*mod\??\s+([\w-]+)(?:\s+(?:'([^']*)'|"([^"]*)"))?""", re.MULTILINE
)

# Keywords of idf_component_register(); a REQUIRES list ends at the next one.
_REGISTER_KEYWORDS = {
    "SRCS",
    "SRC_DIRS",
    "EXCLUDE_SRCS",
    "INCLUDE_DIRS",
    "PRIV_INCLUDE_DIRS",
    "LDFRAGMENTS",
    "REQUIRES",
    "PRIV_REQUIRES",
    "REQUIRED_IDF_TARGETS",
    "EMBED_FILES",
    "EMBED_TXTFILES",
    "KCONFIG",
    "KCONFIG_PROJBUILD",
    "WHOLE_ARCHIVE",
}

_TOKEN = re.compile(r'"((?:[^"\\]|\\.)*)"|([^\s"()]+)')
_VAR = re.compile(r"\$\{([^}]*)\}")
_EXTRA_DIRS_CALL = re.compile(
    r"\b(set|list)\s*\(\s*(APPEND\s+)?EXTRA_COMPONENT_DIRS\b([^)]*)\)",
    re.IGNORECASE,
)
_TRIMMED_BUILD = re.compile(
    r"\bMINIMAL_BUILD\b|\bset\s*\(\s*COMPONENTS\b", re.IGNORECASE
)
_REGISTER_CALL = re.compile(r"\bidf_component_register\s*\(([^)]*)\)", re.IGNORECASE)


@dataclass(frozen=True)
class Finding:
    project: str
    message: str

    def __str__(self) -> str:
        return f"{self.project}: {self.message}"


def strip_comments(text: str) -> str:
    """Drop `#` line comments that are not inside a quoted argument."""
    out = []
    for line in text.splitlines():
        in_quote = False
        cut = len(line)
        for i, ch in enumerate(line):
            if ch == '"' and (i == 0 or line[i - 1] != "\\"):
                in_quote = not in_quote
            elif ch == "#" and not in_quote:
                cut = i
                break
        out.append(line[:cut])
    return "\n".join(out)


def tokens(args: str) -> list[str]:
    return [quoted or bare for quoted, bare in _TOKEN.findall(args)]


def extra_component_dirs(
    cmake_text: str, project_dir: Path
) -> tuple[list[Path], list[str]]:
    """Return (resolved EXTRA_COMPONENT_DIRS, unresolvable raw entries).

    `set` replaces the list and `list(APPEND ...)` extends it, in file order.
    Relative entries resolve against the project directory, as ESP-IDF does.
    """
    dirs: list[Path] = []
    unresolved: list[str] = []
    variables = {
        "CMAKE_CURRENT_LIST_DIR": str(project_dir),
        "CMAKE_CURRENT_SOURCE_DIR": str(project_dir),
        "CMAKE_SOURCE_DIR": str(project_dir),
        "PROJECT_DIR": str(project_dir),
    }
    for call, append, args in _EXTRA_DIRS_CALL.findall(strip_comments(cmake_text)):
        if call.lower() == "list" and not append:
            continue  # list(REMOVE_ITEM ...) and friends: not modelled
        if call.lower() == "set":
            dirs = []
        for raw in tokens(args):
            missing = [v for v in _VAR.findall(raw) if v not in variables]
            if missing or "$ENV{" in raw:
                unresolved.append(raw)
                continue
            value = _VAR.sub(lambda m: variables[m.group(1)], raw)
            path = Path(value)
            if not path.is_absolute():
                path = project_dir / path
            dirs.append(Path(_normalise(path)))
    return dirs, unresolved


def _normalise(path: Path) -> str:
    """Collapse `..` without touching the filesystem (symlinks stay as written)."""
    parts: list[str] = []
    for part in path.parts:
        if part == "..":
            if parts and parts[-1] not in ("..", path.anchor):
                parts.pop()
                continue
        elif part == ".":
            continue
        parts.append(part)
    return str(Path(*parts)) if parts else "."


def components_in(directory: Path) -> list[Path]:
    """Components ESP-IDF finds in one EXTRA_COMPONENT_DIRS entry."""
    if (directory / "CMakeLists.txt").is_file():
        return [directory]
    return sorted(
        d
        for d in directory.iterdir()
        if d.is_dir() and (d / "CMakeLists.txt").is_file()
    )


def required_names(cmake_text: str) -> set[str]:
    """Component names in REQUIRES / PRIV_REQUIRES of idf_component_register()."""
    names: set[str] = set()
    for args in _REGISTER_CALL.findall(strip_comments(cmake_text)):
        collecting = False
        for tok in tokens(args):
            if tok in _REGISTER_KEYWORDS:
                collecting = tok in ("REQUIRES", "PRIV_REQUIRES")
            elif collecting:
                names.add(tok)
    return names


def _local_component_cmakes(project_dir: Path) -> list[Path]:
    """CMakeLists of the project's own components: main/ plus anything inside it."""
    found = []
    for cmake in sorted(project_dir.rglob("CMakeLists.txt")):
        rel = cmake.relative_to(project_dir).parts
        if cmake.parent == project_dir or any(
            p in ("build", "managed_components", "external", "test", "tests")
            for p in rel[:-1]
        ):
            continue
        if _REGISTER_CALL.search(cmake.read_text(encoding="utf-8", errors="replace")):
            found.append(cmake)
    return found


def required_closure(project_dir: Path, available: dict[str, Path]) -> set[Path]:
    """Out-of-tree components reached through REQUIRES from the project's own ones."""
    pending: list[str] = []
    for cmake in _local_component_cmakes(project_dir):
        pending.extend(
            required_names(cmake.read_text(encoding="utf-8", errors="replace"))
        )
    seen: set[str] = set()
    reached: set[Path] = set()
    while pending:
        name = pending.pop()
        if name in seen or name not in available:
            continue
        seen.add(name)
        comp = available[name]
        reached.add(comp)
        pending.extend(
            required_names((comp / "CMakeLists.txt").read_text(encoding="utf-8"))
        )
    return reached


def _inside(path: Path, parent: Path) -> bool:
    return path == parent or parent in path.parents


def consumed_components(root: Path, project_path: str) -> tuple[set[Path], list[str]]:
    """Out-of-tree component dirs (under packages/) one project compiles, plus problems."""
    packages = root / "packages"
    project_dir = packages / project_path
    cmake = project_dir / "CMakeLists.txt"
    problems: list[str] = []
    if not cmake.is_file():
        return set(), [f"no CMakeLists.txt at {cmake.relative_to(root)}"]
    text = cmake.read_text(encoding="utf-8")
    dirs, unresolved = extra_component_dirs(text, project_dir)
    for raw in unresolved:
        problems.append(
            f"cannot resolve EXTRA_COMPONENT_DIRS entry {raw!r} in {cmake.relative_to(root)}"
        )

    available: dict[str, Path] = {}
    for d in dirs:
        if _inside(d, project_dir) or not _inside(d, packages):
            continue  # under the project's own path, or outside build.yml's reach
        if not d.is_dir():
            problems.append(
                f"EXTRA_COMPONENT_DIRS entry {d.relative_to(root)} in "
                f"{cmake.relative_to(root)} does not exist"
            )
            continue
        for comp in components_in(d):
            available.setdefault(comp.name, comp)

    if _TRIMMED_BUILD.search(strip_comments(text)):
        return required_closure(project_dir, available), problems
    return set(available.values()), problems


def covered(component: Path, packages: Path, entries: list[str]) -> bool:
    rel = component.relative_to(packages).as_posix()
    return any(
        rel == e.rstrip("/") or rel.startswith(e.rstrip("/") + "/") for e in entries
    )


def check(root: Path, matrix: list[dict]) -> list[Finding]:
    packages = root / "packages"
    findings: list[Finding] = []
    for entry in matrix:
        if entry.get("system") != "esp32":
            continue
        project = entry["project"]
        comps, problems = consumed_components(root, entry["path"])
        findings.extend(Finding(project, p) for p in problems)
        extra = entry.get("extra_paths", [])
        for comp in sorted(comps):
            if not covered(comp, packages, extra):
                rel = comp.relative_to(packages).as_posix()
                findings.append(
                    Finding(project, f'extra_paths misses "{rel}", which it compiles')
                )
    return findings


def registered_modules(justfile_text: str) -> dict[str, str]:
    """`mod` name -> path as written in the justfile (the name when none is given)."""
    modules: dict[str, str] = {}
    for line in strip_comments(justfile_text).splitlines():
        m = _MOD_LINE.match(line)
        if m:
            name, single, double = m.groups()
            modules[name] = single or double or name
    return modules


def check_registered(
    modules: dict[str, str], matrix: list[dict], exclusions: dict[str, str]
) -> list[Finding]:
    """Findings for registered packages/ projects that CI neither builds nor excludes."""
    in_matrix = {e["path"].rstrip("/") for e in matrix}
    registered: dict[str, str] = {}
    for name, path in modules.items():
        parts = Path(path).parts
        if len(parts) > 1 and parts[0] == "packages":
            registered[Path(*parts[1:]).as_posix()] = name

    findings: list[Finding] = []
    for rel, name in sorted(registered.items(), key=lambda kv: kv[1]):
        if rel not in in_matrix and rel not in exclusions:
            findings.append(
                Finding(
                    name,
                    f"packages/{rel} is registered in the root justfile but has no "
                    f"entry in {MATRIX} and no NOT_IN_MATRIX exclusion",
                )
            )
    for rel, reason in sorted(exclusions.items()):
        if not reason.strip():
            findings.append(Finding(rel, "NOT_IN_MATRIX exclusion has no reason"))
        elif rel in in_matrix:
            findings.append(
                Finding(
                    rel, "has a matrix entry, so its NOT_IN_MATRIX exclusion is stale"
                )
            )
        elif rel not in registered:
            findings.append(
                Finding(
                    rel,
                    "NOT_IN_MATRIX excludes it, but no root justfile mod registers "
                    f"packages/{rel}",
                )
            )
    return findings


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--verbose", action="store_true", help="list each project's components"
    )
    args = parser.parse_args(argv)

    matrix = json.loads((REPO_ROOT / MATRIX).read_text(encoding="utf-8"))
    if args.verbose:
        for entry in matrix:
            if entry.get("system") == "esp32":
                comps, _ = consumed_components(REPO_ROOT, entry["path"])
                names = ", ".join(
                    c.relative_to(REPO_ROOT / "packages").as_posix()
                    for c in sorted(comps)
                )
                print(f"{entry['project']}: {names or '(none out of tree)'}")

    modules = registered_modules((REPO_ROOT / JUSTFILE).read_text(encoding="utf-8"))
    unbuilt = check_registered(modules, matrix, NOT_IN_MATRIX)
    if unbuilt:
        print(f"{MATRIX}: registered projects CI does not build:", file=sys.stderr)
        for f in unbuilt:
            print(f"  {f}", file=sys.stderr)
        print(
            f"Add a {MATRIX} entry for each, or a NOT_IN_MATRIX exclusion with the "
            "reason in tools/check-component-extra-paths.py.",
            file=sys.stderr,
        )

    findings = check(REPO_ROOT, matrix)
    if findings:
        print(f"{MATRIX}: build triggers miss shared components:", file=sys.stderr)
        for f in findings:
            print(f"  {f}", file=sys.stderr)
        print(
            "Add each missing directory (relative to packages/) to that project's "
            f"extra_paths in {MATRIX}.",
            file=sys.stderr,
        )
    if unbuilt or findings:
        return 1
    print(
        f"{MATRIX}: every registered project is built or excluded, and every "
        "out-of-tree component is in its consumers' extra_paths"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
