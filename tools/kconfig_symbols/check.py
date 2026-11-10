"""Fail on an sdkconfig.defaults symbol nothing in the project's build defines.

ESP-IDF drops such a line with only a warning in the build log (issue #707).
For each git-tracked `sdkconfig.defaults*` under packages/, a symbol is known
when one of these defines it:

* ESP-IDF itself: `idf-<version>.txt`, extracted from the espressif/idf image
  the Dockerfile builds on (`just kconfig-symbols::idf`). It holds every
  target's symbols, so a symbol valid only on another target is not caught;
* the project's own components: Kconfig* files under the project directory and
  in the components its EXTRA_COMPONENT_DIRS reach (parsed from the repo);
* its managed components: `components/<ns>__<name>@<version>.txt` for every
  component an idf_component.yml of the project or of those components
  declares, and their recorded dependencies, transitively
  (`just kconfig-symbols::component`). A project with a tracked
  dependencies.lock is checked against the locked version; any other project
  against the highest version recorded;
* the `external/<name>` checkouts its EXTRA_COMPONENT_DIRS point into (cloned
  by a recipe, absent in CI), from `components/external__<name>@<tag>.txt`.

A component with no recorded list is named in the finding, so a symbol it
would define reads as "record the component", never as passing.

Run: PYTHONPATH=tools python3 -m kconfig_symbols.check   (just kconfig-symbols::check)
Exit: 0 when every symbol is known; 1 otherwise.
"""

from __future__ import annotations

import importlib.util
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

from .symbols import (
    SKIP_DIRS,
    SymbolList,
    assignments,
    component_file_name,
    kconfig_symbols,
    lock_versions,
    manifest_dependencies,
    parse_list,
    version_key,
)

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
COMPONENTS_DIR = HERE / "components"

# {component name: {version: list}}
Components = dict[str, dict[str, SymbolList]]


def _extra_paths_module():
    """tools/check-component-extra-paths.py parses EXTRA_COMPONENT_DIRS; reuse it."""
    path = REPO_ROOT / "tools" / "check-component-extra-paths.py"
    spec = importlib.util.spec_from_file_location("check_component_extra_paths", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def idf_list(here: Path = HERE) -> SymbolList:
    lists = sorted(here.glob("idf-v*.txt"))
    if len(lists) != 1:
        raise SystemExit(f"expected one idf-v*.txt in {here}, found {len(lists)}")
    return parse_list(lists[0].read_text(encoding="utf-8"))


def component_lists(directory: Path = COMPONENTS_DIR) -> Components:
    out: Components = {}
    for path in sorted(directory.glob("*.txt")):
        parsed = parse_list(path.read_text(encoding="utf-8"))
        name = parsed.header["component"]
        out.setdefault(name, {})[parsed.header["version"]] = parsed
    return out


def _files(root: Path, predicate) -> list[Path]:
    found = []
    for path in sorted(root.rglob("*")):
        rel = path.relative_to(root).parts
        if any(p in SKIP_DIRS for p in rel[:-1]) or not path.is_file():
            continue
        if predicate(path.name):
            found.append(path)
    return found


def component_roots(project_dir: Path) -> tuple[list[Path], list[str]]:
    """(the project directory plus every out-of-tree component it compiles,
    the `external/<name>` checkouts its EXTRA_COMPONENT_DIRS point into).

    An external/ checkout is cloned by a recipe or a CI step and is not in git,
    so its symbols come from a recorded list, like a managed component's.
    """
    roots = [project_dir]
    externals: list[str] = []
    cmake = project_dir / "CMakeLists.txt"
    if not cmake.is_file():
        return roots, externals
    mod = _extra_paths_module()
    dirs, _unresolved = mod.extra_component_dirs(
        cmake.read_text(encoding="utf-8"), project_dir
    )
    for d in dirs:
        rel = d.relative_to(project_dir).parts if project_dir in d.parents else ()
        if len(rel) >= 2 and rel[0] == "external":
            externals.append(f"external/{rel[1]}")
        elif d.is_dir() and not (d == project_dir or project_dir in d.parents):
            roots.extend(mod.components_in(d))
    return roots, externals


def pick(
    name: str, components: Components, locked: dict[str, str]
) -> SymbolList | None:
    """The recorded list for `name` this project builds against, if any."""
    versions = components.get(name, {})
    if name in locked:
        return versions.get(locked[name])
    if not versions:
        return None
    return versions[max(versions, key=version_key)]


@dataclass(frozen=True)
class Known:
    symbols: frozenset[str]
    recorded: tuple[str, ...]
    unrecorded: tuple[str, ...]


def known_symbols(
    project_dir: Path,
    idf: SymbolList,
    components: Components,
    locked: dict[str, str] | None = None,
) -> Known:
    locked = locked or {}
    roots, externals = component_roots(project_dir)
    symbols = set(idf.symbols)
    pending: list[str] = list(externals)
    for root in roots:
        for path in _files(root, lambda n: n.startswith("Kconfig")):
            symbols |= kconfig_symbols(path.read_text(encoding="utf-8"))
        for path in _files(root, lambda n: n == "idf_component.yml"):
            pending.extend(manifest_dependencies(path.read_text(encoding="utf-8")))
    seen: set[str] = set()
    recorded, unrecorded = [], []
    while pending:
        name = pending.pop()
        if name in seen:
            continue
        seen.add(name)
        chosen = pick(name, components, locked)
        if chosen is None:
            unrecorded.append(f"{name}@{locked[name]}" if name in locked else name)
            continue
        recorded.append(f"{name}@{chosen.header['version']}")
        symbols |= chosen.symbols
        pending.extend(chosen.depends)
    return Known(frozenset(symbols), tuple(sorted(recorded)), tuple(sorted(unrecorded)))


def _record_hint(project: str, entry: str) -> str:
    name = entry.split("@", 1)[0]
    if name.startswith("external/"):
        return f"just kconfig-symbols::component {project}/{name} <tag>"
    return (
        "just kconfig-symbols::component "
        f"{project}/managed_components/{component_file_name(name)}"
    )


def check_file(
    defaults: Path,
    idf: SymbolList,
    components: Components,
    locked: dict[str, str] | None = None,
    root: Path = REPO_ROOT,
) -> list[str]:
    known = known_symbols(defaults.parent, idf, components, locked)
    rel = defaults.relative_to(root).as_posix()
    project = defaults.parent.relative_to(root).as_posix()
    findings = []
    for line, symbol in assignments(defaults.read_text(encoding="utf-8")):
        if symbol in known.symbols:
            continue
        msg = (
            f"{rel}:{line}: CONFIG_{symbol} is not a Kconfig symbol in ESP-IDF "
            f"{idf.header.get('version', '?')}, the project's components"
        )
        if known.recorded:
            msg += f" or {', '.join(known.recorded)}"
        if known.unrecorded:
            msg += (
                f"; not recorded, so not checked: {', '.join(known.unrecorded)}"
                f" (build the project, then `{_record_hint(project, known.unrecorded[0])}`)"
            )
        findings.append(msg)
    return findings


def _git_ls(root: Path, pattern: str) -> list[Path]:
    out = subprocess.run(
        ["git", "ls-files", "-z", f":(glob){pattern}"],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return [root / p for p in out.split("\0") if p]


def tracked_defaults(root: Path = REPO_ROOT) -> list[Path]:
    return _git_ls(root, "packages/**/sdkconfig.defaults*")


def tracked_locks(root: Path = REPO_ROOT) -> dict[Path, dict[str, str]]:
    """{project dir: locked versions} for every git-tracked dependencies.lock.

    An untracked lock is a local build's output and differs between machines,
    so only tracked ones count.
    """
    return {
        p.parent: lock_versions(p.read_text(encoding="utf-8"))
        for p in _git_ls(root, "packages/**/dependencies.lock")
    }


def check_repo(root: Path = REPO_ROOT) -> tuple[int, list[str]]:
    idf, components, locks = idf_list(), component_lists(), tracked_locks(root)
    files = tracked_defaults(root)
    findings = [
        f
        for path in files
        for f in check_file(path, idf, components, locks.get(path.parent), root)
    ]
    return len(files), findings


def main() -> int:
    count, findings = check_repo()
    if not count:
        print("no tracked sdkconfig.defaults found under packages/", file=sys.stderr)
        return 1
    if findings:
        print("\n".join(findings), file=sys.stderr)
        print(
            f"\n{len(findings)} unknown symbol(s). ESP-IDF ignores these lines with"
            " only a build-log warning: delete the line, or fix the name.",
            file=sys.stderr,
        )
        return 1
    print(f"{count} sdkconfig.defaults files: every symbol is defined")
    return 0


if __name__ == "__main__":
    sys.exit(main())
