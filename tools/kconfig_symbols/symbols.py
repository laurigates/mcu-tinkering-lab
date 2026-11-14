"""Parse Kconfig files, sdkconfig.rename files, symbol lists and sdkconfig.defaults.

Stdlib only: the extractor runs inside the espressif/idf image and the check
runs in pre-commit, neither of which has PyYAML or kconfiglib to hand.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

# `config FOO` / `menuconfig FOO` define an assignable symbol. A `choice NAME`
# is not assignable from sdkconfig, so it is left out on purpose. A trailing
# `# comment` is allowed (ESP-IDF v5.4 esp_driver_cam/Kconfig has one).
_CONFIG_RE = re.compile(
    r"^[ \t]*(?:menu)?config[ \t]+([A-Za-z0-9_]+)[ \t]*(?:#.*)?$", re.MULTILINE
)
# sdkconfig.rename: `CONFIG_OLD CONFIG_NEW` (the new name may carry a `!` for an
# inverted bool). The old name stays accepted, with a deprecation note.
_RENAME_RE = re.compile(
    r"^[ \t]*CONFIG_([A-Za-z0-9_]+)[ \t]+!?CONFIG_[A-Za-z0-9_]+", re.MULTILINE
)
# sdkconfig.defaults: `CONFIG_FOO=value` or `# CONFIG_FOO is not set`.
_ASSIGN_RE = re.compile(r"^CONFIG_([A-Za-z0-9_]+)=")
_UNSET_RE = re.compile(r"^#[ \t]*CONFIG_([A-Za-z0-9_]+) is not set[ \t]*$")
_HEADER_RE = re.compile(r"^#[ \t]*([a-z]+):[ \t]*(.*?)[ \t]*$")

# Directories never part of a build: test and example apps define their own
# symbols (espressif/tinyusb ships example projects with Kconfig.projbuild
# files), and a project's build/ or managed_components/ are generated.
SKIP_DIRS = frozenset(
    {
        "build",
        "managed_components",
        "test_apps",
        "test",
        "tests",
        "host_test",
        "examples",
        ".git",
    }
)


def kconfig_symbols(text: str) -> set[str]:
    """Symbols one Kconfig file defines."""
    return set(_CONFIG_RE.findall(text))


def rename_symbols(text: str) -> set[str]:
    """Old (still accepted) names one sdkconfig.rename file lists."""
    return set(_RENAME_RE.findall(text))


def _walk(root: Path) -> list[Path]:
    out = []
    for path in sorted(root.rglob("*")):
        rel = path.relative_to(root).parts
        if any(p in SKIP_DIRS for p in rel[:-1]) or not path.is_file():
            continue
        out.append(path)
    return out


def tree_symbols(root: Path) -> set[str]:
    """Symbols every Kconfig* and sdkconfig.rename* file under `root` defines."""
    found: set[str] = set()
    for path in _walk(root):
        name = path.name
        if name.startswith("Kconfig"):
            found |= kconfig_symbols(path.read_text(encoding="utf-8", errors="replace"))
        elif name.startswith("sdkconfig.rename"):
            found |= rename_symbols(path.read_text(encoding="utf-8", errors="replace"))
    return found


@dataclass(frozen=True)
class SymbolList:
    """A committed symbol list: `# key: value` header lines, then one symbol a line."""

    header: dict[str, str]
    symbols: frozenset[str]

    @property
    def depends(self) -> list[str]:
        return self.header.get("depends", "").split()


def parse_list(text: str) -> SymbolList:
    header: dict[str, str] = {}
    symbols: set[str] = set()
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("#"):
            m = _HEADER_RE.match(line)
            if m:
                header[m.group(1)] = m.group(2)
            continue
        symbols.add(line)
    return SymbolList(header, frozenset(symbols))


def format_list(header: dict[str, str], symbols: set[str]) -> str:
    lines = [f"# {k}: {v}" for k, v in header.items() if v]
    lines += sorted(symbols)
    return "\n".join(lines) + "\n"


def assignments(text: str) -> list[tuple[int, str]]:
    """(line number, symbol) for every assignment in an sdkconfig.defaults."""
    out = []
    for n, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        m = _ASSIGN_RE.match(line) or _UNSET_RE.match(line)
        if m:
            out.append((n, m.group(1)))
    return out


_FENCE_RE = re.compile(r"^[ \t]*(`{3,}|~{3,})(.*)$")


def fenced_assignments(text: str) -> list[tuple[int, str]]:
    """(line number, symbol) for every sdkconfig assignment inside a fenced
    code block of a Markdown file, whatever the fence's info string.

    A fence closes on a line of the same character, at least as long as the
    opener, with nothing after it; an unclosed fence runs to the end.
    """
    out = []
    fence = None
    for n, line in enumerate(text.splitlines(), 1):
        m = _FENCE_RE.match(line)
        if fence is None:
            if m:
                fence = m.group(1)
            continue
        closes = m and m.group(1)[0] == fence[0] and len(m.group(1)) >= len(fence)
        if closes and not m.group(2).strip():
            fence = None
            continue
        out.extend((n, symbol) for _, symbol in assignments(line))
    return out


def manifest_dependencies(text: str) -> list[str]:
    """Component names under `dependencies:` in an idf_component.yml.

    Only the keys are needed, so this reads the two-space-indented keys of the
    top-level `dependencies:` mapping rather than parsing YAML. `idf` is the
    framework, not a component. A name without a namespace is espressif's.
    """
    names: list[str] = []
    inside = False
    for line in text.splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if not line.startswith((" ", "\t")):
            inside = line.split("#", 1)[0].strip() == "dependencies:"
            continue
        if not inside:
            continue
        m = re.match(r"^  (?! )['\"]?([A-Za-z0-9_.\-/]+)['\"]?[ \t]*:", line)
        if m and m.group(1) != "idf":
            name = m.group(1)
            names.append(name if "/" in name else f"espressif/{name}")
    return names


def lock_versions(text: str) -> dict[str, str]:
    """{component: version} from a dependencies.lock.

    Entries are the two-space keys under the top-level `dependencies:`; each
    carries a four-space `version:` line. `idf` is left out.
    """
    versions: dict[str, str] = {}
    inside = False
    current = None
    for line in text.splitlines():
        if not line.strip():
            continue
        if not line.startswith(" "):
            inside = line.strip() == "dependencies:"
            current = None
            continue
        if not inside:
            continue
        key = re.match(r"^  (?! )['\"]?([A-Za-z0-9_.\-/]+)['\"]?:[ \t]*$", line)
        if key:
            current = key.group(1)
            continue
        ver = re.match(r"^    version:[ \t]*['\"]?([^'\"\s]+)", line)
        if ver and current and current != "idf":
            versions[current] = ver.group(1)
    return versions


def version_key(version: str) -> tuple:
    """Sort key for component versions like `1.14.1`, `1.7.6~2`, `v1.0.0`."""
    return tuple(int(n) for n in re.findall(r"\d+", version))


def component_file_name(name: str) -> str:
    """`espressif/mdns` -> `espressif__mdns`, the managed_components/ dir name."""
    return name.replace("/", "__")
