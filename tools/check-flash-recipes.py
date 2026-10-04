#!/usr/bin/env python3
"""Assert that every project composing a SHARED flash recipe actually fits it.

`tools/esp32-idf.just` offers two shared flash recipes, and each bakes in
assumptions that are invisible at the call site. A project writes one line —

    flash: (_s3-flash bin_name)

— and inherits a bootloader offset, a flash-size value written into the
bootloader header, a fixed app offset, and the absence of an ota_data segment.
None of that is stated where the line is written, and three of the four fail
SILENTLY: the board flashes, boots, and misbehaves later.

Prose did not prevent this. `.claude/rules/containerized-builds.md` already said
"exotic flash layouts stay inline" and listed offsets and argfiles as the tell —
flash SIZE was never mentioned, so a project with an ordinary app-at-0x10000
layout on an 8 MB part read as standard and composed the recipe anyway. Caught by
`just --dry-run` only because someone happened to run it. This script is that
check made mechanical, so it does not depend on anyone remembering.

It also audits EVERY flash recipe, hand-written ones included, by expanding it
with `just --dry-run` and checking the files it names, and the offset in front
of each, against what an ESP-IDF build writes and where it expects them (see
check_flash_recipe_outputs). Hand-written recipes stay
hand-written for layout reasons, so the structural audit above cannot read them;
their dry-run text needs no structure to check. That half needs `just` on PATH
and fails, rather than skipping, without it.

Run: python3 tools/check-flash-recipes.py [--verbose]
Exit: 0 clean; 1 on any finding (a shared-recipe mismatch, a flash recipe
naming a file the build does not write, writing one at the wrong offset or at
one that cannot be worked out, or skipping otadata, a failed dry-run), when `just` is missing, or when the
otadata predicate or `just --summary` cannot run.
Tests: python3 -m unittest tools/test_check_flash_recipes.py
"""

from __future__ import annotations

import argparse
import functools
import os
import re
import shutil
import subprocess
import sys
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# esptool arguments baked into each shared recipe. Keep in step with
# tools/esp32-idf.just — the FLASH_SIZE entry is the one that bit us.
SHARED_RECIPES = {
    "_s3-flash": {
        "chip": "esp32s3",
        "targets": {"esp32s3"},
        "flash_size": "4MB",  # hardcoded, NOT detected
        "bootloader_offset": 0x0,
        "app_offset": 0x10000,
        "writes_otadata": False,
    },
    "_esp32-flash": {
        "chip": "esp32",
        "targets": {"esp32"},
        "flash_size": None,  # `--flash-size detect`, so any size is fine
        "bootloader_offset": 0x1000,
        "app_offset": 0x10000,
        "writes_otadata": False,
    },
}

SIZE_TO_BYTES = {
    "1MB": 1 << 20,
    "2MB": 2 << 20,
    "4MB": 4 << 20,
    "8MB": 8 << 20,
    "16MB": 16 << 20,
    "32MB": 32 << 20,
}

# `flash: (_s3-flash bin_name)`, with optional dependencies before the group.
# Anchored at column 0 because that is where a just recipe header lives, which is
# also what keeps a COMMENT mentioning the recipe from matching — robocar/unified
# and robocar/main both explain in prose why they stayed inline, and counting
# those as consumers would report the exact projects that got this right. Those
# inline recipes are not unchecked: check_flash_recipe_outputs audits them.
RECIPE_RE = re.compile(
    r"^(?P<name>[a-z0-9][a-z0-9-]*)\s*:[^\n]*?\((?P<shared>_s3-flash|_esp32-flash)\b",
    re.MULTILINE,
)
TARGET_RE = re.compile(r'^target\s*:=\s*"(?P<target>[^"]+)"', re.MULTILINE)


@dataclass
class Finding:
    project: str
    code: str
    detail: str


@dataclass
class Project:
    justfile: Path
    recipe: str
    shared: str
    target: str | None = None
    flash_size: str | None = None
    partition_source: str = "(idf default)"
    app_offset: int | None = None
    has_otadata: bool = False
    findings: list[Finding] = field(default_factory=list)

    @property
    def name(self) -> str:
        return str(self.justfile.parent.relative_to(REPO_ROOT))


def parse_sdkconfig(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.is_file():
        return values
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, raw = line.partition("=")
        values[key.strip()] = raw.strip().strip('"')
    return values


def parse_partitions(path: Path) -> tuple[int | None, bool]:
    """Return (first app partition offset, whether an otadata partition exists)."""
    app_offset: int | None = None
    has_otadata = False
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        cols = [c.strip() for c in line.split(",")]
        if len(cols) < 4:
            continue
        _name, ptype, subtype, offset = cols[0], cols[1], cols[2], cols[3]
        if ptype == "data" and subtype == "ota":
            has_otadata = True
        if ptype == "app" and app_offset is None and offset:
            try:
                app_offset = int(offset, 0)
            except ValueError:
                pass
    return app_offset, has_otadata


ATTRIBUTE_HINT = (
    "a just attribute must sit IMMEDIATELY above its recipe; a comment or blank "
    "line between them orphans it and `just` refuses to parse the whole file"
)


def check_attribute_placement() -> list[Finding]:
    """Catch an attribute separated from its recipe by a comment or blank line.

    `just` reports this as `error: extraneous attribute` and refuses to parse,
    which breaks EVERY justfile importing the offending file — so a stray
    comment in tools/esp32-idf.just takes out every ESP-IDF project at once.

    Checked textually rather than by shelling out to `just`, so it works on a CI
    runner that has no `just` installed. It is here rather than in a separate
    script because this file is already the pre-commit hook for justfiles, and
    the failure it catches is the one that bit while editing the very recipes
    the rest of this script audits.
    """
    findings: list[Finding] = []
    targets = sorted((REPO_ROOT / "packages").rglob("justfile"))
    targets += sorted((REPO_ROOT / "tools").glob("*.just"))
    targets.append(REPO_ROOT / "justfile")

    for path in targets:
        if not path.is_file():
            continue
        lines = path.read_text().split("\n")
        for i, line in enumerate(lines):
            stripped = line.strip()
            if not (stripped.startswith("[") and stripped.endswith("]")):
                continue
            # Look ahead to the next meaningful line.
            for follower in lines[i + 1 :]:
                nxt = follower.strip()
                if not nxt:
                    continue  # blank alone is tolerated by just
                if nxt.startswith("#"):
                    findings.append(
                        Finding(
                            str(path.relative_to(REPO_ROOT)),
                            "ORPHANED_ATTRIBUTE",
                            f"line {i + 1}: {stripped} is followed by a comment — "
                            + ATTRIBUTE_HINT,
                        )
                    )
                break
    return findings


JUSTFILE_DIRECTORY_HINT = (
    "inside a module or import, justfile_directory() is the ROOT justfile's "
    "directory, not this file's; use source_directory()"
)
JUSTFILE_DIRECTORY_RE = re.compile(r"\bjustfile_directory\(\)")


def check_justfile_directory() -> list[Finding]:
    """Catch `justfile_directory()` in any justfile other than the root one.

    Every package justfile is loaded as a `mod` of the root justfile, and in a
    module `justfile_directory()` returns the ROOT justfile's directory. A path
    built from it therefore points outside the package — `<repo>/../main` for
    robocar's coordination justfile (issue #605), `<repo>/external/bluepad32`
    for gamepad-synth. Nothing errors: `just --list` parses, and the recipe
    only fails (or writes somewhere unexpected) when it runs. `source_directory()`
    is the directory of the file it appears in, which is what these paths mean.

    Comment lines are skipped so prose explaining the trap (tools/esp32.just)
    does not report itself.
    """
    findings: list[Finding] = []
    targets = sorted((REPO_ROOT / "packages").rglob("justfile"))
    targets += sorted((REPO_ROOT / "packages").rglob("*.just"))
    targets += sorted((REPO_ROOT / "tools").rglob("justfile"))
    targets += sorted((REPO_ROOT / "tools").glob("*.just"))
    # `mod schematics 'docs/schematics'` is a module outside packages/ and tools/.
    targets += sorted((REPO_ROOT / "docs").rglob("justfile"))

    for path in targets:
        if not path.is_file():
            continue
        for i, line in enumerate(path.read_text().split("\n")):
            if line.lstrip().startswith("#"):
                continue
            if JUSTFILE_DIRECTORY_RE.search(line):
                findings.append(
                    Finding(
                        str(path.relative_to(REPO_ROOT)),
                        "JUSTFILE_DIRECTORY_IN_MODULE",
                        f"line {i + 1}: " + JUSTFILE_DIRECTORY_HINT,
                    )
                )
    return findings


# ---------------------------------------------------------------------------
# Every flash recipe, shared or hand-written: audit what `just --dry-run` says
# it would run against what the build actually produces (issue #608).
#
# The shared-recipe audit above can reason about structure because it knows the
# recipe. A hand-written recipe has no structure to know — but its dry-run is
# just text naming files, and the files an ESP-IDF build writes are a short,
# fixed list. So the check reads the expanded command and asks three questions
# that need no knowledge of how the recipe is written:
#   1. is every .bin it names one the build produces?
#   2. if it is a full flash and the partition table has an otadata row, does it
#      write ota_data_initial.bin?
#   3. is each file written at the offset the project's target and partition
#      table give it (issue #651)? A right file at a wrong offset overwrites
#      whatever partition sits there, and esptool reports success.
# robocar-main and robocar-camera failed both for months (fixed in #598): their
# recipes flashed build/robocar-{main,camera}.bin, which no build ever wrote, and
# the camera recipe never wrote otadata on an ota_0/ota_1 table with rollback.
# ---------------------------------------------------------------------------

APP_SUFFIX = ".bin"
BOOTLOADER_OUTPUT = "bootloader/bootloader.bin"
PARTITION_TABLE_OUTPUT = "partition_table/partition-table.bin"
OTADATA_OUTPUT = "ota_data_initial.bin"
# Written by every ESP-IDF build regardless of project name. otadata only exists
# when the table has a `data,ota` row, but naming it on a table without one is a
# missing-file error at flash time, not a silent one, so it is allowed here.
FIXED_BUILD_OUTPUTS = (
    BOOTLOADER_OUTPUT,
    PARTITION_TABLE_OUTPUT,
    OTADATA_OUTPUT,
)

# Where the ROM loads the second-stage bootloader from, per target. Copied from
# ESP-IDF v5.4 components/bootloader/Kconfig.projbuild, BOOTLOADER_OFFSET_IN_FLASH
# ("not configurable in ESP-IDF"): 0x1000 on esp32/esp32s2, 0x2000 on
# esp32p4/esp32c5, 0x0 on every other target. A target outside this table gets
# no offset, and a recipe flashing its bootloader is reported unverifiable.
BOOTLOADER_OFFSETS = {
    "esp32": 0x1000,
    "esp32s2": 0x1000,
    "esp32s3": 0x0,
    "esp32c2": 0x0,
    "esp32c3": 0x0,
    "esp32c6": 0x0,
    "esp32c61": 0x0,
    "esp32h2": 0x0,
    "esp32c5": 0x2000,
    "esp32p4": 0x2000,
}
DEFAULT_PARTITION_TABLE_OFFSET = 0x8000
# ESP-IDF's built-in tables (single app, large single app, two OTA) leave the
# offsets blank; gen_esp32part.py places nvs and phy_init after the table and
# aligns the first app to 64 KB, which lands on 0x10000 behind a table at 0x8000.
BUILT_IN_TABLE_APP_OFFSET = 0x10000
# parttool.py's --partition-boot-default search, which is where `idf.py flash`
# writes the app: factory if the table has one, else the lowest ota_N.
BOOT_APP_SUBTYPES = ("factory", *(f"ota_{n}" for n in range(16)))


@dataclass
class FlashLayout:
    """Where each build output of one project belongs in flash.

    None means the offset could not be determined; `why` says why, and is
    quoted in the finding. For otadata, None means the table has no otadata row.
    """

    bootloader: int | None
    partition_table: int | None
    otadata: int | None
    app: int | None
    why: dict[str, str] = field(default_factory=dict)


BIN_TOKEN_RE = re.compile(r"""[^\s"'`=]+\.bin\b""")
# `<offset> <file>.bin`, the pair esptool write-flash takes. A .bin with no
# number in front (a `test -f` guard, an argument to another tool) is not being
# written at an address, so it has no offset to check.
OFFSET_PAIR_RE = re.compile(
    r"(?<![\w.:/-])(?P<offset>0[xX][0-9a-fA-F]+|\d+)\s+(?P<token>"
    + BIN_TOKEN_RE.pattern
    + ")"
)
ESPTOOL_RE = re.compile(r"\besptool(?:\.py)?\b")
# ESP-IDF writes build/flash_args itself, with every part the table needs —
# otadata included — so a recipe that hands esptool the argfile is correct by
# construction and has no paths of its own to check.
ARGFILE_RE = re.compile(r"\s@(?:flash_args|flash_project_args)\b")
CMAKE_PROJECT_RE = re.compile(
    r"^\s*project\(\s*(?P<name>[A-Za-z0-9_.+-]+)", re.MULTILINE
)
MOD_RE = re.compile(
    r"""^mod\s+(?P<name>[A-Za-z0-9_-]+)\s+['"](?P<path>[^'"]+)['"]""", re.MULTILINE
)
OTADATA_PREDICATE = REPO_ROOT / "tools" / "lib" / "otadata-predicate.sh"
# A port that cannot exist, so no recipe expansion can name a real device.
DRY_RUN_ENV = {"PORT": "/dev/ttyDUMMY", "UART_PORT": "/dev/ttyDUMMY"}


def _display(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def split_build_path(token: str, module_dir: Path) -> tuple[Path, str] | None:
    """Split a .bin path into (project directory, path inside its build/).

    Relative paths resolve against the module directory, which is where `just`
    runs a module's recipes. Returns None when no `build` component exists.
    """
    raw = Path(token)
    full = Path(os.path.normpath(raw if raw.is_absolute() else module_dir / raw))
    parts = full.parts
    if "build" not in parts:
        return None
    i = len(parts) - 1 - parts[::-1].index("build")
    return Path(*parts[:i]), "/".join(parts[i + 1 :])


def audit_dry_run(
    recipe: str,
    module_dir: Path,
    text: str,
    project_name: Callable[[Path], str | None],
    layout: Callable[[Path], FlashLayout],
) -> tuple[str, list[Finding]]:
    """Audit one recipe's dry-run text. Returns (status, findings).

    status is "not-esptool" (out of scope: ESPHome, picotool, pybricks),
    "argfile" (delegates to the build's own flash_args), or "checked".
    """
    # A shebang recipe's dry-run prints its comments; prose explaining why a
    # file is (or is not) written must not count as writing it.
    body = "\n".join(ln for ln in text.splitlines() if not ln.lstrip().startswith("#"))
    if not ESPTOOL_RE.search(body):
        return "not-esptool", []

    tokens = list(dict.fromkeys(BIN_TOKEN_RE.findall(body)))
    if not tokens:
        if ARGFILE_RE.search(body):
            return "argfile", []
        return "checked", [
            Finding(
                recipe,
                "NO_BUILD_OUTPUTS",
                "runs esptool but names no build/*.bin and no @flash_args, so "
                "nothing it writes can be checked against the build",
            )
        ]

    findings: list[Finding] = []
    written: dict[Path, set[str]] = {}
    for token in tokens:
        split = split_build_path(token, module_dir)
        if split is None:
            findings.append(
                Finding(
                    recipe, "UNKNOWN_BIN", f"{token} is not under any project's build/"
                )
            )
            continue
        project_dir, rel = split
        name = project_name(project_dir)
        if name is None:
            findings.append(
                Finding(
                    recipe,
                    "NOT_A_PROJECT",
                    f"{token}: {_display(project_dir)} has no ESP-IDF CMakeLists.txt project()",
                )
            )
            continue
        allowed = (name + APP_SUFFIX, *FIXED_BUILD_OUTPUTS)
        if rel not in allowed:
            findings.append(
                Finding(
                    recipe,
                    "UNKNOWN_BIN",
                    f"{token}: the {_display(project_dir)} build writes "
                    f"build/{', build/'.join(allowed)} — not build/{rel} "
                    f"(the app file is named after CMake project({name}))",
                )
            )
        written.setdefault(project_dir, set()).add(rel)

    findings += audit_offsets(recipe, module_dir, body, project_name, layout)

    for project_dir, rels in written.items():
        # Only a FULL flash owes otadata. An app-only recipe leaving the OTA
        # state alone is the point of an app-only recipe.
        if PARTITION_TABLE_OUTPUT in rels and OTADATA_OUTPUT not in rels:
            if layout(project_dir).otadata is not None:
                findings.append(
                    Finding(
                        recipe,
                        "OTADATA_UNWRITTEN",
                        f"{_display(project_dir)}'s partition table has an otadata "
                        f"row but this full flash never writes build/{OTADATA_OUTPUT}, "
                        "which `idf.py flash` writes for this table; on an "
                        "ota_0/ota_1 table the stale OTA state left behind decides "
                        "which slot boots",
                    )
                )
    return "checked", findings


def audit_offsets(
    recipe: str,
    module_dir: Path,
    body: str,
    project_name: Callable[[Path], str | None],
    layout: Callable[[Path], FlashLayout],
) -> list[Finding]:
    """Check the offset in front of each build output against its project's layout.

    Files that are not a build output were already reported by audit_dry_run
    and are skipped here, so one bad path does not produce two findings.
    """
    findings: list[Finding] = []
    for match in OFFSET_PAIR_RE.finditer(body):
        token = match.group("token")
        try:
            offset = int(match.group("offset"), 0)
        except ValueError:  # e.g. "0100": base-0 int() rejects leading zeros
            continue
        split = split_build_path(token, module_dir)
        if split is None:
            continue
        project_dir, rel = split
        name = project_name(project_dir)
        if name is None:
            continue
        role = {
            BOOTLOADER_OUTPUT: "bootloader",
            PARTITION_TABLE_OUTPUT: "partition_table",
            OTADATA_OUTPUT: "otadata",
            name + APP_SUFFIX: "app",
        }.get(rel)
        if role is None:
            continue
        parts = layout(project_dir)
        expected = getattr(parts, role)
        if expected is None:
            # No otadata row: the build writes no ota_data_initial.bin, so the
            # flash fails loudly on the missing file (see FIXED_BUILD_OUTPUTS).
            if role != "otadata":
                findings.append(
                    Finding(
                        recipe,
                        "OFFSET_UNVERIFIABLE",
                        f"{token} at 0x{offset:x}: no offset to check it against "
                        f"— {parts.why.get(role, 'unknown')}",
                    )
                )
            continue
        if offset != expected:
            findings.append(
                Finding(
                    recipe,
                    "WRONG_OFFSET",
                    f"{token} is written at 0x{offset:x}, but it belongs at "
                    f"0x{expected:x} ({parts.why.get(role, role)}); esptool "
                    "writes it there anyway, over whatever partition sits at "
                    f"0x{offset:x}",
                )
            )
    return findings


def cmake_project_name(project_dir: Path) -> str | None:
    """The CMake project() name of an ESP-IDF project, which names build/<name>.bin."""
    cmake = project_dir / "CMakeLists.txt"
    if not cmake.is_file():
        return None
    text = cmake.read_text()
    if "project.cmake" not in text:  # a Pico SDK or plain CMake project
        return None
    match = CMAKE_PROJECT_RE.search(text)
    return match.group("name") if match else None


def partition_table(project_dir: Path) -> tuple[Path, int | None]:
    """The table the build uses and its otadata offset, from the release path's parser.

    Asks tools/lib/otadata-predicate.sh rather than re-parsing here, which keeps
    this check, the flasher manifests and the release assembly from disagreeing
    about which table a project builds (issues #541, #560). A predicate that
    fails to run raises — reading that as "no otadata row" would fail this
    check open.
    """
    script = (
        'source "$1" || exit 2; t=$(resolve_partition_table "$2") || exit 2; '
        'printf "%s\\n" "$t"; parse_otadata_offset "$t"'
    )
    proc = subprocess.run(
        ["bash", "-c", script, "_", str(OTADATA_PREDICATE), str(project_dir)],
        capture_output=True,
        text=True,
        check=False,
    )
    lines = proc.stdout.split("\n")
    if proc.returncode != 0 or len(lines) < 2 or not lines[0]:
        raise RuntimeError(
            f"otadata predicate failed for {_display(project_dir)}: {proc.stderr.strip()}"
        )
    otadata = lines[1].strip()
    return Path(lines[0]), int(otadata) if otadata else None


def boot_app_offset(table: Path) -> tuple[int | None, str]:
    """Offset of the partition `idf.py flash` writes the app to, and its name.

    Returns (None, reason) when the table has no app partition, or when that
    partition's offset is blank and left to gen_esp32part.py to place.
    """
    apps: dict[str, str] = {}
    for line in table.read_text().splitlines():
        cols = [c.strip() for c in line.split("#", 1)[0].split(",")]
        if len(cols) >= 4 and cols[1] == "app":
            apps.setdefault(cols[2], cols[3])
    for subtype in BOOT_APP_SUBTYPES:
        if subtype in apps:
            where = f"{_display(table)}'s {subtype} partition"
            try:
                return int(apps[subtype], 0), where
            except ValueError:
                return None, f"{where} has no explicit offset"
    return None, f"{_display(table)} has no factory or ota_N app partition"


def flash_layout(project_dir: Path) -> FlashLayout:
    """Where an ESP-IDF build of project_dir expects each output to be flashed."""
    why: dict[str, str] = {}
    cfg = parse_sdkconfig(project_dir / "sdkconfig.defaults")

    # `just build` runs `idf.py set-target {{target}}`, so the justfile's target
    # is the chip the bootloader in build/ was built for.
    justfile = project_dir / "justfile"
    match = TARGET_RE.search(justfile.read_text()) if justfile.is_file() else None
    target = match.group("target") if match else cfg.get("CONFIG_IDF_TARGET")
    bootloader = BOOTLOADER_OFFSETS.get(target) if target else None
    if target is None:
        why["bootloader"] = (
            f"{_display(project_dir)} sets no target in its justfile or sdkconfig.defaults"
        )
    elif bootloader is None:
        why["bootloader"] = f'target "{target}" is not in BOOTLOADER_OFFSETS'
    else:
        why["bootloader"] = f"the {target} ROM loads its bootloader there"

    raw = cfg.get("CONFIG_PARTITION_TABLE_OFFSET")
    try:
        table_offset = int(raw, 0) if raw else DEFAULT_PARTITION_TABLE_OFFSET
        why["partition_table"] = (
            "CONFIG_PARTITION_TABLE_OFFSET" if raw else "ESP-IDF's default table offset"
        )
    except ValueError:
        table_offset = None
        why["partition_table"] = f"CONFIG_PARTITION_TABLE_OFFSET={raw} is not a number"

    table, otadata = partition_table(project_dir)
    why["otadata"] = f"{_display(table)}'s otadata row"
    if table.is_file():
        app, why["app"] = boot_app_offset(table)
    elif table_offset == DEFAULT_PARTITION_TABLE_OFFSET:
        app = BUILT_IN_TABLE_APP_OFFSET
        why["app"] = "ESP-IDF's built-in partition table"
    else:
        app = None
        why["app"] = (
            "a built-in partition table behind a moved partition table; "
            "its app offset is not computed here"
        )

    return FlashLayout(
        bootloader=bootloader,
        partition_table=table_offset,
        otadata=otadata,
        app=app,
        why=why,
    )


def flash_recipes(just: str) -> tuple[list[tuple[str, Path]], list[Finding]]:
    """Every public module recipe with "flash" in its name, with its directory."""
    modules = {
        m.group("name"): REPO_ROOT / m.group("path")
        for m in MOD_RE.finditer((REPO_ROOT / "justfile").read_text())
    }
    summary = subprocess.run(
        [just, "--summary"], cwd=REPO_ROOT, capture_output=True, text=True, check=True
    ).stdout.split()
    recipes: list[tuple[str, Path]] = []
    findings: list[Finding] = []
    for path in summary:
        module, sep, name = path.rpartition("::")
        if not sep or "flash" not in name:
            continue
        if module not in modules:
            findings.append(
                Finding(
                    path,
                    "UNRESOLVED_MODULE",
                    "no top-level `mod` line names this module",
                )
            )
            continue
        recipes.append((path, modules[module]))
    return recipes, findings


def dry_run(just: str, recipe: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [just, "--dry-run", recipe],
        cwd=REPO_ROOT,
        env={**os.environ, **DRY_RUN_ENV},
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        check=False,
    )


def check_flash_recipe_outputs(just: str) -> tuple[dict[str, str], list[Finding]]:
    recipes, findings = flash_recipes(just)
    with ThreadPoolExecutor(max_workers=8) as pool:
        runs = dict(
            zip(
                [r for r, _ in recipes],
                pool.map(lambda r: dry_run(just, r[0]), recipes),
            )
        )

    # Recipes share projects (flash and flash-monitor, robocar::flash-main and
    # robocar-main::flash), and each lookup runs the predicate in bash.
    layout = functools.cache(flash_layout)
    statuses: dict[str, str] = {}
    for recipe, module_dir in recipes:
        proc = runs[recipe]
        if proc.returncode != 0:
            statuses[recipe] = "dry-run-failed"
            tail = (proc.stderr.strip().splitlines() or ["(no output)"])[-1]
            findings.append(Finding(recipe, "DRY_RUN_FAILED", tail))
            continue
        status, found = audit_dry_run(
            recipe,
            module_dir,
            proc.stdout + proc.stderr,
            project_name=cmake_project_name,
            layout=layout,
        )
        statuses[recipe] = status
        findings += found
    return statuses, findings


def collect() -> list[Project]:
    projects: list[Project] = []
    for justfile in sorted((REPO_ROOT / "packages").rglob("justfile")):
        text = justfile.read_text()
        for match in RECIPE_RE.finditer(text):
            proj = Project(
                justfile=justfile,
                recipe=match.group("name"),
                shared=match.group("shared"),
            )
            target_match = TARGET_RE.search(text)
            proj.target = target_match.group("target") if target_match else None

            cfg = parse_sdkconfig(justfile.parent / "sdkconfig.defaults")
            proj.flash_size = cfg.get("CONFIG_ESPTOOLPY_FLASHSIZE")

            custom = cfg.get("CONFIG_PARTITION_TABLE_CUSTOM_FILENAME")
            if custom:
                table = justfile.parent / custom
                if table.is_file():
                    proj.partition_source = custom
                    proj.app_offset, proj.has_otadata = parse_partitions(table)
            elif cfg.get("CONFIG_PARTITION_TABLE_TWO_OTA") == "y":
                proj.partition_source = "(idf two-ota preset)"
                proj.has_otadata = True

            projects.append(proj)
    return projects


def audit(proj: Project) -> None:
    spec = SHARED_RECIPES[proj.shared]

    # 1. Bootloader offset is chosen by WHICH recipe, and differs per chip
    #    (0x1000 on ESP32, 0x0 on ESP32-S3). A mismatch produces a board that
    #    does not boot at all — the loudest of the four, and the only loud one.
    if proj.target and proj.target not in spec["targets"]:
        proj.findings.append(
            Finding(
                proj.name,
                "TARGET_MISMATCH",
                f'target "{proj.target}" but {proj.shared} flashes a '
                f"{spec['chip']} bootloader at 0x{spec['bootloader_offset']:x}",
            )
        )

    # 2. Flash size is written into the bootloader header by esptool. Declaring
    #    more than the recipe writes means the header understates the part, and
    #    any partition past the header size is unreachable.
    want = spec["flash_size"]
    if want and proj.flash_size:
        declared = SIZE_TO_BYTES.get(proj.flash_size)
        baked = SIZE_TO_BYTES.get(want)
        if declared and baked and declared > baked:
            proj.findings.append(
                Finding(
                    proj.name,
                    "FLASH_SIZE",
                    f"sdkconfig declares {proj.flash_size} but {proj.shared} "
                    f"hardcodes --flash-size {want}; inline the recipe",
                )
            )

    # 3. An otadata partition that nobody writes leaves stale OTA state in
    #    flash. Silent, and it matters most with app rollback enabled.
    if proj.has_otadata and not spec["writes_otadata"]:
        proj.findings.append(
            Finding(
                proj.name,
                "OTADATA_UNWRITTEN",
                f"{proj.partition_source} has an otadata partition but "
                f"{proj.shared} never writes ota_data_initial.bin",
            )
        )

    # 4. The app offset is hardcoded at 0x10000 in both shared recipes.
    if proj.app_offset is not None and proj.app_offset != spec["app_offset"]:
        proj.findings.append(
            Finding(
                proj.name,
                "APP_OFFSET",
                f"{proj.partition_source} puts the app at 0x{proj.app_offset:x} "
                f"but {proj.shared} writes it to 0x{spec['app_offset']:x}",
            )
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--verbose", action="store_true", help="list every consumer and flash recipe"
    )
    args = parser.parse_args()

    # The output audit needs `just` to expand recipes. Missing it must fail the
    # check, not skip it: a skipped audit prints the same STATUS=OK as a clean one.
    just = shutil.which("just")
    if just is None:
        print("ERROR: `just` is not on PATH; the flash-recipe output audit needs it")
        print("STATUS=FAIL")
        return 1

    projects = collect()
    for proj in projects:
        audit(proj)

    findings = [f for p in projects for f in p.findings]
    findings += check_attribute_placement()
    findings += check_justfile_directory()
    # A lookup that cannot run (the otadata predicate, `just --summary`) has no
    # answer to report; end on the same STATUS line as every other failure
    # instead of a traceback.
    try:
        statuses, output_findings = check_flash_recipe_outputs(just)
    except (RuntimeError, subprocess.CalledProcessError) as exc:
        stderr = (getattr(exc, "stderr", None) or "").strip()
        print(f"ERROR: flash-recipe output audit could not run: {exc}")
        if stderr:
            print(stderr)
        print("STATUS=FAIL")
        return 1
    findings += output_findings
    flagged = {f.project for f in output_findings}

    print("=== SHARED FLASH RECIPE CONSUMERS ===")
    for proj in projects:
        if args.verbose or proj.findings:
            print(
                f"  {proj.name}: {proj.recipe} -> {proj.shared} "
                f"target={proj.target} size={proj.flash_size or '(default)'} "
                f"table={proj.partition_source} "
                f"app={'0x%x' % proj.app_offset if proj.app_offset is not None else '-'} "
                f"otadata={'yes' if proj.has_otadata else 'no'}"
            )
    print(f"CONSUMERS={len(projects)}")

    print("=== FLASH RECIPE OUTPUTS (just --dry-run) ===")
    for recipe, status in statuses.items():
        if args.verbose or recipe in flagged:
            print(f"  {recipe}: {status}")
    for status in ("checked", "argfile", "not-esptool", "dry-run-failed"):
        count = sum(1 for s in statuses.values() if s == status)
        print(f"FLASH_RECIPES_{status.upper().replace('-', '_')}={count}")

    if findings:
        print("=== MISMATCHES ===")
        for f in findings:
            print(f"  {f.project}: {f.code}: {f.detail}")
        print(f"MISMATCHES={len(findings)}")
        print("STATUS=FAIL")
        print()
        print(
            "A shared flash recipe bakes in a bootloader offset, a flash-size\n"
            "value, an app offset, and the absence of ota_data. When a project\n"
            "does not fit, keep its flash recipe INLINE with explicit offsets --\n"
            "see packages/robocar/unified/justfile for the worked example.\n"
            "Any flash recipe must name only files the build writes: the app is\n"
            "build/<CMake project() name>.bin, and a full flash on a table with\n"
            "an otadata row writes build/ota_data_initial.bin too, and each file\n"
            "goes at the offset the target and partition table give it. Verify with:\n"
            "PORT=/dev/ttyDUMMY just --dry-run <module>::flash"
        )
        return 1

    print("MISMATCHES=0")
    print("STATUS=OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
