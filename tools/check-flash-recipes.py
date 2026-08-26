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

Run: python3 tools/check-flash-recipes.py [--verbose]
Exit: 0 clean, 1 if any project mismatches its recipe.
"""

from __future__ import annotations

import argparse
import re
import sys
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
# those as consumers would report the exact projects that got this right.
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
    parser.add_argument("--verbose", action="store_true", help="list every consumer")
    args = parser.parse_args()

    projects = collect()
    for proj in projects:
        audit(proj)

    findings = [f for p in projects for f in p.findings]

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
            "see packages/robocar/unified/justfile for the worked example, and\n"
            "verify with: PORT=/dev/ttyDUMMY just --dry-run <module>::flash"
        )
        return 1

    print("MISMATCHES=0")
    print("STATUS=OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
