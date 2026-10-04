#!/usr/bin/env python3
"""Regenerate tracked ESP-IDF dependencies.lock files from their manifests.

A tracked lock pins every managed component at the version it was first
resolved to; nothing moves it afterwards (issue #643). This script is the one
update path, used by `just refresh-idf-locks` and by the scheduled
refresh-idf-locks.yml workflow. The procedure is documented in
.claude/rules/esp-idf-dependency-locks.md.

Two modes, because discovery needs git and regeneration needs ESP-IDF:

    python3 tools/refresh-idf-locks.py --list
        Print the project directory of every git-tracked dependencies.lock.
        Run on the host or CI runner.

    python3 tools/refresh-idf-locks.py <project-dir>...
        For each project, run `idf.py update-dependencies` (delete the lock,
        reconfigure, re-resolve the idf_component.yml ranges). Run inside the
        espressif/idf container, with idf.py on PATH.

Per project:

- The target comes from the lock's own `target:` line, passed both as
  -DIDF_TARGET and as IDF_TARGET in the environment. idf.py rejects the two
  disagreeing, and esp-idf-ci-action exports IDF_TARGET for the whole job.
- The build directory and sdkconfig go to a temporary directory, so a local
  build/ or sdkconfig is never touched.
- A lock whose only entry is `idf` is skipped and the skip is printed: there
  is nothing to re-resolve, and the idf version follows the container image.
  Any other source (registry `service`, `git`, `local`) gets a refresh.
- On failure the original lock is put back, the remaining projects still run,
  and the script exits 1 naming every project that failed, including when
  idf.py cannot be started at all.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

LOCK = "dependencies.lock"
TARGET_RE = re.compile(r"^target:\s*(\S+)\s*$", re.MULTILINE)
# Every entry's source block carries a `type:` line: `idf` for the idf entry,
# `service` for registry components, `git` / `local` for the others. [ \t]
# rather than \s, so a match cannot run across a line break.
SOURCE_TYPE_RE = re.compile(r"^[ \t]+type:[ \t]*(\S+)[ \t]*$", re.MULTILINE)


def list_tracked() -> list[str]:
    """Project dirs of every git-tracked lock under packages/."""
    # :(glob) makes `*` stop at `/`; `**/` still spans any depth.
    out = subprocess.run(
        ["git", "ls-files", "-z", f":(glob)packages/**/{LOCK}"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return sorted(str(Path(p).parent) for p in out.split("\0") if p)


def refresh(project: Path) -> str:
    """Regenerate one lock. Returns a status word; raises RuntimeError on failure."""
    lock = project / LOCK
    if not lock.is_file():
        raise RuntimeError(f"no {LOCK}")
    original = lock.read_text()

    match = TARGET_RE.search(original)
    if not match:
        raise RuntimeError(f"{LOCK} has no top-level `target:` line")
    target = match.group(1)

    if all(t == "idf" for t in SOURCE_TYPE_RE.findall(original)):
        return "skip (only the idf entry — nothing to re-resolve)"

    with tempfile.TemporaryDirectory(prefix="idf-lock-") as scratch:
        cmd = [
            "idf.py",
            "-C",
            str(project),
            "-B",
            str(Path(scratch) / "build"),
            f"-DSDKCONFIG={Path(scratch) / 'sdkconfig'}",
            f"-DIDF_TARGET={target}",
            "update-dependencies",
        ]
        print(f"+ {' '.join(cmd)}", flush=True)
        result = subprocess.run(cmd, env=dict(os.environ, IDF_TARGET=target))

    if result.returncode != 0 or not lock.is_file():
        lock.write_text(original)
        reason = (
            f"idf.py exited {result.returncode}"
            if result.returncode != 0
            else f"idf.py succeeded but wrote no {LOCK}"
        )
        raise RuntimeError(f"{reason}; original lock restored")

    return "changed" if lock.read_text() != original else "unchanged"


def main(argv: list[str]) -> int:
    if argv == ["--list"]:
        print("\n".join(list_tracked()))
        return 0
    if not argv or any(a.startswith("-") for a in argv):
        print(__doc__, file=sys.stderr)
        return 2

    failures: list[str] = []
    for arg in argv:
        project = Path(arg)
        try:
            status = refresh(project)
        except (RuntimeError, OSError) as err:
            # OSError: idf.py missing from PATH or not executable. idf.py never
            # ran, so the lock is untouched and there is nothing to restore.
            failures.append(f"{project}: {err}")
            status = "FAILED"
        print(f"{project}: {status}", flush=True)

    if failures:
        print("\nLock refresh failed for:", file=sys.stderr)
        for line in failures:
            print(f"  {line}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
