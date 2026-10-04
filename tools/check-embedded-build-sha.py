#!/usr/bin/env python3
"""Assert that a firmware binary embeds the build commit SHA it is published under.

robocar-unified compiles the commit it was built from into the app
(ROBOCAR_BUILD_SHA, resolved from git at CMake configure time) and its OTA
compares that string against the "buildSha" the web-flasher manifest declares
(issue #627). The two are produced independently -- one by CMake inside the
espressif/idf container, one by tools/generate-flasher-manifests.sh on the
runner -- so nothing but this check says they name the same commit.

The failure this exists for is quiet: if git cannot read the checkout at
configure time (in the container, a root process against a runner-owned tree
is "dubious ownership"), the firmware embeds "unknown", every build still
succeeds, and every robot then reports its own build as unidentifiable. So the
check looks for the exact SHA terminated by a NUL -- a "<sha>-dirty" string
contains the SHA as a prefix and must not pass.

Which projects embed it is declared per project in flasher.json as
`"embedsBuildSha": true`; callers read that and run this only for those.

Run:  python3 tools/check-embedded-build-sha.py <app.bin> <40-hex-sha>
Exit: 0 found, 1 not found or bad arguments.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

SHA_RE = re.compile(r"^[0-9a-f]{40}$")


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print(f"usage: {argv[0]} <app.bin> <40-hex-sha>", file=sys.stderr)
        return 1
    binary, sha = Path(argv[1]), argv[2]
    if not SHA_RE.match(sha):
        print(f"ERROR: '{sha}' is not 40 lowercase hex digits", file=sys.stderr)
        return 1
    if not binary.is_file():
        print(f"ERROR: {binary} not found", file=sys.stderr)
        return 1

    data = binary.read_bytes()
    needle = sha.encode("ascii")
    if needle + b"\0" in data:
        print(f"OK: {binary} embeds build SHA {sha}")
        return 0
    if needle + b"-dirty\0" in data:
        print(
            f"ERROR: {binary} embeds {sha}-dirty -- it was built from a tree with "
            "uncommitted changes to tracked files, so it is not that commit",
            file=sys.stderr,
        )
        return 1
    print(
        f"ERROR: {binary} does not embed build SHA {sha} -- check the configure log's "
        "'robocar-unified build SHA:' line; 'unknown' means git could not read the "
        "checkout, a different SHA means the build and the manifest saw different commits",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
