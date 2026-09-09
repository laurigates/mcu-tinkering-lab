#!/usr/bin/env python3
"""Guard the two ways a printable Typst document silently stops tracking hardware.

Both checks exist because of defects that shipped, and both are mechanical —
they belong outside a reviewer's attention, not inside it.

--- 1. A hardcoded pin/channel number is invisible to the drift guard ---

`.github/workflows/build-guide-check.yml` regenerates `docs/auto/pin_defs.typ`
from `main/pin_config.h`, recompiles every document, and fails on a byte diff.
That catches a stale document only where the document *interpolates* a
generated binding. A literal cannot go stale by construction: the PDF recompiles
identically and the guard reports "up to date" while the page prints a channel
number the firmware stopped using.

Observed 2026-09 (robocar-unified): the build guide's PCA9685 channel-map table
listed `([8], [Motor R — IN1 (dir)])` through `([13], …)` as literals. When the
motor channels were renumbered to follow the TB6612FNG's pin order, the table
kept printing the old assignment and the guard stayed green — in the one
document a person builds the robot from.

So: inside a table whose first header cell names a hardware coordinate (Ch,
Channel, GPIO, Pin), a row's first cell may not be a bare integer. Ranges
(`14–15`), names (`D2`), and interpolations (`#PCA_CH_SERVO_PAN`) all pass; only
a lone number fails, and the report names the binding to use instead.

--- 2. Git's default pathspec `*` crosses `/` ---

The guard discovers documents with `git ls-files ':(glob)**/docs/*.typ'`. The
`:(glob)` magic is load-bearing: git's default pathspec matching is fnmatch
WITHOUT FNM_PATHNAME, so a bare `*` spans path separators and the same pattern
also matches `<proj>/docs/auto/pin_defs.typ` — the generated include, which has
no PDF beside it and would fail the guard's "No committed PDF" branch on every
run. The bug is invisible in review because the pattern reads correctly.

This check asserts the discovery set is what the guard needs, and control-tests
it by confirming the bare glob still behaves differently. If git ever changed
those semantics the control fails and this check stops being vacuous quietly.

Usage: python3 tools/check-typst-docs.py [--verbose]
Exit 0 when clean, 1 on any violation.
"""

import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

# First header cell values that mean "this column holds a hardware coordinate".
COORDINATE_HEADERS = {"Ch", "Channel", "GPIO", "Pin", "Pad"}

DISCOVERY_PATHSPEC = ":(glob)**/docs/*.typ"
BARE_PATHSPEC = "**/docs/*.typ"


def git_ls_files(*pathspecs: str) -> list[str]:
    out = subprocess.run(
        ["git", "ls-files", *pathspecs],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    return [line for line in out.stdout.split("\n") if line]


def load_bindings(defs_path: Path) -> dict[str, str]:
    """name -> value, as the value is rendered in Typst output."""
    bindings = {}
    for m in re.finditer(r"^#let (\w+) = (.+)$", defs_path.read_text(), re.MULTILINE):
        bindings[m.group(1)] = m.group(2).strip().strip('"')
    return bindings


def check_literal_coordinates(verbose: bool) -> list[str]:
    """Report table rows whose coordinate cell is a bare integer."""
    problems = []
    for defs_path in sorted(REPO_ROOT.glob("packages/*/*/docs/auto/pin_defs.typ")):
        proj = defs_path.parent.parent.parent
        bindings = load_bindings(defs_path)
        by_value = {}
        for name, value in bindings.items():
            by_value.setdefault(value, []).append(name)

        for doc in sorted(proj.glob("docs/*.typ")):
            in_coordinate_table = False
            for lineno, line in enumerate(doc.read_text().split("\n"), 1):
                stripped = line.strip()
                if stripped.startswith("//"):
                    continue

                header = re.match(r"^\(\[([^\]]*)\],", stripped)
                if header:
                    cell = header.group(1).strip()
                    if cell in COORDINATE_HEADERS:
                        in_coordinate_table = True
                        continue
                    if not in_coordinate_table:
                        continue
                    # A row inside a coordinate table.
                    if re.fullmatch(r"\d+", cell):
                        rel = doc.relative_to(REPO_ROOT)
                        # Several bindings can share a value (channel 6 is both
                        # PCA_CH_SERVO_PAN and I2C_SCL_PIN), so name them all —
                        # picking one would send the reader to the wrong fact.
                        names = sorted(by_value.get(cell, []))
                        fix = (
                            "use " + " or ".join(f"#{n}" for n in names)
                            if names
                            else "no generated binding has this value"
                        )
                        problems.append(
                            f"{rel}:{lineno}: coordinate cell is the literal '{cell}' "
                            f"({fix}) — a literal is invisible to the drift guard\n"
                            f"    {stripped[:100]}"
                        )
                    continue

                # A blank line or a closing paren ends the table.
                if stripped in (")", "),", "") or stripped.startswith(("=", "#let ")):
                    in_coordinate_table = False

            if verbose:
                print(f"scanned {doc.relative_to(REPO_ROOT)}")
    return problems


def check_discovery_pathspec(verbose: bool) -> list[str]:
    """Assert the guard's discovery set, and control-test the magic that makes it."""
    problems = []
    discovered = git_ls_files(DISCOVERY_PATHSPEC)

    generated = [p for p in discovered if "/docs/auto/" in p]
    if generated:
        problems.append(
            f"'{DISCOVERY_PATHSPEC}' matched generated include(s) {generated} — the guard "
            "would demand a PDF beside them and fail on every run"
        )

    for doc in discovered:
        if not (REPO_ROOT / doc).with_suffix(".pdf").exists():
            problems.append(
                f"{doc} is discovered by the drift guard but has no committed PDF beside it; "
                "compile it with `just <project>::build-guide` and commit both"
            )

    # Control: without the magic, git's fnmatch spans '/' and the sets differ.
    # If this ever stops holding, the assertion above has become vacuous.
    bare = git_ls_files(BARE_PATHSPEC)
    if set(bare) == set(discovered):
        problems.append(
            f"control failed: '{BARE_PATHSPEC}' and '{DISCOVERY_PATHSPEC}' now return the same "
            "set, so this check no longer proves the ':(glob)' magic is doing anything. "
            "Re-derive git's pathspec behaviour before trusting the check above."
        )

    if verbose:
        print(f"discovery ({DISCOVERY_PATHSPEC}): {len(discovered)} document(s)")
        for d in discovered:
            print(f"  {d}")
        print(f"control  ({BARE_PATHSPEC}): {len(bare)} path(s)")
    return problems


def main() -> int:
    verbose = "--verbose" in sys.argv
    problems = check_literal_coordinates(verbose) + check_discovery_pathspec(verbose)

    if problems:
        print("Typst document checks FAILED:\n")
        for p in problems:
            print(f"  - {p}")
        print(
            "\nBoth failures are silent in CI otherwise: a literal recompiles identically,\n"
            "and a mis-globbed discovery either skips a document or demands a PDF that\n"
            "cannot exist. See tools/check-typst-docs.py for the cases behind each."
        )
        return 1

    print(
        "Typst documents OK: no literal hardware coordinates, discovery set is sound."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
