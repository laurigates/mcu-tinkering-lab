"""The frozen balancebot inputs behind #749's netlist export, for the unit tests.

`testdata/balancebot-carrier.xml` is KiCad's netlist export of #749's
schematic and `testdata/balancebot-carrier.net.json` is that export
normalised. The tests that compare a built netlist with them build it from
`testdata/balancebot-749/`, a frozen copy of the inputs it came from:
hardware.toml, pin_config.h, board.toml, the hand-kept `.kicad_pro`, the XIAO
RP2350 board page and the parts registry, plus the perfboard.toml the
perfboard tests lay out. They do not read the live project, so a legitimate balancebot
edit (a pin moved in pin_config.h, a part added) does not fail them. A few
tests do read it, as `--check` does: the committed schematic and the committed
perfboard outputs must be what the generators write. So after
such an edit the unit suite, and the kicad-carrier hook, pass once both
`just balancebot::carrier-schematic` and `just balancebot::perfboard` have run.
The check stamp (`kicad.stamp`: KiCad's own export of that schematic is the
join's netlist) also checks the live project, in CI only.

`hardware.toml` and `pin_config.h` carry a `.frozen` suffix, so nothing that
scans the repo for those names (the tools/hardware real-data tests, CI path
filters, pre-commit hooks) takes the copy for a project. `materialise()`
drops the suffix in a temp tree.

Edit the frozen files only when an input format changes (a renamed
hardware.toml key, a new required registry field), and keep what they build
equal to the fixture.
"""

from __future__ import annotations

import atexit
import functools
import shutil
import tempfile
from pathlib import Path

from .paths import REPO_ROOT

FROZEN = Path(__file__).resolve().parent / "testdata/balancebot-749"
PROJECT_REL = Path("packages/robotics/balancebot")
SUFFIX = ".frozen"


def materialise(root: Path) -> Path:
    """Write the frozen inputs under `root` in the repo layout; returns the project.

    The vendored library is a symlink at its repo path, so generated
    library-table paths have the committed depth.
    """
    shutil.copytree(FROZEN, root, dirs_exist_ok=True)
    for path in sorted(root.rglob(f"*{SUFFIX}")):
        path.rename(path.with_name(path.name.removesuffix(SUFFIX)))
    (root / "tools/kicad").mkdir(parents=True, exist_ok=True)
    (root / "tools/kicad/lib").symlink_to(REPO_ROOT / "tools/kicad/lib")
    # Hints name the project's root-justfile module (`paths.just_module`).
    (root / "justfile").write_text(f"mod balancebot '{PROJECT_REL.as_posix()}'\n")
    return root / PROJECT_REL


@functools.cache
def shared() -> tuple[Path, Path]:
    """One materialised tree per test run, for tests that only read it: (root, project)."""
    tmp = tempfile.TemporaryDirectory(prefix="kicad-749-")
    atexit.register(tmp.cleanup)
    root = Path(tmp.name)
    return root, materialise(root)
