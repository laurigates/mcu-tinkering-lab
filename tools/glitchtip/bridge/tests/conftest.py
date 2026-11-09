"""Shared fixtures."""

import shutil
import subprocess
from pathlib import Path

import pytest

TINY_C = Path(__file__).parents[2] / "fixtures" / "tiny.c"
ELF_MAGIC = b"\x7fELF"


@pytest.fixture(scope="session")
def tiny_elf(tmp_path_factory) -> Path:
    """Compile tiny.c to an ELF object with DWARF; skip if this host can't."""
    out = tmp_path_factory.mktemp("elf") / "fixture.elf"
    attempts = []
    if shutil.which("clang"):  # macOS: cc emits Mach-O, so ask clang for an ELF target
        attempts.append(["clang", "--target=x86_64-linux-gnu"])
    if shutil.which("cc"):
        attempts.append(["cc"])
    for cmd in attempts:
        done = subprocess.run(
            [*cmd, "-g", "-O0", "-c", "-o", str(out), str(TINY_C)],
            capture_output=True,
            check=False,
        )
        if done.returncode == 0 and out.read_bytes()[:4] == ELF_MAGIC:
            return out
    pytest.skip("no compiler here produces an ELF object")
