"""Every release-please package reports the version release-please gave it.

release-please's ``simple`` strategy bumps an existing ``version.txt`` in the
package directory and never creates one, and each firmware project's
CMakeLists.txt reads that file into its app version. A package without the
file therefore ships every release under its CMake fallback (0.1.0):
chatterbox at 1.1.5 and balancebot at 0.1.14 did, until #1078.

Stdlib unittest only, so the pre-commit hook runs it with nothing but python3:

    python3 -m unittest tools/test_release_please_versions.py
"""

from __future__ import annotations

import json
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CONFIG = ROOT / "release-please-config.json"
MANIFEST = ROOT / ".release-please-manifest.json"


def simple_packages() -> list[str]:
    """Package paths whose release type, own or inherited, is ``simple``."""
    config = json.loads(CONFIG.read_text())
    default = config.get("release-type")
    return sorted(
        path
        for path, package in config["packages"].items()
        if package.get("release-type", default) == "simple"
    )


def tracked(path: str) -> bool:
    out = subprocess.run(
        ["git", "-C", str(ROOT), "ls-files", "--error-unmatch", path],
        capture_output=True,
    )
    return out.returncode == 0


class ReleasePleaseVersionsTest(unittest.TestCase):
    def test_config_names_simple_packages(self):
        # Control: an empty list would pass every check below.
        self.assertIn("packages/robotics/robocar", simple_packages())

    def test_every_simple_package_has_a_version_txt_at_its_manifest_version(self):
        manifest = json.loads(MANIFEST.read_text())
        problems = []
        for package in simple_packages():
            version_txt = f"{package}/version.txt"
            if package not in manifest:
                problems.append(f"{package}: no entry in {MANIFEST.name}")
            elif not tracked(version_txt):
                problems.append(
                    f"{version_txt}: missing; seed it at {manifest[package]} "
                    "(release-please bumps the file but never creates it)"
                )
            else:
                text = (ROOT / version_txt).read_text().strip()
                if text != manifest[package]:
                    problems.append(
                        f"{version_txt}: {text!r}, manifest says {manifest[package]!r}"
                    )
        self.assertEqual(problems, [])

    def test_every_cmake_package_reads_its_version_txt(self):
        # A version.txt nothing reads is the same bug one step later: the
        # binary still reports the CMake fallback.
        unread = [
            package
            for package in simple_packages()
            if (ROOT / package / "CMakeLists.txt").is_file()
            and "version.txt" not in (ROOT / package / "CMakeLists.txt").read_text()
        ]
        self.assertEqual(unread, [])


if __name__ == "__main__":
    unittest.main()
