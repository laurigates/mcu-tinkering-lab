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
import re
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


_VERSION_READ = re.compile(
    r"^\s*file\s*\(\s*(READ|STRINGS)\s+[^)\n]*version\.txt", re.MULTILINE
)


def reads_version_txt(cmake: str) -> bool:
    """Whether a CMakeLists.txt reads version.txt with file(READ|STRINGS ...).

    Comment lines are dropped first: a comment that names the file is not a
    read, and neither is the ``if(EXISTS ...)`` guard around one.
    """
    code = "\n".join(
        line for line in cmake.splitlines() if not line.lstrip().startswith("#")
    )
    return bool(_VERSION_READ.search(code))


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
            and not reads_version_txt((ROOT / package / "CMakeLists.txt").read_text())
        ]
        self.assertEqual(unread, [])

    def test_a_comment_naming_version_txt_is_not_a_read(self):
        # The check above once looked for the bare substring, which the
        # comment over each read satisfies on its own.
        read = 'file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" PROJECT_VER)\n'
        comment = "# Read version from release-please managed version.txt\n"
        self.assertTrue(reads_version_txt(comment + read))
        self.assertTrue(reads_version_txt("  file( STRINGS version.txt V )\n"))
        self.assertFalse(reads_version_txt(comment + "project(x VERSION 0.1.0)\n"))
        self.assertFalse(reads_version_txt("# " + read))
        self.assertFalse(
            reads_version_txt('if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")\n')
        )


if __name__ == "__main__":
    unittest.main()
