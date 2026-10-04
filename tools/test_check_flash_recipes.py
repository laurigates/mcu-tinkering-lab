"""Unit tests for the hand-written flash-recipe audit in check-flash-recipes.py.

Stdlib unittest only, so the pre-commit hook can run it on a CI runner that
has nothing but python3:

    python3 -m unittest tools/test_check_flash_recipes.py

The dry-run text below is the shape `PORT=/dev/ttyDUMMY just --dry-run
<module>::flash` actually prints; the two lookups (CMake project name, otadata
row) are injected so no test depends on the real packages/ tree.
"""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

_SPEC = importlib.util.spec_from_file_location(
    "check_flash_recipes", Path(__file__).with_name("check-flash-recipes.py")
)
cfr = importlib.util.module_from_spec(_SPEC)
assert _SPEC.loader is not None
# Registered before exec: @dataclass resolves its module through sys.modules.
sys.modules[_SPEC.name] = cfr
_SPEC.loader.exec_module(cfr)

MAIN = Path("/repo/packages/robocar/main")
DOCS = Path("/repo/packages/robocar/docs")

PROJECTS = {MAIN: "idf-robocar"}
OTADATA = {MAIN}


def esptool(*parts: str) -> str:
    """A dry-run body in the shape robocar-main::flash prints."""
    return (
        "#!/usr/bin/env bash\nset -euo pipefail\n"
        'esptool --chip esp32 -p /dev/ttyDUMMY -b "${BAUD:-460800}" \\\n'
        "    write-flash --flash-mode dio --flash-size detect \\\n"
        + " \\\n".join(f"    {p}" for p in parts)
        + '\necho "Flashed OK"\n'
    )


def audit(text: str, module_dir: Path = MAIN, projects=None, otadata=None):
    projects = PROJECTS if projects is None else projects
    otadata = OTADATA if otadata is None else otadata
    return cfr.audit_dry_run(
        "mod::flash",
        module_dir,
        text,
        project_name=projects.get,
        has_otadata=lambda d: d in otadata,
    )


FULL = (
    "0x1000 build/bootloader/bootloader.bin",
    "0x8000 build/partition_table/partition-table.bin",
    "0xd000 build/ota_data_initial.bin",
    "0x10000 build/idf-robocar.bin",
)


class BinPathTests(unittest.TestCase):
    def test_a_recipe_naming_only_build_outputs_passes(self):
        status, findings = audit(esptool(*FULL))
        self.assertEqual(status, "checked")
        self.assertEqual(findings, [])

    def test_an_app_bin_not_named_after_the_cmake_project_fails(self):
        # The #598 bug: the recipe flashed robocar-main.bin, the build writes
        # idf-robocar.bin.
        text = esptool(*FULL[:3], "0x10000 build/robocar-main.bin")
        _, findings = audit(text)
        self.assertEqual([f.code for f in findings], ["UNKNOWN_BIN"])
        self.assertIn("build/robocar-main.bin", findings[0].detail)
        self.assertIn("idf-robocar.bin", findings[0].detail)

    def test_a_misspelled_fixed_output_fails(self):
        text = esptool("0x1000 build/bootloader.bin", *FULL[1:])
        _, findings = audit(text)
        self.assertEqual([f.code for f in findings], ["UNKNOWN_BIN"])

    def test_a_path_into_a_sibling_project_is_checked_against_that_project(self):
        # robocar::flash-main lives in docs/ and flashes ../main/build/...
        text = esptool(*(f"{p.split()[0]} {DOCS}/../main/{p.split()[1]}" for p in FULL))
        status, findings = audit(text, module_dir=DOCS)
        self.assertEqual(status, "checked")
        self.assertEqual(findings, [])

    def test_a_build_dir_that_is_not_an_esp_idf_project_fails(self):
        text = esptool(
            *(f"{p.split()[0]} {DOCS}/../nowhere/{p.split()[1]}" for p in FULL)
        )
        _, findings = audit(text, module_dir=DOCS)
        self.assertTrue(findings)
        self.assertEqual({f.code for f in findings}, {"NOT_A_PROJECT"})


class OtadataTests(unittest.TestCase):
    def test_a_full_flash_that_skips_otadata_on_an_ota_table_fails(self):
        # The robocar-camera half of #598.
        text = esptool(FULL[0], FULL[1], FULL[3])
        _, findings = audit(text)
        self.assertEqual([f.code for f in findings], ["OTADATA_UNWRITTEN"])

    def test_a_full_flash_without_otadata_is_fine_on_a_table_without_one(self):
        text = esptool(FULL[0], FULL[1], FULL[3])
        _, findings = audit(text, otadata=set())
        self.assertEqual(findings, [])

    def test_a_comment_naming_otadata_does_not_count_as_writing_it(self):
        # A shebang recipe's dry-run prints its comments verbatim.
        text = "# also writes build/ota_data_initial.bin\n" + esptool(
            FULL[0], FULL[1], FULL[3]
        )
        _, findings = audit(text)
        self.assertEqual([f.code for f in findings], ["OTADATA_UNWRITTEN"])

    def test_an_app_only_flash_is_not_required_to_write_otadata(self):
        # Without the partition table it is not a full flash; leaving otadata
        # alone is then the point of the recipe.
        _, findings = audit(esptool(FULL[3]))
        self.assertEqual(findings, [])


class ScopeTests(unittest.TestCase):
    def test_a_recipe_that_does_not_run_esptool_is_skipped(self):
        status, findings = audit("esphome run esp32-wireguard-ha.yaml\n")
        self.assertEqual(status, "not-esptool")
        self.assertEqual(findings, [])

    def test_the_build_generated_argfile_is_accepted(self):
        # ESP-IDF writes build/flash_args itself, otadata included.
        text = "cd build && esptool --chip esp32s3 -p /dev/ttyDUMMY write-flash @flash_args\n"
        status, findings = audit(text)
        self.assertEqual(status, "argfile")
        self.assertEqual(findings, [])

    def test_esptool_with_neither_bins_nor_an_argfile_fails(self):
        status, findings = audit(
            "esptool --chip esp32 -p /dev/ttyDUMMY write-flash 0x0 $IMG\n"
        )
        self.assertEqual(status, "checked")
        self.assertEqual([f.code for f in findings], ["NO_BUILD_OUTPUTS"])


class LookupTests(unittest.TestCase):
    """The two lookups the audit tests above inject, run against real files."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def write(self, name: str, text: str) -> None:
        (self.dir / name).write_text(text)

    def test_an_otadata_row_in_the_resolved_table_reads_true(self):
        self.write(
            "sdkconfig.defaults", 'CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="t.csv"\n'
        )
        self.write(
            "t.csv", "nvs,data,nvs,0x9000,0x4000,\notadata,data,ota,0xd000,0x2000,\n"
        )
        self.assertTrue(cfr.otadata_partition(self.dir))

    def test_a_table_without_an_otadata_row_reads_false(self):
        self.write(
            "partitions.csv",
            "nvs,data,nvs,0x9000,0x6000,\nfactory,app,factory,0x10000,1M,\n",
        )
        self.assertFalse(cfr.otadata_partition(self.dir))

    def test_a_predicate_that_cannot_run_raises_rather_than_reading_false(self):
        # Reading a broken predicate as "no otadata row" would fail the check open.
        missing = self.dir / "no-such-predicate.sh"
        with mock.patch.object(cfr, "OTADATA_PREDICATE", missing):
            with self.assertRaises(RuntimeError):
                cfr.otadata_partition(self.dir)

    def test_the_cmake_project_name_names_the_app_file(self):
        self.write(
            "CMakeLists.txt",
            "cmake_minimum_required(VERSION 3.16)\n"
            "# project(old-name)\n"
            "include($ENV{IDF_PATH}/tools/cmake/project.cmake)\n"
            "project(idf-robocar)\n",
        )
        self.assertEqual(cfr.cmake_project_name(self.dir), "idf-robocar")

    def test_a_cmake_project_that_is_not_esp_idf_has_no_name(self):
        self.write(
            "CMakeLists.txt",
            "cmake_minimum_required(VERSION 3.13)\nproject(balancebot)\n",
        )
        self.assertIsNone(cfr.cmake_project_name(self.dir))

    def test_a_directory_without_cmakelists_has_no_name(self):
        self.assertIsNone(cfr.cmake_project_name(self.dir))


if __name__ == "__main__":
    unittest.main()
