"""Unit tests for every check in check-flash-recipes.py.

Stdlib unittest only, so the pre-commit hook can run it on a CI runner that
has nothing but python3:

    python3 -m unittest tools/test_check_flash_recipes.py

The dry-run text below is the shape `PORT=/dev/ttyDUMMY just --dry-run
<module>::flash` actually prints; the two lookups (CMake project name, otadata
row) are injected so no test depends on the real packages/ tree. The file-set
checks (attribute placement, justfile_directory(), the shared-recipe audit)
walk REPO_ROOT, so their tests build a temporary tree and point REPO_ROOT at it.
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


# ---------------------------------------------------------------------------
# The file-set checks: each walks REPO_ROOT with its own globs and skip rules,
# so these tests point REPO_ROOT at a temporary tree. A glob or skip-rule edit
# that stops a check from seeing a file then fails here instead of passing on
# everything while pre-commit stays green (issue #634).
# ---------------------------------------------------------------------------


class TempRepo(unittest.TestCase):
    """A throwaway repo tree with REPO_ROOT patched to point at it."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        patcher = mock.patch.object(cfr, "REPO_ROOT", self.root)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.addCleanup(self._tmp.cleanup)

    def write(self, rel: str, text: str) -> None:
        path = self.root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)


class JustfileDirectoryTests(TempRepo):
    USES = 'project_dir := justfile_directory() / "main"\n\nbuild:\n    echo hi\n'

    def reported(self) -> dict[str, list[str]]:
        found: dict[str, list[str]] = {}
        for f in cfr.check_justfile_directory():
            self.assertEqual(f.code, "JUSTFILE_DIRECTORY_IN_MODULE")
            found.setdefault(f.project, []).append(f.detail)
        return found

    def test_a_module_justfile_under_packages_is_reported(self):
        self.write("packages/robocar/docs/justfile", self.USES)
        self.assertEqual(list(self.reported()), ["packages/robocar/docs/justfile"])

    def test_a_just_import_under_packages_is_reported(self):
        self.write("packages/audio/shared/helpers.just", self.USES)
        self.assertEqual(list(self.reported()), ["packages/audio/shared/helpers.just"])

    def test_a_module_justfile_under_docs_is_reported(self):
        # `mod schematics 'docs/schematics'` lives outside packages/ and tools/.
        self.write("docs/schematics/justfile", self.USES)
        self.assertEqual(list(self.reported()), ["docs/schematics/justfile"])

    def test_tools_imports_and_justfiles_are_reported(self):
        self.write("tools/esp32.just", self.USES)
        self.write("tools/hardware/justfile", self.USES)
        self.assertEqual(
            sorted(self.reported()), ["tools/esp32.just", "tools/hardware/justfile"]
        )

    def test_the_finding_names_the_offending_line(self):
        self.write("packages/a/justfile", "# header\n\n" + self.USES)
        self.assertEqual(
            [d.split(":")[0] for d in self.reported()["packages/a/justfile"]],
            ["line 3"],
        )

    def test_a_whole_line_comment_mentioning_it_is_not_reported(self):
        # tools/esp32.just explains the trap in prose; that must not report itself.
        self.write(
            "tools/esp32.just",
            "# In a module, justfile_directory() is the ROOT's directory.\n"
            "    # use source_directory(), never justfile_directory()\n"
            "project_dir := source_directory()\n",
        )
        self.assertEqual(self.reported(), {})

    def test_a_real_use_with_a_trailing_comment_is_still_reported(self):
        # Only WHOLE-line comments are skipped; a skip on any `#` would hide this.
        self.write(
            "packages/a/justfile",
            'project_dir := justfile_directory() / "main"  # build root\n',
        )
        self.assertEqual(list(self.reported()), ["packages/a/justfile"])

    def test_the_root_justfile_may_use_it(self):
        # There justfile_directory() is its own directory, which is correct.
        self.write("justfile", self.USES)
        self.assertEqual(self.reported(), {})


class AttributePlacementTests(TempRepo):
    def codes(self) -> dict[str, list[str]]:
        found: dict[str, list[str]] = {}
        for f in cfr.check_attribute_placement():
            found.setdefault(f.project, []).append(f.code)
        return found

    ORPHANED = "[private]\n# Flash an ESP32-S3.\n_s3-flash bin:\n    esptool\n"

    def test_an_attribute_separated_from_its_recipe_by_a_comment_is_reported(self):
        # The #478 break: a comment inserted above `_s3-flash bin:` in a shared import.
        self.write("tools/esp32-idf.just", self.ORPHANED)
        self.assertEqual(self.codes(), {"tools/esp32-idf.just": ["ORPHANED_ATTRIBUTE"]})

    def test_package_and_root_justfiles_are_checked_too(self):
        self.write("packages/robocar/unified/justfile", self.ORPHANED)
        self.write("justfile", self.ORPHANED)
        self.assertEqual(
            sorted(self.codes()), ["justfile", "packages/robocar/unified/justfile"]
        )

    def test_a_comment_after_a_blank_line_is_still_reported(self):
        self.write("tools/esp32-idf.just", "[private]\n\n# note\n_x:\n    true\n")
        self.assertEqual(self.codes(), {"tools/esp32-idf.just": ["ORPHANED_ATTRIBUTE"]})

    def test_an_attribute_directly_above_its_recipe_is_fine(self):
        self.write(
            "tools/esp32-idf.just",
            "# Flash an ESP32-S3.\n[private]\n[no-cd]\n_s3-flash bin:\n    esptool\n",
        )
        self.assertEqual(self.codes(), {})

    @unittest.expectedFailure
    def test_an_attribute_separated_from_its_recipe_by_a_blank_line_is_reported(self):
        # just 1.58.0 rejects this with `error: extraneous attribute`, but the
        # check skips blank lines as tolerated. Issue #675; remove this
        # decorator with the fix.
        self.write("tools/esp32-idf.just", "[private]\n\n_x:\n    true\n")
        self.assertEqual(self.codes(), {"tools/esp32-idf.just": ["ORPHANED_ATTRIBUTE"]})


class SharedRecipeTests(TempRepo):
    """collect() + audit(): a shared-flash consumer must fit the recipe's baked-in layout."""

    FACTORY_TABLE = "nvs,data,nvs,0x9000,0x6000,\nfactory,app,factory,0x10000,1M,\n"
    FOUR_MB = 'CONFIG_ESPTOOLPY_FLASHSIZE="4MB"\n'

    def consumer(
        self,
        *,
        shared: str = "_s3-flash",
        target: str = "esp32s3",
        sdkconfig: str = FOUR_MB,
        table: str | None = FACTORY_TABLE,
        rel: str = "packages/demo/thing",
    ) -> None:
        self.write(
            f"{rel}/justfile",
            f'target := "{target}"\nbin_name := "thing"\n\n'
            f"flash: ({shared} bin_name)\n",
        )
        if table is not None:
            sdkconfig += 'CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"\n'
            self.write(f"{rel}/partitions.csv", table)
        self.write(f"{rel}/sdkconfig.defaults", sdkconfig)

    def findings(self) -> list[str]:
        projects = cfr.collect()
        for proj in projects:
            cfr.audit(proj)
        return [f.code for p in projects for f in p.findings]

    def test_a_consumer_that_fits_has_no_findings(self):
        # The control: the consumer() defaults. Every mismatch test below
        # overrides exactly one of them.
        self.consumer()
        projects = cfr.collect()
        self.assertEqual([p.name for p in projects], ["packages/demo/thing"])
        self.assertEqual(self.findings(), [])

    def test_an_8mb_part_on_the_4mb_s3_recipe_is_a_mismatch(self):
        self.consumer(sdkconfig='CONFIG_ESPTOOLPY_FLASHSIZE="8MB"\n')
        self.assertEqual(self.findings(), ["FLASH_SIZE"])

    def test_an_8mb_part_on_the_esp32_recipe_is_fine(self):
        # _esp32-flash passes `--flash-size detect`, so any size fits.
        self.consumer(
            shared="_esp32-flash",
            target="esp32",
            sdkconfig='CONFIG_ESPTOOLPY_FLASHSIZE="8MB"\n',
        )
        self.assertEqual(self.findings(), [])

    def test_an_otadata_partition_is_a_mismatch(self):
        self.consumer(
            table="nvs,data,nvs,0x9000,0x4000,\notadata,data,ota,0xd000,0x2000,\n"
            "ota_0,app,ota_0,0x10000,1M,\n"
        )
        self.assertEqual(self.findings(), ["OTADATA_UNWRITTEN"])

    def test_the_idf_two_ota_preset_is_a_mismatch(self):
        # The preset replaces the custom table, so it is the one input changed.
        self.consumer(
            sdkconfig=self.FOUR_MB + "CONFIG_PARTITION_TABLE_TWO_OTA=y\n", table=None
        )
        self.assertEqual(self.findings(), ["OTADATA_UNWRITTEN"])

    def test_an_app_not_at_0x10000_is_a_mismatch(self):
        self.consumer(
            table="nvs,data,nvs,0x9000,0x6000,\nfactory,app,factory,0x20000,1M,\n"
        )
        self.assertEqual(self.findings(), ["APP_OFFSET"])

    def test_a_target_the_recipe_does_not_flash_is_a_mismatch(self):
        self.consumer(target="esp32")
        self.assertEqual(self.findings(), ["TARGET_MISMATCH"])

    def test_a_consumer_with_dependencies_before_the_group_is_collected(self):
        self.write(
            "packages/demo/thing/justfile",
            'target := "esp32s3"\nflash: credentials (_s3-flash bin_name)\n',
        )
        self.write(
            "packages/demo/thing/sdkconfig.defaults",
            'CONFIG_ESPTOOLPY_FLASHSIZE="8MB"\n',
        )
        self.assertEqual(self.findings(), ["FLASH_SIZE"])

    def test_prose_naming_a_shared_recipe_is_not_a_consumer(self):
        # robocar/unified explains in a comment why it stayed inline.
        self.write(
            "packages/robocar/unified/justfile",
            "# Not `flash: (_s3-flash bin_name)`: this is an 8 MB part.\n"
            "flash:\n    esptool write-flash @flash_args\n",
        )
        self.write(
            "packages/robocar/unified/sdkconfig.defaults",
            'CONFIG_ESPTOOLPY_FLASHSIZE="8MB"\n',
        )
        self.assertEqual(cfr.collect(), [])


if __name__ == "__main__":
    unittest.main()
