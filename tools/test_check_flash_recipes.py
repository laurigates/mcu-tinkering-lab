"""Unit tests for every check in check-flash-recipes.py.

Stdlib unittest only, so the pre-commit hook can run it on a CI runner that
has nothing but python3:

    python3 -m unittest tools/test_check_flash_recipes.py

The dry-run text below is the shape `PORT=/dev/ttyDUMMY just --dry-run
<module>::flash` actually prints; the two lookups (CMake project name, flash
layout) are injected so no test depends on the real packages/ tree. The file-set
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
# robocar-main's layout: esp32, default partition-table offset, otadata at 0xd000.
ESP32_OTA = cfr.FlashLayout(
    bootloader=0x1000, partition_table=0x8000, otadata=0xD000, app=0x10000
)
NO_OTADATA = cfr.FlashLayout(
    bootloader=0x1000, partition_table=0x8000, otadata=None, app=0x10000
)
LAYOUTS = {MAIN: ESP32_OTA}


def esptool(*parts: str) -> str:
    """A dry-run body in the shape robocar-main::flash prints."""
    return (
        "#!/usr/bin/env bash\nset -euo pipefail\n"
        'esptool --chip esp32 -p /dev/ttyDUMMY -b "${BAUD:-460800}" \\\n'
        "    write-flash --flash-mode dio --flash-size detect \\\n"
        + " \\\n".join(f"    {p}" for p in parts)
        + '\necho "Flashed OK"\n'
    )


def audit(text: str, module_dir: Path = MAIN, projects=None, layouts=None):
    projects = PROJECTS if projects is None else projects
    layouts = LAYOUTS if layouts is None else layouts
    return cfr.audit_dry_run(
        "mod::flash",
        module_dir,
        text,
        project_name=projects.get,
        layout=layouts.__getitem__,
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
        _, findings = audit(text, layouts={MAIN: NO_OTADATA})
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


class OffsetTests(unittest.TestCase):
    """Each file must sit where the target and partition table put it (#651)."""

    def codes(self, text: str, layout=ESP32_OTA) -> list[str]:
        _, findings = audit(text, layouts={MAIN: layout})
        return [f.code for f in findings]

    def test_every_part_at_its_layout_offset_passes(self):
        self.assertEqual(self.codes(esptool(*FULL)), [])

    def test_otadata_at_the_wrong_offset_fails(self):
        # The #651 negative control: llm-telegram's otadata row is at 0xe000.
        layout = cfr.FlashLayout(
            bootloader=0x1000, partition_table=0x8000, otadata=0xE000, app=0x10000
        )
        _, findings = audit(esptool(*FULL), layouts={MAIN: layout})
        self.assertEqual([f.code for f in findings], ["WRONG_OFFSET"])
        self.assertIn("build/ota_data_initial.bin", findings[0].detail)
        self.assertIn("0xd000", findings[0].detail)
        self.assertIn("0xe000", findings[0].detail)

    def test_the_app_at_the_wrong_offset_fails(self):
        text = esptool(*FULL[:3], "0x20000 build/idf-robocar.bin")
        self.assertEqual(self.codes(text), ["WRONG_OFFSET"])

    def test_the_partition_table_follows_its_configured_offset(self):
        layout = cfr.FlashLayout(
            bootloader=0x1000, partition_table=0x9000, otadata=0xD000, app=0x10000
        )
        self.assertEqual(self.codes(esptool(*FULL), layout), ["WRONG_OFFSET"])

    def test_an_esp32_bootloader_at_the_s3_offset_fails(self):
        text = esptool("0x0 build/bootloader/bootloader.bin", *FULL[1:])
        self.assertEqual(self.codes(text), ["WRONG_OFFSET"])

    def test_an_s3_bootloader_at_the_esp32_offset_fails(self):
        layout = cfr.FlashLayout(
            bootloader=0x0, partition_table=0x8000, otadata=0xD000, app=0x10000
        )
        self.assertEqual(self.codes(esptool(*FULL), layout), ["WRONG_OFFSET"])

    def test_every_wrong_part_is_reported_not_only_the_first(self):
        text = esptool(
            "0x0 build/bootloader/bootloader.bin",
            "0x9000 build/partition_table/partition-table.bin",
            "0xe000 build/ota_data_initial.bin",
            "0x20000 build/idf-robocar.bin",
        )
        self.assertEqual(self.codes(text), ["WRONG_OFFSET"] * 4)

    def test_decimal_and_uppercase_hex_offsets_are_read(self):
        text = esptool(
            "4096 build/bootloader/bootloader.bin",
            "0X8000 build/partition_table/partition-table.bin",
            "0xD000 build/ota_data_initial.bin",
            "65536 build/idf-robocar.bin",
        )
        self.assertEqual(self.codes(text), [])

    def test_a_wrong_decimal_offset_fails(self):
        # 57344 is 0xe000; a check that only read hex would skip it.
        text = esptool(*FULL[:2], "57344 build/ota_data_initial.bin", FULL[3])
        self.assertEqual(self.codes(text), ["WRONG_OFFSET"])

    def test_a_single_line_command_is_checked_too(self):
        # gamepad-synth's recipe prints every pair on one line.
        text = (
            "esptool --chip esp32 -p /dev/ttyDUMMY -b 460800 write_flash "
            "--flash-size 4MB --flash-freq 80m "
            "0x1000 build/bootloader/bootloader.bin "
            "0x8000 build/partition_table/partition-table.bin "
            "0xd000 build/ota_data_initial.bin 0x1000 build/idf-robocar.bin\n"
        )
        self.assertEqual(self.codes(text), ["WRONG_OFFSET"])

    def test_a_sibling_project_is_checked_against_its_own_layout(self):
        # robocar::flash-main lives in docs/ and flashes ../main/build/...
        text = esptool(
            *(f"{p.split()[0]} {DOCS}/../main/{p.split()[1]}" for p in FULL[:3]),
            f"0x20000 {DOCS}/../main/build/idf-robocar.bin",
        )
        _, findings = audit(text, module_dir=DOCS)
        self.assertEqual([f.code for f in findings], ["WRONG_OFFSET"])

    def test_a_part_whose_offset_is_unknown_cannot_pass(self):
        # An unrecognised target has no bootloader offset to compare against;
        # reading that as "fine" would fail the check open.
        layout = cfr.FlashLayout(
            bootloader=None,
            partition_table=0x8000,
            otadata=0xD000,
            app=0x10000,
            why={"bootloader": 'target "esp32x" is not one ESP-IDF v5.4 knows'},
        )
        _, findings = audit(esptool(*FULL), layouts={MAIN: layout})
        self.assertEqual([f.code for f in findings], ["OFFSET_UNVERIFIABLE"])
        self.assertIn("esp32x", findings[0].detail)

    def test_an_unknown_app_or_partition_table_offset_cannot_pass_either(self):
        for role in ("app", "partition_table"):
            with self.subTest(role=role):
                fields = dict(
                    bootloader=0x1000,
                    partition_table=0x8000,
                    otadata=0xD000,
                    app=0x10000,
                )
                fields[role] = None
                layout = cfr.FlashLayout(**fields, why={role: f"no {role} offset"})
                _, findings = audit(esptool(*FULL), layouts={MAIN: layout})
                self.assertEqual([f.code for f in findings], ["OFFSET_UNVERIFIABLE"])
                self.assertIn(f"no {role} offset", findings[0].detail)

    def test_otadata_named_without_an_otadata_row_is_not_offset_checked(self):
        # Without the row the build writes no ota_data_initial.bin, so esptool
        # fails loudly on the missing file; there is no offset to compare.
        self.assertEqual(self.codes(esptool(*FULL), NO_OTADATA), [])

    def test_a_bin_without_an_offset_in_front_is_not_offset_checked(self):
        text = "test -f build/idf-robocar.bin\n" + esptool(*FULL)
        self.assertEqual(self.codes(text), [])


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

    def test_the_otadata_row_of_the_resolved_table_gives_its_offset(self):
        self.write(
            "sdkconfig.defaults", 'CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="t.csv"\n'
        )
        self.write(
            "t.csv", "nvs,data,nvs,0x9000,0x4000,\notadata,data,ota,0xd000,0x2000,\n"
        )
        self.assertEqual(cfr.flash_layout(self.dir).otadata, 0xD000)

    def test_an_otadata_row_off_the_usual_address_keeps_its_offset(self):
        # llm-telegram's table: a larger nvs pushes otadata to 0xe000.
        self.write(
            "partitions.csv",
            "nvs,data,nvs,0x9000,0x5000,\notadata,data,ota,0xe000,0x2000,\n"
            "factory,app,factory,0x10000,1792K,\n",
        )
        self.assertEqual(cfr.flash_layout(self.dir).otadata, 0xE000)

    def test_a_predicate_that_fails_after_resolving_the_table_raises(self):
        # The table path is printed before the otadata lookup runs, so a
        # lookup that dies must not read as "no otadata row".
        stub = self.dir / "stub-predicate.sh"
        stub.write_text('resolve_partition_table() { echo "$1/partitions.csv"; }\n')
        with mock.patch.object(cfr, "OTADATA_PREDICATE", stub):
            with self.assertRaises(RuntimeError):
                cfr.flash_layout(self.dir)

    def test_a_table_without_an_otadata_row_has_no_otadata_offset(self):
        self.write(
            "partitions.csv",
            "nvs,data,nvs,0x9000,0x6000,\nfactory,app,factory,0x10000,1M,\n",
        )
        self.assertIsNone(cfr.flash_layout(self.dir).otadata)

    def test_a_predicate_that_cannot_run_raises_rather_than_reading_none(self):
        # Reading a broken predicate as "no otadata row" would fail the check open.
        missing = self.dir / "no-such-predicate.sh"
        with mock.patch.object(cfr, "OTADATA_PREDICATE", missing):
            with self.assertRaises(RuntimeError):
                cfr.flash_layout(self.dir)

    def test_the_bootloader_offset_follows_the_justfile_target(self):
        # components/bootloader/Kconfig.projbuild, BOOTLOADER_OFFSET_IN_FLASH.
        for target, offset in (
            ("esp32", 0x1000),
            ("esp32s2", 0x1000),
            ("esp32s3", 0x0),
            ("esp32c2", 0x0),
            ("esp32c3", 0x0),
            ("esp32c6", 0x0),
            ("esp32c61", 0x0),
            ("esp32h2", 0x0),
            ("esp32c5", 0x2000),
            ("esp32p4", 0x2000),
        ):
            with self.subTest(target=target):
                self.write("justfile", f'target := "{target}"\n')
                self.assertEqual(cfr.flash_layout(self.dir).bootloader, offset)

    def test_the_target_falls_back_to_sdkconfig_defaults(self):
        self.write("sdkconfig.defaults", 'CONFIG_IDF_TARGET="esp32s3"\n')
        self.assertEqual(cfr.flash_layout(self.dir).bootloader, 0x0)

    def test_the_justfile_target_wins_over_sdkconfig_defaults(self):
        # `just build` runs `idf.py set-target {{target}}`.
        self.write("sdkconfig.defaults", 'CONFIG_IDF_TARGET="esp32s3"\n')
        self.write("justfile", 'target := "esp32"\n')
        self.assertEqual(cfr.flash_layout(self.dir).bootloader, 0x1000)

    def test_a_missing_target_has_no_bootloader_offset(self):
        layout = cfr.flash_layout(self.dir)
        self.assertIsNone(layout.bootloader)
        self.assertIn("target", layout.why["bootloader"])

    def test_an_unknown_target_has_no_bootloader_offset(self):
        self.write("justfile", 'target := "esp32x"\n')
        layout = cfr.flash_layout(self.dir)
        self.assertIsNone(layout.bootloader)
        self.assertIn("esp32x", layout.why["bootloader"])

    def test_the_partition_table_offset_defaults_to_0x8000(self):
        self.assertEqual(cfr.flash_layout(self.dir).partition_table, 0x8000)

    def test_the_partition_table_offset_is_read_from_sdkconfig_defaults(self):
        self.write("sdkconfig.defaults", "CONFIG_PARTITION_TABLE_OFFSET=0x9000\n")
        self.assertEqual(cfr.flash_layout(self.dir).partition_table, 0x9000)

    def test_the_app_goes_to_the_factory_partition_when_there_is_one(self):
        self.write(
            "partitions.csv",
            "nvs,data,nvs,0x9000,0x4000,\notadata,data,ota,0xd000,0x2000,\n"
            "ota_0,app,ota_0,0x10000,1M,\nfactory,app,factory,0x110000,1M,\n",
        )
        self.assertEqual(cfr.flash_layout(self.dir).app, 0x110000)

    def test_without_a_factory_partition_the_app_goes_to_the_lowest_ota_slot(self):
        # parttool.py's boot-default search: factory, then ota_0 .. ota_15.
        self.write(
            "partitions.csv",
            "otadata,data,ota,0xd000,0x2000,\n"
            "ota_1,app,ota_1,0x10000,1M,\nota_0,app,ota_0,0x110000,1M,\n",
        )
        self.assertEqual(cfr.flash_layout(self.dir).app, 0x110000)

    def test_an_auto_placed_app_partition_has_no_offset_to_check(self):
        self.write(
            "partitions.csv", "nvs,data,nvs,,0x6000,\nfactory,app,factory,,1M,\n"
        )
        layout = cfr.flash_layout(self.dir)
        self.assertIsNone(layout.app)
        self.assertIn("factory", layout.why["app"])

    def test_a_table_without_an_app_partition_has_no_app_offset(self):
        self.write("partitions.csv", "nvs,data,nvs,0x9000,0x6000,\n")
        self.assertIsNone(cfr.flash_layout(self.dir).app)

    def test_a_built_in_table_puts_the_app_at_0x10000(self):
        # No partitions.csv: ESP-IDF's presets place factory after nvs and
        # phy_init, which lands on 0x10000 behind a table at 0x8000.
        self.assertEqual(cfr.flash_layout(self.dir).app, 0x10000)

    def test_a_built_in_table_behind_a_moved_partition_table_is_not_guessed(self):
        self.write("sdkconfig.defaults", "CONFIG_PARTITION_TABLE_OFFSET=0x10000\n")
        self.assertIsNone(cfr.flash_layout(self.dir).app)

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
