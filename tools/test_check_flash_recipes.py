"""Unit tests for every check in check-flash-recipes.py.

Stdlib unittest only, so the pre-commit hook can run it on a CI runner that
has nothing but python3:

    python3 -m unittest tools/test_check_flash_recipes.py

The dry-run text below is the shape `PORT=/dev/ttyDUMMY just --dry-run
<module>::flash` actually prints; the two lookups (CMake project name, flash
layout) are injected so no test depends on the real packages/ tree. The file-set
checks (attribute placement, justfile_directory(), the shared-recipe audit)
walk REPO_ROOT, so their tests build a temporary tree and point REPO_ROOT at it.
The otadata predicate (tools/lib/otadata-predicate.sh) is shared with the
release path, so its shell callers' use of it is tested here too: the predicate
directly, and tools/generate-flasher-manifests.sh on a throwaway tree.
"""

from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import os
import re
import subprocess
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
# An otadata row whose offset is blank, left to gen_esp32part.py to place (#689).
AUTO_OTADATA = cfr.FlashLayout(
    bootloader=0x1000,
    partition_table=0x8000,
    otadata=None,
    otadata_auto=True,
    app=0x10000,
    why={"otadata": "t.csv's otadata row has a blank offset"},
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

    def test_a_blank_offset_otadata_row_still_requires_otadata(self):
        # #689: the row exists, so the build writes the file and a full flash owes it.
        text = esptool(FULL[0], FULL[1], FULL[3])
        _, findings = audit(text, layouts={MAIN: AUTO_OTADATA})
        self.assertEqual([f.code for f in findings], ["OTADATA_UNWRITTEN"])


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

    def test_otadata_with_a_blank_table_offset_cannot_pass(self):
        # #689: the row exists but its offset is placed at build time, so the
        # offset in the recipe has nothing to be checked against.
        _, findings = audit(esptool(*FULL), layouts={MAIN: AUTO_OTADATA})
        self.assertEqual([f.code for f in findings], ["OFFSET_UNVERIFIABLE"])
        self.assertIn("blank offset", findings[0].detail)

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

    def custom(self, table: str, sdkconfig: str = "") -> None:
        """A project table the build reads: partitions.csv under CUSTOM=y."""
        self.write(
            "sdkconfig.defaults", "CONFIG_PARTITION_TABLE_CUSTOM=y\n" + sdkconfig
        )
        self.write("partitions.csv", table)

    def test_the_otadata_row_of_the_resolved_table_gives_its_offset(self):
        self.write(
            "sdkconfig.defaults",
            'CONFIG_PARTITION_TABLE_CUSTOM=y\nCONFIG_PARTITION_TABLE_CUSTOM_FILENAME="t.csv"\n',
        )
        self.write(
            "t.csv", "nvs,data,nvs,0x9000,0x4000,\notadata,data,ota,0xd000,0x2000,\n"
        )
        self.assertEqual(cfr.flash_layout(self.dir).otadata, 0xD000)

    def test_a_project_csv_is_ignored_unless_the_custom_table_is_selected(self):
        # #684, switch-usb-proxy's case: without CONFIG_PARTITION_TABLE_CUSTOM=y
        # ESP-IDF builds its built-in single-app table and never reads the CSV.
        self.write(
            "sdkconfig.defaults",
            'CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"\n',
        )
        self.write(
            "partitions.csv",
            "nvs,data,nvs,0x9000,0x4000,\notadata,data,ota,0xd000,0x2000,\n"
            "factory,app,factory,0x20000,1M,\n",
        )
        layout = cfr.flash_layout(self.dir)
        self.assertFalse(layout.has_otadata_row)
        self.assertEqual(layout.app, 0x10000)
        self.assertIn("partitions_singleapp.csv", layout.why["app"])

    def test_an_otadata_row_off_the_usual_address_keeps_its_offset(self):
        # llm-telegram's table: a larger nvs pushes otadata to 0xe000.
        self.custom(
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

    def test_a_blank_offset_otadata_row_is_a_row_without_an_offset(self):
        # #689: not "no row" — the build still writes ota_data_initial.bin.
        self.custom(
            "nvs,data,nvs,,0x4000,\notadata,data,ota,,0x2000,\n"
            "factory,app,factory,0x10000,1M,\n",
        )
        layout = cfr.flash_layout(self.dir)
        self.assertIsNone(layout.otadata)
        self.assertTrue(layout.otadata_auto)
        self.assertTrue(layout.has_otadata_row)
        self.assertIn("blank offset", layout.why["otadata"])

    def test_a_table_without_an_otadata_row_has_no_otadata_offset(self):
        self.custom(
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
        self.custom(
            "nvs,data,nvs,0xa000,0x6000,\nfactory,app,factory,0x10000,1M,\n",
            sdkconfig="CONFIG_PARTITION_TABLE_OFFSET=0x9000\n",
        )
        self.assertEqual(cfr.flash_layout(self.dir).partition_table, 0x9000)

    def test_the_app_goes_to_the_factory_partition_when_there_is_one(self):
        self.custom(
            "nvs,data,nvs,0x9000,0x4000,\notadata,data,ota,0xd000,0x2000,\n"
            "ota_0,app,ota_0,0x10000,1M,\nfactory,app,factory,0x110000,1M,\n",
        )
        self.assertEqual(cfr.flash_layout(self.dir).app, 0x110000)

    def test_without_a_factory_partition_the_app_goes_to_the_lowest_ota_slot(self):
        # parttool.py's boot-default search: factory, then ota_0 .. ota_15.
        self.custom(
            "otadata,data,ota,0xd000,0x2000,\n"
            "ota_1,app,ota_1,0x10000,1M,\nota_0,app,ota_0,0x110000,1M,\n",
        )
        self.assertEqual(cfr.flash_layout(self.dir).app, 0x110000)

    def test_an_auto_placed_app_partition_has_no_offset_to_check(self):
        self.custom("nvs,data,nvs,,0x6000,\nfactory,app,factory,,1M,\n")
        layout = cfr.flash_layout(self.dir)
        self.assertIsNone(layout.app)
        self.assertIn("factory", layout.why["app"])

    def test_a_table_without_an_app_partition_has_no_app_offset(self):
        self.custom("nvs,data,nvs,0x9000,0x6000,\n")
        self.assertIsNone(cfr.flash_layout(self.dir).app)

    def test_the_default_built_in_table_puts_the_app_at_0x10000(self):
        # No table selected: CONFIG_PARTITION_TABLE_SINGLE_APP, whose factory
        # gen_esp32part.py places at 0x10000 behind a table at 0x8000.
        layout = cfr.flash_layout(self.dir)
        self.assertEqual(layout.app, 0x10000)
        self.assertFalse(layout.has_otadata_row)

    def test_each_built_in_preset_reads_its_own_layout(self):
        # #684: offsets from ESP-IDF v5.4's gen_esp32part.py
        # (tools/lib/idf-partition-tables/), not one guess for every preset.
        for preset, otadata, app in (
            ("CONFIG_PARTITION_TABLE_SINGLE_APP=y", None, 0x10000),
            ("CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE=y", None, 0x10000),
            ("CONFIG_PARTITION_TABLE_TWO_OTA=y", 0xD000, 0x10000),
            # No factory: the app goes to ota_0, which the larger nvs pushes to 0x20000.
            ("CONFIG_PARTITION_TABLE_TWO_OTA_LARGE=y", 0xF000, 0x20000),
            ("CONFIG_PARTITION_TABLE_SINGLE_APP_ENCRYPTED_NVS=y", None, 0x10000),
            ("CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE_ENC_NVS=y", None, 0x10000),
            ("CONFIG_PARTITION_TABLE_TWO_OTA_ENCRYPTED_NVS=y", 0xD000, 0x10000),
        ):
            with self.subTest(preset=preset):
                self.write("sdkconfig.defaults", preset + "\n")
                layout = cfr.flash_layout(self.dir)
                self.assertEqual((layout.otadata, layout.app), (otadata, app))

    def test_a_core_dump_to_flash_selects_the_coredump_variant(self):
        # PARTITION_TABLE_FILENAME's defaults in components/partition_table/
        # Kconfig.projbuild; the coredump variant moves two_ota's ota_0.
        self.write(
            "sdkconfig.defaults",
            "CONFIG_PARTITION_TABLE_TWO_OTA=y\nCONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y\n",
        )
        layout = cfr.flash_layout(self.dir)
        self.assertIn("partitions_two_ota_coredump.csv", layout.why["app"])
        self.assertEqual(layout.otadata, 0xD000)

    def test_a_later_assignment_wins_as_in_sdkconfig(self):
        for lines, table in (
            (
                "CONFIG_PARTITION_TABLE_TWO_OTA=y\nCONFIG_PARTITION_TABLE_SINGLE_APP=y\n",
                "partitions_singleapp.csv",
            ),
            (
                "CONFIG_PARTITION_TABLE_TWO_OTA=y\n# CONFIG_PARTITION_TABLE_TWO_OTA is not set\n",
                "partitions_singleapp.csv",
            ),
            (
                "CONFIG_PARTITION_TABLE_TWO_OTA=y\nCONFIG_PARTITION_TABLE_TWO_OTA=n\n",
                "partitions_singleapp.csv",
            ),
            (
                "CONFIG_PARTITION_TABLE_SINGLE_APP=y\nCONFIG_PARTITION_TABLE_TWO_OTA=y\n",
                "partitions_two_ota.csv",
            ),
        ):
            with self.subTest(lines=lines):
                self.write("sdkconfig.defaults", lines)
                self.assertIn(table, cfr.flash_layout(self.dir).why["app"])

    def test_partition_table_filename_in_sdkconfig_defaults_is_ignored(self):
        # It has no prompt, so kconfig derives it and ignores a default for it
        # (checked with `idf.py reconfigure` in espressif/idf:v5.4): with
        # CUSTOM=y the table is CUSTOM_FILENAME, which defaults to partitions.csv.
        self.write(
            "sdkconfig.defaults",
            'CONFIG_PARTITION_TABLE_CUSTOM=y\nCONFIG_PARTITION_TABLE_FILENAME="other.csv"\n',
        )
        self.write("other.csv", "otadata,data,ota,0xd000,0x2000,\n")
        self.write(
            "partitions.csv",
            "nvs,data,nvs,0x9000,0x6000,\nfactory,app,factory,0x10000,1M,\n",
        )
        self.assertFalse(cfr.flash_layout(self.dir).has_otadata_row)

    def test_a_built_in_table_behind_a_moved_partition_table_fails_closed(self):
        # The resolved copies assume the table at 0x8000; gen_esp32part.py
        # places every partition after wherever the table actually is.
        self.write("sdkconfig.defaults", "CONFIG_PARTITION_TABLE_OFFSET=0x10000\n")
        with self.assertRaisesRegex(RuntimeError, "PARTITION_TABLE_OFFSET"):
            cfr.flash_layout(self.dir)

    def test_a_preset_with_no_table_for_its_options_fails_closed(self):
        # Kconfig gives TWO_OTA_LARGE no coredump variant, so no filename at all.
        self.write(
            "sdkconfig.defaults",
            "CONFIG_PARTITION_TABLE_TWO_OTA_LARGE=y\nCONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y\n",
        )
        with self.assertRaisesRegex(RuntimeError, "TWO_OTA_LARGE"):
            cfr.flash_layout(self.dir)

    def test_a_selected_custom_table_that_does_not_exist_fails_closed(self):
        # Reading a missing table as "no otadata row" would fail the check open.
        self.write("sdkconfig.defaults", "CONFIG_PARTITION_TABLE_CUSTOM=y\n")
        with self.assertRaisesRegex(RuntimeError, "partitions.csv"):
            cfr.flash_layout(self.dir)

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


PREDICATE = Path(__file__).with_name("lib") / "otadata-predicate.sh"
GENERATOR = Path(__file__).with_name("generate-flasher-manifests.sh")


class PredicateTests(unittest.TestCase):
    """tools/lib/otadata-predicate.sh, called the way its shell callers call it."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.dir = Path(self._tmp.name)

    def sh(self, call: str, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["bash", "-c", f'source "$0" || exit 99; {call}', str(PREDICATE), *args],
            capture_output=True,
            text=True,
            check=False,
        )

    def table(self, text: str) -> str:
        path = self.dir / "t.csv"
        path.write_text(text)
        return str(path)

    def test_an_explicit_otadata_offset_prints_in_decimal(self):
        csv = self.table(
            "nvs,data,nvs,0x9000,0x4000,\notadata,data,ota,0xd000,0x2000,\n"
        )
        self.assertEqual(self.sh('parse_otadata_offset "$1"', csv).stdout, "53248\n")

    def test_a_blank_otadata_offset_prints_auto(self):
        # #689: the third state, a row whose offset gen_esp32part.py places.
        csv = self.table("nvs,data,nvs,,0x4000,\notadata, data, ota, ,0x2000,\n")
        self.assertEqual(self.sh('parse_otadata_offset "$1"', csv).stdout, "auto\n")

    def test_no_otadata_row_prints_nothing(self):
        csv = self.table(
            "nvs,data,nvs,0x9000,0x6000,\nfactory,app,factory,0x10000,1M,\n"
        )
        proc = self.sh('parse_otadata_offset "$1"', csv)
        self.assertEqual((proc.returncode, proc.stdout), (0, "\n"))

    def test_a_blank_offset_row_counts_as_an_otadata_partition(self):
        # build-firmware.yml requires ota_data_initial.bin exactly when this holds.
        auto = self.table("otadata,data,ota,,0x2000,\n")
        self.assertEqual(self.sh('has_otadata_partition "$1"', auto).returncode, 0)
        none = self.table("nvs,data,nvs,0x9000,0x6000,\n")
        self.assertEqual(self.sh('has_otadata_partition "$1"', none).returncode, 1)

    def test_a_missing_table_is_an_error_not_no_row(self):
        # #684: resolve_partition_table names only tables that exist, so a
        # missing one is a broken call; reading it as "no row" fails open.
        missing = str(self.dir / "nope.csv")
        proc = self.sh('parse_otadata_offset "$1"', missing)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("nope.csv", proc.stderr)
        self.assertEqual(self.sh('has_otadata_partition "$1"', missing).returncode, 2)

    def test_a_built_in_preset_resolves_to_its_resolved_copy(self):
        (self.dir / "sdkconfig.defaults").write_text(
            "CONFIG_PARTITION_TABLE_TWO_OTA=y\n"
        )
        proc = self.sh('resolve_partition_table "$1"', str(self.dir))
        self.assertEqual(proc.returncode, 0, proc.stderr)
        table = Path(proc.stdout.strip())
        self.assertEqual(
            table, PREDICATE.parent / "idf-partition-tables/partitions_two_ota.csv"
        )
        self.assertTrue(table.is_file())


class BuiltInTableTripwireTests(unittest.TestCase):
    """The resolved built-in tables are ESP-IDF's for the version the builds use.

    tools/lib/idf-partition-tables/ holds gen_esp32part.py's output for one
    ESP-IDF version. An upgrade (issue #734) can move a preset's offsets, so
    this fails until IDF_VERSION names the new version, and the way to make it
    pass is to run regenerate.sh, which writes tables for that version.
    """

    TABLES = PREDICATE.parent / "idf-partition-tables"
    REPO = Path(__file__).resolve().parent.parent

    def test_the_tables_are_for_the_esp_idf_version_the_builds_use(self):
        version = (self.TABLES / "IDF_VERSION").read_text().strip()
        used = {
            "Dockerfile": re.search(
                r"^FROM espressif/idf:(\S+)",
                (self.REPO / "Dockerfile").read_text(),
                re.MULTILINE,
            ),
        }
        for workflow in ("build-firmware.yml", "_ci-build-esp32.yml"):
            text = (self.REPO / ".github/workflows" / workflow).read_text()
            used[workflow] = re.search(
                r"^\s*IDF_VERSION:\s*'([^']+)'", text, re.MULTILINE
            )
        for where, match in used.items():
            with self.subTest(where=where):
                self.assertIsNotNone(match, f"no ESP-IDF version found in {where}")
                self.assertEqual(
                    match.group(1),
                    version,
                    f"{where} builds with ESP-IDF {match.group(1)}, but the resolved "
                    f"built-in tables are from {version}: run "
                    "tools/lib/idf-partition-tables/regenerate.sh after setting IDF_VERSION",
                )

    def test_every_table_says_which_version_wrote_it(self):
        version = (self.TABLES / "IDF_VERSION").read_text().strip()
        tables = sorted(self.TABLES.glob("partitions_*.csv"))
        self.assertTrue(tables)
        for table in tables:
            with self.subTest(table=table.name):
                self.assertTrue(
                    table.read_text().startswith(f"# ESP-IDF {version} "),
                    "regenerate.sh did not write this table for IDF_VERSION",
                )


class ManifestGeneratorTests(unittest.TestCase):
    """tools/generate-flasher-manifests.sh's otadata part, run on a throwaway tree."""

    FLASHER = {
        "name": "Thing",
        "description": "d",
        "chipFamily": "ESP32",
        "board": "b",
        "appBinaryName": "thing.bin",
    }

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        self.project = self.root / "packages" / "demo" / "thing"
        self.project.mkdir(parents=True)
        (self.project / "flasher.json").write_text(json.dumps(self.FLASHER))
        (self.project / "sdkconfig.defaults").write_text(
            'CONFIG_PARTITION_TABLE_CUSTOM=y\nCONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"\n'
        )

    def generate(self, table: str) -> subprocess.CompletedProcess[str]:
        (self.project / "partitions.csv").write_text(table)
        return subprocess.run(
            ["bash", str(GENERATOR), "v1", "firmware"],
            cwd=self.root,
            env={**os.environ, "BUILD_SHA": "0" * 40},
            capture_output=True,
            text=True,
            check=False,
        )

    def parts(self) -> dict[str, int]:
        manifest = json.loads((self.root / "firmware/thing/manifest.json").read_text())
        return {p["path"]: p["offset"] for p in manifest["builds"][0]["parts"]}

    def test_an_explicit_otadata_row_is_a_manifest_part(self):
        proc = self.generate(
            "nvs,data,nvs,0x9000,0x4000,\notadata,data,ota,0xd000,0x2000,\n"
            "factory,app,factory,0x10000,1M,\n"
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(self.parts()["ota_data_initial.bin"], 0xD000)

    def test_a_blank_offset_otadata_row_fails_the_release(self):
        # #689: leaving the part out would publish a manifest that never writes
        # otadata; guessing its offset could write it over another partition.
        proc = self.generate(
            "nvs,data,nvs,,0x4000,\notadata,data,ota,,0x2000,\nfactory,app,factory,,1M,\n"
        )
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("otadata", proc.stderr)
        self.assertIn("thing", proc.stderr)
        self.assertFalse((self.root / "firmware/thing/manifest.json").exists())

    def test_a_built_in_two_ota_preset_lists_its_otadata(self):
        # #684: the preset has an otadata row although the project has no CSV.
        (self.project / "sdkconfig.defaults").write_text(
            "CONFIG_PARTITION_TABLE_TWO_OTA=y\n"
        )
        proc = subprocess.run(
            ["bash", str(GENERATOR), "v1", "firmware"],
            cwd=self.root,
            env={**os.environ, "BUILD_SHA": "0" * 40},
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(self.parts()["ota_data_initial.bin"], 0xD000)
        self.assertEqual(self.parts()["thing.bin"], 0x10000)


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

    def test_a_mention_only_in_a_trailing_comment_is_not_reported(self):
        # Issue #678: a correct line explaining what it avoids.
        self.write(
            "packages/a/justfile",
            'project_dir := source_directory() / "main"  # not justfile_directory(): see #605\n',
        )
        self.assertEqual(self.reported(), {})

    def test_a_hash_inside_a_string_does_not_start_a_comment(self):
        # The `#` is part of the string, so the use after it is real code.
        for line in (
            'x := "a#b" + justfile_directory()\n',
            "x := 'a # b' + justfile_directory()\n",
            'x := "a\\" # b" + justfile_directory()\n',
            '    echo "#" {{justfile_directory()}}\n',
        ):
            with self.subTest(line=line):
                self.write("packages/a/justfile", line)
                self.assertEqual(list(self.reported()), ["packages/a/justfile"])

    def test_a_hash_inside_a_word_does_not_start_a_comment(self):
        # Neither just nor sh starts a comment at a `#` glued to a word.
        self.write(
            "packages/a/justfile", "    curl url#frag {{justfile_directory()}}\n"
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

    def test_an_attribute_separated_from_its_recipe_by_a_blank_line_is_reported(self):
        # just 1.58.0 rejects this with `error: extraneous attribute` (#675).
        self.write("tools/esp32-idf.just", "[private]\n\n_x:\n    true\n")
        self.assertEqual(self.codes(), {"tools/esp32-idf.just": ["ORPHANED_ATTRIBUTE"]})

    def test_a_bracketed_shell_test_in_a_recipe_body_is_not_an_attribute(self):
        # `[ -f x ]` in an indented recipe line is shell, not a just attribute,
        # so a blank line after it orphans nothing.
        self.write(
            "tools/esp32-idf.just",
            "_x:\n    [ -f build/x.bin ]\n\n    true\n",
        )
        self.assertEqual(self.codes(), {})


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
            sdkconfig += (
                "CONFIG_PARTITION_TABLE_CUSTOM=y\n"
                'CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"\n'
            )
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

    def test_the_idf_two_ota_large_preset_misses_otadata_and_the_app_offset(self):
        # #684: this preset has no factory, and its ota_0 sits at 0x20000.
        self.consumer(
            sdkconfig=self.FOUR_MB + "CONFIG_PARTITION_TABLE_TWO_OTA_LARGE=y\n",
            table=None,
        )
        self.assertEqual(self.findings(), ["OTADATA_UNWRITTEN", "APP_OFFSET"])

    def test_a_csv_the_build_does_not_select_is_not_read(self):
        # #684: an otadata row in an unselected CSV is not in the build's table.
        self.consumer(
            sdkconfig=self.FOUR_MB,
            table=None,
        )
        self.write(
            "packages/demo/thing/partitions.csv",
            "nvs,data,nvs,0x9000,0x4000,\notadata,data,ota,0xd000,0x2000,\n"
            "ota_0,app,ota_0,0x20000,1M,\n",
        )
        self.assertEqual(self.findings(), [])

    def test_a_blank_offset_otadata_row_is_a_mismatch(self):
        # #689 through the shared-recipe audit: the row exists, nothing writes it.
        self.consumer(
            table="nvs,data,nvs,,0x4000,\notadata,data,ota,,0x2000,\n"
            "factory,app,factory,0x10000,1M,\n"
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


class OtaWriterTableTests(TempRepo):
    """A project that writes OTA images needs otadata and two app slots (#1076).

    The four ThinkPack boxes started the thinkpack-ota receiver on ESP-IDF's
    factory-only table, so esp_ota_get_next_update_partition() returned NULL
    and every apply failed at runtime. Nothing at build time noticed.
    """

    PROJECT = "packages/demo/box"
    CMAKE = (
        "cmake_minimum_required(VERSION 3.16)\n"
        "include($ENV{IDF_PATH}/tools/cmake/project.cmake)\n"
        "project(demo-box)\n"
    )
    RECEIVER = (
        "void app_main(void)\n{\n"
        "    ESP_ERROR_CHECK(thinkpack_ota_receiver_init(BOX_GLOWBUG));\n}\n"
    )
    TWO_SLOTS = (
        "nvs,data,nvs,0x9000,0x6000,\n"
        "phy_init,data,phy,0xf000,0x1000,\n"
        "otadata,data,ota,0x10000,0x2000,\n"
        "ota_0,app,ota_0,0x20000,0x1F0000,\n"
        "ota_1,app,ota_1,0x210000,0x1F0000,\n"
    )

    def project(self, main_c: str, sdkconfig: str = "", table: str | None = None):
        self.write(f"{self.PROJECT}/CMakeLists.txt", self.CMAKE)
        self.write(f"{self.PROJECT}/main/main.c", main_c)
        self.write(f"{self.PROJECT}/sdkconfig.defaults", sdkconfig)
        if table is not None:
            self.write(f"{self.PROJECT}/partitions.csv", table)

    def codes(self) -> list[str]:
        return [f.code for f in cfr.check_ota_writer_tables()]

    def test_a_receiver_on_the_default_factory_only_table_fails(self):
        # The #1076 state: CONFIG_PARTITION_TABLE_SINGLE_APP=y.
        self.project(self.RECEIVER, "CONFIG_PARTITION_TABLE_SINGLE_APP=y\n")
        self.assertEqual(self.codes(), ["OTA_TABLE"])

    def test_a_receiver_with_no_partition_line_at_all_fails(self):
        self.project(self.RECEIVER)
        self.assertEqual(self.codes(), ["OTA_TABLE"])

    def test_a_direct_apply_call_counts_as_writing_too(self):
        self.project("void f(void) { thinkpack_ota_apply(&cp, buf, n); }\n")
        self.assertEqual(self.codes(), ["OTA_TABLE"])

    def test_a_receiver_on_a_two_slot_table_passes(self):
        self.project(self.RECEIVER, "CONFIG_PARTITION_TABLE_CUSTOM=y\n", self.TWO_SLOTS)
        self.assertEqual(self.codes(), [])

    def test_the_built_in_two_ota_preset_passes(self):
        self.project(self.RECEIVER, "CONFIG_PARTITION_TABLE_TWO_OTA=y\n")
        self.assertEqual(self.codes(), [])

    def test_a_table_with_one_ota_slot_fails(self):
        one = "\n".join(self.TWO_SLOTS.splitlines()[:-1]) + "\n"
        self.project(self.RECEIVER, "CONFIG_PARTITION_TABLE_CUSTOM=y\n", one)
        self.assertEqual(self.codes(), ["OTA_TABLE"])

    def test_two_slots_without_an_otadata_row_fail(self):
        no_otadata = "".join(
            ln + "\n" for ln in self.TWO_SLOTS.splitlines() if "data,ota" not in ln
        )
        self.project(self.RECEIVER, "CONFIG_PARTITION_TABLE_CUSTOM=y\n", no_otadata)
        self.assertEqual(self.codes(), ["OTA_TABLE"])

    def test_a_factory_slot_does_not_count_as_an_ota_slot(self):
        factory = self.TWO_SLOTS.replace("ota_1,app,ota_1", "factory,app,factory")
        self.project(self.RECEIVER, "CONFIG_PARTITION_TABLE_CUSTOM=y\n", factory)
        self.assertEqual(self.codes(), ["OTA_TABLE"])

    def test_a_table_shared_from_a_parent_directory_is_read(self):
        # The ThinkPack boxes share packages/components/thinkpack-ota/partitions.csv.
        self.write("packages/components/ota/partitions.csv", self.TWO_SLOTS)
        self.project(
            self.RECEIVER,
            "CONFIG_PARTITION_TABLE_CUSTOM=y\n"
            'CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="../../components/ota/partitions.csv"\n',
        )
        self.assertEqual(self.codes(), [])

    def test_a_call_inside_a_comment_is_not_a_writer(self):
        self.project(
            "/* thinkpack_ota_receiver_init(BOX) is started elsewhere */\n"
            "// thinkpack_ota_apply(&cp, buf, n);\n"
        )
        self.assertEqual(self.codes(), [])

    def test_a_sender_that_only_links_the_component_is_not_a_writer(self):
        # brainbox: builds and broadcasts chunks, never applies an image.
        self.project("void push(void) { thinkpack_ota_sha256_verify(img, n, sha); }\n")
        self.assertEqual(self.codes(), [])

    def test_a_writer_in_a_nested_main_source_is_found(self):
        self.project("int x;\n")
        self.write(f"{self.PROJECT}/main/ota/rx.c", self.RECEIVER)
        self.assertEqual(self.codes(), ["OTA_TABLE"])

    def run_main(self) -> tuple[int, str]:
        # main() itself, so a finding the check returns but main() drops fails
        # here. The dry-run audit needs `just` and the real root justfile; it is
        # stubbed to report nothing, and the temp tree has no other inputs.
        out = io.StringIO()
        with (
            mock.patch.object(sys, "argv", ["check-flash-recipes.py"]),
            mock.patch.object(cfr.shutil, "which", return_value="just"),
            mock.patch.object(cfr, "check_flash_recipe_outputs", return_value=({}, [])),
            contextlib.redirect_stdout(out),
        ):
            code = cfr.main()
        return code, out.getvalue()

    def test_main_fails_on_a_receiver_with_a_single_app_table(self):
        self.project(self.RECEIVER, "CONFIG_PARTITION_TABLE_SINGLE_APP=y\n")
        code, out = self.run_main()
        self.assertIn(f"{self.PROJECT}: OTA_TABLE:", out)
        self.assertIn("STATUS=FAIL", out)
        self.assertEqual(code, 1)

    def test_main_passes_on_a_receiver_with_a_two_slot_table(self):
        # The control: the same tree with a table that can take an image.
        self.project(self.RECEIVER, "CONFIG_PARTITION_TABLE_CUSTOM=y\n", self.TWO_SLOTS)
        code, out = self.run_main()
        self.assertIn("STATUS=OK", out)
        self.assertEqual(code, 0)


class OtaWriterTableRepoTests(unittest.TestCase):
    """The real tree: the check sees the ThinkPack receivers and they pass."""

    def test_every_thinkpack_receiver_is_found_and_has_two_slots(self):
        writers = {p.name for p in cfr.ota_writer_projects() if "thinkpack" in str(p)}
        # A control: an empty set would pass the assertion below vacuously.
        self.assertEqual(writers, {"boombox", "chatterbox", "finderbox", "glowbug"})
        self.assertEqual(cfr.check_ota_writer_tables(), [])


if __name__ == "__main__":
    unittest.main()
