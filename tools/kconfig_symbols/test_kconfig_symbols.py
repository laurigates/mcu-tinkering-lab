"""Tests for kconfig_symbols: the parsers, the per-project check, and the repo.

Run: python3 -m unittest discover -s tools/kconfig_symbols -t tools
"""

from __future__ import annotations

import contextlib
import io
import re
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

from kconfig_symbols import check, extract
from kconfig_symbols.symbols import (
    assignments,
    format_list,
    kconfig_symbols,
    lock_versions,
    manifest_dependencies,
    parse_list,
    rename_symbols,
    tree_symbols,
)

REPO = Path(__file__).resolve().parents[2]


def _write(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(textwrap.dedent(text), encoding="utf-8")
    return path


def _list(component: str, version: str, symbols: set[str], depends: str = ""):
    header = {"component": component, "version": version, "depends": depends}
    return parse_list(format_list(header, symbols))


class ParserTests(unittest.TestCase):
    def test_config_and_menuconfig_define_symbols_choice_does_not(self):
        text = """
            menu "Example"
                config FOO
                    bool "foo"
                menuconfig BAR_BAZ
                    bool "bar"
                choice QUX_MODE
                    prompt "mode"
                    config QUX_A
                        bool "a"
                endchoice
            endmenu
            """
        self.assertEqual(kconfig_symbols(text), {"FOO", "BAR_BAZ", "QUX_A"})

    def test_trailing_comment_after_the_symbol_name_still_defines_it(self):
        # ESP-IDF v5.4 esp_driver_cam/Kconfig:17 is
        # `config CAM_CTLR_ISP_DVP_ISR_IRAM_SAFE  # remove this ...`.
        text = (
            "config CAM_CTLR_ISP_DVP_ISR_IRAM_SAFE  # remove when DVP is done\n"
            "    bool\n"
            "menuconfig WITH_TAB\t# note\n"
            "config NOT A_SYMBOL\n"
        )
        self.assertEqual(
            kconfig_symbols(text), {"CAM_CTLR_ISP_DVP_ISR_IRAM_SAFE", "WITH_TAB"}
        )

    def test_rename_keeps_old_names_including_inverted(self):
        text = "# comment\nCONFIG_OLD_ONE CONFIG_NEW_ONE\nCONFIG_OLD_TWO  !CONFIG_NEW_TWO\n"
        self.assertEqual(rename_symbols(text), {"OLD_ONE", "OLD_TWO"})

    def test_assignments_read_set_and_unset_lines_only(self):
        text = (
            "# A comment naming CONFIG_NOT_ME\n"
            "CONFIG_A=y\n"
            "\n"
            '  CONFIG_B="x"\n'
            "# CONFIG_C is not set\n"
            "CONFIG_D_NOT_AN_ASSIGNMENT\n"
        )
        self.assertEqual(assignments(text), [(2, "A"), (4, "B"), (5, "C")])

    def test_manifest_dependencies(self):
        text = """
            ## comment
            version: "1.0.0"
            dependencies:
              idf:
                version: '>=5.4'
              # mDNS
              espressif/mdns: '^1.8.0'
              esp_tinyusb:
                version: ">=1.4.0"
              "esp-idf-lib/i2cdev": "^2.0.0"
            targets:
              - esp32s3
            """
        self.assertEqual(
            manifest_dependencies(textwrap.dedent(text)),
            ["espressif/mdns", "espressif/esp_tinyusb", "esp-idf-lib/i2cdev"],
        )

    def test_manifest_with_empty_dependencies(self):
        self.assertEqual(manifest_dependencies("dependencies: {}\n"), [])

    def test_lock_versions(self):
        text = """
            dependencies:
              espressif/esp_tinyusb:
                component_hash: abc
                dependencies:
                - name: espressif/tinyusb
                  version: '>=0.14.2'
                source:
                  registry_url: https://components.espressif.com/
                  type: service
                version: 1.7.6~2
              idf:
                source:
                  type: idf
                version: 5.4.0
            direct_dependencies:
            - espressif/esp_tinyusb
            target: esp32s3
            version: 2.0.0
            """
        self.assertEqual(
            lock_versions(textwrap.dedent(text)), {"espressif/esp_tinyusb": "1.7.6~2"}
        )

    def test_list_round_trip(self):
        parsed = _list("a/b", "1.0.0", {"Y", "X"}, depends="c/d e/f")
        self.assertEqual(parsed.symbols, {"X", "Y"})
        self.assertEqual(parsed.depends, ["c/d", "e/f"])
        self.assertEqual(parsed.header["version"], "1.0.0")

    def test_tree_skips_test_example_and_build_dirs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            _write(root / "comp" / "Kconfig", "config REAL\n")
            _write(root / "comp" / "sdkconfig.rename", "CONFIG_OLD CONFIG_REAL\n")
            _write(
                root / "comp" / "test_apps" / "main" / "Kconfig.projbuild", "config T\n"
            )
            _write(root / "comp" / "examples" / "a" / "Kconfig.projbuild", "config E\n")
            _write(root / "build" / "Kconfig", "config GEN\n")
            self.assertEqual(tree_symbols(root), {"REAL", "OLD"})


class ExtractTests(unittest.TestCase):
    """The writer of the committed lists, run on synthetic trees."""

    def test_managed_component_list_names_version_and_dependencies(self):
        with tempfile.TemporaryDirectory() as tmp:
            comp = Path(tmp) / "managed_components" / "espressif__esp32-camera"
            _write(
                comp / "idf_component.yml",
                "version: 2.1.9\ndependencies:\n  idf: '>=5.1'\n  espressif/esp_jpeg: '*'\n",
            )
            _write(comp / "Kconfig", "config CAMERA_CORE1\n")
            _write(
                comp / "examples" / "x" / "Kconfig.projbuild", "config EXAMPLE_ONLY\n"
            )
            out = Path(tmp) / "out"
            self.assertEqual(
                extract.main(["component", str(comp), "--out-dir", str(out)]), 0
            )
            parsed = parse_list((out / "espressif__esp32-camera@2.1.9.txt").read_text())
        self.assertEqual(parsed.header["component"], "espressif/esp32-camera")
        self.assertEqual(parsed.depends, ["espressif/esp_jpeg"])
        self.assertEqual(parsed.symbols, {"CAMERA_CORE1"})

    def test_external_checkout_needs_a_version(self):
        with tempfile.TemporaryDirectory() as tmp:
            comp = Path(tmp) / "external" / "vendor"
            _write(comp / "src" / "Kconfig", "config VENDOR_OPT\n")
            out = Path(tmp) / "out"
            with (
                self.assertRaises(SystemExit),
                contextlib.redirect_stderr(io.StringIO()),
            ):
                extract.main(["component", str(comp), "--out-dir", str(out)])
            extract.main(
                ["component", str(comp), "--version", "v1", "--out-dir", str(out)]
            )
            parsed = parse_list((out / "external__vendor@v1.txt").read_text())
        self.assertEqual(parsed.header["component"], "external/vendor")
        self.assertEqual(parsed.symbols, {"VENDOR_OPT"})

    def test_idf_list_reads_root_kconfig_components_and_renames(self):
        with tempfile.TemporaryDirectory() as tmp:
            idf = Path(tmp)
            _write(idf / "Kconfig", "config IDF_TARGET\n")
            _write(
                idf / "components" / "esp_wifi" / "Kconfig", "config ESP_WIFI_ENABLED\n"
            )
            _write(
                idf / "components" / "esp_wifi" / "sdkconfig.rename",
                "CONFIG_SW_COEXIST_ENABLE CONFIG_ESP_COEX_SW_COEXIST_ENABLE\n",
            )
            parsed = parse_list(extract.idf_list(idf, "v9.9", "image@sha256:x"))
        self.assertEqual(
            parsed.symbols, {"IDF_TARGET", "ESP_WIFI_ENABLED", "SW_COEXIST_ENABLE"}
        )
        self.assertEqual(parsed.header["version"], "v9.9")


class CheckFileTests(unittest.TestCase):
    """check_file against a synthetic repo with one project."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        self.project = self.root / "packages" / "demo" / "app"
        _write(self.project / "CMakeLists.txt", "project(app)\n")
        _write(self.project / "main" / "Kconfig.projbuild", "config APP_OWN\n")
        _write(
            self.project / "main" / "idf_component.yml",
            "dependencies:\n  espressif/top: '^1.0.0'\n",
        )
        self.idf = parse_list(format_list({"version": "v9.9"}, {"IDF_ONE"}))
        self.components = {
            "espressif/top": {
                "1.0.0": _list("espressif/top", "1.0.0", {"TOP"}, "espressif/leaf")
            },
            "espressif/leaf": {"2.0.0": _list("espressif/leaf", "2.0.0", {"LEAF"})},
        }

    def tearDown(self):
        self._tmp.cleanup()

    def _check(self, defaults: str, locked=None) -> list[str]:
        path = _write(self.project / "sdkconfig.defaults", defaults)
        return check.check_file(path, self.idf, self.components, locked, root=self.root)

    def test_known_symbols_from_every_source_pass(self):
        defaults = "CONFIG_IDF_ONE=y\nCONFIG_APP_OWN=y\nCONFIG_TOP=y\n# CONFIG_LEAF is not set\n"
        self.assertEqual(self._check(defaults), [])

    def test_unknown_symbol_is_reported_with_its_line(self):
        findings = self._check("CONFIG_IDF_ONE=y\nCONFIG_MDNS_ENABLED=y\n")
        self.assertEqual(len(findings), 1)
        self.assertIn(
            "packages/demo/app/sdkconfig.defaults:2: CONFIG_MDNS_ENABLED", findings[0]
        )
        self.assertIn("espressif/leaf@2.0.0, espressif/top@1.0.0", findings[0])

    def test_unrecorded_component_is_named_not_trusted(self):
        _write(
            self.project / "components" / "x" / "idf_component.yml",
            "dependencies:\n  someone/unlisted: '*'\n",
        )
        findings = self._check("CONFIG_FROM_UNLISTED=y\n")
        self.assertEqual(len(findings), 1)
        self.assertIn("not recorded, so not checked: someone/unlisted", findings[0])
        self.assertIn("managed_components/someone__unlisted", findings[0])

    def test_highest_recorded_version_without_a_lock(self):
        self.components["espressif/leaf"]["10.0.0"] = _list(
            "espressif/leaf", "10.0.0", {"LEAF_NEW"}
        )
        self.assertEqual(self._check("CONFIG_LEAF_NEW=y\n"), [])
        self.assertEqual(len(self._check("CONFIG_LEAF=y\n")), 1)

    def test_locked_version_is_used_and_must_be_recorded(self):
        self.components["espressif/leaf"]["10.0.0"] = _list(
            "espressif/leaf", "10.0.0", {"LEAF_NEW"}
        )
        locked = {"espressif/leaf": "2.0.0"}
        self.assertEqual(self._check("CONFIG_LEAF=y\n", locked), [])
        self.assertEqual(len(self._check("CONFIG_LEAF_NEW=y\n", locked)), 1)
        findings = self._check("CONFIG_LEAF=y\n", {"espressif/leaf": "3.0.0"})
        self.assertEqual(len(findings), 1)
        self.assertIn("not recorded, so not checked: espressif/leaf@3.0.0", findings[0])

    def test_extra_component_dirs_contribute_kconfig_and_manifests(self):
        shared = self.root / "packages" / "components" / "shared"
        _write(shared / "CMakeLists.txt", "idf_component_register()\n")
        _write(shared / "Kconfig", "config SHARED_OWN\n")
        _write(shared / "idf_component.yml", "dependencies:\n  espressif/leaf: '*'\n")
        _write(
            self.project / "CMakeLists.txt",
            'set(EXTRA_COMPONENT_DIRS "../../components/shared")\nproject(app)\n',
        )
        _write(self.project / "main" / "idf_component.yml", "dependencies: {}\n")
        self.assertEqual(self._check("CONFIG_SHARED_OWN=y\nCONFIG_LEAF=y\n"), [])
        self.assertEqual(len(self._check("CONFIG_TOP=y\n")), 1)

    def test_external_checkout_resolves_through_its_recorded_list(self):
        # The checkout is cloned by a recipe, so it is absent here, as in CI.
        _write(
            self.project / "CMakeLists.txt",
            'set(EXTRA_COMPONENT_DIRS "external/vendor/src/components")\nproject(app)\n',
        )
        findings = self._check("CONFIG_VENDOR_OPT=y\n")
        self.assertEqual(len(findings), 1)
        self.assertIn("not recorded, so not checked: external/vendor", findings[0])
        self.assertIn("packages/demo/app/external/vendor <tag>", findings[0])
        self.components["external/vendor"] = {
            "v1": _list("external/vendor", "v1", {"VENDOR_OPT"})
        }
        self.assertEqual(self._check("CONFIG_VENDOR_OPT=y\n"), [])


class ExtraPathsModuleTests(unittest.TestCase):
    def test_module_is_registered_before_it_runs(self):
        # Before Python 3.12, @dataclass in check-component-extra-paths.py looks
        # its module up in sys.modules and raises AttributeError if it is not
        # there. CI runs pre-commit on 3.11, so this holds on every version.
        sys.modules.pop("check_component_extra_paths", None)
        module = check._extra_paths_module()
        self.assertIs(sys.modules.get("check_component_extra_paths"), module)


class RepoTests(unittest.TestCase):
    """The committed lists and every tracked sdkconfig.defaults."""

    def test_idf_list_matches_the_image_the_builds_use(self):
        # Tripwire for the ESP-IDF bump (#734): the committed symbol list must
        # come from the same ESP-IDF the Dockerfile and CI build with. When this
        # fails, run `just kconfig-symbols::idf` and delete the old list.
        idf = check.idf_list()
        dockerfile = (REPO / "Dockerfile").read_text(encoding="utf-8")
        from_tag = re.search(
            r"^FROM espressif/idf:(v[0-9.]+)", dockerfile, re.MULTILINE
        )
        ci = (REPO / ".github/workflows/_ci-build-esp32.yml").read_text(
            encoding="utf-8"
        )
        ci_version = re.search(r"IDF_VERSION:\s*'(v[0-9.]+)'", ci)
        self.assertEqual(idf.header["version"], from_tag.group(1))
        self.assertEqual(idf.header["version"], ci_version.group(1))

    def test_idf_list_is_a_real_extraction(self):
        # Controls in both directions, so an empty or wrong extraction cannot
        # pass every project vacuously. The two dead symbols are the ones
        # issue #707 found the v5.4 build reporting as unknown.
        idf = check.idf_list()
        self.assertGreater(len(idf.symbols), 3000)
        for present in (
            "IDF_TARGET_ESP32S3",
            "FREERTOS_HZ",
            "ESP_CONSOLE_USB_SERIAL_JTAG",
            # Defined on a `config NAME  # comment` line in v5.4.
            "CAM_CTLR_ISP_DVP_ISR_IRAM_SAFE",
            "ESP_TIME_FUNCS_USE_ESP_TIMER",
        ):
            self.assertIn(present, idf.symbols)
        for dead in ("MDNS_ENABLED", "ESP_WIFI_ESPNOW_ENABLED"):
            self.assertNotIn(dead, idf.symbols)

    def test_component_lists_are_well_formed(self):
        lists = check.component_lists()
        self.assertTrue(lists)
        for name, versions in lists.items():
            self.assertIn("/", name)
            # A component with no Kconfig records an empty list, so it is not
            # reported as unrecorded; the header still names it.
            for version, parsed in versions.items():
                self.assertTrue(version, name)
                self.assertEqual(parsed.header["component"], name)

    def test_every_tracked_sdkconfig_defaults_symbol_is_defined(self):
        count, findings = check.check_repo()
        self.assertGreater(count, 10)
        self.assertEqual(findings, [], "\n" + "\n".join(findings))


if __name__ == "__main__":
    unittest.main()
