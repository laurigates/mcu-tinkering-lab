"""Tests for check-ota-consumer-agreement.py.

    python3 -m unittest tools/test_check_ota_consumer_agreement.py

Most tests point the script at a scratch tree holding one project (a
flasher.json, main/config.h and release-please-config.json). The ones named
`real` run it on the repository, so the comparison is known to have something
to compare there.
"""

from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

_SPEC = importlib.util.spec_from_file_location(
    "check_ota_consumer_agreement",
    Path(__file__).with_name("check-ota-consumer-agreement.py"),
)
ota = importlib.util.module_from_spec(_SPEC)
assert _SPEC.loader is not None
# Registered before exec: @dataclass resolves its module through sys.modules.
sys.modules[_SPEC.name] = ota
_SPEC.loader.exec_module(ota)

PROJECT = "packages/robo/car"
PAGES = "https://example.invalid/site"
GOOD_URL = f"{PAGES}/firmware/car/manifest.json"
CONFIG_H = """\
#define OTA_MANIFEST_URL \\
    "{url}"
#define OTA_MQTT_NOTIFY_TOPIC "{topic}"
"""


class ScratchTreeTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        self.project = self.root / PROJECT
        (self.project / "main").mkdir(parents=True)
        (self.project / "flasher.json").write_text(
            json.dumps({"otaAssetName": "car.bin", "otaNotifyTopic": "car/ota/notify"})
        )
        (self.root / "release-please-config.json").write_text(
            json.dumps({"packages": {PROJECT: {"component": "car"}}})
        )
        self.config_h(GOOD_URL)
        patch = mock.patch.object(ota, "REPO_ROOT", self.root)
        patch.start()
        self.addCleanup(patch.stop)

    def config_h(self, url: str, topic: str = "car/ota/notify") -> None:
        (self.project / "main" / "config.h").write_text(
            CONFIG_H.format(url=url, topic=topic)
        )

    def run_main(self) -> tuple[int, str]:
        out = io.StringIO()
        with (
            mock.patch.object(sys, "argv", ["check-ota-consumer-agreement.py"]),
            contextlib.redirect_stdout(out),
        ):
            code = ota.main()
        return code, out.getvalue()

    def test_a_url_naming_the_project_directory_passes(self):
        code, out = self.run_main()
        self.assertEqual(code, 0, out)
        self.assertIn("PROJECTS=1", out)
        self.assertIn("STATUS=OK", out)
        # A query string is not part of the path.
        self.config_h(GOOD_URL + "?v=1")
        self.assertEqual(self.run_main()[0], 0)

    def test_a_url_naming_another_directory_fails(self):
        # robocar's directory was `unified` until the project moved; firmware
        # built with the old URL gets a 404 on every update check.
        for url in (
            f"{PAGES}/firmware/unified/manifest.json",
            f"{PAGES}/firmware/robo-car/manifest.json",
            f"{PAGES}/xfirmware/car/manifest.json",
            f"{PAGES}/firmware/car/other.json",
        ):
            with self.subTest(url=url):
                self.config_h(url)
                code, out = self.run_main()
                self.assertEqual(code, 1, out)
                self.assertIn(f"{PROJECT}: MANIFEST_URL_MISMATCH", out)

    def test_a_url_that_is_not_one_string_literal_fails(self):
        (self.project / "main" / "config.h").write_text(
            f'#define PAGES_BASE "{PAGES}"\n'
            '#define OTA_MANIFEST_URL PAGES_BASE "/firmware/car/manifest.json"\n'
        )
        code, out = self.run_main()
        self.assertEqual(code, 1, out)
        self.assertIn(f"{PROJECT}: MANIFEST_URL_UNREADABLE", out)

    def test_a_url_defined_in_a_source_file_is_read_too(self):
        (self.project / "main" / "config.h").write_text("")
        (self.project / "main" / "ota_manager.c").write_text(
            f'#define OTA_MANIFEST_URL "{PAGES}/firmware/unified/manifest.json"\n'
        )
        code, out = self.run_main()
        self.assertEqual(code, 1, out)
        self.assertIn("main/ota_manager.c polls", out)

    def test_the_notify_topic_must_match_flasher_json_and_the_component(self):
        self.config_h(GOOD_URL, topic="car-unified/ota/notify")
        code, out = self.run_main()
        self.assertEqual(code, 1, out)
        self.assertIn(f"{PROJECT}: NOTIFY_TOPIC_MISMATCH", out)
        self.config_h(GOOD_URL)
        (self.root / "release-please-config.json").write_text(
            json.dumps({"packages": {PROJECT: {"component": "boat"}}})
        )
        code, out = self.run_main()
        self.assertEqual(code, 1, out)
        self.assertIn(f"{PROJECT}: NOTIFY_TOPIC_CONVENTION_MISMATCH", out)

    def test_a_skipped_project_is_not_read(self):
        # Control for test_no_project_is_skipped: this is what an entry does.
        self.config_h(f"{PAGES}/firmware/unified/manifest.json")
        with mock.patch.object(ota, "SKIP_PROJECTS", {PROJECT}):
            self.assertEqual(self.run_main()[0], 0)
        self.assertEqual(self.run_main()[0], 1)


class RealTreeTest(unittest.TestCase):
    def test_no_project_is_skipped(self):
        # An entry switches off every comparison for its project. The last
        # one outlived its reason with nothing failing.
        self.assertEqual(ota.SKIP_PROJECTS, set())

    def test_the_real_tree_passes(self):
        out = io.StringIO()
        with (
            mock.patch.object(sys, "argv", ["check-ota-consumer-agreement.py"]),
            contextlib.redirect_stdout(out),
        ):
            code = ota.main()
        self.assertEqual(code, 0, out.getvalue())

    def test_the_real_robocar_url_is_one_the_check_reads(self):
        # The comparison must have a URL to compare: if the macro moved to a
        # file or a form the check does not read, it would pass on nothing.
        project = ota.REPO_ROOT / "packages" / "robotics" / "robocar"
        urls = [
            url
            for src in sorted(project.glob("main/*.[ch]"))
            for url in ota.MANIFEST_URL_RE.findall(src.read_text(encoding="utf-8"))
        ]
        self.assertEqual(len(urls), 1, urls)
        self.assertTrue(urls[0].endswith("/firmware/robocar/manifest.json"), urls[0])


if __name__ == "__main__":
    unittest.main()
