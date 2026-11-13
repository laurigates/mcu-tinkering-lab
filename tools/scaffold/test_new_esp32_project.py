"""Host tests for tools/scaffold/new-esp32-project.sh (issue #1078, item 4).

The script copies an existing project into packages/<domain>/<name>. Each test
runs it, with its prompts answered on stdin, in a throwaway tree that holds a
copy of the script and of the template project, never in the repository, since
the script writes under packages/.

    python3 -m unittest discover -s tools/scaffold
"""

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SCRIPT = REPO / "tools" / "scaffold" / "new-esp32-project.sh"
TEMPLATE = REPO / "packages" / "camera-vision" / "cam-webserver"

# Files that carry the source project's release identity or resolved state.
RELEASE_FILES = ("flasher.json", "CHANGELOG.md", "CLAUDE.md", "dependencies.lock")
# Answers to the prompts: domain 7 is sensors.
SENSORS = "7"


def make_tree(root: Path) -> None:
    """Copy the script and the template project into a throwaway tree."""
    (root / "tools" / "scaffold").mkdir(parents=True)
    shutil.copy2(SCRIPT, root / "tools" / "scaffold" / SCRIPT.name)
    # Ignore only build output and generated config, so sdkconfig.defaults
    # comes along as it does from the real template.
    shutil.copytree(
        TEMPLATE,
        root / "packages" / "camera-vision" / "cam-webserver",
        ignore=shutil.ignore_patterns(
            "build", "managed_components", "sdkconfig", "sdkconfig.old"
        ),
    )


def run_scaffold(root: Path, answers: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["bash", str(root / "tools" / "scaffold" / SCRIPT.name)],
        input="\n".join(answers) + "\n",
        capture_output=True,
        text=True,
        cwd=root,
        timeout=60,
    )


class TemplateControl(unittest.TestCase):
    """The template still holds what the scaffold must not pass on.

    Without this, the absence checks below would pass on a template that no
    longer has the files at all.
    """

    def test_template_holds_release_identity(self):
        for name in RELEASE_FILES:
            self.assertTrue((TEMPLATE / name).is_file(), name)
        self.assertTrue((TEMPLATE / ".gitignore").is_file())
        self.assertNotEqual((TEMPLATE / "version.txt").read_text().strip(), "0.1.0")
        self.assertIn(
            'mdns_hostname_set("esp32-cam")', (TEMPLATE / "main" / "main.c").read_text()
        )


class DefaultTemplate(unittest.TestCase):
    """Project type 1: a copy of cam-webserver."""

    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.root = Path(cls._tmp.name)
        make_tree(cls.root)
        template = cls.root / "packages" / "camera-vision" / "cam-webserver"
        # Local state a developer's checkout of the template can hold.
        (template / "main" / "credentials.h").write_text('#define WIFI_PASS "secret"\n')
        (template / "build").mkdir()
        (template / "build" / "esp32-cam-webserver.bin").write_bytes(b"\0")
        (template / "sdkconfig").write_text("CONFIG_FROM_TEMPLATE=y\n")
        (template / "sdkconfig.old").write_text("CONFIG_FROM_TEMPLATE=y\n")
        cls.result = run_scaffold(cls.root, ["probe-proj", SENSORS, "1"])
        cls.project = cls.root / "packages" / "sensors" / "probe-proj"

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def test_script_succeeds(self):
        self.assertEqual(
            self.result.returncode, 0, self.result.stdout + self.result.stderr
        )
        self.assertTrue((self.project / "CMakeLists.txt").is_file())

    def test_release_identity_not_copied(self):
        for name in RELEASE_FILES:
            self.assertFalse((self.project / name).exists(), name)

    def test_version_starts_at_0_1_0(self):
        self.assertEqual((self.project / "version.txt").read_text(), "0.1.0\n")

    def test_dotfiles_copied(self):
        self.assertEqual(
            (self.project / ".gitignore").read_text(),
            (TEMPLATE / ".gitignore").read_text(),
        )

    def test_local_state_not_copied(self):
        self.assertFalse((self.project / "main" / "credentials.h").exists())
        self.assertFalse((self.project / "build").exists())
        self.assertFalse((self.project / "sdkconfig").exists())
        self.assertFalse((self.project / "sdkconfig.old").exists())
        self.assertTrue((self.project / "main" / "credentials.h.example").is_file())
        self.assertEqual(
            (self.project / "sdkconfig.defaults").read_text(),
            (TEMPLATE / "sdkconfig.defaults").read_text(),
        )

    def test_mdns_hostname_is_project_name(self):
        main_c = (self.project / "main" / "main.c").read_text()
        self.assertIn('mdns_hostname_set("probe-proj")', main_c)
        self.assertIn("probe-proj.local", main_c)
        self.assertNotIn("esp32-cam.local", main_c)
        self.assertNotIn('mdns_hostname_set("esp32-cam")', main_c)

    def test_closing_advice_names_release_setup(self):
        out = self.result.stdout
        self.assertIn("release-please-config.json", out)
        self.assertIn("flasher.json", out)


class CopyFromExisting(unittest.TestCase):
    """Project type 4: the same clean-up keyed on file names, for any source."""

    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.root = Path(cls._tmp.name)
        make_tree(cls.root)
        source = cls.root / "packages" / "games" / "src-proj"
        (source / "main").mkdir(parents=True)
        for name in RELEASE_FILES:
            (source / name).write_text("from src-proj\n")
        (source / "version.txt").write_text("1.2.3\n")
        (source / ".gitignore").write_text("build/\n")
        (source / "CMakeLists.txt").write_text("project(src-proj)\n")
        (source / "main" / "main.c").write_text(
            '// mDNS: src-host.local\nmdns_hostname_set("src-host");\n'
        )
        cls.result = run_scaffold(
            cls.root, ["copy-proj", SENSORS, "4", "games/src-proj"]
        )
        cls.project = cls.root / "packages" / "sensors" / "copy-proj"

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def test_script_succeeds(self):
        self.assertEqual(
            self.result.returncode, 0, self.result.stdout + self.result.stderr
        )

    def test_release_identity_not_copied(self):
        for name in RELEASE_FILES:
            self.assertFalse((self.project / name).exists(), name)
        self.assertEqual((self.project / "version.txt").read_text(), "0.1.0\n")
        self.assertTrue((self.project / ".gitignore").is_file())

    def test_mdns_hostname_is_project_name(self):
        main_c = (self.project / "main" / "main.c").read_text()
        self.assertEqual(
            main_c, '// mDNS: copy-proj.local\nmdns_hostname_set("copy-proj");\n'
        )


if __name__ == "__main__":
    unittest.main()
