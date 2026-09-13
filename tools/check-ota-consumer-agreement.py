#!/usr/bin/env python3
"""Assert that OTA firmware and flasher.json agree on asset names and topics.

`build-firmware.yml` attaches each project's release asset under its
`flasher.json` `otaAssetName`, and (per project) publishes an MQTT wake-up to
its `otaNotifyTopic`. Neither value is read from the firmware at build time --
they are two independent strings, one in JSON, one compiled into C, and
nothing keeps them in sync. Issue #540 is exactly this class of bug: the
consumers were written against names the release pipeline never published,
and it was never caught because no build or CI step compared the two sides.

Two shapes of consumer, both covered here:

  TRIGGERED mode (`cfg.triggered_asset_filename = "X";`) builds a direct
  download URL from the literal, so X must equal otaAssetName exactly -- there
  is no wildcard to paper over a mismatch.

  PULL mode (`#define OTA_FIRMWARE_FILENAME_MATCH "X"`) feeds X to
  fnmatch(X, release_asset_name, 0) -- a FULL-STRING match, not a substring
  match, despite older docs in this component describing it as one. X must
  therefore be a pattern that matches otaAssetName, not equal it.

  MQTT (`#define OTA_MQTT_NOTIFY_TOPIC "X"`) must equal otaNotifyTopic
  exactly -- build-firmware.yml resolves which project to wake by scanning
  every flasher.json for a matching otaNotifyTopic, so a device subscribed to
  a topic no flasher.json advertises will never hear the release that was
  meant for it.

A fourth check, independent of the three above: `otaNotifyTopic` itself must
follow `<release-please component>/ota/notify`, where the component comes
from `release-please-config.json` -- that is the string build-firmware.yml's
notify step parses out of the fired release tag (`<component>-v<semver>`) to
find which flasher.json to publish to. Getting the macro and flasher.json to
agree with EACH OTHER (the three checks above) is not sufficient if both
agree on the wrong string: matching typos pass the macro-vs-flasher.json
checks and still mean the notify step can never find this project. A project
absent from release-please-config.json cannot be checked against the
convention and is reported as advisory rather than failed.

Run:  python3 tools/check-ota-consumer-agreement.py [--verbose]
Exit: 0 clean (including advisory-only), 1 on any blocking mismatch.
"""

from __future__ import annotations

import argparse
import fnmatch
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# On main, robocar-unified's config.h still carries the pre-#409 shared
# filename pattern and notify topic. PR #559 (the #539 rewrite that drops
# esp_ghota for a Pages-manifest poll) DOES still read OTA_MQTT_NOTIFY_TOPIC
# -- it becomes the check-now nudge that wakes the poll task early -- and
# changes it to "robocar-unified/ota/notify", matching flasher.json. So this
# skip is only needed until #559 merges, not because #559 stops reading the
# macro. #559 depends on this PR (#558) for flasher.json/build-firmware.yml
# to actually publish to that topic, so land #558 before #559.
# Remove this entry once #559 lands and re-run with --verbose to confirm it
# reports 0 findings on its own.
SKIP_PROJECTS = {"packages/robocar/unified"}

TRIGGERED_ASSET_RE = re.compile(r'triggered_asset_filename\s*=\s*"([^"]+)"')
FILENAME_MATCH_RE = re.compile(r'#define\s+OTA_FIRMWARE_FILENAME_MATCH\s+"([^"]+)"')
NOTIFY_TOPIC_RE = re.compile(r'#define\s+OTA_MQTT_NOTIFY_TOPIC\s+"([^"]+)"')


@dataclass
class Finding:
    project: str
    code: str
    detail: str
    advisory: bool = False


def _read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return ""


def _load_flasher(path: Path) -> dict:
    try:
        return json.loads(_read(path) or "{}")
    except json.JSONDecodeError as exc:
        return {"__error__": str(exc)}


def discover(root: Path) -> list[Path]:
    return sorted(root.glob("packages/*/*/flasher.json"))


def load_release_please_components(root: Path) -> dict[str, str]:
    """Map each release-please package path to its `component`.

    Read fresh from release-please-config.json every run -- this must never
    be a hand-written list, since that is exactly the kind of copy the two
    sides of this checker are supposed to catch drifting apart.
    """
    cfg = _load_flasher(root / "release-please-config.json")
    return {
        pkg_path: info.get("component", "")
        for pkg_path, info in cfg.get("packages", {}).items()
        if isinstance(info, dict) and info.get("component")
    }


def check_notify_topic_convention(
    flasher_json: Path, components: dict[str, str]
) -> list[Finding]:
    """otaNotifyTopic must be `<release-please component>/ota/notify`.

    This is independent of check_project's macro-vs-flasher.json checks:
    those catch the macro and flasher.json disagreeing with EACH OTHER, not
    both agreeing on a string build-firmware.yml's tag-parsing notify step
    can never produce.
    """
    project_dir = flasher_json.parent
    name = project_dir.relative_to(REPO_ROOT).as_posix()
    cfg = _load_flasher(flasher_json)
    if "__error__" in cfg:
        return []  # already reported as INVALID_JSON by check_project

    ota_topic = cfg.get("otaNotifyTopic")
    if ota_topic is None:
        return []  # a missing otaNotifyTopic is reported by check_project

    component = components.get(name)
    if component is None:
        return [
            Finding(
                name,
                "NO_RELEASE_PLEASE_COMPONENT",
                f"{flasher_json.name} has otaNotifyTopic '{ota_topic}' but "
                f"'{name}' has no entry in release-please-config.json, so the "
                "<component>/ota/notify convention can't be checked for it -- "
                "build-firmware.yml's notify step parses the component out of "
                "the fired release tag, and a project release-please never "
                "tags can never trigger that step at all",
                advisory=True,
            )
        ]

    expected = f"{component}/ota/notify"
    if ota_topic != expected:
        return [
            Finding(
                name,
                "NOTIFY_TOPIC_CONVENTION_MISMATCH",
                f"{flasher_json.name} otaNotifyTopic is '{ota_topic}' but the "
                f"release-please component for '{name}' is '{component}', so "
                f"the expected topic is '{expected}' -- build-firmware.yml "
                "parses the component out of the fired release tag and looks "
                "up the flasher.json whose otaNotifyTopic matches it exactly; "
                "a topic that agrees with a compiled-in macro but not with "
                "this convention will still never be found",
            )
        ]
    return []


def check_project(flasher_json: Path) -> list[Finding]:
    project_dir = flasher_json.parent
    name = project_dir.relative_to(REPO_ROOT).as_posix()
    if name in SKIP_PROJECTS:
        return []
    cfg = _load_flasher(flasher_json)
    findings: list[Finding] = []

    if "__error__" in cfg:
        return [
            Finding(name, "INVALID_JSON", f"{flasher_json.name}: {cfg['__error__']}")
        ]

    ota_asset = cfg.get("otaAssetName")
    ota_topic = cfg.get("otaNotifyTopic")
    rel_flasher = flasher_json.relative_to(REPO_ROOT).as_posix()

    for src in sorted(project_dir.glob("main/*.c")):
        rel_src = src.relative_to(REPO_ROOT).as_posix()
        for m in TRIGGERED_ASSET_RE.finditer(_read(src)):
            compiled = m.group(1)
            if ota_asset is None:
                findings.append(
                    Finding(
                        name,
                        "MISSING_OTA_ASSET_NAME",
                        f"{rel_src} downloads '{compiled}' by exact URL, but "
                        f"{rel_flasher} has no otaAssetName",
                    )
                )
            elif compiled != ota_asset:
                findings.append(
                    Finding(
                        name,
                        "ASSET_NAME_MISMATCH",
                        f"{rel_src} downloads '{compiled}' but {rel_flasher} "
                        f"otaAssetName is '{ota_asset}' -- triggered_asset_filename "
                        "builds a direct URL, so this 404s at fetch time",
                    )
                )

    for hdr in sorted(project_dir.glob("main/*.h")):
        rel_hdr = hdr.relative_to(REPO_ROOT).as_posix()
        text = _read(hdr)

        for m in FILENAME_MATCH_RE.finditer(text):
            pattern = m.group(1)
            if ota_asset is None:
                findings.append(
                    Finding(
                        name,
                        "MISSING_OTA_ASSET_NAME",
                        f"{rel_hdr} matches release assets against '{pattern}', "
                        f"but {rel_flasher} has no otaAssetName",
                    )
                )
            elif not fnmatch.fnmatch(ota_asset, pattern):
                findings.append(
                    Finding(
                        name,
                        "ASSET_PATTERN_MISMATCH",
                        f"{rel_hdr} pattern '{pattern}' does not match "
                        f"{rel_flasher} otaAssetName '{ota_asset}' "
                        "(fnmatch(pattern, name, 0) is a full-string match, not "
                        "a substring match)",
                    )
                )

        for m in NOTIFY_TOPIC_RE.finditer(text):
            compiled_topic = m.group(1)
            if ota_topic is None:
                findings.append(
                    Finding(
                        name,
                        "MISSING_OTA_NOTIFY_TOPIC",
                        f"{rel_hdr} subscribes to '{compiled_topic}', but "
                        f"{rel_flasher} has no otaNotifyTopic",
                    )
                )
            elif compiled_topic != ota_topic:
                findings.append(
                    Finding(
                        name,
                        "NOTIFY_TOPIC_MISMATCH",
                        f"{rel_hdr} subscribes to '{compiled_topic}' but "
                        f"{rel_flasher} otaNotifyTopic is '{ota_topic}' -- "
                        "build-firmware.yml publishes to the flasher.json topic, "
                        "so this device would never hear its own release",
                    )
                )

    return findings


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    projects = discover(REPO_ROOT)
    components = load_release_please_components(REPO_ROOT)
    findings: list[Finding] = []

    print("=== PROJECTS ===")
    for flasher_json in projects:
        rel = flasher_json.parent.relative_to(REPO_ROOT).as_posix()
        found = check_project(flasher_json)
        found = found + check_notify_topic_convention(flasher_json, components)
        findings.extend(found)
        if args.verbose or found:
            print(f"  {rel}: {len(found)} finding(s)")
    print(f"PROJECTS={len(projects)}")

    advisory = [f for f in findings if f.advisory]
    blocking = [f for f in findings if not f.advisory]

    if advisory:
        print("=== ADVISORY ===")
        for f in advisory:
            print(f"  {f.project}: {f.code}: {f.detail}")

    if blocking:
        print("=== FINDINGS ===")
        for f in blocking:
            print(f"  {f.project}: {f.code}: {f.detail}")
        print(f"FINDINGS={len(blocking)}")
        print(f"ADVISORY={len(advisory)}")
        print("STATUS=FAIL")
        return 1

    print("FINDINGS=0")
    print(f"ADVISORY={len(advisory)}")
    print("STATUS=OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
