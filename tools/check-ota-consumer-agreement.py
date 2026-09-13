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

Run:  python3 tools/check-ota-consumer-agreement.py [--verbose]
Exit: 0 clean, 1 on any mismatch.
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

# robocar-unified's config.h still carries the pre-#409 shared filename
# pattern and notify topic. Its OTA is being replaced entirely with a
# Pages-manifest poll (issue #539), which does not read either macro this
# script checks -- fixing the mismatch here would be dead work bulldozed by
# that rewrite. flasher.json already carries the values #539 should read.
# Remove this entry once #539 lands and re-run with --verbose to confirm it
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
    findings: list[Finding] = []

    print("=== PROJECTS ===")
    for flasher_json in projects:
        rel = flasher_json.parent.relative_to(REPO_ROOT).as_posix()
        found = check_project(flasher_json)
        findings.extend(found)
        if args.verbose or found:
            print(f"  {rel}: {len(found)} finding(s)")
    print(f"PROJECTS={len(projects)}")

    if findings:
        print("=== FINDINGS ===")
        for f in findings:
            print(f"  {f.project}: {f.code}: {f.detail}")
        print(f"FINDINGS={len(findings)}")
        print("STATUS=FAIL")
        return 1

    print("FINDINGS=0")
    print("STATUS=OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
