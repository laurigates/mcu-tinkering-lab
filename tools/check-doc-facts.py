#!/usr/bin/env python3
"""Assert that hand-written docs still agree with the firmware they describe.

Two CI guards already keep the GENERATED documentation honest: build-guide-check
recompiles `docs/build-guide.pdf` from its `.typ` plus `docs/auto/pin_defs.typ`
(regenerated from `main/pin_config.h`), and schematics-check re-renders every
SVG from its circuit source. Both diff the result. Both work.

Neither looks at a sentence a human typed. In 2026-08 all three artifacts were
byte-identical to their generators while the prose around them said the camera
was an OV2640 (it reports PID 0x3660), that WiFi provisioning used Bluetooth
(it is Improv Serial over USB), that the planner ran at ~1 Hz (15 s, and dormant
by default since ADR-022), and that TCA9548A ch2 was reserved (MCP23017, per the
same file's own prose). The OV2640 correction had landed in camera_pins.h in
#446; four prose copies never got it.

So this script covers the other side of that boundary, with two checks:

  INVENTORY   every GPIO in main/pin_config.h is mentioned in WIRING.md, and
              every named TCA9548A channel names its device there. Catches
              OMISSIONS -- a pin nobody documented -- which no diff-based guard
              can, because the drift is a thing that is not there. The onboard
              PDM microphone went undocumented this way for two releases.

  FACTS       a small table of "the docs must not claim X while the source
              plainly does Y". Each rule is CONDITIONAL on a probe against the
              firmware, so it fires only where the contradiction is real: the
              BLE rule is silent in a project that genuinely uses BLE, and
              speaks only where the firmware is running Improv Serial.

The FACTS table is a ratchet, not a proof. It cannot know which claims matter --
somebody has to notice that a fact is load-bearing and add a row. The intended
moment is the same commit that writes the finding into `.claude/rules/`: the
OV3660 discovery produced `camera-sensor-identity.md` and touched no doc, and a
row here would have cost one line.

Run:  python3 tools/check-doc-facts.py [--verbose]
Exit: 0 clean, 1 on any contradiction or undocumented pin.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# Docs written for somebody BUILDING the hardware. CLAUDE.md is deliberately
# excluded: it contrasts OV2640 against OV3660 at length on purpose, and an
# allowlist long enough to permit that would permit the original bug too.
DOC_GLOBS = ("README.md", "WIRING.md", "docs/build-guide.typ")


@dataclass(frozen=True)
class FactRule:
    name: str
    # The rule applies only where this matches the project's firmware, so a
    # project that legitimately does the other thing is never flagged.
    probe_glob: str
    probe: str
    # Claim shapes that are then wrong. Deliberately specific: a broad
    # `BLE|Bluetooth` would flag robocar-unified's README, which says "not the
    # BLE variant" precisely to stop this confusion. A check that cries wolf
    # gets disabled, and then it protects nothing.
    forbid: tuple[str, ...]
    why: str


FACT_RULES = (
    FactRule(
        name="improv-is-serial-not-ble",
        probe_glob="main/*.c",
        probe=r"improv_wifi_process_byte",
        forbid=(
            r"Improv\s+\w*\s*BLE",
            r"(?:WiFi|provisioning)\s+over\s+Bluetooth",
            r"advertises\s+an?\s+\*?Improv",
        ),
        why=(
            "firmware feeds console bytes to improv_wifi_process_byte(), i.e. "
            "Improv SERIAL over USB. No BLE service is ever advertised, so a "
            "builder following this doc waits for a device that never appears."
        ),
    ),
    FactRule(
        name="camera-sensor-identity",
        probe_glob="main/*.[ch]",
        probe=r"OV3660_PID|0x3660",
        forbid=(r"OV2640",),
        why=(
            "this board reports PID=0x3660. The two sensors take incompatible "
            "register maps and an incompatible set_gainceiling() argument, so "
            "naming the wrong one sends the next reader to the wrong datasheet "
            "(see .claude/rules/camera-sensor-identity.md)."
        ),
    ),
)


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


def _strip_comments(text: str) -> str:
    """Drop // and /* */ so a rationale in a comment is not read as a claim."""
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", " ", text)


def discover(root: Path) -> list[Path]:
    """Projects carrying both a pin header and a wiring doc."""
    return sorted(
        p.parent.parent
        for p in root.glob("packages/*/*/main/pin_config.h")
        if (p.parent.parent / "WIRING.md").is_file()
    )


def check_inventory(proj: Path, wiring: str) -> list[Finding]:
    name = proj.relative_to(REPO_ROOT).as_posix()
    header = _read(proj / "main" / "pin_config.h")
    findings: list[Finding] = []

    for macro, num in re.findall(r"^\s*#define\s+(\w+)\s+GPIO_NUM_(\d+)", header, re.M):
        # A builder looks up the number, so that is what must be present.
        # \b keeps GPIO4 from being satisfied by GPIO41.
        if not re.search(rf"GPIO_?(?:NUM_)?{num}\b", wiring):
            findings.append(
                Finding(
                    name, "UNDOCUMENTED_PIN", f"{macro} = GPIO{num} is in no WIRING.md"
                )
            )

    for dev, ch in re.findall(
        r"^\s*#define\s+I2C_BUS_CHANNEL_(\w+)\s+(\d+)", header, re.M
    ):
        if not re.search(rf"\b{re.escape(dev)}\b", wiring, re.I):
            findings.append(
                Finding(
                    name,
                    "UNDOCUMENTED_CHANNEL",
                    f"TCA9548A ch{ch} carries {dev}, which WIRING.md never names",
                )
            )
    return findings


def check_facts(proj: Path) -> list[Finding]:
    name = proj.relative_to(REPO_ROOT).as_posix()
    findings: list[Finding] = []

    for rule in FACT_RULES:
        applies = any(
            re.search(rule.probe, _strip_comments(_read(f)))
            for f in sorted(proj.glob(rule.probe_glob))
        )
        if not applies:
            continue
        for doc_glob in DOC_GLOBS:
            doc = proj / doc_glob
            if not doc.is_file():
                continue
            for line_no, line in enumerate(_read(doc).splitlines(), 1):
                for pattern in rule.forbid:
                    if re.search(pattern, line):
                        findings.append(
                            Finding(
                                name,
                                rule.name,
                                f"{doc_glob}:{line_no}: {line.strip()[:90]}",
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
    for proj in projects:
        rel = proj.relative_to(REPO_ROOT).as_posix()
        wiring = _read(proj / "WIRING.md")
        found = check_inventory(proj, wiring) + check_facts(proj)
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
        print()
        for rule in FACT_RULES:
            if any(f.code == rule.name for f in findings):
                print(f"{rule.name}: {rule.why}")
        if any(f.code.startswith("UNDOCUMENTED") for f in findings):
            print(
                "UNDOCUMENTED_*: main/pin_config.h is the source of truth and "
                "WIRING.md mirrors it for humans.\nAdd the pin or channel there "
                "-- an on-module part still gets a row saying nothing is wired."
            )
        return 1

    print("FINDINGS=0")
    print("STATUS=OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
