"""The continuity checklist: what must beep, per build step, and which rails must not.

Built from the nets, not from the wires, so a wire the plan got wrong still
fails the check: each net names one probe pad and every other pad on it. The
steps are the wire list's, so a step can be checked as soon as it is soldered.
"""

from __future__ import annotations

from itertools import combinations

from .config import PerfboardConfig
from .grid import hole_name
from .wirelist import BANNER


def pad(parts, node):
    ref, pin = node
    return f"{ref}.{pin} ({hole_name(*parts[ref].pads[pin])})"


def checklist(parts, nets, cfg: PerfboardConfig):
    lines = [
        f"# {cfg.title} continuity checklist",
        "",
        BANNER,
        "",
        "Board unpowered, modules out of their sockets, meter on continuity. "
        "Pads are named as in `perfboard.md`, from the component side.",
        "After each step of the wire list, hold one probe on the net's first pad "
        "and touch every pad after it: each must beep.",
    ]
    for step, st in enumerate(cfg.steps):
        lines += [
            "",
            f"## Step {step + 1}: {st.title}",
            "",
            "| ✓ | Net | Probe on | Must beep to |",
            "|---|---|---|---|",
        ]
        for net in st.nets:
            first, *rest = nets[net]
            targets = ", ".join(pad(parts, n) for n in rest)
            lines.append(f"| ☐ | {net} | {pad(parts, first)} | {targets} |")
    if cfg.isolated:
        lines += [
            "",
            "## Before first power-up",
            "",
            "Each pair must **not** beep. Then read it on the resistance range: "
            "a reading that starts low and climbs is a capacitor charging, kilohms "
            "is a resistor between the rails, and one that stays near 0 Ω is a short. "
            "Fit the modules only when every pair passes.",
            "",
            "| ✓ | Rail | Rail | Probe on | And on |",
            "|---|---|---|---|---|",
        ]
        for a, b in combinations(cfg.isolated, 2):
            lines.append(
                f"| ☐ | {a} | {b} | {pad(parts, nets[a][0])} | {pad(parts, nets[b][0])} |"
            )
    return "\n".join(lines) + "\n"
