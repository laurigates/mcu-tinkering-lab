"""Check the carrier schematic against the firmware's pin map.

Every role in src/pin_config.h that reaches the XIAO header must land, on the
carrier, on the net this board gives that role. The pad a role lands on comes
from the hardware join (tools/hardware, ADR-021): pin_config.h's GPIO, looked up
in docs/reference/boards/xiao-rp2350.md. The schematic side comes from KiCad's
own netlist export, which names each XIAO pin by function ("D4_5").

    just balancebot::carrier-check

Exits non-zero naming every role that disagrees, so renumbering a pin in
pin_config.h without rewiring the carrier (or the reverse) fails here.
"""

import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[2]  # packages/robotics/balancebot
REPO = PROJECT.parents[2]
SCH = HERE.parent / "balancebot-carrier" / "balancebot-carrier.kicad_sch"
sys.path.insert(0, str(REPO / "tools"))

from hardware.model import join  # noqa: E402

MCU_REF = "U1"
# pin_config.h role -> the carrier net that carries it.
ROLE_NETS = {
    "PIN_IMU_SDA": "IMU_SDA",
    "PIN_IMU_SCL": "IMU_SCL",
    "PIN_IMU_INT": "IMU_INT",
    "PIN_STEP_LEFT": "STEP_L",
    "PIN_STEP_RIGHT": "STEP_R",
    "PIN_DIR_LEFT": "DIR_L",
    "PIN_DIR_RIGHT": "DIR_R",
    "PIN_MOTOR_NENABLE": "NENABLE",
    "PIN_PACK_SENSE": "PACK_SENSE",
}


def mcu_pad_nets():
    """XIAO pad name (D0..D10, 3V3_OUT, ...) -> net name, from the schematic."""
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "carrier.xml"
        subprocess.run(
            [
                "kicad-cli",
                "sch",
                "export",
                "netlist",
                "--format",
                "kicadxml",
                "-o",
                str(out),
                str(SCH),
            ],
            check=True,
            capture_output=True,
        )
        root = ET.parse(out).getroot()
    pads = {}
    for net in root.iter("net"):
        for node in net.iter("node"):
            if node.get("ref") == MCU_REF:
                pads[node.get("pinfunction").rsplit("_", 1)[0]] = net.get(
                    "name"
                ).lstrip("/")
    return pads


def main():
    model = join(PROJECT)
    pads = mcu_pad_nets()
    broken_out = {role for role in model.roles if model.pin_for(role) is not None}
    errors = []
    for role in sorted(broken_out - set(ROLE_NETS)):
        errors.append(
            f"{role} reaches the header but ROLE_NETS has no carrier net for it"
        )
    for role, want in sorted(ROLE_NETS.items()):
        if role not in model.roles:
            errors.append(f"{role} is not in pin_config.h any more")
            continue
        pin = model.pin_for(role)
        if pin is None:
            errors.append(
                f"{role} (GPIO{model.roles[role]}) no longer reaches the header"
            )
            continue
        got = pads.get(pin.name)
        if got != want:
            errors.append(
                f"{role}: GPIO{pin.gpio} is {pin.name}, which the carrier wires to {got!r}, not {want!r}"
            )
        else:
            print(f"ok  {role:18s} GPIO{pin.gpio:<3d} {pin.name:4s} -> {got}")
    if errors:
        print("\n".join("FAIL " + e for e in errors))
        sys.exit(1)


if __name__ == "__main__":
    main()
