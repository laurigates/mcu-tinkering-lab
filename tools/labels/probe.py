"""Ask the cutter one question without cutting anything.

    <venv python> tools/labels/probe.py <driver src dir> marks <plan.json> [top mm] [left mm]
    <venv python> tools/labels/probe.py <driver src dir> depth <0-10>

marks  runs only the registration-mark search a print-and-cut job starts with,
       from the start position cut.sh uses (20 mm in) or the one given, and
       prints the cutter's reply. The media moves under the sensor; the blade
       never comes down. Use it to find where a sheet on the mat is found
       before spending a sticker on a test cut.
depth  sets the AutoBlade depth (J1, then TF<depth>,1) and waits for the
       cutter to finish adjusting. Read the number in the blade's window
       afterwards: it is the only confirmation the cutter gives.

The connection comes from the same environment cut.sh reads: LABEL_CUTTER
(usb, ble or bluetooth), LABEL_CUTTER_NAME, LABEL_CUTTER_ID, LABEL_CUTTER_ADDR.
Both probes were written and used over Bluetooth LE; USB is untested.
"""

import json
import os
import sys
import time

MODEL = "Silhouette_Portrait2"
DEFAULT_TOP_MM = 20.0
# The driver's own mark setup (Graphtec.plot): orientation, Cameo/Portrait mark
# type, 20 mm arms, 0.5 mm strokes.
MARK_SETUP = ["FN0", "TB50,0", "TB99", "TB52,2", "TB51,400", "TB53,10", "TB55,1"]
REPLIES = {
    b"    0\x03": "found",
    b"    1\x03": "not found: no marks where it looked (no sheet, or start the search further in)",
    b"   -1\x03": "not found: the search started too far in; unload and load the mat",
}


def units(mm: float) -> int:
    """Millimetres to the cutter's unit, 1/20 mm."""
    return round(mm * 20)


def mark_search_command(regmarks: dict, top: float, left: float) -> str:
    """The automatic mark search, started top/left mm from the machine origin."""
    return f"TB123,{units(regmarks['l'])},{units(regmarks['w'])},{units(top)},{units(left)}"


def depth_commands(depth: int) -> list[str]:
    """Tool-select, then the depth: TF alone lands on the wrong depth on a Portrait 2."""
    if not 0 <= depth <= 10:
        raise ValueError(f"depth {depth} is outside 0-10")
    return ["J1", f"TF{depth},1"]


def connection(env: dict) -> dict:
    """SilhouetteCameo keyword arguments for the connection LABEL_CUTTER names."""
    kind = env.get("LABEL_CUTTER", "usb")
    if kind == "usb":
        return {}
    if kind == "ble":
        if env.get("LABEL_CUTTER_ID"):
            return {
                "bluetooth_le": True,
                "bluetooth_identifier": env["LABEL_CUTTER_ID"],
            }
        return {
            "bluetooth_le": True,
            "bluetooth_name": env.get("LABEL_CUTTER_NAME", "PORTRAIT2"),
        }
    if kind == "bluetooth":
        if not env.get("LABEL_CUTTER_ADDR"):
            raise ValueError("LABEL_CUTTER=bluetooth needs LABEL_CUTTER_ADDR")
        return {"bluetooth_addr": env["LABEL_CUTTER_ADDR"]}
    raise ValueError(f"LABEL_CUTTER is '{kind}'; use usb, ble or bluetooth")


def open_cutter(driver_dir: str):
    sys.path.insert(0, driver_dir)
    from silhouette.Graphtec import SilhouetteCameo

    kwargs = connection(dict(os.environ))
    # Over Bluetooth the driver does not recognise the Portrait 2's firmware name.
    force = MODEL if kwargs else None
    return SilhouetteCameo(log=sys.stderr, force_hardware=force, **kwargs)


def probe_marks(dev, plan_path: str, top: float, left: float) -> int:
    with open(plan_path) as f:
        regmarks = json.load(f)["regmarks"]
    command = mark_search_command(regmarks, top, left)
    dev.send_command(MARK_SETUP)
    start = time.monotonic()
    dev.send_command(command)
    reply = dev.read(timeout=60000)
    meaning = REPLIES.get(reply, "unknown reply")
    print(f"{command}: {reply!r} after {time.monotonic() - start:.0f} s: {meaning}")
    return 0 if reply == b"    0\x03" else 1


def set_depth(dev, depth: int) -> int:
    dev.send_command(depth_commands(depth))
    start = time.monotonic()
    ready = 0
    while ready < 2 and time.monotonic() - start < 60:
        time.sleep(1)
        ready = ready + 1 if dev.status() == "ready" else 0
    print(
        f"depth {depth} sent; the cutter adjusted for {time.monotonic() - start:.0f} s."
    )
    print("Read the number in the AutoBlade's window: it should show", depth)
    return 0


def main(argv: list[str]) -> int:
    if len(argv) < 4 or argv[2] not in ("marks", "depth"):
        print(__doc__, file=sys.stderr)
        return 2
    dev = open_cutter(argv[1])
    try:
        if argv[2] == "marks":
            top = float(argv[4]) if len(argv) > 4 else DEFAULT_TOP_MM
            left = float(argv[5]) if len(argv) > 5 else 0.0
            return probe_marks(dev, argv[3], top, left)
        return set_depth(dev, int(argv[3]))
    finally:
        if dev.transport is not None:
            dev.transport.close()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
