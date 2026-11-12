"""Run sendto_silhouette.py in dry-run mode with the registration-mark search
answered, so a print-and-cut job can be checked without a cutter.

    <venv python> tools/labels/dry_cut.py <sendto_silhouette.py> <driver args...>

With --dry_run=True and no cutter found, the driver sends nothing and every read
returns None. A job with registration marks then stops at the mark search,
which waits for the cutter's "found" reply ("    0" + ETX), so a plain dry run
never reaches the cut paths. This stands in for that one reply and leaves
everything else to the driver; the command transcript (--cmdfile) is then the
full job a Portrait 2 would receive, mark search included.

A dry run never opens the cutter, even with one plugged in. The driver probes
USB in a dry run too: with a cutter attached it opens it, sends it the status
and firmware queries (it holds back only commands that are not queries), and
reads the mark-search reply from it, which never comes because the search
command was not sent. So the USB probe here finds nothing, which is the
driver's own dry run with no cutter attached (_find_usb_device()), and a dry run
gives the same transcript whether a cutter is plugged in or not.
"""

import runpy
import sys
from pathlib import Path

import autoblade

FOUND = b"    0\x03"
NO_DEVICE = "Crashtest Dummy Device"  # what the driver names a dry run with no cutter


def patch_dry(graphtec) -> None:
    """Keep graphtec.SilhouetteCameo off USB and answer its mark search."""

    def find_no_usb_device(self):
        print(
            f"No USB probe in a dry run (tools/labels/dry_cut.py); {NO_DEVICE}",
            file=self.log,
        )
        self.hardware = {"name": NO_DEVICE}
        return None

    graphtec.SilhouetteCameo._find_usb_device = find_no_usb_device

    real_read = graphtec.SilhouetteCameo.read

    def read(self, *args, **kwargs):
        resp = real_read(self, *args, **kwargs)
        return FOUND if resp is None else resp

    graphtec.SilhouetteCameo.read = read


def main() -> None:
    driver = Path(sys.argv[1]).resolve()
    if "--dry_run=True" not in sys.argv[2:]:
        sys.exit("dry_cut.py only runs dry: pass --dry_run=True")
    Graphtec = autoblade.load_graphtec(driver)
    autoblade.patch_depth(Graphtec)  # as a live job, so the transcript matches it
    patch_dry(Graphtec)
    sys.argv = [str(driver), *sys.argv[2:]]
    runpy.run_path(str(driver), run_name="__main__")


if __name__ == "__main__":
    main()
