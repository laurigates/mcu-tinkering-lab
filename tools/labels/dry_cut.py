"""Run sendto_silhouette.py in dry-run mode with the registration-mark search
answered, so a print-and-cut job can be checked without a cutter.

    <venv python> tools/labels/dry_cut.py <sendto_silhouette.py> <driver args...>

With --dry_run=True the driver sends nothing and every read returns None. A job
with registration marks then stops at the mark search, which waits for the
cutter's "found" reply ("    0" + ETX), so a plain dry run never reaches the
cut paths. This stands in for that one reply and leaves everything else to the
driver; the command transcript (--cmdfile) is then the full job a Portrait 2
would receive, mark search included.
"""

import runpy
import sys
from pathlib import Path

FOUND = b"    0\x03"


def main() -> None:
    driver = Path(sys.argv[1]).resolve()
    if "--dry_run=True" not in sys.argv[2:]:
        sys.exit("dry_cut.py only runs dry: pass --dry_run=True")
    sys.path.insert(0, str(driver.parent))
    from silhouette import Graphtec  # noqa: E402  (the driver's own package)

    real_read = Graphtec.SilhouetteCameo.read

    def read(self, *args, **kwargs):
        resp = real_read(self, *args, **kwargs)
        return FOUND if resp is None else resp

    Graphtec.SilhouetteCameo.read = read
    sys.argv = [str(driver), *sys.argv[2:]]
    runpy.run_path(str(driver), run_name="__main__")


if __name__ == "__main__":
    main()
