"""Run sendto_silhouette.py with the AutoBlade depth sent to a Portrait 2.

    <venv python> tools/labels/autoblade.py <sendto_silhouette.py> <driver args...>

The driver sends the AutoBlade depth command (TF<depth>,<holder>) only to the
Cameo 3 and Cameo 4 lines (Graphtec.PRODUCT_LINE_CAMEO3_ON). A Portrait 2 takes
an AutoBlade and acts on the command, but is not in that list, so --depth never
reached it and the blade stayed at the depth it was last set to. With the blade
left deep, a kiss cut went through the liner.

On the Portrait 2 the depth command needs a tool-select (J1) before it. Sent
alone, TF1,1 moved the blade but left it at the wrong depth; "J1 TF<d>,1" put
the indicator on 1, 5 and 8 when asked for each.

patch_depth() wraps the driver's setup(): after the driver's own setup, a
cutter outside that list is sent J1 and the depth the job asked for. Everything else
is left to the driver. dry_cut.py applies the same patch, so a dry-run
transcript shows the command a live job sends.
"""

import runpy
import sys
from pathlib import Path

MIN_DEPTH, MAX_DEPTH = 0, 10
TOOLHOLDER = 1  # the only holder a Portrait has; the driver uses 1 for depth too


def patch_depth(graphtec) -> None:
    """Make graphtec.SilhouetteCameo.setup() send the depth to models the driver skips."""
    real_setup = graphtec.SilhouetteCameo.setup

    def setup(self, *args, **kwargs):
        result = real_setup(self, *args, **kwargs)
        depth = kwargs.get("depth")
        if not kwargs.get("autoblade") or depth is None:
            return result
        if self.product_id() in graphtec.PRODUCT_LINE_CAMEO3_ON:
            return result  # the driver has sent it
        depth = max(MIN_DEPTH, min(MAX_DEPTH, int(depth)))
        # The depth command alone moved the blade to the wrong depth (9, then 6,
        # asked for 1). After a tool-select it landed on the depth asked for.
        self.send_command(f"J{TOOLHOLDER}")
        self.send_command(f"TF{depth},{TOOLHOLDER}")
        print(f"depth: {depth} (sent by tools/labels/autoblade.py)", file=self.log)
        return result

    graphtec.SilhouetteCameo.setup = setup


def load_graphtec(driver: Path):
    """Import the driver's own silhouette.Graphtec, from beside sendto_silhouette.py."""
    sys.path.insert(0, str(driver.parent))
    from silhouette import Graphtec

    return Graphtec


def main() -> None:
    driver = Path(sys.argv[1]).resolve()
    patch_depth(load_graphtec(driver))
    sys.argv = [str(driver), *sys.argv[2:]]
    runpy.run_path(str(driver), run_name="__main__")


if __name__ == "__main__":
    main()
