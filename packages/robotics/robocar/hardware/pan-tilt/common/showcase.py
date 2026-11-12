"""``python -m common.showcase entries/<slug>``: tools/mechanism's ``mechanism.showcase``."""

import sys

from mechanism.showcase import *  # noqa: F403 (entries import names from here)
from mechanism.showcase import main

if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
