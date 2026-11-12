"""``python -m common.quickview entries/<slug>``: tools/mechanism's ``mechanism.quickview``."""

import sys

from mechanism.quickview import *  # noqa: F403 (entries import names from here)
from mechanism.quickview import main

if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
