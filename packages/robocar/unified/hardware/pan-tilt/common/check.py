"""``python -m common.check entries/<slug>``: tools/mechanism's ``mechanism.check``."""

import sys

from mechanism.check import *  # noqa: F403 (entries import names from here)
from mechanism.check import main

if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
