"""``python -m common.animate entries/<slug>``: tools/mechanism's ``mechanism.animate``."""

import sys

from mechanism.animate import main

if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
