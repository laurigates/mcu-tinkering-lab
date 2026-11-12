"""The pan-tilt tournament's shared stand-ins (fixtures.py) and the entry
points it ran its gates through. The gates themselves are tools/mechanism's;
this puts that package on the path for ``python -m common.<gate>`` and for
entries that ``from common import fixtures``."""

import sys
from pathlib import Path

_MECHANISM = str(Path(__file__).resolve().parents[6] / "tools" / "mechanism")
if _MECHANISM not in sys.path:
    sys.path.insert(0, _MECHANISM)
