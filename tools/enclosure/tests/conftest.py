import sys
from pathlib import Path

# tools/ for the stdlib kicad package, tools/enclosure for this one.
HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent.parent), str(HERE.parent)]
