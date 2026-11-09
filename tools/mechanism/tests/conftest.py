import sys
from pathlib import Path

# tools/mechanism for this package, tools/ for cadstd (ADR-035).
HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent), str(HERE.parent.parent)]
