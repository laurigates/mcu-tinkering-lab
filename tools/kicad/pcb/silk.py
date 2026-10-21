"""Silkscreen stroke floor for library footprints (pcbnew-free).

KiCad's stock footprints draw their silkscreen at 0.12 mm (some 0.10 or
0.127 mm), under JLCPCB's 0.15 mm minimum stroke: a polarity mark ("+" on an
electrolytic, a diode's cathode bar) drawn that thin may print broken or not
at all. build_board.py widens each footprint's silkscreen stroke below the
floor to it, as it already keeps reference text at 0.16 mm or more. A
filled shape's zero-width outline stays zero.

Loaded by path from build_board.py and test_silk.py (`unittest discover` does
not reach pcb/), so it imports nothing from KiCad.
"""

# JLCPCB's minimum silkscreen line width, in nanometres (KiCad's unit).
MIN_STROKE_NM = 150_000


def widened(width_nm, minimum_nm=MIN_STROKE_NM):
    """The stroke to print for a silkscreen item drawn `width_nm` wide."""
    if width_nm <= 0 or width_nm >= minimum_nm:
        return width_nm
    return minimum_nm
