"""Silkscreen stroke floor and pad clearance for library footprints (pcbnew-free).

KiCad's stock footprints draw their silkscreen at 0.12 mm (some 0.10 or
0.127 mm), under JLCPCB's 0.15 mm minimum stroke: a polarity mark ("+" on an
electrolytic, a diode's cathode bar) drawn that thin may print broken or not
at all. build_board.py widens each footprint's silkscreen stroke below the
floor to it, as it already keeps reference text at 0.16 mm or more. A
filled shape's zero-width outline stays zero.

JLCPCB's CAM also deletes silkscreen within 0.15 mm of a pad's mask opening,
without asking (character-design-specifications, docs/reference/jlc-blog/).
build_board.py cuts those pieces out of each footprint's silkscreen lines
(`clip_segment`), so the board shows what will be printed; the carriers'
.kicad_dru rule "silkscreen 0.15 mm clear of pads" checks the result.

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


# JLCPCB's silkscreen-to-pad clearance, and the margin build_board.py adds so a
# clipped line's sampled end never lands inside the DRC rule's limit.
PAD_CLEARANCE_NM = 150_000
MARGIN_NM = 25_000


def clearance(width_nm, mask_nm=0):
    """Distance from a pad's copper to a silk line's centre for the line to survive."""
    return PAD_CLEARANCE_NM + width_nm // 2 + max(mask_nm, 0) + MARGIN_NM


def clip_segment(a, b, blocked, step_nm, min_len_nm):
    """The pieces of line a-b that survive, in a-to-b order.

    The line is cut into equal pieces at most `step_nm` long; `blocked(p, q)`
    says whether piece p-q comes too close to a pad. Runs of unblocked pieces
    are kept, and a run shorter than `min_len_nm` is dropped as a sliver.
    """
    (ax, ay), (bx, by) = a, b
    length = ((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5
    n = max(1, -(-int(length) // step_nm))

    def at(i):
        if i == 0:
            return a
        if i == n:
            return b
        return (ax + round((bx - ax) * i / n), ay + round((by - ay) * i / n))

    pieces, start = [], None
    for i in range(n + 1):
        free = i < n and not blocked(at(i), at(i + 1))
        if free and start is None:
            start = i
        elif not free and start is not None:
            if length * (i - start) / n >= min_len_nm:
                pieces.append((at(start), at(i)))
            start = None
    return pieces
