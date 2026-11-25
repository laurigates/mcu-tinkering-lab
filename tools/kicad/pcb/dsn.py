"""Rewrite a KiCad Specctra DSN's single net class into the board's width classes.

Pure text in, text out, so it is unit-tested without KiCad
(tools/kicad/test_dsn.py loads this file by path). It runs under KiCad's
bundled Python 3.9 as the sibling of route_board.py: no 3.10+ syntax, stdlib
`re`, `fnmatch` and `math` only.

KiCad exports every net into one `(class kicad_default ...)` with the
`.kicad_pro` Default class's via. `rewrite(text, classes, signal)` replaces it
with one class per entry of `classes`, in order, then a `signal` class holding
every other net, and adds a padstack and a `(via ...)` entry for each class via
the export does not already have.

`classes` are board.toml's `[[net_classes]]` as tools/kicad's board input
carries them: `{"name", "nets", "track_um", "clearance_um", "via_um"}`, `nets`
being net names or fnmatch patterns over the DSN's nets. `signal` is
`[signal]`: `{"track_um", "clearance_um", "via_um"}`. A via `[d, drill]` in um
is KiCad's padstack `Via[0-1]_<d>:<drill>_um`.

`track_keepouts(text, tracks, origin, margin_um)` adds a keepout on its layer
over each segment of board.toml's `[[tracks]]`. Freerouting 2.5.0 drops the
DSN's `(wiring ...)` section, fixed and protected wires included, and routes
other nets straight through them; a keepout it honours.
`drop_pins(text, pins)` leaves a pad such a track already joins to its net
out of the netlist, so Freerouting does not route to it again.
"""

import fnmatch
import math
import re

_DEFAULT_CLASS = re.compile(r"    \(class kicad_default (.*?)\n      \(circuit", re.S)
_DEFAULT_VIA = re.compile(r'^    \(via "([^"]*)"\)$', re.M)


class DsnError(Exception):
    """The DSN or the classes cannot be rewritten as asked."""


def via_name(via_um):
    """`[1000, 500]` -> KiCad's padstack name `Via[0-1]_1000:500_um`."""
    diameter, drill = via_um
    return f"Via[0-1]_{int(diameter)}:{int(drill)}_um"


def class_block(name, nets, width, clearance, via):
    return (
        f"    (class {name} {' '.join(nets)}\n"
        f'      (circuit\n        (use_via "{via}")\n      )\n'
        f"      (rule\n        (width {width})\n        (clearance {clearance})\n      )\n    )\n"
    )


def expand(patterns, all_nets, where):
    """The nets of `all_nets` the patterns name, in pattern order, then net order."""
    out = []
    for pattern in patterns:
        if any(c in pattern for c in "*?["):
            hits = [n for n in all_nets if fnmatch.fnmatchcase(n, pattern)]
            if not hits:
                raise DsnError(f"{where}: {pattern!r} matches no net in the board")
        elif pattern in all_nets:
            hits = [pattern]
        else:
            raise DsnError(f"{where}: class net {pattern!r} is not in the board")
        out.extend(n for n in hits if n not in out)
    return out


def rewrite(text, classes, signal):
    """`text` with the kicad_default class split into `classes` plus `signal`."""
    m = _DEFAULT_CLASS.search(text)
    if not m:
        raise DsnError("DSN has no kicad_default class to rewrite")
    vias = _DEFAULT_VIA.findall(text)
    if len(vias) != 1:
        raise DsnError(f"DSN has {len(vias)} default (via ...) lines, expected 1")
    default_via = vias[0]
    signal_via = via_name(signal["via_um"])
    if default_via != signal_via:
        raise DsnError(
            f"DSN default via {default_via!r} is not [signal] via_um's {signal_via!r}; "
            "the .kicad_pro Default net class and board.toml [signal] must agree"
        )
    all_nets = m.group(1).split()
    owner = {}
    blocks = []
    extra_vias = {}
    for cls in classes:
        name = cls["name"]
        nets = expand(cls["nets"], all_nets, f"class {name!r}")
        for n in nets:
            if n in owner:
                raise DsnError(f"net {n!r} is in classes {owner[n]!r} and {name!r}")
            owner[n] = name
        via = via_name(cls["via_um"])
        if via != signal_via:
            extra_vias.setdefault(via, cls["via_um"][0])
        blocks.append(
            class_block(name, nets, cls["track_um"], cls["clearance_um"], via)
        )
    signal_nets = [n for n in all_nets if n not in owner]
    blocks.append(
        class_block(
            "signal",
            signal_nets,
            signal["track_um"],
            signal["clearance_um"],
            signal_via,
        )
    )
    start = m.start()
    end = text.index("    )\n", text.index("(rule", m.end())) + len("    )\n")
    text = text[:start] + "".join(blocks) + text[end:]

    lib_end = text.index("  )\n  (network")
    pads = "".join(
        f'    (padstack "{name}"\n      (shape (circle F.Cu {d}))\n'
        f"      (shape (circle B.Cu {d}))\n      (attach off)\n    )\n"
        for name, d in extra_vias.items()
    )
    text = text[:lib_end] + pads + text[lib_end:]
    listed = " ".join(f'"{v}"' for v in [signal_via, *extra_vias])
    return text.replace(f'(via "{default_via}")', f"(via {listed})", 1)


def track_keepouts(text, tracks, origin, margin_um):
    """`text` with a keepout on its layer over every segment of each track.

    Each keepout is the segment's rectangle widened by half the track's width
    plus `margin_um` (the widest class clearance) to each side and run on that
    far past an interior corner, so the bend is covered, but flush with the
    polyline's own two ends: a track that ends inside a pad leaves the pad's
    centre to the router. Coordinates are the DSN's um with y negated. They go
    before the structure's `(via ...)`.
    """
    if not tracks:
        return text
    anchor = text.find('\n    (via "')
    if anchor < 0:
        raise DsnError("DSN has no structure (via ...) line to add keepouts before")
    ox, oy = origin
    lines = []
    for tr in tracks:
        h = tr["width_mm"] * 500 + margin_um  # um
        pts = tr["points"]
        last = len(pts) - 2
        for i, ((ax, ay), (bx, by)) in enumerate(zip(pts, pts[1:])):
            run_a = 0 if i == 0 else 1
            run_b = 0 if i == last else 1
            ax, ay = (ox + ax) * 1000, (oy + ay) * 1000
            bx, by = (ox + bx) * 1000, (oy + by) * 1000
            length = math.hypot(bx - ax, by - ay)
            if length == 0:
                raise DsnError(f"track {tr['name']!r} has a zero-length segment")
            dx, dy = (bx - ax) / length * h, (by - ay) / length * h
            nx, ny = -dy, dx
            corners = [
                (ax - run_a * dx - nx, ay - run_a * dy - ny),
                (bx + run_b * dx - nx, by + run_b * dy - ny),
                (bx + run_b * dx + nx, by + run_b * dy + ny),
                (ax - run_a * dx + nx, ay - run_a * dy + ny),
            ]
            corners.append(corners[0])
            coords = "  ".join(f"{round(x)} {-round(y)}" for x, y in corners)
            lines.append(f'    (keepout "" (polygon {tr["layer"]} 0  {coords}))')
    return text[: anchor + 1] + "\n".join(lines) + text[anchor:]


def rect_keepouts(text, keepouts, origin):
    """`text` with a keepout on its one layer over each [[route_keepouts]] rect.

    These bind Freerouting only: the board gets no rule area, so footprints and
    the pour may still go there. Coordinates are the DSN's um with y negated;
    they go before the structure's `(via ...)`.
    """
    if not keepouts:
        return text
    anchor = text.find('\n    (via "')
    if anchor < 0:
        raise DsnError("DSN has no structure (via ...) line to add keepouts before")
    ox, oy = origin
    lines = []
    for k in keepouts:
        x0, y0, x1, y1 = (
            (ox + k["rect"][0]) * 1000,
            (oy + k["rect"][1]) * 1000,
            (ox + k["rect"][2]) * 1000,
            (oy + k["rect"][3]) * 1000,
        )
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
        coords = "  ".join(f"{round(x)} {-round(y)}" for x, y in corners)
        lines.append(f'    (keepout "" (polygon {k["layer"]} 0  {coords}))')
    return text[: anchor + 1] + "\n".join(lines) + text[anchor:]


def no_via_at_smd(text):
    """`text` with `(control (via_at_smd off))` in its structure.

    Without it Freerouting may drop a via inside an SMD pad, where the hole
    drains the pad's paste at reflow.
    """
    if "(via_at_smd" in text:
        return text
    anchor = text.find('\n    (via "')
    if anchor < 0:
        raise DsnError("DSN has no structure (via ...) line to add a control before")
    return text[: anchor + 1] + "    (control (via_at_smd off))" + text[anchor:]


def drop_pins(text, pins):
    """`text` with each `(net, "REF-PAD")` of `pins` left out of that net.

    route_board.py drops the pad under each [[tracks]] entry's first point: the
    track joins it to the pad under its last point, which stays in the net, so
    Freerouting need not reach it. Left in, Freerouting routes to it again, and
    a pad the track's keepouts box in can stop it finishing the net at all.
    """
    for net, pin in pins:
        found = re.search(rf'\(net ("?){re.escape(net)}\1\n\s*\(pins ([^)]*)\)', text)
        if found is None:
            raise DsnError(f"DSN has no net {net!r}")
        listed = found.group(2)
        # " PIN" after another pin, else "PIN" and what follows it at the start
        name = re.escape(pin)
        kept, n = re.subn(rf" {name}(?=\s|$)", "", listed)
        if n == 0:
            kept, n = re.subn(rf"^{name}(?:\s+|$)", "", listed)
        if n != 1:
            raise DsnError(f"{pin} is not a pin of net {net!r}")
        text = text[: found.start(2)] + kept + text[found.end(2) :]
    return text


_STUCK = re.compile(r"insert trace failed .*? target: \((-?\d+),(-?\d+)\)")


def stuck_points(log, origin, units_per_mm=10000):
    """Board-relative mm of each point Freerouting failed to reach, from its log.

    `log` is Freerouting's output at DEBUG; each "insert trace failed ...
    target: (x,y)" names a point in DSN units (KiCad writes resolution um 10,
    so 10000 per mm) with y negated. The points come back once each, in the
    order first logged, as (x, y) in mm from board.toml's [outline] origin.
    """
    seen = []
    for m in _STUCK.finditer(log):
        x = round(int(m.group(1)) / units_per_mm - origin[0], 2)
        y = round(-int(m.group(2)) / units_per_mm - origin[1], 2)
        if (x, y) not in seen:
            seen.append((x, y))
    return seen


_FAILED = re.compile(r"Failed to route connection between (.*?), because ([^.\n]*)")
_PIN_OF = re.compile(r"pin (?:#(\d+) )?of component #(\d+)")
_PLACE = re.compile(r'\(place\s+("[^"]+"|\S+)')
_COMPONENT = re.compile(r'\(component\s+("[^"]+"|\S+)')
_IMAGE_PIN = re.compile(r'\(pin\s+\S+\s+(?:\(rotate [^)]*\)\s+)?("[^"]+"|\S+)\s')


def _unquote(name):
    return name[1:-1] if name.startswith('"') else name


def _pins_by_index(dsn_text):
    """[(ref, image)] by DSN place order, and {image: [pin names]} by pin order."""
    pl_start = dsn_text.index("(placement")
    lib_start = dsn_text.index("(library")
    places, images, image = [], {}, None
    for line in dsn_text[pl_start:lib_start].splitlines():
        c = _COMPONENT.search(line)
        if c:
            image = _unquote(c.group(1))
        p = _PLACE.search(line)
        if p:
            places.append((_unquote(p.group(1)), image))
    for block in dsn_text[lib_start:].split("(image ")[1:]:
        name = block.split(None, 1)[0]
        images[_unquote(name)] = [
            _unquote(m.group(1)) for m in _IMAGE_PIN.finditer(block)
        ]
    return places, images


def _pads(items, places, images):
    pads = []
    for pin, comp in _PIN_OF.findall(items):
        ref, img = places[int(comp) - 1]
        pads.append(f"{ref}-{images.get(img, [])[int(pin)]}" if pin else ref)
    return pads


def failed_connections(log, dsn_text):
    """Freerouting's failed connections from its DEBUG log, by pad.

    Each "Failed to route connection between A and B, because R" names pins
    as "pin #k of component #n": n counts the DSN's (place ...) entries from
    1 in file order, k the component image's (pin ...) entries from 0 (a pin
    logged without its number comes back as its REF). Returns (reason,
    A pads, B pads) once each, in the order first logged; a side made only of
    tracks and vias is an empty tuple. "the maze search algorithm could not be
    created" is a start item with no way out (a walled-in pad); "no connection
    was found between their nets" with one small side is the pad the rest of
    the net cannot reach (carrier-pcb trap 28).
    """
    places, images = _pins_by_index(dsn_text)
    out = []
    for items, reason in _FAILED.findall(log):
        a, _, b = items.partition(" and ")
        entry = (
            reason.strip(),
            tuple(_pads(a, places, images)),
            tuple(_pads(b, places, images)),
        )
        if entry not in out:
            out.append(entry)
    return out


def failed_searches(log, dsn_text):
    """The connections whose route search could not start, one string each.

    The pads of both sides, "REF-PAD, REF-PAD ..." (failed_connections with
    reason "the maze search algorithm could not be created").
    """
    out = []
    for reason, a, b in failed_connections(log, dsn_text):
        if reason == "the maze search algorithm could not be created":
            text = ", ".join(a + b)
            if text and text not in out:
                out.append(text)
    return out
