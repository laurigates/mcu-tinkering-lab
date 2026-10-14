"""Rewrite a KiCad Specctra DSN's single net class into the board's width classes.

Pure text in, text out, so it is unit-tested without KiCad
(tools/kicad/test_dsn.py loads this file by path). It runs under KiCad's
bundled Python 3.9 as the sibling of route_board.py: no 3.10+ syntax, stdlib
`re` and `fnmatch` only.

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
"""

import fnmatch
import re

_DEFAULT_CLASS = re.compile(r"    \(class kicad_default (.*?)\n      \(circuit", re.S)
_DEFAULT_VIA = re.compile(r'^    \(via "([^"]*)"\)$', re.M)


class DsnError(Exception):
    """The DSN or the classes cannot be rewritten as asked."""


def via_name(via_um):
    """`[1000, 500]` -> KiCad's padstack name `Via[0-1]_1000:500_um`."""
    diameter, drill = via_um
    return "Via[0-1]_%d:%d_um" % (diameter, drill)


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
