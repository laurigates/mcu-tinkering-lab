"""Derive a socket-mount XIAO footprint and symbol from Seeed's XIAO-RP2350-DIP.

Seeed's DIP footprint is for soldering the XIAO flat onto a host board: it
carries an Edge.Cuts cutout under the module and plated holes for the ten bottom
pads (15-24) along that cutout. On this carrier the XIAO sits in two 1x7 female
headers, so the cutout would mill a hole in the board and the bottom pads have
nothing to meet. This keeps the vendor geometry for pads 1-14 and drops:

- every Edge.Cuts graphic
- every pad numbered 15-24
- the SMD castellation pads (the socket pins only need the through-holes)

The symbol gets the matching cut (pins 15-24 removed), so schematic parity has
no pins without pads. Seeed's DIP footprint and symbol stay in the library,
unmodified, as the source.

Run with any Python 3; it does not need pcbnew:

    python3 scripts/derive_xiao_socket.py
"""

import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
LIB = os.path.join(os.path.dirname(HERE), "balancebot-carrier", "lib")
FP_SRC = os.path.join(LIB, "Seeed_XIAO.pretty", "XIAO-RP2350-DIP.kicad_mod")
FP_DST = os.path.join(LIB, "Seeed_XIAO.pretty", "XIAO-RP2350-Socket.kicad_mod")
SYM_LIB = os.path.join(LIB, "Seeed_XIAO.kicad_sym")
VENDOR, DERIVED = "XIAO-RP2350-DIP", "XIAO-RP2350-Socket"
KEPT_PADS = range(1, 15)


def blocks(text):
    """Top-level s-expression spans inside ``text``."""
    spans, depth, start = [], 0, None
    for i, ch in enumerate(text):
        if ch == "(":
            if depth == 0:
                start = i
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth == 0:
                spans.append((start, i + 1))
    return spans


def inner(text):
    """The body of the outermost s-expression: after its head, before its ')'."""
    head = re.match(r"\(\S+ \"[^\"]*\"", text)
    return text[head.end() : text.rstrip().rindex(")")]


def keep_fp_item(item):
    if '(layer "Edge.Cuts")' in item:
        return False
    m = re.match(r'\(pad "(\d+)" (\w+)', item)
    return not m or (int(m.group(1)) in KEPT_PADS and m.group(2) == "thru_hole")


def derive_footprint():
    body = inner(open(FP_SRC).read())
    kept = [body[a:b] for a, b in blocks(body) if keep_fp_item(body[a:b])]
    pads = sorted(
        int(re.match(r'\(pad "(\d+)"', k).group(1))
        for k in kept
        if k.startswith("(pad ")
    )
    if pads != list(KEPT_PADS):
        raise SystemExit(f"expected one through-hole pad for each of 1-14, got {pads}")
    open(FP_DST, "w").write(f'(footprint "{DERIVED}"\n\t' + "\n\t".join(kept) + "\n)\n")
    print(f"wrote {FP_DST}: {len(pads)} pads")


def strip_pins(symbol):
    """Drop (pin ...) blocks numbered outside KEPT_PADS, at any depth."""
    out, i = [], 0
    for m in re.finditer(r"\(pin ", symbol):
        if m.start() < i:
            continue
        a = m.start()
        b = a + blocks(symbol[a:])[0][1]
        number = re.search(r'\(number "(\d+)"', symbol[a:b])
        if number and int(number.group(1)) not in KEPT_PADS:
            out.append(symbol[i:a].rstrip(" \t"))
            i = b
    out.append(symbol[i:])
    return "".join(out)


def derive_symbol():
    lib = open(SYM_LIB).read()
    header = lib[: lib.index("(symbol ")].rstrip()
    symbols = {}
    for a, b in blocks(lib[len(header) :]):
        item = lib[len(header) + a : len(header) + b]
        name = re.match(r'\(symbol "([^"]+)"', item)
        if name:
            symbols[name.group(1)] = item
    if VENDOR not in symbols:
        raise SystemExit(f"{VENDOR} not in {SYM_LIB}")
    derived = strip_pins(symbols[VENDOR]).replace(f'"{VENDOR}', f'"{DERIVED}')
    derived = re.sub(
        r'(\(property "Footprint" )"[^"]*"',
        rf'\1"Seeed_XIAO:{DERIVED}"',
        derived,
        count=1,
    )
    numbers = sorted(int(n) for n in re.findall(r'\(number "(\d+)"', derived))
    if numbers != list(KEPT_PADS):
        raise SystemExit(f"derived symbol pins {numbers}, expected 1-14")
    symbols[DERIVED] = derived
    out = header + "\n\t" + "\n\t".join(symbols[n] for n in (VENDOR, DERIVED)) + "\n)\n"
    open(SYM_LIB, "w").write(out)
    print(f"wrote {SYM_LIB}: {DERIVED} with pins 1-14")


if __name__ == "__main__":
    derive_footprint()
    derive_symbol()
