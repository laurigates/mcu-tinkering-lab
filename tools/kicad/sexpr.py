"""S-expression spans in KiCad files, without a full parser.

KiCad writes one item per line, tab-indented, so most questions are about a
block: where it starts, where its matching `)` is, and what its direct children
are. These helpers answer those and nothing more; the text between is kept
byte for byte, which is what embedding a library symbol and copying a footprint
need.

Every scan is string-aware: a `(` or `)` inside a quoted string (`"R (1%)"`)
does not count, and `\\"` inside a string does not end it.
"""

from __future__ import annotations

import re

from .errors import KicadError


def block_at(text: str, start: int) -> int:
    """The index just past the `)` matching the `(` at `text[start]`."""
    if text[start : start + 1] != "(":
        raise KicadError(
            f"offset {start}: expected '(', got {text[start : start + 1]!r}"
        )
    depth = 0
    i = start
    n = len(text)
    while i < n:
        ch = text[i]
        if ch == '"':
            i += 1
            while i < n and text[i] != '"':
                i += 2 if text[i] == "\\" else 1
        elif ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth == 0:
                return i + 1
        i += 1
    raise KicadError(f"offset {start}: unbalanced '(' (no matching ')')")


def spans(text: str) -> list[tuple[int, int]]:
    """(start, end) of every top-level s-expression in `text`, in order."""
    out = []
    i = 0
    n = len(text)
    while i < n:
        ch = text[i]
        if ch == '"':
            i += 1
            while i < n and text[i] != '"':
                i += 2 if text[i] == "\\" else 1
            i += 1
        elif ch == "(":
            end = block_at(text, i)
            out.append((i, end))
            i = end
        elif ch == ")":
            raise KicadError(f"offset {i}: unbalanced ')'")
        else:
            i += 1
    return out


def inner(text: str) -> str:
    """The body of the outermost s-expression: after its head, before its `)`.

    The head is the keyword and, when present, its first quoted argument:
    `(symbol "R"` or `(footprint "R_0805"`.
    """
    head = re.match(r'\s*\(\S+(?: "(?:[^"\\]|\\.)*")?', text)
    if not head:
        raise KicadError(f"not an s-expression: {text[:40]!r}")
    return text[head.end() : text.rstrip().rindex(")")]


def children(block: str) -> list[str]:
    """The direct child s-expressions of `block`, each as its own text."""
    body = inner(block)
    return [body[a:b] for a, b in spans(body)]


def head(block: str) -> tuple[str, str | None]:
    """(keyword, first quoted argument or None) of an s-expression."""
    m = re.match(r'\s*\((\S+?)(?:[\s)]|$)(?:\s*"((?:[^"\\]|\\.)*)")?', block)
    if not m:
        raise KicadError(f"not an s-expression: {block[:40]!r}")
    name = m.group(2)
    return m.group(1), None if name is None else unquote(name)


def unquote(s: str) -> str:
    """A KiCad string's content: `\\"` -> `"` and `\\\\` -> `\\`."""
    return re.sub(r"\\(.)", r"\1", s)


def top_level_blocks(text: str, indent: str) -> list[tuple[int, str]]:
    """(line start, block) for every block whose line starts with `indent` + "(".

    The block runs from the start of its line (indent included) to its matching
    `)`, so a library symbol comes out exactly as written. Blocks nested inside a
    found block are not reported, so `indent="\\t"` gives a library's symbols and
    not their sub-units, even when a sub-unit's text happens to sit at that indent.
    """
    out = []
    pos = 0
    pat = re.compile(rf"^{re.escape(indent)}\(", re.M)
    while True:
        m = pat.search(text, pos)
        if not m:
            return out
        open_at = m.end() - 1
        end = block_at(text, open_at)
        out.append((m.start(), text[m.start() : end]))
        pos = end


def atoms(block: str) -> list[str]:
    """The bare (unquoted, non-list) atoms directly inside `block`, after the keyword.

    `(at 1.27 -2.54 90)` -> ["1.27", "-2.54", "90"]; `(attr smd)` -> ["smd"].
    """
    body = block.strip()
    body = body[1 : body.rindex(")")]
    out = []
    i = 0
    n = len(body)
    tok = ""
    depth = 0
    while i < n:
        ch = body[i]
        if ch == '"':
            i += 1
            while i < n and body[i] != '"':
                i += 2 if body[i] == "\\" else 1
            tok = ""
        elif ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        elif ch.isspace():
            if tok and depth == 0:
                out.append(tok)
            tok = ""
        elif depth == 0:
            tok += ch
        i += 1
    if tok:
        out.append(tok)
    return out[1:]


def child(block: str, keyword: str) -> str | None:
    """The first direct child of `block` whose keyword is `keyword`, or None."""
    for c in children(block):
        if head(c)[0] == keyword:
            return c
    return None
