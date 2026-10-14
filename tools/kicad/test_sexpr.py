"""Tests for the s-expression span helpers.

python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import unittest

from kicad import sexpr
from kicad.errors import KicadError

LIB = """\
(kicad_symbol_lib
\t(version 20251024)
\t(symbol "A"
\t\t(property "Value" "R (1%) \\"x\\""
\t\t\t(at 0 0 0)
\t\t)
\t\t(symbol "A_1_1"
\t\t\t(pin passive line
\t\t\t\t(at 0 3.81 270)
\t\t\t)
\t\t)
\t)
\t(symbol "B")
)
"""


class BlockTests(unittest.TestCase):
    def test_block_at_matches_nesting(self):
        text = "(a (b c) (d (e)))tail"
        self.assertEqual(text[: sexpr.block_at(text, 0)], "(a (b c) (d (e)))")
        self.assertEqual(text[3 : sexpr.block_at(text, 3)], "(b c)")

    def test_parentheses_inside_strings_do_not_count(self):
        text = '(property "Value" "R (1%" (at 0 0))x'
        self.assertEqual(sexpr.block_at(text, 0), len(text) - 1)

    def test_escaped_quote_does_not_end_a_string(self):
        text = '(p "a \\" ) (" x)'
        self.assertEqual(sexpr.block_at(text, 0), len(text))

    def test_unbalanced_raises(self):
        with self.assertRaises(KicadError):
            sexpr.block_at("(a (b)", 0)
        with self.assertRaises(KicadError):
            sexpr.spans("(a))")
        with self.assertRaises(KicadError):
            sexpr.block_at("a", 0)

    def test_spans_are_top_level_only(self):
        text = ' (a (b)) "(not)" (c)'
        self.assertEqual([text[a:b] for a, b in sexpr.spans(text)], ["(a (b))", "(c)"])

    def test_top_level_blocks_keep_indent_and_skip_nested(self):
        found = sexpr.top_level_blocks(LIB, "\t")
        self.assertEqual(
            [sexpr.head(b) for _, b in found],
            [("version", None), ("symbol", "A"), ("symbol", "B")],
        )
        start, block = found[1]
        self.assertTrue(block.startswith('\t(symbol "A"\n'))
        self.assertTrue(block.endswith("\n\t)"))
        self.assertEqual(LIB[start : start + len(block)], block)

    def test_inner_children_head_atoms(self):
        _, block = sexpr.top_level_blocks(LIB, "\t")[1]
        kids = sexpr.children(block)
        self.assertEqual([sexpr.head(k) for k in kids][1], ("symbol", "A_1_1"))
        self.assertEqual(sexpr.head(kids[0]), ("property", "Value"))
        pin = sexpr.child(kids[1], "pin")
        self.assertEqual(sexpr.atoms(pin), ["passive", "line"])
        self.assertEqual(sexpr.atoms(sexpr.child(pin, "at")), ["0", "3.81", "270"])
        self.assertIsNone(sexpr.child(pin, "length"))

    def test_quoted_value_round_trips(self):
        _, block = sexpr.top_level_blocks(LIB, "\t")[1]
        prop = sexpr.child(block, "property")
        self.assertEqual(sexpr.atoms(prop), [])
        # The second string is the value; head() reports only the first.
        self.assertIn('"R (1%) \\"x\\""', prop)
        self.assertEqual(sexpr.unquote('R (1%) \\"x\\"'), 'R (1%) "x"')


if __name__ == "__main__":
    unittest.main()
