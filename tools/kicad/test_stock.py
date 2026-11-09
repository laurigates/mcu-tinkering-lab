"""Tests for kicad.stock: carrier BOM lines against a parts-stock inventory.

Every inventory here is synthetic, written to a temp dir; the real
../parts-stock is never read.

python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import contextlib
import io
import os
import shutil
import tempfile
import tomllib
import unittest
from pathlib import Path
from unittest import mock

from kicad import registry, stock
from kicad.errors import KicadError
from kicad.registry import StockSpec
from kicad.stock import Need

CAP = """schema = 1
[item]
name = "0805 100 nF 50 V X7R"
category = "capacitor"
kind = "part"
package = "0805"
[stock]
mode = "count"
qty = 100
[specs]
value = "100nF"
voltage = "50V"
dielectric = "X7R"
"""

KIT = """schema = 1
[item]
name = "0805 ceramic capacitor kit, 16 values"
category = "capacitor"
kind = "kit"
package = "0805"
[stock]
mode = "level"
level = "full"
[specs]
unknown = ["voltage", "dielectric"]
[[contents]]
value = "100nF"
qty = 20
[[contents]]
value = "1uF"
qty = 20
"""

MOSFET = """schema = 1
[item]
name = "ZXMP3A16G P-MOSFET"
category = "transistor"
kind = "part"
package = "SOT-223"
mpn = "ZXMP3A16GTA"
[stock]
mode = "count"
qty = 5
"""

C0805 = StockSpec("", "capacitor", "0805", ("voltage", "dielectric"))
CPELEC = StockSpec("", "capacitor", "6.3x7.7", ("voltage",))
R0805 = StockSpec("", "resistor", "0805", ())
ZXMP4 = StockSpec("ZXMP4A16GTA", "", "", ())


def write_tree(test: unittest.TestCase, files: dict[str, str]) -> Path:
    """A synthetic inventory in a temp dir, removed when the test ends."""
    root = Path(tempfile.mkdtemp(prefix="parts-stock-test-"))
    test.addCleanup(shutil.rmtree, root)
    for rel, text in files.items():
        path = root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    return root


def item(text: str, slug: str = "x") -> stock.Item:
    return stock.parse_item(Path(f"items/c/{slug}.toml"), tomllib.loads(text))


def need(refs, value, key, lcsc="", buy="", slug="c0805") -> Need:
    return Need(tuple(refs), slug, value, key, lcsc, buy)


def count_item(slug, category, package, value="", qty=10, specs="", mpn=""):
    mpn_line = f'mpn = "{mpn}"\n' if mpn else ""
    value_line = f'value = "{value}"\n' if value else ""
    return item(
        f'schema = 1\n[item]\nname = "{slug}"\ncategory = "{category}"\n'
        f'kind = "part"\npackage = "{package}"\n{mpn_line}'
        f'[stock]\nmode = "count"\nqty = {qty}\n[specs]\n{value_line}{specs}',
        slug,
    )


class ValueTests(unittest.TestCase):
    def test_spellings_of_one_value_agree(self):
        for a, b in (
            ("100n", "100nF"),
            ("100nF", "0.1 µF"),
            ("100nF", "0.1μF"),
            ("100NF", "100nf"),
            ("10k", "10kΩ"),
            ("10k", "10kohm"),
            ("10K", "10 k"),
            ("4k7", "4.7k"),
            ("4R7", "4.7"),
            ("1M", "1MΩ"),
            ("100uF 35V", "100uF"),
        ):
            with self.subTest(a=a, b=b):
                self.assertTrue(stock.same_value(a, b))

    def test_different_values_differ(self):
        for a, b in (("100nF", "10nF"), ("10k", "1k"), ("1M", "1m"), ("3A", "2A")):
            with self.subTest(a=a, b=b):
                self.assertFalse(stock.same_value(a, b))

    def test_voltages(self):
        self.assertEqual(stock.needed_volts("100uF 35V"), 35.0)
        self.assertEqual(stock.needed_volts("3A hold 24V"), 24.0)
        self.assertIsNone(stock.needed_volts("100nF"))
        for v, want in (("50V", 50.0), ("6.3 V", 6.3), (50, 50.0), ("16", 16.0)):
            self.assertEqual(stock.volts(v), want)
        self.assertIsNone(stock.volts("high"))


class JudgeTests(unittest.TestCase):
    def test_on_hand_counts_the_rated_item_and_not_the_kit(self):
        line = stock.judge(
            need(["C3"], "100nF", C0805), [item(CAP, "cap"), item(KIT, "kit")]
        )
        self.assertEqual(line.status, stock.ON_HAND)
        self.assertEqual([i.slug for i in line.counted], ["cap"])
        self.assertEqual([i.slug for i, _ in line.unrated], ["kit"])
        self.assertIn("enough for 100 board(s)", line.detail)

    def test_kit_with_unknown_voltage_is_rating_unknown(self):
        line = stock.judge(need(["C3"], "100nF", C0805), [item(KIT, "kit")])
        self.assertEqual(line.status, stock.RATING_UNKNOWN)
        self.assertEqual(line.unrated[0][1], "voltage unknown, dielectric unknown")

    def test_a_needed_spec_not_stated_is_rating_unknown(self):
        cap = count_item("c", "capacitor", "0805", "100nF", specs='voltage = "50V"\n')
        line = stock.judge(need(["C3"], "100nF", C0805), [cap])
        self.assertEqual(line.status, stock.RATING_UNKNOWN)
        self.assertEqual(line.unrated[0][1], "dielectric not stated")

    def test_short_on_count(self):
        cap = count_item(
            "c",
            "capacitor",
            "0805",
            "100nF",
            qty=1,
            specs='voltage = "50V"\ndielectric = "X7R"\n',
        )
        line = stock.judge(need(["C1", "C2"], "100nF", C0805), [cap])
        self.assertEqual(line.status, stock.SHORT)
        self.assertEqual(line.detail, "x1 on hand")

    def test_short_on_a_low_level(self):
        low = item(
            'schema = 1\n[item]\nname = "r"\ncategory = "resistor"\nkind = "part"\n'
            'package = "0805"\n[stock]\nmode = "level"\nlevel = "low"\n[specs]\nvalue = "10k"\n'
        )
        line = stock.judge(need(["R1"], "10k", R0805, slug="r0805"), [low])
        self.assertEqual(line.status, stock.SHORT)
        self.assertEqual(line.detail, "level low")

    def test_a_level_item_with_some_is_on_hand_uncounted(self):
        some = item(
            'schema = 1\n[item]\nname = "r"\ncategory = "resistor"\nkind = "part"\n'
            'package = "0805"\n[stock]\nmode = "level"\nlevel = "some"\n[specs]\nvalue = "10kΩ"\n'
        )
        line = stock.judge(
            need(["R1", "R4"], "10k", R0805, slug="r0805"), [some], boards=5
        )
        self.assertEqual(line.status, stock.ON_HAND)
        self.assertEqual(line.detail, "level some (not counted)")

    def test_none_keeps_the_route_and_lists_the_near_match(self):
        n = need(["Q1"], "ZXMP4A16G", ZXMP4, lcsc="C141324", slug="zxmp4a16g")
        line = stock.judge(n, [item(MOSFET, "zxmp3a16g")])
        self.assertEqual(line.status, stock.NONE)
        self.assertEqual([i.slug for i, _ in line.near], ["zxmp3a16g"])
        out = stock.render([line])
        self.assertIn("near match, not counted: zxmp3a16g ZXMP3A16GTA x5", out)
        self.assertIn("route: lcsc C141324", out)

    def test_mpn_matches_only_exactly_and_ignores_case(self):
        exact = count_item("q", "transistor", "SOT-223", qty=3, mpn="zxmp4a16gta")
        line = stock.judge(need(["Q1"], "ZXMP4A16G", ZXMP4), [exact])
        self.assertEqual(line.status, stock.ON_HAND)
        unrelated = count_item("q", "transistor", "SOT-223", qty=3, mpn="AO3401A")
        line = stock.judge(need(["Q1"], "ZXMP4A16G", ZXMP4), [unrelated])
        self.assertEqual((line.status, line.near), (stock.NONE, ()))

    def test_other_values_in_the_package_are_near_matches(self):
        other = count_item(
            "c10n",
            "capacitor",
            "0805",
            "10nF",
            specs='voltage = "50V"\ndielectric = "X7R"\n',
        )
        line = stock.judge(need(["C3"], "100nF", C0805), [other])
        self.assertEqual(line.status, stock.NONE)
        self.assertEqual(line.near[0][1], "value 10nF")

    def test_a_voltage_below_the_value_is_a_near_match(self):
        low = count_item(
            "cp16", "capacitor", "6.3 x 7.7", "100uF", specs='voltage = "16V"\n'
        )
        ok = count_item(
            "cp50", "capacitor", "6.3X7.7", "100uF", qty=4, specs="voltage = 50\n"
        )
        line = stock.judge(need(["C1", "C2"], "100uF 35V", CPELEC), [low, ok])
        self.assertEqual(line.status, stock.ON_HAND)
        self.assertEqual([i.slug for i in line.counted], ["cp50"])
        self.assertEqual(line.near[0][1], "rated 16V, needs 35V")

    def test_boards_scale_the_need(self):
        cap = count_item(
            "c",
            "capacitor",
            "0805",
            "100nF",
            qty=5,
            specs='voltage = "50V"\ndielectric = "X7R"\n',
        )
        n = need(["C1", "C2"], "100nF", C0805)
        self.assertEqual(stock.judge(n, [cap], boards=2).status, stock.ON_HAND)
        line = stock.judge(n, [cap], boards=3)
        self.assertEqual((line.status, line.need_qty), (stock.SHORT, 6))
        self.assertIn("enough for 2 board(s)", stock.judge(n, [cap]).detail)

    def test_no_stock_key(self):
        line = stock.judge(need(["A1"], "DRV8825 left", None), [item(CAP)])
        self.assertEqual(line.status, stock.NO_KEY)


class GroupTests(unittest.TestCase):
    def test_same_entry_and_value_share_a_line(self):
        needs = stock.group(
            [
                ("R1", "r0805", "10k", R0805, "C17414", "aliexpress:1"),
                ("R2", "r0805", "100k", R0805, "C149504", "aliexpress:1"),
                ("R4", "r0805", "10k", R0805, "C17414", "aliexpress:1"),
            ]
        )
        self.assertEqual(
            [(n.refs, n.value) for n in needs],
            [(("R1", "R4"), "10k"), (("R2",), "100k")],
        )
        self.assertEqual(needs[0].count, 2)
        self.assertEqual(needs[0].lcsc, "C17414")


class InventoryTests(unittest.TestCase):
    def test_loads_the_items_of_a_tree(self):
        root = write_tree(
            self,
            {
                "items/capacitor/c0805-100n.toml": CAP,
                "items/capacitor/kit.toml": KIT,
                "items/transistor/zxmp3a16g.toml": MOSFET,
            },
        )
        items = stock.load_items(root)
        self.assertEqual([i.slug for i in items], ["c0805-100n", "kit", "zxmp3a16g"])
        self.assertEqual(items[1].contents, ("100nF", "1uF"))
        self.assertEqual(items[1].unknown, ("voltage", "dielectric"))

    def test_a_missing_inventory_is_a_clear_failure(self):
        missing = Path(tempfile.gettempdir()) / "no-such-parts-stock-xyz"
        with self.assertRaisesRegex(KicadError, "no inventory there"):
            stock.load_items(missing)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = stock.main(
                ["--inventory", str(missing), "packages/robotics/balancebot"]
            )
        self.assertEqual(code, 1)
        self.assertIn("FAIL:", out.getvalue())

    def test_a_bad_item_names_its_file(self):
        root = write_tree(
            self, {"items/capacitor/bad.toml": CAP.replace("schema = 1", "schema = 2")}
        )
        with self.assertRaisesRegex(KicadError, "bad.toml: schema 2"):
            stock.load_items(root)

    def test_inventory_dir_env_wins_over_the_default(self):
        with mock.patch.dict(os.environ, {"INVENTORY_DIR": "/somewhere"}):
            self.assertEqual(stock.default_inventory(), Path("/somewhere"))
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop("INVENTORY_DIR", None)
            self.assertEqual(stock.default_inventory().name, "parts-stock")


def order(status: str, oid: str) -> str:
    return (
        f'schema = 1\n[order]\nshop = "aliexpress"\nid = "{oid}"\n'
        f'date = "2026-10-01"\nstatus = "{status}"\n'
        '[[lines]]\ntitle = "x"\nqty = 1\n'
    )


ZXMP4_ORDERED = """schema = 1
[item]
name = "ZXMP4A16G P-MOSFET"
category = "transistor"
kind = "part"
package = "SOT-223"
mpn = "ZXMP4A16GTA"
[stock]
mode = "count"
qty = {qty}
[[sources]]
order = "aliexpress/2026-10-01-1"
qty = 10
[[sources]]
order = "aliexpress/2026-10-01-2"
qty = 20
[[sources]]
order = "aliexpress/2026-10-01-3"
qty = 40
[[sources]]
order = "aliexpress/2026-10-01-4"
qty = 80
"""


class IncomingTests(unittest.TestCase):
    """Pieces on placed or shipped orders are shown, never counted (inv/store.py)."""

    def tree(self, qty: int, statuses=("placed", "shipped", "delivered", "cancelled")):
        files = {
            f"orders/aliexpress/2026-10-01-{n}.toml": order(status, str(n))
            for n, status in enumerate(statuses, start=1)
        }
        files["items/transistor/zxmp4a16g.toml"] = ZXMP4_ORDERED.format(qty=qty)
        return stock.load_items(write_tree(self, files))

    def test_only_placed_and_shipped_orders_are_incoming(self):
        (item_,) = self.tree(0)
        self.assertEqual(item_.incoming, 30)  # 10 placed + 20 shipped
        self.assertEqual(item_.stock_text, "x0 +30 incoming")

    def test_incoming_alone_stays_short(self):
        line = stock.judge(need(["Q1"], "ZXMP4A16G", ZXMP4), self.tree(0), boards=2)
        self.assertEqual(line.status, stock.SHORT)
        self.assertEqual(
            line.detail,
            "x0 on hand, +30 incoming (enough for 30 board(s) once delivered)",
        )
        self.assertIn("route:", stock.render([line]))

    def test_incoming_is_shown_beside_stock_on_hand(self):
        line = stock.judge(need(["Q1"], "ZXMP4A16G", ZXMP4), self.tree(5))
        self.assertEqual(line.status, stock.ON_HAND)
        self.assertEqual(
            line.detail,
            "x5, enough for 5 board(s), "
            "+30 incoming (enough for 35 board(s) once delivered)",
        )

    def test_no_undelivered_order_is_not_incoming(self):
        (item_,) = self.tree(0, ("delivered", "refunded", "cancelled", "unknown"))
        self.assertIsNone(item_.incoming)
        line = stock.judge(need(["Q1"], "ZXMP4A16G", ZXMP4), [item_])
        self.assertEqual((line.status, line.detail), (stock.SHORT, "x0 on hand"))

    def test_an_undelivered_source_without_qty(self):
        out_pack = item(
            'schema = 1\n[item]\nname = "r"\ncategory = "resistor"\nkind = "part"\n'
            'package = "0805"\n[stock]\nmode = "level"\nlevel = "out"\n'
            '[specs]\nvalue = "10k"\n[[sources]]\norder = "aliexpress/2026-10-01-1"\n'
        )
        self.assertIsNone(out_pack.incoming)  # parsed without the orders
        with_orders = stock.parse_item(
            Path("items/resistor/r.toml"),
            tomllib.loads(
                'schema = 1\n[item]\nname = "r"\ncategory = "resistor"\n'
                'kind = "part"\npackage = "0805"\n[stock]\nmode = "level"\n'
                'level = "out"\n[specs]\nvalue = "10k"\n[[sources]]\n'
                'order = "aliexpress/2026-10-01-1"\n'
            ),
            {"aliexpress/2026-10-01-1": "shipped"},
        )
        line = stock.judge(need(["R1"], "10k", R0805, slug="r0805"), [with_orders])
        self.assertEqual(line.status, stock.SHORT)
        self.assertEqual(line.detail, "level out, +incoming (quantity not stated)")

    def test_a_bad_order_status_names_its_file(self):
        files = {
            "orders/aliexpress/2026-10-01-1.toml": order("lost", "1"),
            "items/transistor/zxmp4a16g.toml": ZXMP4_ORDERED.format(qty=0),
        }
        with self.assertRaisesRegex(KicadError, "2026-10-01-1.toml.*'lost'"):
            stock.load_items(write_tree(self, files))


ENTRY = """\
[part]
name = "x"
source = "test"
symbol = "Device:C"
footprint = "Capacitor_SMD:C_0805_2012Metric"

[stock]
{stock}

[pins]
A = 1
B = 2
"""


class RegistryStockTests(unittest.TestCase):
    def parse(self, body: str) -> registry.PartSpec:
        text = ENTRY.format(stock=body)
        return registry.parse(tomllib.loads(text), "x", Path("x.toml"))

    def test_each_form_parses(self):
        self.assertEqual(
            self.parse('mpn = "B5819W"').stock, StockSpec("B5819W", "", "", ())
        )
        spec = self.parse(
            'category = "capacitor"\npackage = "0805"\nneeds = ["voltage"]'
        )
        self.assertEqual(spec.stock, StockSpec("", "capacitor", "0805", ("voltage",)))

    def test_no_table_is_none(self):
        text = ENTRY.replace("[stock]\n{stock}\n", "")
        self.assertIsNone(
            registry.parse(tomllib.loads(text), "x", Path("x.toml")).stock
        )

    def test_bad_tables_fail(self):
        for body, msg in (
            (
                'mpn = "B5819W"\ncategory = "diode"\npackage = "SOD-123"',
                "not both forms",
            ),
            ('mpn = "B5819W"\npackage = "SOD-123"', "not both forms"),
            ('needs = ["voltage"]', "needs mpn, or both category and package"),
            ('category = "capacitor"', "needs mpn, or both category and package"),
            ('category = "fuse"\npackage = "1812"', "category 'fuse'"),
            ('mpn = "B5819W"\ncolour = "red"', "unknown key"),
            ('mpn = "B5819W"\nneeds = "voltage"', "needs must be a list"),
            ('mpn = "B5819W"\nneeds = ["v", "v"]', "twice"),
        ):
            with self.subTest(body=body), self.assertRaisesRegex(KicadError, msg):
                self.parse(body)

    def test_categories_match_the_inventory_schema(self):
        # A copy of parts-stock/inv/schema.py CATEGORIES (schema v1); this pins
        # the copy's shape, not the other repo.
        self.assertEqual(len(registry.STOCK_CATEGORIES), 14)
        self.assertIn("other", registry.STOCK_CATEGORIES)

    def test_the_balancebot_stock_keys(self):
        self.assertEqual(registry.load("zxmp4a16g").stock.mpn, "ZXMP4A16GTA")
        self.assertEqual(registry.load("tps563208-sot23-6").stock.mpn, "TPS563208DDCR")
        c = registry.load("c0805").stock
        self.assertEqual(
            (c.category, c.package, c.needs),
            ("capacitor", "0805", ("voltage", "dielectric")),
        )
        self.assertEqual(registry.load("polyfuse-1812").stock.category, "other")


class CarrierTests(unittest.TestCase):
    """The report over the balancebot carrier, against a synthetic inventory."""

    def test_balancebot_report(self):
        root = write_tree(
            self,
            {
                "items/capacitor/c0805-100n-50v-x7r.toml": CAP,
                "items/capacitor/c0805-kit-ibuw.toml": KIT,
                "items/transistor/zxmp3a16g.toml": MOSFET,
            },
        )
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = stock.main(
                ["--inventory", str(root), "packages/robotics/balancebot"]
            )
        text = out.getvalue()
        self.assertEqual(code, 0, text)
        sections = {}
        current = None
        for line in text.splitlines():
            if line.startswith("  ") and not line.startswith("    "):
                current = line.strip().split(":")[0]
                sections[current] = []
            elif current and line.startswith("    ") and not line.startswith("      "):
                sections[current].append(line.split()[0])
        self.assertIn("C3", sections[stock.ON_HAND])
        self.assertIn("Q1", sections[stock.NONE])
        self.assertIn("near match, not counted: zxmp3a16g", text)
        self.assertNotIn("H1", text)  # mounting holes are not in the BOM


if __name__ == "__main__":
    unittest.main()
