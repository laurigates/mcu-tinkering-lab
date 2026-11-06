"""Tests for the JLCPCB part lookup, kicad.lcsc.

Offline: every response is a recorded JLCPCB answer from testdata/lcsc/
(trimmed to the fields kicad.lcsc reads and a few neighbours), served by a
fake opener that fails the test on any request it was not given. The clock
is injected too, so cache freshness is exact.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import contextlib
import copy
import http.client
import io
import json
import os
import tempfile
import unittest
import urllib.request
from datetime import UTC, datetime, timedelta
from pathlib import Path
from unittest import mock

from kicad import lcsc
from kicad.errors import KicadError

DATA = Path(__file__).parent / "testdata/lcsc"
NOW = datetime(2026, 11, 6, 12, 0, tzinfo=UTC)
DAY = timedelta(hours=24)


def recorded(name: str) -> dict:
    return json.loads((DATA / f"{name}.json").read_text(encoding="utf-8"))


class FakeJlc:
    """An opener answering from recorded payloads: {lcsc: (detail, search)}.

    Counts requests, and raises `error` (OSError by default: the network or
    JLCPCB failing) for every number in `down`.
    """

    def __init__(
        self,
        answers: dict[str, tuple[dict, dict]],
        down=(),
        error: Exception | None = None,
    ):
        self.answers = answers
        self.down = set(down)
        self.error = error or OSError("connection refused")
        self.requests: list[str] = []

    def __call__(self, req: urllib.request.Request) -> bytes:
        if req.data is None:
            code = req.full_url.rsplit("=", 1)[1]
            kind = 0
        else:
            code = json.loads(req.data)["keyword"]
            kind = 1
        self.requests.append(f"{'search' if kind else 'detail'} {code}")
        # Never a personal header: only the User-Agent the API needs.
        assert req.get_header("User-agent") == lcsc.USER_AGENT
        assert not any("mail" in h.lower() for h, _ in req.header_items())
        if code in self.down:
            raise self.error
        return json.dumps(self.answers[code][kind]).encode()


def answers(*codes: str) -> dict[str, tuple[dict, dict]]:
    return {c: (recorded(f"detail-{c}"), recorded(f"search-{c}")) for c in codes}


UNKNOWN = "C999999999"


def with_unknown(table: dict) -> dict:
    table[UNKNOWN] = (recorded("detail-unknown"), recorded("search-unknown"))
    return table


def out_of_stock(code: str) -> tuple[dict, dict]:
    detail, search = copy.deepcopy(answers(code)[code])
    detail["data"]["stockCount"] = 0
    detail["data"]["overseasStockCount"] = 0
    for item in search["data"]["componentPageInfo"]["list"]:
        if item["componentCode"] == code:
            item["stockCount"] = 0
    return detail, search


class TempCache(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.cache = Path(tmp.name) / "lcsc"

    def lookup(self, code, jlc, now=NOW, **kw):
        kw.setdefault("max_age", DAY)
        with contextlib.redirect_stderr(io.StringIO()) as err:
            got = lcsc.lookup(code, cache=self.cache, opener=jlc, now=lambda: now, **kw)
        return got, err.getvalue()


class ParseTests(unittest.TestCase):
    def test_detail_fields(self):
        part = lcsc.part_from(
            "C468250",
            lcsc.trim_detail(recorded("detail-C468250")["data"]),
            lcsc.trim_search(recorded("search-C468250"), "C468250"),
        )
        self.assertEqual(part.mpn, "TPS563208DDCR")
        self.assertEqual(part.package, "TSOT-23-6")
        self.assertEqual(part.library_type, "expand")
        self.assertEqual(part.status, "yes")
        self.assertEqual(part.stock, 17262)
        self.assertEqual(part.price_1, 0.1299)
        self.assertEqual(part.min_qty, 5)
        self.assertIs(part.preferred, False)
        self.assertEqual(part.klass, "extended")

    def test_basic(self):
        part = lcsc.part_from(
            "C17414",
            lcsc.trim_detail(recorded("detail-C17414")["data"]),
            lcsc.trim_search(recorded("search-C17414"), "C17414"),
        )
        self.assertEqual(part.klass, "basic")
        self.assertEqual(part.min_qty, 20)

    def test_preferred(self):
        part = lcsc.part_from(
            "C17756",
            lcsc.trim_detail(recorded("detail-C17756")["data"]),
            lcsc.trim_search(recorded("search-C17756"), "C17756"),
        )
        self.assertIs(part.preferred, True)
        self.assertEqual(part.klass, "preferred")

    def test_search_matches_the_exact_code_only(self):
        # The search is fuzzy: C17414's second hit is C20622501, whose MPN is
        # "C17414", and an unknown number still lists 41 unrelated parts.
        search = recorded("search-C17414")
        hits = search["data"]["componentPageInfo"]["list"]
        hits.reverse()
        self.assertEqual(lcsc.trim_search(search, "C17414")["stockCount"], 50219315)
        self.assertIsNone(lcsc.trim_search(recorded("search-unknown"), UNKNOWN))

    def test_price_at_1_reads_the_tier_not_initial_price(self):
        # Recorded: C9900140859's qty-1 tier is 0.0398, its initialPrice
        # 0.0205 (the 100+ tier); the in-use fixtures have the two equal.
        detail = lcsc.trim_detail(recorded("detail-C9900140859")["data"])
        self.assertEqual(detail["initialPrice"], 0.0205)
        self.assertEqual(lcsc.part_from("C9900140859", detail, None).price_1, 0.0398)
        detail = lcsc.trim_detail(recorded("detail-C468250")["data"])
        del detail["initialPrice"]
        self.assertEqual(lcsc.part_from("C468250", detail, None).price_1, 0.1299)

    def test_price_at_1_falls_back_to_initial_price(self):
        detail = lcsc.trim_detail(recorded("detail-C468250")["data"])
        detail["prices"] = [{"startNumber": 5, "endNumber": -1, "productPrice": 0.2}]
        detail["initialPrice"] = 0.3
        self.assertEqual(lcsc.part_from("C468250", detail, None).price_1, 0.3)

    def test_discontinued(self):
        # Recorded 2026-11-06: componentStatus "yes" on a part JLCPCB marks
        # isBuyComponent "0", "This product is no longer manufactured."
        detail = lcsc.trim_detail(recorded("detail-C9900140859")["data"])
        part = lcsc.part_from("C9900140859", detail, None)
        self.assertEqual(part.status, "yes")
        self.assertFalse(part.buyable)
        self.assertEqual(part.no_buy_reason, "This product is no longer manufactured.")
        ok = lcsc.part_from(
            "C17414", lcsc.trim_detail(recorded("detail-C17414")["data"]), None
        )
        self.assertTrue(ok.buyable)

    def test_unknown_library_type_is_extended(self):
        detail = lcsc.trim_detail(recorded("detail-C17414")["data"])
        detail["componentLibraryType"] = "mystery"
        part = lcsc.part_from("C17414", detail, None)
        self.assertEqual(part.klass, "extended")
        self.assertEqual(part.library_type, "mystery")

    def test_stock_is_the_smaller_pool(self):
        detail = lcsc.trim_detail(recorded("detail-C468250")["data"])
        detail["stockCount"] = 109466
        search = lcsc.trim_search(recorded("search-C468250"), "C468250")
        part = lcsc.part_from("C468250", detail, search)
        self.assertEqual(part.stock, 17262)
        self.assertEqual((part.detail_stock, part.search_stock), (109466, 17262))


class FetchTests(TempCache):
    def test_unknown_part(self):
        jlc = FakeJlc(with_unknown({}))
        record = lcsc.fetch(UNKNOWN, jlc)
        self.assertIsNone(record["detail"])

    def test_bad_answer_is_a_fetch_error(self):
        jlc = FakeJlc({"C1": ({"code": 500, "data": None, "message": "busy"}, {})})
        with self.assertRaisesRegex(KicadError, "C1: .*500"):
            lcsc.fetch("C1", jlc)

    def test_network_error_is_a_kicad_error(self):
        jlc = FakeJlc(answers("C17414"), down={"C17414"})
        with self.assertRaisesRegex(KicadError, "C17414: .*connection refused"):
            lcsc.fetch("C17414", jlc)

    def test_truncated_body_is_a_kicad_error(self):
        # http.client.IncompleteRead is an HTTPException, not an OSError.
        jlc = FakeJlc(
            answers("C17414"),
            down={"C17414"},
            error=http.client.IncompleteRead(b"abc", 100),
        )
        with self.assertRaisesRegex(KicadError, "C17414: .*lookup failed"):
            lcsc.fetch("C17414", jlc)

    def test_cache_dir_follows_xdg(self):
        with mock.patch.dict(os.environ, {"XDG_CACHE_HOME": "/x/cache"}):
            self.assertEqual(lcsc.cache_dir(), Path("/x/cache/mcu-tinkering-lab/lcsc"))
        env = {k: v for k, v in os.environ.items() if k != "XDG_CACHE_HOME"}
        with mock.patch.dict(os.environ, env, clear=True):
            self.assertEqual(
                lcsc.cache_dir(), Path.home() / ".cache/mcu-tinkering-lab/lcsc"
            )


class CacheTests(TempCache):
    def test_fetches_then_caches(self):
        jlc = FakeJlc(answers("C17414"))
        got, _ = self.lookup("C17414", jlc)
        self.assertFalse(got.stale)
        self.assertEqual(len(jlc.requests), 2)
        self.assertEqual(sorted(p.name for p in self.cache.iterdir()), ["C17414.json"])
        cached = json.loads((self.cache / "C17414.json").read_text())
        self.assertEqual(cached["fetched_at"], NOW.isoformat())
        self.assertEqual(cached["detail"]["componentCode"], "C17414")
        self.assertIs(cached["search"]["preferredComponentFlag"], False)

    def test_fresh_cache_makes_no_request(self):
        self.lookup("C17414", FakeJlc(answers("C17414")))
        jlc = FakeJlc({})
        got, _ = self.lookup("C17414", jlc, now=NOW + DAY - timedelta(minutes=1))
        self.assertEqual(jlc.requests, [])
        self.assertEqual(got.part.mpn, "0805W8F1002T5E")
        self.assertFalse(got.stale)

    def test_expired_cache_is_refetched(self):
        self.lookup("C17414", FakeJlc(answers("C17414")))
        jlc = FakeJlc(answers("C17414"))
        later = NOW + DAY + timedelta(minutes=1)
        got, _ = self.lookup("C17414", jlc, now=later)
        self.assertEqual(len(jlc.requests), 2)
        self.assertEqual(got.fetched_at, later)

    def test_refresh_always_fetches(self):
        self.lookup("C17414", FakeJlc(answers("C17414")))
        jlc = FakeJlc(answers("C17414"))
        self.lookup("C17414", jlc, refresh=True)
        self.assertEqual(len(jlc.requests), 2)

    def test_stale_fallback_on_fetch_error(self):
        self.lookup("C17414", FakeJlc(answers("C17414")))
        jlc = FakeJlc(answers("C17414"), down={"C17414"})
        got, err = self.lookup("C17414", jlc, now=NOW + 3 * DAY)
        self.assertTrue(got.stale)
        self.assertEqual(got.fetched_at, NOW)
        self.assertEqual(got.part.stock, 50219315)
        self.assertIn("WARN: C17414", err)
        self.assertIn("connection refused", err)

    def test_stale_fallback_on_truncated_body(self):
        self.lookup("C17414", FakeJlc(answers("C17414")))
        jlc = FakeJlc(
            answers("C17414"),
            down={"C17414"},
            error=http.client.IncompleteRead(b"abc", 100),
        )
        got, err = self.lookup("C17414", jlc, now=NOW + 3 * DAY)
        self.assertTrue(got.stale)
        self.assertIn("WARN: C17414", err)

    def test_unwritable_cache_still_answers(self):
        # The cache directory's parent is a file: the write fails, the
        # fresh answer is still returned, with a warning.
        self.cache.parent.mkdir(parents=True, exist_ok=True)
        blocker = self.cache.parent / "file"
        blocker.write_text("x")
        self.cache = blocker / "lcsc"
        got, err = self.lookup("C17414", FakeJlc(answers("C17414")))
        self.assertEqual(got.part.klass, "basic")
        self.assertFalse(got.stale)
        self.assertIn("WARN: C17414: could not cache", err)

    def test_fetch_error_without_cache_fails(self):
        jlc = FakeJlc(answers("C17414"), down={"C17414"})
        with self.assertRaisesRegex(KicadError, "C17414: .*no cached copy"):
            self.lookup("C17414", jlc)

    def test_offline_uses_cache_only(self):
        self.lookup("C17414", FakeJlc(answers("C17414")))
        jlc = FakeJlc({})
        got, _ = self.lookup("C17414", jlc, offline=True, now=NOW + 3 * DAY)
        self.assertEqual(jlc.requests, [])
        self.assertTrue(got.stale)
        with self.assertRaisesRegex(KicadError, "C468250: .*--offline"):
            self.lookup("C468250", jlc, offline=True)
        self.assertEqual(jlc.requests, [])

    def test_unknown_part_is_cached_too(self):
        jlc = FakeJlc(with_unknown({}))
        got, _ = self.lookup(UNKNOWN, jlc)
        self.assertIsNone(got.part)
        got, _ = self.lookup(UNKNOWN, FakeJlc({}), offline=True)
        self.assertIsNone(got.part)

    def test_atomic_write_leaves_no_tmp_file(self):
        lcsc.write_cache(self.cache / "C1.json", {"fetched_at": "x"})
        lcsc.write_cache(self.cache / "C1.json", {"fetched_at": "y"})
        self.assertEqual([p.name for p in self.cache.iterdir()], ["C1.json"])
        self.assertEqual(
            json.loads((self.cache / "C1.json").read_text())["fetched_at"], "y"
        )

    def test_failed_write_keeps_the_old_file_and_no_tmp(self):
        path = self.cache / "C1.json"
        lcsc.write_cache(path, {"fetched_at": "old"})
        with self.assertRaises(TypeError):
            lcsc.write_cache(path, {"fetched_at": object()})
        self.assertEqual([p.name for p in self.cache.iterdir()], ["C1.json"])
        self.assertEqual(json.loads(path.read_text())["fetched_at"], "old")

    def test_corrupt_cache_is_refetched(self):
        self.cache.mkdir(parents=True)
        (self.cache / "C17414.json").write_text("{not json")
        jlc = FakeJlc(answers("C17414"))
        got, _ = self.lookup("C17414", jlc)
        self.assertEqual(len(jlc.requests), 2)
        self.assertEqual(got.part.klass, "basic")


PART_TOML = """\
[part]
name = "{slug}"
source = "test"
symbol = "Device:R"
footprint = "Resistor_SMD:{slug}"
{lcsc}
[pins]
"""


class UsedTests(unittest.TestCase):
    """`used` over a temp repo: two registry entries and two projects."""

    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        parts = self.root / "docs/reference/parts"
        parts.mkdir(parents=True)
        for slug, code in (("buck", "C468250"), ("r0805", ""), ("spare", "C8598")):
            lc = f'lcsc = "{code}"' if code else ""
            (parts / f"{slug}.toml").write_text(PART_TOML.format(slug=slug, lcsc=lc))
        a = self.root / "packages/dom/a"
        b = self.root / "packages/dom/b"
        a.mkdir(parents=True)
        b.mkdir(parents=True)
        (a / "hardware.toml").write_text(
            '[source]\npart = "r0805"\n'
            '[parts.reg]\npart = "buck"\nref = "U1"\n'
            '[parts.pullup]\npart = "r0805"\nref = "R1"\nlcsc = "C17414"\n'
            '[parts.motor]\nname = "no part"\n'
        )
        (b / "hardware.toml").write_text(
            '[parts.pullup]\npart = "r0805"\nref = "R1"\nlcsc = "C17414"\n'
            '[parts.reg]\npart = "buck"\nref = "U2"\nlcsc = "C6186"\n'
        )
        self.a, self.b = a, b

    def labels(self, found):
        return {c: sorted(u.label for u in uses) for c, uses in found.items()}

    def test_registry_and_every_project(self):
        found = lcsc.used(repo_root=self.root)
        self.assertEqual(
            self.labels(found),
            {
                "C468250": ["dom/a [parts.reg]", "parts/buck"],
                "C8598": ["parts/spare"],
                "C17414": ["dom/a [parts.pullup]", "dom/b [parts.pullup]"],
                # the part's own lcsc overrides the registry entry's
                "C6186": ["dom/b [parts.reg]"],
            },
        )
        self.assertEqual({u.footprint for u in found["C468250"]}, {"Resistor_SMD:buck"})

    def test_project_narrows(self):
        found = lcsc.used([self.a], repo_root=self.root)
        self.assertEqual(
            self.labels(found),
            {
                "C468250": ["dom/a [parts.reg]"],
                "C17414": ["dom/a [parts.pullup]"],
            },
        )

    def test_project_without_hardware_toml_fails(self):
        with self.assertRaisesRegex(KicadError, "hardware.toml"):
            lcsc.used([self.root / "packages/dom/none"], repo_root=self.root)

    def test_repo_uses_include_the_carriers(self):
        found = lcsc.used()
        self.assertIn("C468250", found)  # registry: TPS563208
        self.assertIn("C17414", found)  # balancebot pull-up


class WhereTests(unittest.TestCase):
    def test_groups_a_projects_parts(self):
        labels = ["parts/buck", "dom/a [parts.reg]"] + [
            f"dom/b [parts.r{i}]" for i in range(4)
        ]
        self.assertEqual(
            lcsc._where(labels), "parts/buck, dom/a [reg], dom/b [4 parts]"
        )
        self.assertEqual(lcsc._where([lcsc.COMMAND_LINE]), lcsc.COMMAND_LINE)


class CliTests(TempCache):
    """`main` with the fake opener, on numbers given on the command line."""

    def run_main(self, argv, jlc, now=NOW):
        with (
            mock.patch.dict(os.environ, {"XDG_CACHE_HOME": str(self.cache.parent)}),
            contextlib.redirect_stdout(io.StringIO()) as out,
            contextlib.redirect_stderr(io.StringIO()) as err,
        ):
            code = lcsc.main(argv, opener=jlc, now=lambda: now)
        return code, out.getvalue(), err.getvalue()

    def test_all_good_with_an_extended_part_passes(self):
        jlc = FakeJlc(answers("C17414", "C17756", "C468250"))
        code, out, _ = self.run_main(["C17414", "C17756", "C468250"], jlc)
        self.assertEqual(code, 0, out)
        self.assertIn("TPS563208DDCR", out)
        self.assertIn("WARN: C468250: extended", out)
        self.assertIn("3 part(s): 1 basic, 1 preferred, 1 extended", out)
        self.assertNotIn("FAIL", out)

    def test_unknown_part_fails(self):
        jlc = FakeJlc(with_unknown(answers("C17414")))
        code, out, _ = self.run_main(["C17414", UNKNOWN], jlc)
        self.assertEqual(code, 1)
        self.assertIn(f"FAIL: {UNKNOWN}: unknown to JLCPCB", out)

    def test_out_of_stock_fails(self):
        jlc = FakeJlc({"C468250": out_of_stock("C468250")})
        code, out, _ = self.run_main(["C468250"], jlc)
        self.assertEqual(code, 1)
        self.assertIn("FAIL: C468250: no assembly stock", out)

    def test_not_orderable_fails(self):
        detail, search = answers("C17414")["C17414"]
        detail = copy.deepcopy(detail)
        detail["data"]["componentStatus"] = "no"
        code, out, _ = self.run_main(["C17414"], FakeJlc({"C17414": (detail, search)}))
        self.assertEqual(code, 1)
        self.assertIn("FAIL: C17414: not orderable (componentStatus 'no')", out)

    def test_discontinued_with_stock_fails(self):
        detail, search = copy.deepcopy(answers("C468250")["C468250"])
        detail["data"]["isBuyComponent"] = "0"
        detail["data"]["noBuyReason"] = "This product is no longer manufactured."
        code, out, _ = self.run_main(
            ["--json", "C468250"], FakeJlc({"C468250": (detail, search)})
        )
        self.assertEqual(code, 1, out)
        row = json.loads(out)["parts"][0]
        self.assertEqual(row["buyable"], False)
        self.assertEqual(
            row["no_buy_reason"], "This product is no longer manufactured."
        )
        self.assertIn("not orderable", " ".join(row["failures"]))
        self.assertIn("no longer manufactured", " ".join(row["failures"]))

    def test_discontinued_text_row(self):
        jlc = FakeJlc(answers("C9900140859"))
        code, out, _ = self.run_main(["C9900140859"], jlc)
        self.assertEqual(code, 1)
        self.assertIn(
            "FAIL: C9900140859: not orderable: This product is no longer manufactured.",
            out,
        )
        self.assertIn("not buyable", out)

    def test_unfetchable_without_cache_fails(self):
        jlc = FakeJlc(answers("C17414"), down={"C17414"})
        code, out, _ = self.run_main(["C17414"], jlc)
        self.assertEqual(code, 1)
        self.assertIn("FAIL: C17414: ", out)

    def test_stale_row_is_marked(self):
        self.run_main(["C17414"], FakeJlc(answers("C17414")))
        jlc = FakeJlc(answers("C17414"), down={"C17414"})
        code, out, err = self.run_main(["C17414"], jlc, now=NOW + 3 * DAY)
        self.assertEqual(code, 0, out)
        self.assertIn("stale", out)
        self.assertIn("WARN: C17414", err)

    def test_json(self):
        jlc = FakeJlc(answers("C17756", "C468250"))
        code, out, _ = self.run_main(["--json", "C17756", "C468250"], jlc)
        self.assertEqual(code, 0)
        data = json.loads(out)
        self.assertEqual(data["summary"], {"basic": 0, "preferred": 1, "extended": 1})
        rows = {r["lcsc"]: r for r in data["parts"]}
        self.assertEqual(rows["C468250"]["class"], "extended")
        self.assertEqual(rows["C468250"]["stock"], 17262)
        self.assertEqual(rows["C17756"]["used_in"], ["(command line)"])
        self.assertTrue(data["ok"])

    def test_bad_number_fails(self):
        code, out, _ = self.run_main(["R17414"], FakeJlc({}))
        self.assertEqual(code, 1)
        self.assertIn("FAIL: ", out)

    def test_project(self):
        project = self.cache.parent / "packages/dom/p"
        project.mkdir(parents=True)
        (project / "hardware.toml").write_text(
            '[parts.pullup]\npart = "r0805"\nref = "R1"\nlcsc = "C17414"\n'
        )
        code, out, _ = self.run_main(
            ["--project", str(project)], FakeJlc(answers("C17414"))
        )
        self.assertEqual(code, 0, out)
        self.assertIn("1 part(s): 1 basic, 0 preferred, 0 extended", out)


if __name__ == "__main__":
    unittest.main()
