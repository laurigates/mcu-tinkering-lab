"""Tests for jlc-blog.py, offline: the HTTP layer is replaced by a stub.

Stdlib unittest only, so the pre-commit hook runs it with nothing but python3:

    python3 -m unittest tools/test_jlc_blog.py
"""

from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).with_name("jlc-blog.py")
SPEC = importlib.util.spec_from_file_location("jlc_blog", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
JB = importlib.util.module_from_spec(SPEC)
sys.modules["jlc_blog"] = JB
SPEC.loader.exec_module(JB)


class StubApi:
    """Answers the three endpoints from canned data and records each call."""

    def __init__(self, posts: list[dict], tree: dict, details: dict[str, dict]):
        self.posts = posts
        self.tree = tree
        self.details = details
        self.calls: list[tuple[str, dict]] = []

    def __call__(self, path: str, params: dict | None = None, body: dict | None = None):
        self.calls.append((path, body if body is not None else params or {}))
        if path == "/newsBlog/selectNewsBlogList":
            start = (body["pageNum"] - 1) * body["pageSize"]
            page = self.posts[start : start + body["pageSize"]]
            return {"total": len(self.posts), "list": page}
        if path == "/categoryManage/tree":
            return self.tree
        if path == "/newsBlog/queryNewsBlogDetail":
            return self.details[params["locationUri"]]
        raise AssertionError(f"unexpected path {path}")


TREE = {
    "parent": [
        {
            "cmsCategoryKeyId": 1,
            "categoryName": "PCB Manufacturing",
            "categoryUrl": "pcb-manufacturing",
            "children": [
                {
                    "cmsCategoryKeyId": 11,
                    "categoryName": "Design for Manufacturing",
                    "categoryUrl": "dfm",
                    "children": [],
                }
            ],
        }
    ]
}


def post(n: int, cat: int = 11) -> dict:
    return {
        "locationUri": f"post-{n}",
        "title": f"Post {n}",
        "cmsCategoryKeyId": cat,
        "publishFirstTime": 1741915105000,
        "updateTime": 1789020346000,
        "tdkKeywords": "DFM, PCB",
        "previewContent": "preview",
        "readTime": 449,
    }


class HtmlToMarkdown(unittest.TestCase):
    def test_headings_paragraphs_and_lists(self):
        md = JB.html_to_markdown(
            "<h2>Why DFM</h2><p>Catch <strong>issues</strong> early.</p>"
            "<ul><li>Annular ring</li><li>Solder mask sliver</li></ul>"
            "<ol><li>First</li><li>Second</li></ol>"
        )
        self.assertIn("## Why DFM", md)
        self.assertIn("Catch **issues** early.", md)
        self.assertIn("- Annular ring\n- Solder mask sliver", md)
        self.assertIn("1. First\n2. Second", md)

    def test_table_becomes_pipe_rows_with_header_rule(self):
        md = JB.html_to_markdown(
            "<table><tr><th>Item</th><th>Min</th></tr>"
            "<tr><td>Trace</td><td>0.1 mm</td></tr></table>"
        )
        self.assertIn("| Item | Min |\n| --- | --- |\n| Trace | 0.1 mm |", md)

    def test_links_keep_target_and_scripts_are_dropped(self):
        md = JB.html_to_markdown(
            '<p>See <a href="https://jlcpcb.com/capabilities">capabilities</a>.</p>'
            "<script>var x=1;</script><style>p{}</style>"
        )
        self.assertIn("[capabilities](https://jlcpcb.com/capabilities)", md)
        self.assertNotIn("var x", md)
        self.assertNotIn("p{}", md)

    def test_images_keep_alt_and_source(self):
        md = JB.html_to_markdown('<img src="https://rs.jlcpcb.com/a.png" alt="Pad">')
        self.assertIn("![Pad](https://rs.jlcpcb.com/a.png)", md)

    def test_whitespace_collapses_and_no_triple_blank_lines(self):
        md = JB.html_to_markdown("<p>a\n   b</p><p></p><p></p><p>c</p>")
        self.assertIn("a b", md)
        self.assertNotIn("\n\n\n", md)


class Index(unittest.TestCase):
    def test_paginates_until_total_and_names_categories(self):
        api = StubApi([post(n) for n in range(5)], TREE, {})
        rows = JB.build_index(api, site="pcb", page_size=2)
        self.assertEqual([r["slug"] for r in rows], [f"post-{n}" for n in range(5)])
        pages = [b["pageNum"] for p, b in api.calls if p.endswith("selectNewsBlogList")]
        self.assertEqual(pages, [1, 2, 3])
        self.assertEqual(rows[0]["category"], "Design for Manufacturing")
        self.assertEqual(
            rows[0]["category_path"], ["PCB Manufacturing", "Design for Manufacturing"]
        )
        self.assertEqual(rows[0]["url"], "https://jlcpcb.com/blog/post-0")
        self.assertEqual(rows[0]["published"], "2025-03-14")

    def test_list_request_carries_site_use_type_and_language(self):
        api = StubApi([post(0)], TREE, {})
        JB.build_index(api, site="3dp", page_size=10)
        _, body = next(c for c in api.calls if c[0].endswith("selectNewsBlogList"))
        self.assertEqual(body["useType"], 1)
        self.assertEqual(body["languageCode"], "en")
        self.assertEqual(JB.SITES["3dp"].url("x"), "https://jlc3dp.com/blog/x")

    def test_unknown_category_id_is_kept_not_dropped(self):
        api = StubApi([post(0, cat=999)], TREE, {})
        (row,) = JB.build_index(api, site="pcb", page_size=10)
        self.assertEqual(row["category"], "")
        self.assertEqual(row["category_id"], 999)


class Search(unittest.TestCase):
    ROWS = [
        {
            "slug": "dfm-analysis",
            "title": "DFM Analysis",
            "category": "Design for Manufacturing",
            "keywords": "dfm",
            "preview": "",
        },
        {
            "slug": "pcb-color",
            "title": "Choosing PCB Color",
            "category": "Surface Coating",
            "keywords": "solder mask",
            "preview": "green",
        },
    ]

    def test_matches_title_category_keywords_case_insensitively(self):
        self.assertEqual(
            [r["slug"] for r in JB.search(self.ROWS, "solder MASK")], ["pcb-color"]
        )
        self.assertEqual(
            [r["slug"] for r in JB.search(self.ROWS, "manufactur")], ["dfm-analysis"]
        )

    def test_category_filter_uses_slug_or_name(self):
        rows = JB.search(self.ROWS, "", category="surface coating")
        self.assertEqual([r["slug"] for r in rows], ["pcb-color"])


class Fetch(unittest.TestCase):
    def test_writes_markdown_with_front_matter(self):
        api = StubApi(
            [],
            TREE,
            {
                "post-0": {
                    "title": "Post 0",
                    "locationUri": "post-0",
                    "categoryName": "Design for Manufacturing",
                    "publishFirstTime": 1741915105000,
                    "updateTime": 1789020346000,
                    "content": "<h2>Rules</h2><p>Keep 0.1 mm.</p>",
                }
            },
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = JB.fetch_post(api, "post-0", site="pcb", out_dir=Path(tmp))
            text = path.read_text()
        self.assertTrue(text.startswith("---\n"))
        meta = json.loads(text.split("\n")[1].removeprefix("meta: "))
        self.assertEqual(meta["url"], "https://jlcpcb.com/blog/post-0")
        self.assertEqual(meta["category"], "Design for Manufacturing")
        self.assertIn("# Post 0", text)
        self.assertIn("## Rules\n\nKeep 0.1 mm.", text)
        _, params = api.calls[-1]
        self.assertEqual(params["useType"], 0)

    def test_cached_file_is_not_refetched(self):
        api = StubApi([], TREE, {})
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "post-0.md").write_text("cached")
            JB.fetch_post(api, "post-0", site="pcb", out_dir=Path(tmp))
        self.assertEqual(api.calls, [])


if __name__ == "__main__":
    unittest.main()
