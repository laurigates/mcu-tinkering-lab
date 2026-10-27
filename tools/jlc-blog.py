#!/usr/bin/env python3
"""Index, search and fetch JLCPCB / JLC3DP / JLCCNC blog posts as Markdown.

The blogs are server-rendered, but the CMS behind them answers unauthenticated
JSON (found in the site's Nuxt bundles; robots.txt disallows crawling /api, so
keep the request rate low and cache what you fetch):

    POST /api/overseas-core-platform/newsBlog/selectNewsBlogList
         {"pageNum", "pageSize", "languageCode", "useType"}      -> {total, list}
    GET  /api/overseas-core-platform/newsBlog/queryNewsBlogDetail
         ?locationUri=<slug>&useType=<n>&languageCode=en          -> post with HTML `content`
    GET  /api/overseas-core-platform/categoryManage/tree
         ?businessType=2&useType=<n>&languageCode=en              -> {parent: [category tree]}

useType selects the site: 0 jlcpcb.com/blog, 1 jlc3dp.com/blog, 3 jlccnc.com/blog.
The sitemap lists only part of the posts (995 of 1101 on 2026-10-27).

Usage:
    jlc-blog.py index  [--site pcb] [-o index.jsonl]       # all post metadata, one JSON per line
    jlc-blog.py search PATTERN [--index F] [--category C]   # regex over title/category/keywords
    jlc-blog.py fetch  SLUG... [--site pcb] [--out DIR]     # DIR/<slug>.md, cached
"""

from __future__ import annotations

import argparse
import datetime as dt
import html
import json
import re
import sys
import time
import urllib.parse
import urllib.request
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from html.parser import HTMLParser
from pathlib import Path
from typing import Any

API = "https://jlcpcb.com/api/overseas-core-platform"
USER_AGENT = "mcu-tinkering-lab jlc-blog.py (research; low rate)"
REQUEST_GAP_S = 0.5


@dataclass(frozen=True)
class Site:
    use_type: int
    base: str

    def url(self, slug: str) -> str:
        return f"{self.base}/blog/{slug}"


SITES = {
    "pcb": Site(0, "https://jlcpcb.com"),
    "3dp": Site(1, "https://jlc3dp.com"),
    "cnc": Site(3, "https://jlccnc.com"),
}

Api = Callable[..., Any]


def http_api(path: str, params: dict | None = None, body: dict | None = None) -> Any:
    """Call one CMS endpoint and return its `data`, raising on a non-200 answer."""
    url = API + path
    if params:
        url += "?" + urllib.parse.urlencode(params)
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, headers={"User-Agent": USER_AGENT})
    if data is not None:
        req.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(req, timeout=30) as resp:
        payload = json.load(resp)
    time.sleep(REQUEST_GAP_S)
    if payload.get("code") != 200:
        raise RuntimeError(f"{path}: {payload.get('message')}")
    return payload["data"]


# --- HTML -> Markdown ---------------------------------------------------------

_BLOCK = {"p", "div", "section", "article", "blockquote", "figure", "figcaption"}
_SKIP = {"script", "style", "noscript", "svg", "iframe"}


class _Markdown(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.out: list[str] = []
        self.skip = 0
        self.lists: list[list] = []  # [kind, counter]
        self.href: list[str | None] = []
        self.row: list[str] | None = None
        self.cell: list[str] | None = None
        self.table: list[tuple[list[str], bool]] = []
        self.header_row = False

    def _emit(self, text: str) -> None:
        (self.cell if self.cell is not None else self.out).append(text)

    def _block(self) -> None:
        if self.cell is None:
            self.out.append("\n\n")

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        a = dict(attrs)
        if tag in _SKIP:
            self.skip += 1
        elif self.skip:
            return
        elif re.fullmatch(r"h[1-6]", tag):
            self._block()
            self._emit("#" * int(tag[1]) + " ")
        elif tag in _BLOCK:
            self._block()
        elif tag == "br":
            self._emit("\n")
        elif tag in ("ul", "ol"):
            self._block()
            self.lists.append([tag, 0])
        elif tag == "li":
            kind = self.lists[-1] if self.lists else ["ul", 0]
            kind[1] += 1
            indent = "  " * max(len(self.lists) - 1, 0)
            marker = f"{kind[1]}." if kind[0] == "ol" else "-"
            self._emit(f"\n{indent}{marker} ")
        elif tag in ("strong", "b"):
            self._emit("**")
        elif tag in ("em", "i"):
            self._emit("*")
        elif tag == "code":
            self._emit("`")
        elif tag == "a":
            self.href.append(a.get("href"))
            self._emit("[")
        elif tag == "img":
            src = a.get("src") or a.get("data-src")
            if src:
                self._emit(f"![{a.get('alt') or ''}]({src})")
        elif tag == "table":
            self._block()
            self.table = []
        elif tag == "tr":
            self.row = []
            self.header_row = False
        elif tag in ("td", "th"):
            self.cell = []
            self.header_row |= tag == "th"

    def handle_endtag(self, tag: str) -> None:
        if tag in _SKIP:
            self.skip = max(self.skip - 1, 0)
        elif self.skip:
            return
        elif re.fullmatch(r"h[1-6]", tag) or tag in _BLOCK:
            self._block()
        elif tag in ("ul", "ol"):
            if self.lists:
                self.lists.pop()
            self._block()
        elif tag in ("strong", "b"):
            self._emit("**")
        elif tag in ("em", "i"):
            self._emit("*")
        elif tag == "code":
            self._emit("`")
        elif tag == "a":
            href = self.href.pop() if self.href else None
            self._emit(f"]({href})" if href else "]")
        elif tag in ("td", "th") and self.cell is not None and self.row is not None:
            self.row.append(_squash("".join(self.cell)).replace("|", "\\|"))
            self.cell = None
        elif tag == "tr" and self.row is not None:
            self.table.append((self.row, self.header_row))
            self.row = None
        elif tag == "table":
            self.out.append(_render_table(self.table))
            self._block()
            self.table = []

    def handle_data(self, data: str) -> None:
        if not self.skip:
            # Source newlines are whitespace in HTML; only <br> and blocks break lines.
            self._emit(re.sub(r"\s+", " ", data))


def _squash(text: str) -> str:
    return re.sub(r"\s+", " ", text).strip()


def _render_table(rows: list[tuple[list[str], bool]]) -> str:
    if not rows:
        return ""
    lines = []
    for i, (cells, _) in enumerate(rows):
        lines.append("| " + " | ".join(cells) + " |")
        if i == 0:
            lines.append("| " + " | ".join("---" for _ in cells) + " |")
    return "\n".join(lines)


def html_to_markdown(source: str) -> str:
    parser = _Markdown()
    parser.feed(source)
    parser.close()
    text = "".join(parser.out)
    lines = []
    for line in text.split("\n"):
        if line.startswith("|"):
            lines.append(line)
        else:
            lead = (
                re.match(r"\s*", line).group(0) if re.match(r"\s*[-\d]", line) else ""
            )
            lines.append(lead + re.sub(r"[ \t\r\f\v]+", " ", line).strip())
    text = "\n".join(lines)
    text = re.sub(r"\*\*\s*\*\*", "", text)
    text = re.sub(r"\n{3,}", "\n\n", text)
    return html.unescape(text).strip() + "\n"


# --- index / search / fetch ---------------------------------------------------


def _date(ms: Any) -> str:
    if not ms:
        return ""
    return dt.datetime.fromtimestamp(int(ms) / 1000, dt.UTC).date().isoformat()


def category_paths(tree: dict) -> dict[int, list[str]]:
    """Map category id -> [parent name, ..., name] over the whole tree."""
    paths: dict[int, list[str]] = {}

    def walk(nodes: Iterable[dict], prefix: list[str]) -> None:
        for node in nodes:
            path = prefix + [node["categoryName"]]
            paths[int(node["cmsCategoryKeyId"])] = path
            walk(node.get("children") or [], path)

    walk(tree.get("parent") or [], [])
    return paths


def build_index(
    api: Api, site: str = "pcb", page_size: int = 100, language: str = "en"
) -> list[dict]:
    s = SITES[site]
    cats = category_paths(
        api(
            "/categoryManage/tree",
            params={"businessType": 2, "useType": s.use_type, "languageCode": language},
        )
    )
    rows: list[dict] = []
    page = 1
    while True:
        data = api(
            "/newsBlog/selectNewsBlogList",
            body={
                "pageNum": page,
                "pageSize": page_size,
                "languageCode": language,
                "useType": s.use_type,
            },
        )
        for p in data["list"]:
            cid = int(p.get("cmsCategoryKeyId") or 0)
            path = cats.get(cid, [])
            rows.append(
                {
                    "slug": p["locationUri"],
                    "title": p["title"],
                    "url": s.url(p["locationUri"]),
                    "category": path[-1] if path else "",
                    "category_path": path,
                    "category_id": cid,
                    "published": _date(p.get("publishFirstTime")),
                    "updated": _date(p.get("updateTime")),
                    "keywords": p.get("tdkKeywords") or "",
                    "preview": _squash(p.get("previewContent") or "")[:300],
                    "read_time_s": p.get("readTime"),
                }
            )
        if not data["list"] or page * page_size >= data["total"]:
            return rows
        page += 1


def search(
    rows: Iterable[dict], pattern: str, category: str | None = None
) -> list[dict]:
    rx = re.compile(pattern, re.IGNORECASE)
    hits = []
    for r in rows:
        if category:
            names = [
                c.lower() for c in r.get("category_path") or [r.get("category", "")]
            ]
            names.append((r.get("category") or "").lower())
            if category.lower() not in names:
                continue
        hay = " ".join(
            str(r.get(k, ""))
            for k in ("title", "category", "keywords", "preview", "slug")
        )
        if rx.search(hay):
            hits.append(r)
    return hits


def fetch_post(
    api: Api,
    slug: str,
    site: str = "pcb",
    out_dir: Path = Path("."),
    language: str = "en",
) -> Path:
    path = out_dir / f"{slug}.md"
    if path.exists():
        return path
    s = SITES[site]
    p = api(
        "/newsBlog/queryNewsBlogDetail",
        params={"locationUri": slug, "useType": s.use_type, "languageCode": language},
    )
    meta = {
        "title": p["title"],
        "url": s.url(slug),
        "category": p.get("categoryName") or "",
        "published": _date(p.get("publishFirstTime")),
        "updated": _date(p.get("updateTime")),
    }
    body = html_to_markdown(p.get("content") or "")
    out_dir.mkdir(parents=True, exist_ok=True)
    path.write_text(
        f"---\nmeta: {json.dumps(meta, ensure_ascii=False)}\n---\n\n# {p['title']}\n\n{body}"
    )
    return path


def _load(index: Path) -> list[dict]:
    return [json.loads(line) for line in index.read_text().splitlines() if line.strip()]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    i = sub.add_parser("index", help="write all post metadata as JSON lines")
    i.add_argument("--site", choices=SITES, default="pcb")
    i.add_argument("-o", "--output", type=Path)
    s = sub.add_parser("search", help="regex over an index")
    s.add_argument("pattern")
    s.add_argument("--index", type=Path, default=Path("jlc-blog-index.jsonl"))
    s.add_argument("--category")
    f = sub.add_parser("fetch", help="fetch posts as Markdown")
    f.add_argument("slugs", nargs="+")
    f.add_argument("--site", choices=SITES, default="pcb")
    f.add_argument("--out", type=Path, default=Path("jlc-blog"))
    args = ap.parse_args(argv)

    if args.cmd == "index":
        rows = build_index(http_api, site=args.site)
        text = "".join(json.dumps(r, ensure_ascii=False) + "\n" for r in rows)
        if args.output:
            args.output.write_text(text)
            print(f"{len(rows)} posts -> {args.output}", file=sys.stderr)
        else:
            sys.stdout.write(text)
    elif args.cmd == "search":
        for r in search(_load(args.index), args.pattern, args.category):
            print(
                f"{r['published']}  {r['category'][:28]:28}  {r['url']}  {r['title']}"
            )
    else:
        for slug in args.slugs:
            print(fetch_post(http_api, slug, site=args.site, out_dir=args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
