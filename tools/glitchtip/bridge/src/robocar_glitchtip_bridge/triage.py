"""Read GlitchTip issues and events from the command line, without the UI.

    robocar-glitchtip issues [--project robocar] [--query is:unresolved] [--limit 20] [--json]
    robocar-glitchtip event <issue-id> [--json]

Text output is compact and stable, for a person or an agent; --json prints the
API's own response. Configuration (environment, written by
`just glitchtip::bootstrap`):

  GLITCHTIP_DOMAIN     base URL               (default http://localhost:8000)
  GLITCHTIP_API_TOKEN  read-only API token    (org, project, event read)
  GLITCHTIP_ORG_SLUG   organisation slug      (default robocar-lab)

Endpoints (GlitchTip 6.2, apps/issue_events/api): organisation issue list
`/api/0/organizations/<org>/issues/` (filters `project=<id>`, `query`), the
organisation's projects `/api/0/organizations/<org>/projects/`, and the
latest event of an issue `/api/0/issues/<id>/events/latest/`.

The formatters are pure functions of the decoded JSON; `api_get` is the only
I/O.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.parse
import urllib.request
from typing import Any


def api_get(
    base: str, token: str, path: str, params: dict[str, Any] | None = None
) -> Any:
    """GET a GlitchTip API path and decode the JSON body."""
    url = base.rstrip("/") + path
    if params:
        url += "?" + urllib.parse.urlencode(
            {k: v for k, v in params.items() if v not in (None, "")}, doseq=True
        )
    req = urllib.request.Request(
        url, headers={"Authorization": f"Bearer {token}", "Accept": "application/json"}
    )
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.load(resp)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="robocar-glitchtip", description=__doc__.split("\n")[0]
    )
    sub = parser.add_subparsers(dest="cmd", required=True)
    p_issues = sub.add_parser("issues", help="list issues, most recently seen first")
    p_issues.add_argument("--project", help="project slug (default: all)")
    p_issues.add_argument(
        "--query", default="is:unresolved", help="GlitchTip search query"
    )
    p_issues.add_argument("--limit", type=int, default=20)
    p_issues.add_argument("--json", action="store_true", help="print the API response")
    p_event = sub.add_parser("event", help="the latest event of one issue")
    p_event.add_argument("issue_id", type=int)
    p_event.add_argument("--json", action="store_true", help="print the API response")
    args = parser.parse_args(argv)

    base = os.environ.get("GLITCHTIP_DOMAIN", "http://localhost:8000")
    token = os.environ.get("GLITCHTIP_API_TOKEN", "")
    org = os.environ.get("GLITCHTIP_ORG_SLUG", "robocar-lab")
    if not token:
        print(
            "no GLITCHTIP_API_TOKEN; run `just glitchtip::bootstrap`", file=sys.stderr
        )
        return 2

    if args.cmd == "issues":
        projects = api_get(base, token, f"/api/0/organizations/{org}/projects/")
        params: dict[str, Any] = {"query": args.query, "limit": args.limit}
        if args.project:
            ids = [p["id"] for p in projects if p.get("slug") == args.project]
            if not ids:
                slugs = ", ".join(sorted(p.get("slug", "?") for p in projects))
                print(f"no project {args.project!r}; have: {slugs}", file=sys.stderr)
                return 2
            params["project"] = ids[0]
        issues = api_get(base, token, f"/api/0/organizations/{org}/issues/", params)
        if args.json:
            print(json.dumps(issues, indent=2))
        else:
            print(format_issues(issues[: args.limit]))
        return 0

    issue = api_get(base, token, f"/api/0/issues/{args.issue_id}/")
    event = api_get(base, token, f"/api/0/issues/{args.issue_id}/events/latest/")
    if args.json:
        print(json.dumps({"issue": issue, "event": event}, indent=2))
    else:
        print(format_event(issue, event))
    return 0


# Tags GlitchTip or the SDK add on the server side; they say nothing about the robot.
_SERVER_TAGS = {"environment", "server_name"}


def _release(issue: dict[str, Any]) -> str:
    rel = issue.get("lastRelease") or issue.get("firstRelease") or {}
    return rel.get("version") or "-"


def format_issues(issues: list[dict[str, Any]]) -> str:
    """One line per issue: id, project, level, event count, last seen, release, title."""
    if not issues:
        return "no issues"
    lines = []
    for i in issues:
        project = (i.get("project") or {}).get("slug", "?")
        lines.append(
            f"#{i.get('id')} {project:<10} {i.get('level', '?'):<7} "
            f"{i.get('count', '?')}x last {i.get('lastSeen', '?')} "
            f"{_release(i)}  {i.get('title', '')}"
        )
    return "\n".join(lines)


def _frame_line(frame: dict[str, Any]) -> str:
    addr = frame.get("instruction_addr") or frame.get("instructionAddr") or "?"
    if not frame.get("function") and not frame.get("filename"):
        return f"    {addr} ?"
    where = frame.get("filename") or "?"
    line = frame.get("lineNo") or frame.get("lineno")
    if line:
        where += f":{line}"
    return f"    {addr} {frame.get('function') or '?'} {where}"


def format_event(issue: dict[str, Any], event: dict[str, Any]) -> str:
    """The latest event of an issue as plain text: what an agent needs to triage it."""
    project = (issue.get("project") or {}).get("slug", "?")
    out = [
        f"Issue #{issue.get('id')} {issue.get('shortId', '')} [{project}] "
        f"{issue.get('level', '?')}, {issue.get('count', '?')} event(s), "
        f"first {issue.get('firstSeen', '?')}, last {issue.get('lastSeen', '?')}",
        f"Release: {_release(issue)}",
    ]
    if issue.get("permalink"):
        out.append(f"UI: {issue['permalink']}")
    context = event.get("context") or {}
    received = event.get("dateReceived", "?")
    if context.get("robot_timestamp_missing"):
        received += " (bridge receive time: the robot has no clock)"
    out.append(f"Event {event.get('eventID', '?')} received {received}")
    out.append(f"Message: {event.get('message') or event.get('title') or ''}")

    entries = {e.get("type"): e.get("data") or {} for e in event.get("entries") or []}
    for exc in (entries.get("exception") or {}).get("values") or []:
        out.append(f"Exception: {exc.get('type', '?')}: {exc.get('value', '')}")
        frames = list(reversed((exc.get("stacktrace") or {}).get("frames") or []))
        if frames:
            out.append("  frames (innermost first):")
            out.extend(_frame_line(f) for f in frames)

    tags = [
        f"{t['key']}={t['value']}"
        for t in event.get("tags") or []
        if t.get("key") not in _SERVER_TAGS
    ]
    if tags:
        out.append("Tags: " + ", ".join(tags))
    extra = [f"{k}={v}" for k, v in context.items() if k != "robot_timestamp_missing"]
    if extra:
        out.append("Extra: " + ", ".join(extra))

    crumbs = (entries.get("breadcrumbs") or {}).get("values") or []
    if crumbs:
        out.append(f"Breadcrumbs (oldest first, {len(crumbs)}):")
        out.extend(
            f"  [{c.get('category') or 'log'}] {c.get('message') or ''}" for c in crumbs
        )
    return "\n".join(out)


if __name__ == "__main__":
    sys.exit(main())
