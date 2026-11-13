"""A tiny in-process stand-in for robocar-twin, speaking the same JSONL protocol.

It is not a model of the planner: it requests on a tick only when a console
line, a sound or a new scene arrived since the previous tick, which is enough
to exercise the harness (request servicing, latency windows, suppression,
backoff records, timeouts and protocol errors) without the C twin.

It is not a model of the executor either. A ``drive`` goal turns both wheels
forward at once, any other goal coasts them, and a front ``range`` under 15 cm
brakes a drive at the event's own time: no ramp, no read cycle, no rear sensor.
``range_fail`` and ``floor`` are accepted and change nothing. With ``head`` in
the config it writes one ``head`` record at 0 ms and frames carry no pose.
That is enough for the runner to carry ``wheels`` and ``head`` records and for
the assertions to be run against a trace the harness collected.

With ``"plant": "external"`` in the config it ticks every 33 ms from 0, as the
twin's executor does: each tick writes a ``wheels`` record if the drive
changed, then one ``plant_request`` carrying the wheels, and blocks for the
``plant_response``. A front range under 15 cm or no floor under the front
sensor in that response brakes a drive from the next tick on. Ticks inside a
reply's latency window exchange before the reply is handled. Like the twin it
refuses ``range``, ``range_fail`` and ``floor`` events in that mode (exit 2).

Test-only config keys: ``fake_bad_key`` (send a wrong key), ``fake_exit``
(exit with this status before ``done``), ``fake_hang`` (sleep forever),
``fake_garbage`` (write a non-JSON line), ``fake_bad_plant_request`` (send
plant requests without their wheels).
"""

from __future__ import annotations

import hashlib
import json
import sys
import time

TICK_MS = 33
PLANT_EVENTS = ("range", "range_fail", "floor")


def out(obj: dict) -> None:
    sys.stdout.write(json.dumps(obj) + "\n")
    sys.stdout.flush()


def key_of(descriptor: dict) -> str:
    canon = json.dumps(
        descriptor, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    )
    return hashlib.sha256(canon.encode("utf-8")).hexdigest()


def parse_reply(body: str) -> tuple[str, str]:
    parts = json.loads(body)["candidates"][0]["content"]["parts"]
    goal, speech = "stop", ""
    for p in parts:
        fc = p.get("functionCall", {})
        if fc.get("name") == "speak":
            speech = fc.get("args", {}).get("text", "")
        elif fc.get("name"):
            goal = fc["name"]
    return goal, speech


def main() -> int:
    config: dict = {}
    events: list[dict] = []
    for line in sys.stdin:
        msg = json.loads(line)
        if msg["t"] == "config":
            config = msg
        elif msg["t"] == "run":
            break
        else:
            events.append(msg)
    if config.get("fake_hang"):
        time.sleep(3600)
    if config.get("fake_garbage"):
        sys.stdout.write("not json\n")
        sys.stdout.flush()

    plant = config.get("plant") == "external"
    if plant:
        scripted = [e["t"] for e in events if e["t"] in PLANT_EVENTS]
        if scripted:
            print(
                f"fake twin: '{scripted[0]}' event with an external plant",
                file=sys.stderr,
            )
            return 2

    period = config.get("planner_period_ms", 15000)
    duration = config["duration_ms"]
    events.sort(key=lambda e: e["at_ms"])
    state = {"scene": None, "pose": "C", "woken": False}
    voice_windows: list[tuple[int, int]] = []
    wheels = {"goal": "stop", "blocked": False, "drive": "coast", "duty": 0}

    def set_wheels(at_ms: int) -> None:
        """Write a `wheels` record if the goal and the obstacle changed the drive."""
        if wheels["goal"] != "drive":
            drive, duty = "coast", 0
        elif wheels["blocked"]:
            drive, duty = "brake", 0
        else:
            drive, duty = "forward", 2457
        if drive != wheels["drive"]:
            wheels["drive"], wheels["duty"] = drive, duty
            side = {"drive": drive, "duty": duty}
            out({"t": "wheels", "at_ms": at_ms, "left": side, "right": side})

    def exec_tick(tick: int, at_ms: int) -> None:
        """One executor tick with a plant: wheels, then the exchange."""
        set_wheels(at_ms)
        request = {"t": "plant_request", "id": tick, "at_ms": at_ms, "dt_ms": TICK_MS}
        if not config.get("fake_bad_plant_request"):
            side = {"drive": wheels["drive"], "duty": wheels["duty"]}
            request.update(left=side, right=side)
        out(request)
        resp = json.loads(sys.stdin.readline())
        assert resp["t"] == "plant_response" and resp["id"] == tick, resp
        front = resp["range_cm"]["front"]
        wheels["blocked"] = (front is not None and front < 15) or not resp["floor"][
            "front"
        ]

    def apply(e: dict) -> None:
        if e["t"] in ("console", "sound"):
            state["woken"] = True
        elif e["t"] == "frame":
            if state["scene"] is not None and e["scene"] != state["scene"]:
                state["woken"] = True
            # A frame has no pose when the config sets `head`.
            state["scene"], state["pose"] = e["scene"], e.get("pose", "C")
        elif e["t"] in ("range", "range_fail") and e.get("sensor", "front") == "front":
            wheels["blocked"] = e["t"] == "range" and e["cm"] < 15
            set_wheels(e["at_ms"])
        elif e["t"] == "voice_turn":
            end = e["at_ms"] + e["dur_ms"]
            voice_windows.append((e["at_ms"], end))
            out(
                {
                    "t": "utterance",
                    "at_ms": end,
                    "source": "voice_turn",
                    "text": e["reply"],
                }
            )

    if config.get("head"):
        out(
            {
                "t": "head",
                "at_ms": 0,
                "pan_deg": 0,
                "tilt_deg": 0,
                "mode": "gaze",
                "settled": False,
            }
        )

    backoff = 0
    next_tick = 0
    req_id = 0
    idx = 0
    ticks = 0
    t = 0

    def advance(to: int) -> None:
        """Apply every event and, with a plant, every executor tick up to
        ``to``, in time order; an event goes before a tick at the same instant."""
        nonlocal idx, ticks
        while True:
            event_at = events[idx]["at_ms"] if idx < len(events) else None
            tick_at = ticks * TICK_MS if plant else None
            event_first = tick_at is None or (
                event_at is not None and event_at <= tick_at
            )
            if event_first and event_at is not None and event_at <= to:
                apply(events[idx])
                idx += 1
                continue
            if tick_at is not None and tick_at <= to:
                exec_tick(ticks, tick_at)
                ticks += 1
                continue
            return

    while t <= duration:
        advance(t)
        if t == next_tick:
            out(
                {
                    "t": "cycle",
                    "at_ms": t,
                    "why": "C" if state["woken"] else "Z",
                    "warranted": state["woken"],
                }
            )
            if state["woken"]:
                state["woken"] = False
                req_id += 1
                descriptor = {
                    "kind": "plan",
                    "scene": state["scene"],
                    "pose": state["pose"],
                    "gate": "-",
                    "speak_allowed": True,
                }
                key = "0" * 64 if config.get("fake_bad_key") else key_of(descriptor)
                out(
                    {
                        "t": "model_request",
                        "id": req_id,
                        "at_ms": t,
                        "kind": "plan",
                        "descriptor": descriptor,
                        "key": key,
                    }
                )
                resp = json.loads(sys.stdin.readline())
                assert resp["t"] == "model_response" and resp["id"] == req_id
                handled = t + resp["latency_ms"]
                # Events inside the latency window land before the reply is
                # handled, and so do the window's plant exchanges.
                advance(handled)
                if resp["status"] == 200:
                    goal, speech = parse_reply(resp["body"])
                    if speech:
                        spoke = any(a <= handled and b >= t for a, b in voice_windows)
                        if spoke:
                            out(
                                {
                                    "t": "suppressed",
                                    "at_ms": handled,
                                    "reason": "person_spoke",
                                    "text": speech,
                                }
                            )
                        else:
                            out(
                                {
                                    "t": "utterance",
                                    "at_ms": handled,
                                    "source": "planner",
                                    "text": speech,
                                }
                            )
                    out({"t": "goal", "at_ms": handled, "kind": goal})
                    wheels["goal"] = goal
                    if not plant:  # with a plant the next tick writes the wheels
                        set_wheels(handled)
                    backoff = 0
                else:
                    backoff = 1 if backoff == 0 else min(backoff * 2, 4)
                    out(
                        {"t": "force_stop", "at_ms": handled, "reason": "backend_error"}
                    )
                    out({"t": "backoff", "at_ms": handled, "periods": backoff})
            next_tick = t + period * (1 + backoff)
        if config.get("fake_exit") and t >= period:
            return int(config["fake_exit"])
        # Step to the next instant something happens (both are > t here).
        upcoming = [next_tick] + ([events[idx]["at_ms"]] if idx < len(events) else [])
        if plant:
            upcoming.append(ticks * TICK_MS)
        t = min(upcoming)
    out({"t": "done", "at_ms": duration})
    return 0


if __name__ == "__main__":
    sys.exit(main())
