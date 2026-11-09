"""handle() end to end, with a transport that records what would be sent."""

from pathlib import Path

import sentry_sdk
from sentry_sdk.transport import Transport

from robocar_glitchtip_bridge.main import handle
from robocar_glitchtip_bridge.symbolicate import ElfStore

FIXTURES = Path(__file__).parents[2] / "fixtures"


class Recorder(Transport):
    def __init__(self, options=None):
        super().__init__(options)
        self.events = []

    def capture_envelope(self, envelope):
        self.events.extend(
            item.payload.json for item in envelope.items if item.type == "event"
        )


def client(recorder):
    return sentry_sdk.Client(dsn="http://key@localhost:1/1", transport=recorder)


def test_routes_each_source_to_its_own_client(tmp_path):
    firmware, planner = Recorder(), Recorder()
    clients = {"robocar": client(firmware), "robocar-ai": client(planner)}
    elves = ElfStore(tmp_path)

    sent = handle((FIXTURES / "reset-brownout.json").read_bytes(), clients, elves)
    handle((FIXTURES / "planner-note.json").read_bytes(), clients, elves)

    assert sent == "0123456789abcdef0123456789abcdef"
    assert [e["message"]["formatted"] for e in firmware.events] == ["reset: BROWNOUT"]
    assert [e["tags"]["kind"] for e in planner.events] == ["difficulty"]


def test_drops_malformed_and_unrouted(tmp_path):
    recorder = Recorder()
    clients = {"robocar": client(recorder)}  # no robocar-ai DSN
    elves = ElfStore(tmp_path)
    assert handle(b"not json", clients, elves) is None
    assert handle((FIXTURES / "planner-note.json").read_bytes(), clients, elves) is None
    assert recorder.events == []
