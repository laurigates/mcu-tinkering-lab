"""The JSONL stepper, ``python -m robocar_sim.plant`` (ADR-036 "Plant process")."""

import io
import json
import os
import select
import subprocess
import sys

import pytest

from robocar_sim.plant import run_stepper

ARENA = {
    "floor": [[-50, -60], [150, -60], [150, 60], [-50, 60]],
    "walls": [{"from": [150, -60], "to": [150, 60]}],
}
INIT = {"t": "init", "seed": 7, "arena": ARENA, "start": {"x_cm": 0, "y_cm": 0, "heading_deg": 0}}
FORWARD = {"drive": "forward", "duty": 2409}
STEP = {"t": "step", "dt_ms": 33, "left": FORWARD, "right": FORWARD}

STEPPER = [sys.executable, "-m", "robocar_sim.plant"]


def lines(*messages):
    return "".join(json.dumps(m) + "\n" for m in messages)


def answer(*messages):
    """Run the stepper in-process; return (exit status, replies, raw output)."""
    out = io.StringIO()
    status = run_stepper(io.StringIO(lines(*messages)), out)
    return status, [json.loads(line) for line in out.getvalue().splitlines()], out.getvalue()


class TestProtocol:
    def test_init_is_answered_with_ready(self):
        status, replies, raw = answer(INIT)

        assert status == 0
        assert replies == [{"t": "ready"}]
        assert raw == '{"t":"ready"}\n'

    def test_a_step_is_answered_with_a_reading(self):
        status, replies, raw = answer(INIT, STEP)

        assert status == 0
        reading = replies[1]
        assert list(reading) == ["t", "at_ms", "pose", "range_cm", "floor", "collided", "off_floor"]
        assert reading["t"] == "reading"
        assert reading["at_ms"] == 33
        assert list(reading["pose"]) == ["x_cm", "y_cm", "heading_deg"]
        # The reading ADR-036 section 3 prints, to the 0.001 the stepper
        # rounds a pose to. It moves with plant.yaml's motor estimates, and
        # the ADR's example with it.
        assert reading["pose"]["x_cm"] == 0.086
        assert reading["pose"]["y_cm"] == 0.0
        assert reading["pose"]["heading_deg"] == 0.0
        assert reading["range_cm"]["rear"] is None
        assert 138 <= reading["range_cm"]["front"] <= 144
        assert reading["floor"] == {"front": True, "rear": True}
        assert reading["collided"] is False
        assert reading["off_floor"] is False
        # Compact JSON, one line per message, nothing else.
        assert raw.count("\n") == 2
        assert " " not in raw

    def test_at_ms_accumulates_over_steps(self):
        _, replies, _ = answer(INIT, STEP, {**STEP, "dt_ms": 10}, STEP)

        assert [r["at_ms"] for r in replies[1:]] == [33, 43, 76]

    def test_each_wheel_is_driven_by_its_own_command(self):
        turn = {**STEP, "left": {"drive": "reverse", "duty": 3000}, "right": FORWARD}

        _, replies, _ = answer(INIT, *[turn] * 15)

        assert replies[-1]["pose"]["heading_deg"] > 5.0, "left back, right forward: turns left"

    def test_the_start_pose_is_used(self):
        start = {"x_cm": 100, "y_cm": 20, "heading_deg": 90}
        coast = {"drive": "coast", "duty": 0}

        _, replies, _ = answer({**INIT, "start": start}, {**STEP, "left": coast, "right": coast})

        assert replies[1]["pose"] == {"x_cm": 100.0, "y_cm": 20.0, "heading_deg": 90.0}

    def test_end_of_input_is_a_clean_exit(self):
        assert answer() == (0, [], "")

    def test_blank_lines_are_skipped(self):
        out = io.StringIO()

        status = run_stepper(io.StringIO("\n" + lines(INIT) + "\n\n" + lines(STEP)), out)

        assert status == 0
        assert len(out.getvalue().splitlines()) == 2


class TestErrors:
    @pytest.mark.parametrize(
        ("messages", "fragment"),
        [
            ([STEP], "expected t='init', got t='step'"),
            ([{**INIT, "seed": -1}], "seed: expected a non-negative integer"),
            ([{**INIT, "arena": {}}], "arena: missing key(s) floor"),
            ([{**INIT, "arena": {**ARENA, "roof": 1}}], "arena: unknown key(s) roof"),
            ([{**INIT, "config": {}}], "init: unknown key(s) config"),
            ([{k: v for k, v in INIT.items() if k != "start"}], "init: missing key(s) start"),
            ([{**INIT, "start": {"x_cm": 0, "y_cm": 0}}], "init.start: missing key(s) heading_deg"),
            ([{**INIT, "start": {"x_cm": 149, "y_cm": 0, "heading_deg": 0}}], "start: the chassis"),
            ([INIT, INIT], "expected t='step', got t='init'"),
            ([INIT, {**STEP, "dt_ms": 0}], "dt_ms: expected 1..1000"),
            ([INIT, {**STEP, "dt_ms": 33.5}], "dt_ms: expected an integer"),
            ([INIT, {**STEP, "id": 1}], "step: unknown key(s) id"),
            ([INIT, {"t": "step", "dt_ms": 33, "left": FORWARD}], "step: missing key(s) right"),
            (
                [INIT, {**STEP, "left": {"drive": "stop", "duty": 0}}],
                "step.left.drive: expected one of forward, reverse, coast, brake",
            ),
            (
                [INIT, {**STEP, "right": {"drive": "forward", "duty": 4096}}],
                "step.right.duty: expected 0..4095",
            ),
            ([INIT, {**STEP, "right": {"drive": "forward"}}], "step.right: missing key(s) duty"),
            ([INIT, [1, 2]], "step: expected a JSON object"),
        ],
    )
    def test_a_bad_message_gets_one_error_and_a_non_zero_exit(self, messages, fragment):
        status, replies, _ = answer(*messages)

        assert status != 0
        assert replies[-1]["t"] == "error"
        assert list(replies[-1]) == ["t", "message"]
        assert fragment in replies[-1]["message"]
        assert [r["t"] for r in replies[:-1]] == ["ready"] * (len(messages) - 1)

    def test_a_line_that_is_not_json_is_an_error(self):
        out = io.StringIO()

        status = run_stepper(io.StringIO("init\n"), out)

        assert status != 0
        assert json.loads(out.getvalue())["message"].startswith("not a JSON line")

    def test_json_nested_too_deep_to_decode_is_an_error(self, tmp_path):
        # Run as a process: an uncaught RecursionError would leave a traceback
        # on stderr and no error line.
        result = run_process("[" * 100_000 + "\n", tmp_path)

        assert result.returncode == 1
        assert result.stderr == ""
        reply = json.loads(result.stdout)
        assert list(reply) == ["t", "message"]
        assert reply["t"] == "error"
        assert reply["message"].startswith("not a JSON line")

    def test_nothing_is_read_after_an_error(self):
        status, replies, _ = answer(INIT, {**STEP, "dt_ms": 0}, STEP)

        assert status != 0
        assert [r["t"] for r in replies] == ["ready", "error"]


def stepper_env(**overrides):
    """The environment without PYTHONUNBUFFERED, which would flush for the stepper."""
    env = {k: v for k, v in os.environ.items() if k != "PYTHONUNBUFFERED"}
    env.update(overrides)
    return env


def run_process(stdin_text, cwd):
    return subprocess.run(
        STEPPER,
        input=stdin_text,
        capture_output=True,
        text=True,
        cwd=cwd,
        timeout=60,
        check=False,
        env=stepper_env(),
    )


class TestProcess:
    def test_two_runs_on_the_same_input_are_byte_identical(self, tmp_path):
        full = {"drive": "forward", "duty": 4095}
        brake = {"drive": "brake", "duty": 0}
        script = lines(
            INIT,
            *[STEP] * 40,
            *[{**STEP, "left": brake, "right": brake}] * 10,
            *[{**STEP, "left": full, "right": full}] * 150,  # into the wall, held
            *[{**STEP, "left": full}] * 20,  # uneven wheels against the wall
        )

        first = run_process(script, tmp_path)
        second = run_process(script, tmp_path)

        assert first.returncode == 0, first.stderr
        assert first.stderr == ""
        assert first.stdout == second.stdout
        replies = [json.loads(line) for line in first.stdout.splitlines()]
        assert [r["t"] for r in replies] == ["ready"] + ["reading"] * 220
        assert replies[-1]["collided"], "the script ends against the wall"
        # Run from an empty directory: the robot description comes from the
        # package, and the plant leaves no file behind.
        assert list(tmp_path.iterdir()) == []

    def test_a_bad_init_exits_non_zero_with_an_error_line(self, tmp_path):
        result = run_process(lines({**INIT, "arena": {"floor": [[0, 0], [1, 1]]}}), tmp_path)

        assert result.returncode != 0
        assert result.stdout.count("\n") == 1
        reply = json.loads(result.stdout)
        assert reply["t"] == "error"
        assert "arena.floor: a polygon needs at least 3 points" in reply["message"]

    def test_a_bad_step_exits_non_zero_with_an_error_line(self, tmp_path):
        result = run_process(lines(INIT, STEP, {**STEP, "dt_ms": -5}), tmp_path)

        assert result.returncode != 0
        replies = [json.loads(line) for line in result.stdout.splitlines()]
        assert [r["t"] for r in replies] == ["ready", "reading", "error"]

    def test_each_reply_arrives_before_the_next_request_is_sent(self, tmp_path):
        """The harness blocks on every reply, so each must be flushed."""

        def exchange(process, message):
            process.stdin.write((json.dumps(message) + "\n").encode())
            process.stdin.flush()
            fd = process.stdout.fileno()
            reply = b""
            while not reply.endswith(b"\n"):
                ready, _, _ = select.select([fd], [], [], 30)
                assert ready, f"no reply to {message['t']} within 30 s"
                chunk = os.read(fd, 65536)
                assert chunk, "the stepper closed its output"
                reply += chunk
            return json.loads(reply)

        with subprocess.Popen(
            STEPPER,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            cwd=tmp_path,
            bufsize=0,
            env=stepper_env(),
        ) as process:
            assert exchange(process, INIT) == {"t": "ready"}
            assert exchange(process, STEP)["at_ms"] == 33
            assert exchange(process, STEP)["at_ms"] == 66
            process.stdin.close()
            assert process.wait(timeout=30) == 0

    def test_input_that_is_not_utf8_is_an_error_line(self, tmp_path):
        """With a strictly decoding stdin the bad bytes would otherwise raise."""
        result = subprocess.run(
            STEPPER,
            input=b'{"t":"init"\xff}\n',
            capture_output=True,
            cwd=tmp_path,
            timeout=60,
            check=False,
            env=stepper_env(PYTHONIOENCODING="utf-8:strict"),
        )

        assert result.returncode != 0
        assert result.stderr == b""
        reply = json.loads(result.stdout)
        assert reply["t"] == "error"
        assert reply["message"].startswith("not a JSON line")
