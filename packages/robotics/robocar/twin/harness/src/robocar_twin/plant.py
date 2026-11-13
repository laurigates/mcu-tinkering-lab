"""Plant ports: where the executor's range and floor readings come from.

``ScriptedPlant`` is the default: the scenario's ``range``, ``range_fail`` and
``floor`` events are the world, and the twin is never told to block on a plant.
``PhysicsPlant`` runs ``robocar_sim.plant`` (packages/robotics/robocar/simulation)
as a child process and relays the twin's per-tick ``plant_request`` to it
(ADR-036 section 3). The plant is a separate process so that this project's
``uv.lock`` and the simulation's stay independent.

The stepper protocol is the simulation's (its README): one ``init`` answered by
``ready``, then one ``step`` answered by one ``reading`` each, with exactly the
keys below. A bad message is answered by one ``error`` line and a non-zero exit.
"""

from __future__ import annotations

import contextlib
import json
import os
import queue
import shlex
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from .scenario import Scenario

PLANT_CMD_ENV = "ROBOCAR_PLANT_CMD"
WHEEL_DRIVES = ("forward", "reverse", "coast", "brake")
_STDERR_TAIL = 2000
_EOF = object()


class PlantError(RuntimeError):
    """The plant did not start, exited, reported an error or answered out of order."""


class PlantTimeout(PlantError):
    """The plant did not answer inside the run's wall-clock budget."""


@dataclass(frozen=True)
class PlantRequest:
    """One ``plant_request`` from the twin: the wheels after executor tick ``id``."""

    id: int
    at_ms: int
    dt_ms: int
    left: dict[str, Any]
    right: dict[str, Any]

    @classmethod
    def from_record(cls, rec: dict[str, Any]) -> PlantRequest:
        """Raises KeyError, TypeError or ValueError on a malformed record."""

        def wheel(side: str) -> dict[str, Any]:
            w = rec[side]
            drive, duty = w["drive"], w["duty"]
            if drive not in WHEEL_DRIVES:
                raise ValueError(f"{side}.drive is {drive!r}")
            if isinstance(duty, bool) or not isinstance(duty, int):
                raise ValueError(f"{side}.duty is {duty!r}")
            return {"drive": drive, "duty": duty}

        ints = {k: rec[k] for k in ("id", "at_ms", "dt_ms")}
        for k, v in ints.items():
            if isinstance(v, bool) or not isinstance(v, int):
                raise ValueError(f"{k} is {v!r}")
        return cls(left=wheel("left"), right=wheel("right"), **ints)

    def to_step(self) -> dict[str, Any]:
        """The stepper's ``step`` message. It refuses any other key, ``id`` and
        ``at_ms`` included."""
        return {
            "t": "step",
            "dt_ms": self.dt_ms,
            "left": self.left,
            "right": self.right,
        }


@dataclass(frozen=True)
class PlantReading:
    """One ``reading`` from the plant: the world at ``at_ms``."""

    at_ms: int
    pose: dict[str, float]
    range_cm: dict[str, int | None]
    floor: dict[str, bool]
    collided: bool
    off_floor: bool

    @classmethod
    def from_message(cls, msg: dict[str, Any]) -> PlantReading:
        """Raises KeyError, TypeError or ValueError on a malformed reading."""
        at_ms = msg["at_ms"]
        if isinstance(at_ms, bool) or not isinstance(at_ms, int):
            raise ValueError(f"at_ms is {at_ms!r}")
        pose = {k: msg["pose"][k] for k in ("x_cm", "y_cm", "heading_deg")}
        for k, v in pose.items():
            if isinstance(v, bool) or not isinstance(v, int | float):
                raise ValueError(f"pose.{k} is {v!r}")
        ranges = {k: msg["range_cm"][k] for k in ("front", "rear")}
        for k, v in ranges.items():
            if v is not None and (
                isinstance(v, bool) or not isinstance(v, int | float)
            ):
                raise ValueError(f"range_cm.{k} is {v!r}")
        floor = {k: msg["floor"][k] for k in ("front", "rear")}
        flags = {"collided": msg["collided"], "off_floor": msg["off_floor"]}
        for k, v in {**{f"floor.{k}": v for k, v in floor.items()}, **flags}.items():
            if not isinstance(v, bool):
                raise ValueError(f"{k} is {v!r}")
        return cls(at_ms, pose, ranges, floor, **flags)

    def to_protocol(self, request_id: int) -> dict[str, Any]:
        """The ``plant_response`` the twin blocks for."""
        return {
            "t": "plant_response",
            "id": request_id,
            "range_cm": self.range_cm,
            "floor": self.floor,
        }

    def pose_state(self) -> dict[str, Any]:
        """What a ``pose`` trace record reports, without its time."""
        return {**self.pose, "collided": self.collided, "off_floor": self.off_floor}


class PlantPort(Protocol):
    """The source of the twin's range and floor readings for one run."""

    name: str

    def start(self, timeout_s: float) -> None:
        """Bring the plant up before the twin starts. Raises PlantError."""
        ...

    def step(self, request: PlantRequest, timeout_s: float) -> PlantReading:
        """Advance the world by ``request.dt_ms`` under the request's wheels."""
        ...

    def close(self) -> None:
        """Release the plant. Safe to call twice and after a failure."""
        ...


class ScriptedPlant:
    """The scenario's own events are the world; there is no plant to step."""

    name = "scripted"

    def start(self, timeout_s: float) -> None:
        return None

    def step(self, request: PlantRequest, timeout_s: float) -> PlantReading:
        raise PlantError(
            f"the twin sent plant_request {request.id} at {request.at_ms} ms, but the "
            "scenario's plant is scripted and its config never asked for one"
        )

    def close(self) -> None:
        return None


def default_simulation_dir() -> Path:
    """packages/robotics/robocar/simulation: the directory next to this harness's
    own twin directory, wherever the scenario file lives."""
    from .runner import default_twin_dir  # local import: runner imports this module

    return default_twin_dir().parent / "simulation"


def plant_command() -> list[str]:
    """``$ROBOCAR_PLANT_CMD`` split as a shell would, else the simulation's stepper
    under its own locked environment."""
    env = os.environ.get(PLANT_CMD_ENV)
    if env:
        return shlex.split(env)
    return [
        "uv",
        "run",
        "--project",
        str(default_simulation_dir()),
        "--locked",
        "--quiet",
        "python",
        "-m",
        "robocar_sim.plant",
    ]


class PhysicsPlant:
    """``robocar_sim.plant`` as a child process, stepped once per executor tick."""

    name = "physics"

    def __init__(
        self,
        seed: int,
        arena: dict[str, Any],
        start: dict[str, Any],
        cmd: list[str] | None = None,
    ):
        self._init = {"t": "init", "seed": seed, "arena": arena, "start": start}
        self._cmd = list(cmd) if cmd is not None else plant_command()
        self._proc: subprocess.Popen[str] | None = None
        self._lines: queue.Queue[object] = queue.Queue()
        self._stderr: list[str] = []
        self._threads: list[threading.Thread] = []
        self.steps = 0

    @property
    def returncode(self) -> int | None:
        """The child's exit status; None before it started or while it runs."""
        return None if self._proc is None else self._proc.poll()

    def _fail(self, msg: str, exc_type: type[PlantError] = PlantError) -> PlantError:
        self.close()
        stderr = "".join(self._stderr)[-_STDERR_TAIL:]
        return exc_type(f"plant: {msg}\nplant stderr:\n{stderr or '  (empty)'}")

    def _send(self, obj: dict[str, Any]) -> None:
        assert self._proc is not None and self._proc.stdin is not None
        try:
            self._proc.stdin.write(json.dumps(obj, separators=(",", ":")) + "\n")
            self._proc.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            raise self._fail(
                f"the plant closed its stdin ({exc}) before '{obj['t']}' was sent"
            ) from exc

    def _receive(self, want: str, timeout_s: float) -> dict[str, Any]:
        """The plant's next line, which must be a ``want`` message."""
        try:
            item = self._lines.get(timeout=max(timeout_s, 0.0))
        except queue.Empty:
            raise self._fail(
                f"no '{want}' within the run's wall-clock budget", PlantTimeout
            ) from None
        if item is _EOF:
            assert self._proc is not None
            with contextlib.suppress(subprocess.TimeoutExpired):
                self._proc.wait(timeout=5)
            raise self._fail(
                f"the plant exited with status {self._proc.poll()} before sending '{want}'"
            )
        assert isinstance(item, str)
        try:
            msg = json.loads(item)
        except json.JSONDecodeError as exc:
            raise self._fail(
                f"the plant wrote a line that is not JSON: {item[:200]!r}"
            ) from exc
        kind = msg.get("t") if isinstance(msg, dict) else None
        if kind == "error":
            raise self._fail(f"the plant reported an error: {msg.get('message')}")
        if kind != want:
            raise self._fail(
                f"the plant answered out of order: expected '{want}', got {item[:200]!r}"
            )
        return msg

    def start(self, timeout_s: float) -> None:
        if self._proc is not None:
            raise PlantError("plant: already started")
        deadline = time.monotonic() + timeout_s
        try:
            self._proc = subprocess.Popen(
                self._cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                encoding="utf-8",
                bufsize=1,
                # Its own process group, so that close() can end a stepper
                # started by a wrapper such as `uv run` along with the wrapper.
                start_new_session=True,
            )
        except OSError as exc:
            raise PlantError(f"plant: cannot start {self._cmd}: {exc}") from exc
        proc = self._proc

        def pump_stdout() -> None:
            assert proc.stdout is not None
            for line in proc.stdout:
                if line.strip():
                    self._lines.put(line)
            self._lines.put(_EOF)

        def pump_stderr() -> None:
            assert proc.stderr is not None
            for line in proc.stderr:
                self._stderr.append(line)

        self._threads = [
            threading.Thread(target=pump_stdout, daemon=True),
            threading.Thread(target=pump_stderr, daemon=True),
        ]
        for t in self._threads:
            t.start()
        self._send(self._init)
        self._receive("ready", deadline - time.monotonic())

    def step(self, request: PlantRequest, timeout_s: float) -> PlantReading:
        if self._proc is None:
            raise PlantError("plant: step before start")
        self._send(request.to_step())
        msg = self._receive("reading", timeout_s)
        try:
            reading = PlantReading.from_message(msg)
        except (KeyError, TypeError, ValueError) as exc:
            raise self._fail(f"malformed reading ({exc!r}): {msg!r}") from exc
        # The plant's clock is the sum of the steps it was given, and the
        # twin's request covers the tick that starts at its at_ms. The two
        # disagree when a step or a reading was lost.
        want = request.at_ms + request.dt_ms
        if reading.at_ms != want:
            raise self._fail(
                f"the plant answered out of order: its reading is at {reading.at_ms} ms, "
                f"and plant_request {request.id} ends at {want} ms"
            )
        self.steps += 1
        return reading

    def close(self) -> None:
        """End the child and reap it: end of input first, then a kill of its
        process group. The group is killed because the stepper behind `uv run`
        is a grandchild that holds the output pipes, and it is killed whether
        or not the direct child has exited, because a wrapper can exit and
        leave the stepper running."""
        proc = self._proc
        if proc is None:
            return
        if proc.stdin is not None:
            with contextlib.suppress(OSError):
                proc.stdin.close()
        with contextlib.suppress(subprocess.TimeoutExpired):
            proc.wait(timeout=2)
        with contextlib.suppress(ProcessLookupError):
            os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
        for t in self._threads:
            t.join(timeout=1)
        for f in (proc.stdout, proc.stderr):
            if f is not None:
                with contextlib.suppress(OSError):
                    f.close()


def plant_for(scenario: Scenario, cmd: list[str] | None = None) -> PlantPort:
    """The plant a scenario's ``plant:`` key names. ``cmd`` replaces the physics
    plant's command, which tests use to run a fake one."""
    spec = scenario.plant
    if spec is None:
        return ScriptedPlant()
    return PhysicsPlant(scenario.seed, spec.arena, spec.start, cmd=cmd)
