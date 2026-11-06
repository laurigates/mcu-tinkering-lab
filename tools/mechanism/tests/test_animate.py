"""The travel animation: its pose path, the clashes between gate poses, the video."""

import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest
from build123d import Box, Pos, Rot
from mechanism import animate, check
from mechanism.check import Body


def test_the_path_visits_every_gate_pose_one_joint_step_at_a_time():
    mod = SimpleNamespace()
    path = animate.path(mod, between=0)
    assert sorted(path) == sorted(check.DEFAULT_SWEEP)
    for a, b in zip(path, path[1:], strict=False):
        assert sum(x != y for x, y in zip(a, b, strict=True)) == 1


def test_between_frames_interpolate_each_step():
    mod = SimpleNamespace(SWEEP=[(0, 0), (90, 0)])
    assert animate.path(mod, between=2) == [(0, 0), (30, 0), (60, 0), (90, 0)]


def test_a_three_joint_sweep_is_ordered_too():
    mod = SimpleNamespace(
        SWEEP=[(p, t, b) for p in (0, 1) for t in (0, 1) for b in (0, 1)]
    )
    path = animate.path(mod, between=0)
    assert sorted(path) == sorted(mod.SWEEP)
    for a, b in zip(path, path[1:], strict=False):
        assert sum(x != y for x, y in zip(a, b, strict=True)) == 1


def post_at_45():
    """An arm that clears the post at pan 0 and 90 and hits it at 45 only."""

    def assembly(pan, tilt):
        arm = Rot(0, 0, pan) * Pos(15, 0, 0) * Box(30, 4, 4)
        post = Rot(0, 0, 45) * Pos(20, 0, 0) * Box(4, 4, 10)
        return [Body("arm", arm, "moving", printed=True), Body("post", post, "base")]

    return SimpleNamespace(assembly=assembly, SWEEP=[(0, 0), (90, 0)])


def test_a_clash_between_gate_poses_is_reported_as_missed_by_the_gate():
    mod = post_at_45()
    assert check.sweep(mod) == {}
    frames = animate.frames(mod, animate.path(mod, between=1))
    assert [f.gate for f in frames] == [True, False, True]
    assert [bool(f.clashes) for f in frames] == [False, True, False]
    summary = animate.summary(mod, frames)
    assert summary["gate_clashes"] == {}
    assert list(summary["between_clashes"]) == ["pan 45 tilt 0"]
    (c,) = summary["between_clashes"]["pan 45 tilt 0"]
    assert (c["a"], c["b"]) == ("arm", "post")


def test_holds_give_motion_one_frame_and_stops_longer():
    step = animate.Frame((15, 0), gate=False, clashes=[])
    gate = animate.Frame((30, 0), gate=True, clashes=[])
    hit = animate.Frame((45, 0), gate=False, clashes=[{"a": "x", "b": "y", "mm3": 1.0}])
    assert animate.hold(step, fps=24) == 1
    assert animate.hold(gate, fps=24) == round(24 * animate.HOLD_GATE_S)
    assert animate.hold(hit, fps=24) == round(24 * animate.HOLD_CLASH_S)
    assert animate.hold(step, fps=24, end=True) == round(24 * animate.HOLD_END_S)


def _ffprobe(path: Path, entry: str) -> str:
    out = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-count_frames",
            "-show_entries",
            f"stream={entry}",
            "-of",
            "csv=p=0",
            str(path),
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    return out.stdout.strip()


def test_the_video_is_av1_with_every_held_frame(tmp_path: Path):
    pv = pytest.importorskip("pyvista")
    try:
        p = pv.Plotter(off_screen=True, window_size=(64, 64))
        p.screenshot(str(tmp_path / "probe.png"))
        p.close()
    except Exception as e:  # no libOSMesa / EGL
        pytest.skip(f"no off-screen VTK: {e}")
    if animate.av1_encoder() is None:
        pytest.skip("no ffmpeg with an AV1 encoder")

    design = tmp_path / "arm"
    design.mkdir()
    (design / "design.py").write_text(
        "from build123d import Box, Pos, Rot\n"
        "from mechanism.check import Body\n"
        "SWEEP = [(0, 0), (90, 0)]\n"
        "def printed_parts():\n"
        "    return {'arm': Pos(15, 0, 0) * Box(30, 4, 4)}\n"
        "def assembly(pan, tilt):\n"
        "    arm = Rot(0, 0, pan) * printed_parts()['arm']\n"
        "    post = Rot(0, 0, 45) * Pos(20, 0, 0) * Box(4, 4, 10)\n"
        "    return [Body('arm', arm, 'moving', printed=True), Body('post', post, 'base')]\n"
    )
    # odd width: the encoder needs even dimensions, so the writer pads
    assert (
        animate.main(
            [str(design), "--between", "1", "--size", "161x120", "--fps", "12"]
        )
        == 1
    )
    out = design / "output" / "animation"
    video = out / "sweep.mp4"
    assert _ffprobe(video, "codec_name") == "av1"
    # gate+start, clash, gate+end: each held for its longest reason
    want = 2 * round(12 * animate.HOLD_END_S) + round(12 * animate.HOLD_CLASH_S)
    assert int(_ffprobe(video, "nb_read_frames")) == want
    assert (out / "sweep-sheet.png").exists()
    assert (out / "sweep.json").exists()
