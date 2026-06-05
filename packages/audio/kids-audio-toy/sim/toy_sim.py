# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "numpy",
#     "opencv-python",
#     "sounddevice",
# ]
# ///
"""
Host simulator for the ESP32 Kids Audio Toy — webcam in, speakers out.

Runs the REAL firmware logic (main/audio_core.c, loaded via ctypes) on the
Mac so you can iterate without flashing. The webcam stands in for the three
pots and the 555 timer; the speakers stand in for the piezo. Only the parts
that genuinely need hardware are faked:

    pots (3 ADC channels)   <- brightness of the left/middle/right thirds
    555 timer (mod channel) <- frame-to-frame motion (wave fast = vibrato)
    piezo (hardware PWM)     -> software-synthesized square wave to speakers
    LED                      -> on-screen indicator

The mapping math, modulation smoothing/clamp, and modulated-frequency logic
are NOT reimplemented here — they are the same compiled C the firmware runs,
so what you hear is what the board would do (minus the analog waveform shape).

Build the shared lib and run via `just sim`, or directly:
    cc -dynamiclib -O2 -o sim/libaudio_core.dylib main/audio_core.c
    uv run sim/toy_sim.py

Controls: ESC or 'q' to quit.
"""

import argparse
import ctypes
import sys
import threading
from pathlib import Path

import cv2
import numpy as np
import sounddevice as sd

SAMPLE_RATE = 44_100
AMPLITUDE = 0.20  # output level (square waves are harsh — keep it gentle)
ADC_MAX = 4095
# Motion -> modulation gain. The 555 idles mid-rail (ADC ~2048 -> 0 Hz mod),
# so motion pushes the reading upward from mid-scale into positive vibrato.
MOTION_GAIN = 60.0
MOD_REST_ADC = 2048


class AudioParams(ctypes.Structure):
    _fields_ = [
        ("pitch_hz", ctypes.c_float),
        ("duration_ms", ctypes.c_float),
        ("interval_ms", ctypes.c_float),
    ]


def load_core() -> ctypes.CDLL:
    lib_path = Path(__file__).parent / "libaudio_core.dylib"
    if not lib_path.exists():
        sys.exit(
            f"Shared lib not found: {lib_path}\n"
            "Build it first:  cc -dynamiclib -O2 -o sim/libaudio_core.dylib "
            "main/audio_core.c\n(or just run `just sim`, which builds it for you)"
        )
    lib = ctypes.CDLL(str(lib_path))
    lib.audio_map_adc_to_range.restype = ctypes.c_float
    lib.audio_map_adc_to_range.argtypes = [
        ctypes.c_uint32,
        ctypes.c_float,
        ctypes.c_float,
    ]
    lib.audio_params_from_adc.restype = None
    lib.audio_params_from_adc.argtypes = [
        ctypes.c_uint32,
        ctypes.c_uint32,
        ctypes.c_uint32,
        ctypes.POINTER(AudioParams),
    ]
    lib.audio_update_modulation.restype = ctypes.c_float
    lib.audio_update_modulation.argtypes = [ctypes.c_float, ctypes.c_uint32]
    lib.audio_modulated_freq.restype = ctypes.c_float
    lib.audio_modulated_freq.argtypes = [ctypes.c_float, ctypes.c_float]
    return lib


class State:
    """Shared state across the webcam (main), scheduler, and audio threads.
    Python's GIL makes the individual scalar reads/writes here atomic enough
    for a demo — no lock needed for single floats/ints."""

    def __init__(self) -> None:
        # Inputs, written by the webcam loop.
        self.pitch_adc = 0
        self.duration_adc = 0
        self.interval_adc = 0
        self.mod_adc = MOD_REST_ADC
        # Audio target, written by the scheduler, read by the audio callback.
        self.freq = 440.0
        self.gate = 0.0
        # For the on-screen overlay, written by the scheduler.
        self.disp_pitch = 0.0
        self.disp_duration = 0.0
        self.disp_interval = 0.0
        self.disp_mod = 0.0


def make_audio_callback(state: State):
    phase = 0.0
    gain = 0.0  # current output gain, ramped toward the gate to avoid clicks

    def callback(outdata, frames, _time, status):
        nonlocal phase, gain
        if status:
            print(status, file=sys.stderr)
        freq = state.freq
        target = state.gate * AMPLITUDE

        # Per-sample phase for a phase-continuous square wave.
        incr = 2.0 * np.pi * freq / SAMPLE_RATE
        phases = phase + incr * np.arange(1, frames + 1)
        wave = np.sign(np.sin(phases)).astype(np.float32)
        phase = float(phases[-1] % (2.0 * np.pi))

        # Linear gain ramp across the block so gate transitions don't click.
        gains = np.linspace(gain, target, frames, dtype=np.float32)
        gain = target
        outdata[:, 0] = wave * gains

    return callback


def scheduler(lib: ctypes.CDLL, state: State, stop: threading.Event) -> None:
    """Mirror the firmware audio_task loop: read inputs, play one beep for
    `duration`, go silent for `interval`, repeat."""
    modulation = 0.0
    params = AudioParams()
    while not stop.is_set():
        lib.audio_params_from_adc(
            state.pitch_adc,
            state.duration_adc,
            state.interval_adc,
            ctypes.byref(params),
        )
        modulation = lib.audio_update_modulation(modulation, state.mod_adc)
        freq = lib.audio_modulated_freq(params.pitch_hz, modulation)

        state.freq = freq
        state.disp_pitch = params.pitch_hz
        state.disp_duration = params.duration_ms
        state.disp_interval = params.interval_ms
        state.disp_mod = modulation

        state.gate = 1.0
        if stop.wait(params.duration_ms / 1000.0):
            break
        state.gate = 0.0
        if stop.wait(params.interval_ms / 1000.0):
            break
    state.gate = 0.0


def region_brightness(gray: np.ndarray) -> tuple[int, int, int]:
    """Mean brightness of the left/middle/right thirds, each scaled to ADC."""
    w = gray.shape[1]
    third = w // 3
    means = (
        gray[:, :third].mean(),
        gray[:, third : 2 * third].mean(),
        gray[:, 2 * third :].mean(),
    )
    return tuple(int(m / 255.0 * ADC_MAX) for m in means)


def draw_overlay(frame: np.ndarray, state: State) -> None:
    h, w = frame.shape[:2]
    third = w // 3
    for x in (third, 2 * third):
        cv2.line(frame, (x, 0), (x, h), (60, 60, 60), 1)

    labels = [
        (f"pitch {state.disp_pitch:.0f} Hz", third // 2),
        (f"dur {state.disp_duration:.0f} ms", third + third // 2),
        (f"intvl {state.disp_interval:.0f} ms", 2 * third + third // 2),
    ]
    for text, cx in labels:
        size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
        cv2.putText(
            frame,
            text,
            (cx - size[0] // 2, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    cv2.putText(
        frame,
        f"mod {state.disp_mod:+.0f} Hz  (wave to vibrato)",
        (10, h - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (200, 200, 0),
        1,
        cv2.LINE_AA,
    )

    # LED indicator: filled while a beep is sounding.
    on = state.gate > 0.5
    color = (0, 255, 0) if on else (40, 40, 40)
    cv2.circle(frame, (w - 30, 30), 12, color, -1 if on else 2)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Webcam/speaker simulator for the Kids Audio Toy."
    )
    parser.add_argument(
        "--camera", type=int, default=0, help="camera index (default: 0)"
    )
    args = parser.parse_args()

    lib = load_core()
    state = State()
    stop = threading.Event()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        sys.exit(
            f"Could not open camera {args.camera}. On macOS, grant the terminal "
            "Camera access in System Settings > Privacy & Security > Camera."
        )

    prev_gray = None
    sched = threading.Thread(target=scheduler, args=(lib, state, stop), daemon=True)

    print(
        "Left/middle/right brightness -> pitch/duration/interval. "
        "Wave at the camera for vibrato. ESC or 'q' to quit."
    )
    try:
        with sd.OutputStream(
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype="float32",
            callback=make_audio_callback(state),
        ):
            sched.start()
            while not stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    print("Camera read failed.", file=sys.stderr)
                    break
                frame = cv2.flip(frame, 1)  # mirror for intuitive interaction
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                state.pitch_adc, state.duration_adc, state.interval_adc = (
                    region_brightness(gray)
                )
                if prev_gray is not None:
                    motion = float(
                        np.abs(
                            gray.astype(np.int16) - prev_gray.astype(np.int16)
                        ).mean()
                    )
                    state.mod_adc = int(
                        np.clip(MOD_REST_ADC + motion * MOTION_GAIN, 0, ADC_MAX)
                    )
                prev_gray = gray

                draw_overlay(frame, state)
                cv2.imshow("Kids Audio Toy — host sim", frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
    finally:
        stop.set()
        cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
