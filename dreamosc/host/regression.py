#!/usr/bin/env python3
"""Golden regression: the C++ core (stretch_core.h via stretchcore) must agree
with the Python reference (stretchseq.py) on the properties that matter.

Not bit-equality — the two use different RNGs, so phase realizations differ by
design. We assert what should match regardless: output length, overall level
(RMS), and spectral-envelope shape, across a sweep of spread values (the control
whose model we just changed). Exits non-zero on any failure so CI / a pre-commit
hook can gate on it.

Run:  ./regression.py           (builds stretchcore if needed, makes a test wav)
Deps: numpy in host/.venv
"""

import os
import subprocess
import sys
import wave

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
IN_WAV = os.path.join(HERE, "reg_in.wav")
CPP_BIN = os.path.join(HERE, "stretchcore")
PY_REF = os.path.join(HERE, "stretchseq.py")

# Tolerances, set from measured agreement with headroom so normal float/RNG
# variation does not flap the gate.
#
# RMS: the C++ path (sine LUT + measured olaGain) and the Python path (numpy's
# exact FFT + analytic sine fade) differ by ~0.1-0.5 dB when steps overlap, but
# up to ~1.3 dB per step at spread 1.0, where steps are end-to-end and each
# stands alone so the single-step fade/normalization mismatch is undiluted. This
# is an implementation difference, not a bug — 1.5 dB covers it while still
# catching a real regression (e.g. the 4.7 dB spread-driven level swing the spec
# warns about if constant-loudness compensation breaks).
RMS_DB_TOL = 1.5           # |20log10(cpp/py)| must be under this
SPECTRUM_CORR_MIN = 0.85   # smoothed magnitude-spectrum correlation floor
SEED_HEX = "0x12345678"
SEED_DEC = str(int(SEED_HEX, 16))

# (stretch, duration, spread) cases. Spread sweep is the point.
CASES = [
    (50.0, 4.0, 0.0),
    (50.0, 4.0, 0.5),
    (50.0, 4.0, 1.0),
    (200.0, 2.0, 0.5),
]


def make_input():
    sr = 48000
    t = np.arange(int(2.0 * sr)) / sr
    x = (0.5 * np.sin(2 * np.pi * (220 + 60 * t) * t)
         + 0.3 * np.sin(2 * np.pi * 660 * t)
         + 0.2 * np.sin(2 * np.pi * 1500 * t))
    x /= np.max(np.abs(x)) * 1.02
    pcm = (x * 32767).astype("<i2")
    with wave.open(IN_WAV, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(pcm.tobytes())


def build_cpp():
    if os.path.exists(CPP_BIN):
        return
    subprocess.run(
        ["c++", "-std=c++17", "-O2", "-I..", "host_main.cpp", "-o", "stretchcore"],
        cwd=HERE, check=True,
    )


def load(path):
    with wave.open(path, "rb") as w:
        d = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2")
    return d.astype(float) / 32768.0


def smooth(x, k=257):
    return np.convolve(x, np.ones(k) / k, "same")


def compare(cpp, py):
    """Return (ok, messages) for one case."""
    msgs = []
    ok = True

    if abs(len(cpp) - len(py)) > 2:
        ok = False
        msgs.append(f"length mismatch: cpp={len(cpp)} py={len(py)}")
    n = min(len(cpp), len(py))
    cpp, py = cpp[:n], py[:n]
    if n == 0:
        return False, ["empty output"]

    def rms(v):
        return float(np.sqrt(np.mean(v ** 2)))
    rc, rp = rms(cpp), rms(py)
    if rp <= 0:
        ok = False
        msgs.append("python RMS is zero")
    else:
        db = 20 * np.log10(rc / rp)
        if abs(db) > RMS_DB_TOL:
            ok = False
        msgs.append(f"RMS {rc:.4f}/{rp:.4f} = {db:+.2f} dB (tol {RMS_DB_TOL})")

    C, P = smooth(np.abs(np.fft.rfft(cpp))), smooth(np.abs(np.fft.rfft(py)))
    corr = float(np.corrcoef(C, P)[0, 1])
    if corr < SPECTRUM_CORR_MIN:
        ok = False
    msgs.append(f"spectrum corr {corr:.4f} (min {SPECTRUM_CORR_MIN})")

    if not np.all(np.isfinite(cpp)):
        ok = False
        msgs.append("cpp output has non-finite samples")

    return ok, msgs


def main():
    make_input()
    build_cpp()

    all_ok = True
    for stretch, duration, spread in CASES:
        cpp_wav = os.path.join(HERE, "reg_cpp.wav")
        py_wav = os.path.join(HERE, "reg_py.wav")
        subprocess.run(
            [CPP_BIN, IN_WAV, cpp_wav, "--stretch", str(stretch),
             "--duration", str(duration), "--spread", str(spread),
             "--seed", SEED_HEX],
            cwd=HERE, check=True, stdout=subprocess.DEVNULL,
        )
        subprocess.run(
            [sys.executable, PY_REF, IN_WAV, py_wav,
             "--stretch", str(stretch), "--duration", str(duration),
             "--spread", str(spread), "--seed", SEED_DEC],
            cwd=HERE, check=True, stdout=subprocess.DEVNULL,
        )
        ok, msgs = compare(load(cpp_wav), load(py_wav))
        tag = "PASS" if ok else "FAIL"
        label = f"stretch={stretch} dur={duration} spread={spread}"
        print(f"[{tag}] {label}")
        for m in msgs:
            print(f"        {m}")
        all_ok = all_ok and ok

    print()
    if all_ok:
        print("regression OK — C++ core agrees with Python reference")
        return 0
    print("regression FAILED — C++ core diverged from Python reference")
    return 1


if __name__ == "__main__":
    sys.exit(main())
