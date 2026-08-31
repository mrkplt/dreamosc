"""
PaulStretch -- extreme audio time-stretching.

Algorithm by Nasca Octavian Paul (public domain). This is a clean-room
reimplementation of the same idea, with a small library API plus a CLI.

The core loop is short:

    for each overlapping window of the input:
        take the FFT
        keep the magnitudes, replace every phase with a random one
        take the inverse FFT
        window it again and overlap-add into the output

The output windows advance at half a window per step; the *input* windows
advance at half a window / stretch. That ratio is the whole stretch. Because
the phases are thrown away and re-randomized every frame, consecutive frames
never line up in a way the ear can hear as a repeat, which is why this stays
smooth at 50x where a normal phase-vocoder turns into a stuttering mess. The
tradeoff is that transients are destroyed -- this is a texture/pad machine,
not a tool for slowing down a drum loop.

Usage:
    python paulstretch.py input.wav output.wav --stretch 8 --window 0.25
"""

from __future__ import annotations

import argparse
import wave

import numpy as np


def optimize_windowsize(n: int) -> int:
    """Round n up to the next 5-smooth number (factors of 2, 3, 5 only).

    FFTs of such sizes are much faster than sizes with a large prime factor.
    """
    candidate = n
    while True:
        m = candidate
        for prime in (2, 3, 5):
            while m % prime == 0:
                m //= prime
        if m == 1:
            return candidate
        candidate += 1


def make_window(size: int) -> np.ndarray:
    """The PaulStretch window: (1 - x^2) ** 1.25 for x in [-1, 1].

    It gets applied twice per frame (once before the FFT, once after the
    inverse FFT), so what matters for overlap-add is w^2. At 50% overlap this
    shape sums to ~0.977 at the seams instead of the 1.0 a Hann window would
    give -- a ~0.2 dB ripple, inaudible, and the smoother rolloff is worth it.
    """
    x = np.linspace(-1.0, 1.0, size, dtype=np.float64)
    return (1.0 - x ** 2) ** 1.25


def paulstretch(
    samples: np.ndarray,
    samplerate: int,
    stretch: float,
    window_seconds: float = 0.25,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    """Time-stretch audio by `stretch` without changing pitch.

    Args:
        samples: float array, shape (channels, frames), nominally in [-1, 1].
        samplerate: in Hz, used only to convert window_seconds to samples.
        stretch: 1.0 is no change, 8.0 is eight times longer. Values below 1
            compress and work, but the algorithm is really built for >= 1.
        window_seconds: window length. Short (0.05) keeps some of the original
            character; long (1.0+) smears everything into a drone. 0.25 is a
            good default.
        rng: optional numpy Generator, for reproducible output.

    Returns:
        float array, shape (channels, new_frames).
    """
    if samples.ndim != 2:
        raise ValueError("samples must have shape (channels, frames)")
    if stretch <= 0:
        raise ValueError("stretch must be positive")
    if rng is None:
        rng = np.random.default_rng()

    samples = samples.astype(np.float64, copy=True)
    nchannels, nsamples = samples.shape

    window_size = optimize_windowsize(max(16, int(window_seconds * samplerate)))
    window_size = (window_size // 2) * 2
    half = window_size // 2

    # Fade out the last 50 ms of the input so the final frame doesn't end on a
    # hard edge, which the FFT would smear across the whole frame as a click.
    fade = min(max(16, int(samplerate * 0.05)), nsamples)
    samples[:, nsamples - fade:] *= np.linspace(1.0, 0.0, fade)

    window = make_window(window_size)
    displace = half / stretch  # how far the *input* pointer moves per frame

    nframes = max(1, int(np.ceil(nsamples / displace)))
    output = np.zeros((nchannels, nframes * half + window_size))

    prev = np.zeros((nchannels, window_size))
    pos = 0.0

    for i in range(nframes):
        start = int(pos)
        chunk = samples[:, start:start + window_size]
        if chunk.shape[1] < window_size:
            chunk = np.pad(chunk, ((0, 0), (0, window_size - chunk.shape[1])))

        spectrum = np.fft.rfft(chunk * window)

        # Discard the phases, keep the magnitudes.
        magnitudes = np.abs(spectrum)
        phases = rng.uniform(0.0, 2.0 * np.pi, magnitudes.shape)
        # Bin 0 (DC) and Nyquist are real; randomizing them injects a DC offset,
        # so keep their sign and leave the magnitude alone.
        phases[:, 0] = 0.0
        if window_size % 2 == 0:
            phases[:, -1] = 0.0

        frame = np.fft.irfft(magnitudes * np.exp(1j * phases), n=window_size)
        frame *= window

        # Overlap-add: second half of the previous frame + first half of this one.
        output[:, i * half:(i + 1) * half] = frame[:, :half] + prev[:, half:]
        prev = frame

        pos += displace
        if pos >= nsamples:
            nframes = i + 1
            break

    output = output[:, :nframes * half]
    return np.clip(output, -1.0, 1.0)


# --- WAV I/O (16-bit PCM, stdlib only) ------------------------------------


def read_wav(path: str) -> tuple[int, np.ndarray]:
    with wave.open(path, "rb") as f:
        if f.getsampwidth() != 2:
            raise ValueError("only 16-bit PCM WAV files are supported")
        rate = f.getframerate()
        nchannels = f.getnchannels()
        raw = f.readframes(f.getnframes())
    data = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
    return rate, data.reshape(-1, nchannels).T.copy()


def write_wav(path: str, samplerate: int, samples: np.ndarray) -> None:
    interleaved = samples.T.reshape(-1)
    pcm = np.clip(interleaved * 32767.0, -32768, 32767).astype("<i2")
    with wave.open(path, "wb") as f:
        f.setnchannels(samples.shape[0])
        f.setsampwidth(2)
        f.setframerate(samplerate)
        f.writeframes(pcm.tobytes())


def main() -> None:
    p = argparse.ArgumentParser(description="PaulStretch extreme time-stretch")
    p.add_argument("input")
    p.add_argument("output")
    p.add_argument("-s", "--stretch", type=float, default=8.0)
    p.add_argument("-w", "--window", type=float, default=0.25,
                   help="window length in seconds (default 0.25)")
    p.add_argument("--seed", type=int, default=None)
    args = p.parse_args()

    rate, samples = read_wav(args.input)
    rng = np.random.default_rng(args.seed)
    out = paulstretch(samples, rate, args.stretch, args.window, rng)
    write_wav(args.output, rate, out)

    print(f"{samples.shape[1] / rate:.2f}s -> {out.shape[1] / rate:.2f}s")


if __name__ == "__main__":
    main()
