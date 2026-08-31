"""
stretchseq.py - a step sequencer that addresses positions inside a virtual
PaulStretch.

The stretch is never rendered. Each step names a percentage through the
stretch; the engine pulls audio from that point on demand.

Usage:
    python stretchseq.py source.wav out.wav \
        --steps 10,13,16,19,22,25,28,31 \
        --drift 0.5 \
        --stretch 50 --duration 4.0 --spread 1.0 --passes 4
"""

import argparse
import wave

import numpy as np


# ----------------------------------------------------------------------------
# WAV I/O (16-bit PCM, mono or stereo)
# ----------------------------------------------------------------------------

def load_wav(path):
    with wave.open(path, "rb") as w:
        sr = w.getframerate()
        n_ch = w.getnchannels()
        raw = w.readframes(w.getnframes())
    data = np.frombuffer(raw, dtype=np.int16).astype(np.float64) / 32768.0
    if n_ch > 1:
        data = data.reshape(-1, n_ch).T
    else:
        data = data.reshape(1, -1)
    return data, sr


def save_wav(path, data, sr):
    peak = np.max(np.abs(data))
    if peak > 0.999:
        data = data * (0.999 / peak)
    ints = (np.clip(data, -1.0, 1.0) * 32767.0).astype(np.int16)
    interleaved = ints.T.reshape(-1)
    with wave.open(path, "wb") as w:
        w.setnchannels(data.shape[0])
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(interleaved.tobytes())


# ----------------------------------------------------------------------------
# StretchField - random access into a stretch that is never rendered
# ----------------------------------------------------------------------------

class StretchField:
    """A virtual PaulStretch of the source, addressable by fraction.

    A PaulStretch frame depends only on the source window beneath it and a
    random seed; no state carries between frames. So a point at fraction f of
    a stretch of any length is just source position f * source_length, and
    random access costs the same as sequential access.
    """

    def __init__(self, source, sr, window_sec=0.25, seed=None):
        self.source = np.atleast_2d(source)
        self.n_ch, self.n_src = self.source.shape
        self.sr = sr
        self.W = self._optimize_windowsize(int(window_sec * sr))
        self.W += self.W % 2
        self.H = self.W // 2                      # output hop, fixed
        x = np.linspace(-1.0, 1.0, self.W, dtype=np.float64)
        self.window = (1.0 - x ** 2) ** 1.25
        self.seed = 0 if seed is None else int(seed)

    @staticmethod
    def _optimize_windowsize(n):
        """Round up to the next 5-smooth number so the FFT stays fast."""
        orig = n
        while True:
            n = orig
            for f in (2, 3, 5):
                while n % f == 0:
                    n //= f
            if n < 2:
                return orig
            orig += 1

    def _read(self, start, length):
        """Read from the source with wraparound, so any position is legal."""
        idx = (np.arange(length) + int(start)) % self.n_src
        return self.source[:, idx]

    def pull(self, position, stretch, duration):
        """Return `duration` seconds of stretch, starting at `position`.

        position : fraction in [0, 1) through the stretch
        stretch  : stretch factor (source seconds per output second = 1/stretch)
        duration : output length in seconds
        """
        n_out = int(round(duration * self.sr))
        n_frames = n_out // self.H + 2
        position = position % 1.0
        src_start = position * self.n_src
        src_hop = self.H / float(stretch)          # how far the read head moves

        # The phase seed is derived from the position, not from a running
        # counter, so the same position always yields the same audio. That is
        # what makes zero drift a literal repeat and makes a pinned step
        # cacheable.
        rng = np.random.default_rng([self.seed, int(position * (1 << 44))])

        # Overlap-add means every output sample is the sum of two frames, so a
        # pull that simply starts at frame 0 is missing the contribution of the
        # frame that would have preceded it: its first half-window comes out
        # ~3.4 dB down and spectrally thin. The dependency is exactly one frame
        # deep, so rendering one frame of pre-roll and discarding it puts the
        # pull in steady state from its first sample. Random access stays O(1).
        out = np.zeros((self.n_ch, (n_frames + 2) * self.H + self.W))
        for k in range(-1, n_frames):
            frame = self._read(src_start + k * src_hop, self.W) * self.window

            spec = np.fft.rfft(frame, axis=1)
            mag = np.abs(spec)
            phase = rng.uniform(0.0, 2.0 * np.pi, mag.shape)
            spec = mag * np.exp(1j * phase)
            spec[:, 0] = 0.0                       # DC
            spec[:, -1] = 0.0                      # Nyquist

            frame = np.fft.irfft(spec, n=self.W, axis=1) * self.window
            off = (k + 1) * self.H
            out[:, off:off + self.W] += frame

        return out[:, self.H:self.H + n_out]


# ----------------------------------------------------------------------------
# Sequencer
# ----------------------------------------------------------------------------

class Sequencer:
    """Eight (or any number of) steps on an even time lattice.

    Per step : position (% through the stretch), drift (+/- % tolerance).
    Global   : stretch factor, step duration, spread.
    """

    def __init__(self, field, positions, drifts,
                 stretch=50.0, duration=4.0, spread=1.0, seed=None):
        self.field = field
        self.positions = np.asarray(positions, dtype=np.float64) / 100.0
        drifts = np.broadcast_to(np.asarray(drifts, dtype=np.float64),
                                 self.positions.shape)
        self.drifts = drifts / 100.0
        self.stretch = float(stretch)
        self.duration = float(duration)
        self.spread = float(spread)
        self.rng = np.random.default_rng(seed)

    @property
    def interval(self):
        """Seconds between head starts, as a fraction of duration. spread is
        0..1: 0 fires all heads together (interval 0), 1 spaces them end-to-end
        (a head starts as the previous one ends)."""
        s = min(max(self.spread, 0.0), 1.0)
        return self.duration * s

    def _envelope(self, n):
        """Equal-power (sine) fade in and out over the whole step."""
        return np.sin(np.pi * (np.arange(n) + 0.5) / n)

    def render(self, passes=1):
        sr = self.field.sr
        n_steps = len(self.positions)
        step_n = int(round(self.duration * sr))
        hop_n = int(round(self.interval * sr))
        total = hop_n * (n_steps * passes - 1) + step_n

        out = np.zeros((self.field.n_ch, total))
        power = np.zeros(total)                    # sum of squared envelopes
        env = self._envelope(step_n)

        for i in range(n_steps * passes):
            s = i % n_steps
            d = self.drifts[s]
            pos = self.positions[s] + (self.rng.uniform(-d, d) if d > 0 else 0.0)
            pos = float(np.clip(pos, 0.0, 1.0))

            audio = self.field.pull(pos, self.stretch, self.duration)
            start = i * hop_n
            out[:, start:start + step_n] += audio * env
            power[start:start + step_n] += env ** 2

        # Constant loudness: uncorrelated sources sum in power, so dividing by
        # the square root of the summed squared envelopes holds level flat
        # however many steps are stacked.
        out *= 1.0 / np.sqrt(np.maximum(power, 1e-9))
        return out


# ----------------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("input")
    p.add_argument("output")
    p.add_argument("--steps", default="10,13,16,19,22,25,28,31",
                   help="step positions as percentages through the stretch")
    p.add_argument("--drift", default="0.0",
                   help="one value for all steps, or a comma list per step")
    p.add_argument("--stretch", type=float, default=50.0)
    p.add_argument("--duration", type=float, default=4.0,
                   help="step duration in seconds (global)")
    p.add_argument("--spread", type=float, default=1.0,
                   help="0 = butt-joined, 1 = two steps sounding, 2 = three")
    p.add_argument("--passes", type=int, default=1)
    p.add_argument("--window", type=float, default=0.25,
                   help="analysis window in seconds")
    p.add_argument("--seed", type=int, default=None)
    args = p.parse_args()

    positions = [float(v) for v in args.steps.split(",")]
    drifts = [float(v) for v in args.drift.split(",")]
    if len(drifts) == 1:
        drifts = drifts * len(positions)

    source, sr = load_wav(args.input)
    field = StretchField(source, sr, window_sec=args.window, seed=args.seed)
    seq = Sequencer(field, positions, drifts,
                    stretch=args.stretch, duration=args.duration,
                    spread=args.spread, seed=args.seed)
    save_wav(args.output, seq.render(passes=args.passes), sr)

    print(f"{len(positions)} steps x {args.passes} pass(es), "
          f"interval {seq.interval:.3f}s -> {args.output}")


if __name__ == "__main__":
    main()
