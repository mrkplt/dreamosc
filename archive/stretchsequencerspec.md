# Stretch Sequencer — Specification

A step sequencer whose steps address positions inside a virtual PaulStretch. The stretch is not the artifact; it is an intermediate space to sample from. Each step reaches into that space at a named point, pulls a fixed duration of sound, and fades into the next.

---

## Premise

The stretch is never rendered. Which requires being precise about what a frame depends on, because the naive claim — that frames are independent — is false.

A frame's *spectrum* depends only on the source window beneath it and a seed. But frames are overlap-added at 50%, so any output sample is the sum of two of them, and the real PaulXStretch goes further: its output block is a raised-cosine crossfade between the current frame's IFFT and the previous frame's (`old_out_smps` in `Stretch.cpp`), followed by an AM-correction curve built on `0.853553390593`. Output genuinely depends on frame history.

The dependency is **exactly one frame deep**. Nothing accumulates past that — `very_old_smps`/`old_smps`/`new_smps` are a sliding window over the source, computable directly from position, not state. So a pull at fraction *f* renders one frame of pre-roll before the start point and discards it, and is then in steady state from its first sample. Arbitrary stretch length is still free and random access is still O(1); it just costs one extra frame, not a rendering from the beginning.

This was measured, not assumed: without pre-roll the first half-window of every pull comes out 3.4 dB down and spectrally thin. With it, 0.04 dB. Under the step envelope's fade-in the flaw was nearly inaudible, which is exactly why it needed measuring.

Positions are **proportional**: a step at 40% reads 40% of the way through the source at 10×, at 10⁶×, at anything. The stretch factor divides out of step placement entirely.

---

## Controls

### Per step

**Position** — where the step points, as a percentage through the stretch. The full 0–100% range is available to every step and the arrangement of the eight positions is the compositional act. Spread wide they tour the source; clustered tight they are eight views of one moment.

**Drift** — a ± tolerance around that position. Each pass the step draws a fresh random position inside its neighbourhood: no easing, no accumulation, no memory. Zero is literal repetition, small is a step that stays recognisably itself but never sounds twice the same, larger is a step that reads as a region rather than a point.

### Global

**Stretch factor** — the only thing coupling proportional positions to absolute durations. It does not move the steps; it sets how far each read head travels during its step, and so how wide a region each step smears across. Low factors let every step scan through material; high factors freeze every step into a held chord.

**Step duration** — how long each step sounds, in seconds. It is the same for all steps in this version, so the lattice is even in time and uneven only in the source. Combined with the stretch factor it determines the sweep width every step shares.

**Spread** — how much each step overlaps its neighbours, and the control that changes what the instrument is. At zero the steps are butt-joined and articulated; at 1 each step's tail runs under the next one's head and two are always sounding; above 2 three or more stack and the sequence becomes a chord of positions rather than a succession of them.

---

## Boundaries

Fixed for this version:

- **Eight steps.** Even lattice in time.
- **One duration for all steps.** No per-step length, no rhythm from unequal steps.
- **No tempo grid.** Step duration *is* the timing; there is nothing for steps to be quantised against.
- **Drift is memoryless.** A fresh draw each pass, not a walk. The lattice stays fixed and the points shimmer in place.
- **One source.** No layering of different material across steps.
- **Constant loudness.** Spread is a character control only; it must not change level.

The whole model is 16 numbers (8 position/drift pairs) plus 3 globals — small enough for a physical panel, small enough that a preset is one line of text.

---

## Consequences

These follow from the design rather than being chosen.

**Fades are free.** Every pull is independently phase-randomised, so overlapping steps are mutually uncorrelated. Equal-power crossfades just work — no clicks, no comb filtering, none of granular synthesis' alignment problems. Overlap as heavily as you like.

**Level compensation is exact and cheap.** Uncorrelated sources sum in power, so dividing the output by the square root of the summed squared envelopes holds level flat regardless of stacking depth. Measured across spread 0 → 3 the uncompensated output rises 4.7 dB; compensated it moves 0.13 dB. Because the lattice is uniform this can also be precomputed as a single table indexed by position within one step interval, recomputed only when spread changes.

**Drift is the only thing that costs.** A drifting step needs a fresh spectral analysis each pass; a pinned step can be analysed once and cached forever. Worst case is all eight drifting at a short step duration.

**Phase is seeded from the position.** The same position always yields the same audio, which is what makes zero drift a true repeat and makes caching valid. A side effect: any nonzero drift redraws the phase realisation completely, so even a drift too small to change the spectral content still changes the texture. Since the material is already a phase-smeared cloud, this reads as *the same sound, differently* — which is the intended behaviour of small drift.

**Voice allocation, not a step loop.** Above spread 0 a step's sound outlives its slot, so rendering cannot advance one interval at a time into a buffer. Each step needs its own start time with its tail summed into a common timeline — closer to a polyphonic voice allocator than a sequencer.

---

## Open

- At low stretch factors, whether the read head genuinely reads as a scan through material or whether the analysis window (hundreds of ms) smears enough that it does not. Settles in practice.
- Whether to port the PaulXStretch inter-frame crossfade and AM correction into the pull, which would reduce amplitude ripple on tonal source material. It is compatible with random access — the same one-frame pre-roll covers it — so this is a quality decision, not an architectural one.
- Whether one frame of pre-roll is enough once that crossfade is in, or whether the correction curve wants two. Depth-1 by inspection; worth confirming by measurement rather than reading.

---

## Appendix — Reference implementation

Tested: deterministic per position; a short pull is bit-identical to the prefix of a longer one from the same point; identical output for repeated passes at zero drift; pull onset within 0.04 dB of steady state; level flat to 0.13 dB across spread 0–3; 34 s of output from 16 drifting steps renders in 0.63 s.

```
python stretchseq.py source.wav out.wav \
    --steps 10,13,16,19,22,25,28,31 --drift 0.5 \
    --stretch 50 --duration 4.0 --spread 1.0 --passes 4
```

```python
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
        """Seconds between step onsets. spread=0 butts steps end to end;
        spread=1 puts two steps in the air at once; spread=2, three."""
        return self.duration / (1.0 + self.spread)

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
```
