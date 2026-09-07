# Tests

Host-side tests for the platform-free DSP core. The whole point of keeping
`stretch_core.h` / `shy_fft.h` free of Daisy/Arduino headers is that they run on
a desktop compiler, where tests are fast and a regression is caught before it
reaches hardware. This follows standard embedded practice: **isolate logic from
hardware, test the logic on the host.**

## Run everything

```
./run.sh          # from dreamosc/test/ — runs both layers, exits non-zero on failure
```

Needs the host venv (`dreamosc/host/.venv` with numpy) for the regression layer.
Create it once:

```
python3 -m venv ../host/.venv && ../host/.venv/bin/pip install numpy
```

## Two layers

**1. Unit tests — `test_stretch_core.cpp` (Catch2).**
Properties of a tuned/randomized algorithm, not exact sample values:
determinism (same config → identical output), no `NaN`/`Inf`, output bounded,
seam continuity at every frame size (the "silence between pieces" guard),
pre-roll level, no startup or single-step hole, constant loudness across fade,
live-control latency (frame size, position, stretch reach the sounding head
within a hop or two), scheduling under a cost-modelled producer
(`render_costed`: each render charges its modelled cost; holds are counted and
must never become silence), ring-out cap and expiry, drift, and renders across
44.1/48/96 kHz. Catch2 is vendored as `catch_amalgamated.hpp` (v2.13.10, single
header — no build step, no package manager).

**2. Two drivers in `test_support.h`.** `render()` drains `service()` between
every sample (an infinitely fast producer: sees baked content, never
scheduling). `render_costed()` gives the producer a time budget and charges
each render `cost(w)`; plug in the bench's per-size cost to reproduce device
scheduling on the host. The former Python golden regression is retired.

## Adding a test

- A new DSP property → a `TEST_CASE` in `test_stretch_core.cpp`. Prefer an
  invariant (bounds, energy, length, determinism) over a magic expected number.
- A code-review finding → a `TEST_CASE` in `test_findings.cpp` tagged
  `[finding][!mayfail]` that reproduces it AND measures its effect on the
  rendered audio (click detector, silent windows, first-difference sample).
  `[!mayfail]` reports "failed as expected" without failing the gate; drop the
  tag once the finding is remediated so the test becomes a guard. ISR
  preemption is simulated through the `SS_HOOK` points (test build only).
  Each test writes its audio to `/tmp/dreamosc_findings/*.wav` for listening.
- Tolerances are set from measurement **with a written reason** (see the
  seam-continuity test's self-calibrated floor). Do not loosen a tolerance to
  hide a regression; if a gap is an implementation difference, say why.
