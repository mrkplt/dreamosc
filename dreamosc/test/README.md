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
the corrected spread model (`interval = duration * spread`, pattern length
`(SS_STEPS-1)·dur·spread + dur`), constant-loudness invariant (level roughly flat
across spread), spread clamped to `[0,1]`, and renders across 44.1/48/96 kHz.
Catch2 is vendored as `catch_amalgamated.hpp` (v2.13.10, single header — no build
step, no package manager).

**2. Golden regression — `../host/regression.py`.**
The C++ core vs the Python reference (`stretchseq.py`). Not bit-equality — the two
use different RNGs, so phases differ by design. Asserts length, RMS level, and
spectral-envelope shape within documented tolerances across a spread sweep. This
is the guard that the C++ port still matches the reference behavior.

## Adding a test

- A new DSP property → a `TEST_CASE` in `test_stretch_core.cpp`. Prefer an
  invariant (bounds, energy, length, determinism) over a magic expected number.
- A new behavior the Python reference also models → add a case to `CASES` in
  `regression.py`.
- Tolerances are set from measured agreement **with a written reason** — see the
  comment on `RMS_DB_TOL`. Do not loosen a tolerance to hide a regression; if a
  gap is an implementation difference, say why in the comment.
