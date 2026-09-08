# Open issues

Known, unresolved issues, and what the bench owes the frame model.

## The frame model is host-tested, partially heard

Everything below `../archive/history/REVIEW_responsiveness.md` proposed is implemented and green
on the host and links for the device. A first bench pass (single active step,
`steps=1`, no crossfade march) confirmed the instrument runs clean at 480 MHz
with ring-out removed; the multi-head march cases below are still owed.

### Measured on the bench (480 MHz, single sounding head)

The HLTH/COST split finally gets these off the board (they were truncated
before). At `steps=1`, `act=1 arm=1 free=8`, `du=0 late=0 clip=0`, `stk=176`:

| frame | `COST` (samples/frame) | steady `max_us` | note |
|---|---|---|---|
| 4096  | 128     | ~1177 µs  | |
| 16384 | 672–673 | ~14500 µs | a control-driven **pair** re-render spiked `max_us` to **28721 µs** (~2× a frame) the moment the size changed (`rfr=1`), then settled; `slack=8192` absorbed it, `du=0` |

`isr_us ≈ 18700 µs/s` sustained (~1.9% duty) at one head, block 32. The 16384
`COST` (672) ≈ 14 ms matches the measured single-frame `max_us` almost exactly,
so the cost model is accurate at 16384. **The real worst case at 16384 is the
~28 ms pair re-render, not the ~14 ms single** — relevant if the F5b bound is
tightened. These replace the 400 MHz / ~18 ms guesses below.

### Still owed

- **Raw cut level by ear** at fade 0, 4096 and 16384. Host measurement: the
  first 5 ms after a cut now sits within the material's own wander (deepest
  −2 to −9 dB in 5 ms windows) instead of −34/−62 dB.
- **`du` (holds) and `late` at 16384** under a 0.25 s march with a full 0.5
  crossfade (cur + inc both render 16384 pairs through the seam) — the
  multi-head throughput case, NOT covered by the single-step pass above. The
  cost-modelled host harness passes; measure real `du` on the bench.
- **`isr_us` at block 32 at a crossfade seam (two gated heads).** The ISR now
  blends two frame reads per gated head per sample (it used to read one ring
  sample); the main loop lost its whole per-sample kernel in exchange. Max
  concurrent is two sounding heads plus one armed rendering ahead.
- **480 MHz** clean on the codec and QSPI paths (`pod.Init(true)`).
- **Refresh feel:** turning position/stretch/frame on a sounding head. `rfr`
  counts re-renders; `slack` should stay positive. If a fast knob sweep drives
  `du` up, the refresh threshold (0.002 in position) or the minimum gap
  (4 × cost, ≥ 10 ms) is the knob.

## Surfaced by the alpha4 cleanup review (open, not fixed)

Found while reviewing the codebase for cleanup, deliberately NOT changed —
each one alters what the instrument does or how it feels, so it is a decision
and/or a bench item, not a cleanup. Recorded so they are not lost.

- **The sample blob's rate is never applied.** `tools/wav2raw.py` promises
  "the firmware reads the header and scales playback accordingly";
  `decodeSampleBlob()` returns `rate` and `dreamosc.cpp` ignores it (and
  `sd_source.h` returns `out_samplerate` that nothing consumes). A non-48 kHz
  blob plays pitch-shifted. Applying it is a sound change.
- **QSPI and SD disagree on the `Source` length rule.** QSPI wrap-pads to
  `SOURCE_LEN` (10 s) and sets `len` to that; `sd_source.h` sets `len` to the
  file's sample count. Position 0..1 spans different material under each, and
  every step position tuned by ear so far is against the padded scale. Decide
  the rule once (a shared `finishSource()` both loaders call) before wiring SD.
- **Control poll is blocked behind renders** (`dreamosc.cpp` main loop): the
  1 ms `processControls()` check sits between `service()` calls, so it waits
  for the in-flight render — ~1.2 ms at 4096, 14.5 / 28.7 ms (single / pair)
  at 16384. libDaisy's encoder debounce needs the A-phase low on two
  consecutive calls, so a fast spin at 16384 can drop or mis-sign detents.
  Fix: read the encoder + buttons from a 1 kHz timer IRQ below audio priority
  into atomics. Sound-neutral. Verify with a detent counter on the KNOB line.
- **Refresh-gap floor of one hop** (`stretch_core.h` `service()`:
  `gap = max(4·cost, w/2, 480)`): a knob moved just after a refresh cannot
  refresh again until a hop later, which can push the move one boundary
  further out — worst case ~2 hops, not the 1 hop THEORY §7 states. Dropping
  the `w/2` floor allows ≤4 refreshes/hop at 4096 (CPU has room). Depends on
  the poll fix above. Verify `rfr` up, `du` 0, `slack` > 0.
- **Armed-head pre-roll pairs re-render on every gap while the next step's
  knob moves**, though only the last pair before `due` matters (28.7 ms of
  throwaway work per gap at 16384). Defer an armed head's REFRESH until
  `due − clock` is within ~8×cost.
- **Tier-B refactors** — done as bit-identical commits (host goldens 4/4,
  suite green, device symbols smaller): `chooseRender()`, `pickWork()` +
  `CostModel`, `isArmedState`, `qAt`, `ssClampPos`, `cosAtF`, dead `Staged`
  fields (676d4a7); per-block `Block{seamGeom, steps}` snapshot with F9's
  seam-time re-check removed, `nextOnset_` on the Sequencer instead of `due_`
  on every Head, `retireOutgoing()`/`armAfter()` in `tick()` (next commit).
  Correction to the review: the `activeSteps` snapshot IS bit-identical for
  every input — the main loop is the only writer and cannot preempt the ISR,
  so the value is constant across a `render()` on the device, and the host
  runs the block-top check before every sample; the seam-time re-check was
  dead in both. Still open: `render()` re-deriving `slack` after its F7
  re-sync (a latency item, below); `gUnderruns`/`gClips` as Sequencer members
  (three platform files of churn for no latency value — skipped);
  interleaving/relocating the blend tables (wait for `isr_max` bench data).
- ~~Host tests run per-block housekeeping per SAMPLE~~ — done:
  `test/test_block_cadence.cpp` drives the core at the device's `render(buf,
  32)` cadence (`drive_blocks` in `test_support.h`). Finding: after go-live the
  block-32 and per-sample renders are **bit-identical** under a drained
  producer (the housekeeping cadence changes when a descriptor is dropped or a
  deadline restated, never which frame plays); the only cadence-dependent
  moment is startup, where the first block is `late` silence because the
  producer cannot run inside a block (go-live at sample 32 vs 1). The
  scheduling-sensitive cases (F3, F9, the 16384 crossfade march, the
  live-control latencies) pass at block cadence. Note there is no "control
  landed mid-block" window on the device either: the main loop is the only
  writer of the controls and cannot preempt the ISR, so a control is constant
  for a whole `render()`.
- **On-board fingerprint.** `make PROFILE=1` now prints `crc=` on the COST
  line: a CRC-32 of a fixed-config 1.5 s render taken at boot with audio
  stopped (`renderFingerprint()` in `stretch_core.h`, pinned to the harness by
  a test). Two firmware builds that print the same `crc` render the same
  samples on the board — the host goldens cannot say that (the M7 build fuses
  multiply-adds, the host build does not). Procedure: flash the reference
  build, read `crc`, flash the candidate, compare. `isr_max` (worst single
  audio callback, µs) is on the same line for the seam-cost bench item above.

## Code-review findings on the frame model (all remediated, all guarded)

Each finding from the review of 6917a0a has a test in `test/test_findings.cpp`
that reproduced it on the host (measuring the rendered audio) and now guards
the fix. What the bench still owes here:

- **F5b, throughput under a full crossfade:** growing to 16384 while cur + inc
  both render pairs at a seam is bounded holds in the following 2 s. A hold is a
  spectral freeze on one head, never silence. This is CPU, not scheduling (the
  pre-roll pairs are several old hops of work). The single-head bench pass now
  pins the 16384 cost: **~14 ms per frame, ~28 ms per pair re-render at 480 MHz**
  (measured, replacing the ~18 ms/400 MHz model). Still owed: `du` under the
  *two-head* 0.5-crossfade march (single-step ran `du=0`, but that is one head,
  not the seam case); tighten the test bound from that measurement.
- **F8, ADC jitter:** position is now written from the raw pot. If a parked,
  engaged knob shows `rfr` ticking every hop, the pot's noise exceeds the
  0.002 refresh threshold; raise it or re-introduce a light smoother.

## Not measured

- AXI SRAM vs SDRAM for the FFT scratch (placed per ST's guidance; the speedup
  was never confirmed).

## Closed by measurement (recorded so they are not re-tried)

- **ShyFFT vs CMSIS-DSP `arm_rfft_fast_f32` at 16384** — not a candidate.
  `arm_rfft_fast_init_f32` supports 32..4096 only, in both the pinned libDaisy
  copy and upstream `main`; it cannot serve 8192 or 16384. ShyFFT stays.

- **Compiler flags `-O3` + `-fmove-loop-invariants` + `-fno-math-errno`**
  (tried 2026-09-08, commit 856fac4, reverted). Bench at 480 MHz, frame 16384,
  `steps=1`, `fade=0.5`, `dur=260 ms`: `COST 16384` **704–720** vs alpha4's
  672–673 (+5–7% slower), `COST 4096` **142** vs 128 (+11% slower), pair
  re-render `max_us` 30,047–30,132 vs 28,721 (+5%); `stk` 176 → 216; `du=0
  late=0 clip=0` throughout. Binary +3.7%. The render is memory-bound at
  16384 (ShyFFT ping-pongs 128 KB through the 16 KB D-cache per pass), so
  arithmetic-side flags cannot help there, and at 4096 `-O3`'s unrolling made
  the codegen worse outright. GCC's tree-level hoisting (`-ftree-loop-im`,
  on at -O2) was already doing what `-fmove-loop-invariants` re-enables.
  Caveat: the alpha4 reference was a single-head run (`fade=0`); the +5–7%
  at 16384 may include some two-head cache eviction, but the +11% at 4096
  cannot. libDaisy's `-O2` stands. `-ffast-math` was never tried and must
  not be: FP reassociation changes the rendered audio.

## Closed by construction (recorded so they are not re-opened)

- Cushion depth vs render cost: there is no cushion. The producer deadline is
  `h − phase` and the scheduler is earliest-deadline-first.
- The 32-bit wrap in `svc_us`/`max_us`: `System::GetUs()` wraps every 21.5 s
  (TIM2 at 200 MHz). The profiler now takes raw tick deltas.
- The one-lookahead hole in single-step mode and the 341 ms startup silence:
  a pool head per visit; startup is one pre-roll pair.
