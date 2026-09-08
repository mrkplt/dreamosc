# Open issues

Known, unresolved issues, and what the bench owes the frame model.

## The frame model is host-tested, partially heard

Everything below `REVIEW_responsiveness.md` proposed is implemented and green
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

- **Compiler flags (`-O3`, `-fmove-loop-invariants`, `-fno-math-errno`)** —
  `dreamosc/Makefile`, host-tested build only. Sound-neutral by construction
  (no FP reassociation; `-ffast-math` deliberately off) but the speedup is a
  hypothesis. Compare `COST` and `max_us` against the alpha4 numbers above
  (4096: 128 / ~1177 µs; 16384: 672–673 / ~14500 µs single, 28721 µs pair)
  and confirm `du=0 late=0 clip=0` at both sizes. Code grew 107,360 →
  111,288 B (SRAM 48.5% → 49.3%). Note `stk` cannot see render-depth stack
  growth from the extra inlining: `profSampleStack()` samples MSP between
  `service()` calls, never inside a render, so `stk` never sees render depth
  (a profiler gap in its own right). Watch for a hang rather than a number.
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
  `du` up, the refresh threshold (0.001 in position) or the minimum gap
  (4 × cost, ≥ 10 ms) is the knob.

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

- ShyFFT vs CMSIS-DSP `arm_rfft_fast_f32` at 16384 (the 64 KB scratch is four
  times the D-cache; the bit-reversal and early passes miss).
- AXI SRAM vs SDRAM for the FFT scratch (placed per ST's guidance; the speedup
  was never confirmed).

## Closed by construction (recorded so they are not re-opened)

- Cushion depth vs render cost: there is no cushion. The producer deadline is
  `h − phase` and the scheduler is earliest-deadline-first.
- The 32-bit wrap in `svc_us`/`max_us`: `System::GetUs()` wraps every 21.5 s
  (TIM2 at 200 MHz). The profiler now takes raw tick deltas.
- The one-lookahead hole in single-step mode and the 341 ms startup silence:
  a pool head per visit; startup is one pre-roll pair.
