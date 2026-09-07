# Open issues

Known, unresolved issues, and what the bench owes the frame model.

## The frame model is host-tested, not yet heard

Everything below `REVIEW_responsiveness.md` proposed is implemented and green
on the host (72 test cases) and links for the device, but none of it has been
heard on the Pod. In `make PROFILE=1` terms, the bench owes:

- **Raw cut level by ear** at fade 0, 4096 and 16384. Host measurement: the
  first 5 ms after a cut now sits within the material's own wander (deepest
  −2 to −9 dB in 5 ms windows) instead of −34/−62 dB.
- **`du` (holds) and `late` at 16384** under a 0.25 s march with ring-out at
  2 s and 8 s. The cost-modelled host harness passes at the old bench cost
  (~18 ms per 16384 frame); the real number at 480 MHz is unknown. `cost=`
  on the HLTH line is the measured per-size render cost in samples.
- **`isr_us` at block 32 with six gated heads.** The ISR now blends two frame
  reads per gated head per sample (it used to read one ring sample); the main
  loop lost its whole per-sample kernel in exchange.
- **480 MHz** clean on the codec and QSPI paths (`pod.Init(true)`).
- **Refresh feel:** turning position/stretch/frame on a sounding head. `rfr`
  counts re-renders; `slack` should stay positive. If a fast knob sweep drives
  `du` up, the refresh threshold (0.001 in position) or the minimum gap
  (4 × cost, ≥ 10 ms) is the knob.

## Code-review findings on the frame model (all remediated, all guarded)

Each finding from the review of 6917a0a has a test in `test/test_findings.cpp`
that reproduced it on the host (measuring the rendered audio) and now guards
the fix. What the bench still owes here:

- **F5b, throughput at the cap:** growing to 16384 with six gated heads is
  43 holds in the following 2 s at the modelled cost (~18 ms per 16384 frame
  at 400 MHz). A hold is a ~43 ms spectral freeze on one head, never silence.
  This is CPU, not scheduling (six pre-roll pairs are more than five old hops
  of work). Re-measure `du` on the bench at 480 MHz after a frame-size growth
  with ring-out at 8 s; tighten the test bound from the real cost.
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
