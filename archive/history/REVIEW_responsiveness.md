# Architecture review: responsiveness, gaps, and underruns

Scope: `stretch_core.h`, `dreamosc.cpp`, the test harness, and the libDaisy
configuration they sit on, read against the spec (`archive/stretchsequencerspec.md`),
Fizzy #155, and `OPEN_ISSUES.md`. Sound character is deliberately set aside;
the question is why the instrument keeps needing bigger buffers and still gaps.

Every claim is tagged:

- **[measured]** run on the host with `host/seam_probe.cpp` (uncommitted review aid).
- **[read]** verified by reading source (ours or libDaisy's).
- **[bench]** a hypothesis that only `make PROFILE=1` can confirm.

## Short version

The hunch is right. The two-clock principle in #155 (reader owns time, writer
owns supply) is sound and the code honours it. What grew around it is a
sample-granular cushion layer that re-buffers audio the synthesis already
produces in whole frames, plus a scheduler that services that cushion in head-
index order with no notion of deadline. The cushion depth is then the only knob,
and it is simultaneously the knob-to-ear latency and the underrun margin, so
every underrun fix costs latency and every latency win costs underruns. That is
the loop `OPEN_ISSUES.md` describes.

Separately, three things the design's own text promises are not what the code
does, and one of them is the "silence between pieces":

1. A raw cut is not amplitude-continuous. Every head fades in from zero over its
   first hop because the spec's one-frame pre-roll was dropped. **[measured]**
2. Position and frame size are latched at pre-warm, not live. **[read]**
3. The chip runs at 400 MHz, not the 480 MHz every doc assumes. **[read]**

## What is right and should stay

- The two-clock split: the ISR owns time and the seam; rendering is on the main
  loop. Keep it.
- `seamPhase()` as a pure, tested decision function.
- Platform-free cores with a real host suite; the profiler directive.
- Hot FFT scratch in AXI SRAM via our own linker script. SDRAM is configured
  cacheable and bufferable by libDaisy's MPU region 1, so sequential frame
  reads from SDRAM are served from L1. **[read]** `libDaisy/src/sys/system.cpp:538-545`
- Release/acquire discipline on the SPSC arm and release queues.

## Findings

### 1. The raw cut fades in from silence (pre-roll was dropped) — [measured]

`Head::arm()` renders one frame and sets `in_.old = nullptr` ("first hop rises
from silence"). `laneSample()` with `old == nullptr` is `cur[hop+phase] * (1-a)`
with `a` running 1 to 0, so the first hop of every head is a raised-cosine fade
in from zero. The spec (`stretchsequencerspec.md`, "Premise") says exactly this
is why a pull must render one frame of pre-roll and discard it: "without pre-roll
the first half-window of every pull comes out 3.4 dB down". #155 then asserted
amplitude at a raw cut is "continuous by construction". It is not, because the
incoming head is not in steady state when its gate opens.

Probe, 8 steps, 1 s dwell, stretch 50, `fade 0`, `service()` drained fully
between samples so this is baked content, not scheduling:

| frame | level at the cut | time more than 3 dB under steady state |
|-------|------------------|----------------------------------------|
| 4096  | −34 dB           | 20–35 ms                               |
| 16384 | −62 dB           | 155–165 ms                             |

At `fade 0.25` the crossfade envelope masks most of it (deepest 25 ms window −5 dB
at 4096, −1 dB at 16384), so this is a raw-cut and short-fade problem. It scales
with frame size, which is why large windows sound worse at seams.

Fix: `arm()` renders two frames (the pre-roll frame into `old`, then `cur`), so
the head's first emitted sample is mid-stream. One extra FFT per onset, during
pre-warm, off the critical path. This is a sound change (it removes a dip), so it
is a proposal for the bench, not an edit. The same fix makes the fade-0 ruling in
#155 true.

### 2. Position and frame size are latched, not live — [read]

#155: "Live position/stretch/frame-size, NO latching (a dwell can be a minute; a
latched knob would be dead that whole time)." Stretch is done right: the lane
holds `const float* stretch` and reads it at every `renderFrame`. Position is
consumed once, in `arm()`, into `in_.srcPos`. Frame size is snapshotted in
`arm()` into `w_/h_/passes_/win_/synthGain_`, with a 64 KB window memcpy per arm.
So on the sounding head, the frame knob takes effect at that step's next visit,
up to a full dwell later (60 s at the top of the range), and the position knob
never moves the head you are hearing.

The snapshot exists to dodge the fast-scroll race (a head rendering while
`setWindow()` rewrites the shared `gWindow`). The race is only possible because
`gWindow` is mutable. Precompute all seven window curves and gains once at init
(about 32K floats total, 128 KB; fits AXI beside `gWork`/`gSpec`, or SDRAM since
it is a sequential read). #155 ruled out "8 window LUTs in AXI (8x64KB busts the
480KB region)"; that arithmetic assumed every table at the maximum size. The
sizes are 16384 down to 256, so the sum is about 130 KB, not 512 KB. Then nothing
is ever rewritten, there is no race to snapshot against, no per-head `win_` copy,
and each render can read the live size. A size change mid-life is then a raw
spectral cut (render a fresh pre-roll pair at the new size), which the design
already accepts. Note the burst: at 16384 with several heads sounding, a size
change means two FFTs per head at once; measure it.

For position: keep the head's base position separate from its accumulated travel
(`srcPos = position*len + travel`), read `position` at each render, and the knob
moves the sounding head at its next frame. Drift stays a per-fire draw.

### 3. The firmware runs at 400 MHz — [read]

`pod.Init()` is called with no argument. `DaisyPod::Init(bool boost = false)` maps
to `System::Config::Defaults()`, which selects `FREQ_400MHZ`; `Boost()` selects
480. `dreamosc.cpp:366`, `libDaisy/src/daisy_pod.h:39`, `libDaisy/src/sys/system.h:35-49`.
`hardware_spec.md` and CLAUDE.md both state 480 MHz.

The profiler's own numbers confirm it. `System::GetUs()` is `TIM2 CNT / (2×PCLK1 / 1e6)`.
At 400 MHz sysclk, PCLK1 is 100 MHz, the timer runs at 200 MHz, and the 32-bit
counter wraps every 21.47 s (21,474,836 µs). A `GetUs()` delta taken across the
wrap is `2^32 − 21,474,836 + real = 4,273,492,460 + real`. `OPEN_ISSUES.md` recorded
`max_us = 4,273,496,017`, which is that constant plus 3,557 µs, a normal single
service call at 4096. At 480 MHz the constant would be 4,277,071,599 and the
recorded value would be impossible. So the "32-bit wrap during a starved second"
is not evidence of a hang; it is the timer period, and it poisons `svc_us`,
`avg_us` and `max_us` once every 21 s. It also means every `max_us` on record
was measured with 20% less CPU than the docs assume.

Two actions: `pod.Init(true)` (Electrosmith's supported boost mode; verify on the
bench that the codec and QSPI paths stay clean), and in the profiler take
`GetTick()` deltas and convert the difference, which is wrap-safe. Same bug in
`profIsrUs`.

### 4. The per-sample ring is redundant with the frames it is filled from — [read]

Synthesis produces whole frames; the output is a per-sample raised-cosine blend of
two of them. The current design then re-buffers that blend, sample by sample,
into a per-head ring via `emit()`/`laneSample()`, and the ISR reads the ring. The
ring buys nothing the frames do not already provide, and it costs:

- A second per-sample kernel on the main loop, per head: two float divisions,
  two interpolated LUT reads, two SDRAM frame reads and one SDRAM ring write per
  sample (`laneSample`/`emit`). With six renderers that is six times 48k samples
  per second of main-loop work before any FFT.
- The cushion. `fillTarget()` is the amount of already-blended future the ISR
  drains, so it is added to every control's latency. #155 chose the per-sample
  kernel precisely so the cushion could be ~10 ms; the frame-proportional commit
  then set it to a full hop anyway. The build now pays for both: the fine-grained
  kernel and a hop-sized cushion.
- Five window-lengths of SDRAM per head (`2W + 2W ring + W window`).

Stretch latency today is: wait for the sounding head's next `renderFrame` (up to
one hop), plus the cushion (one hop), plus one hop of blend before the new frame
dominates. Two to three hops: 85–130 ms at 4096, 340–510 ms at 16384.

Replace the ring with three frame slots per head (`old`, `cur`, `next`). The ISR
blends directly from `old`/`cur` (about ten flops per sample with precomputed
per-hop coefficient tables; no divisions). The producer's whole contract becomes
"`next` is rendered before `phase` reaches `h`". Stretch is read at that render,
which can be as late as the deadline allows (just-in-time, margin = measured
render cost), so latency is at most one hop plus the blend, and there is no
cushion to size. Memory per head drops from 5W to 3W. `ring_`, `wr_`/`rr_`,
`SS_SLICE`, `fillTarget()`, `emit()`, `topUp()`, `takeMinFill()` and the whole
open issue go away. The one-hop floor is PaulXStretch's own: a frame is what it
is until the next one. The ISR does gain work here (two frame reads per gated
head per sample instead of one ring read), so re-read `isr_us` after the change;
the main loop loses the whole per-sample kernel in exchange.

### 5. The scheduler is index-ordered and deadline-blind — [read]

`Sequencer::service()` returns after the first head in index order whose fill is
below target, and it emits at most 128 samples, rendering a whole FFT inline if a
hop boundary falls inside that slice. Consequences:

- A remnant in slot 13 is only serviced after every step head in 0–7 is at
  target. Nothing weighs who is closest to starving.
- One 16384 render blocks everything for its whole duration (about 18 ms per
  `OPEN_ISSUES`, at 400 MHz): every other head's ring drains, the control poll
  is skipped, the encoder misses detents. The wall-clock poll was added to
  survive this; it treats the symptom.
- The bench signature in `OPEN_ISSUES.md` is the tell: at 4096 with three or
  four remnants, `du` climbs into the thousands while `max_us` stays ~3600. No
  single call is long, yet heads starve. That is scheduling and protocol, not
  CPU exhaustion. It is exactly the hunch that prompted this review.

Under the frame model every head has an exact deadline (`h − phase` samples).
Service the smallest one first (earliest-deadline-first over at most six heads is
a six-element scan). Report slack in the profiler; a negative slack is then a
real overload, not a scheduling artifact. If control-poll jitter still matters at
16384, split `renderFrame` into resumable stages (window+forward FFT, phase
draw, inverse+copy) and poll between stages.

### 6. Ring-out adoption guarantees a gap and a skip at every ring-out seam — [read]

Sequence at `d.end` with ring-out on: the ISR pushes a release request and moves
`cur_` on. The departing head stays gated but is no longer read (it is not in
the remnant range), so its stream is silent from this sample. Later the main
loop runs `releaseToRemnant()`: a 192 KB SDRAM-to-SDRAM copy of two frames plus
the window, then `adoptLane()` sets an empty ring and opens the gate. The ISR
reads the empty ring and counts underruns until `topUp()` reaches that slot,
which is behind all eight step heads in the index scan. The departing head's
already-rendered cushion (up to a hop) is thrown away, so the remnant resumes
`fill` samples ahead of where the listener last heard it. On the host none of
this is visible because the harness drains `service()` between every sample.

The persistent one-head-per-step identity is what forces the adoption dance,
and it is also why single-step mode re-arms the sounding head in place (341 ms
of literal silence per dwell, all-zero samples in the probe) and why startup is
341 ms of silence. A head pool with allocate-at-pre-warm keeps everything #155
wanted from persistence (a head is armed and filling before its gate opens, its
identity is fixed from pre-warm to release) and makes ring-out simply "do not
free the departing head until its ring-out elapses". No copy, no empty ring, no
remnant slots distinct from step slots, one code path, and the single-step and
startup holes close because the next visit gets a fresh head from the pool.

### 7. The ISR writes a Lane the main loop is reading — [read]

`Head::remnantTick()` runs in the ISR and does `in_ = Lane()` when a remnant
expires. `Head::topUp()`/`emit()` run on the main loop and read `in_` per sample
(`in_.cur`, `in_.old`, `in_.h`, `in_.phase`). The comment claims single-writer
per phase; the Lane is not. The null checks make the visible effect mild (a few
samples into a ring nobody reads, an in-bounds read from a stale frame), but it
is a data race on a multi-word struct and the kind of thing that only ever
misbehaves on hardware. Under the pool model the ISR never mutates head state;
it sets a release flag and the main loop frees the head.

### 8. Hot-path costs worth a bench A/B — [bench]

None of these are measured. Each is a small, isolated change with a number to
read off `max_us`/`isr_us`.

- `Source::at()` does an integer modulo and a branch per sample inside
  `renderFrame`, 16384 times per frame at the largest window. Split the read at
  the wrap point once and use two straight loops.
- `laneSample()` divides twice per sample by a per-hop constant. Precompute
  `a[phase]` and `corr[phase]` per hop size (they are fixed curves), or use a
  rotating phasor. Under finding 4 this kernel moves into the ISR, so it should
  be division-free anyway.
- `renderFrame` multiplies by `synthGain_` over `w` samples in the copy-out; fold
  it into the `w/2` bin loop.
- `Sequencer::next()` recomputes `durSamples()` (float multiply and round) and
  `seamPhase()` every sample and scans six remnant atomics every sample. Decide
  once per block and run tight per-sample loops over the two or three heads
  that are actually gated.
- Audio block size is 4, so 12,000 callbacks per second. The audio latency this
  buys is invisible against a 43–171 ms hop. A block of 32–48 cuts ISR entry
  overhead and jitter and gives the main loop longer uninterrupted runs. Read
  `isr_us` before and after.
- ShyFFT vs CMSIS-DSP `arm_rfft_fast_f32`. An 18 ms real-FFT pair at 16384 is
  slow for an M7 even at 400 MHz; the 64 KB scratch buffers are four times the
  16 KB D-cache, so the bit-reversal and early passes miss. CMSIS is already in
  libDaisy's tree. Worth one afternoon.

### 9. The host suite cannot see any of this — [read]

- `testutil::render()` drains `service()` fully between samples. Producer
  latency is zero by construction, so no scheduling underrun, adoption gap, or
  index-order starvation can ever fail a test.
- The click detector explicitly skips ±1024 samples around every seam. That is
  where finding 1 lives.
- Existing assertions accept "< 5% silent", "a one-time cold-arrival gap", and
  one lookahead of silence per single-step dwell. The user's invariant is
  "silence between pieces is a bug", and no test encodes it.

Add, in this order:

1. **Seam continuity.** For every seam, 5 ms RMS immediately after the cut
   within 3 dB of 5 ms RMS immediately before it, at every frame size, at fade
   0. Fails today at every size (finding 1); passes with pre-roll.
2. **Cost-modelled producer.** A harness where each `renderFrame` at size `w`
   charges `cost(w)` microseconds from a per-block budget, with `cost(w)` read
   from the bench. Then `du` is reproducible on the host with device numbers
   plugged in, and a scheduler change can be judged before flashing.
3. **No startup or single-step hole** beyond the first pre-warm.

## Target shape

Not a rewrite of the principle; a removal of the layer between it and the audio.

```
Sequencer (ISR, per block)             Head pool (main loop)
-----------------------------          -------------------------------------
decide seam phase once per block       heads[N]: old/cur/next frame slots,
gated heads = {cur, inc, ringing...}   base position, travel, rng, stretch*,
per sample: sum over gated heads of    live size at render, deadline = h-phase
  blend(old, cur, phase) * envelope    service(): EDF over heads with next
at hop boundary: rotate old<-cur<-next   unrendered; render one frame; return
if next not ready: count underrun,     arm(step): alloc head, render pre-roll
  hold last frame (spectral, not gap)    pair (old+cur), gate stays shut
release(head): clear flag, free later  release: mark; free when ring-out done
```

Latency after the change, with no cushion to tune:

| control    | today                          | target                          |
|------------|--------------------------------|---------------------------------|
| stretch    | 2–3 hops                       | ≤ 1 hop + blend                 |
| frame size | up to one full dwell           | ≤ 1 hop (render pre-roll pair)  |
| position   | next visit of that step        | ≤ 1 hop on the sounding head    |
| duration   | live (already right)           | live                            |
| seam       | 1-hop fade-in from zero        | steady state from sample one    |

## Action plan, in order of value over risk

1. **`pod.Init(true)`** and the wrap-safe profiler. Mechanical. Re-read
   `max_us` at 4096 and 16384 so every later comparison has a correct baseline.
2. **Pre-roll in `arm()`** (finding 1) plus the seam-continuity test. One extra
   FFT per onset; removes the measured dip. Bench by ear at fade 0, 16384.
3a. **Immutable per-size windows** (finding 2). Deletes the snapshot path and a
   64 KB memcpy per arm. Output is unchanged; only where the curve is read from.
3b. **Live frame size and position on the sounding head** (finding 2). Changes
   what the knob does mid-dwell, and at a size change every sounding head
   re-renders a pre-roll pair. Bench it, especially the burst at 16384.
4. **Frames as the unit + EDF service** (findings 4, 5). This is the structural
   change; it deletes the cushion and the open issue. Stage it exactly as #155
   was staged: sequential heads first, benched to `du = 0` at 16384, then ring-out.
5. **Head pool instead of persistent slots + adoption** (findings 6, 7). Falls
   out naturally once step 4 is in, because a head is then just three frames and
   a deadline.
6. **Hot-path A/Bs** (finding 8), each measured on its own.
7. **Cost-modelled host harness** (finding 9), ideally before step 4 so the new
   scheduler is judged on the host with bench costs before it is flashed.

Steps 1 and 3a are sound-neutral. Steps 2, 3b, 4 and 5 change what comes out of
the codec and are proposals until heard on the bench.
