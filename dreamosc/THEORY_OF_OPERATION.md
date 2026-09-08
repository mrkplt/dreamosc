# dreamosc — Theory of Operation

How the instrument works, from the synthesis math down to the memory ordering
between the audio interrupt and the main loop. This is the reference for
anyone reasoning about a change to `stretch_core.h` or `dreamosc.cpp`. The
spec (`archive/stretchsequencerspec.md`) says what the instrument is; this says
how the code makes it so.

Sections:

1. The synthesis: one PaulXStretch frame, and why frames are the unit
2. A head: two frames and a phase
3. The head pool and its state machine
4. The staged-frame queue: how the main loop hands frames to the ISR
5. The scheduler: earliest deadline first, speculative refresh, cost model
6. The sequencer clock: dwell and seams
7. Live controls and their latencies
8. Determinism
9. Memory map and placement
10. Timing and CPU model
11. Diagnostics (`make PROFILE=1`)
12. Testing: what the host can and cannot see
13. Invariants (the list to check against any change)

---

## 1. The synthesis: one PaulXStretch frame, and why frames are the unit

PaulXStretch extracts a spectrum from a window of source, randomizes every
bin's phase, inverse-transforms it, and plays the resulting periodic
waveform. Stretch comes from advancing the source read position by only
`hop / stretch` samples per output hop of `hop` samples. The output is
noise-like in phase but carries the source's spectral envelope, so it sounds
like the material frozen in place.

**Rendering one frame** (`Head::renderFrame`), for window size `w` and hop
`h = w/2`:

1. Read `w` source samples starting at the frame's source position (wrapping
   at the source's end, with at most one split, no per-sample modulo) and
   multiply by the analysis window `(1 − x²)^1.25` (Nasca's; a rectangular
   window leaked 0.6% out-of-band energy and was audibly scratchy).
2. Forward real FFT (`ShyFFT`, runtime length `log2(w)` passes) in the AXI
   SRAM scratch `gWork`/`gSpec`.
3. For every bin `1..w/2−1`: keep the magnitude, draw a fresh phase from a
   xorshift stream seeded for this frame, multiply by the per-size synthesis
   gain (undoes the FFT's `×w` and the window's mean). DC and Nyquist are
   zeroed.
4. Inverse FFT, copy the `w`-sample periodic waveform to the head's frame
   buffer in SDRAM.

Cost is `O(w log w)`; measured at 480 MHz: ~14.5 ms per 16384-point frame
(~28.7 ms for a pre-roll pair), ~1.2 ms at 4096 (see OPEN_ISSUES.md).

**Playing frames.** Each output hop is a raised-cosine blend of the previous
frame's first half against the current frame's second half, multiplied by the
AM-correction curve:

```
a(p)    = 0.5 + 0.5·cos(π·p/h)                          1 → 0 across the hop
corr(p) = 0.853553 − (1 − 0.853553)·cos(2π·p/h)          0.707 at the joint, 1.0 mid-hop
out(p)  = ( old[p]·a(p) + cur[h + p]·(1 − a(p)) ) · corr(p)
```

At `p = 0` the output is 100% `old` at its circular wrap point; at `p = h/2`
it is an equal mix of two uncorrelated signals, whose power sums to 0.5, so
`corr` attenuates the joint to 1/√2 and leaves the mid-hop at 1.0, flattening
the envelope. Because the IFFT waveform is periodic, `cur[2h−1] → cur[0]` is
continuous, so at the next boundary (`old ← cur`) there is no step. The two
curves are precomputed per hop size (`gBlendA`, `gBlendC`) so the per-sample
cost is two table reads, two frame reads, four multiplies.

**Why frames are the unit.** Everything the ISR needs to produce a sample is
two frames and a phase. There is no reason to re-buffer the blend sample by
sample: the frames already are the buffer. The producer's only contract is
"the next frame is staged before `phase` reaches `h`", a deadline known
exactly. This removed the sample ring, the cushion, and the latency the
cushion added, and made the deadline schedulable.

**Pre-roll.** A head cannot start on frame 0 alone: the first hop would blend
against nothing and fade in from silence over a whole hop (measured −34 dB at
the cut at 4096, −62 dB at 16384 before this was fixed). So a head goes live
with two frames rendered: the pre-roll frame at source position `−h/stretch`
and frame 0 at the position itself. Its first sample is mid-stream at full
level, and a raw cut between heads is amplitude-continuous. The dependency is
exactly one frame deep, so one pre-roll frame is sufficient and there is no
transient beyond it.

---

## 2. A head: two frames and a phase

`Head` is one life at one step: a pool slot with six frame buffers of `SS_W`
floats in SDRAM. The ISR side holds:

| field | meaning |
|---|---|
| `old_`, `cur_` (+ `oldIdx_`, `curIdx_`) | the two frames being blended |
| `phase_`, `h_` | position inside the hop and the hop length of the frames in play |
| `A_`, `C_` | the blend/correction tables for `h_` |
| `state_` | FREE / ARMED / READY / GATED |
| `life_` | bumped at every allocation; tags everything the main loop publishes |
| `step_`, `lifeSeed_`, `driftOff_` | which step, the phase seed, the drift offset drawn at arm |

(The armed head's deadline is not head state: there is exactly one armed head
at a time, so the Sequencer holds it as `nextOnset_`, section 6.)

`Head::tick()` produces one sample and advances the phase. At the boundary it
applies the next staged frame (section 4); if nothing is staged it **holds**:
`old_ = cur_`, which replays the current frame (periodic, so continuous) and
counts a hold. A hold is a spectral freeze of one hop. It is never silence,
and it is the only degradation mode under overload.

The main-loop side holds the bookkeeping for rendering: which frame index is
next, the travel (accumulated source advance) of the frame in `cur`, the size
it was rendered at, and the control values the last staged frame used.

---

## 3. The head pool and its state machine

Heads are not bound to steps. A step visit allocates a head from the pool of
`SS_HEADS = 10`, uses it, and frees it. This is what removed the adoption copy,
the empty-ring gap, and the single-step and startup holes of the previous
design.

```
FREE ──ISR alloc(step, seed, drift, life++)──▶ ARMED
ARMED ──main loop stages the pre-roll pair──▶ READY          (a hint; see below)
READY ──ISR goLive(): apply the pair, phase 0──▶ GATED
GATED ──ISR end of dwell──▶ FREE
```

Single-writer rules:

- The ISR writes every transition except ARMED→READY, and owns `old_`,
  `cur_`, `phase_`, `h_`, `applied_`, `qTail_` (and the Sequencer's
  `nextOnset_`).
- The main loop writes ARMED→READY (by compare-and-swap from ARMED, so a head
  the ISR freed and re-allocated meanwhile can never be marked READY by a stale
  render), the descriptors, and `qHead_`.
- READY is only a hint. `goLive()` validates by looking for a staged pair
  tagged with the current life; if none is there it returns false and the
  outgoing head keeps sounding (section 6). Nothing trusts the hint.

Allocation (`Sequencer::allocHead`, ISR) takes the first FREE slot. Worst-case
live occupancy is three heads — current + incoming (during a seam) + one armed
for the next step — and the pool is sized well above that (a `static_assert`
pins `SS_HEADS ≥ 4`), so a FREE slot always exists and allocation never has to
steal.

Freeing (`Head::free`) clears the state and the frame pointers. It does not
touch the queue or the life; the next `alloc()` bumps `life_` and drains the
queue, and the main loop discovers the new life on its next pass and resets
its bookkeeping (`syncApplied`).

---

## 4. The staged-frame queue

The main loop hands frames to the ISR through a per-head ring of `SS_DESCQ`
descriptors. A descriptor says what to apply at the next boundary:

```
Staged { life, frameIdx, kind (SINGLE | PAIR), a, b, sizeIdx, travel }
SINGLE: old ← cur, cur ← buf[a]              (the ordinary next frame)
PAIR:   old ← buf[a], cur ← buf[b], h ← w/2   (pre-roll at go-live, or a size change)
```

**Ownership.** `qHead_` is written by the main loop (push), `qTail_` by the
ISR (pop). Descriptor slots in `[tail, head)` are ISR-readable and never
rewritten by the main loop; the main loop refuses to push when
`head − tail ≥ SS_DESCQ − 1`.

**Validity.** The ISR applies a descriptor only if `life == life_` and
`frameIdx == applied_ + 1`. At a boundary it pops everything queued and
applies the *last* valid one, so a refresh (a re-render of the same frame with
newer controls) supersedes the original. Stale entries (old life, already
applied) are discarded. `applied_` and `appliedSlot_` are then published
(release) so the main loop can read the applied descriptor's `travel` and size
to continue from.

**Per-block drain.** Once per audio block the ISR also drops every queued
descriptor that a later valid one supersedes (`isrDrainSuperseded`). This is
what lets a waiting head take any number of refreshes: the queue never holds
more than the one descriptor that will actually be applied plus the one being
added.

**Buffer safety by construction.** The main loop renders only into buffers
not referenced by (a) any queued descriptor, (b) the ISR's `cur`, (c) the
ISR's `old`, read in that order. The ISR claims buffers only by popping a
descriptor (SINGLE: queued→cur, cur→old; PAIR: queued→old, queued→cur) or by a
hold (old ← cur, claims nothing). Reading the tail before old/cur means a pop
between the reads leaves the popped buffer excluded either way; reading `cur`
before `old` means a SINGLE pop between those two loads leaves the buffer that
just became `old` already captured as `cur`. Six buffers cover the worst case:
old, cur, a staged pair, and a refresh pair.

**Life tags.** A render can be in progress when the ISR frees and re-allocates
the head. The render captured the life at its start and re-checks it before
publishing; a mismatch discards the work. A publish that slipped through
carries the old life tag and is ignored. `alloc()` drains the queue so at most
one such stale entry can exist, which the buffer budget tolerates.

**Memory ordering.** Every cross-side handoff is a release store paired with
an acquire load: descriptor contents before `qHead_`; frame rotation before
`applied_`; `qTail_` after the rotation; state after the fields it guards.
ISR-side reads of state use relaxed loads because the ISR is the writer of
every transition it acts on. On the single-core M7 the ISR preempts the main
loop and never the reverse, so the only torn-read hazard is a main-loop read
of a multi-word value the ISR writes; none is read that way (travel comes
from a descriptor the main loop wrote; `nextOnset_` and the indices are
single words).

---

## 5. The scheduler

`Sequencer::service()` is the main loop's whole DSP job: pick the most urgent
render, do it, return. Each call:

1. For every non-free head (`pickWork()`), `plan()` reports what it wants and
   its deadline:
   - **REQUIRED**: no frame staged for `applied + 1`. Deadline: for a gated
     head, `clock + (h − phase)`; for an armed head, the sequencer's
     `nextOnset_`.
   - **REFRESH**: a frame is staged but the controls it used differ from live
     (size index or stretch differ, or position moved by more than 0.002), the
     queue has room, and at least one hop (or `4 × cost`, or 10 ms) has passed
     since this head's last refresh.
   - **NONE**: nothing to do.
2. Pick the REQUIRED render with the earliest deadline (wrap-safe signed
   compare). Exception: an ARMED head's pre-roll pair whose due is more than
   `8 × cost` away yields to a slack-valid REFRESH, since it is due a whole
   dwell later and would otherwise cost a knob turn a hop for no gain. A gated
   head's REQUIRED frame always wins.
3. A REFRESH runs only with slack: `deadline − clock > 2 × cost × frames`. A
   refresh that would miss its boundary is worse than none.
4. `render()` re-syncs with the ISR (so a boundary between plan and render
   cannot stage an already-played frame), chooses SINGLE or PAIR
   (`chooseRender()`, a pure function), picks free buffers, renders,
   publishes.

**Size change.** A gated head whose live window differs from the size of its
`cur` needs a PAIR at the new size. If that pair would miss this boundary
(`slack ≤ 2 × cost`) but could fit a full hop, a SINGLE at the old size is
staged instead and the pair renders right after the boundary with a whole hop
of slack. If a pair cannot fit a full hop either, it renders now. Growing to
16384 while cur + inc both need pairs at a seam is throughput-bound (the pairs
are several old hops of work at the measured cost) and produces bounded holds.

**Cost model.** `CostModel` holds the per-frame render cost per size in ISR
samples, seeded from `0.003766 · w · log2 w` (~18 ms at 16384 / 48 kHz, the
old 400 MHz figure; the bench now measures ~14.5 ms at 480 MHz), then
tracked as a recent max with slow decay (`c −= c/64` per render). It is
measured from the ISR's sample counter across the render; a zero delta (the
host harness) leaves it alone. `setCostEstimate()` lets the firmware seed it
from bench numbers and lets the costed host harness pin it.

---

## 6. The sequencer clock

The ISR owns time. Per block, `Sequencer::render()` snapshots the live
controls into a `Block` (dwell length `round(duration · sr)`, seam geometry,
the active step count), does the per-block housekeeping (queue drains, the
armed head's deadline, a re-arm if the step count shrank under the armed
head), then ticks samples against that snapshot. The main loop is the only
writer of the controls and cannot preempt the ISR, so they are constant across
a block anyway; the snapshot makes that a property of the code, and is why
`tick()` has no per-sample step-count check.

**Dwell and seam.** `seamGeom(dur, fade)` gives `fadeLen = dur · fade` (fade
clamped to 0..0.5) and `onset = dur − fadeLen`. A dwell runs `elapsed_` from 0.
At `elapsed_ ≥ onset` the incoming head goes live if it is READY;
`seamStart_` and `fadeLen_` are frozen. During the seam the
incoming head fades in and the outgoing fades out under an equal-power
quarter-sine (`sin²+cos² = 1`, constant power for uncorrelated sources). At
`fp = elapsed_ − seamStart_ ≥ fadeLen_` the outgoing head is freed, the
incoming becomes current, and `elapsed_` restarts at 1 (that sample is the
new dwell's first). Fade 0 collapses this to a raw cut at `elapsed_ == dur`:
the outgoing head stops, the incoming starts at full level (pre-rolled), the
spectrum changes, the amplitude does not.

**Arming.** The moment a head goes live, the *next* step's head is allocated
and armed (`armNext`), so the pre-warm lead is the whole dwell. Its deadline
is the Sequencer's `nextOnset_ = clock + samplesToNextOnset()`: the rest of
this dwell up to onset, or, inside a seam, the rest of the fade plus the
incoming head's dwell up to its onset. It is re-stamped every block so a live
duration or fade change keeps it honest.

**Late, never silent.** If the incoming head is not READY at onset (only
possible when duration is cranked below the time a pre-roll pair takes to
render, or under overload), the outgoing head keeps sounding and `late_`
counts the samples. Nothing goes silent; the step runs long.

**Concurrency.** Heads are strictly sequential: at most two sound at once, and
only during a seam (current + incoming). A third head — the next step's — is
armed and rendering its pre-roll a whole dwell ahead, so the heaviest
concurrent render load is three heads (cur + inc + armed), never more.

**Startup and single-step.** At startup nothing sounds until the first head's
pair is rendered (a few ms), then it goes live; there is no lookahead wait.
With one active step, each visit still gets a fresh head, so there is no
re-arm hole.

---

## 7. Live controls and their latencies

| control | read where | reaches the ear |
|---|---|---|
| stretch | every render (advance per hop = `h / stretch`) | ≤ 1 hop + blend: a staged frame is refreshed with slack, else the next frame |
| position (of the step a head plays) | every render (`base = position + driftOff`) | ≤ 1 hop + blend, moves the *sounding* head; also the pre-warmed next head |
| frame size | every render | ≤ 1 hop (pair at the new size at the next boundary); 2 hops when the pair cannot fit what is left of the current hop |
| duration | every block | immediate (the dwell clock is live and unquantized) |
| fade | every block; frozen at go-live | next seam |
| drift | drawn once per life at arm | next visit of that step |
| step count | every block | next go-live (an armed head for a now-invalid step is re-armed at once) |

The one-hop floor is PaulXStretch's own: a frame is what it is until the next
one. Latency above that floor has been removed: there is no cushion, and the
hop-level decision (render next) is taken as early as possible with a
re-render when the controls move.

Refresh discipline: at most one refresh per hop per head, position deltas
under 0.2% ignored, and never a refresh that would miss its boundary. The
panel writes position from the raw pot (snapped by the fast-move grid), not
the one-pole smoothed read, so a turn is a few discrete updates rather than a
250 ms creep that would cost a render per hop.

---

## 8. Determinism

- Per-frame phase seed = `ssHash2(lifeSeed, frameIdx)`. A re-render of the
  same frame with the same controls is bit-identical regardless of when the
  scheduler ran it.
- `lifeSeed = seed ^ hash(position at arm, drift-adjusted)`, so the same
  position always yields the same phases: zero drift is a literal repeat.
- Drift offsets are drawn by the ISR from a sequencer-owned xorshift in arm
  order, so a fixed configuration renders identically on every run.
- Go-live can depend on readiness (a late go-live shifts the timeline), but
  only when the producer is slower than a pre-roll pair per dwell.

---

## 9. Memory map and placement

| what | where | size at SS_W 16384 |
|---|---|---|
| head pool: 10 heads × 6 frames × 16384 floats | SDRAM (`DSY_SDRAM_BSS`), plain array | ~3.9 MB |
| source buffer (10 s at 48 kHz) | SDRAM | ~1.9 MB |
| `gWindows` (7 window curves), `gBlendA`, `gBlendC` | SDRAM, written once by `init()` | ~255 KB |
| `gWork`, `gSpec` FFT scratch | AXI SRAM (`.axisram_bss`, our linker script) | 128 KB |
| `Sequencer` object (heads' atomics, queues), `gTab` (FFT twiddles, sin LUT) | DTCM (`.bss`) | ~60 KB |

Rules: SDRAM is unpowered at static-init time and its section is NOLOAD, so
only plain arrays live there, filled after `pod.Init()`; never a constructed
object (a `Sequencer` placed there once booted with garbage and no sound). A
frame buffer is always fully written by a render before any descriptor can
reference it, so NOLOAD garbage never reaches the output. SDRAM is configured
cacheable and bufferable by libDaisy's MPU region 1, and every access to it
here is sequential (frame reads, table reads, source reads), so it is served
from the 16 KB L1 in practice. The FFT scratch is random-stride, so it lives
in AXI SRAM.

---

## 10. Timing and CPU model

- Clock: 480 MHz (`pod.Init(true)`; libDaisy's default is 400).
- Audio: 48 kHz, block 32 (0.67 ms). The block is invisible against the hop
  (43 ms at 4096, 171 ms at 16384); it exists to cut interrupt overhead.
- ISR per sample: for each gated head, two frame reads, two table reads, the
  blend; plus the seam envelope (two LUT reads) during a seam.
- Main loop per hop per gated head: one frame render. Steady-state load is
  `gated × cost(w) / h`. At most three heads render concurrently (cur + inc +
  armed); at the measured cost, three at 16384 is ~25%, three at 4096 is ~8%.
  A size change adds one pair per head, once.
- Overload degrades to holds (frame repeats) and late go-lives; both are
  counted, neither is silence.

---

## 11. Diagnostics (`make PROFILE=1`)

Per-second lines over USB serial. Timing uses raw `System::GetTick()` deltas
because `GetUs()` wraps every 21.5 s.

`SET`: stretch, duration, global drift, fade, requested frame,
`hop` (what the sounding head is actually playing), current step, step count,
block, encoder page, panel slot.

`HLTH`: `act` gated step heads (at most 2), `arm` armed, `free` pool slots
(a leak shows as a steady decline), `units` renders, `svc_us`/`avg_us`/
`max_us` main-loop DSP time, `isr_us`, `du` holds, `late` samples a seam
waited, `rfr` control-driven re-renders, `clip` output clamp hits, `slack`
minimum samples-to-deadline at render start (negative = a render started past
its boundary). A separate `COST` line carries the per-size estimate in samples
(16384 down to 256) and `stk` stack high-water.

The directive stands: every runtime value goes on one of these lines.

---

## 12. Testing: what the host can and cannot see

- `render()` drains `service()` between every sample: an infinitely fast
  producer. It sees baked content (pre-roll level, seam continuity,
  determinism, live-control latency) and never scheduling.
- `CostedProducer` / `render_costed()` charge each rendered frame its
  modelled cost and pin the sequencer's estimate to the same model. It sees
  holds, late go-lives, and starvation, reproducibly, from a per-size cost
  you can take off the bench.
- `SS_HOOK` points (test build only) call the ISR side from inside a
  main-loop routine at a named instruction, which is the only way a
  single-threaded host can exercise a preemption race. `test_findings.cpp`
  uses them; each finding test measures the rendered audio (click detector,
  silent windows, first-difference sample), so "is it real" and "does it
  matter" are numbers.
- Host tests cannot hear. Every sound-affecting change is a hypothesis until
  the bench confirms it; `OPEN_ISSUES.md` lists what the bench owes.

---

## 13. Invariants

Check any change against these.

1. Output is never silent while a head is gated: a missing frame is a hold,
   a not-ready incoming head extends the outgoing one.
2. A head goes live only with both frames rendered (pre-roll); the first
   sample is at steady-state level.
3. Within a head, the output is continuous: frames are periodic, rotation and
   hold happen only at `phase == 0`, a size change swaps a whole pair.
4. The main loop never writes a buffer the ISR references or a descriptor
   references; the ISR never reads a buffer the main loop is writing.
5. Every atomic has one writer per transition; every cross-side handoff is
   release/acquire; the ISR applies only descriptors tagged with the current
   life and the next frame index.
6. At most two heads sound at once (cur + inc at a seam); worst-case live
   occupancy is three (cur + inc + armed); pool ≥ 4; six buffers per head.
7. A fixed configuration renders identically; a refresh with unchanged
   controls is bit-identical.
8. No control latches at arm except drift. Stretch, position and size are
   read at every render.
9. Every runtime value is on a PROFILE line.
