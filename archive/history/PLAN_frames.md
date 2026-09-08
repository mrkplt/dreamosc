# Plan: frames as the unit, latency first

Status: IMPLEMENTED (host-tested, device build links, not yet heard on the
bench — see OPEN_ISSUES.md). Kept as the design record.

Goal: minimize control-to-ear latency and remove every silence gap. Memory and
clock are cheap; latency is not. Implements the action plan in
`REVIEW_responsiveness.md` in one pass (steps 1 through 5), because the pieces
interlock: pre-roll needs a pair slot, live frame size needs immutable windows,
the head pool needs the frame model.

## The model

**Head** = a life at one step: a pool slot holding six frame buffers (old, cur,
up to two staged, up to two for a refresh), a base position, accumulated
travel, and a life number. A head owns SUPPLY; it never touches time.

**Sequencer** (ISR) owns TIME and the seam. Per sample, for each gated head it
blends `old[phase]` against `cur[h+phase]` under the raised-cosine and the
AM-correction curve (precomputed per hop size), sums under the seam envelope,
and at the hop boundary rotates frames. No ring, no cushion.

**Main loop** renders into staged slots, earliest deadline first, and
re-renders a staged frame when the controls it was rendered with have changed
and there is slack. Rendering is speculative and cheap to redo; that is what
"clock is cheap" buys: robustness of an early render AND the responsiveness of
a late one.

## Head state machine (single writer per transition)

```
FREE --ISR alloc (step, life++)--> ARMED --main: pre-roll pair staged--> READY
READY --ISR go-live: apply pair, open gate--> GATED
GATED --ISR end (ringout>0)--> RINGING --ISR remain==0 or ditch--> FREE
GATED --ISR end (ringout==0)--> FREE
```

- ISR writes: FREE→ARMED, READY→GATED, GATED→RINGING/FREE, RINGING→FREE, and
  the frame rotation (old/cur indices, phase, h).
- Main loop writes: ARMED→READY, and the staged descriptor (buffers, w, h,
  travel, the control values used, life tag). Published with a release store;
  the ISR ignores a staged descriptor whose life tag is stale.
- Buffer safety by construction: the main loop only writes buffers not
  referenced by old/cur, and the ISR only changes old/cur by consuming a staged
  descriptor (which requires the main loop to have finished) or by a hold
  (old = cur, which frees a buffer and claims none).

## Rotation at a hop boundary (ISR)

- staged SINGLE: old = cur, cur = staged.a
- staged PAIR (pre-roll pair; also a frame-size change): old = staged.a,
  cur = staged.b, h = staged.h
- nothing staged: old = cur (repeat the frame; amplitude continuous, spectrum
  repeats), count a hold. This is the underrun path; it is never silence.

## Sequencer timing

- Arm the NEXT step's head the moment the current step goes live. Pre-warm
  lead = the whole dwell. Single-step and startup get a fresh head each visit,
  so there is no re-arm hole and startup silence is one pair render.
- Go-live requires the incoming head READY. If it is not (only possible on a
  hard crank-down), the outgoing head keeps sounding until it is: the step
  runs late, never silent. Counted as `late` in the profiler.
- Seam envelope and fade semantics unchanged (equal-power quarter-sine,
  fadeLen frozen at go-live). Ring-out unchanged in meaning: the departing head
  keeps rendering at full volume for `ringout` seconds, then a raw stop.
- Cap: gated heads (cur + incoming + ringing) never exceed SS_RENDER_CAP;
  ditch the oldest ringing head to admit a new one.

## Live controls, and how each reaches the ear

| control    | mechanism                                            | latency          |
|------------|------------------------------------------------------|------------------|
| stretch    | read at render; staged frame refreshed if changed    | ≤ 1 hop + blend  |
| position   | base read at render; staged frame refreshed          | ≤ 1 hop + blend  |
| frame size | staged PAIR at the new size, applied at boundary     | ≤ 1 hop          |
| duration   | live, unquantized (unchanged)                        | immediate        |
| fade       | frozen at go-live (unchanged)                        | next seam        |
| drift      | drawn once per life at arm (unchanged)               | next visit       |

Refresh rule: a staged frame is re-rendered when (stretch, position, size) it
was rendered with differ from live AND slack (samples to the deadline) exceeds
twice the worst observed render cost for that size. Cost is measured in ISR
samples (a global sample counter), so the core stays platform-free.

## Determinism

Per-frame phase seed = hash(life seed, frame index), not a running RNG, so a
refresh reproduces the same phases for the same controls, and output does not
depend on scheduling. Life seed derives from the drift-adjusted position at
arm (zero drift is still a literal repeat).

## Tables (immutable after init)

- Seven windows and gains (16384 down to 256), ~130 KB, plus blend/correction
  curves per hop, ~130 KB. Both in SDRAM (sequential reads, cached). Nothing
  is ever rewritten after `init()`, so the fast-scroll race cannot exist.
- gWork/gSpec stay in AXI SRAM.

## Firmware

- `pod.Init(true)`: 480 MHz (docs said 480; it was 400).
- Audio block 32 (was 4): the hop dominates latency; fewer ISR entries.
- Profiler: wrap-safe tick deltas; HLTH gains gated/ringing/armed, holds,
  late go-lives, refreshes, min slack, per-size render cost.

## Tests

Keep every invariant that still applies (finite, bounded, deterministic,
interior click detector, constant loudness, multi-rate, all sizes, drift,
ring-out cap and expiry, step count). Add:

1. Seam continuity: 5 ms RMS after each cut within 3 dB of 5 ms RMS before it,
   fade 0, every size.
2. No startup or single-step hole beyond one pair render.
3. Rate-limited producer: holds counted, output never silent once started.
4. Live frame size reaches the sounding head within about one hop.
5. Live position reaches the sounding head within about one hop.
6. A refresh with unchanged controls is bit-identical (determinism of the
   per-frame seed).

## Docs

CLAUDE.md mental model + memory section, CONTROLS.md, hardware_spec.md (clock),
OPEN_ISSUES.md (cushion issue closed by construction; list what the bench owes).
