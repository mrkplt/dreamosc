# dreamosc — Control surface

The current Pod control mapping. This is the DEVELOPMENT / access surface (reach
every parameter to unblock DSP work), **not** the intended performance interface
— the design wants a knob per parameter (see Fizzy #134/#146/#145, and #148 which
argues for encoders over pots on the real hardware build). Add dedicated controls
as the hardware allows; don't treat two-knobs-plus-encoder as the design.

Panel: 2 knobs, encoder (turn + click), 2 buttons, 2 RGB LEDs, and a permanently
attached SSD1309 OLED (I²C, not yet wired — Fizzy #141).

## Knob modes (button-driven)

The two knobs edit either the GLOBAL parameters or ONE selected step. **Button 1**
cycles the mode; **button 2** jumps back to GLOBAL:

```
GLOBAL ──button1──▶ step 1 ──button1──▶ … ──button1──▶ step 8 ──button1──▶ GLOBAL
   ▲                                                                          │
   └──────────────────────── button 2 (from anywhere) ───────────────────────┘
```

| Mode | knob1 | knob2 | led1 |
|------|-------|-------|------|
| **GLOBAL** | duration (0.25..60 s) | global drift (0..0.03) | off |
| **step i** | that step's position (0..1) | that step's per-step drift (0..0.03) | ROYGBIVW (step 1..8) |

- **Global drift is the FLOOR** for each step's per-step drift; effective
  per-step drift = `max(perStep, global)`, clamped to [0,1]. Global lifts every
  step to at least that much shimmer; a step can go HIGHER with its own knob but
  never lower. (The two are kept in separate storage so a global change never
  corrupts the per-step shadow — a double-add bug that shipped once, back when
  the fold was additive, is now guarded by a test.)
- **Drift range is 0..3%** of the source (max 0.03), with a fine grid of 0.03%
  (0.0003) on a slow turn — 100 fine detents span the range.
- **led1 = ROYGBIVW** shows the selected step (red=1 … white=8), off in GLOBAL.

## PICKUP (soft takeover) — everywhere

Landing on any slot (GLOBAL or a step) does **not** snap its value to the pot. A
knob takes over its parameter for the current slot only after it has physically
**moved** since arriving; until then the slot holds its stored value. This lets
you tour the steps and change only the ones you touch.

Two implementation facts that were hard-won bugs:
- **Detect movement on the RAW knob.** A one-pole smoother caps the per-pass
  delta below any threshold, so detecting on the smoothed read means pickup
  never engages. Duration and drift WRITE the smoothed value; position writes
  the raw (snapped) value, see below.
- **Freeze the move reference at an ANCHOR** (the pot position at slot entry) and
  don't update it until the knob engages. If the reference updates every poll it
  *chases* the pot, so a SLOW turn never accumulates a threshold-crossing delta
  and the control feels dead. Anchor + accumulate is the fix.

## Encoder

Turn drives the current page; click cycles the page. led2 = page hue (**RoYG**
over the four pages, in click order) drawn from the **same ROYGBIVW palette as
led1** (`hueROYGBIVW`, so a color means the same on both LEDs and orange/yellow
stay distinct). **Brightness on EVERY page tracks that page's encoded level** —
a bright LED always means "this parameter is turned up".

Click order: **stretch → steps → fade → window → stretch** (the page
index is also the color index, so click order = RoYG).

| Page | led2 hue | encoder turn | brightness = |
|------|----------|--------------|--------------|
| **stretch** | red | index `STRETCH_STOPS` (1×..10000×) | stretch index (low dim → max bright) |
| **steps** | orange | active step count 1..8 → `seq.setSteps()` (one/detent) | step count (few dim → 8 bright) |
| **fade** | yellow | crossfade overlap 0..0.5 (additive) | fade amount (0 dim → 0.5 bright) |
| **window** (frame) | green | the core's size index (`ssSizeW`: 16384, 8192, 4096, 2048, 1024, 512, 256) → `seq.setFrame()` (largest first; **CW shrinks**; **default 4096** = the core's `SS_W_DEFAULT`, derived) | knob position (CCW dim → CW bright) |

- All four use `levelBrightness` (a `[floor, 1.0]` map with a dim floor so the
  bottom of a range is still lit, never off): `stretchBrightness`,
  `stepBrightness`, `fadeBrightness`, `frameBrightness`.
  The level reads at a glance without the OLED.

- **Stretch is a detent table**, not continuous: PaulStretch factors aren't
  perceptually linear, so what matters is the regime (scan/drift/freeze). Fine
  1..10, coarsening up to 10000× (10000× is transient-squelch territory — freezes
  a cymbal hit into wash). Slow click = 1 stop, fast spin = 3 stops/detent.
- **Fade** = crossfade seam overlap. 0 = butt-joint (hard cut). There is no
  separate on/off toggle — fade 0 IS the butt-joint case (a toggle was byte-
  identical to it and was removed, freeing button 2 for the mode navigation).
- **Frame size** is a SOUND-CHARACTER control (Fizzy #136): the window is the
  chunk of source FFT'd to extract the spectrum, so it sets frequency resolution.
  Small (256) = grainy/articulated, and on tonal material an audible per-hop
  WOBBLE (coarse bins beat fast). Large (up to 16384 ≈ 0.34 s) = glassy/frozen
  and turns that wobble into PaulXStretch's slow characteristic SHIMMER (fine
  bins). Default 4096; grow it (CCW) for shimmer, shrink it (CW) for grain. It is
  a LIVE control: every sounding head re-renders a pre-roll pair at the new size
  and switches at a hop boundary. Shrinking lands at the next boundary (one
  hop). Growing to a size whose pair costs more than what is left of the
  current hop lands one boundary later (two hops), and under a full crossfade a
  growth to 16384 renders pairs for both seam heads at once, which can hold
  (repeat) frames for a few hops — see the F5 finding test. The profiler's `hop`
  shows what the sounding head is actually playing.
- **Position and stretch are live too.** A staged frame is re-rendered when the
  controls it was made with change (if there is slack before its deadline), so a
  turn reaches the ear within about one hop plus the blend. Position moves the
  SOUNDING head, not just the next visit. The NEXT step's pre-warmed head
  re-renders too, as often as you turn (the ISR drops superseded frames once
  per block, finding F4), so it always goes live at the latest position.
- **Pot smoothing and refreshes** (finding F8): position is written from the
  RAW pot (snapped by the fast-move grid), not the one-pole smoothed read, so
  a turn is a few discrete updates rather than a ~250 ms creep that would cost
  a re-render per hop. The core also spaces refreshes per head at 4× the
  measured render cost (≥ 10 ms; there is no one-hop floor any more, so a
  second move within the same hop lands at that hop's boundary — L2) and
  ignores position deltas under 0.2%. The NEXT step's pre-warmed head only
  re-renders its pair once go-live is within 8× cost (L3), so turning its
  knob all dwell long costs one pair, not one per gap, and it still goes live
  at the latest position. Duration and drift keep the smoother. Refreshes
  land at hop boundaries; they do not click.
- **Duration is now LIVE and UNQUANTIZED** (#155). The step dwell is exactly
  `round(duration·sr)` samples, independent of frame size — so frame size no
  longer bends step timing (the old model quantized the dwell to the analysis-hop
  grid, which snapped short steps at large windows). Turn duration down mid-dwell
  and the sequence fast-marches immediately, even out of a minute-long dwell.
- **Step count** (Fizzy #149): how many of the 8 steps the sequence walks, 1..8.
  Fewer steps = a shorter, faster-repeating pattern (a real compositional
  control). The `position[]`/`drift[]` arrays stay sized to 8; only steps below
  the count are walked. The step-select nav (button1) only visits ACTIVE steps,
  and dropping the count off a currently-selected step snaps the panel back to
  GLOBAL. On the real hardware build this likely becomes per-step select buttons
  (Fizzy #151), a mask that generalizes this contiguous count.

### Encoder speed model

`Encoder::Increment()` only ever returns ±1, so turn SPEED is inferred from the
time GAP between detents (`encoderFast`, ≤40 ms = fast), NOT from step magnitude.
Fast = coarse step, slow = fine.

### Control polling

**Encoder and buttons are read from a 2 kHz timer IRQ** (TIM5, below audio
priority), not the main loop. The main loop polls between `service()` calls,
i.e. behind whatever render is in flight (1.2 ms at 4096, 14–29 ms at
16384), and libDaisy's encoder debounce needs two *consecutive* 1 ms samples,
so a fast spin at 16384 dropped or mis-signed detents (L1). The IRQ debounces
and pushes timestamped events into a lock-free ring (`PanelQueue` /
`drainPanelEvents` in `controls_core.h`, host-tested); the main loop drains
the ring on its **1 ms wall-clock tick** and applies each event with its
*original* timing, so the fast/slow speed model is unaffected by how long a
render blocked the loop, and a click before a detent still changes the page
first. The knobs (ADC read, smoothing, pickup) and LEDs stay on the main-loop
tick. The IRQ owns the `Encoder`/`Switch` objects exclusively. Profiler:
`det=` (detents applied per second — count them against the physical clicks)
and `drop=` (ring overflows, must stay 0). Audio block is 32 samples.

## Where the logic lives

All the pure decision logic (mode/pickup/drift-fold, encoder stepping, the
stretch detent table, the per-page encoder dispatch, LED colors and levels) is
in the platform-free, host-tested **`controls_core.h`** (`PanelEditor`,
`foldDrift`, `stepAdditive`/`stepIndex`/`stepCount`, `STRETCH_STOPS`,
`EncoderState`/`applyEncoder`/`pageBrightness`, `pageColor`/`stepColor`).
`dreamosc.cpp` is only the hardware glue that reads the panel and calls in. See
the testing-culture note in `../CLAUDE.md`.

## Diagnostics

`make PROFILE=1` prints the full control + instrument state over USB serial each
second (`SET`/`KNOB`/`POS`/`DRF`/`HLTH`/`COST` lines). Every variable parameter must
appear there — see the profiler directive in `../CLAUDE.md`.
