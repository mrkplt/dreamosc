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
| **step i** | unassigned (each head's position is its row-2 pot) | that step's per-step drift (0..0.03) | ROYGBIVW (step 1..8) |

- **Global drift is the FLOOR** for each step's per-step drift; effective
  per-step drift = `max(perStep, global)`, clamped to [0,1]. Global lifts every
  step to at least that much shimmer; a step can go HIGHER with its own knob but
  never lower. (The two are kept in separate storage so a global change never
  corrupts the per-step shadow — a double-add bug that shipped once, back when
  the fold was additive, is now guarded by a test.)
- **Drift range is 0..3%** of the source (max 0.03), with a fine grid of 0.03%
  (0.0003) on a slow turn — 100 fine detents span the range.
- **led1 = ROYGBIVW** shows the selected step (red=1 … white=8), off in GLOBAL.

### Row-2 pots: one position knob per head (Fizzy #157)

Eight pots behind a CD4051 8:1 mux, common output on **A7** (D22), selects
A/B/C on **D7/D8/D9** (the Seed pins labelled SS/SCK/MI). The mux is scanned
by **ADC2** (`mux_adc.h`), leaving the Pod's own ADC1 and its two knobs
untouched. ADC2 paces the scan itself: its end-of-conversion interrupt
latches the read, steps the mux and starts the next conversion (16×
oversampled, 32.5-cycle sampling), so all eight pots are read in ~0.85 ms
and every 1 ms control
poll sees a fresh value for each.

- **Knob N (left to right) sets head N's position**, 0..1 of the file, and
  nothing else: eight independent `Knob`s, each reading its own `MuxPin` and
  bound to `seq.position[N]` with `positionSpec()` (a fast turn snaps to 5%
  landmarks while it moves, a slow turn is continuous), pickup from boot.
- **The pots are not wired in panel order.** `KNOB_MUX_CH` in
  `controls_core.h` maps panel knob → mux channel (from the PCB netlist:
  X2 X1 X0 X3 X5 X7 X6 X4).
- **knob1 no longer edits position** (it is unassigned in step mode), so
  nothing else writes a head's position and there is no ownership rule.
- PROFILE `MUX` line: the eight pots in panel order (raw ×1000), then
  `L=` (pickup engaged), knob 1 first. The
  positions themselves are the `POS` line. `scn=` = full scans this second
  (~1150..1190; must be > 1000), `o1=` = ADC1 (Pod knob) overruns, ever
  (must stay 0).
- **Boot:** a `MuxPin` is not `ready()` until ADC2 has converted every input
  once, and a `Knob` does nothing while its input is not ready, so the pots'
  pickup starts from where they really sit.

## Every pot is a `Knob`

Every physical pot -- the Pod's two knobs and the eight row-2 pots -- is a
`Knob` (`controls_core.h`); a new pot uses `Knob` directly or derives from it,
so they all feel the same. A `Knob` is injected with the **`AnalogInput`** it
reads (`analog_input.h`) and never knows what is behind it: the Pod's knobs
are `ControlPin`s (`control_pin.h`, over libDaisy's `AnalogControl`
`pod.knob1`/`pod.knob2`, read raw -- not its filtered `Value()`), the row-2 pots are `MuxPin`s (`mux_adc.h`, one 4051 channel
each), tests use a `FakeInput`. A `Knob` is **bound** to the float it writes
and the `KnobSpec` that maps onto it; `PanelEditor` re-binds the Pod's knobs
per slot. Polled once per 1 ms, a `Knob` owns:
- **pickup** (below), detected on the raw read;
- **speed**, `|Δraw|` per poll, for the fast/slow grid;
- **flutter**: `Knob::DEADBAND` (0.002 of travel, twice the ~0.001 measured
  on a resting pot) holds the value still until the pot moves past it, so a
  resting knob writes a constant. A slow turn moves in steps just over 0.2%;
- **de-zippering**: a one-pole (`Knob::SMOOTH`) over the deadbanded read, for
  parameters whose `KnobSpec` asks for it (`smooth = true`: duration, drift).
  Position does not (`positionSpec()`, see "Pot smoothing and refreshes").

## PICKUP (soft takeover) — everywhere

Landing on any slot (GLOBAL or a step) does **not** snap its value to the pot. A
knob takes over its parameter for the current slot only after it has physically
**moved** since arriving; until then the slot holds its stored value. This lets
you tour the steps and change only the ones you touch.

Two implementation facts that were hard-won bugs:
- **Detect movement on the RAW knob.** A one-pole smoother caps the per-pass
  delta below any threshold, so detecting on the smoothed read means pickup
  never engages. Duration and drift WRITE the smoothed value; position writes
  the deadbanded raw (snapped) value, see below.
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
  deadbanded RAW pot (snapped by the fast-move grid), not the one-pole
  smoothed read, so
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

**The whole panel — encoder, buttons, knobs and LEDs — is read in the AUDIO
CALLBACK**, at the top of every callback before the block is rendered
(`processControls()` in `dreamosc.cpp`; 1.5 kHz at block 32). That is where
Electrosmith's own Pod examples read it, and it is the only placement that
works here: the main loop is busy with non-preemptible 1.2–29 ms FFT renders,
and libDaisy's encoder debounce needs two *consecutive* 1 ms samples, so a
poll that can only run between renders drops or mis-signs detents on a fast
spin at 16384 (and the OLED, #141, would block the loop further). The
callback therefore **owns every control value the Sequencer reads**; the
main loop is a pure producer that only reads them to render frames (see
THEORY §6). A display goes in the main loop, never the callback (a 128×64
I²C frame is ~10 ms of bus time).

Rates inside the callback: libDaisy's debouncers self-limit to one sample
per millisecond (edges are valid only on the call that sampled them; the
pure `applyDigitalControls()` in `controls_core.h` applies click-then-detent
in that same call). The **knobs are sampled once per millisecond on purpose**:
the smoother and the pickup speed detector are per-call filters tuned at
1 kHz, and sampling them at 1.5 kHz would change how duration and drift feel.
The LEDs are libDaisy software PWM and want the steady callback rate.
Profiler: `det=` (detents applied per second — count them against the
physical clicks); `isr_max` on the COST line includes the panel read. Audio
block is 32 samples.

## Where the logic lives

All the pure decision logic (mode/pickup/drift-fold, encoder stepping, the
stretch detent table, the per-page encoder dispatch, LED colors and levels) is
in the platform-free, host-tested **`controls_core.h`** (`Knob`, `PanelEditor`,
`KNOB_MUX_CH`,
`foldDrift`, `stepAdditive`/`stepIndex`/`stepCount`, `STRETCH_STOPS`,
`EncoderState`/`applyEncoder`/`pageBrightness`, `pageColor`/`stepColor`).
`dreamosc.cpp` is only the hardware glue that reads the panel and calls in. See
the testing-culture note in `../CLAUDE.md`.

## Diagnostics

`make PROFILE=1` prints the full control + instrument state over USB serial each
second (`SET`/`KNOB`/`POS`/`DRF`/`HLTH`/`COST` lines). Every variable parameter must
appear there — see the profiler directive in `../CLAUDE.md`.
