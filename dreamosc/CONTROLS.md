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
| **GLOBAL** | duration (0.25..60 s) | global drift (0..0.25) | off |
| **step i** | that step's position (0..1) | that step's per-step drift (0..0.25) | ROYGBIVW (step 1..8) |

- **Global drift is ADDITIVE** on top of each step's per-step drift; effective
  per-step drift = `perStep + global`, clamped to [0,1]. (The two are kept in
  separate storage so a global change never corrupts the per-step shadow — a
  double-add bug that shipped once and is now guarded by a test.)
- **led1 = ROYGBIVW** shows the selected step (red=1 … white=8), off in GLOBAL.

## PICKUP (soft takeover) — everywhere

Landing on any slot (GLOBAL or a step) does **not** snap its value to the pot. A
knob takes over its parameter for the current slot only after it has physically
**moved** since arriving; until then the slot holds its stored value. This lets
you tour the steps and change only the ones you touch.

Two implementation facts that were hard-won bugs:
- **Detect movement on the RAW knob, write the SMOOTHED value.** A one-pole
  smoother caps the per-pass delta below any threshold, so detecting on the
  smoothed read means pickup never engages.
- **Freeze the move reference at an ANCHOR** (the pot position at slot entry) and
  don't update it until the knob engages. If the reference updates every poll it
  *chases* the pot, so a SLOW turn never accumulates a threshold-crossing delta
  and the control feels dead. Anchor + accumulate is the fix.

## Encoder

Turn drives the current page; click cycles the page. led2 = page hue, brightness
= crossfade active (`fade > 0`).

| Page | led2 | encoder turn |
|------|------|--------------|
| **stretch** | blue | index a detent table `STRETCH_STOPS` (1×..10000×) |
| **fade** | green | crossfade overlap 0..0.5 (additive) |
| **frame** | yellow | index `FRAME_STOPS` {256,512,1024,2048,4096} → `seq.setFrame()` |

- **Stretch is a detent table**, not continuous: PaulStretch factors aren't
  perceptually linear, so what matters is the regime (scan/drift/freeze). Fine
  1..10, coarsening up to 10000× (10000× is transient-squelch territory — freezes
  a cymbal hit into wash). Slow click = 1 stop, fast spin = 3 stops/detent.
- **Fade** = crossfade seam overlap. 0 = butt-joint (hard cut). There is no
  separate on/off toggle — fade 0 IS the butt-joint case (a toggle was byte-
  identical to it and was removed, freeing button 2 for the mode navigation).
- **Frame size** is a SOUND-CHARACTER control (Fizzy #136): small = grainy/
  articulated, large = glassy/frozen. Takes effect on the next voice fire.

### Encoder speed model

`Encoder::Increment()` only ever returns ±1, so turn SPEED is inferred from the
time GAP between detents (`encoderFast`, ≤40 ms = fast), NOT from step magnitude.
Fast = coarse step, slow = fine.

### Control polling

Controls poll on a **1 ms wall-clock tick** (`System::GetNow()`), NOT every-Nth-
`service()`: a `service()` call is a full FFT (~2.4 ms), so a service-count gate
polled the encoder only ~6×/s and dropped most detents.

## Where the logic lives

All the pure decision logic (mode/pickup/drift-fold, encoder stepping, LED colors)
is in the platform-free, host-tested **`controls_core.h`** (`PanelEditor`,
`foldDrift`, `stepAdditive`/`stepRatio`/`stepIndex`, `pageColor`/`stepColor`).
`dreamosc.cpp` is only the hardware glue that reads the panel and calls in. See
the testing-culture note in `../CLAUDE.md`.

## Diagnostics

`make PROFILE=1` prints the full control + instrument state over USB serial each
second (`SET`/`KNOB`/`POS`/`DRF`/`HLTH` lines). Every variable parameter must
appear there — see the profiler directive in `../CLAUDE.md`.
