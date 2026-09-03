// controls_core.h - platform-free control-surface logic for the Pod firmware.
//
// The firmware's per-step editing has real decision logic -- knob PICKUP (soft
// takeover), step selection, and folding per-step drift with a global drift --
// that is worth testing on the host. dreamosc.cpp itself can't be host-compiled
// (it pulls in daisy_pod.h), so the pure logic lives here, free of any platform
// header, and dreamosc.cpp is just the glue that reads the hardware and calls in.
//
// NOTE: this is the DEVELOPMENT/access control surface (reach every per-step
// parameter one step at a time), not the intended performance interface (a knob
// per parameter -- see cards #134/#146/#145). It exists to unblock DSP work.
#ifndef CONTROLS_CORE_H
#define CONTROLS_CORE_H

#include <math.h>
#include <stdint.h>

#include "stretch_core.h"   // for SS_STEPS

// Encoder pages (the parameter the encoder turn drives; click cycles). Kept in
// the core so the LED-color mapping below can be tested against it.
enum EncoderPage {
  PAGE_STRETCH  = 0,   // blue
  PAGE_FADE     = 1,   // green
  PAGE_DURATION = 2,   // yellow
  PAGE_DRIFT    = 3,   // purple (GLOBAL drift, added on top of per-step)
  PAGE_COUNT    = 4,
};

struct Rgb { float r, g, b; };

// Fold a step's per-step drift with the global drift: additive, clamped into
// [0,1] position space. Kept separate so the two never entangle (an earlier
// in-place version double-added global every pass -- exactly the bug a test
// catches).
inline float foldDrift(float perStep, float global) {
  float eff = perStep + global;
  if (eff < 0.0f) eff = 0.0f;
  return eff > 1.0f ? 1.0f : eff;
}

// --- Encoder value stepping -------------------------------------------------
// The encoder detent is +-1 (libDaisy Encoder::Increment never returns more),
// so turn SPEED can only be inferred from the time GAP between detents. All the
// encoder-driven controls share this: a small gap (fast spin) uses a coarse
// step, a large gap (deliberate click) a fine one. These helpers are the pure
// math; the caller supplies the gap it measured from System::GetNow().

inline bool encoderFast(uint32_t gapMs, uint32_t fastThreshMs = 40) {
  return gapMs <= fastThreshMs;
}

inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Additive step (fade, global drift): value +/- perDetent*inc, clamped.
inline float stepAdditive(float value, int inc, float perDetent,
                          float lo, float hi) {
  return clampf(value + perDetent * (float)inc, lo, hi);
}

// Ratio step (duration): value * ratio^inc, clamped. inc<0 divides.
inline float stepRatio(float value, int inc, float ratio, float lo, float hi) {
  float f = powf(ratio, (float)(inc < 0 ? -inc : inc));
  float out = inc < 0 ? value / f : value * f;
  return clampf(out, lo, hi);
}

// Index step into a detent table (stretch): idx + stops*inc, clamped to
// [0, count-1]. Returns the new index.
inline int stepIndex(int idx, int inc, int stopsPerDetent, int count) {
  int out = idx + inc * stopsPerDetent;
  if (out < 0) out = 0;
  if (out > count - 1) out = count - 1;
  return out;
}

// One-pole knob smoothing on a raw ADC read. First read jumps to the raw value
// (primed=false), afterward eases toward it. Returns the smoothed value AND
// updates `state`.
inline float smoothKnob(float& state, float raw, bool primed, float coeff = 0.02f) {
  if (!primed) { state = raw; return raw; }
  state += coeff * (raw - state);
  return state;
}

// Advance the encoder page, wrapping.
inline EncoderPage nextPage(EncoderPage p) {
  return (EncoderPage)((p + 1) % PAGE_COUNT);
}

// LED2 color for the encoder page: hue = page, brightness `b` folded in so the
// caller can encode a second signal (crossfade-active) via brightness.
inline Rgb pageColor(EncoderPage page, float b) {
  switch (page) {
    case PAGE_STRETCH:  return {0.0f, 0.0f, b};        // blue
    case PAGE_FADE:     return {0.0f, b,    0.0f};      // green
    case PAGE_DURATION: return {b,    b,    0.0f};      // yellow
    case PAGE_DRIFT:    return {b * 0.6f, 0.0f, b};     // purple
    default:            return {0.0f, 0.0f, 0.0f};
  }
}

// LED1 color for the selected step, ROYGBIVW over the 8 steps. `i` is clamped
// to [0, SS_STEPS-1].
inline Rgb stepColor(int i) {
  static const Rgb table[SS_STEPS] = {
    {0.6f, 0.0f,  0.0f},   // 1 red
    {0.6f, 0.25f, 0.0f},   // 2 orange
    {0.5f, 0.5f,  0.0f},   // 3 yellow
    {0.0f, 0.6f,  0.0f},   // 4 green
    {0.0f, 0.0f,  0.6f},   // 5 blue
    {0.25f,0.0f,  0.6f},   // 6 indigo
    {0.5f, 0.0f,  0.6f},   // 7 violet
    {0.5f, 0.5f,  0.5f},   // 8 white
  };
  if (i < 0) i = 0;
  if (i > SS_STEPS - 1) i = SS_STEPS - 1;
  return table[i];
}

// Per-step knob editing with PICKUP. Two knobs edit the SELECTED step's
// position and (per-step) drift. Selecting a new step does NOT snap it to the
// pot position; a knob takes over its parameter for the current step only once
// it has physically MOVED (> moveThresh raw) since selection. Until then the
// step holds its stored value. This lets you tour the 8 steps and change only
// the ones you touch.
//
// The caller owns the Sequencer (position[]/drift[]); this class owns the edit
// STATE (which step, the pickup latches, the per-step drift shadow, last knob
// reads). It never touches hardware -- the caller passes raw knob values in and
// reads results out.
class StepEditor {
 public:
  // Call once before the first update(), with the initial raw knob reads, so a
  // stationary knob at boot doesn't count as a move.
  void prime(float k1, float k2) {
    k1Last_ = k1;
    k2Last_ = k2;
    primed_ = true;
  }

  bool primed() const { return primed_; }
  int  selected() const { return sel_; }
  float perStepDrift(int i) const { return perStepDrift_[i]; }

  // Advance to the next step (wraps), and RE-ARM pickup for it: its knobs won't
  // take over until moved again.
  void advanceStep() {
    sel_ = (sel_ + 1) % SS_STEPS;
    k1Live_[sel_] = false;
    k2Live_[sel_] = false;
  }

  // One control pass. Raw knob values k1/k2 in [0,1]; on the selected step, once
  // a knob has moved past moveThresh it drives that parameter. Writes the
  // selected step's position (0..1) and the per-step drift shadow (0..driftMax),
  // then folds per-step + global into seq.drift[] for EVERY step.
  void update(Sequencer& seq, float k1, float k2, float globalDrift,
              float moveThresh = 0.02f, float driftMax = 0.25f) {
    if (!primed_) prime(k1, k2);

    if (!k1Live_[sel_] && fabsf(k1 - k1Last_) > moveThresh) k1Live_[sel_] = true;
    if (!k2Live_[sel_] && fabsf(k2 - k2Last_) > moveThresh) k2Live_[sel_] = true;
    if (k1Live_[sel_]) seq.position[sel_]  = k1;
    if (k2Live_[sel_]) perStepDrift_[sel_] = driftMax * k2;
    k1Last_ = k1;
    k2Last_ = k2;

    for (int i = 0; i < SS_STEPS; i++)
      seq.drift[i] = foldDrift(perStepDrift_[i], globalDrift);
  }

 private:
  int   sel_ = 0;
  bool  k1Live_[SS_STEPS] = {false};
  bool  k2Live_[SS_STEPS] = {false};
  float perStepDrift_[SS_STEPS] = {0};
  float k1Last_ = 0.0f, k2Last_ = 0.0f;
  bool  primed_ = false;
};

#endif  // CONTROLS_CORE_H
