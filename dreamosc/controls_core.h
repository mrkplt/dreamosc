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

// Encoder pages (the parameter the encoder turn drives; click cycles). Just
// stretch and fade -- duration and drift moved to the knobs (global mode).
enum EncoderPage {
  PAGE_STRETCH  = 0,   // blue
  PAGE_FADE     = 1,   // green
  PAGE_COUNT    = 2,
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

// Two-knob panel editor with a mode + PICKUP everywhere.
//
// MODES, cycled by button1: GLOBAL (slot 0) then steps 1..SS_STEPS, then back to
// GLOBAL (a SS_STEPS+1 loop). button2 jumps straight to GLOBAL.
//   - GLOBAL: knob1 = global duration, knob2 = global drift.
//   - step i: knob1 = that step's position, knob2 = that step's per-step drift.
//
// PICKUP everywhere: landing on any slot (global or a step) does NOT snap its
// value to the pot. A knob takes over its parameter for the CURRENT slot only
// after it has physically MOVED (> moveThresh) since arriving. Each of the
// SS_STEPS+1 slots has its own k1/k2 latch, so touring never disturbs untouched
// values. Effective drift per step = per-step + global, clamped.
//
// The caller owns the Sequencer and the global duration/drift floats; this class
// owns the mode/pickup/shadow state and never touches hardware.
constexpr int PE_GLOBAL = 0;                 // slot 0 = global page
constexpr int PE_NSLOTS = SS_STEPS + 1;      // global + 8 steps

class PanelEditor {
 public:
  // Prime with the initial RAW knob reads so a stationary pot at boot isn't a
  // move.
  void prime(float r1, float r2) { k1Anchor_ = r1; k2Anchor_ = r2; primed_ = true; }
  bool primed() const { return primed_; }

  int  slot() const { return slot_; }        // 0 = global, 1..SS_STEPS = step
  bool inGlobal() const { return slot_ == PE_GLOBAL; }
  int  step() const { return slot_ - 1; }    // valid only when !inGlobal()
  float perStepDrift(int i) const { return perStepDrift_[i]; }
  // Diagnostic: has each knob's pickup engaged on the CURRENT slot?
  bool k1Live() const { return k1Live_[slot_]; }
  bool k2Live() const { return k2Live_[slot_]; }
  float anchor1() const { return k1Anchor_; }
  float anchor2() const { return k2Anchor_; }

  // button1: GLOBAL -> step1 -> ... -> step8 -> GLOBAL. Re-arms pickup for the
  // slot we land on.
  void advance() { goTo((slot_ + 1) % PE_NSLOTS); }
  // button2: shortcut back to GLOBAL (re-arms its pickup).
  void toGlobal() { goTo(PE_GLOBAL); }

  // One control pass. Movement is detected on the RAW knob (r1/r2) -- the
  // physical pot position -- while the VALUE written uses the smoothed knob
  // (k1/k2). Detecting on the smoothed value is wrong: a one-pole caps the
  // per-pass delta below any sane threshold (turning a pot to the far end moves
  // the smoothed read only ~2% that pass), so pickup would never engage. The
  // pickup latch tracks the pot; the smoothing only de-zippers the output.
  // In GLOBAL, knobs drive *dur and *gdrift; in a step, position + per-step
  // drift. Always folds per-step + global into seq.drift[].
  void update(Sequencer& seq, float* dur, float* gdrift,
              float r1, float r2, float k1, float k2,
              float moveThresh = 0.02f, float driftMax = 0.25f,
              float durMin = 0.25f, float durMax = 60.0f, float gdriftMax = 0.25f) {
    if (!primed_) { prime(r1, r2); }

    // Anchor pending from a slot change (goTo can't see the raw reads): capture
    // the pot position on arrival as this slot's move reference.
    if (anchorPending_) { k1Anchor_ = r1; k2Anchor_ = r2; anchorPending_ = false; }

    // Pickup: measure movement from the ANCHOR (pot position at entry), which is
    // FROZEN until the knob engages -- so a SLOW sweep accumulates and eventually
    // crosses the threshold. (Comparing against the previous poll let the
    // reference chase the pot, so a slow turn never accumulated a crossing delta
    // and pickup never engaged -- the "control feels dead" bug.) After engaging,
    // the anchor is irrelevant; the knob drives the value directly.
    if (!k1Live_[slot_] && fabsf(r1 - k1Anchor_) > moveThresh) k1Live_[slot_] = true;
    if (!k2Live_[slot_] && fabsf(r2 - k2Anchor_) > moveThresh) k2Live_[slot_] = true;

    if (slot_ == PE_GLOBAL) {
      if (k1Live_[slot_]) *dur    = durMin + (durMax - durMin) * k1;   // duration
      if (k2Live_[slot_]) *gdrift = gdriftMax * k2;                    // global drift
    } else {
      int s = slot_ - 1;
      if (k1Live_[slot_]) seq.position[s]  = k1;                       // position
      if (k2Live_[slot_]) perStepDrift_[s] = driftMax * k2;            // per-step drift
    }

    for (int i = 0; i < SS_STEPS; i++)
      seq.drift[i] = foldDrift(perStepDrift_[i], *gdrift);
  }

 private:
  void goTo(int slot) {
    slot_ = slot;
    k1Live_[slot_] = false;   // re-arm pickup on arrival
    k2Live_[slot_] = false;
    anchorPending_ = true;    // next update() captures the anchor from raw reads
  }

  int   slot_ = PE_GLOBAL;
  bool  k1Live_[PE_NSLOTS] = {false};
  bool  k2Live_[PE_NSLOTS] = {false};
  float perStepDrift_[SS_STEPS] = {0};
  float k1Anchor_ = 0.0f, k2Anchor_ = 0.0f;   // frozen move reference (per entry)
  bool  anchorPending_ = false;
  bool  primed_ = false;
};

#endif  // CONTROLS_CORE_H
