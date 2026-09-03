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

// Encoder pages (the parameter the encoder turn drives; click cycles):
// stretch, fade, frame size (#136), step count (#149). Duration and drift live
// on the knobs (global mode). LED2 shows the page as RoYG (see pageColor).
enum EncoderPage {
  PAGE_STRETCH  = 0,   // blue
  PAGE_FADE     = 1,   // green
  PAGE_FRAME    = 2,   // yellow (frame/window size, #136)
  PAGE_STEPS    = 3,   // cyan (active step count, #149)
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

// --- Speed-adaptive pot quantization ----------------------------------------
// A knob's raw ADC read is effectively continuous (12-bit + noise), so you
// can't reliably land on a clean value. These map a raw pot to a value with
// SPEED-ADAPTIVE resolution, the same fast/slow philosophy as the encoder: a
// FAST move (large per-poll raw delta) snaps to a COARSE grid so you land on
// round landmarks; a SLOW move uses a FINE grid (or continuous) for exact
// placement. `speed` is the caller's measured per-poll |Δraw| (raw is 0..1).
//
// Speed threshold: per-poll raw delta above which a move counts as FAST. At the
// 1 kHz control poll a deliberate spin moves the pot several % per poll; a slow
// dial creeps <1%. 0.01 (1% of full travel per poll) separates them well.
inline bool potFast(float speed, float thresh = 0.01f) { return speed > thresh; }

// Snap `value` to a grid of `grid` size (grid<=0 = continuous, pass-through),
// clamped to [lo, hi].
inline float snapTo(float value, float grid, float lo, float hi) {
  float v = value;
  if (grid > 0.0f) v = (float)((int)(value / grid + 0.5f)) * grid;
  return v < lo ? lo : (v > hi ? hi : v);
}

// One knob's behavior, reusable across all pots so FEEL stays consistent as we
// add knobs -- the only per-knob differences are the RANGE and the two GRIDS.
// Maps a normalized 0..1 pot read to a parameter value in [lo, hi] with
// SPEED-ADAPTIVE resolution: a FAST move snaps to `fastGrid` (coarse landmarks),
// a SLOW move to `slowGrid` (fine placement; 0 = continuous). Grids are in the
// OUTPUT units, so e.g. position lo=0 hi=1 fastGrid=0.05 (5%) slowGrid=0 (cont),
// or drift lo=0 hi=0.25 fastGrid=0.25/30 slowGrid=0.001 (0.1%).
struct KnobSpec {
  float lo, hi;         // parameter range
  float fastGrid;       // coarse grid, used on a fast move
  float slowGrid;       // fine grid (0 = continuous), used on a slow move
};

// Apply a KnobSpec: scale the 0..1 read to [lo,hi], pick the grid by speed,
// snap. `speed` is |Δraw| this poll (raw in 0..1).
inline float applyKnob(const KnobSpec& k, float read01, float speed) {
  float v = k.lo + (k.hi - k.lo) * read01;
  float grid = potFast(speed) ? k.fastGrid : k.slowGrid;
  return snapTo(v, grid, k.lo, k.hi);
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

// Integer count step (active step count #149): value + inc, clamped to
// [lo, hi]. One unit per detent -- a small integer range wants no fast/coarse
// mode. Returns the new count.
inline int stepCount(int value, int inc, int lo, int hi) {
  int out = value + inc;
  if (out < lo) out = lo;
  if (out > hi) out = hi;
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

// LED2 color for the encoder page: hue = page (RoYG over the four pages, in
// click order), brightness `b` folded in so the caller can encode a second
// signal via brightness. On PAGE_STEPS the brightness encodes the active step
// count (see stepBrightness); on the others it encodes crossfade-active.
inline Rgb pageColor(EncoderPage page, float b) {
  switch (page) {
    case PAGE_STRETCH:  return {b,        0.0f,      0.0f};   // red
    case PAGE_FADE:     return {b,        b * 0.35f, 0.0f};   // orange
    case PAGE_FRAME:    return {b * 0.5f, b * 0.5f,  0.0f};   // yellow
    case PAGE_STEPS:    return {0.0f,     b,         0.0f};   // green
    default:            return {0.0f,     0.0f,      0.0f};
  }
}

// Brightness for the PAGE_STEPS LED: green whose INTENSITY tracks the active
// step count -- fewer steps dim, the full sequence bright -- so the count reads
// at a glance without a separate display. Maps count in [1, SS_STEPS] to a
// [floorB, 1.0] brightness (a small floor so 1 step is still visibly lit).
inline float stepBrightness(int activeSteps, float floorB = 0.15f) {
  int n = activeSteps < 1 ? 1 : (activeSteps > SS_STEPS ? SS_STEPS : activeSteps);
  float t = (float)(n - 1) / (float)(SS_STEPS - 1);   // 0 at 1 step, 1 at SS_STEPS
  return floorB + (1.0f - floorB) * t;
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
  void prime(float r1, float r2) {
    k1Anchor_ = r1; k2Anchor_ = r2;
    r1Prev_ = r1; r2Prev_ = r2;   // so the first pass reads speed 0, not a jump
    primed_ = true;
  }
  bool primed() const { return primed_; }

  int  slot() const { return slot_; }        // 0 = global, 1..SS_STEPS = step
  bool inGlobal() const { return slot_ == PE_GLOBAL; }
  int  step() const { return slot_ - 1; }    // valid only when !inGlobal()
  float perStepDrift(int i) const { return perStepDrift_[i]; }
  // Diagnostic: has each knob's pickup engaged on the CURRENT slot?
  bool k1Live() const { return k1Live_[slot_]; }
  bool k2Live() const { return k2Live_[slot_]; }
  // Diagnostic: last per-poll knob speed (|Δraw|) and its fast/slow verdict,
  // for tuning the potFast threshold from board data.
  float speed1() const { return spd1_; }
  float speed2() const { return spd2_; }
  bool  fast1()  const { return potFast(spd1_); }
  bool  fast2()  const { return potFast(spd2_); }
  float anchor1() const { return k1Anchor_; }
  float anchor2() const { return k2Anchor_; }

  // button1: GLOBAL -> step1 -> ... -> stepN -> GLOBAL, where N = activeSteps
  // (#149): the nav only visits ACTIVE steps, so shrinking the sequence shrinks
  // the tour. `activeSteps` is clamped into [1, SS_STEPS]; default SS_STEPS
  // preserves the old full-tour behavior. Re-arms pickup for the slot we land on.
  void advance(int activeSteps = SS_STEPS) {
    int n = activeSteps < 1 ? 1 : (activeSteps > SS_STEPS ? SS_STEPS : activeSteps);
    // Slots in play: GLOBAL (0) + steps 1..n, so (n + 1) slots, wrapping.
    goTo((slot_ + 1) % (n + 1));
  }
  // button2: shortcut back to GLOBAL (re-arms its pickup).
  void toGlobal() { goTo(PE_GLOBAL); }

  // Keep the editor on a valid slot when the active step count shrinks (#149):
  // if we're parked on a step past the new count, jump back to GLOBAL so the
  // knobs never edit an inactive step. Call after the step count changes.
  void clampToActive(int activeSteps) {
    int n = activeSteps < 1 ? 1 : (activeSteps > SS_STEPS ? SS_STEPS : activeSteps);
    if (slot_ > n) goTo(PE_GLOBAL);
  }

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

    // Per-poll knob SPEED (|Δraw| since last update). All four knob targets use
    // the SAME speed-adaptive behavior (applyKnob) so feel is consistent; only
    // the KnobSpec (range + fast/slow grids) differs per target.
    float spd1 = fabsf(r1 - r1Prev_);
    float spd2 = fabsf(r2 - r2Prev_);
    spd1_ = spd1; spd2_ = spd2;   // exposed for the profiler (threshold tuning)
    r1Prev_ = r1;
    r2Prev_ = r2;

    // Per-knob specs. Position: fast = 5% (20 detents), slow = continuous.
    // Drift: fast = driftMax/30, slow = 0.1%. Duration: continuous both (a
    // smooth sweep; grid TBD). Global drift: same as per-step drift.
    const KnobSpec POSITION { 0.0f, 1.0f, 0.05f, 0.0f };
    const KnobSpec DRIFT    { 0.0f, driftMax, driftMax / 30.0f, 0.001f };
    const KnobSpec DURATION { durMin, durMax, 0.0f, 0.0f };
    const KnobSpec GDRIFT   { 0.0f, gdriftMax, gdriftMax / 30.0f, 0.001f };

    if (slot_ == PE_GLOBAL) {
      if (k1Live_[slot_]) *dur    = applyKnob(DURATION, k1, spd1);
      if (k2Live_[slot_]) *gdrift = applyKnob(GDRIFT,   k2, spd2);
    } else {
      int s = slot_ - 1;
      if (k1Live_[slot_]) seq.position[s]  = applyKnob(POSITION, k1, spd1);
      if (k2Live_[slot_]) perStepDrift_[s] = applyKnob(DRIFT,    k2, spd2);
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
  float r1Prev_ = 0.0f, r2Prev_ = 0.0f;       // last raw reads (per-poll speed)
  float spd1_ = 0.0f, spd2_ = 0.0f;           // last measured speed (diagnostic)
  bool  anchorPending_ = false;
  bool  primed_ = false;
};

#endif  // CONTROLS_CORE_H
