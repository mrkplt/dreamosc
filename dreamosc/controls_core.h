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

// Encoder pages (the parameter the encoder turn drives; click cycles). Duration
// and drift live on the knobs (global mode); these four are the encoder's.
// Click order: stretch -> steps -> fade -> frame(window) -> stretch.
// The page INDEX maps to the ROYGBIVW hue via hueROYGBIVW (0=red..3=green), so
// this order is also the LED color order: stretch=red, steps=orange, fade=yellow,
// window=green.
enum EncoderPage {
  PAGE_STRETCH  = 0,   // red
  PAGE_STEPS    = 1,   // orange (active step count, #149)
  PAGE_FADE     = 2,   // yellow
  PAGE_FRAME    = 3,   // green  (frame/window size, #136)
  PAGE_COUNT    = 4,
};

struct Rgb { float r, g, b; };

// The two clamps every helper below uses. One definition each; no hand-rolled
// min/max chains.
inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
inline int clampi(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
// A hue at a display brightness (both LEDs scale the shared palette this way).
inline Rgb scaleRgb(Rgb h, float b) { return {h.r * b, h.g * b, h.b * b}; }

// Fold a step's per-step drift with the global drift: global drift is the FLOOR
// -- it lifts every step to at least `global`, and a step's own per-step drift
// only takes over when it exceeds the floor. So global sets a baseline shimmer
// on all steps at once, and each step can go HIGHER (never lower) than that with
// its own knob. Clamped into [0,1] position space. Kept a pure function so the
// two never entangle (an earlier additive+in-place version double-added global
// every pass -- exactly the bug a test catches). NOTE: this is max(), not add --
// raising global no longer pushes an already-drifting step even further.
inline float foldDrift(float perStep, float global) {
  return clampf(perStep > global ? perStep : global, 0.0f, 1.0f);
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
  return clampf(v, lo, hi);
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

// Index step into a detent table (stretch): idx + stops*inc, clamped to
// [0, count-1]. Returns the new index.
inline int stepIndex(int idx, int inc, int stopsPerDetent, int count) {
  return clampi(idx + inc * stopsPerDetent, 0, count - 1);
}

// Integer count step (active step count #149): value + inc, clamped to
// [lo, hi]. One unit per detent -- a small integer range wants no fast/coarse
// mode. Returns the new count.
inline int stepCount(int value, int inc, int lo, int hi) {
  return clampi(value + inc, lo, hi);
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

// ROYGBIVW hue at unit intensity (each entry's peak channel = 1.0), the ONE
// palette both LEDs share. led1 (step select) and led2 (encoder page) draw from
// the same eight hues so a color means the same thing on both and orange/yellow
// stay distinguishable (hand-rolled page colors made 2 and 3 read alike). `i` is
// clamped to [0, SS_STEPS-1]. Multiply by a brightness to get a displayable Rgb.
inline Rgb hueROYGBIVW(int i) {
  static const Rgb table[SS_STEPS] = {
    {1.0f,  0.0f,  0.0f},   // 1 red
    {1.0f,  0.45f, 0.0f},   // 2 orange
    {1.0f,  1.0f,  0.0f},   // 3 yellow
    {0.0f,  1.0f,  0.0f},   // 4 green
    {0.0f,  0.0f,  1.0f},   // 5 blue
    {0.4f,  0.0f,  1.0f},   // 6 indigo
    {0.7f,  0.0f,  1.0f},   // 7 violet
    {1.0f,  1.0f,  1.0f},   // 8 white
  };
  return table[clampi(i, 0, SS_STEPS - 1)];
}

// LED2 color for the encoder page: the page's ROYGBIVW hue (RoYG over the four
// pages, in click order -- SAME palette as led1) scaled by brightness `b`. Hue
// = which parameter; `b` = that parameter's LEVEL (every page tracks its own
// encoded level -- stretch/fade/frame/steps brightness via the *Brightness
// helpers below). So a bright LED always means "this parameter is turned up".
inline Rgb pageColor(EncoderPage page, float b) {
  return scaleRgb(hueROYGBIVW((int)page), b);   // pages 0..3 -> red/orange/yellow/green
}

// led2 brightness convention: EVERY page's LED intensity tracks that page's
// encoded LEVEL, so brightness is always "how far up this parameter is" and the
// hue is just which parameter. Map a normalized level `t` (0..1) to a displayable
// [floorB, 1.0] brightness -- a small floor so the bottom of the range is still
// visibly lit rather than off. All the per-page helpers below feed this.
inline float levelBrightness(float t, float floorB = 0.15f) {
  return floorB + (1.0f - floorB) * clampf(t, 0.0f, 1.0f);
}

// PAGE_STEPS green intensity = active step count, count in [1, SS_STEPS].
inline float stepBrightness(int activeSteps, float floorB = 0.15f) {
  int n = ssClampSteps(activeSteps);
  return levelBrightness((float)(n - 1) / (float)(SS_STEPS - 1), floorB);
}

// PAGE_STRETCH red intensity = stretch detent index, idx in [0, count-1].
inline float stretchBrightness(int idx, int count, float floorB = 0.15f) {
  if (count < 2) return 1.0f;
  int i = clampi(idx, 0, count - 1);
  return levelBrightness((float)i / (float)(count - 1), floorB);
}

// PAGE_FADE orange intensity = crossfade amount over its full range [0, fadeMax]
// (default 0.5 = the overlap ceiling). fade 0 (butt-joint) sits at the dim floor.
inline float fadeBrightness(float fade, float fadeMax = 0.5f, float floorB = 0.15f) {
  return levelBrightness(fadeMax > 0.0f ? fade / fadeMax : 0.0f, floorB);
}

// PAGE_FRAME green intensity tracks the KNOB position, not window size: it
// follows the detent INDEX so counterclockwise (lower idx) dims and clockwise
// (higher idx) brightens -- the light moves the way the knob turns. The table is
// largest-first (idx 0 = SS_W), so CW also shrinks the window; brightness rising
// as the window shrinks is intentional (it tracks the knob, not the size).
inline float frameBrightness(int idx, int count, float floorB = 0.15f) {
  return stretchBrightness(idx, count, floorB);   // idx 0 dim -> max idx bright
}

// LED1 color for the selected step, ROYGBIVW over the 8 steps, at a fixed
// display brightness. Draws from the shared hueROYGBIVW palette. `i` is clamped
// to [0, SS_STEPS-1].
inline Rgb stepColor(int i, float b = 0.6f) {
  return scaleRgb(hueROYGBIVW(i), b);
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
// values. Effective drift per step = max(per-step, global), clamped (foldDrift).
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

  // button1: GLOBAL -> step1 -> ... -> stepN -> GLOBAL, where N = activeSteps
  // (#149): the nav only visits ACTIVE steps, so shrinking the sequence shrinks
  // the tour. `activeSteps` is clamped into [1, SS_STEPS]; default SS_STEPS
  // preserves the old full-tour behavior. Re-arms pickup for the slot we land on.
  void advance(int activeSteps = SS_STEPS) {
    int n = ssClampSteps(activeSteps);
    // Slots in play: GLOBAL (0) + steps 1..n, so (n + 1) slots, wrapping.
    goTo((slot_ + 1) % (n + 1));
  }
  // button2: shortcut back to GLOBAL (re-arms its pickup).
  void toGlobal() { goTo(PE_GLOBAL); }

  // Keep the editor on a valid slot when the active step count shrinks (#149):
  // if we're parked on a step past the new count, jump back to GLOBAL so the
  // knobs never edit an inactive step. Call after the step count changes.
  void clampToActive(int activeSteps) {
    if (slot_ > ssClampSteps(activeSteps)) goTo(PE_GLOBAL);
  }

  // One control pass. Movement is detected on the RAW knob (r1/r2) -- the
  // physical pot position -- while the VALUE written uses the smoothed knob
  // (k1/k2). Detecting on the smoothed value is wrong: a one-pole caps the
  // per-pass delta below any sane threshold (turning a pot to the far end moves
  // the smoothed read only ~2% that pass), so pickup would never engage. The
  // pickup latch tracks the pot; the smoothing only de-zippers the output.
  // In GLOBAL, knobs drive *dur and *gdrift; in a step, position + per-step
  // drift. Always folds max(per-step, global) into seq.drift[].
  void update(Sequencer& seq, float* dur, float* gdrift,
              float r1, float r2, float k1, float k2,
              float moveThresh = 0.02f, float driftMax = 0.03f,
              float durMin = 0.25f, float durMax = 60.0f, float gdriftMax = 0.03f) {
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
    // Drift: fast = driftMax/30, slow = 0.03% (0.0003) -- 0..driftMax(3%) is
    // 100 fine detents of 0.03% each. Duration: continuous both (a smooth
    // sweep; grid TBD). Global drift: same as per-step drift.
    const KnobSpec POSITION { 0.0f, 1.0f, 0.05f, 0.0f };
    const KnobSpec DRIFT    { 0.0f, driftMax, driftMax / 30.0f, 0.0003f };
    const KnobSpec DURATION { durMin, durMax, 0.0f, 0.0f };
    const KnobSpec GDRIFT   { 0.0f, gdriftMax, gdriftMax / 30.0f, 0.0003f };

    if (slot_ == PE_GLOBAL) {
      if (k1Live_[slot_]) *dur    = applyKnob(DURATION, k1, spd1);
      if (k2Live_[slot_]) *gdrift = applyKnob(GDRIFT,   k2, spd2);
    } else {
      int s = slot_ - 1;
      // POSITION is written from the RAW pot (r1), not the smoothed read: the
      // core applies position at frame boundaries (no zipper to smooth), and a
      // one-pole's ~250 ms creep would cost a re-render per hop for its whole
      // tail (finding F8) and add its own settling latency. The fast-move grid
      // in applyKnob still snaps; ADC jitter sits below the core's refresh
      // threshold. Duration and drift keep the smoother.
      if (k1Live_[slot_]) seq.position[s]  = applyKnob(POSITION, r1, spd1);
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

// --- Encoder: the detent tables, the page state, and one detent's effect ----
// These used to live in dreamosc.cpp (un-host-compilable): the stretch table,
// the frame-index state, the per-page dispatch and the led2 brightness switch.
// They are pure decisions over (page, inc, fast) and belong here, tested.

// Stretch is a fixed, musically-spaced DETENT TABLE rather than a continuous
// range: PaulStretch factors are not perceptually linear, so what matters is the
// regime (scan / drift / freeze), not the exact number. Fine 1..10, then coarser
// as character stops changing: by 2 to 20, by 5 to 50, by 10 to 100, by 25 to
// 300, by 100 to 1000. The encoder moves an INDEX into this table.
constexpr float STRETCH_STOPS[] = {
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
  12, 14, 16, 18, 20,
  25, 30, 35, 40, 45, 50,
  60, 70, 80, 90, 100,
  125, 150, 175, 200, 225, 250, 275, 300,
  400, 500, 600, 700, 800, 900, 1000,
  // Above 1000x is transient-squelch territory: 1000x still lets a sharp hit
  // (a cymbal) punch through as a transient; ~10000x freezes it into sustained
  // wash. By 500s through the low thousands (where the freeze character still
  // changes), then by 1000s to 10000x.
  1500, 2000, 2500, 3000,
  4000, 5000, 6000, 7000, 8000, 9000, 10000,
};
constexpr int STRETCH_NSTOPS = (int)(sizeof(STRETCH_STOPS) / sizeof(STRETCH_STOPS[0]));   // 52
constexpr int STRETCH_DEFAULT_IDX = 20;   // 50x
static_assert(STRETCH_STOPS[STRETCH_DEFAULT_IDX] == 50.0f, "boot stretch is 50x");
// The stretch factor at a detent index (clamped into the table).
inline float stretchStop(int idx) { return STRETCH_STOPS[clampi(idx, 0, STRETCH_NSTOPS - 1)]; }

// Frame/window size (#136) is the core's own size index: 0 = SS_W (16384,
// ~0.34 s, PaulXStretch's shimmer regime) ... SS_NSIZES-1 = SS_W_MIN, i.e.
// ssSizeW(idx). LARGEST FIRST, so a clockwise detent (inc +1) walks toward
// SMALLER windows: turning right shrinks the frame. Smaller = grainier/more
// articulated/wobbly on tonal material; larger = glassy/frozen shimmer. The
// value goes to seq.setFrame(), which is LIVE: every sounding head re-renders a
// pre-roll pair at the new size for its next hop boundary. The DEFAULT is the
// core's SS_W_DEFAULT (4096, not the max) -- derived, so host tests and the
// device cannot boot at different windows.
constexpr int FRAME_DEFAULT_IDX = ssSizeIdx(SS_W_DEFAULT);

// The encoder's state: which page it drives, and the two index-valued
// parameters (stretch detent, frame size) whose VALUE lives in the Sequencer
// but whose INDEX is the encoder's.
struct EncoderState {
  EncoderPage page       = PAGE_STRETCH;
  int         stretchIdx = STRETCH_DEFAULT_IDX;
  int         frameIdx   = FRAME_DEFAULT_IDX;
};

// Push the encoder's index-valued parameters into the Sequencer (boot sync).
inline void encoderSync(const EncoderState& e, Sequencer& seq) {
  seq.stretch = stretchStop(e.stretchIdx);
  seq.setFrame(ssSizeW(e.frameIdx));
}

// One encoder detent (`inc` = +-1, `fast` from encoderFast) on the current
// page. Stretch: 3 stops per detent on a fast spin, 1 on a slow click. Frame:
// one size per detent. Steps (#149): one per detent (a small integer range
// wants no fast/coarse mode), and the panel nav is kept on a valid slot if the
// count shrank past the selected step. Fade: additive, 0.04 fast / 0.005 slow
// per detent over 0..0.5.
inline void applyEncoder(EncoderState& e, Sequencer& seq, PanelEditor& panel,
                         int inc, bool fast) {
  if (inc == 0) return;
  switch (e.page) {
    case PAGE_STRETCH:
      e.stretchIdx = stepIndex(e.stretchIdx, inc, fast ? 3 : 1, STRETCH_NSTOPS);
      seq.stretch  = stretchStop(e.stretchIdx);
      break;
    case PAGE_FRAME:
      e.frameIdx = stepIndex(e.frameIdx, inc, 1, SS_NSIZES);
      seq.setFrame(ssSizeW(e.frameIdx));
      break;
    case PAGE_STEPS:
      seq.setSteps(stepCount(seq.activeSteps, inc, 1, SS_STEPS));
      panel.clampToActive(seq.activeSteps);
      break;
    case PAGE_FADE:
      seq.fade = stepAdditive(seq.fade, inc, fast ? 0.04f : 0.005f, 0.0f, 0.5f);
      break;
    default:
      break;
  }
}

// led2 brightness for the current page = that page's encoded LEVEL, so a bright
// LED always means "this parameter is turned up":
//   stretch -> stretch detent index, steps -> active step count,
//   fade -> crossfade amount (0..0.5), frame -> frame-size index (CW bright).
inline float pageBrightness(const EncoderState& e, const Sequencer& seq) {
  switch (e.page) {
    case PAGE_STRETCH: return stretchBrightness(e.stretchIdx, STRETCH_NSTOPS);
    case PAGE_FADE:    return fadeBrightness(seq.fade);
    case PAGE_FRAME:   return frameBrightness(e.frameIdx, SS_NSIZES);
    case PAGE_STEPS:   return stepBrightness(seq.activeSteps);
    default:           return levelBrightness(0.0f);
  }
}

#endif  // CONTROLS_CORE_H
