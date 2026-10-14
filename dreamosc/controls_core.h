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
#include "analog_input.h"   // what a Knob reads

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
// A knob's raw ADC read is effectively continuous (16-bit + noise), so you
// can't reliably land on a clean value. These map a pot to a value with
// SPEED-ADAPTIVE resolution, the same fast/slow philosophy as the encoder: a
// FAST move snaps to a COARSE grid so you land on round landmarks; a SLOW move
// uses a FINE grid (or continuous) for exact placement. `speed` is the
// caller's measured |d raw / dt| in pot travel per SECOND (raw is 0..1).
//
// Speed threshold, in travel per second: 10 (a full sweep in 0.1 s). This is
// the bench-tuned 0.01-per-1-ms-poll threshold expressed in time rather than
// per call, so it means the same thing at whatever rate the knobs are polled.
constexpr float POT_FAST_SPEED = 10.0f;
inline bool potFast(float speed, float thresh = POT_FAST_SPEED) { return speed > thresh; }

// Snap `value` to a grid of `grid` size (grid<=0 = continuous, pass-through),
// clamped to [lo, hi].
inline float snapTo(float value, float grid, float lo, float hi) {
  float v = value;
  if (grid > 0.0f) v = (float)((int)(value / grid + 0.5f)) * grid;
  return clampf(v, lo, hi);
}

// --- Pot conditioning, ported from Mutable Instruments Beads -----------------
// Beads (pichenettes/eurorack, beads/cv_reader.h PotChannel, beads/
// pot_controller.h, stmlib/dsp/hysteresis_filter.h; MIT) conditions every pot
// as: one-pole -> hysteresis, then a PotController for soft takeover. (Beads'
// PotChannel also applies per-pot response laws -- attenuverter, detune --
// that are Beads' own sound design, not part of the pot primitive; not here.)
// The constants below are Beads', converted from "per call at Beads' rate" to
// rate-free units where they were per call, so they mean the same here at any
// control rate. Beads calls CvReader::Read() once per audio block at its
// kBlockRate = 4000 Hz (beads.cc).

// One-pole time constant. Beads: ONE_POLE(lp_, v, 0.05f) per 4 kHz block,
// i.e. tau = -0.25 ms / ln(1 - 0.05) = 4.874 ms. We keep the TIME; the
// coefficient for an interval dt is 1 - exp(-dt / tau), which is exactly
// 0.05 at dt = 0.25 ms (tested).
constexpr float KNOB_TAU_S = 4.8739e-3f;
inline float onePoleCoeff(float dt, float tau) {
  return dt > 0.0f ? 1.0f - expf(-dt / tau) : 0.0f;
}

// Hysteresis threshold, in pot travel (a level, so rate-free). Beads uses
// 0.001 on its TIME pot -- the one whose wobble would be heard -- and 0 on
// the rest. Every pot here drives something a wobble would cost: position
// re-renders a frame on a 0.002 change (finding F8), duration and drift move
// the sound, and the OLED shows positions. So every KnobSpec defaults to it.
constexpr float KNOB_HYSTERESIS = 0.001f;

// stmlib's HysteresisFilter: the output follows the input at a distance of
// `threshold` once the input moves farther than that, and holds otherwise.
// After a move the output sits exactly `threshold` behind the input, so input
// noise of up to 2 x threshold peak-to-peak never moves it. Unlike a deadband
// that jumps to the input, a slow turn moves the output continuously. One
// deviation from stmlib: a Knob seats `value` at its first read instead of
// starting from 0 (Beads' one-pole also starts from 0 and ramps; ours seats,
// so a pot at rest at boot is not a move).
struct Hysteresis {
  float value = 0.0f;
  float process(float x, float threshold) {
    if (threshold == 0.0f) {
      value = x;
    } else {
      const float error = x - value;
      if (error > threshold)       value = x - threshold;
      else if (error < -threshold) value = x + threshold;
    }
    return value;
  }
};

// One knob's behavior, reusable across all pots so FEEL stays consistent as we
// add knobs -- the only per-knob differences are the RANGE, the two GRIDS and
// the hysteresis. Maps a conditioned 0..1 pot to a parameter value in
// [lo, hi] with SPEED-ADAPTIVE resolution: a FAST move snaps to `fastGrid`
// (coarse landmarks), a SLOW move to `slowGrid` (fine placement; 0 =
// continuous). Grids are in the OUTPUT units, so e.g. position lo=0 hi=1
// fastGrid=0.05 (5%) slowGrid=0 (cont), or drift lo=0 hi=0.03
// fastGrid=0.03/30 slowGrid=0.0003.
struct KnobSpec {
  float  lo, hi;                         // parameter range
  float  fastGrid;                       // coarse grid, used on a fast move
  float  slowGrid;                       // fine grid (0 = continuous), used on a slow move
  float  hysteresis = KNOB_HYSTERESIS;   // pot travel; 0 = none (Beads' default)
};

// Apply a KnobSpec: scale the 0..1 read to [lo,hi], pick the grid by speed,
// snap. `speed` is |d raw / dt| in travel per second.
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

// Soft takeover: the CATCHING_UP algorithm of Beads' PotController
// (beads/pot_controller.h), the part of it that is a pot primitive. (The
// rest of PotController -- LOCKING / HIDDEN_PARAMETER and scale/offset -- is
// Beads' shift-button hidden-parameter UI and parameter mapping: Beads uses
// it only on its three mix pots, and the KnobSpec does our mapping. Not
// here.) Its input is the conditioned pot (0..1); main() is the 0..1 value
// it drives.
//   TRACKING:    main follows the pot.
//   CATCHING_UP: main moves RELATIVELY: each pot move of more than
//                CATCH_MOVE moves main by the same amount scaled by a skew
//                ratio that makes main and the pot reach the end of travel
//                together, so it never jumps; once they meet (within
//                CATCH_MOVE) it is TRACKING again.
// The move is measured against a REFERENCE that only advances when the
// threshold is crossed, so small moves accumulate and the result does not
// depend on how often process() is called (Beads runs it at 1 kHz; a Knob
// runs it on every poll).
//
// Entry: Beads enters CATCHING_UP when a hidden-parameter edit ends
// (Unlock). A Knob enters it whenever it starts addressing a parameter whose
// stored value need not be where the pot is (boot, a panel slot change):
// catchUp().
enum PotState : uint8_t {
  POT_TRACKING,
  POT_CATCHING_UP,
};

class PotTakeover {
 public:
  static constexpr float CATCH_MOVE = 0.005f;

  // main holds `stored` (0..1) until the pot, now at `pot`, moves; then it
  // catches up without a jump.
  void catchUp(float stored, float pot) {
    stored_ = clampf(stored, 0.0f, 1.0f);
    previous_ = pot;
    main_ = stored_;
    state_ = POT_CATCHING_UP;
  }

  // One update with the conditioned pot `v`. Returns true when main_ moved
  // in CATCHING_UP (a Knob's "engaged" signal); TRACKING moves it every call.
  bool process(float v) {
    if (state_ == POT_TRACKING) {
      main_ = previous_ = v;
      return false;
    }
    if (fabsf(v - previous_) <= CATCH_MOVE) return false;
    const float delta = v - previous_;
    float skew = delta > 0.0f
        ? (1.001f - stored_) / (1.001f - previous_)
        : (0.001f + stored_) / (0.001f + previous_);
    skew = clampf(skew, 0.1f, 10.0f);
    stored_ = clampf(stored_ + skew * delta, 0.0f, 1.0f);
    if (fabsf(stored_ - v) < CATCH_MOVE) state_ = POT_TRACKING;
    previous_ = v;
    main_ = stored_;
    return true;
  }

  float    main() const  { return main_; }
  PotState state() const { return state_; }

 private:
  PotState state_ = POT_TRACKING;
  float    main_ = 0.0f, stored_ = 0.0f, previous_ = 0.0f;
};

// ONE physical pot: every knob on the instrument is a Knob, so they all share
// the same feel and the same conditioning. A Knob is injected with the
// AnalogInput it reads (analog_input.h: an ADC pin, a mux channel, a test
// fake -- it never knows which) and BOUND to the parameter it writes (a float
// and the KnobSpec that maps onto it). poll(dt) is called with the time since
// this knob's previous poll -- any rate; nothing in it is per-call -- and
// keeps, in Beads' order:
//   - ONE-POLE (KNOB_TAU_S) -> HYSTERESIS (spec.hysteresis): the
//     conditioned 0..1 read. A resting pot's flutter is averaged by the
//     one-pole and stopped by the hysteresis, so the conditioned read is
//     constant at rest, and the one-pole's settling tail ends inside the
//     hysteresis too (no creep reaches the parameter).
//   - TAKEOVER (PotTakeover): arriving at a parameter (boot, rearm())
//     never writes it; the first move of more than CATCH_MOVE starts moving
//     it relatively from its stored value, and it converges on the pot
//     without a jump, then tracks.
//   - SPEED: |d raw / dt|, travel per second, for the speed-adaptive grid
//     (applyKnob). Raw, so a real turn is never hidden by the filters.
class Knob {
 public:
  explicit Knob(const AnalogInput& in) : in_(&in) {}
  Knob(const AnalogInput& in, float* target, const KnobSpec& spec) : in_(&in) { bind(target, spec); }

  // Point the knob at a parameter. Binding does NOT re-arm takeover (rearm()
  // does), so a caller may re-bind every poll. Unbound, the knob still reads
  // and tracks (conditioning, takeover, speed stay current) but writes
  // nothing.
  void bind(float* target, const KnobSpec& spec) { target_ = target; spec_ = spec; }
  void unbind() { target_ = nullptr; }

  // Seat the filters where the input is now, so a stationary pot is not a
  // move. Does nothing until the input is ready; the first ready poll()
  // seats itself if this was never called.
  void prime() { if (in_->ready()) seat(in_->read()); }

  // Start addressing the bound parameter afresh: nothing is written until
  // the pot moves, then it catches up from the parameter's stored value
  // (read at the next poll, so bind first or in the same pass).
  void rearm() { live_ = false; catchPending_ = true; }

  // One poll, `dt` seconds after this knob's previous one: read the input,
  // condition it, run takeover, and write the bound parameter when live.
  // Returns false (and does nothing at all) while the input is not ready.
  bool poll(float dt) {
    if (!in_->ready()) return false;
    const float raw = in_->read();
    if (!primed_) seat(raw);
    speed_ = dt > 0.0f ? fabsf(raw - prev_) / dt : 0.0f;
    prev_  = raw;
    if (dt != coeffDt_) { coeff_ = onePoleCoeff(dt, KNOB_TAU_S); coeffDt_ = dt; }
    lp_ += coeff_ * (raw - lp_);
    const float v = hyst_.process(lp_, spec_.hysteresis);
    if (catchPending_) {
      ctrl_.catchUp(target_ ? normalised(*target_) : v, v);
      catchPending_ = false;
    }
    engaged_ = ctrl_.process(v) && !live_;
    if (engaged_) live_ = true;
    if (live_ && target_) *target_ = value();
    return true;
  }

  // The bound parameter's value: takeover's main through the speed grid.
  float value() const { return applyKnob(spec_, ctrl_.main(), speed_); }

  bool     live() const     { return live_; }      // has written since arriving
  bool     engaged() const  { return engaged_; }   // started writing on THIS poll
  bool     bound() const    { return target_ != nullptr; }
  float    speed() const    { return speed_; }     // travel per second
  float    raw() const      { return prev_; }
  float    smoothed() const { return lp_; }        // after the one-pole
  float    filtered() const { return hyst_.value; } // after the hysteresis
  PotState state() const    { return ctrl_.state(); }

 private:
  void seat(float raw) {
    prev_ = raw;
    lp_ = hyst_.value = raw;
    primed_ = true;
  }
  float normalised(float x) const {
    const float span = spec_.hi - spec_.lo;
    return span != 0.0f ? clampf((x - spec_.lo) / span, 0.0f, 1.0f) : 0.0f;
  }

  const AnalogInput* in_;
  float*        target_ = nullptr;
  KnobSpec      spec_   = KnobSpec{0.0f, 1.0f, 0.0f, 0.0f};
  PotTakeover   ctrl_;
  Hysteresis    hyst_;
  float lp_ = 0.0f, prev_ = 0.0f, speed_ = 0.0f;
  float coeff_ = 0.0f, coeffDt_ = -1.0f;
  // Takeover is pending from construction: pickup from boot.
  bool  primed_ = false, live_ = false, engaged_ = false, catchPending_ = true;
};

// One character for a Knob's takeover state, for the PROFILE MUX lines:
// '-' = has not written since arriving (waiting for a move), then the
// takeover state: 'C' catching up, 'T' tracking.
inline char knobStateChar(const Knob& k) {
  if (!k.live()) return '-';
  return k.state() == POT_TRACKING ? 'T' : 'C';
}

// The position knob's spec: 0..1 of the source; a fast move snaps to 5%
// landmarks (20 detents), a slow move is continuous. The row-2 pots' spec
// (one per head, see KNOB_MUX_CH).
inline KnobSpec positionSpec() { return KnobSpec{0.0f, 1.0f, 0.05f, 0.0f}; }

// The duration knob's spec (0.25..60 s, continuous): knob1 in GLOBAL mode
// (PanelEditor::update).
inline KnobSpec durationSpec(float durMin = 0.25f, float durMax = 60.0f) {
  return KnobSpec{durMin, durMax, 0.0f, 0.0f};
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
//   - step i: knob1 = unassigned (reads, writes nothing -- each head's
//     position has its own row-2 pot), knob2 = that step's per-step drift.
//
// TAKEOVER everywhere: landing on any slot (global or a step) does NOT snap
// its value to the pot. Every arrival re-arms both knobs, so a knob writes
// the CURRENT slot's parameter only after it has physically moved (more than
// PotTakeover::CATCH_MOVE), and then catches up from the stored value
// without a jump (Beads' CATCHING_UP): touring never disturbs untouched
// values, and touching one never leaps it to the pot.
// Effective drift per step = max(per-step, global), clamped (foldDrift).
//
// The two knobs are Knobs injected with their inputs (the Pod's ADC pins on
// the device, fakes in tests); this class re-binds them to the slot's
// parameters every poll. The caller owns the Sequencer and the global
// duration/drift floats; this class owns the mode and per-step drift shadow
// and never touches hardware.
constexpr int PE_GLOBAL = 0;                 // slot 0 = global page

class PanelEditor {
 public:
  PanelEditor(const AnalogInput& in1, const AnalogInput& in2) : k1_(in1), k2_(in2) {}

  // Seat both knobs where their pots are now, so a stationary pot at boot
  // isn't a move.
  void prime() { k1_.prime(); k2_.prime(); }

  int  slot() const { return slot_; }        // 0 = global, 1..SS_STEPS = step
  bool inGlobal() const { return slot_ == PE_GLOBAL; }
  int  step() const { return slot_ - 1; }    // valid only when !inGlobal()
  float perStepDrift(int i) const { return perStepDrift_[i]; }
  // The two panel knobs (diagnostics: the PROFILE KNOB line).
  const Knob& knob1() const { return k1_; }
  const Knob& knob2() const { return k2_; }
  // Diagnostic: has each knob written on the CURRENT slot?
  bool k1Live() const { return k1_.live(); }
  bool k2Live() const { return k2_.live(); }
  // Diagnostic: last knob speed (travel per second) and its fast/slow
  // verdict, for tuning the potFast threshold from board data.
  float speed1() const { return k1_.speed(); }
  float speed2() const { return k2_.speed(); }
  bool  fast1()  const { return potFast(k1_.speed()); }
  bool  fast2()  const { return potFast(k2_.speed()); }

  // button1: GLOBAL -> step1 -> ... -> stepN -> GLOBAL, where N = activeSteps
  // (#149): the nav only visits ACTIVE steps, so shrinking the sequence shrinks
  // the tour. `activeSteps` is clamped into [1, SS_STEPS]; default SS_STEPS
  // preserves the old full-tour behavior. Re-arms takeover for the slot we land on.
  void advance(int activeSteps = SS_STEPS) {
    int n = ssClampSteps(activeSteps);
    // Slots in play: GLOBAL (0) + steps 1..n, so (n + 1) slots, wrapping.
    goTo((slot_ + 1) % (n + 1));
  }
  // button2: shortcut back to GLOBAL (re-arms its takeover).
  void toGlobal() { goTo(PE_GLOBAL); }

  // Keep the editor on a valid slot when the active step count shrinks (#149):
  // if we're parked on a step past the new count, jump back to GLOBAL so the
  // knobs never edit an inactive step. Call after the step count changes.
  void clampToActive(int activeSteps) {
    if (slot_ > ssClampSteps(activeSteps)) goTo(PE_GLOBAL);
  }

  // One control pass, `dt` seconds after the previous one: bind each knob to
  // the current slot's parameter, poll it (it reads its own input and writes
  // when live), then fold max(per-step, global) into seq.drift[].
  void update(Sequencer& seq, float* dur, float* gdrift, float dt,
              float driftMax = 0.03f, float durMin = 0.25f, float durMax = 60.0f,
              float gdriftMax = 0.03f) {
    // Per-knob specs. Drift: fast = driftMax/30, slow = 0.03% (0.0003) --
    // 0..driftMax(3%) is 100 fine detents of 0.03% each. Duration: continuous
    // both (a smooth sweep; grid TBD). Global drift: same as per-step drift.
    const KnobSpec DRIFT    { 0.0f, driftMax, driftMax / 30.0f, 0.0003f };
    const KnobSpec DURATION = durationSpec(durMin, durMax);
    const KnobSpec GDRIFT   { 0.0f, gdriftMax, gdriftMax / 30.0f, 0.0003f };

    if (slot_ == PE_GLOBAL) {
      k1_.bind(dur, DURATION);
      k2_.bind(gdrift, GDRIFT);
    } else {
      k1_.unbind();
      k2_.bind(&perStepDrift_[slot_ - 1], DRIFT);
    }
    k1_.poll(dt);
    k2_.poll(dt);

    for (int i = 0; i < SS_STEPS; i++)
      seq.drift[i] = foldDrift(perStepDrift_[i], *gdrift);
  }

 private:
  // Arriving on a slot re-arms both knobs' takeover: a knob writes the new
  // slot's parameter only after it moves (no snap to the pot).
  void goTo(int slot) {
    slot_ = slot;
    k1_.rearm();
    k2_.rearm();
  }

  int   slot_ = PE_GLOBAL;
  float perStepDrift_[SS_STEPS] = {0};
  Knob  k1_, k2_;
};

// --- The row-2 pots: one position knob per head (Fizzy #157) ----------------
// Eight pots behind the 4051 on A7, one per read head: panel knob i (0 =
// leftmost) sets head i's position. Each is an independent Knob bound to
// seq.position[i] with positionSpec(), reading its own MuxPin (mux_adc.h);
// nothing relates one pot to another. The pots are not wired to the 4051 in
// panel order; this table is the PCB netlist (RV3 RV2 RV1 RV4 RV6 RV8 RV7 RV5
// left to right, wipers on X2 X1 X0 X3 X5 X7 X6 X4), used once, when the
// firmware builds the eight MuxPins.
constexpr int MUX_KNOBS = 8;
static_assert(MUX_KNOBS == SS_STEPS, "one row-2 pot per head");
constexpr int KNOB_MUX_CH[MUX_KNOBS] = {2, 1, 0, 3, 5, 7, 6, 4};

// The row-3 pots: a second 4051 sharing the select lines, its common on A1.
// Scanned and conditioned like row 2 but bound to nothing yet (what they
// control is undecided); their raw reads are on the PROFILE MUX2 line. They
// use KNOB_MUX_CH on the assumption the row-3 board is the row-2 PCB; the
// MUX2 line confirms or refutes that (turn knob N, the Nth value sweeps).
constexpr int MUX_BANKS = 2;

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

// One debounce sample of the digital panel, applied: the encoder click
// (page), the encoder detent (`inc` = +-1 or 0, speed from the gap since the
// previous detent's own timestamp), and the two buttons' rising edges. The
// click is applied BEFORE the detent so a detent sampled in the same
// millisecond as a click lands on the page the player is looking at. The
// firmware calls this from the audio callback with libDaisy's edge flags,
// which are valid only on the call that sampled them. Returns the detents
// applied (0 or 1), for the profiler.
inline int applyDigitalControls(EncoderState& enc, Sequencer& seq, PanelEditor& panel,
                                bool click, int inc, bool button1, bool button2,
                                uint32_t ms, uint32_t& lastDetentMs) {
  if (click)   enc.page = nextPage(enc.page);
  if (button1) panel.advance(seq.activeSteps);
  if (button2) panel.toGlobal();
  if (inc == 0) return 0;
  bool fast = encoderFast(ms - lastDetentMs);
  lastDetentMs = ms;
  applyEncoder(enc, seq, panel, inc, fast);
  return 1;
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
