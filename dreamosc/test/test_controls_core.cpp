// Unit tests for the platform-free control-surface logic (controls_core.h):
// knob PICKUP (soft takeover), step selection, and per-step + global drift
// folding. This logic used to live in dreamosc.cpp (un-host-compilable), where
// a double-add drift bug slipped through; extracting it here makes it testable.
//
// No CATCH_CONFIG_MAIN here -- test_stretch_core.cpp defines it; these link
// into the same binary. test_support.h provides the gTab/gWork/gSpec globals.

#include "catch_amalgamated.hpp"

#include <vector>

#include "test_support.h"
#include "controls_core.h"

namespace {

// A Sequencer to write into; StepEditor needs its position[]/drift[].
Sequencer& fresh_seq() {
  static std::vector<float> pool(SS_POOL_FLOATS);
  static Sequencer seq;
  static bool init = false;
  if (!init) { gTab.init(); init = true; }
  auto srcbuf = new std::vector<float>(48000, 0.0f);   // leaked; test-scope
  static Source src;
  src.data = srcbuf->data();
  src.len = srcbuf->size();
  seq.init(&src, 48000, pool.data());
  return seq;
}

}  // namespace

TEST_CASE("foldDrift: additive, clamped to [0,1]") {
  REQUIRE(foldDrift(0.0f, 0.0f) == Approx(0.0f));
  REQUIRE(foldDrift(0.1f, 0.2f) == Approx(0.3f));     // additive
  REQUIRE(foldDrift(0.9f, 0.5f) == Approx(1.0f));     // clamps high
  REQUIRE(foldDrift(-1.0f, 0.0f) == Approx(0.0f));    // clamps low
  REQUIRE(foldDrift(0.25f, 0.0f) == Approx(0.25f));   // global 0 = per-step only
  REQUIRE(foldDrift(0.0f, 0.15f) == Approx(0.15f));   // per-step 0 = global only
}

// Helpers: a PanelEditor starts in GLOBAL; advance() once lands on step 1
// (slot 1 -> step index 0). These tests exercise the step slots and the global
// slot with pickup. dur/gdrift are the caller-owned globals.

TEST_CASE("panel starts in GLOBAL; advance walks steps then returns to GLOBAL") {
  PanelEditor pe;
  REQUIRE(pe.inGlobal());
  REQUIRE(pe.slot() == PE_GLOBAL);
  for (int s = 1; s <= SS_STEPS; s++) { pe.advance(); REQUIRE(pe.slot() == s); }
  pe.advance();                          // after step 8 -> back to GLOBAL
  REQUIRE(pe.inGlobal());
}

TEST_CASE("advance(N) tours only the active steps then wraps to GLOBAL (#149)") {
  // With activeSteps = 3, button1 should walk GLOBAL -> 1 -> 2 -> 3 -> GLOBAL,
  // never visiting steps 4..8 (they're not in the sequence).
  PanelEditor pe;
  REQUIRE(pe.inGlobal());
  pe.advance(3); REQUIRE(pe.slot() == 1);
  pe.advance(3); REQUIRE(pe.slot() == 2);
  pe.advance(3); REQUIRE(pe.slot() == 3);
  pe.advance(3); REQUIRE(pe.inGlobal());     // wraps after the last ACTIVE step
  // activeSteps = 1: GLOBAL <-> step 1 only.
  pe.advance(1); REQUIRE(pe.slot() == 1);
  pe.advance(1); REQUIRE(pe.inGlobal());
}

TEST_CASE("clampToActive: shrinking the count off a parked step returns to GLOBAL (#149)") {
  PanelEditor pe;
  pe.advance(); pe.advance(); pe.advance();   // full tour: on step 3
  REQUIRE(pe.slot() == 3);
  pe.clampToActive(3);                        // still valid: stays put
  REQUIRE(pe.slot() == 3);
  pe.clampToActive(2);                        // step 3 now inactive: back to GLOBAL
  REQUIRE(pe.inGlobal());
  // On an active step, clamp is a no-op.
  pe.advance(2);                              // step 1
  pe.clampToActive(2);
  REQUIRE(pe.slot() == 1);
}

TEST_CASE("button2 (toGlobal) jumps back to GLOBAL from any step") {
  PanelEditor pe;
  pe.advance(); pe.advance(); pe.advance();   // on step 3
  REQUIRE(pe.slot() == 3);
  pe.toGlobal();
  REQUIRE(pe.inGlobal());
}

TEST_CASE("pickup: a freshly entered step is NOT snapped to the knob") {
  Sequencer& seq = fresh_seq();
  float dur = 1.0f, gd = 0.0f;
  PanelEditor pe;
  pe.prime(0.5f, 0.5f);
  pe.advance();                          // step 1 (index 0)
  float p0 = seq.position[0];
  pe.update(seq, &dur, &gd, 0.5f, 0.5f, 0.5f, 0.5f); // knob stationary since prime -> no snap
  REQUIRE(seq.position[0] == Approx(p0));
}

TEST_CASE("pickup: the knob takes over only after it moves past threshold") {
  Sequencer& seq = fresh_seq();
  float dur = 1.0f, gd = 0.0f;
  PanelEditor pe;
  pe.prime(0.30f, 0.0f);
  pe.advance();                          // step 1 (index 0)
  pe.update(seq, &dur, &gd, 0.31f, 0.0f, 0.31f, 0.0f);   // below threshold: no takeover
  float before = seq.position[0];
  pe.update(seq, &dur, &gd, 0.80f, 0.0f, 0.80f, 0.0f);   // moved past threshold: takes over
  REQUIRE(seq.position[0] == Approx(0.80f));
  REQUIRE(seq.position[0] != Approx(before));
  pe.update(seq, &dur, &gd, 0.805f, 0.0f, 0.805f, 0.0f);  // stays live for small moves
  REQUIRE(seq.position[0] == Approx(0.805f));
}

TEST_CASE("entering a new step re-arms pickup (stationary knob doesn't snap it)") {
  Sequencer& seq = fresh_seq();
  float dur = 1.0f, gd = 0.0f;
  PanelEditor pe;
  pe.prime(0.10f, 0.5f);
  pe.advance();                             // step 1
  // After a slot switch the FIRST update re-anchors the move reference to the
  // current pot (models the poll where the button was pressed, knob still).
  pe.update(seq, &dur, &gd, 0.10f, 0.5f, 0.10f, 0.5f);   // stationary: re-anchor
  pe.update(seq, &dur, &gd, 0.90f, 0.5f, 0.90f, 0.5f);   // now MOVE: step 1 takes over
  REQUIRE(seq.position[0] == Approx(0.90f));
  float p1 = seq.position[1];
  pe.advance();                             // step 2, pickup re-armed
  pe.update(seq, &dur, &gd, 0.90f, 0.5f, 0.90f, 0.5f);   // knob stationary at 0.90: step 2 unchanged
  REQUIRE(seq.position[1] == Approx(p1));
}

TEST_CASE("pickup engages on a SLOW sweep (reference must not chase the pot)") {
  // The real hardware bug: a pot turned slowly moves only a little each 1 kHz
  // poll. If the move-detection reference updates to the current raw EVERY pass,
  // it chases the pot and the per-pass delta never exceeds threshold, so pickup
  // never engages and the control feels dead. The reference must stay anchored
  // (at prime / slot entry) until pickup engages, so slow movement ACCUMULATES.
  Sequencer& seq = fresh_seq();
  float dur = 1.0f, gd = 0.0f;
  PanelEditor pe;
  pe.prime(0.30f, 0.0f);   // start anchored at 0.30
  // Sweep knob1 slowly from 0.30 up, 0.005 per poll (well below the 0.02
  // threshold per step) -- a realistic slow turn sampled at poll rate.
  bool engaged = false;
  for (int i = 1; i <= 40 && !engaged; i++) {
    float r = 0.30f + 0.005f * i;      // 0.305, 0.310, ... crosses 0.02 by step 4+
    pe.update(seq, &dur, &gd, r, 0.0f, r, 0.0f);
    if (dur != Approx(1.0f)) engaged = true;   // duration moved => pickup engaged
  }
  REQUIRE(engaged);   // a slow sweep of ~0.2 total MUST engage pickup
}

TEST_CASE("pickup re-anchors on slot change: globals work after returning from a step") {
  // Regression: r1Last_ (the move-detection reference) is shared across slots.
  // Without re-anchoring on a slot switch, it carried over stale -- a step left
  // the knob at 1.0, then GLOBAL compared against 1.0 and pickup never
  // re-engaged, so global duration/drift silently stopped working after
  // returning from a step. This is the exact reported symptom.
  Sequencer& seq = fresh_seq();
  float dur = 1.0f, gd = 0.0f;
  PanelEditor pe;
  pe.prime(0.0f, 0.0f);

  // In a STEP, drive knob1 all the way to 1.0 -- this leaves the shared move
  // reference (r1Last_) at 1.0. (Stationary re-anchor pass, then move.)
  pe.advance();                                       // step 1
  pe.update(seq, &dur, &gd, 0.0f, 0.0f, 0.0f, 0.0f);  // re-anchor at 0
  pe.update(seq, &dur, &gd, 1.0f, 0.0f, 1.0f, 0.0f);  // move 0->1.0: pos -> 1.0
  REQUIRE(seq.position[0] == Approx(1.0f));

  // Return to GLOBAL with the knob still parked at 1.0.
  pe.toGlobal();
  // Stationary knob at 1.0: no takeover yet (correct -- pickup, not snapped).
  float durBefore = dur;
  pe.update(seq, &dur, &gd, 1.0f, 0.0f, 1.0f, 0.0f);
  REQUIRE(dur == Approx(durBefore));
  // Move the knob down to 0.5. WITHOUT the slot-change re-anchor, r1Last_ was
  // still 1.0 from the step, |1.0-1.0|=0 that first stationary pass updated it,
  // and the interaction left global pickup unreliable. With the fix, this
  // deliberate move engages and duration follows.
  pe.update(seq, &dur, &gd, 0.5f, 0.0f, 0.5f, 0.0f);
  REQUIRE(dur == Approx(0.25f + (60.0f - 0.25f) * 0.5f));
}

TEST_CASE("GLOBAL mode: knobs drive duration and global drift, with pickup") {
  Sequencer& seq = fresh_seq();
  float dur = 1.0f, gd = 0.0f;
  PanelEditor pe;
  pe.prime(0.0f, 0.0f);                     // in GLOBAL
  // Stationary knobs: no change.
  pe.update(seq, &dur, &gd, 0.0f, 0.0f, 0.0f, 0.0f);
  REQUIRE(dur == Approx(1.0f));
  REQUIRE(gd == Approx(0.0f));
  // Move knob1 -> duration = durMin + (durMax-durMin)*k1 (defaults 0.25..60).
  pe.update(seq, &dur, &gd, 0.5f, 0.0f, 0.5f, 0.0f);
  REQUIRE(dur == Approx(0.25f + (60.0f - 0.25f) * 0.5f));
  // Move knob2 -> global drift = 0.25 * k2.
  pe.update(seq, &dur, &gd, 0.5f, 0.4f, 0.5f, 0.4f);
  REQUIRE(gd == Approx(0.25f * 0.4f));
}

TEST_CASE("per-step drift is independent between steps; global adds to all") {
  Sequencer& seq = fresh_seq();
  float dur = 1.0f, gd = 0.0f;
  PanelEditor pe;
  pe.prime(0.0f, 0.0f);
  pe.advance();                             // step 1 (index 0)
  pe.update(seq, &dur, &gd, 0.0f, 0.0f, 0.0f, 0.0f);     // re-anchor at 0
  pe.update(seq, &dur, &gd, 0.0f, 0.80f, 0.0f, 0.80f);   // knob2 0->0.80: perStep[0] = 0.25*0.8 = 0.2
  REQUIRE(pe.perStepDrift(0) == Approx(0.20f));
  REQUIRE(pe.perStepDrift(1) == Approx(0.0f));   // untouched
  REQUIRE(seq.drift[0] == Approx(0.20f));        // + global(0)
  REQUIRE(seq.drift[1] == Approx(0.0f));

  // Raise global (back in a step; global still folds into ALL steps). Passing
  // gd via pointer -- set it, then update to re-fold.
  gd = 0.10f;
  pe.update(seq, &dur, &gd, 0.0f, 0.80f, 0.0f, 0.80f);
  REQUIRE(seq.drift[0] == Approx(0.30f));   // 0.20 + 0.10
  REQUIRE(seq.drift[1] == Approx(0.10f));   // 0.00 + 0.10
  REQUIRE(pe.perStepDrift(0) == Approx(0.20f));   // shadow UNCHANGED (no double-add)

  gd = 0.05f;
  pe.update(seq, &dur, &gd, 0.0f, 0.80f, 0.0f, 0.80f);
  REQUIRE(seq.drift[0] == Approx(0.25f));   // 0.20 + 0.05, not accumulated
  REQUIRE(pe.perStepDrift(0) == Approx(0.20f));
}

TEST_CASE("effective drift clamps at 1.0 when per-step + global overflow") {
  Sequencer& seq = fresh_seq();
  float dur = 1.0f, gd = 0.0f;
  PanelEditor pe;
  pe.prime(0.0f, 0.0f);
  pe.advance();                             // step 1
  pe.update(seq, &dur, &gd, 0.0f, 0.0f, 0.0f, 0.0f);    // re-anchor at 0
  pe.update(seq, &dur, &gd, 0.0f, 1.0f, 0.0f, 1.0f);    // perStep[0] = 0.25 (max via knob)
  gd = 0.9f;
  pe.update(seq, &dur, &gd, 0.0f, 1.0f, 0.0f, 1.0f);    // 0.25 + 0.9 = 1.15 -> clamp 1.0
  REQUIRE(seq.drift[0] == Approx(1.0f));
}

// --- encoder stepping helpers -----------------------------------------------

TEST_CASE("encoderFast: gap <= threshold is fast") {
  REQUIRE(encoderFast(10));           // quick spin
  REQUIRE(encoderFast(40));           // exactly at default threshold
  REQUIRE_FALSE(encoderFast(41));     // deliberate click
  REQUIRE(encoderFast(100, 200));     // custom threshold
}

TEST_CASE("stepAdditive: value moves by perDetent*inc, clamped") {
  REQUIRE(stepAdditive(0.10f, +1, 0.04f, 0.0f, 0.5f) == Approx(0.14f));
  REQUIRE(stepAdditive(0.10f, -1, 0.04f, 0.0f, 0.5f) == Approx(0.06f));
  REQUIRE(stepAdditive(0.02f, -1, 0.04f, 0.0f, 0.5f) == Approx(0.0f));   // clamp lo
  REQUIRE(stepAdditive(0.49f, +1, 0.04f, 0.0f, 0.5f) == Approx(0.5f));   // clamp hi
}

TEST_CASE("stepRatio: value scales by ratio^inc, clamped; inc<0 divides") {
  REQUIRE(stepRatio(1.0f, +1, 2.0f, 0.25f, 60.0f) == Approx(2.0f));
  REQUIRE(stepRatio(2.0f, -1, 2.0f, 0.25f, 60.0f) == Approx(1.0f));
  REQUIRE(stepRatio(40.0f, +1, 2.0f, 0.25f, 60.0f) == Approx(60.0f));    // clamp hi
  REQUIRE(stepRatio(0.3f, -1, 2.0f, 0.25f, 60.0f) == Approx(0.25f));     // clamp lo
}

TEST_CASE("stepIndex: index moves by stops*inc, clamped to [0,count-1]") {
  REQUIRE(stepIndex(20, +1, 3, 56) == 23);
  REQUIRE(stepIndex(20, -1, 1, 56) == 19);
  REQUIRE(stepIndex(0,  -1, 3, 56) == 0);      // clamp lo
  REQUIRE(stepIndex(54, +1, 3, 56) == 55);     // clamp hi (not 57)
}

TEST_CASE("stepCount: +/-1 per detent, clamped to [lo,hi] (#149)") {
  REQUIRE(stepCount(4, +1, 1, SS_STEPS) == 5);
  REQUIRE(stepCount(4, -1, 1, SS_STEPS) == 3);
  REQUIRE(stepCount(1, -1, 1, SS_STEPS) == 1);          // clamp lo (never 0)
  REQUIRE(stepCount(SS_STEPS, +1, 1, SS_STEPS) == SS_STEPS);  // clamp hi
}

TEST_CASE("smoothKnob: first read jumps, then eases") {
  float s = 0.0f;
  REQUIRE(smoothKnob(s, 0.7f, /*primed=*/false) == Approx(0.7f));   // jump
  REQUIRE(s == Approx(0.7f));
  float v = smoothKnob(s, 1.0f, /*primed=*/true, 0.5f);             // ease half
  REQUIRE(v == Approx(0.85f));
  REQUIRE(s == Approx(0.85f));
}

// --- page + LED color -------------------------------------------------------

TEST_CASE("nextPage cycles stretch -> steps -> fade -> frame -> stretch") {
  REQUIRE(nextPage(PAGE_STRETCH) == PAGE_STEPS);
  REQUIRE(nextPage(PAGE_STEPS)   == PAGE_FADE);
  REQUIRE(nextPage(PAGE_FADE)    == PAGE_FRAME);
  REQUIRE(nextPage(PAGE_FRAME)   == PAGE_STRETCH);   // wraps (4 pages)
}

TEST_CASE("pageColor: RoYG in click order, shared palette, brightness scales") {
  // led2 draws the same ROYGBIVW hues as led1, indexed by page position, so the
  // click order (stretch/steps/fade/frame) IS the color order red/orange/yellow/
  // green. Orange and yellow must be DISTINGUISHABLE (the bug: hand-rolled colors
  // made two pages read alike).
  Rgb red    = pageColor(PAGE_STRETCH, 0.6f);   // page 0 = red
  Rgb orange = pageColor(PAGE_STEPS,   0.6f);   // page 1 = orange
  Rgb yellow = pageColor(PAGE_FADE,    0.6f);   // page 2 = yellow
  Rgb green  = pageColor(PAGE_FRAME,   0.6f);   // page 3 = green
  REQUIRE(red.r == Approx(0.6f)); REQUIRE(red.g == Approx(0.0f)); REQUIRE(red.b == Approx(0.0f));
  REQUIRE(green.g == Approx(0.6f)); REQUIRE(green.r == Approx(0.0f)); REQUIRE(green.b == Approx(0.0f));
  // orange = red-dominant with partial green; yellow = equal red+green. The
  // green channel is what separates them, and it must differ clearly.
  REQUIRE(orange.g < yellow.g);                 // orange has less green than yellow
  REQUIRE(yellow.r == Approx(yellow.g));        // yellow is equal R+G
  REQUIRE(orange.g < orange.r);                 // orange is red-dominant
  REQUIRE(fabsf(orange.g - yellow.g) > 0.1f);   // clearly distinguishable, not alike
  // brightness actually scales the hue
  REQUIRE(pageColor(PAGE_STRETCH, 0.15f).r == Approx(0.15f));
  // led1 and led2 share the palette: page hue == step hue at equal brightness.
  Rgb s0 = stepColor(0, 0.6f);
  REQUIRE(red.r == Approx(s0.r)); REQUIRE(red.g == Approx(s0.g)); REQUIRE(red.b == Approx(s0.b));
}

TEST_CASE("stepBrightness: orange intensity tracks the active step count (#149)") {
  // 1 step = the dim floor, SS_STEPS = full brightness, monotonic between.
  REQUIRE(stepBrightness(1) == Approx(0.15f));            // floor
  REQUIRE(stepBrightness(SS_STEPS) == Approx(1.0f));      // full
  REQUIRE(stepBrightness(4) > stepBrightness(2));         // more steps = brighter
  REQUIRE(stepBrightness(7) < stepBrightness(SS_STEPS));
  // clamped outside [1, SS_STEPS]
  REQUIRE(stepBrightness(0) == Approx(0.15f));
  REQUIRE(stepBrightness(99) == Approx(1.0f));
  // paired with pageColor, the STEPS LED (page 1) is orange at that brightness:
  // red-dominant with partial green, full red at max count.
  Rgb c = pageColor(PAGE_STEPS, stepBrightness(SS_STEPS));
  REQUIRE(c.r == Approx(1.0f)); REQUIRE(c.g > 0.0f); REQUIRE(c.g < c.r); REQUIRE(c.b == Approx(0.0f));
}

TEST_CASE("stretchBrightness: red intensity tracks the stretch detent index") {
  // low index = dim floor, top index = full, monotonic between.
  REQUIRE(stretchBrightness(0, 56) == Approx(0.15f));     // floor
  REQUIRE(stretchBrightness(55, 56) == Approx(1.0f));     // full (last of 56)
  REQUIRE(stretchBrightness(40, 56) > stretchBrightness(10, 56));
  // clamped, and a degenerate 1-stop table reads full.
  REQUIRE(stretchBrightness(-3, 56) == Approx(0.15f));
  REQUIRE(stretchBrightness(999, 56) == Approx(1.0f));
  REQUIRE(stretchBrightness(0, 1) == Approx(1.0f));
  // paired with pageColor, the STRETCH LED is pure red at that brightness
  Rgb c = pageColor(PAGE_STRETCH, stretchBrightness(55, 56));
  REQUIRE(c.r == Approx(1.0f)); REQUIRE(c.g == Approx(0.0f)); REQUIRE(c.b == Approx(0.0f));
}

TEST_CASE("levelBrightness: normalized level -> [floor, 1], clamped") {
  REQUIRE(levelBrightness(0.0f) == Approx(0.15f));    // floor
  REQUIRE(levelBrightness(1.0f) == Approx(1.0f));     // full
  REQUIRE(levelBrightness(0.5f) == Approx(0.15f + 0.85f * 0.5f));
  REQUIRE(levelBrightness(-1.0f) == Approx(0.15f));   // clamp lo
  REQUIRE(levelBrightness(2.0f) == Approx(1.0f));     // clamp hi
}

TEST_CASE("every page's led2 brightness tracks its own encoded level") {
  // fade: 0 (butt-joint) = floor, 0.5 (max overlap) = full, monotonic.
  REQUIRE(fadeBrightness(0.0f) == Approx(0.15f));
  REQUIRE(fadeBrightness(0.5f) == Approx(1.0f));
  REQUIRE(fadeBrightness(0.25f) > fadeBrightness(0.1f));
  // frame: brightness follows the KNOB (detent index), CW brightens. idx 0
  // (fully CCW, largest window) = dim floor; max idx (fully CW, smallest) = full.
  REQUIRE(frameBrightness(0, 5) == Approx(0.15f));    // idx 0 = CCW end = dim floor
  REQUIRE(frameBrightness(4, 5) == Approx(1.0f));     // idx 4 = CW end = full
  REQUIRE(frameBrightness(3, 5) > frameBrightness(1, 5));   // clockwise = brighter
  // Each page's hue is correct AND its brightness is proportional, not on/off.
  // fade is page 2 = YELLOW: brighter fade lifts both channels together.
  Rgb fadeMid = pageColor(PAGE_FADE, fadeBrightness(0.25f));
  Rgb fadeLo  = pageColor(PAGE_FADE, fadeBrightness(0.0f));
  REQUIRE(fadeMid.r == Approx(fadeMid.g));            // yellow = equal R+G
  REQUIRE(fadeMid.r > fadeLo.r);                      // brighter at more fade
  // frame is page 3 = GREEN: brightness rides the green channel only. The CW end
  // (max idx) is the brightest green.
  Rgb frameHi = pageColor(PAGE_FRAME, frameBrightness(4, 5));
  REQUIRE(frameHi.g == Approx(1.0f)); REQUIRE(frameHi.r == Approx(0.0f)); REQUIRE(frameHi.b == Approx(0.0f));
}

TEST_CASE("stepColor: 8 distinct colors, index clamped") {
  Rgb prev = stepColor(0);
  for (int i = 1; i < SS_STEPS; i++) {
    Rgb c = stepColor(i);
    bool differs = (c.r != prev.r) || (c.g != prev.g) || (c.b != prev.b);
    REQUIRE(differs);   // each step visibly different from the last
    prev = c;
  }
  // out-of-range clamps rather than reading garbage
  Rgb lo = stepColor(-5), hi = stepColor(99);
  Rgb first = stepColor(0), last = stepColor(SS_STEPS - 1);
  REQUIRE(lo.r == Approx(first.r)); REQUIRE(lo.g == Approx(first.g)); REQUIRE(lo.b == Approx(first.b));
  REQUIRE(hi.r == Approx(last.r));  REQUIRE(hi.g == Approx(last.g));  REQUIRE(hi.b == Approx(last.b));
}

// --- speed-adaptive knob quantization ---------------------------------------

TEST_CASE("potFast: per-poll raw delta above threshold is fast") {
  REQUIRE(potFast(0.05f));            // a quick spin
  REQUIRE_FALSE(potFast(0.005f));     // a slow dial
  REQUIRE(potFast(0.011f));           // just over the 0.01 default
  REQUIRE_FALSE(potFast(0.01f));      // exactly at threshold is NOT fast
}

TEST_CASE("snapTo: grid rounding, clamp; grid 0 = continuous") {
  REQUIRE(snapTo(0.42f, 0.05f, 0.0f, 1.0f) == Approx(0.40f));   // nearest 0.05
  REQUIRE(snapTo(0.43f, 0.05f, 0.0f, 1.0f) == Approx(0.45f));
  REQUIRE(snapTo(0.42f, 0.0f,  0.0f, 1.0f) == Approx(0.42f));   // continuous
  REQUIRE(snapTo(1.20f, 0.05f, 0.0f, 1.0f) == Approx(1.00f));   // clamp hi
  REQUIRE(snapTo(-0.1f, 0.05f, 0.0f, 1.0f) == Approx(0.00f));   // clamp lo
}

TEST_CASE("applyKnob: fast move snaps to coarse grid, slow to fine") {
  // Position: fast = 5% grid, slow = continuous.
  KnobSpec pos{0.0f, 1.0f, 0.05f, 0.0f};
  // 0.423 moved FAST -> nearest 5% = 0.40; 0.437 -> 0.45.
  REQUIRE(applyKnob(pos, 0.423f, /*speed=*/0.05f) == Approx(0.40f));
  REQUIRE(applyKnob(pos, 0.437f, /*speed=*/0.05f) == Approx(0.45f));
  // Same read moved SLOW -> continuous, lands exactly.
  REQUIRE(applyKnob(pos, 0.423f, /*speed=*/0.002f) == Approx(0.423f));

  // Drift: range 0..0.25, fast = 0.25/30 grid, slow = 0.001 (0.1%).
  KnobSpec drift{0.0f, 0.25f, 0.25f / 30.0f, 0.001f};
  float fastD = applyKnob(drift, 0.5f, /*speed=*/0.05f);   // 0.5*0.25=0.125
  // 0.125 / (0.25/30) = 15.0 -> exact detent
  REQUIRE(fastD == Approx(0.125f));
  float slowD = applyKnob(drift, 0.5137f, /*speed=*/0.001f);  // 0.128425 -> 0.1%
  REQUIRE(slowD == Approx(0.128f));   // snapped to 0.001 grid
}

TEST_CASE("applyKnob: one behavior, ranges differ per spec") {
  // Same call shape scales into different ranges -- the point of one knob object.
  KnobSpec dur{0.25f, 60.0f, 0.0f, 0.0f};   // continuous
  REQUIRE(applyKnob(dur, 0.0f, 0.0f) == Approx(0.25f));
  REQUIRE(applyKnob(dur, 1.0f, 0.0f) == Approx(60.0f));
  REQUIRE(applyKnob(dur, 0.5f, 0.0f) == Approx(0.25f + (60.0f - 0.25f) * 0.5f));
}
