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

TEST_CASE("pickup: a freshly selected step is NOT snapped to the knob") {
  Sequencer& seq = fresh_seq();
  float p0 = seq.position[0];
  StepEditor ed;
  ed.prime(0.5f, 0.5f);
  // Knob sits at 0.5 but hasn't MOVED since prime -> step 0 keeps its value.
  ed.update(seq, 0.5f, 0.5f, 0.0f);
  REQUIRE(seq.position[0] == Approx(p0));   // unchanged: no pickup yet
}

TEST_CASE("pickup: the knob takes over only after it moves past threshold") {
  Sequencer& seq = fresh_seq();
  StepEditor ed;
  ed.prime(0.30f, 0.0f);
  // Tiny jitter below threshold: still no takeover.
  ed.update(seq, 0.31f, 0.0f, 0.0f, /*moveThresh=*/0.02f);
  float before = seq.position[0];
  // A deliberate move past threshold: knob1 takes over, position tracks it.
  ed.update(seq, 0.80f, 0.0f, 0.0f, /*moveThresh=*/0.02f);
  REQUIRE(seq.position[0] == Approx(0.80f));
  REQUIRE(seq.position[0] != Approx(before));
  // Once live, it keeps tracking even small moves.
  ed.update(seq, 0.805f, 0.0f, 0.0f);
  REQUIRE(seq.position[0] == Approx(0.805f));
}

TEST_CASE("step select wraps and re-arms pickup for the new step") {
  Sequencer& seq = fresh_seq();
  StepEditor ed;
  ed.prime(0.5f, 0.5f);
  REQUIRE(ed.selected() == 0);
  for (int i = 1; i < SS_STEPS; i++) { ed.advanceStep(); REQUIRE(ed.selected() == i); }
  ed.advanceStep();
  REQUIRE(ed.selected() == 0);   // wraps

  // On a new step, a stationary knob does NOT snap it: edit step 0, advance to
  // step 1, and confirm step 1 isn't pulled to the knob until it moves.
  ed.update(seq, 0.90f, 0.0f, 0.0f);   // (knob already live on 0 from prime move? no)
  // Move knob1 to take over step 0.
  ed.prime(0.10f, 0.5f);
  ed.update(seq, 0.90f, 0.5f, 0.0f);   // moved 0.10->0.90 : step 0 takes over
  REQUIRE(seq.position[0] == Approx(0.90f));
  float p1 = seq.position[1];
  ed.advanceStep();                    // now on step 1, pickup re-armed
  ed.update(seq, 0.90f, 0.5f, 0.0f);   // knob stationary at 0.90 -> step 1 unchanged
  REQUIRE(seq.position[1] == Approx(p1));
}

TEST_CASE("per-step drift is independent between steps; global adds to all") {
  Sequencer& seq = fresh_seq();
  StepEditor ed;
  ed.prime(0.0f, 0.0f);
  // Set step 0's per-step drift via knob2 (move it), leave others at 0.
  ed.update(seq, 0.0f, 0.80f, 0.0f);   // knob2 0->0.80 takes over: perStep = 0.25*0.8 = 0.2
  REQUIRE(ed.perStepDrift(0) == Approx(0.20f));
  REQUIRE(ed.perStepDrift(1) == Approx(0.0f));   // untouched
  // seq.drift[] = perStep + global(0) here.
  REQUIRE(seq.drift[0] == Approx(0.20f));
  REQUIRE(seq.drift[1] == Approx(0.0f));

  // Now raise global drift: it adds to EVERY step's effective drift, live,
  // without corrupting the per-step shadow (the double-add regression).
  ed.update(seq, 0.0f, 0.80f, 0.10f);
  REQUIRE(seq.drift[0] == Approx(0.30f));   // 0.20 + 0.10
  REQUIRE(seq.drift[1] == Approx(0.10f));   // 0.00 + 0.10
  REQUIRE(ed.perStepDrift(0) == Approx(0.20f));   // shadow UNCHANGED (key!)

  // Global change again -- must not accumulate on the shadow.
  ed.update(seq, 0.0f, 0.80f, 0.05f);
  REQUIRE(seq.drift[0] == Approx(0.25f));   // 0.20 + 0.05, NOT 0.20+0.10+0.05
  REQUIRE(ed.perStepDrift(0) == Approx(0.20f));
}

TEST_CASE("effective drift clamps at 1.0 when per-step + global overflow") {
  Sequencer& seq = fresh_seq();
  StepEditor ed;
  ed.prime(0.0f, 0.0f);
  ed.update(seq, 0.0f, 1.0f, 0.0f);        // perStep = 0.25 (max via knob)
  ed.update(seq, 0.0f, 1.0f, 0.9f);        // 0.25 + 0.9 = 1.15 -> clamp 1.0
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

TEST_CASE("smoothKnob: first read jumps, then eases") {
  float s = 0.0f;
  REQUIRE(smoothKnob(s, 0.7f, /*primed=*/false) == Approx(0.7f));   // jump
  REQUIRE(s == Approx(0.7f));
  float v = smoothKnob(s, 1.0f, /*primed=*/true, 0.5f);             // ease half
  REQUIRE(v == Approx(0.85f));
  REQUIRE(s == Approx(0.85f));
}

// --- page + LED color -------------------------------------------------------

TEST_CASE("nextPage cycles stretch -> fade -> duration -> drift -> stretch") {
  REQUIRE(nextPage(PAGE_STRETCH)  == PAGE_FADE);
  REQUIRE(nextPage(PAGE_FADE)     == PAGE_DURATION);
  REQUIRE(nextPage(PAGE_DURATION) == PAGE_DRIFT);
  REQUIRE(nextPage(PAGE_DRIFT)    == PAGE_STRETCH);   // wraps
}

TEST_CASE("pageColor: distinct hue per page, brightness folded in") {
  Rgb blue = pageColor(PAGE_STRETCH, 0.6f);
  REQUIRE(blue.b == Approx(0.6f)); REQUIRE(blue.r == Approx(0.0f));
  Rgb green = pageColor(PAGE_FADE, 0.6f);
  REQUIRE(green.g == Approx(0.6f)); REQUIRE(green.b == Approx(0.0f));
  Rgb yellow = pageColor(PAGE_DURATION, 0.6f);
  REQUIRE(yellow.r == Approx(0.6f)); REQUIRE(yellow.g == Approx(0.6f));
  Rgb purple = pageColor(PAGE_DRIFT, 0.6f);
  REQUIRE(purple.b == Approx(0.6f)); REQUIRE(purple.r > 0.0f); REQUIRE(purple.g == Approx(0.0f));
  // brightness actually scales
  REQUIRE(pageColor(PAGE_STRETCH, 0.15f).b == Approx(0.15f));
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
