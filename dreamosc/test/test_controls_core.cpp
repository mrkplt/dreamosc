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

// A Sequencer to write into; PanelEditor needs its position[]/drift[].
Sequencer& fresh_seq() {
  static std::vector<float> pool(SS_POOL_FLOATS);
  static std::vector<float> srcbuf(48000, 0.0f);
  static Sequencer seq;
  static bool init = false;
  if (!init) { gTab.init(); init = true; }
  static MemSource src;
  src.data = srcbuf.data();
  src.len = srcbuf.size();
  // init() deliberately leaves the public controls (stretch, duration, fade,
  // frameSize, activeSteps) alone -- they are the player's -- so a test that
  // lowered the step count would leak into the next. Reconstruct first.
  seq.~Sequencer();
  new (&seq) Sequencer();
  seq.init(&src, 48000, pool.data());
  return seq;
}

// The control poll interval these tests drive at: 1 ms (any dt works; the
// Knob's filters are in time, and a test below proves rate independence).
constexpr float DT = 1e-3f;

// A PanelEditor wired to two fake pots: set the pots, then poll.
struct Rig {
  FakeInput   in1, in2;
  PanelEditor pe{in1, in2};
  void prime(float r1, float r2) { in1.v = r1; in2.v = r2; pe.prime(); }
  void update(Sequencer& seq, float& dur, float& gd, float r1, float r2, float driftMax = 0.03f) {
    in1.v = r1; in2.v = r2;
    pe.update(seq, &dur, &gd, DT, driftMax);
  }
};

// Hold both knobs at (r1, r2) long enough for the one-pole (tau 4.9 ms) to
// settle: 2 s is ~400 time constants. The first poll of a move is FAST
// (coarse grid), the rest are still (slow grid): settled values sit on the
// SLOW grid, KNOB_HYSTERESIS short of the pot on the side it came from.
void settle(Rig& rig, Sequencer& seq, float& dur, float& gd, float r1, float r2,
            float driftMax = 0.03f) {
  for (int i = 0; i < 2000; i++) rig.update(seq, dur, gd, r1, r2, driftMax);
}

// Turn the knobs from where they are to (r1, r2) like a hand would: 0.001
// of travel per poll (1 travel/s, a slow turn), then settle. Takeover
// (catch-up) only moves on movement, so this is how a test takes a knob
// to an end stop to bring a parameter and its pot together.
void turn(Rig& rig, Sequencer& seq, float& dur, float& gd, float r1, float r2,
          float driftMax = 0.03f) {
  float a = rig.in1.v, b = rig.in2.v;
  while (fabsf(a - r1) > 1e-6f || fabsf(b - r2) > 1e-6f) {
    a += clampf(r1 - a, -0.001f, 0.001f);
    b += clampf(r2 - b, -0.001f, 0.001f);
    rig.update(seq, dur, gd, a, b, driftMax);
  }
  settle(rig, seq, dur, gd, r1, r2, driftMax);
}

}  // namespace

TEST_CASE("foldDrift: global is the floor (max), clamped to [0,1]") {
  REQUIRE(foldDrift(0.0f, 0.0f) == Approx(0.0f));
  REQUIRE(foldDrift(0.1f, 0.2f) == Approx(0.2f));     // global floor wins
  REQUIRE(foldDrift(0.2f, 0.1f) == Approx(0.2f));     // per-step exceeds floor -> per-step
  REQUIRE(foldDrift(0.9f, 0.5f) == Approx(0.9f));     // per-step wins, no overflow
  REQUIRE(foldDrift(1.5f, 0.0f) == Approx(1.0f));     // clamps high
  REQUIRE(foldDrift(-1.0f, 0.0f) == Approx(0.0f));    // clamps low
  REQUIRE(foldDrift(0.25f, 0.0f) == Approx(0.25f));   // global 0 = per-step only
  REQUIRE(foldDrift(0.0f, 0.15f) == Approx(0.15f));   // per-step 0 = global floor
}

// Helpers: a PanelEditor starts in GLOBAL; advance() once lands on step 1
// (slot 1 -> step index 0). These tests exercise the step slots and the global
// slot with pickup. dur/gdrift are the caller-owned globals.

TEST_CASE("panel starts in GLOBAL; advance walks steps then returns to GLOBAL") {
  Rig rig; PanelEditor& pe = rig.pe;
  REQUIRE(pe.inGlobal());
  REQUIRE(pe.slot() == PE_GLOBAL);
  for (int s = 1; s <= SS_STEPS; s++) { pe.advance(); REQUIRE(pe.slot() == s); }
  pe.advance();                          // after step 8 -> back to GLOBAL
  REQUIRE(pe.inGlobal());
}

TEST_CASE("advance(N) tours only the active steps then wraps to GLOBAL (#149)") {
  // With activeSteps = 3, button1 should walk GLOBAL -> 1 -> 2 -> 3 -> GLOBAL,
  // never visiting steps 4..8 (they're not in the sequence).
  Rig rig; PanelEditor& pe = rig.pe;
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
  Rig rig; PanelEditor& pe = rig.pe;
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
  Rig rig; PanelEditor& pe = rig.pe;
  pe.advance(); pe.advance(); pe.advance();   // on step 3
  REQUIRE(pe.slot() == 3);
  pe.toGlobal();
  REQUIRE(pe.inGlobal());
}

TEST_CASE("takeover: a freshly entered step is NOT snapped to the knob") {
  Sequencer& seq = fresh_seq();
  float dur = 1.0f, gd = 0.0f;
  Rig rig;
  rig.prime(0.5f, 0.5f);
  rig.pe.advance();                      // step 1 (index 0)
  rig.update(seq, dur, gd, 0.5f, 0.5f);  // knobs stationary since prime -> no snap
  REQUIRE(rig.pe.perStepDrift(0) == 0.0f);
}

TEST_CASE("takeover: nothing is written until the knob moves; then it moves from the stored value, no jump") {
  Sequencer& seq = fresh_seq();
  float dur = 1.0f, gd = 0.0f;
  Rig rig;
  rig.prime(0.0f, 0.30f);
  rig.pe.advance();                        // step 1 (index 0): knob2 = its drift (stored 0)
  settle(rig, seq, dur, gd, 0.0f, 0.302f); // inside the hysteresis: nothing
  REQUIRE(rig.pe.perStepDrift(0) == 0.0f);
  REQUIRE_FALSE(rig.pe.k2Live());
  turn(rig, seq, dur, gd, 0.0f, 0.32f);    // a small real move: it writes ...
  REQUIRE(rig.pe.k2Live());
  REQUIRE(rig.pe.perStepDrift(0) > 0.0f);
  // ... moving RELATIVELY from 0 (about 1.4x the 0.02 move), nowhere near
  // the pot's 0.32 x 0.03 = 0.0096: no jump.
  REQUIRE(rig.pe.perStepDrift(0) < 0.002f);
  turn(rig, seq, dur, gd, 0.0f, 1.0f);     // at the end stop the two meet ...
  REQUIRE(rig.pe.perStepDrift(0) == Approx(0.03f).margin(0.0003f));
  turn(rig, seq, dur, gd, 0.0f, 0.70f);    // ... and from then on it tracks
  REQUIRE(rig.pe.perStepDrift(0) == Approx(0.03f * 0.70f).margin(0.0003f));
}

TEST_CASE("entering a new step re-arms takeover (stationary knob doesn't snap it)") {
  Sequencer& seq = fresh_seq();
  float dur = 1.0f, gd = 0.0f;
  Rig rig;
  rig.prime(0.0f, 0.10f);
  rig.pe.advance();                             // step 1
  turn(rig, seq, dur, gd, 0.0f, 0.90f);         // MOVE: step 1 changes
  REQUIRE(rig.pe.perStepDrift(0) > 0.02f);
  rig.pe.advance();                             // step 2, takeover re-armed
  settle(rig, seq, dur, gd, 0.0f, 0.90f);       // knob stationary at 0.90: step 2 unchanged
  REQUIRE(rig.pe.perStepDrift(1) == 0.0f);
}

TEST_CASE("takeover engages on a SLOW sweep (the reference must not chase the pot)") {
  // The real hardware bug: a pot turned slowly moves only a little each
  // poll. If the move reference followed the pot EVERY pass, the per-pass
  // delta would never cross the threshold and the control would feel dead.
  // PotTakeover's reference only advances when the threshold is crossed,
  // so slow movement ACCUMULATES.
  Sequencer& seq = fresh_seq();
  float dur = 1.0f, gd = 0.0f;
  Rig rig;
  rig.prime(0.30f, 0.0f);
  bool engaged = false;
  for (int i = 1; i <= 400 && !engaged; i++) {
    float r = 0.30f + 0.0002f * i;     // 0.2 travel/s: far below the catch-up step per poll
    rig.update(seq, dur, gd, r, 0.0f);
    if (dur != Approx(1.0f)) engaged = true;
  }
  REQUIRE(engaged);
}

TEST_CASE("takeover re-arms on slot change: globals work after returning from a step") {
  // Regression: takeover must be re-armed on every slot switch. Without it,
  // a step left the knob at 1.0, then GLOBAL compared against 1.0 and never
  // re-engaged, so global drift silently stopped working after returning
  // from a step. This is the exact reported symptom.
  Sequencer& seq = fresh_seq();
  float dur = 1.0f, gd = 0.0f;
  Rig rig;
  rig.prime(0.0f, 0.0f);

  rig.pe.advance();                             // step 1
  turn(rig, seq, dur, gd, 0.0f, 1.0f);          // drive knob2 to the top
  REQUIRE(rig.pe.perStepDrift(0) == Approx(0.03f).margin(0.0003f));

  rig.pe.toGlobal();                            // knob parked at 1.0, global drift 0
  settle(rig, seq, dur, gd, 0.0f, 1.0f);
  REQUIRE(gd == 0.0f);                          // stationary: untouched
  // Turning DOWN from 1.0 with the value already at the bottom: the two
  // meet at the bottom, so it holds at 0 (Beads' skew clamps at 0.1x) ...
  turn(rig, seq, dur, gd, 0.0f, 0.5f);
  REQUIRE(gd == Approx(0.0f).margin(0.0003f));
  // ... and turning back UP moves it, meeting the pot at the top.
  turn(rig, seq, dur, gd, 0.0f, 1.0f);
  REQUIRE(gd == Approx(0.03f).margin(0.0003f));
  turn(rig, seq, dur, gd, 0.0f, 0.5f);
  REQUIRE(gd == Approx(0.03f * 0.5f).margin(0.0003f));
}

TEST_CASE("step mode: knob1 is unassigned -- turning it writes nothing") {
  Sequencer& seq = fresh_seq();
  float before[SS_STEPS];
  for (int i = 0; i < SS_STEPS; i++) before[i] = seq.position[i];
  float dur = 1.0f, gd = 0.0f;
  Rig rig;
  rig.prime(0.0f, 0.0f);
  rig.pe.advance();                             // step 1
  turn(rig, seq, dur, gd, 1.0f, 0.0f);          // knob1 all the way up
  REQUIRE(rig.pe.knob1().live());               // it engaged (it still reads) ...
  REQUIRE_FALSE(rig.pe.knob1().bound());        // ... but is bound to nothing
  REQUIRE(dur == 1.0f);
  for (int i = 0; i < SS_STEPS; i++) REQUIRE(seq.position[i] == before[i]);
}

TEST_CASE("GLOBAL mode: knobs drive duration and global drift, with takeover") {
  Sequencer& seq = fresh_seq();
  float dur = 1.0f, gd = 0.0f;
  Rig rig;
  rig.prime(0.0f, 0.0f);                    // in GLOBAL
  settle(rig, seq, dur, gd, 0.0f, 0.0f);    // stationary knobs: no change
  REQUIRE(dur == 1.0f);
  REQUIRE(gd == 0.0f);
  // Duration is stored at 1 s (0.0126 of its 0.25..60 range) with the pot at
  // 0: turning up moves it relatively, then up to the stop they meet ...
  turn(rig, seq, dur, gd, 1.0f, 0.0f);
  REQUIRE(rig.pe.k1Live());
  REQUIRE(dur == Approx(60.0f).margin(59.75f * 0.002f));
  // ... and from there it tracks: duration = durMin + (durMax-durMin)*k1.
  turn(rig, seq, dur, gd, 0.5f, 0.0f);
  REQUIRE(dur == Approx(0.25f + 59.75f * 0.5f).margin(59.75f * 0.002f));
  // Global drift is stored at 0 with its pot at 0: already together, so it
  // tracks from the first move. gdriftMax * k2, on the slow grid (0.0003).
  turn(rig, seq, dur, gd, 0.5f, 0.4f);
  REQUIRE(gd == Approx(0.03f * 0.4f).margin(0.0003f));
}

TEST_CASE("per-step drift is independent between steps; global is the floor for all") {
  Sequencer& seq = fresh_seq();
  float dur = 1.0f, gd = 0.0f;
  Rig rig; PanelEditor& pe = rig.pe;
  rig.prime(0.0f, 0.0f);
  pe.advance();                             // step 1 (index 0)
  // This case tests the FOLD/independence math (foldDrift = max), not the drift
  // max -- pass an explicit driftMax=0.25 so the designed 0.2 / global values are
  // reachable and the arithmetic stays legible. Stored 0 with the pot at 0:
  // together, so it tracks.
  // Margin = one slow-grid step (0.0003) plus the hysteresis (the settled
  // read sits KNOB_HYSTERESIS short of the pot), in drift units.
  const float G = 0.0003f + 0.25f * KNOB_HYSTERESIS;
  turn(rig, seq, dur, gd, 0.0f, 0.80f, 0.25f);   // knob2 0->0.80: perStep[0] = 0.25*0.8 = 0.2
  REQUIRE(pe.perStepDrift(0) == Approx(0.20f).margin(G));
  REQUIRE(pe.perStepDrift(1) == Approx(0.0f));   // untouched
  REQUIRE(seq.drift[0] == Approx(0.20f).margin(G));   // max(0.20, global 0)
  REQUIRE(seq.drift[1] == Approx(0.0f));

  // Raise global (back in a step; global is the floor for ALL steps). Passing
  // gd via pointer -- set it, then update to re-fold. Floor 0.10 is BELOW step0's
  // own 0.20, so step0 keeps 0.20; step1 (own 0) rises to the 0.10 floor.
  gd = 0.10f;
  rig.update(seq, dur, gd, 0.0f, 0.80f, 0.25f);
  REQUIRE(seq.drift[0] == Approx(0.20f).margin(G));   // max(0.20, 0.10) -> own value wins
  REQUIRE(seq.drift[1] == Approx(0.10f));   // max(0.00, 0.10) -> floor
  REQUIRE(pe.perStepDrift(0) == Approx(0.20f).margin(G));   // shadow UNCHANGED (no double-add)

  // Floor now ABOVE step0's own 0.20 -> step0 lifts to the 0.25 floor; the
  // per-step shadow still does not move (fold is non-destructive).
  gd = 0.25f;
  rig.update(seq, dur, gd, 0.0f, 0.80f, 0.25f);
  REQUIRE(seq.drift[0] == Approx(0.25f));   // max(0.20, 0.25) -> floor wins (exact gd)
  REQUIRE(pe.perStepDrift(0) == Approx(0.20f).margin(G));
}

TEST_CASE("effective drift with max-fold stays in range; a floor above per-step wins") {
  Sequencer& seq = fresh_seq();
  float dur = 1.0f, gd = 0.0f;
  Rig rig; PanelEditor& pe = rig.pe;
  rig.prime(0.0f, 0.0f);
  pe.advance();                             // step 1
  // max-fold cannot overflow from two in-range inputs (unlike the old additive
  // fold). Drive per-step to its 0.25 max, then a floor ABOVE it: the floor wins,
  // and the result never exceeds 1.0. Explicit driftMax=0.25 for legible values.
  turn(rig, seq, dur, gd, 0.0f, 1.0f, 0.25f);      // perStep[0] = 0.25 (max via knob)
  REQUIRE(seq.drift[0] == Approx(0.25f).margin(0.0003f + 0.25f * KNOB_HYSTERESIS));
  gd = 0.9f;                                // floor above per-step
  rig.update(seq, dur, gd, 0.0f, 1.0f, 0.25f);
  REQUIRE(seq.drift[0] == Approx(0.9f));    // max(0.25, 0.9) -> floor, still <= 1.0
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

TEST_CASE("clampf / clampi / ssClampSteps: the shared clamps") {
  REQUIRE(clampf(0.5f, 0.0f, 1.0f) == 0.5f);
  REQUIRE(clampf(-1.0f, 0.0f, 1.0f) == 0.0f);
  REQUIRE(clampf(2.0f, 0.0f, 1.0f) == 1.0f);
  REQUIRE(clampi(5, 0, 9) == 5);
  REQUIRE(clampi(-3, 0, 9) == 0);
  REQUIRE(clampi(12, 0, 9) == 9);
  // Active step count: [1, SS_STEPS], the one range setSteps / the panel nav /
  // the steps LED all share.
  REQUIRE(ssClampSteps(0) == 1);
  REQUIRE(ssClampSteps(-5) == 1);
  REQUIRE(ssClampSteps(3) == 3);
  REQUIRE(ssClampSteps(SS_STEPS) == SS_STEPS);
  REQUIRE(ssClampSteps(SS_STEPS + 1) == SS_STEPS);
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

// --- page + LED color -------------------------------------------------------

TEST_CASE("nextPage cycles stretch -> steps -> fade -> frame -> stretch") {
  // Four encoder pages (RoYG); frame wraps back to stretch.
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

// --- encoder: detent tables, state, one detent's effect ---------------------
// This logic lived in dreamosc.cpp (un-host-compilable) until the alpha4
// cleanup; these are its first tests.

TEST_CASE("STRETCH_STOPS: 52 strictly increasing stops from 1x to 10000x, boot at 50x") {
  // (The firmware's old comment said 56; 10+5+6+5+8+7+4+7 = 52. Pinned here so
  // the count is checked, not asserted in a comment.)
  REQUIRE(STRETCH_NSTOPS == 52);
  REQUIRE(STRETCH_STOPS[0] == 1.0f);
  REQUIRE(STRETCH_STOPS[STRETCH_NSTOPS - 1] == 10000.0f);
  for (int i = 1; i < STRETCH_NSTOPS; i++) REQUIRE(STRETCH_STOPS[i] > STRETCH_STOPS[i - 1]);
  REQUIRE(stretchStop(STRETCH_DEFAULT_IDX) == 50.0f);
  // stretchStop clamps into the table rather than reading past it.
  REQUIRE(stretchStop(-1) == 1.0f);
  REQUIRE(stretchStop(999) == 10000.0f);
}

TEST_CASE("FRAME_DEFAULT_IDX is derived from the core's SS_W_DEFAULT") {
  // The firmware can no longer boot at a different window than the host suite:
  // the index IS the core's size index of SS_W_DEFAULT.
  REQUIRE(ssSizeW(FRAME_DEFAULT_IDX) == SS_W_DEFAULT);
  REQUIRE(FRAME_DEFAULT_IDX == 2);            // 16384, 8192, [4096]
  EncoderState e;
  REQUIRE(e.page == PAGE_STRETCH);
  REQUIRE(e.stretchIdx == STRETCH_DEFAULT_IDX);
  REQUIRE(e.frameIdx == FRAME_DEFAULT_IDX);
}

TEST_CASE("encoderSync: boot pushes stretch 50x and window SS_W_DEFAULT into the Sequencer") {
  Sequencer& seq = fresh_seq();
  seq.stretch = 1.0f; seq.setFrame(SS_W);
  EncoderState e;
  encoderSync(e, seq);
  REQUIRE(seq.stretch == 50.0f);
  REQUIRE(seq.frameSize == SS_W_DEFAULT);
}

TEST_CASE("applyEncoder: stretch page steps the detent index (3/detent fast, 1 slow), clamped") {
  Sequencer& seq = fresh_seq();
  Rig rig; PanelEditor& pe = rig.pe;
  EncoderState e;                                  // page = stretch, idx 20 (50x)
  encoderSync(e, seq);
  applyEncoder(e, seq, pe, +1, /*fast=*/false);
  REQUIRE(e.stretchIdx == 21); REQUIRE(seq.stretch == 60.0f);
  applyEncoder(e, seq, pe, +1, /*fast=*/true);
  REQUIRE(e.stretchIdx == 24); REQUIRE(seq.stretch == 90.0f);
  applyEncoder(e, seq, pe, -1, /*fast=*/true);
  REQUIRE(e.stretchIdx == 21); REQUIRE(seq.stretch == 60.0f);
  for (int i = 0; i < 100; i++) applyEncoder(e, seq, pe, +1, true);
  REQUIRE(e.stretchIdx == STRETCH_NSTOPS - 1); REQUIRE(seq.stretch == 10000.0f);   // clamp hi
  for (int i = 0; i < 100; i++) applyEncoder(e, seq, pe, -1, true);
  REQUIRE(e.stretchIdx == 0); REQUIRE(seq.stretch == 1.0f);                        // clamp lo
  applyEncoder(e, seq, pe, 0, true);                // a zero increment is a no-op
  REQUIRE(e.stretchIdx == 0);
}

TEST_CASE("applyEncoder: frame page walks the size index one per detent; CW shrinks") {
  Sequencer& seq = fresh_seq();
  Rig rig; PanelEditor& pe = rig.pe;
  EncoderState e; e.page = PAGE_FRAME;
  encoderSync(e, seq);
  REQUIRE(seq.frameSize == 4096);
  applyEncoder(e, seq, pe, +1, true);              // clockwise = smaller (fast is ignored)
  REQUIRE(e.frameIdx == 3); REQUIRE(seq.frameSize == 2048);
  applyEncoder(e, seq, pe, -1, false);
  applyEncoder(e, seq, pe, -1, false);
  REQUIRE(e.frameIdx == 1); REQUIRE(seq.frameSize == 8192);
  for (int i = 0; i < 10; i++) applyEncoder(e, seq, pe, -1, false);
  REQUIRE(e.frameIdx == 0); REQUIRE(seq.frameSize == SS_W);         // clamp at the max
  for (int i = 0; i < 10; i++) applyEncoder(e, seq, pe, +1, false);
  REQUIRE(e.frameIdx == SS_NSIZES - 1); REQUIRE(seq.frameSize == SS_W_MIN);   // clamp at the min
}

TEST_CASE("applyEncoder: steps page changes the count and keeps the panel on a valid slot (#149)") {
  Sequencer& seq = fresh_seq();
  Rig rig; PanelEditor& pe = rig.pe;
  EncoderState e; e.page = PAGE_STEPS;
  REQUIRE(seq.activeSteps == SS_STEPS);
  pe.advance(); pe.advance(); pe.advance();        // parked on step 3
  applyEncoder(e, seq, pe, -1, true);              // 8 -> 7 (fast is ignored)
  REQUIRE(seq.activeSteps == 7);
  REQUIRE(pe.slot() == 3);                         // still active: stays
  for (int i = 0; i < 5; i++) applyEncoder(e, seq, pe, -1, false);   // -> 2
  REQUIRE(seq.activeSteps == 2);
  REQUIRE(pe.inGlobal());                          // step 3 went inactive: back to GLOBAL
  for (int i = 0; i < 10; i++) applyEncoder(e, seq, pe, -1, false);
  REQUIRE(seq.activeSteps == 1);                   // never 0
  for (int i = 0; i < 20; i++) applyEncoder(e, seq, pe, +1, false);
  REQUIRE(seq.activeSteps == SS_STEPS);            // never past 8
}

TEST_CASE("applyEncoder: fade page is additive, 0.04 fast / 0.005 slow, clamped to [0, 0.5]") {
  Sequencer& seq = fresh_seq();
  Rig rig; PanelEditor& pe = rig.pe;
  EncoderState e; e.page = PAGE_FADE;
  seq.fade = 0.0f;
  applyEncoder(e, seq, pe, +1, false); REQUIRE(seq.fade == Approx(0.005f));
  applyEncoder(e, seq, pe, +1, true);  REQUIRE(seq.fade == Approx(0.045f));
  applyEncoder(e, seq, pe, -1, true);  REQUIRE(seq.fade == Approx(0.005f));
  applyEncoder(e, seq, pe, -1, true);  REQUIRE(seq.fade == Approx(0.0f));       // clamp lo
  for (int i = 0; i < 20; i++) applyEncoder(e, seq, pe, +1, true);
  REQUIRE(seq.fade == Approx(0.5f));                                             // clamp hi
}

TEST_CASE("pageBrightness: each page reports its own level through the per-page helper") {
  Sequencer& seq = fresh_seq();
  EncoderState e;
  encoderSync(e, seq);
  seq.fade = 0.25f; seq.setSteps(4);
  e.page = PAGE_STRETCH; REQUIRE(pageBrightness(e, seq) == Approx(stretchBrightness(e.stretchIdx, STRETCH_NSTOPS)));
  e.page = PAGE_FADE;    REQUIRE(pageBrightness(e, seq) == Approx(fadeBrightness(0.25f)));
  e.page = PAGE_FRAME;   REQUIRE(pageBrightness(e, seq) == Approx(frameBrightness(e.frameIdx, SS_NSIZES)));
  e.page = PAGE_STEPS;   REQUIRE(pageBrightness(e, seq) == Approx(stepBrightness(4)));
  // The frame page's brightness tracks the KNOB: CW (larger idx) is brighter.
  e.page = PAGE_FRAME;
  float dim = pageBrightness(e, seq);
  e.frameIdx = SS_NSIZES - 1;
  REQUIRE(pageBrightness(e, seq) > dim);
  // An out-of-range page falls back to the dim floor rather than garbage.
  e.page = (EncoderPage)PAGE_COUNT;
  REQUIRE(pageBrightness(e, seq) == Approx(levelBrightness(0.0f)));
}

// --- speed-adaptive knob quantization ---------------------------------------

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
  REQUIRE(applyKnob(pos, 0.423f, /*speed=*/50.0f) == Approx(0.40f));
  REQUIRE(applyKnob(pos, 0.437f, /*speed=*/50.0f) == Approx(0.45f));
  // Same read moved SLOW -> continuous, lands exactly.
  REQUIRE(applyKnob(pos, 0.423f, /*speed=*/2.0f) == Approx(0.423f));

  // Drift: range 0..0.25, fast = 0.25/30 grid, slow = 0.001 (0.1%).
  KnobSpec drift{0.0f, 0.25f, 0.25f / 30.0f, 0.001f};
  float fastD = applyKnob(drift, 0.5f, /*speed=*/50.0f);   // 0.5*0.25=0.125
  // 0.125 / (0.25/30) = 15.0 -> exact detent
  REQUIRE(fastD == Approx(0.125f));
  float slowD = applyKnob(drift, 0.5137f, /*speed=*/1.0f);  // 0.128425 -> 0.1%
  REQUIRE(slowD == Approx(0.128f));   // snapped to 0.001 grid
}

TEST_CASE("applyKnob: one behavior, ranges differ per spec") {
  // Same call shape scales into different ranges -- the point of one knob object.
  KnobSpec dur{0.25f, 60.0f, 0.0f, 0.0f};   // continuous
  REQUIRE(applyKnob(dur, 0.0f, 0.0f) == Approx(0.25f));
  REQUIRE(applyKnob(dur, 1.0f, 0.0f) == Approx(60.0f));
  REQUIRE(applyKnob(dur, 0.5f, 0.0f) == Approx(0.25f + (60.0f - 0.25f) * 0.5f));
}

// --- one debounce sample of the digital panel, applied (audio-callback path) --

TEST_CASE("applyDigitalControls: detent speed comes from the gap to the previous detent") {
  Sequencer& seq = fresh_seq();
  Rig rig; PanelEditor& pe = rig.pe;
  EncoderState e;                                  // stretch page, idx 20 (50x)
  encoderSync(e, seq);
  uint32_t lastMs = 0;
  // 1000 ms after "never": slow (+1 stop); then 5 ms later: fast (+3); 5 ms
  // later again: fast (+3).
  REQUIRE(applyDigitalControls(e, seq, pe, false, +1, false, false, 1000, lastMs) == 1);
  REQUIRE(applyDigitalControls(e, seq, pe, false, +1, false, false, 1005, lastMs) == 1);
  REQUIRE(applyDigitalControls(e, seq, pe, false, +1, false, false, 1010, lastMs) == 1);
  REQUIRE(e.stretchIdx == 20 + 1 + 3 + 3);
  REQUIRE(lastMs == 1010);
  // A sample with nothing on it is a no-op and does not disturb the timing.
  REQUIRE(applyDigitalControls(e, seq, pe, false, 0, false, false, 1011, lastMs) == 0);
  REQUIRE(e.stretchIdx == 27);
  REQUIRE(lastMs == 1010);
}

TEST_CASE("applyDigitalControls: a click in the same sample as a detent moves the page first") {
  Sequencer& seq = fresh_seq();
  Rig rig; PanelEditor& pe = rig.pe;
  EncoderState e;
  encoderSync(e, seq);
  uint32_t lastMs = 0;
  applyDigitalControls(e, seq, pe, /*click=*/true, -1, false, false, 500, lastMs);
  REQUIRE(e.page == PAGE_STEPS);                   // stretch -> steps
  REQUIRE(seq.activeSteps == SS_STEPS - 1);        // the detent hit the steps page
  REQUIRE(e.stretchIdx == STRETCH_DEFAULT_IDX);    // not the stretch page
}

TEST_CASE("applyDigitalControls: buttons advance / return to GLOBAL, one edge per sample") {
  Sequencer& seq = fresh_seq();
  Rig rig; PanelEditor& pe = rig.pe;
  EncoderState e;
  uint32_t lastMs = 0;
  for (int i = 0; i < 3; i++) applyDigitalControls(e, seq, pe, false, 0, true, false, i, lastMs);
  REQUIRE(pe.slot() == 3);                         // three advances: GLOBAL -> step 3
  applyDigitalControls(e, seq, pe, false, 0, false, true, 4, lastMs);
  REQUIRE(pe.inGlobal());
  applyDigitalControls(e, seq, pe, false, 0, true, false, 5, lastMs);
  REQUIRE(pe.slot() == 1);
  // Both in one sample: button1 then button2, so GLOBAL wins.
  applyDigitalControls(e, seq, pe, false, 0, true, true, 6, lastMs);
  REQUIRE(pe.inGlobal());
}

// --- Pot conditioning ported from Beads --------------------------------------

TEST_CASE("onePoleCoeff: Beads' 0.05 per 4 kHz block, as a time constant") {
  REQUIRE(onePoleCoeff(0.25e-3f, KNOB_TAU_S) == Approx(0.05f).epsilon(1e-4));
  // The same filter at 1 kHz: four Beads blocks' worth, 1 - 0.95^4.
  REQUIRE(onePoleCoeff(1e-3f, KNOB_TAU_S) == Approx(1.0f - powf(0.95f, 4.0f)).epsilon(1e-4));
  REQUIRE(onePoleCoeff(0.0f, KNOB_TAU_S) == 0.0f);
}

TEST_CASE("Hysteresis: holds inside the threshold, follows at the threshold outside it") {
  Hysteresis h;
  h.value = 0.5f;
  REQUIRE(h.process(0.5009f, 0.001f) == 0.5f);
  REQUIRE(h.process(0.4991f, 0.001f) == 0.5f);
  REQUIRE(h.process(0.5030f, 0.001f) == Approx(0.5020f));   // follows, threshold behind
  REQUIRE(h.process(0.5035f, 0.001f) == Approx(0.5025f));   // continuously, not in steps
  REQUIRE(h.process(0.5015f, 0.001f) == Approx(0.5025f));   // back inside: holds
  REQUIRE(h.process(0.4000f, 0.001f) == Approx(0.4010f));
  REQUIRE(h.process(0.7f, 0.0f) == 0.7f);                   // threshold 0: pass-through
}

TEST_CASE("Hysteresis: noise up to 2x the threshold peak-to-peak never moves a settled output") {
  const float thr = KNOB_HYSTERESIS, span = 2.0f * thr * 0.999f;
  for (float dir : {+1.0f, -1.0f}) {
    for (float rest = 0.1f; rest < 0.9f; rest += 0.0137f) {
      Hysteresis h;
      h.value = rest - dir * 0.05f;
      const float settled = h.process(rest, thr);   // arrive from afar
      for (int i = 0; i < 100; i++)
        REQUIRE(h.process((i & 1) ? rest - dir * span : rest, thr) == settled);
    }
  }
}

TEST_CASE("PotTakeover: TRACKING follows the pot") {
  PotTakeover c;
  c.process(0.3f);
  REQUIRE(c.main() == 0.3f);
  c.process(0.7f);
  REQUIRE(c.main() == 0.7f);
  REQUIRE(c.state() == POT_TRACKING);
}

TEST_CASE("PotTakeover: CATCHING_UP moves relatively and meets the pot at the end of travel") {
  PotTakeover c;
  c.catchUp(0.2f, 0.7f);                      // value at 0.2, pot at 0.7
  REQUIRE(c.main() == Approx(0.2f));
  REQUIRE_FALSE(c.process(0.703f));           // under CATCH_MOVE: nothing
  REQUIRE(c.main() == Approx(0.2f));
  float prevMain = c.main();
  for (float p = 0.7f; p <= 1.0f; p += 0.001f) {
    c.process(p);
    REQUIRE(c.main() >= prevMain);            // monotonic with the pot ...
    REQUIRE(c.main() - prevMain < 0.03f);     // ... and never a jump
    prevMain = c.main();
  }
  REQUIRE(c.state() == POT_TRACKING);         // met at the top
  REQUIRE(c.main() == Approx(1.0f).margin(PotTakeover::CATCH_MOVE));
  // The skew made the 0.3 of pot travel cover the 0.8 of value travel.
}

TEST_CASE("PotTakeover: turning toward a value already at the stop holds it there") {
  PotTakeover c;
  c.catchUp(0.0f, 1.0f);                      // value at the bottom, pot at the top
  for (float p = 1.0f; p >= 0.5f; p -= 0.001f) c.process(p);
  REQUIRE(c.main() == 0.0f);
  REQUIRE(c.state() == POT_CATCHING_UP);
}

TEST_CASE("PotTakeover: small moves accumulate (the reference does not chase the pot)") {
  PotTakeover c;
  c.catchUp(0.5f, 0.3f);
  bool moved = false;
  for (int i = 1; i <= 100 && !moved; i++) moved = c.process(0.3f + 0.0001f * i);   // 1e-4 per call
  REQUIRE(moved);
}

// --- Knob: conditioning + takeover + speed, at any rate ----------------------
// A Knob reads an injected AnalogInput (a FakeInput here) and writes the
// float it is bound to.

TEST_CASE("potFast: speed is travel per second; 10/s is the threshold") {
  REQUIRE(potFast(50.0f));            // a quick spin
  REQUIRE_FALSE(potFast(5.0f));       // a slow dial
  REQUIRE(potFast(10.1f));
  REQUIRE_FALSE(potFast(10.0f));      // exactly at threshold is NOT fast
}

TEST_CASE("Knob: speed is the same physical turn rate at any poll rate") {
  for (float dt : {0.25e-3f, 0.667e-3f, 1e-3f, 2e-3f}) {
    FakeInput in; Knob k(in);
    in.v = 0.2f; k.poll(dt);
    in.v = 0.2f + 20.0f * dt; k.poll(dt);     // 20 travel/s
    REQUIRE(k.speed() == Approx(20.0f).epsilon(1e-3));
    in.v += 2.0f * dt; k.poll(dt);            // 2 travel/s
    REQUIRE(k.speed() == Approx(2.0f).epsilon(1e-2));
  }
}

TEST_CASE("Knob: the same gesture conditions to the same values at any poll rate") {
  // A pot swept 0.2 -> 0.8 over 100 ms, then held. Sampled at fixed times,
  // the conditioned read and the written parameter agree across rates:
  // nothing in the Knob is per call.
  auto pot = [](float t) { return t < 0.1f ? 0.2f + 6.0f * t : 0.8f; };
  auto run = [&](float dt, float tAt, float& filt, float& out) {
    FakeInput in; float target = 0.2f;        // stored where the pot starts: tracks at once
    Knob k(in, &target, positionSpec());
    in.v = pot(0.0f); k.prime();
    for (float t = dt; t <= tAt + 1e-6f; t += dt) { in.v = pot(t); k.poll(dt); }
    filt = k.filtered(); out = target;
  };
  for (float tAt : {0.05f, 0.12f, 0.3f}) {
    float f0, o0; run(1e-3f, tAt, f0, o0);
    for (float dt : {0.25e-3f, 0.5e-3f, 0.667e-3f}) {
      float f, o; run(dt, tAt, f, o);
      // Tolerance: the ramp is sampled at different instants (one 1 ms poll
      // of a 6/s ramp is 0.006) and the one-pole lags it by tau x 6/s = 0.03
      // at every rate; mid-ramp samples differ by up to one poll's lag.
      REQUIRE(f == Approx(f0).margin(0.008f));
      REQUIRE(o == Approx(o0).margin(0.008f));
    }
  }
}

TEST_CASE("Knob: nothing is written until the pot moves; a move writes from the stored value") {
  FakeInput in; float out = 0.5f;
  Knob k(in, &out, durationSpec());           // stored 0.5 s = 0.0042 of range
  in.v = 0.50f;
  for (int i = 0; i < 100; i++) k.poll(DT);   // primes; at rest
  in.v = 0.503f;
  for (int i = 0; i < 100; i++) k.poll(DT);   // inside the hysteresis + catch-up step
  REQUIRE_FALSE(k.live());
  REQUIRE(out == 0.5f);                       // nothing written
  bool engagedOnce = false;
  for (int i = 0; i < 100; i++) {
    in.v = 0.503f + 0.0002f * i; k.poll(DT);
    if (k.engaged()) { REQUIRE_FALSE(engagedOnce); engagedOnce = true; }
  }
  REQUIRE(engagedOnce);                       // engaged on exactly one poll
  REQUIRE(k.live());
  // Moved up from 0.5 s by the ~0.02 turn times Beads' skew (about 2x here:
  // 0.996 of value travel left over 0.5 of pot travel), so about 0.04 of
  // the 59.75 s range: ~3 s, nowhere near the pot's ~31 s.
  REQUIRE(out > 0.5f);
  REQUIRE(out < 5.0f);
}

TEST_CASE("Knob: catches up without a jump and then tracks") {
  FakeInput in; float pos = 0.2f;
  Knob k(in, &pos, positionSpec());
  in.v = 0.7f; k.poll(DT);
  float prev = pos;
  for (float p = 0.7f; p <= 1.0f; p += 0.0005f) {
    in.v = p; k.poll(DT);
    REQUIRE(fabsf(pos - prev) < 0.03f);      // never a jump
    prev = pos;
  }
  for (int i = 0; i < 200; i++) k.poll(DT);
  // Within catch-up's step of the top: Beads' catch-up advances in steps
  // of CATCH_MOVE, so it can stop just short of the stop ...
  REQUIRE(pos == Approx(1.0f).margin(2.0f * PotTakeover::CATCH_MOVE + KNOB_HYSTERESIS));
  // ... and closes the gap as the pot comes back down (the skew narrows it
  // toward the bottom), then tracks.
  for (float p = 1.0f; p >= 0.3f; p -= 0.0005f) { in.v = p; k.poll(DT); }
  in.v = 0.3f;                                // (the float loop ends a hair above)
  for (int i = 0; i < 200; i++) k.poll(DT);
  REQUIRE(k.state() == POT_TRACKING);
  REQUIRE(pos == Approx(0.3f).margin(KNOB_HYSTERESIS + 1e-4f));   // tracking the pot
}

TEST_CASE("Knob: a fast move snaps to the coarse grid while it moves") {
  FakeInput in; float pos = 0.3f;
  Knob k(in, &pos, positionSpec());
  in.v = 0.3f; k.poll(DT);
  in.v = 0.334f; k.poll(DT);                  // 34 travel/s: fast
  REQUIRE(k.speed() > POT_FAST_SPEED);
  if (k.live()) REQUIRE(fmodf(pos + 1e-4f, 0.05f) < 2e-4f);   // on the 5% grid
  for (int i = 0; i < 200; i++) k.poll(DT);   // still: continuous
  REQUIRE(k.speed() == 0.0f);
  REQUIRE(pos == Approx(0.333f).margin(0.002f));
}

TEST_CASE("Knob: rearm re-reads the stored value; a stationary pot then writes nothing") {
  FakeInput in; float a = 0.0f, b = 0.9f;
  Knob k(in, &a, positionSpec());
  in.v = 0.0f; k.poll(DT);
  for (float p = 0.0f; p <= 0.5f; p += 0.001f) { in.v = p; k.poll(DT); }
  for (int i = 0; i < 200; i++) k.poll(DT);
  REQUIRE(k.live());
  REQUIRE(a == Approx(0.5f).margin(0.003f));
  k.bind(&b, positionSpec());
  k.rearm();
  for (int i = 0; i < 200; i++) k.poll(DT);   // stationary at the new parameter
  REQUIRE_FALSE(k.live());
  REQUIRE(b == 0.9f);
  REQUIRE(a == Approx(0.5f).margin(0.003f));  // the old one is no longer written
  for (float p = 0.5f; p <= 0.6f; p += 0.001f) { in.v = p; k.poll(DT); }
  REQUIRE(k.live());
  REQUIRE(b > 0.9f);                          // catching up from 0.9, upward
}

TEST_CASE("Knob: unbound, it reads and tracks but writes nothing; bind() does not re-arm") {
  FakeInput in; float target = -1.0f;
  Knob k(in);
  in.v = 0.0f; k.poll(DT);
  for (float p = 0.0f; p <= 0.5f; p += 0.001f) { in.v = p; k.poll(DT); }
  for (int i = 0; i < 200; i++) k.poll(DT);
  REQUIRE(k.live());
  REQUIRE(k.state() == POT_TRACKING);
  REQUIRE(target == -1.0f);
  k.bind(&target, positionSpec());            // still live and tracking: writes the pot next poll
  k.poll(DT);
  REQUIRE(target == Approx(0.5f).margin(0.002f));
  k.unbind();
  in.v = 0.6f; for (int i = 0; i < 200; i++) k.poll(DT);
  REQUIRE(target == Approx(0.5f).margin(0.002f));
}

TEST_CASE("Knob: an input that is not ready is never read, so its first real read is not a move") {
  FakeInput in; float pos = 0.25f;
  Knob k(in, &pos, positionSpec());
  in.rdy = false;
  in.v = 0.0f;  REQUIRE_FALSE(k.poll(DT));    // pre-scan zeros: ignored entirely
  in.v = 0.5f;  REQUIRE_FALSE(k.poll(DT));
  in.rdy = true;
  in.v = 0.9f;
  for (int n = 0; n < 100; n++) REQUIRE(k.poll(DT));   // the pot's real resting position
  REQUIRE_FALSE(k.live());
  REQUIRE(pos == 0.25f);
}

TEST_CASE("Knob: a resting pot's flutter stops changing a written value once its peaks are seen") {
  // The bench measured ~0.001 per-poll flutter on a resting pot at 16x
  // oversampling; the mux now runs 4x (noise x2), so test +-0.002, square
  // (the worst case for a one-pole), at the block rate and at 1 kHz.
  // stmlib's hysteresis sits exactly one threshold behind the input it last
  // followed, so the first noise peaks on that side nudge it (by at most the
  // one-pole's ripple, ~1e-4 here); after that, noise within 2x the
  // threshold never moves it again.
  for (float dt : {32.0f / 48000.0f, 1e-3f}) {
    for (KnobSpec spec : {positionSpec(), durationSpec()}) {
      FakeInput in; float out = spec.lo;
      Knob k(in, &out, spec);
      in.v = 0.0f; k.poll(dt);
      for (float p = 0.0f; p <= 0.613f; p += 0.0005f) { in.v = p; k.poll(dt); }
      in.v = 0.613f; for (int i = 0; i < 1000; i++) k.poll(dt);
      REQUIRE(k.live());
      const float atRest = out;
      for (int i = 0; i < 100; i++) { in.v = (i & 1) ? 0.615f : 0.611f; k.poll(dt); }
      const float settled = out;
      // Measured 2.0e-4 of range (the one-pole's ripple on a +-0.002
      // square at these rates is 1.4..2e-4); bound at 3e-4.
      REQUIRE(fabsf(settled - atRest) <= (spec.hi - spec.lo) * 3e-4f);
      for (int i = 0; i < 3000; i++) {
        in.v = (i & 1) ? 0.615f : 0.611f; k.poll(dt);
        REQUIRE(out == settled);
      }
    }
  }
}

TEST_CASE("Knob: after a move the written value settles in tens of ms (no long creep)") {
  // The one-pole (tau 4.9 ms) settles to the OLED's resolution (0.0005)
  // within ~30 ms and below the core's 0.002 refresh threshold sooner; the
  // hysteresis then follows its asymptotic tail down to float resolution
  // (~11 tau, ~55 ms), in steps far too small to refresh or redraw. The old
  // 0.02-per-ms smoother crept for ~250 ms (finding F8).
  FakeInput in; float pos = 0.1f;
  Knob k(in, &pos, positionSpec());
  const float dt = 32.0f / 48000.0f;
  in.v = 0.1f; k.poll(dt);
  for (float p = 0.1f; p < 0.4f; p += 0.002f) { in.v = p; k.poll(dt); }   // into tracking
  in.v = 0.4f;
  int lastChange = 0; float prev = pos;
  for (int i = 1; i < 2000; i++) { k.poll(dt); if (pos != prev) lastChange = i; prev = pos; }
  REQUIRE((float)lastChange * dt < 0.1f);
  // Re-run, sampling at 40 ms: within the OLED's resolution of the final.
  FakeInput in2; float pos2 = 0.1f;
  Knob k2(in2, &pos2, positionSpec());
  in2.v = 0.1f; k2.poll(dt);
  for (float p = 0.1f; p < 0.4f; p += 0.002f) { in2.v = p; k2.poll(dt); }
  in2.v = 0.4f;
  for (int i = 0; i < (int)(0.04f / dt); i++) k2.poll(dt);
  REQUIRE(fabsf(pos2 - pos) < 0.0005f);
  REQUIRE(pos == Approx(0.4f).margin(KNOB_HYSTERESIS + 1e-4f));
}

TEST_CASE("knobStateChar: '-' until it writes, then the takeover state") {
  FakeInput in; float pos = 0.2f;
  Knob k(in, &pos, positionSpec());
  in.v = 0.7f; k.poll(DT);
  REQUIRE(knobStateChar(k) == '-');
  for (float p = 0.7f; p <= 0.8f; p += 0.001f) { in.v = p; k.poll(DT); }
  REQUIRE(knobStateChar(k) == 'C');           // catching up from 0.2
  for (float p = 0.8f; p >= 0.0f; p -= 0.001f) { in.v = p; k.poll(DT); }
  in.v = 0.0f; for (int i = 0; i < 200; i++) k.poll(DT);
  REQUIRE(knobStateChar(k) == 'T');           // met at the bottom
}

// --- the row-2 pots: one independent position Knob per head ----------------

TEST_CASE("KNOB_MUX_CH: panel order is the netlist, and the table is a permutation of the 8 channels") {
  const int expect[MUX_KNOBS] = {2, 1, 0, 3, 5, 7, 6, 4};   // RV3 RV2 RV1 RV4 RV6 RV8 RV7 RV5
  bool seen[MUX_KNOBS] = {false};
  for (int k = 0; k < MUX_KNOBS; k++) {
    REQUIRE(KNOB_MUX_CH[k] == expect[k]);
    REQUIRE(KNOB_MUX_CH[k] >= 0);
    REQUIRE(KNOB_MUX_CH[k] < MUX_KNOBS);
    REQUIRE_FALSE(seen[KNOB_MUX_CH[k]]);
    seen[KNOB_MUX_CH[k]] = true;
  }
}

// The firmware's eight head-position knobs, as dreamosc.cpp builds them: knob
// i reads its own input and is bound to seq.position[i] with positionSpec().
struct HeadKnobs {
  FakeInput in[MUX_KNOBS];
  std::vector<Knob> k;
  explicit HeadKnobs(Sequencer& seq) {
    for (int i = 0; i < MUX_KNOBS; i++) k.emplace_back(in[i], &seq.position[i], positionSpec());
  }
  void set(const float v[MUX_KNOBS]) { for (int i = 0; i < MUX_KNOBS; i++) in[i].v = v[i]; }
  void poll() { for (auto& x : k) x.poll(DT); }
};

TEST_CASE("head knobs: no position changes until a pot moves; then only its own head") {
  Sequencer seq;
  float before[SS_STEPS];
  for (int i = 0; i < SS_STEPS; i++) before[i] = seq.position[i];
  HeadKnobs hk(seq);
  const float rest[MUX_KNOBS] = {0.9f, 0.8f, 0.7f, 0.6f, 0.5f, 0.4f, 0.3f, 0.2f};
  hk.set(rest);
  for (int n = 0; n < 50; n++) hk.poll();
  for (int i = 0; i < SS_STEPS; i++) REQUIRE(seq.position[i] == before[i]);
  for (int k = 0; k < MUX_KNOBS; k++) {
    for (int n = 1; n <= 30; n++) { hk.in[k].v = rest[k] + 0.001f * n; hk.poll(); }
    for (int n = 0; n < 50; n++) hk.poll();
    for (int i = 0; i < SS_STEPS; i++) {
      if (i <= k) REQUIRE(seq.position[i] != before[i]);
      else        REQUIRE(seq.position[i] == before[i]);
    }
  }
}

TEST_CASE("head knobs: a slow first turn writes; flutter on the others writes nothing") {
  Sequencer seq;
  float before[SS_STEPS];
  for (int i = 0; i < SS_STEPS; i++) before[i] = seq.position[i];
  HeadKnobs hk(seq);
  const float rest[MUX_KNOBS] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  hk.set(rest);
  hk.poll();
  for (int n = 1; n <= 300; n++) {
    for (int i = 0; i < MUX_KNOBS; i++)          // ADC flutter on every pot ...
      hk.in[i].v = 0.5f + ((n + i) & 1 ? 0.002f : -0.002f);
    hk.in[6].v = 0.5f + 0.001f * n;              // ... and a slow turn on pot 7
    hk.poll();
  }
  REQUIRE(hk.k[6].live());
  REQUIRE(seq.position[6] != before[6]);
  for (int i = 0; i < SS_STEPS; i++) {
    if (i == 6) continue;
    REQUIRE_FALSE(hk.k[i].live());
    REQUIRE(seq.position[i] == before[i]);
  }
}
