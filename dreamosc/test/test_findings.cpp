// Findings from the deep code review of 6917a0a, each turned into a test that
// (a) reproduces the finding on the host and (b) measures its effect on the
// RENDERED AUDIO, so "is it real" and "does it matter" are numbers, not
// arguments.
//
// History: each of these was first written tagged [!mayfail] and FAILED on
// 6917a0a (reproducing the finding: F1 one click at the injected boundary;
// F4 a hop at the wrong position; F2 estimate 63 vs 186 charged; F3 slack -1;
// F5 holds on a size change; F7 one wasted render; F8 seven refreshes per
// touch; F9 32 samples late). All were remediated in the following commit;
// the tags are gone and these are now the regression guards. F11 confirmed
// clean from the start (a hold is click-free).
//
// ISR preemption is simulated through SS_HOOK points in stretch_core.h (test
// build only): the hook runs the ISR side (seq.next()) from inside a main-loop
// routine at the exact instruction the finding names.
//
// Each test writes a WAV of the rendered audio to /tmp/dreamosc_findings/ so a
// failing case can be listened to on the host.

#include "catch_amalgamated.hpp"

#include <sys/stat.h>
#include <vector>
#include <string>

#include "test_support.h"
#include "controls_core.h"

using namespace testutil;

namespace {

const char* wavdir() {
  static const char* d = "/tmp/dreamosc_findings";
  mkdir(d, 0755);
  return d;
}
std::string wavpath(const char* name) { return std::string(wavdir()) + "/" + name; }

// The F1/F7 harness: run `total` samples with a drained producer, nudging
// stretch at n == 3000 so the pending frame is REFRESHED (the render the hook
// fires inside), and call `probe()` after EVERY service() call so a test can
// capture head state the moment its hook has fired, before a later render
// overwrites it.
template <typename P>
void run_refresh_probe(Sequencer& seq, std::vector<float>& out, uint32_t total, P probe) {
  for (uint32_t n = 0; n < total; n++) {
    if (n == 3000) seq.stretch = 51.0f;
    for (int g = 0; g < 64; g++) {
      bool did = seq.service();
      probe();
      if (!did) break;
    }
    out.push_back(seq.next());
  }
}

// Hook plumbing: one static context the C function pointer can reach.
struct HookCtx {
  Sequencer* seq = nullptr;
  std::vector<float>* out = nullptr;
  int   id = 0;            // which hook to act on
  bool  fired = false;
  size_t firedAt = 0;
  int   pickedBuf = -1, oldAfter = -1, curAfter = -1;
} gHook;

// On the named hook, if this is the SOUNDING head with a pending staged frame,
// run the ISR forward until that head's frames rotate (its hop boundary).
void hook_rotate_cur_head(int id, void* ctx) {
  if (id != gHook.id || gHook.fired) return;
  Head* h = (Head*)ctx;
  Sequencer& seq = *gHook.seq;
  if (seq.dbgCur() < 0 || h != &seq.dbgHead(seq.dbgCur())) return;
  if (!h->dbgPending()) return;
  int o = h->dbgOldIdx();
  int guard = 0;
  while (h->dbgOldIdx() == o && guard++ < SS_W) gHook.out->push_back(seq.next());
  gHook.fired = true;
  gHook.firedAt = gHook.out->size();
}

struct HookGuard {
  HookGuard(int id, Sequencer* s, std::vector<float>* o) {
    gHook = HookCtx(); gHook.seq = s; gHook.out = o; gHook.id = id;
    gSsTestHook = hook_rotate_cur_head;
  }
  ~HookGuard() { gSsTestHook = nullptr; }
};

}  // namespace

// ---------------------------------------------------------------------------
// F1: Head::render() free-buffer scan loads the ISR's two frame indices
// separately. Loading old THEN cur let a hop boundary between the two loads
// (SINGLE staged) mark the buffer that just became `old` as free; the render
// then overwrote the frame the ISR was blending from, mid-hop (one click).
// Fixed by loading cur first; hook 1 still injects the boundary between the
// two loads, so this guards the ordering.
// ---------------------------------------------------------------------------
TEST_CASE("F1: buffer-pick race overwrites the frame the ISR is blending (audible click)",
          "[finding]") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 0.0f);
  std::vector<float> out;
  HookGuard hg(1, &seq, &out);

  // Steady state: past the first boundary, mid-hop. Then change stretch so the
  // pending frame is REFRESHED; hook 1 fires inside that refresh's buffer scan.
  // Capture the pick of the render the hook fired in, before any later render
  // overwrites dbgLastPick().
  run_refresh_probe(seq, out, 48000, [&] {
    if (gHook.fired && gHook.pickedBuf < 0) {
      Head& h = seq.dbgHead(seq.dbgCur());
      gHook.pickedBuf = h.dbgLastPick();
      gHook.oldAfter = h.dbgOldIdx(); gHook.curAfter = h.dbgCurIdx();
    }
  });
  write_wav(wavpath("F1_buffer_pick_race.wav").c_str(), out);
  REQUIRE(gHook.fired);                       // the scenario was injected

  size_t firstClick = 0;
  int clicks = count_clicks(out, [](size_t) { return false; }, &firstClick);
  WARN("F1: render picked buffer " << gHook.pickedBuf << " while ISR old=" << gHook.oldAfter
       << " cur=" << gHook.curAfter << "; clicks in output: " << clicks
       << (clicks ? " (first at sample " + std::to_string(firstClick) + ", hook fired at "
                    + std::to_string(gHook.firedAt) + ")" : ""));
  // Correct behaviour: the pick is neither of the ISR's live frames, and the
  // interior stays click-free.
  REQUIRE(gHook.pickedBuf != gHook.oldAfter);
  REQUIRE(gHook.pickedBuf != gHook.curAfter);
  REQUIRE(clicks == 0);
}

// ---------------------------------------------------------------------------
// F2: on the host the ISR clock does not advance inside a render, so the
// measured cost is 0 and costSamples_ decays to its floor. The costed tests
// then exercise the refresh/slack rules with a near-zero cost.
// ---------------------------------------------------------------------------
TEST_CASE("F2: costed harness runs the scheduler with a dead cost model", "[finding]") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.25f, 0.0f);
  uint32_t seeded = seq.costSamples(ssSizeIdx(4096));
  auto out = render_costed(seq, 48000 * 4, 0.0038);
  uint32_t after = seq.costSamples(ssSizeIdx(4096));
  uint32_t modelled = (uint32_t)(0.0038 * 4096 * 12);
  WARN("F2: cost estimate for 4096: seeded " << seeded << ", after a 4 s costed run " << after
       << " (harness charges " << modelled << "). No audio effect; test fidelity only.");
  write_wav(wavpath("F2_costed_run.wav").c_str(), out);
  // A faithful harness keeps the estimate near what it charges.
  REQUIRE(after >= modelled / 2);
}

// ---------------------------------------------------------------------------
// F3: with fade > 0 the next head is armed while elapsed_ is already past
// onset, so its due is stamped "now" for the whole seam; the profiler's slack
// reads negative at every crossfaded seam even with an infinite producer.
// ---------------------------------------------------------------------------
TEST_CASE("F3: armed head's deadline reads 'now' during a crossfade seam", "[finding]") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.25f);
  int32_t minSlack = 0x7fffffff;
  uint32_t minDue = 0xffffffffu;
  std::vector<float> out;
  for (uint32_t n = 0; n < 48000 * 3; n++) {
    drain(seq);
    int32_t s = seq.takeMinSlack();
    if (s < minSlack) minSlack = s;
    if (seq.dbgNxt() >= 0) {
      int32_t d = (int32_t)(seq.dbgHead(seq.dbgNxt()).dbgDue() - seq.clock());
      if ((uint32_t)d < minDue) minDue = (uint32_t)d;
    }
    out.push_back(seq.next());
  }
  write_wav(wavpath("F3_seam_due.wav").c_str(), out);
  WARN("F3: min slack reported with an infinite producer: " << minSlack
       << "; min (due - clock) of the armed head: " << (int32_t)minDue
       << ". Audio unaffected (drained producer); scheduler priority + profiler reading.");
  REQUIRE(minSlack >= 0);
}

// ---------------------------------------------------------------------------
// F4: refreshesPending_ only resets when the ISR applies a frame, which for an
// ARMED head is go-live. The second position turn on the upcoming step during
// a dwell is dead until go-live, and then costs a hop.
// ---------------------------------------------------------------------------
TEST_CASE("F4: only one refresh per armed life: a second knob turn on the next step is dead",
          "[finding]") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  auto run = [&](float p1, float p2, size_t* live) {
    Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 2.0f, 0.0f);
    std::vector<float> out;
    *live = 0;
    for (uint32_t n = 0; n < 48000 * 3; n++) {
      if (n == 24000) seq.position[1] = p1;                 // first turn at 0.5 s
      if (n == 60000 && p2 > 0) seq.position[1] = p2;       // second at 1.25 s
      if (*live == 0 && seq.curStep() == 1) *live = n;
      drain(seq);
      out.push_back(seq.next());
    }
    return out;
  };
  size_t liveA, liveB, liveC;
  auto a = run(0.5f, 0.0f, &liveA);   // only ever 0.5
  auto b = run(0.5f, 0.9f, &liveB);   // 0.5 then 0.9 (what the player did)
  auto c = run(0.9f, 0.0f, &liveC);   // 0.9 from the start (what B should sound like)
  REQUIRE(liveA == liveB);
  write_wav(wavpath("F4_dead_second_turn.wav").c_str(), b);
  size_t diffBA = b.size(), diffBC = b.size();
  for (size_t i = liveB; i < b.size(); i++) if (a[i] != b[i]) { diffBA = i; break; }
  for (size_t i = liveB; i < b.size(); i++) if (c[i] != b[i]) { diffBC = i; break; }
  WARN("F4: step 1 went live at " << liveB << "; B first differs from the 0.5-only run "
       << (diffBA - liveB) << " samples after go-live (one hop = 2048), and from the 0.9-from-"
       "the-start run " << (diffBC - liveB) << " samples after go-live. So step 1 sounds "
       "at the OLD position for its first hop, then jumps.");
  // Correct behaviour: the second turn reached the pre-roll, so B matches C at
  // go-live and departs from A immediately.
  REQUIRE(diffBA - liveB < 64);
}

// ---------------------------------------------------------------------------
// F5: a frame-size change on a gated head was a REQUIRED pair with no slack
// check, so a pair that could not make its boundary held (repeated) a frame.
// Fixed: when the pair would miss but could fit in a full hop, a SINGLE at the
// old size is staged instead and the pair renders right after the boundary.
// Two cases: (a) SCHEDULING -- a single sounding head, the pair fits a hop:
// no holds at all; (b) THROUGHPUT -- a fast dwell with a full 0.5 crossfade,
// so cur + inc both render 16384 pairs through the seam (plus the armed
// pre-roll), the heaviest concurrent render load the sequential model allows
// (three heads; ring-out that once stacked six is gone). Holds may occur while
// those pairs turn around; the guard is that they stay bounded and never go
// silent. (The pre-fix "1 hold" was measured with a harness that charged a
// pair as one frame; the honest number is a handful.)
// ---------------------------------------------------------------------------
namespace {
struct GrowthRun { uint32_t holds; int gated; int silent; };
GrowthRun grow_to_16384(float fade, std::vector<float>& out) {
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.25f, fade);
  CostedProducer p(0.0038);
  uint32_t holdsBefore = 0; int gated = 0;
  for (uint32_t n = 0; n < 48000 * 4; n++) {
    if (n == 48000 * 2) {
      holdsBefore = seq.holds(); gated = seq.activeVoices();
      seq.setFrame(16384);
    }
    p.step(seq, n);
    out.push_back(seq.next());
  }
  return { seq.holds() - holdsBefore, gated, silent_windows(out, first_audible(out)) };
}
}  // namespace

TEST_CASE("F5a: growing to 16384 with the pair fitting a hop is hold-free", "[finding]") {
  gTab.init();
  std::vector<float> out;
  GrowthRun r = grow_to_16384(0.0f, out);                   // butt-joint: 1 head
  write_wav(wavpath("F5a_growth_two_heads.wav").c_str(), out);
  WARN("F5a: " << r.gated << " gated heads at the change; holds in the 2 s after: " << r.holds
       << "; silent windows: " << r.silent);
  REQUIRE(r.silent == 0);
  REQUIRE(r.holds == 0);
}

TEST_CASE("F5b: growing to 16384 under a full crossfade is throughput-bound: bounded holds, no silence",
          "[finding]") {
  gTab.init();
  std::vector<float> out;
  GrowthRun r = grow_to_16384(0.5f, out);                   // cur + inc both render
  write_wav(wavpath("F5b_growth_at_cap.wav").c_str(), out);
  WARN("F5b: " << r.gated << " gated heads at the change; holds in the 2 s after: " << r.holds
       << " (each a spectral freeze on one head); silent windows: " << r.silent);
  REQUIRE(r.silent == 0);
  // Bounded holds under the heaviest concurrent-render case (cur + inc pairs at
  // 16384). 60 leaves margin for scheduling order; if this rises, the fallback
  // regressed. If the render cost drops on the bench (480 MHz, CMSIS FFT),
  // tighten it from the new measurement.
  REQUIRE(r.holds <= 60);
}

// ---------------------------------------------------------------------------
// F7: plan() and render() read the applied frame index separately. If the
// ISR applies the pending frame between them (a refresh pick), the refresh is
// rendered for a frame that is already playing: a wasted render (the ISR
// discards it at the next boundary). A hold follows only if the producer has
// no time left to render the real next frame.
// ---------------------------------------------------------------------------
TEST_CASE("F7: a boundary between plan() and render() wastes a render", "[finding]") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 0.0f);
  std::vector<float> out;
  HookGuard hg(2, &seq, &out);
  int stale = -1;
  run_refresh_probe(seq, out, 48000, [&] {          // the stretch nudge provokes a refresh pick
    if (gHook.fired && stale < 0) stale = seq.dbgHead(seq.dbgCur()).dbgStaleQueued();
  });
  write_wav(wavpath("F7_plan_render_desync.wav").c_str(), out);
  REQUIRE(gHook.fired);
  int clicks = count_clicks(out, [](size_t) { return false; });
  WARN("F7: descriptors queued for an already-applied frame right after the injected "
       "boundary: " << stale << " (a wasted render); holds: " << seq.holds()
       << " (the drained producer had time to render the real next frame); clicks: " << clicks);
  REQUIRE(clicks == 0);
  REQUIRE(stale == 0);
}

// ---------------------------------------------------------------------------
// F8: the knob smoother (coeff 0.02 at 1 ms) creeps for ~250 ms after any
// touch, and every creep step above threshold was a refresh (7 per touch).
// Fixed at both ends: position is now fed from the RAW pot (PanelEditor), and
// the core caps refreshes at one per hop per head with a 0.002 threshold.
// ---------------------------------------------------------------------------
TEST_CASE("F8: a pot move costs at most one refresh raw, one per hop smoothed", "[finding]") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  // Drive position the way the panel now does (raw) and the old way
  // (smoothed), and count refreshes in the second after one 30% move.
  auto run = [&](bool smoothed) {
    Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 0.0f);
    std::vector<float> out;
    float state = 0.1f; bool primed = false;
    uint32_t refreshesAt = 0;
    for (uint32_t n = 0; n < 48000 * 2; n++) {
      if (n % 48 == 0) {                                      // 1 ms control poll
        float raw = n < 48000 ? 0.1f : 0.4f;                  // a 30% move at 1 s
        seq.position[0] = smoothed ? smoothKnob(state, raw, primed) : raw;
        primed = true;
      }
      if (n == 48000) refreshesAt = seq.refreshes();
      drain(seq);
      out.push_back(seq.next());
    }
    write_wav(wavpath(smoothed ? "F8_pot_smoothed.wav" : "F8_pot_raw.wav").c_str(), out);
    REQUIRE(count_clicks(out, [](size_t) { return false; }) == 0);
    return seq.refreshes() - refreshesAt;
  };
  uint32_t raw = run(false), smoothed = run(true);
  WARN("F8: refreshes in the second after one 30% pot move: raw feed " << raw
       << ", smoothed feed " << smoothed << " (cap: one per hop while it creeps).");
  REQUIRE(raw <= 2);
  // Smoothed: the creep lasts ~250 ms = ~6 hops at 4096; the cap holds it there.
  REQUIRE(smoothed <= 8);
}

TEST_CASE("F8: a small position move (0.5%) triggers a refresh; below 0.2% does not", "[finding]") {
  // The 30% move above is only a stimulus with a long visible creep. The
  // actual trigger is a 0.002 change in position since the frame was last
  // rendered (about 20 ms of a 10 s source). Nudge by 0.5% and by 0.1%.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  auto run = [&](float delta) {
    Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 0.0f);
    uint32_t refreshesAt = 0;
    for (uint32_t n = 0; n < 48000 * 2; n++) {
      if (n == 48000) { refreshesAt = seq.refreshes(); seq.position[0] += delta; }
      drain(seq);
      seq.next();
    }
    return seq.refreshes() - refreshesAt;
  };
  uint32_t small = run(0.005f), tiny = run(0.001f);
  WARN("F8: refreshes after a 0.5% move: " << small << "; after a 0.1% move: " << tiny);
  REQUIRE(small == 1);
  REQUIRE(tiny == 0);
}

TEST_CASE("F8: PanelEditor writes position from the raw pot, not the smoothed read", "[finding]") {
  static std::vector<float> pool(SS_POOL_FLOATS);
  auto srcbuf = make_source(0.5f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; seq.init(&src, 48000, pool.data());
  float dur = 1.0f, gd = 0.0f;
  PanelEditor pe;
  pe.prime(0.10f, 0.0f);
  pe.advance();                                              // step 1
  pe.update(seq, &dur, &gd, 0.10f, 0.0f, 0.10f, 0.0f);       // re-anchor
  // Raw pot jumps to 0.80; the smoothed read is still lagging at 0.30.
  pe.update(seq, &dur, &gd, 0.80f, 0.0f, 0.30f, 0.0f);
  REQUIRE(seq.position[0] == Approx(0.80f));                 // raw, not 0.30
}

// ---------------------------------------------------------------------------
// F9: shrinking activeSteps under an armed head is only handled at seam time,
// so that seam runs late by one pre-roll pair render.
// ---------------------------------------------------------------------------
TEST_CASE("F9: step-count shrink under an armed head makes the seam run late", "[finding]") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.5f, 0.0f);
  std::vector<float> out;
  CostedProducer p(0.0038);
  uint32_t lateAt = 0;
  for (uint32_t n = 0; n < 48000 * 3; n++) {
    if (n == 48000 + 4800) { lateAt = seq.lateSamples(); seq.setSteps(2); }
    p.step(seq, n);
    out.push_back(seq.next());
  }
  write_wav(wavpath("F9_steps_shrink_late.wav").c_str(), out);
  uint32_t late = seq.lateSamples() - lateAt;
  WARN("F9: late samples after the shrink: " << late << " (~" << late / 48 << " ms the outgoing "
       "step overran); silent windows: " << silent_windows(out, first_audible(out)));
  REQUIRE(silent_windows(out, first_audible(out)) == 0);
  REQUIRE(late == 0);
}

// ---------------------------------------------------------------------------
// F11: the hold path (old_ = cur_) was never run through the click detector.
// Confirms a hold is continuous (the IFFT frame is periodic). Expected to PASS.
// ---------------------------------------------------------------------------
TEST_CASE("F11: a held (repeated) frame is click-free", "[finding]") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 0.0f);
  std::vector<float> out;
  for (uint32_t i = 0; i < 4096; i++) { drain(seq); out.push_back(seq.next()); }
  uint32_t before = gUnderruns;
  for (uint32_t i = 0; i < 4096 * 3; i++) out.push_back(seq.next());   // starve
  write_wav(wavpath("F11_hold_path.wav").c_str(), out);
  REQUIRE(gUnderruns > before);
  int clicks = count_clicks(out, [](size_t) { return false; });
  WARN("F11: holds " << (gUnderruns - before) << ", clicks " << clicks);
  REQUIRE(clicks == 0);
}
