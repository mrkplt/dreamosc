// Unit tests for the platform-free DSP core (stretch_core.h).
//
// These assert the properties that matter for a tuned/randomized algorithm --
// determinism, invariants, bounds, seam continuity, live-control latency, and
// the scheduling behaviour under a cost-modelled producer -- rather than exact
// sample values. Tolerances are set from measurement with a written reason.
//
// Frame model: a head is (old, cur) frames + a phase; the ISR blends them and
// rotates at hop boundaries from a staged queue the main loop fills. Heads come
// from a pool, are armed a whole dwell ahead, and go live with a pre-roll pair
// so a raw cut is amplitude-continuous. An under-fed head repeats its frame.

#define CATCH_CONFIG_MAIN
#include "catch_amalgamated.hpp"

#include <vector>

#include "test_support.h"

using testutil::make_source;
using testutil::render;
using testutil::render_costed;
using testutil::rms;
using testutil::rms_range;
using testutil::first_audible;

namespace {

void make_seq(Sequencer& seq, const Source* src, float sr, float stretch,
              float duration, float fade = 0.0f, float drift = 0.0f,
              uint32_t seed = 0x12345678u) {
  static std::vector<float> pool(SS_POOL_FLOATS);
  seq.init(src, sr, pool.data(), seed);
  seq.stretch  = stretch;
  seq.duration = duration;
  seq.fade     = fade;
  for (int i = 0; i < SS_STEPS; i++) seq.drift[i] = drift;
}

// Drive for `total` samples with a drained producer, calling `hook(n)` before
// each sample (to poke controls mid-run). Returns the output.
template <typename F>
std::vector<float> drive(Sequencer& seq, uint32_t total, F hook) {
  std::vector<float> out; out.reserve(total);
  for (uint32_t n = 0; n < total; n++) {
    hook(n);
    for (int g = 0; g < 64 && seq.service(); g++) {}
    out.push_back(seq.next());
  }
  return out;
}

// Longest run of 5 ms windows whose RMS is below `thresh`, after `from`.
int silent_windows(const std::vector<float>& out, size_t from, double thresh = 1e-4) {
  const size_t W = 240;
  int worst = 0, run = 0;
  for (size_t s = from; s + W <= out.size(); s += W) {
    if (rms_range(out, s, W) < thresh) { run++; if (run > worst) worst = run; }
    else run = 0;
  }
  return worst;
}

}  // namespace

// --- seamGeom: pure seam geometry ------------------------------------------

TEST_CASE("seamGeom: fade 0 is a raw cut (onset == duration, no overlap)") {
  SeamGeom g = seamGeom(48000, 0.0f);
  REQUIRE(g.onset == 48000);
  REQUIRE(g.fadeLen == 0);
}

TEST_CASE("seamGeom: fade > 0 opens the seam fade*duration before the end") {
  SeamGeom g = seamGeom(48000, 0.25f);
  REQUIRE(g.fadeLen == 12000);
  REQUIRE(g.onset == 36000);
}

TEST_CASE("seamGeom: fade clamps to [0, 0.5]") {
  REQUIRE(seamGeom(48000, -1.0f).fadeLen == 0);
  REQUIRE(seamGeom(48000, 5.0f).fadeLen == 24000);
}

// --- tables -----------------------------------------------------------------

TEST_CASE("tables: windows, gains and blend curves for every size") {
  gTab.init();
  for (int s = 0; s < SS_NSIZES; s++) {
    int w = ssSizeW(s), h = w / 2;
    REQUIRE(ssSizeIdx(w) == s);
    REQUIRE(std::isfinite(gTab.synthGain[s]));
    REQUIRE(gTab.synthGain[s] > 0.0f);
    const float* win = &gWindows[ssWinOff(s)];
    REQUIRE(win[0] == Approx(0.0f).margin(1e-3));
    REQUIRE(win[w / 2] == Approx(1.0f).margin(1e-2));
    const float* A = &gBlendA[ssHopOff(s)];
    const float* C = &gBlendC[ssHopOff(s)];
    REQUIRE(A[0] == Approx(1.0f));                 // hop start: 100% old
    REQUIRE(A[h / 2] == Approx(0.5f).margin(1e-3)); // mid-hop: equal mix
    // Canonical AM correction: attenuate the joint (a single frame, full
    // level) to 1/sqrt2 so it matches the mid-hop equal-power mix, which is
    // left at 1.0. The envelope comes out flat at 0.707 overall.
    REQUIRE(C[0] == Approx(0.70711f).margin(1e-3));
    REQUIRE(C[h / 2] == Approx(1.0f).margin(1e-3));
  }
  REQUIRE(ssWinOff(SS_NSIZES - 1) + SS_W_MIN == SS_WIN_FLOATS);
  REQUIRE(ssHopOff(SS_NSIZES - 1) + SS_W_MIN / 2 == SS_HOP_FLOATS);
}

TEST_CASE("ssClampW: any request lands on a legal power of two") {
  REQUIRE(ssClampW(1) == SS_W_MIN);
  REQUIRE(ssClampW(1 << 20) == SS_W);
  REQUIRE(ssClampW(3000) == 2048);
  REQUIRE(ssClampW(4096) == 4096);
}

TEST_CASE("ssHash2 is deterministic and never zero") {
  REQUIRE(ssHash2(1, 2) == ssHash2(1, 2));
  REQUIRE(ssHash2(1, 2) != ssHash2(2, 1));
  REQUIRE(ssHash2(0, 0) != 0);
}

// --- render-level invariants ------------------------------------------------

TEST_CASE("output contains no NaN or Inf and stays within bounds") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.5f);
  auto out = render(seq);
  REQUIRE(out.size() > 0);
  for (float v : out) { REQUIRE(std::isfinite(v)); REQUIRE(std::abs(v) <= 1.0f); }
}

TEST_CASE("deterministic: same config renders identically") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seqA; make_seq(seqA, &src, 48000, 50.0f, 1.0f, 0.5f);
  Sequencer seqB; make_seq(seqB, &src, 48000, 50.0f, 1.0f, 0.5f);
  auto a = render(seqA);
  auto b = render(seqB);
  REQUIRE(a.size() == b.size());
  for (size_t i = 0; i < a.size(); i++) REQUIRE(a[i] == b[i]);
}

TEST_CASE("startup: audible as soon as the first pre-roll pair is rendered") {
  // With a drained producer the first head goes live on the second sample (the
  // first tick arms it; the next service renders its pair). No lookahead wait.
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.0f);
  auto out = render(seq);
  REQUIRE(first_audible(out) <= 8);
}

// --- seam continuity (the "silence between pieces" guard) -------------------

TEST_CASE("raw cut is amplitude-continuous at every frame size (pre-roll)") {
  // Measured before pre-roll: the first 5 ms after a cut sat 34 dB (4096) to
  // 62 dB (16384) below steady state and recovered over one hop. The test
  // material (a few spectral lines) makes 5 ms windows wander deeply at large
  // windows (up to ~12 dB at 16384 against a hop-long reference), so the
  // threshold is SELF-CALIBRATED: the deepest 5 ms window in the half-dwell
  // BEFORE the cut (steady state, well past the previous seam) sets the floor,
  // and no window in the hop AFTER the cut may sit more than 6 dB under it. A
  // fade-in from silence fails this by 20+ dB at every size.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  for (int w = 512; w <= SS_W; w <<= 1) {
    Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.5f, 0.0f);
    seq.setFrame(w);
    uint32_t dur = seq.durSamples();
    auto out = render(seq, 2);
    size_t s0 = first_audible(out);
    int hop = w / 2;
    for (int k = 1; k <= 3; k++) {
      size_t seam = s0 + (size_t)k * dur;
      size_t before = dur / 2;
      double ref = rms_range(out, seam - before, before);
      REQUIRE(ref > 1e-3);
      double floorDb = 0.0;
      for (size_t s = seam - before; s + 240 <= seam; s += 240) {
        double db = 20.0 * std::log10(std::max(rms_range(out, s, 240), 1e-9) / ref);
        if (db < floorDb) floorDb = db;
      }
      for (size_t s = seam; s + 240 <= seam + (size_t)hop; s += 240) {
        double db = 20.0 * std::log10(std::max(rms_range(out, s, 240), 1e-9) / ref);
        INFO("w=" << w << " seam " << k << " offset " << (s - seam) << " dB " << db
             << " (floor before the cut " << floorDb << ")");
        REQUIRE(db > floorDb - 6.0);
      }
    }
  }
}

TEST_CASE("a head's first hop is at steady-state level (per-head pre-roll)") {
  // Cleaner than the seam test: the first hop after startup vs the fifth hop.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  for (int w : {1024, 4096, 16384}) {
    Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 0.0f);
    seq.setFrame(w);
    auto out = render(seq);
    size_t s0 = first_audible(out);
    int hop = w / 2;
    double first = rms_range(out, s0, hop), fifth = rms_range(out, s0 + 4 * hop, hop);
    double db = 20.0 * std::log10(first / fifth);
    INFO("w=" << w << " first hop vs fifth: " << db << " dB");
    REQUIRE(std::abs(db) < 6.0);
  }
}

TEST_CASE("single active step: no hole between dwells") {
  // The degenerate 1-step case used to re-arm the sounding head in place, with a
  // lookahead of silence per dwell. A pool head per visit closes it.
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.5f, 0.0f);
  seq.setSteps(1);
  auto out = render(seq, 4);
  size_t s0 = first_audible(out);
  REQUIRE(silent_windows(out, s0) == 0);
  REQUIRE(rms(out) > 0.01);
}

TEST_CASE("no local discontinuity in head interiors (click detector)") {
  // Seams are excluded (a raw-cut splice of uncorrelated material is spectral,
  // not a transient -- design ruling). Nothing INSIDE a head may step.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.0f);
  uint32_t dur = seq.durSamples();
  auto out = render(seq, 2);
  size_t s0 = first_audible(out);

  auto near_seam = [&](size_t i) {
    for (uint32_t k = 0; k <= 2 * SS_STEPS + 2; k++) {
      int64_t seam = (int64_t)s0 + (int64_t)k * dur;
      if ((int64_t)i > seam - 1024 && (int64_t)i < seam + 1024) return true;
    }
    return false;
  };

  const int W = 64;
  int clicks = 0;
  for (size_t i = W + 1; i + W < out.size(); i++) {
    if (near_seam(i)) continue;
    float d = std::abs(out[i] - out[i - 1]);
    if (d < 5e-3f) continue;
    float local = 0.0f;
    for (int k = -W; k < W; k++) local += std::abs(out[i + k] - out[i + k - 1]);
    local /= (2 * W);
    if (d > 8.0f * local) {
      if (clicks < 5)
        UNSCOPED_INFO("click at " << i << " (" << i / 48000.0 << "s): delta "
                      << d << " vs local " << local);
      clicks++;
    }
  }
  REQUIRE(clicks == 0);
}

TEST_CASE("level held roughly flat across crossfade (constant-loudness)") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  double prev = -1.0;
  for (float fade : {0.0f, 0.15f, 0.3f, 0.5f}) {
    Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 2.0f, fade);
    double r = rms(render(seq));
    REQUIRE(r > 0.0);
    if (prev > 0.0) REQUIRE(std::abs(20.0 * std::log10(r / prev)) < 3.0);
    prev = r;
  }
}

TEST_CASE("no per-step volume dips at max fade") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.5f);
  auto out = render(seq, 2);
  const uint32_t w = 2400;
  uint32_t begin = 48000, end = (uint32_t)out.size() - 48000;
  float lo = 1e9f, hi = 0.0f;
  for (uint32_t s = begin; s + w < end; s += w / 2) {
    float r = (float)rms_range(out, s, w);
    lo = std::min(lo, r); hi = std::max(hi, r);
  }
  INFO("windowed RMS ratio " << 20.0 * std::log10(hi / std::max(lo, 1e-9f)) << " dB");
  // A continuous head shows ~8.7 dB of windowed-RMS wander; a full-step sine
  // envelope measured 30.6 dB. 12 dB separates them.
  REQUIRE(20.0 * std::log10(hi / std::max(lo, 1e-9f)) < 12.0);
}

TEST_CASE("renders at multiple sample rates without blowing up") {
  gTab.init();
  for (uint32_t sr : {44100u, 48000u, 96000u}) {
    auto srcbuf = make_source(1.5f, sr);
    Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
    Sequencer seq; make_seq(seq, &src, (float)sr, 30.0f, 1.0f, 0.5f);
    auto out = render(seq);
    REQUIRE(out.size() > 0);
    for (float v : out) REQUIRE(std::isfinite(v));
  }
}

// --- the robotic clock -------------------------------------------------------

TEST_CASE("duration is live and unquantized (frame size does not bend timing)") {
  gTab.init();
  auto srcbuf = make_source(1.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.25f, 0.0f);
  uint32_t at4096 = seq.durSamples();
  seq.setFrame(16384);
  REQUIRE(seq.durSamples() == at4096);
  REQUIRE(at4096 == (uint32_t)(0.25f * 48000 + 0.5f));
}

TEST_CASE("duration crank-down mid-dwell fast-marches with no silence") {
  // A minute-long dwell, duration cranked to 0.25 s a third of a second in. The
  // next head is armed a whole dwell ahead, so the march starts immediately and
  // there is no cold-arrival gap at all.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 60.0f, 0.0f);
  int stepsSeen = 0, last = -1;
  auto out = drive(seq, 48000 * 3, [&](uint32_t n) {
    if (n == 14400) seq.duration = 0.25f;
    int s = seq.curStep();
    if (s != last) { stepsSeen++; last = s; }
  });
  for (float v : out) REQUIRE(std::isfinite(v));
  REQUIRE(stepsSeen > 8);                          // it marched
  REQUIRE(silent_windows(out, first_audible(out)) == 0);
  REQUIRE(seq.lateSamples() <= 8);                 // only the startup pair
}

TEST_CASE("steady short-duration march stays fed") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.25f, 0.0f);
  auto out = render(seq, 3);
  REQUIRE(silent_windows(out, first_audible(out)) == 0);
  REQUIRE(seq.holds() == 0);
}

TEST_CASE("shrinking activeSteps under an armed head re-arms on a valid step") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.2f, 0.0f);
  bool bad = false;
  auto out = drive(seq, 48000 * 2, [&](uint32_t n) {
    if (n == 48000 / 2) seq.setSteps(2);
    if (n > 48000 / 2 + 48000 / 5 && seq.curStep() >= 2) bad = true;
  });
  for (float v : out) REQUIRE(std::isfinite(v));
  REQUIRE_FALSE(bad);
  REQUIRE(silent_windows(out, first_audible(out)) == 0);
}

// --- live controls reach the sounding head ---------------------------------

TEST_CASE("live frame size reaches the sounding head within one hop") {
  // 4096 -> 1024: the refresh pair is cheap against the remaining hop, so it
  // replaces the pending frame and applies at the very next boundary.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 0.0f);
  uint32_t when = 48000, seen = 0;
  auto out = drive(seq, 48000 * 2, [&](uint32_t n) {
    if (n == when) seq.setFrame(1024);
    if (n > when && seen == 0 && seq.curHop() == 512) seen = n;
  });
  for (float v : out) { REQUIRE(std::isfinite(v)); REQUIRE(std::abs(v) <= 1.0f); }
  REQUIRE(seen > 0);
  INFO("applied after " << (seen - when) << " samples");
  REQUIRE(seen - when <= 2048 + 64);               // one old-size hop, plus slack
  REQUIRE(silent_windows(out, first_audible(out)) == 0);
}

TEST_CASE("live frame size up to 16384 applies within two hops") {
  // 4096 -> 16384: a 16384 pre-roll pair is modelled at ~36 ms, which can
  // exceed what is left of the current 43 ms hop; the refresh is then refused
  // (it would miss) and the pair lands one boundary later. Two hops max.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 0.0f);
  uint32_t when = 48000, seen = 0;
  auto out = drive(seq, 48000 * 2, [&](uint32_t n) {
    if (n == when) seq.setFrame(16384);
    if (n > when && seen == 0 && seq.curHop() == 8192) seen = n;
  });
  for (float v : out) { REQUIRE(std::isfinite(v)); REQUIRE(std::abs(v) <= 1.0f); }
  REQUIRE(seen > 0);
  INFO("applied after " << (seen - when) << " samples");
  REQUIRE(seen - when <= 2 * 2048 + 64);
  REQUIRE(silent_windows(out, first_audible(out)) == 0);
}

TEST_CASE("changing frame size repeatedly mid-render stays clean") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.25f);
  seq.setFrame(SS_W);
  const int sizes[] = {SS_W, 512, 2048, 256, 1024, SS_W};
  int si = 0;
  uint32_t total = seq.patternSamples();
  auto out = drive(seq, total, [&](uint32_t n) {
    if (n % (total / 64 + 1) == 0) { seq.setFrame(sizes[si % 6]); si++; }
  });
  for (float v : out) { REQUIRE(std::isfinite(v)); REQUIRE(std::abs(v) <= 1.0f); }
  REQUIRE(rms(out) > 0.0f);
  REQUIRE(silent_windows(out, first_audible(out)) == 0);
}

TEST_CASE("live position moves the sounding head (and is deterministic)") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  auto run = [&](bool move) {
    Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 0.0f);
    return drive(seq, 48000, [&](uint32_t n) { if (move && n == 24000) seq.position[0] = 0.7f; });
  };
  auto a = run(false), b = run(true), c = run(true);
  for (size_t i = 0; i < 24000; i++) REQUIRE(a[i] == b[i]);     // identical before
  size_t firstDiff = a.size();
  for (size_t i = 24000; i < a.size(); i++) if (a[i] != b[i]) { firstDiff = i; break; }
  INFO("first difference " << (firstDiff - 24000) << " samples after the move");
  REQUIRE(firstDiff < 24000 + 2 * 2048);            // within two hops
  for (size_t i = 0; i < b.size(); i++) REQUIRE(b[i] == c[i]);   // deterministic
}

TEST_CASE("live stretch reaches the sounding head within two hops") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  auto run = [&](bool move) {
    Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 0.0f);
    return drive(seq, 48000, [&](uint32_t n) { if (move && n == 24000) seq.stretch = 5.0f; });
  };
  auto a = run(false), b = run(true);
  size_t firstDiff = a.size();
  for (size_t i = 24000; i < a.size(); i++) if (a[i] != b[i]) { firstDiff = i; break; }
  REQUIRE(firstDiff < 24000 + 2 * 2048);
}

// --- scheduling under a cost-modelled producer ------------------------------

TEST_CASE("cost-modelled producer at bench cost keeps a fast ring-out march fed") {
  // 0.0038 samples per (w log2 w) = ~18 ms per 16384 frame at 48 kHz, the bench
  // number at the old clock. At 4096 with a 0.25 s march and 2 s ring-out up to
  // six heads render at once; the EDF scheduler must keep them all fed.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.25f, 0.0f);
  seq.ringout = 2.0f;
  auto out = render_costed(seq, 48000 * 4, 0.0038);
  for (float v : out) { REQUIRE(std::isfinite(v)); REQUIRE(std::abs(v) <= 1.0f); }
  REQUIRE(silent_windows(out, first_audible(out)) == 0);
  INFO("holds " << seq.holds() << " late " << seq.lateSamples());
  REQUIRE(seq.holds() == 0);
}

TEST_CASE("a knob turn right after go-live is not starved by the armed pair (costed, 16384)") {
  // At 16384 the next step's pre-roll pair (two renders, ~36 ms modelled) is
  // pending right after every go-live. A position change on the SOUNDING head
  // must still land at its first hop boundary: the far-future pair yields to
  // a slack-valid refresh. Compare against an unchanged run.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  auto run = [&](bool move) {
    Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 0.0f);
    seq.setFrame(16384);
    std::vector<float> out; out.reserve(48000);
    double budget = 0.0; const double cost = 0.0038 * 16384 * 14;
    size_t live = 0;
    for (uint32_t n = 0; n < 48000; n++) {
      if (live == 0 && seq.curHop() > 0) live = n;
      if (move && live > 0 && n == live + 100) seq.position[0] = 0.7f;
      if (n % 32 == 0) {
        budget += 32;
        while (budget >= cost && seq.service()) budget -= cost;
        if (budget > 4.0 * cost) budget = 4.0 * cost;
      }
      out.push_back(seq.next());
    }
    return std::make_pair(out, live);
  };
  auto a = run(false), b = run(true);
  REQUIRE(a.second == b.second);
  size_t live = a.second, firstDiff = a.first.size();
  for (size_t i = live; i < a.first.size(); i++) if (a.first[i] != b.first[i]) { firstDiff = i; break; }
  INFO("first difference " << (firstDiff - live) << " samples after go-live");
  REQUIRE(firstDiff - live <= 8192 + 64);          // at the first boundary
}

TEST_CASE("an overloaded producer degrades to frame repeats, never silence") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.25f, 0.0f);
  seq.ringout = 2.0f;
  uint32_t before = gUnderruns;
  auto out = render_costed(seq, 48000 * 3, 0.05);   // ~13x the bench cost
  for (float v : out) { REQUIRE(std::isfinite(v)); REQUIRE(std::abs(v) <= 1.0f); }
  REQUIRE(gUnderruns > before);                    // holds happened
  REQUIRE(silent_windows(out, first_audible(out)) == 0);   // but no silence
}

TEST_CASE("a starved head repeats its frame and counts holds") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 0.0f);
  for (uint32_t i = 0; i < 4096; i++) { for (int g = 0; g < 64 && seq.service(); g++) {} seq.next(); }
  uint32_t before = gUnderruns;
  std::vector<float> out;
  for (uint32_t i = 0; i < 4096 * 3; i++) out.push_back(seq.next());   // no service
  REQUIRE(gUnderruns > before);
  for (float v : out) REQUIRE(std::isfinite(v));
  REQUIRE(silent_windows(out, 0) == 0);
}

// --- ring-out ----------------------------------------------------------------

namespace {
struct RingoutRun { std::vector<float> out; int peakRemnants; int peakTotal; };
RingoutRun driveRingout(Sequencer& seq, uint32_t n) {
  RingoutRun r{{}, 0, 0};
  for (uint32_t i = 0; i < n; i++) {
    for (int g = 0; g < 128 && seq.service(); g++) {}
    int rem = seq.activeRemnants();
    int tot = seq.activeVoices() + rem;
    if (rem > r.peakRemnants) r.peakRemnants = rem;
    if (tot > r.peakTotal)    r.peakTotal = tot;
    r.out.push_back(seq.next());
  }
  return r;
}
}  // namespace

TEST_CASE("ring-out: a departing head keeps sounding") {
  gTab.init();
  auto srcbuf = make_source(4.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.3f, 0.0f);
  seq.ringout = 1.0f;
  auto r = driveRingout(seq, 48000 * 2);
  for (float v : r.out) { REQUIRE(std::isfinite(v)); REQUIRE(std::abs(v) <= 1.0f); }
  REQUIRE(r.peakRemnants >= 1);
}

TEST_CASE("ring-out: gated heads never exceed SS_RENDER_CAP") {
  gTab.init();
  auto srcbuf = make_source(4.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.15f, 0.0f);
  seq.ringout = 8.0f;
  auto r = driveRingout(seq, 48000 * 3);
  INFO("peak gated " << r.peakTotal);
  REQUIRE(r.peakTotal <= SS_RENDER_CAP);
  REQUIRE(r.peakRemnants >= 1);
  REQUIRE(seq.freeHeads() >= 1);                   // no pool leak
}

TEST_CASE("ring-out: a ringing head stops after its length") {
  gTab.init();
  auto srcbuf = make_source(4.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.5f, 0.0f);
  seq.setSteps(2);
  seq.ringout = 0.5f;
  bool sawRemnant = false, sawExpiry = false;
  for (uint32_t i = 0; i < 48000 * 4; i++) {
    for (int g = 0; g < 128 && seq.service(); g++) {}
    seq.next();
    int rem = seq.activeRemnants();
    if (rem > 0) sawRemnant = true;
    if (sawRemnant && rem == 0) sawExpiry = true;
  }
  REQUIRE(sawRemnant);
  REQUIRE(sawExpiry);
}

TEST_CASE("ring-out == 0 is byte-identical to the default") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer a; make_seq(a, &src, 48000, 50.0f, 0.4f, 0.0f);
  Sequencer b; make_seq(b, &src, 48000, 50.0f, 0.4f, 0.0f);
  b.ringout = 0.0f;
  auto oa = render(a, 3), ob = render(b, 3);
  REQUIRE(oa.size() == ob.size());
  for (size_t i = 0; i < oa.size(); i++) REQUIRE(oa[i] == ob[i]);
}

// --- step count, sizes, drift ------------------------------------------------

TEST_CASE("setSteps clamps to [1, SS_STEPS] (#149)") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f);
  REQUIRE(seq.activeSteps == SS_STEPS);
  seq.setSteps(3);  REQUIRE(seq.activeSteps == 3);
  seq.setSteps(0);  REQUIRE(seq.activeSteps == 1);
  seq.setSteps(-4); REQUIRE(seq.activeSteps == 1);
  seq.setSteps(SS_STEPS + 5); REQUIRE(seq.activeSteps == SS_STEPS);
}

TEST_CASE("a K-step sequence renders bounded and walks only K steps") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.5f);
  seq.setSteps(3);
  int maxStep = 0;
  auto out = drive(seq, seq.patternSamples() * 3, [&](uint32_t) {
    if (seq.curStep() > maxStep) maxStep = seq.curStep();
  });
  for (float s : out) { REQUIRE(std::isfinite(s)); REQUIRE(std::abs(s) <= 1.0f); }
  REQUIRE(rms(out) > 0.0f);
  REQUIRE(maxStep == 2);
}

TEST_CASE("renders cleanly at every frame size") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  for (int w = SS_W_MIN; w <= SS_W; w <<= 1) {
    Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.25f);
    seq.setFrame(w);
    REQUIRE(seq.frameSize == w);
    auto out = render(seq, 2);
    bool audible = false;
    for (float v : out) {
      REQUIRE(std::isfinite(v));
      REQUIRE(std::abs(v) <= 1.0f);
      if (std::abs(v) > 1e-3f) audible = true;
    }
    REQUIRE(audible);
  }
}

TEST_CASE("drift perturbs position within bounds and stays reproducible") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seqA; make_seq(seqA, &src, 48000, 50.0f, 0.5f, 0.0f, 0.3f, 42u);
  Sequencer seqB; make_seq(seqB, &src, 48000, 50.0f, 0.5f, 0.0f, 0.3f, 42u);
  auto a = render(seqA), b = render(seqB);
  REQUIRE(a.size() == b.size());
  for (size_t i = 0; i < a.size(); i++) REQUIRE(a[i] == b[i]);
  Sequencer seqZero; make_seq(seqZero, &src, 48000, 50.0f, 0.5f, 0.0f, 0.0f, 42u);
  auto zero = render(seqZero);
  bool differs = false;
  for (size_t i = 0; i < std::min(a.size(), zero.size()); i++)
    if (a[i] != zero[i]) { differs = true; break; }
  REQUIRE(differs);
  for (float v : a) REQUIRE(std::isfinite(v));
}
