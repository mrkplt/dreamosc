// Unit tests for the platform-free DSP core (stretch_core.h).
//
// These assert the properties that matter for a tuned/randomized algorithm —
// determinism, invariants, bounds, and the control-interaction math — rather
// than exact sample values (see host/ for the golden C++-vs-Python regression).
// Idioms follow embedded/audio-DSP practice: test on the host, guard NaN/Inf and
// signal bounds, check level/energy invariants, and sweep sample rate + spread.

#define CATCH_CONFIG_MAIN
#include "catch_amalgamated.hpp"

#include <vector>

#include "test_support.h"

using testutil::make_source;
using testutil::render;
using testutil::render_rate_limited;
using testutil::rms;

namespace {

// Configure a sequencer in place (Sequencer is non-copyable: its voices hold
// atomics). `src` must outlive it — it holds a Source*.
// Configure a sequencer. `fade` is the crossfade overlap fraction (0..0.5); a
// value > 0 also enables crossfade. fade 0 = butt-joint (heads sequential, one
// at a time), the default.
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

}  // namespace

TEST_CASE("tables initialize and synthGain is finite and positive") {
  gTab.init();
  REQUIRE(std::isfinite(gTab.synthGain));
  REQUIRE(gTab.synthGain > 0.0f);
}

TEST_CASE("output contains no NaN or Inf") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.5f);
  auto out = render(seq);
  REQUIRE(out.size() > 0);
  for (float v : out) REQUIRE(std::isfinite(v));
}

TEST_CASE("output stays within signal bounds") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.5f);
  auto out = render(seq);
  for (float v : out) REQUIRE(std::abs(v) <= 1.5f);  // generous; clip is downstream
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
  for (size_t i = 0; i < a.size(); i++) REQUIRE(a[i] == Approx(b[i]));
}

// (The former "no click at head boundaries" test is gone by design decision:
// heads are steady-state butt joints and the seam splice is accepted. Interior
// discontinuities are guarded by the click-detector test below.)

TEST_CASE("a genuinely starved ring counts an underrun and stays silent") {
  // gUnderruns / the ring-empty branch in Voice::next() is only reached when a
  // PRIMED, already-sounding voice outruns the worker — the onset lookahead
  // deliberately prevents this in the normal case, and the rate-limited test
  // above proves safe degradation without confirming this exact counter fires.
  // Force it directly: let one head prime normally, then drain next() far past
  // what a full ring (SS_RING samples) can supply with zero further service().
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 1.0f);   // long, isolated head

  uint32_t before = gUnderruns;
  // Prime: run service()+next() together long enough to pass the onset and
  // fill the ring, as a healthy main loop would.
  for (uint32_t i = 0; i < SS_LOOKAHEAD + SS_RING; i++) {
    seq.service();
    float s = seq.next();
    REQUIRE(std::isfinite(s));
  }
  // Now starve it: pure next(), no service() at all, for more samples than any
  // ring depth could possibly cover.
  bool saw_silence_after_starve = false;
  for (uint32_t i = 0; i < SS_RING * 4; i++) {
    float s = seq.next();
    REQUIRE(std::isfinite(s));
    if (gUnderruns > before) saw_silence_after_starve = true;
  }
  REQUIRE(gUnderruns > before);
  REQUIRE(saw_silence_after_starve);
}

TEST_CASE("rate-limited worker degrades to silence, not garbage") {
  // The fully-drained render() cannot see the on-device race: with a starved
  // worker a head's buffer may be empty when the callback reads it. Correct
  // degradation is CLEAN SILENCE (the voice waits, envelope frozen) — never a
  // stale/garbage sample. Verify output under starvation stays bounded and
  // finite, and interiors stay discontinuity-free away from the accepted seams.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  const float sr = 48000, dur = 1.0f;
  Sequencer seq; make_seq(seq, &src, sr, 50.0f, dur, 1.0f);
  uint32_t interval = seq.intervalSamples();
  auto out = render_rate_limited(seq, 2, /*block=*/48, /*fftsPerBlock=*/1);

  auto near_seam = [&](size_t i) {
    for (uint32_t k = 0; k <= 2 * SS_STEPS + 2; k++) {
      int64_t seam = (int64_t)SS_LOOKAHEAD + (int64_t)k * interval;
      if ((int64_t)i > seam - 512 && (int64_t)i < seam + 512) return true;
    }
    return false;
  };

  uint32_t step = (uint32_t)(dur * sr);
  float body_max = 0.0f;
  for (uint32_t i = step / 4 + 1; i < step * 3 / 4; i++)
    body_max = std::max(body_max, std::abs(out[i] - out[i - 1]));
  float interior_max = 0.0f;
  for (size_t i = 1; i < out.size(); i++) {
    REQUIRE(std::isfinite(out[i]));
    if (near_seam(i)) continue;
    interior_max = std::max(interior_max, std::abs(out[i] - out[i - 1]));
  }
  INFO("interior max delta " << interior_max << " vs body " << body_max);
  REQUIRE(interior_max <= body_max * 4.0f + 1e-4f);
}

TEST_CASE("no local discontinuity in head interiors (click detector)") {
  // A click is a jump that is large RELATIVE TO ITS LOCAL CONTEXT. Head SEAMS
  // are excluded: by design heads are steady-state butt joints, and the splice
  // of uncorrelated phase-randomized material at the joint is accepted (it
  // reads as spectral, not transient — design owner's call). This test guards
  // the INTERIOR: nothing inside a head may step discontinuously.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 1.0f);
  uint32_t interval = seq.intervalSamples();
  auto out = render(seq, 2);

  // Seam neighborhoods: audible onsets land at SS_LOOKAHEAD + k*interval, give
  // or take arm-queue latency; exclude a generous margin around each.
  auto near_seam = [&](size_t i) {
    for (uint32_t k = 0; k <= 2 * SS_STEPS + 2; k++) {
      int64_t seam = (int64_t)SS_LOOKAHEAD + (int64_t)k * interval;
      if ((int64_t)i > seam - 512 && (int64_t)i < seam + 512) return true;
    }
    return false;
  };

  const int W = 64;   // neighborhood half-width
  int clicks = 0;
  for (size_t i = W + 1; i + W < out.size(); i++) {
    if (near_seam(i)) continue;
    float d = std::abs(out[i] - out[i - 1]);
    if (d < 5e-3f) continue;            // too small to hear as a click
    float local = 0.0f;
    for (int k = -W; k < W; k++)
      local += std::abs(out[i + k] - out[i + k - 1]);
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

TEST_CASE("crossfade seam has no full-volume-tail click") {
  // Regression guard. The crossfade envelope faded a head OUT over its last
  // `overlap` body samples, but the head then kept sounding a one-hop natural
  // tail at env 1.0 -- jumping back to full volume right after the fade-out
  // completed: an audible click at every seam (measured ~0.22 sample jump, 30x
  // the interior). The fix ends a crossfaded head's audible life at len_ (no
  // tail). Assert the max sample-to-sample jump with crossfade on stays in the
  // interior-normal range, not the click range.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};

  auto max_interior_jump = [](Sequencer& s) {
    auto out = render(s, 2);
    float mx = 0.0f;
    for (size_t i = 48001; i + 48000 < out.size(); i++)   // skip ramp + final fade
      mx = std::max(mx, std::abs(out[i] - out[i - 1]));
    return mx;
  };

  // Baseline: butt-joint (no crossfade). Its max interior jump is the natural
  // sample-to-sample delta of this broadband material, seams included.
  Sequencer butt; make_seq(butt, &src, 48000, 50.0f, 1.0f, /*fade=*/0.0f);
  float base = max_interior_jump(butt);

  // Measured (via this exact render() harness, make_source, dur 1s, stretch 50):
  //   butt-joint (fade 0):  0.0824
  //   crossfade 0.10:       0.0824  (identical)
  //   crossfade 0.25:       0.0964
  //   crossfade 0.50:       0.0877
  // So the crossfade's worst interior jump is at most ~1.17x the butt-joint
  // baseline -- pure material variation, no seam artifact. The full-volume-tail
  // BUG measured 0.22 (~2.7x). Bound = 1.25x baseline: comfortably above the
  // measured 1.17x material spread, comfortably below the 2.7x regression.
  for (float fade : {0.1f, 0.25f, 0.5f}) {
    Sequencer xf; make_seq(xf, &src, 48000, 50.0f, 1.0f, fade);
    float withXfade = max_interior_jump(xf);
    INFO("fade " << fade << ": jump " << withXfade << " vs butt-joint " << base);
    REQUIRE(withXfade < base * 1.25f);
  }
}

TEST_CASE("no per-step volume dips at full spread (end-to-end without dips)") {
  // Spread 1.0 means "joined end to end as sonically possible WITHOUT volume
  // dips": heads carry a short equal-power crossfade at their edges and overlap
  // by exactly that crossfade, so the seam is power-flat and the sustain is
  // level. A full-step envelope (fade spanning the whole duration) fails this —
  // each step audibly swells in and out.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 1.0f);
  auto out = render(seq, 2);

  // Short-window RMS (50 ms) across the interior of the render, skipping the
  // startup ramp and final tail.
  const uint32_t w = 2400;
  uint32_t begin = 48000 / 2, end = (uint32_t)out.size() - 48000;
  float lo = 1e9f, hi = 0.0f;
  for (uint32_t s = begin; s + w < end; s += w / 2) {
    double acc = 0.0;
    for (uint32_t i = s; i < s + w; i++) acc += (double)out[i] * out[i];
    float r = (float)std::sqrt(acc / w);
    lo = std::min(lo, r);
    hi = std::max(hi, r);
  }
  INFO("windowed RMS min " << lo << " max " << hi << " ratio "
       << 20.0 * std::log10(hi / std::max(lo, 1e-9f)) << " dB");
  // Threshold from measurement: a SINGLE continuous head (no seams) shows
  // 8.7 dB of natural windowed-RMS wander — phase-randomized material is
  // noise-like — and the trapezoid-envelope sequence measures 8.2 dB (adds
  // nothing). The full-step sine envelope this test was written to catch
  // measured 30.6 dB. 12 dB separates the two with margin.
  REQUIRE(20.0 * std::log10(hi / std::max(lo, 1e-9f)) < 12.0);
}

TEST_CASE("pattern length: heads sequential, overlapping by the crossfade") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  const float sr = 48000, dur = 4.0f;
  // Quantize with the RUNTIME hop (gTab.activeH = default window / 2), NOT the
  // SS_H macro (SS_W/2 = the buffer-MAX hop). Those differ now that SS_W (buffer
  // max) != the default window; the sequencer quantizes to activeH, so the test
  // must too.
  const uint32_t hop = (uint32_t)gTab.activeH;
  const uint32_t len = ((uint32_t)(dur * sr / hop + 0.5f)) * hop;

  SECTION("no crossfade = butt-joint = end-to-end (SS_STEPS durations)") {
    Sequencer seq; make_seq(seq, &src, sr, 50.0f, dur, /*fade=*/0.0f);
    // interval = full duration, so the pattern is SS_STEPS durations + tail.
    REQUIRE(seq.patternSamples() == SS_STEPS * len + hop);
  }
  SECTION("fade 0.25 = heads overlap a quarter duration") {
    Sequencer seq; make_seq(seq, &src, sr, 50.0f, dur, /*fade=*/0.25f);
    uint32_t interval = (uint32_t)(len * 0.75f);   // (1 - overlap)
    REQUIRE(seq.patternSamples() == (SS_STEPS - 1) * interval + len + hop);
  }
  SECTION("fade 0.5 (max) = heads overlap half a duration") {
    Sequencer seq; make_seq(seq, &src, sr, 50.0f, dur, /*fade=*/0.5f);
    uint32_t interval = (uint32_t)(len * 0.5f);
    REQUIRE(seq.patternSamples() == (SS_STEPS - 1) * interval + len + hop);
  }
}

TEST_CASE("configurable step count: setSteps clamps to [1, SS_STEPS] (#149)") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f);

  REQUIRE(seq.activeSteps == SS_STEPS);      // default = full sequence
  seq.setSteps(3);  REQUIRE(seq.activeSteps == 3);
  seq.setSteps(1);  REQUIRE(seq.activeSteps == 1);
  seq.setSteps(0);  REQUIRE(seq.activeSteps == 1);          // clamp lo (never 0)
  seq.setSteps(-4); REQUIRE(seq.activeSteps == 1);
  seq.setSteps(SS_STEPS + 5); REQUIRE(seq.activeSteps == SS_STEPS);   // clamp hi
}

TEST_CASE("pattern length shrinks with the active step count (#149)") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  const float sr = 48000, dur = 4.0f;
  const uint32_t hop = (uint32_t)gTab.activeH;   // runtime hop, not the SS_H macro
  const uint32_t len = ((uint32_t)(dur * sr / hop + 0.5f)) * hop;

  // Butt-joint (interval = full duration): a K-step pattern is K durations + the
  // one-hop tail, so the pattern length is directly proportional to activeSteps.
  Sequencer seq; make_seq(seq, &src, sr, 50.0f, dur, /*fade=*/0.0f);
  uint32_t interval = seq.intervalSamples();
  REQUIRE(interval == len);
  for (int k = 1; k <= SS_STEPS; k++) {
    seq.setSteps(k);
    REQUIRE(seq.patternSamples() == (uint32_t)(k - 1) * interval + len + hop);
  }
  // A single step is just one duration + tail -- the shortest possible loop.
  seq.setSteps(1);
  REQUIRE(seq.patternSamples() == len + hop);
}

TEST_CASE("a K-step sequence renders and stays bounded (#149)") {
  // End-to-end: with activeSteps < SS_STEPS the sequence still produces clean,
  // in-bounds audio (only the active heads fire; the walk wraps at K).
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.5f);
  seq.setSteps(3);
  auto out = render(seq, 3);   // 3 full 3-step patterns: exercises the wrap at K
  for (float s : out) {
    REQUIRE(std::isfinite(s));
    REQUIRE(s <= 1.0f);
    REQUIRE(s >= -1.0f);
  }
  REQUIRE(rms(out) > 0.0f);        // it actually sounds
}

TEST_CASE("interval = (1 - overlap) * duration; overlap clamped to 0.5") {
  gTab.init();
  auto srcbuf = make_source(1.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  uint32_t hop = (uint32_t)gTab.activeH;   // runtime hop, not the SS_H macro
  uint32_t len = ((uint32_t)(4.0f * 48000 / hop + 0.5f)) * hop;

  // fade off -> interval = full duration (butt-joint, one head at a time).
  Sequencer off; make_seq(off, &src, 48000, 50.0f, 4.0f, /*fade=*/0.0f);
  REQUIRE(off.intervalSamples() == len);

  // fade 0.25 -> interval = 0.75 of a duration.
  Sequencer q; make_seq(q, &src, 48000, 50.0f, 4.0f, /*fade=*/0.25f);
  REQUIRE(q.intervalSamples() == (uint32_t)(len * 0.75f));

  // fade beyond 0.5 is clamped to 0.5 (interval never below half a duration —
  // the ceiling that keeps at most two heads overlapping).
  Sequencer hi; make_seq(hi, &src, 48000, 50.0f, 4.0f, /*fade=*/0.9f);
  Sequencer half; make_seq(half, &src, 48000, 50.0f, 4.0f, /*fade=*/0.5f);
  REQUIRE(hi.intervalSamples() == half.intervalSamples());
  REQUIRE(half.intervalSamples() == (uint32_t)(len * 0.5f));
}

TEST_CASE("level held roughly flat across crossfade (constant-loudness)") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};

  // Equal-power crossfade should hold RMS roughly flat from butt-joint through
  // the maximum overlap — the seams sum to constant power, not silence-dips or
  // level-swells.
  double prev = -1.0;
  for (float fade : {0.0f, 0.15f, 0.3f, 0.5f}) {
    Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 2.0f, fade);
    double r = rms(render(seq));
    REQUIRE(r > 0.0);
    if (prev > 0.0) {
      double db = 20.0 * std::log10(r / prev);
      REQUIRE(std::abs(db) < 3.0);   // crossfade must not swing level wildly
    }
    prev = r;
  }
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

TEST_CASE("max crossfade renders audibly with at most two heads overlapping") {
  // At fade 0.5 (max) consecutive heads overlap by half a duration, so exactly
  // two heads sound through each seam and one through each clean middle — never
  // more. Confirm real audible output (the crossfade path fires) and that it
  // stays finite/bounded; the two-head ceiling is what keeps this off the old
  // multi-head CPU pileup.
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.5f, /*fade=*/0.5f);
  auto out = render(seq, 2);
  REQUIRE(out.size() > 0);

  bool any_audible = false;
  for (float v : out) if (std::abs(v) > 1e-3f) { any_audible = true; break; }
  REQUIRE(any_audible);
  for (float v : out) { REQUIRE(std::isfinite(v)); REQUIRE(std::abs(v) <= 1.5f); }
}

TEST_CASE("drift perturbs position within bounds and stays reproducible") {
  // Every other test hardcodes drift=0 for determinism. Drift is a real,
  // spec'd feature (spec.md: "a fresh random position inside its neighbourhood
  // ... no memory") and had zero coverage. Assert: (1) with drift > 0 and a
  // fixed seed, two renders of the SAME config are still identical (the RNG is
  // seeded, not wall-clock random); (2) a large drift measurably changes output
  // vs. zero drift (it is actually wired in, not a dead no-op); (3) drift is
  // clamped into [0,1] position space (no wraparound/out-of-bounds source read).
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};

  Sequencer seqA; make_seq(seqA, &src, 48000, 50.0f, 0.5f, /*fade=*/1.0f,
                           /*drift=*/0.3f, /*seed=*/42u);
  Sequencer seqB; make_seq(seqB, &src, 48000, 50.0f, 0.5f, /*fade=*/1.0f,
                           /*drift=*/0.3f, /*seed=*/42u);
  auto a = render(seqA);
  auto b = render(seqB);
  REQUIRE(a.size() == b.size());
  for (size_t i = 0; i < a.size(); i++) REQUIRE(a[i] == Approx(b[i]));

  Sequencer seqZero; make_seq(seqZero, &src, 48000, 50.0f, 0.5f, 1.0f,
                              /*drift=*/0.0f, /*seed=*/42u);
  auto zero = render(seqZero);
  bool differs = false;
  for (size_t i = 0; i < std::min(a.size(), zero.size()); i++)
    if (a[i] != zero[i]) { differs = true; break; }
  REQUIRE(differs);

  for (float v : a) REQUIRE(std::isfinite(v));
}

TEST_CASE("fade 0 is butt-joint; fade clamps to [0, 0.5]") {
  gTab.init();
  auto srcbuf = make_source(1.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  uint32_t hop = (uint32_t)gTab.activeH;   // runtime hop, not the SS_H macro
  uint32_t len = ((uint32_t)(2.0f * 48000 / hop + 0.5f)) * hop;

  // fade 0: butt-joint -> interval = full duration (no on/off toggle exists;
  // fade 0 IS the butt-joint case).
  Sequencer off; make_seq(off, &src, 48000, 50.0f, 2.0f, /*fade=*/0.0f);
  REQUIRE(off.intervalSamples() == len);

  // fade out of range high -> clamped to 0.5 (interval never below half a
  // duration, the two-head ceiling).
  Sequencer hi; make_seq(hi, &src, 48000, 50.0f, 2.0f, /*fade=*/5.0f);
  Sequencer half; make_seq(half, &src, 48000, 50.0f, 2.0f, /*fade=*/0.5f);
  REQUIRE(hi.intervalSamples() == half.intervalSamples());
  REQUIRE(half.intervalSamples() == (uint32_t)(len * 0.5f));

  // fade negative -> clamped to 0 (butt-joint).
  Sequencer lo; make_seq(lo, &src, 48000, 50.0f, 2.0f, /*fade=*/0.0f);
  lo.fade = -1.0f;
  REQUIRE(lo.intervalSamples() == len);
}

TEST_CASE("setWindow: window + gain recompute for each frame size") {
  gTab.init();
  // Every supported window round-trips through setWindow with finite gain and a
  // correctly-sized window (log2 matches). SS_W is the buffer max.
  for (int w = 64; w <= SS_W; w <<= 1) {
    gTab.setWindow(w);
    REQUIRE(gTab.activeW == w);
    REQUIRE(gTab.activeH == w / 2);
    REQUIRE((1 << gTab.activePasses) == w);          // passes = log2(w)
    REQUIRE(std::isfinite(gTab.synthGain));
    REQUIRE(gTab.synthGain > 0.0f);
    // gWindow is a raised (1-x^2)^1.25 curve: 0 at the edges, 1 at the center.
    // (Split out of StretchTables into a global so the device can place it in
    // AXI SRAM; setWindow builds it.)
    REQUIRE(gWindow[0] == Approx(0.0f).margin(1e-3));
    REQUIRE(gWindow[w / 2] == Approx(1.0f).margin(1e-2));
  }
  gTab.setWindow(SS_W);   // restore default for other tests
}

TEST_CASE("renders cleanly at every frame size") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  for (int w = 512; w <= SS_W; w <<= 1) {
    Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.25f);
    seq.setFrame(w);
    REQUIRE(seq.frameSize == w);
    auto out = render(seq, 2);
    REQUIRE(out.size() > 0);
    bool audible = false;
    for (float v : out) {
      REQUIRE(std::isfinite(v));
      REQUIRE(std::abs(v) <= 1.5f);
      if (std::abs(v) > 1e-3f) audible = true;
    }
    REQUIRE(audible);   // real output at this window, not silence
  }
  gTab.setWindow(SS_W);
}

TEST_CASE("changing frame size mid-render does not corrupt a sounding voice") {
  // Regression: voices snapshot the frame SIZE at start() but used to read the
  // shared window curve + synthGain LIVE. A setFrame() while a voice was mid-
  // render rebuilt gWindow/synthGain for a NEW size, so the sounding voice
  // multiplied its w_-sample frame by a curve/gain built for a different size --
  // a volume jump + broadband noise on every fast frame-size scroll (heard on
  // the bench). The fix snapshots the window curve + gain per voice at start().
  // Here we hammer setFrame across sizes WHILE rendering and require the output
  // stays finite, bounded, and free of a sudden energy spike.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.25f);
  seq.setFrame(SS_W);   // start at the largest window (idx 0)

  const int sizes[] = {SS_W, 512, 2048, 256, 1024, SS_W};
  std::vector<float> out;
  const uint32_t total = seq.patternSamples();   // ~one pattern of churn
  int si = 0;
  for (uint32_t n = 0; n < total; n++) {
    // Change the frame size aggressively mid-flight (every ~1/64 of the pattern),
    // exactly the fast-scroll case that exposed the bug. A voice that snapshots
    // its window at start() renders self-consistently through all of these; a
    // voice reading the shared window LIVE would index gWindow[] past the
    // just-rebuilt (smaller) size and scale by a mismatched gain.
    if (n % (total / 64 + 1) == 0) { seq.setFrame(sizes[si % 6]); si++; }
    for (int g = 0; g < 64 && seq.service(); g++) {}
    float v = seq.next();
    REQUIRE(std::isfinite(v));       // no NaN/Inf from a garbage window read
    REQUIRE(std::abs(v) <= 1.0f);    // stays inside the sequencer's hard clamp
    out.push_back(v);
  }
  REQUIRE(rms(out) > 0.0f);          // still sounding through all the churn
  gTab.setWindow(SS_W);
}

TEST_CASE("frame size is clamped to [64, SS_W]") {
  gTab.init();
  gTab.setWindow(1);       // below min
  REQUIRE(gTab.activeW == 64);
  gTab.setWindow(1 << 20); // above max
  REQUIRE(gTab.activeW == SS_W);
  gTab.setWindow(SS_W);
}
