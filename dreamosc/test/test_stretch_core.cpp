// Unit tests for the platform-free DSP core (stretch_core.h).
//
// These assert the properties that matter for a tuned/randomized algorithm —
// determinism, invariants, bounds, and the control-interaction math — rather
// than exact sample values (see host/ for the golden C++-vs-Python regression).
// Idioms follow embedded/audio-DSP practice: test on the host, guard NaN/Inf and
// signal bounds, check level/energy invariants, and sweep sample rate + spread.

#define CATCH_CONFIG_MAIN
#include "catch_amalgamated.hpp"

#include "test_support.h"

using testutil::make_source;
using testutil::render;
using testutil::render_rate_limited;
using testutil::rms;

namespace {

// Configure a sequencer with default positions. `src` must outlive the returned
// Sequencer (it holds a Source*), which is why callers keep the Source local.
Sequencer make_seq(const Source* src, float sr, float stretch, float duration,
                   float spread, float drift = 0.0f, uint32_t seed = 0x12345678u) {
  Sequencer seq;
  seq.init(src, sr, seed);
  seq.stretch  = stretch;
  seq.duration = duration;
  seq.spread   = spread;
  for (int i = 0; i < SS_STEPS; i++) seq.drift[i] = drift;
  return seq;
}

}  // namespace

TEST_CASE("tables initialize and olaGain is finite and positive") {
  gTab.init();
  REQUIRE(std::isfinite(gTab.olaGain));
  REQUIRE(gTab.olaGain > 0.0f);
}

TEST_CASE("output contains no NaN or Inf") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  auto seq = make_seq(&src, 48000, 50.0f, 1.0f, 0.5f);
  auto out = render(seq);
  REQUIRE(out.size() > 0);
  for (float v : out) REQUIRE(std::isfinite(v));
}

TEST_CASE("output stays within signal bounds") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  auto seq = make_seq(&src, 48000, 50.0f, 1.0f, 0.5f);
  auto out = render(seq);
  for (float v : out) REQUIRE(std::abs(v) <= 1.5f);  // generous; clip is downstream
}

TEST_CASE("deterministic: same config renders identically") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  auto seqA = make_seq(&src, 48000, 50.0f, 1.0f, 0.5f);
  auto seqB = make_seq(&src, 48000, 50.0f, 1.0f, 0.5f);
  auto a = render(seqA);
  auto b = render(seqB);
  REQUIRE(a.size() == b.size());
  for (size_t i = 0; i < a.size(); i++) REQUIRE(a[i] == Approx(b[i]));
}

TEST_CASE("no click at head boundaries (transient-free by construction)") {
  // The instrument is phase-randomized and transient-free, so the output must
  // not jump at a step boundary. The failure this guards: the power
  // normalization (sum/sqrt(power)) divides the per-voice fade out when only one
  // voice sounds, so a head ends at full amplitude and snaps to zero at the
  // handoff — a click every step. We assert the max sample-to-sample delta over
  // the whole render is not wildly larger than the in-body steady-state delta.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  const float sr = 48000, dur = 1.0f;
  auto seq = make_seq(&src, sr, 50.0f, dur, 1.0f);   // spread 1: one voice at a time
  auto out = render(seq, 2);

  // Steady-state reference: largest adjacent delta in the middle of the FIRST
  // head, away from any boundary.
  uint32_t step = (uint32_t)(dur * sr);
  float body_max = 0.0f;
  for (uint32_t i = step / 4 + 1; i < step * 3 / 4; i++)
    body_max = std::max(body_max, std::abs(out[i] - out[i - 1]));

  // Largest adjacent delta anywhere (will land on a boundary if one clicks).
  float overall_max = 0.0f;
  uint32_t at = 0;
  for (uint32_t i = 1; i < out.size(); i++) {
    float d = std::abs(out[i] - out[i - 1]);
    if (d > overall_max) { overall_max = d; at = i; }
  }
  INFO("overall max delta " << overall_max << " at sample " << at
       << " vs body max " << body_max);
  REQUIRE(overall_max <= body_max * 4.0f + 1e-4f);
}

TEST_CASE("no onset click under a rate-limited worker (on-device race)") {
  // The fully-drained render() cannot see the hardware bug: at a head's onset the
  // worker has not filled its buffer, so the callback reads it cold and clicks.
  // Reproduce it by rate-limiting service() to a small FFT budget per block, then
  // require the seam to stay continuous — a correct ready-gate keeps an unready
  // voice silent rather than reading a cold buffer.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  const float sr = 48000, dur = 1.0f;
  auto seq = make_seq(&src, sr, 50.0f, dur, 1.0f);
  // 1 FFT per 48-sample block ~= 1000 FFT/s, far above the ~23/s one voice needs
  // in steady state but NOT enough to fill a cold buffer instantly at onset.
  auto out = render_rate_limited(seq, 2, /*block=*/48, /*fftsPerBlock=*/1);

  uint32_t step = (uint32_t)(dur * sr);
  float body_max = 0.0f;
  for (uint32_t i = step / 4 + 1; i < step * 3 / 4; i++)
    body_max = std::max(body_max, std::abs(out[i] - out[i - 1]));
  float overall_max = 0.0f; uint32_t at = 0;
  for (uint32_t i = 1; i < out.size(); i++) {
    float d = std::abs(out[i] - out[i - 1]);
    if (d > overall_max) { overall_max = d; at = i; }
  }
  INFO("rate-limited max delta " << overall_max << " at " << at
       << " vs body " << body_max);
  REQUIRE(overall_max <= body_max * 4.0f + 1e-4f);
}

TEST_CASE("pattern length follows (SS_STEPS-1)*dur*spread + dur") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  const float sr = 48000, dur = 4.0f;

  SECTION("spread 0 = all heads together = one duration") {
    auto seq = make_seq(&src, sr, 50.0f, dur, 0.0f);
    REQUIRE(seq.patternSamples() == Approx((uint32_t)(dur * sr)).margin(2));
  }
  SECTION("spread 0.5 = heads dur*0.5 apart") {
    auto seq = make_seq(&src, sr, 50.0f, dur, 0.5f);
    uint32_t expected = (uint32_t)((SS_STEPS - 1) * dur * sr * 0.5f + dur * sr);
    REQUIRE(seq.patternSamples() == Approx(expected).margin(2));
  }
  SECTION("spread 1 = end-to-end = SS_STEPS durations") {
    auto seq = make_seq(&src, sr, 50.0f, dur, 1.0f);
    REQUIRE(seq.patternSamples() == Approx((uint32_t)(SS_STEPS * dur * sr)).margin(2));
  }
}

TEST_CASE("interval matches duration*spread (the corrected model)") {
  gTab.init();
  auto srcbuf = make_source(1.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  // 4s duration, 50% spread -> 2s between head starts.
  auto seq = make_seq(&src, 48000, 50.0f, 4.0f, 0.5f);
  REQUIRE(seq.intervalSamples() == Approx((uint32_t)(2.0f * 48000)).margin(2));
  // spread 0 -> interval 0 (all together).
  auto seq0 = make_seq(&src, 48000, 50.0f, 4.0f, 0.0f);
  REQUIRE(seq0.intervalSamples() == 0u);
}

TEST_CASE("level held roughly flat across spread (constant-loudness invariant)") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};

  double prev = -1.0;
  for (float spread : {0.25f, 0.5f, 0.75f, 1.0f}) {
    auto seq = make_seq(&src, 48000, 50.0f, 2.0f, spread);
    double r = rms(render(seq));
    REQUIRE(r > 0.0);
    if (prev > 0.0) {
      double db = 20.0 * std::log10(r / prev);
      REQUIRE(std::abs(db) < 3.0);   // spread must not swing level wildly
    }
    prev = r;
  }
}

TEST_CASE("renders at multiple sample rates without blowing up") {
  gTab.init();
  for (uint32_t sr : {44100u, 48000u, 96000u}) {
    auto srcbuf = make_source(1.5f, sr);
    Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
    auto seq = make_seq(&src, (float)sr, 30.0f, 1.0f, 0.5f);
    auto out = render(seq);
    REQUIRE(out.size() > 0);
    for (float v : out) REQUIRE(std::isfinite(v));
  }
}

TEST_CASE("spread is clamped to [0,1]") {
  gTab.init();
  auto srcbuf = make_source(1.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  auto hi = make_seq(&src, 48000, 50.0f, 2.0f, 5.0f);   // out of range high
  auto at1 = make_seq(&src, 48000, 50.0f, 2.0f, 1.0f);
  REQUIRE(hi.intervalSamples() == at1.intervalSamples());
  auto lo = make_seq(&src, 48000, 50.0f, 2.0f, -1.0f);  // out of range low
  REQUIRE(lo.intervalSamples() == 0u);
}
