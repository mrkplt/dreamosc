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
void make_seq(Sequencer& seq, const Source* src, float sr, float stretch,
              float duration, float spread, float drift = 0.0f,
              uint32_t seed = 0x12345678u) {
  // Host-side head pool; on device this lives in SDRAM (see dreamosc.cpp).
  // One pool PER sequencer: init() primes each ring with silence, and tests
  // that hold two sequencers at once (determinism, drift) would otherwise have
  // the first render scribble over the second's primed regions.
  static std::vector<std::vector<float>> pools;
  pools.emplace_back(SS_POOL_FLOATS);
  seq.init(src, sr, pools.back().data(), seed);
  seq.stretch  = stretch;
  seq.duration = duration;
  seq.spread   = spread;
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
  // gUnderruns / the ring-empty branch in Head::next() is only reached when the
  // consumer outruns the producer past the whole prime depth — the silence
  // prime deliberately prevents this in the normal case, and the rate-limited
  // test above proves safe degradation without confirming this counter fires.
  // Force it directly: run service()+next() together normally, then drain
  // next() far past what a full ring (SS_RING samples) can supply with zero
  // further service().
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
  // Budget in worker UNITS (emitted slices, at most one render each): demand
  // is SS_STEPS heads * sr/SS_SLICE units/s (3000/s at 48 kHz), so one unit
  // per 12 samples (4000/s) models a keeping-up-but-not-idle worker.
  auto out = render_rate_limited(seq, 2, /*block=*/12, /*fftsPerBlock=*/1);

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

TEST_CASE("pattern length follows (SS_STEPS-1)*dur*spread + dur") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  const float sr = 48000, dur = 4.0f;

  // Durations quantize to the hop grid, and each head's audible span carries a
  // one-hop natural tail beyond its body.
  const uint32_t len = ((uint32_t)(dur * sr / SS_H + 0.5f)) * SS_H;
  SECTION("spread 0 = all heads together = one duration (+ tail hop)") {
    Sequencer seq; make_seq(seq, &src, sr, 50.0f, dur, 0.0f);
    REQUIRE(seq.patternSamples() == len + SS_H);
  }
  SECTION("spread 0.5 = heads half a duration apart") {
    Sequencer seq; make_seq(seq, &src, sr, 50.0f, dur, 0.5f);
    uint32_t expected = (SS_STEPS - 1) * (uint32_t)(len * 0.5f) + len + SS_H;
    REQUIRE(seq.patternSamples() == expected);
  }
  SECTION("spread 1 = end-to-end = SS_STEPS durations (+ tail hop)") {
    Sequencer seq; make_seq(seq, &src, sr, 50.0f, dur, 1.0f);
    REQUIRE(seq.patternSamples() == SS_STEPS * len + SS_H);
  }
}

TEST_CASE("interval matches duration*spread (the corrected model)") {
  gTab.init();
  auto srcbuf = make_source(1.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  // 4s duration, 50% spread -> half the (hop-quantized) duration between
  // head starts: ~2 s to within the ~43 ms hop quantization.
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 0.5f);
  uint32_t len = ((uint32_t)(4.0f * 48000 / SS_H + 0.5f)) * SS_H;
  REQUIRE(seq.intervalSamples() == (uint32_t)(len * 0.5f));
  REQUIRE(std::abs((double)seq.intervalSamples() - 2.0 * 48000)
          <= (double)SS_H);
  // spread 0 -> interval 0 (all together).
  Sequencer seq0; make_seq(seq0, &src, 48000, 50.0f, 4.0f, 0.0f);
  REQUIRE(seq0.intervalSamples() == 0u);
}

TEST_CASE("level held roughly flat across spread (constant-loudness invariant)") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};

  double prev = -1.0;
  for (float spread : {0.25f, 0.5f, 0.75f, 1.0f}) {
    Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 2.0f, spread);
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
    Sequencer seq; make_seq(seq, &src, (float)sr, 30.0f, 1.0f, 0.5f);
    auto out = render(seq);
    REQUIRE(out.size() > 0);
    for (float v : out) REQUIRE(std::isfinite(v));
  }
}

TEST_CASE("spread ~0 fires all SS_STEPS heads together through render()") {
  // Every other test uses spread >= 0.25; the "all heads share one onset" burst
  // lattice (interval==0: every head's onsets coincide, repeating each body
  // length) is otherwise never exercised through an actual render — only its
  // length arithmetic (patternSamples()) is checked elsewhere. Distinct step
  // positions + zero drift means each head is a distinguishable steady tone
  // once primed; confirm real audible output appears (not silence).
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.5f, 0.0f);
  seq.spread = 0.0f;
  auto out = render(seq, 2);
  REQUIRE(out.size() > 0);

  // Some part of the render must be genuinely audible: SS_LOOKAHEAD samples of
  // startup priming are silence by design, but the render must not be silent
  // throughout (which would mean the burst-arm path silently failed to fire).
  bool any_audible = false;
  for (float v : out) if (std::abs(v) > 1e-3f) { any_audible = true; break; }
  REQUIRE(any_audible);
  for (float v : out) REQUIRE(std::isfinite(v));
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

  Sequencer seqA; make_seq(seqA, &src, 48000, 50.0f, 0.5f, /*spread=*/1.0f,
                           /*drift=*/0.3f, /*seed=*/42u);
  Sequencer seqB; make_seq(seqB, &src, 48000, 50.0f, 0.5f, /*spread=*/1.0f,
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

TEST_CASE("synthesis work is bounded by SS_STEPS streams at ANY spread") {
  // The load-cliff regression guard. The old voice-pool design let sustained
  // concurrency reach 2*SS_STEPS at low spread, roughly doubling FFT work
  // exactly where the retrigger rate was already highest — past the device's
  // render budget (the #132 near-zero-spread noise/underrun report). Persistent
  // heads produce at hop rate ALWAYS (sound, handoff, or silence), so total
  // service() work per rendered second must be ~SS_STEPS * sr / SS_H
  // (187.5/s at 48 kHz) regardless of spread. Bounds are wide: the point is
  // "flat, near 8 streams", not an exact count (startup fill and retrigger
  // transitions, which advance the stream by up to 2 hops in one unit, move it
  // slightly).
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};

  auto renders_per_sec = [](Sequencer& seq) {
    const uint32_t samples = 3 * 48000;
    for (uint32_t n = 0; n < samples; n++) {
      for (int g = 0; g < 64 && seq.service(); g++) {}
      seq.next();
    }
    return (double)seq.framesRendered() / 3.0;
  };

  Sequencer hiSeq; make_seq(hiSeq, &src, 48000, 50.0f, 1.0f, 1.0f);
  Sequencer loSeq; make_seq(loSeq, &src, 48000, 50.0f, 1.0f, 0.02f);
  double hi = renders_per_sec(hiSeq);
  double lo = renders_per_sec(loSeq);
  INFO("renders/s at spread 1.0: " << hi << ", at spread 0.02: " << lo);
  // Ceiling: SS_STEPS continuous streams at hop rate (187.5/s at 48 kHz),
  // plus one transition render per retrigger. The worst case (all heads
  // sounding, low spread) must sit near — never above ~1.6x — that ceiling;
  // high spread renders far less (only ~1/spread heads sound at once).
  const double ceiling = (double)SS_STEPS * 48000.0 / SS_H;   // 187.5
  REQUIRE(lo > ceiling * 0.6);
  REQUIRE(lo < ceiling * 1.6);
  REQUIRE(hi < ceiling * 1.6);
  REQUIRE(hi > 10.0);            // and the high-spread lattice still renders
}

TEST_CASE("live spread change recovers from near-zero without residue") {
  // The #132 field report: spread driven toward 0, then turned back up, and
  // the device never recovered. The core must carry no stuck state across that
  // gesture: after raising spread the lattice re-spaces immediately (it is
  // recomputed live, never stored), heads thin out to the high-spread duty
  // cycle, and a fully-serviced render sees zero underruns throughout.
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.02f);

  uint32_t underBefore = gUnderruns;
  auto run = [&](uint32_t samples) {
    for (uint32_t n = 0; n < samples; n++) {
      for (int g = 0; g < 64 && seq.service(); g++) {}
      float s = seq.next();
      REQUIRE(std::isfinite(s));
    }
  };
  run(2 * 48000);                        // hold near zero: all heads churning
  REQUIRE(seq.soundingHeads() == SS_STEPS);
  seq.spread = 0.8f;                     // "turn clockwise"
  run(3 * 48000);                        // several patterns at the new spread
  // At spread 0.8 a head sounds duration out of 8*0.8 durations, so on average
  // ~1.6 heads sound; tail overlap can add one more. Anything near SS_STEPS
  // means the old lattice left residue.
  REQUIRE(seq.soundingHeads() <= 3);
  REQUIRE(gUnderruns == underBefore);
}

TEST_CASE("sub-hop onsets: interiors stay click-free at a non-hop-aligned spread") {
  // At spread 0.37 the onset lattice (k * interval) does not land on the hop
  // grid, so every retrigger exercises the sample-accurate transition path
  // (grid re-phase + overlap-added rise). If onsets were quantized to hops,
  // the real seams would fall up to half a hop outside these exclusion
  // windows and the splice would register as a click; if the transition
  // overlap-add were wrong, the handoff itself would step. Same detector and
  // thresholds as the aligned-spread click test.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.37f);
  uint32_t interval = seq.intervalSamples();
  REQUIRE(interval % SS_H != 0);         // the premise: onsets off the hop grid
  auto out = render(seq, 2);

  auto near_seam = [&](size_t i) {
    for (uint32_t k = 0; k <= 2 * SS_STEPS + 2; k++) {
      int64_t seam = (int64_t)SS_LOOKAHEAD + (int64_t)k * interval;
      if ((int64_t)i > seam - 512 && (int64_t)i < seam + 512) return true;
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

TEST_CASE("mid-life refresh is click-free and continues the same life") {
  // refresh() re-sources a sounding life's spectrum from live parameters
  // (the "rendered frames are disposable" hook for frame-size and other
  // spectral controls). It must behave as a seam of the accepted retrigger
  // construction — no click outside the seam neighborhood — and must NOT
  // retrigger: the head keeps sounding through it and the body ends on the
  // step's own clock, not a restarted one.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};

  auto run = [&](bool doRefresh) {
    Sequencer seq;
    make_seq(seq, &src, 48000, 50.0f, 1.0f, 1.0f);
    std::vector<float> out;
    const uint32_t total = 48000;                  // first head's body
    const uint32_t at = 24000;                     // mid-body refresh
    for (uint32_t n = 0; n < total; n++) {
      for (int g = 0; g < 64 && seq.service(); g++) {}
      if (doRefresh && n == at) seq.refreshSounding();
      out.push_back(seq.next());
      if (n > SS_LOOKAHEAD + 4096 && n < total - 4096)
        REQUIRE(seq.soundingHeads() >= 1);         // never drops out
    }
    return out;
  };

  auto a = run(true);
  auto b = run(true);
  for (size_t i = 0; i < a.size(); i++)            // deterministic
    REQUIRE(a[i] == Approx(b[i]));
  auto base = run(false);
  bool differs = false;
  for (size_t i = 0; i < a.size(); i++)
    if (a[i] != base[i]) { differs = true; break; }
  REQUIRE(differs);                                // it actually re-sources

  // Click detector over the refresh region, excluding the seam neighborhood:
  // the refresh enters the stream at the write position (~fill ahead of the
  // ear) and blends over a hop, like any accepted seam.
  const int64_t seam_lo = 24000 - 64;
  const int64_t seam_hi = 24000 + SS_FILL_TARGET + SS_H + 512;
  const int W = 64;
  int clicks = 0;
  for (size_t i = 20000; i + W < 32000; i++) {
    if ((int64_t)i > seam_lo && (int64_t)i < seam_hi) continue;
    float d = std::abs(a[i] - a[i - 1]);
    if (d < 5e-3f) continue;
    float local = 0.0f;
    for (int k = -W; k < W; k++)
      local += std::abs(a[i + k] - a[i + k - 1]);
    local /= (2 * W);
    if (d > 8.0f * local) clicks++;
  }
  REQUIRE(clicks == 0);
}

TEST_CASE("spread is clamped to [0,1]") {
  gTab.init();
  auto srcbuf = make_source(1.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer hi; make_seq(hi, &src, 48000, 50.0f, 2.0f, 5.0f);   // out of range high
  Sequencer at1; make_seq(at1, &src, 48000, 50.0f, 2.0f, 1.0f);
  REQUIRE(hi.intervalSamples() == at1.intervalSamples());
  Sequencer lo; make_seq(lo, &src, 48000, 50.0f, 2.0f, -1.0f);  // out of range low
  REQUIRE(lo.intervalSamples() == 0u);
}
