// Unit tests for the platform-free DSP core (stretch_core.h).
//
// These assert the properties that matter for a tuned/randomized algorithm --
// determinism, invariants, bounds, and the control-interaction math -- rather
// than exact sample values. Idioms follow embedded/audio-DSP practice: test on
// the host, guard NaN/Inf and signal bounds, check level/energy invariants, and
// sweep sample rate + fade.
//
// #155 (persistent-head): the sequencer is now a robotic READ clock (dwell =
// round(duration*sr), NO hop quantization) that owns the seam crossfade; each
// head is a persistent per-step writer. Tests that asserted the old hop-
// quantized lenSamples()/intervalSamples()/patternSamples() model are rewritten
// against the live-duration model, with a written reason on each. The click-
// detector and constant-loudness GUARDS are preserved unchanged in intent.

#define CATCH_CONFIG_MAIN
#include "catch_amalgamated.hpp"

#include <vector>

#include "test_support.h"

using testutil::make_source;
using testutil::render;
using testutil::render_rate_limited;
using testutil::rms;

namespace {

// Configure a sequencer in place (Sequencer is non-copyable: its heads hold
// atomics). `src` must outlive it -- it holds a Source*. `fade` is the crossfade
// overlap fraction (0..0.5); fade 0 = raw cut (heads sequential, one at a time).
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

// --- seamPhase: the pure seam-decision logic, tested directly ---------------

TEST_CASE("seamPhase: fade 0 is a raw cut (onset == end == duration)") {
  // With no crossfade, the incoming head goes live exactly as the outgoing head
  // ends: goLive and end fire the same sample, and there is no overlap window.
  uint32_t dur = 48000;
  SeamDecision at_end = seamPhase(dur, dur, 0.0f, 1024);
  REQUIRE(at_end.goLive);
  REQUIRE(at_end.end);
  REQUIRE(at_end.fadeLen == 0);
  // Before the end, neither goLive nor end.
  SeamDecision mid = seamPhase(dur / 2, dur, 0.0f, 1024);
  REQUIRE_FALSE(mid.goLive);
  REQUIRE_FALSE(mid.end);
}

TEST_CASE("seamPhase: fade > 0 opens the seam fade*duration before the end") {
  uint32_t dur = 48000;
  float fade = 0.25f;
  uint32_t fadeLen = (uint32_t)(dur * fade);       // 12000
  uint32_t onset = dur - fadeLen;                  // 36000
  SeamDecision go = seamPhase(onset, dur, fade, 1024);
  REQUIRE(go.goLive);
  REQUIRE_FALSE(go.end);
  REQUIRE(go.fadeLen == fadeLen);
  // End still fires at the full duration, so the two overlap for fadeLen samples.
  SeamDecision e = seamPhase(dur, dur, fade, 1024);
  REQUIRE(e.end);
}

TEST_CASE("seamPhase: pre-warm fires one lookahead before onset (clamped at 0)") {
  uint32_t dur = 48000, look = 16384;
  // Long dwell: prewarm point is onset - lookahead, comfortably positive.
  uint32_t onset = dur;                            // fade 0
  REQUIRE(seamPhase(onset - look, dur, 0.0f, look).prewarm);
  REQUIRE_FALSE(seamPhase(onset - look - 1, dur, 0.0f, look).prewarm);
  // Short dwell (dur < lookahead): prewarm clamps to elapsed 0.
  uint32_t shortDur = 8000;
  REQUIRE(seamPhase(0, shortDur, 0.0f, look).prewarm);
}

TEST_CASE("seamPhase: fade clamps to [0, 0.5]") {
  uint32_t dur = 48000;
  REQUIRE(seamPhase(dur, dur, -1.0f, 1024).fadeLen == 0);          // clamp lo
  REQUIRE(seamPhase(0, dur, 5.0f, 1024).fadeLen == (uint32_t)(dur * 0.5f));  // hi
}

// --- render-level invariants ------------------------------------------------

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

TEST_CASE("a genuinely starved ring counts an underrun and stays silent") {
  // gUnderruns / the ring-empty branch in Head::next() is reached when a GATED,
  // already-sounding head outruns the worker. Force it: prime a head, then drain
  // next() far past what a full cushion (SS_RING) can supply with zero service().
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 0.0f);   // long, isolated

  uint32_t before = gUnderruns;
  // Prime: run service()+next() together long enough to pass the onset lookahead
  // and fill the cushion, as a healthy main loop would.
  for (uint32_t i = 0; i < SS_LOOKAHEAD + SS_RING; i++) {
    seq.service();
    float s = seq.next();
    REQUIRE(std::isfinite(s));
  }
  // Now starve it: pure next(), no service(), for more than any cushion depth.
  for (uint32_t i = 0; i < SS_RING * 4; i++) {
    float s = seq.next();
    REQUIRE(std::isfinite(s));
  }
  REQUIRE(gUnderruns > before);
}

TEST_CASE("no local discontinuity in head interiors (click detector)") {
  // A click is a jump large RELATIVE TO ITS LOCAL CONTEXT. Head SEAMS are
  // excluded: heads are steady-state and the splice of uncorrelated phase-
  // randomized material at a raw-cut seam is accepted (spectral, not transient --
  // design owner's call, #155). This GUARD (preserved from the old suite) checks
  // the INTERIOR: nothing inside a head may step discontinuously. Seams now land
  // at SS_LOOKAHEAD + k*durSamples (unquantized duration).
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.0f);
  uint32_t dur = (uint32_t)(1.0f * 48000 + 0.5f);
  auto out = render(seq, 2);

  auto near_seam = [&](size_t i) {
    for (uint32_t k = 0; k <= 2 * SS_STEPS + 2; k++) {
      int64_t seam = (int64_t)SS_LOOKAHEAD + (int64_t)k * dur;
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

TEST_CASE("level held roughly flat across crossfade (constant-loudness)") {
  // Equal-power crossfade should hold RMS roughly flat from raw cut through the
  // maximum overlap -- the seams sum to constant power, not silence-dips or
  // level-swells. GUARD preserved from the old suite.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};

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

TEST_CASE("no per-step volume dips at max fade (end-to-end without dips)") {
  // fade 0.5: heads overlap half a duration under an equal-power crossfade, so
  // the seam is power-flat and the sustain is level (no per-step swell). Short-
  // window RMS across the interior must stay within the noise-like material's own
  // wander, not swing like a full-step envelope would. GUARD preserved.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.5f);
  auto out = render(seq, 2);

  const uint32_t w = 2400;
  uint32_t begin = 48000, end = (uint32_t)out.size() - 48000;
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
  // A single continuous head shows ~8.7 dB of natural windowed-RMS wander (noise-
  // like material); the full-step sine envelope this guards against measured
  // 30.6 dB. 12 dB separates them with margin.
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

TEST_CASE("max crossfade renders audibly and bounded (<=3 heads render, <=2 gated)") {
  // At fade 0.5 consecutive heads overlap by half a duration, so two heads are
  // gated through each seam (a third is pre-warming, not yet gated) -- the
  // concurrency ceiling #155 is built around. Confirm audible, finite, bounded.
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.5f, 0.5f);
  auto out = render(seq, 2);
  REQUIRE(out.size() > 0);

  bool any_audible = false;
  for (float v : out) if (std::abs(v) > 1e-3f) { any_audible = true; break; }
  REQUIRE(any_audible);
  for (float v : out) { REQUIRE(std::isfinite(v)); REQUIRE(std::abs(v) <= 1.5f); }
}

// --- the robotic clock: live duration, fast-march, degrade -----------------

TEST_CASE("duration is live and unquantized (frame size does not bend timing)") {
  // #155: durSamples() = round(duration*sr), independent of frame size. The old
  // model quantized to the active hop, so a 16384 window snapped a 0.25 s step to
  // 0.17 s. Now the step length is the same at any frame size.
  gTab.init();
  auto srcbuf = make_source(1.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.25f, 0.0f);

  uint32_t at4096 = seq.durSamples();
  seq.setFrame(16384);
  uint32_t at16384 = seq.durSamples();
  REQUIRE(at4096 == at16384);                 // frame size no longer bends it
  REQUIRE(at4096 == (uint32_t)(0.25f * 48000 + 0.5f));   // exact, unquantized
}

TEST_CASE("duration crank-down mid-dwell fast-marches without permanent silence") {
  // The marquee #155 behavior: a minute-long dwell with the duration knob live.
  // Crank duration DOWN mid-dwell and the sequence must advance (fast-march), not
  // stay frozen for the rest of the old 60 s dwell, and must SELF-HEAL to audible
  // output (a one-time cold-arrival gap on the crank is accepted for stage 1, but
  // it must not go permanently silent or emit non-finite samples / double-fire).
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 60.0f, 0.0f);

  std::vector<float> out;
  const uint32_t total = 48000 * 3;
  for (uint32_t n = 0; n < total; n++) {
    for (int g = 0; g < 64 && seq.service(); g++) {}
    if (n == 14400) seq.duration = 0.25f;     // crank down mid-dwell (~0.3 s in)
    float v = seq.next();
    REQUIRE(std::isfinite(v));
    out.push_back(v);
  }
  // After the crank + a generous settle, the sequence is audibly marching again
  // (not stuck silent). Measure RMS of the last second.
  double tail = rms(std::vector<float>(out.end() - 48000, out.end()));
  INFO("tail rms after crank-down " << tail);
  REQUIRE(tail > 0.01);                        // self-healed to real output
}

TEST_CASE("steady short-duration march stays fed (cushion keeps up)") {
  // A steady fast-march (0.25 s dwell) pre-warms each next head at dwell start and
  // must keep its cushion fed -- so after warmup the output is essentially never
  // silent. This is the stage-1 pre-warm path working under a fast walk.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.25f, 0.0f);

  std::vector<float> out;
  const uint32_t total = 48000 * 3;
  for (uint32_t n = 0; n < total; n++) {
    for (int g = 0; g < 64 && seq.service(); g++) {}
    out.push_back(seq.next());
  }
  int silent = 0, counted = 0;
  for (uint32_t n = 48000; n < total; n++) {   // after 1 s warmup
    counted++;
    if (std::abs(out[n]) < 1e-5f) silent++;
  }
  INFO("silence fraction after warmup " << (100.0 * silent / counted) << "%");
  REQUIRE(silent < counted / 20);              // < 5% silent: cushion keeps up
}

TEST_CASE("activeSteps == 1 re-arms the single head each dwell (#149 degrade)") {
  // With one active step, next_ == cur_: pre-warm is skipped (it would reset the
  // ring the ISR is reading) and the head hard re-arms at end. It must still
  // produce finite, bounded, audible output across several dwells.
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.5f, 0.0f);
  seq.setSteps(1);
  auto out = render(seq, 3);
  for (float v : out) { REQUIRE(std::isfinite(v)); REQUIRE(std::abs(v) <= 1.0f); }
  REQUIRE(rms(out) > 0.0f);                     // it sounds
}

TEST_CASE("configurable step count: setSteps clamps to [1, SS_STEPS] (#149)") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f);

  REQUIRE(seq.activeSteps == SS_STEPS);
  seq.setSteps(3);  REQUIRE(seq.activeSteps == 3);
  seq.setSteps(1);  REQUIRE(seq.activeSteps == 1);
  seq.setSteps(0);  REQUIRE(seq.activeSteps == 1);
  seq.setSteps(-4); REQUIRE(seq.activeSteps == 1);
  seq.setSteps(SS_STEPS + 5); REQUIRE(seq.activeSteps == SS_STEPS);
}

TEST_CASE("a K-step sequence renders and stays bounded (#149)") {
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.5f);
  seq.setSteps(3);
  auto out = render(seq, 3);
  for (float s : out) {
    REQUIRE(std::isfinite(s));
    REQUIRE(s <= 1.0f);
    REQUIRE(s >= -1.0f);
  }
  REQUIRE(rms(out) > 0.0f);
}

TEST_CASE("rate-limited worker degrades to silence, not garbage") {
  // With a starved worker a head's cushion may be empty when the callback reads
  // it. Correct degradation is CLEAN SILENCE (Head::next returns 0, counts an
  // underrun) -- never a stale/garbage sample. Verify output stays bounded and
  // finite under a throttled service budget.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.0f);
  auto out = render_rate_limited(seq, 2, /*block=*/48, /*fftsPerBlock=*/1);
  for (float v : out) {
    REQUIRE(std::isfinite(v));
    REQUIRE(std::abs(v) <= 1.0f);
  }
}

// --- frame size (#136) ------------------------------------------------------

TEST_CASE("setWindow: window + gain recompute for each frame size") {
  gTab.init();
  for (int w = 64; w <= SS_W; w <<= 1) {
    gTab.setWindow(w);
    REQUIRE(gTab.activeW == w);
    REQUIRE(gTab.activeH == w / 2);
    REQUIRE((1 << gTab.activePasses) == w);
    REQUIRE(std::isfinite(gTab.synthGain));
    REQUIRE(gTab.synthGain > 0.0f);
    REQUIRE(gWindow[0] == Approx(0.0f).margin(1e-3));
    REQUIRE(gWindow[w / 2] == Approx(1.0f).margin(1e-2));
  }
  gTab.setWindow(SS_W_DEFAULT);
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
    REQUIRE(audible);
  }
  gTab.setWindow(SS_W_DEFAULT);
}

TEST_CASE("changing frame size mid-render does not corrupt a sounding head") {
  // Regression: heads snapshot the window curve + gain + geometry per ARM, and
  // bake synthGain into each rendered frame, so a live setFrame() while a head is
  // filling cannot make it multiply a w-sample frame by a curve/gain built for a
  // different size (the fast-scroll volume-jump / broadband-noise bug). Hammer
  // setFrame across sizes WHILE rendering and require finite, bounded output.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.25f);
  seq.setFrame(SS_W);

  const int sizes[] = {SS_W, 512, 2048, 256, 1024, SS_W};
  std::vector<float> out;
  const uint32_t total = seq.patternSamples();
  int si = 0;
  for (uint32_t n = 0; n < total; n++) {
    if (n % (total / 64 + 1) == 0) { seq.setFrame(sizes[si % 6]); si++; }
    for (int g = 0; g < 64 && seq.service(); g++) {}
    float v = seq.next();
    REQUIRE(std::isfinite(v));
    REQUIRE(std::abs(v) <= 1.0f);
    out.push_back(v);
  }
  REQUIRE(rms(out) > 0.0f);
  gTab.setWindow(SS_W_DEFAULT);
}

TEST_CASE("frame size is clamped to [64, SS_W]") {
  gTab.init();
  gTab.setWindow(1);
  REQUIRE(gTab.activeW == 64);
  gTab.setWindow(1 << 20);
  REQUIRE(gTab.activeW == SS_W);
  gTab.setWindow(SS_W_DEFAULT);
}

// --- drift ------------------------------------------------------------------

TEST_CASE("drift perturbs position within bounds and stays reproducible") {
  // (1) drift > 0 with a fixed seed is still deterministic across two renders of
  // the same config; (2) large drift measurably changes output vs zero drift;
  // (3) drift stays clamped in [0,1] position space (no out-of-bounds read).
  gTab.init();
  auto srcbuf = make_source(2.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};

  Sequencer seqA; make_seq(seqA, &src, 48000, 50.0f, 0.5f, 0.0f, /*drift=*/0.3f, 42u);
  Sequencer seqB; make_seq(seqB, &src, 48000, 50.0f, 0.5f, 0.0f, /*drift=*/0.3f, 42u);
  auto a = render(seqA);
  auto b = render(seqB);
  REQUIRE(a.size() == b.size());
  for (size_t i = 0; i < a.size(); i++) REQUIRE(a[i] == Approx(b[i]));

  Sequencer seqZero; make_seq(seqZero, &src, 48000, 50.0f, 0.5f, 0.0f, 0.0f, 42u);
  auto zero = render(seqZero);
  bool differs = false;
  for (size_t i = 0; i < std::min(a.size(), zero.size()); i++)
    if (a[i] != zero[i]) { differs = true; break; }
  REQUIRE(differs);

  for (float v : a) REQUIRE(std::isfinite(v));
}
