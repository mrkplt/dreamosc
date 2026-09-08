// The DEVICE cadence: the audio callback calls render(buf, 32), so the
// per-block ISR housekeeping (superseded-frame drain, armed-head deadline,
// step-count shrink re-arm) runs once per 32 samples, and controls land at
// block boundaries. The rest of the suite drives the core per SAMPLE
// (next() == render(&s, 1)), which is a stricter cadence for the housekeeping
// than the board ever runs. These cases re-run the scheduling-sensitive
// properties at the real cadence (drive_blocks), and pin the fingerprint the
// PROFILE firmware prints at boot (`crc=`) to the harness's own number.

#include "catch_amalgamated.hpp"

#include <vector>

#include "test_support.h"

using namespace testutil;

namespace {
constexpr int kBlock = 32;   // AUDIO_BLOCK in dreamosc.cpp

uint32_t crc_of(const std::vector<float>& x) {
  return ~ssCrc32(0xFFFFFFFFu, x.data(), x.size() * sizeof(float));
}
}  // namespace

TEST_CASE("block-32 render is bit-identical to per-sample render (drained producer)",
          "[cadence]") {
  // With an infinitely fast producer every frame is rendered before it is
  // needed at either cadence, so the housekeeping cadence must not change a
  // single sample: it only affects WHEN a descriptor is dropped or a deadline
  // is restated, never WHICH frame plays. Covers three crossfaded seams.
  //
  // The one cadence-dependent moment is STARTUP: the first tick arms head 0,
  // and the producer cannot run inside a block, so at block cadence the first
  // block is `late` silence and go-live lands at sample 32 (the device boots
  // exactly so); per sample it lands at sample 1. Compare aligned at go-live.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  const uint32_t total = 48000 * 3 / 2;
  Sequencer a; make_seq(a, &src, 48000, 50.0f, 0.5f, 0.5f); a.setSteps(3);
  Sequencer b; make_seq(b, &src, 48000, 50.0f, 0.5f, 0.5f); b.setSteps(3);
  auto perSample = drive(a, total, [](uint32_t) {});
  auto perBlock  = drive_blocks(b, total, kBlock, [](uint32_t) {});
  size_t liveS = first_audible(perSample), liveB = first_audible(perBlock);
  REQUIRE(liveS == 1);
  REQUIRE(liveB == (size_t)kBlock);
  REQUIRE(a.lateSamples() == 1);
  REQUIRE(b.lateSamples() == (uint32_t)kBlock);
  size_t n = perSample.size() - liveB, firstDiff = n;
  for (size_t i = 0; i < n; i++)
    if (perSample[liveS + i] != perBlock[liveB + i]) { firstDiff = i; break; }
  INFO("first difference " << firstDiff << " samples after go-live");
  REQUIRE(firstDiff == n);
}

TEST_CASE("renderFingerprint equals the CRC of the block-32 harness output", "[cadence]") {
  // The firmware's boot `crc=` is renderFingerprint() over this exact config
  // (dreamosc.cpp PROFILE); the host can reproduce the number from
  // drive_blocks, so a device-vs-device comparison is the harness's own test.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  auto configure = [&](Sequencer& s) {
    make_seq(s, &src, 48000, 50.0f, 0.5f, 0.5f); s.setSteps(3); s.setFrame(4096);
  };
  Sequencer a; configure(a);
  Sequencer b; configure(b);
  Sequencer c; configure(c); c.position[1] = 0.5f;
  float block[kBlock];
  uint32_t fpA = renderFingerprint(a, block, kBlock, 72000);
  uint32_t fpB = crc_of(drive_blocks(b, 72000, kBlock, [](uint32_t) {}));
  uint32_t fpC = renderFingerprint(c, block, kBlock, 72000);
  REQUIRE(fpA == fpB);
  REQUIRE(fpA != fpC);                       // a control change is visible in it
  REQUIRE(a.holds() == 0);                   // a drained render never holds
}

TEST_CASE("the boot sequence after a fingerprint render restores the audible state exactly",
          "[cadence]") {
  // The PROFILE firmware fingerprints (stretch 50, dwell 0.5, fade 0.5,
  // 3 steps, 4096) and then rebuilds the audible boot state. init() leaves
  // the PUBLIC controls alone by design (they are the player's), so every
  // control the fingerprint touched must be set again explicitly -- the step
  // count leaked once (df0af04 claimed "untouched"; it booted with 3 steps).
  // This mirrors dreamosc.cpp's boot: init, encoderSync, duration, fade,
  // steps; then renders four dwells (a 3-step walk and an 8-step walk only
  // diverge at the fourth) against a never-fingerprinted Sequencer.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  float block[kBlock];
  auto boot = [&](Sequencer& s) {
    make_seq(s, &src, 48000, 50.0f, 1.0f, 0.0f);   // init + the boot values
    s.setFrame(SS_W_DEFAULT);
    s.setSteps(SS_STEPS);
  };
  Sequencer used; make_seq(used, &src, 48000, 50.0f, 0.5f, 0.5f);
  used.setSteps(3); used.setFrame(4096);
  renderFingerprint(used, block, kBlock, 24000);
  boot(used);
  Sequencer fresh; boot(fresh);
  REQUIRE(used.holds() == 0);
  REQUIRE(used.activeSteps == fresh.activeSteps);
  REQUIRE(used.frameSize == fresh.frameSize);
  REQUIRE(used.fade == fresh.fade);
  REQUIRE(used.duration == fresh.duration);
  auto x = drive_blocks(used, 48000 * 4, kBlock, [](uint32_t) {});
  auto y = drive_blocks(fresh, 48000 * 4, kBlock, [](uint32_t) {});
  REQUIRE(x == y);
}

TEST_CASE("block-32: live frame size 4096 -> 1024 reaches the sounding head within one hop",
          "[cadence]") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 0.0f);
  uint32_t when = 48000, seen = 0;
  auto out = drive_blocks(seq, 48000 * 2, kBlock, [&](uint32_t n) {
    if (n == when) seq.setFrame(1024);
    if (n > when && seen == 0 && seq.curHop() == 512) seen = n;
  });
  REQUIRE(seen > 0);
  INFO("applied after " << (seen - when) << " samples");
  REQUIRE(seen - when <= 2048 + 64);
  REQUIRE(silent_windows(out, first_audible(out)) == 0);
}

TEST_CASE("block-32: live position and stretch reach the sounding head within two hops",
          "[cadence]") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  auto run = [&](int what) {
    Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 4.0f, 0.0f);
    return drive_blocks(seq, 48000, kBlock, [&](uint32_t n) {
      if (n == 24000 && what == 1) seq.position[0] = 0.7f;
      if (n == 24000 && what == 2) seq.stretch = 5.0f;
    });
  };
  auto base = run(0);
  for (int what = 1; what <= 2; what++) {
    auto moved = run(what);
    size_t firstDiff = base.size();
    for (size_t i = 24000; i < base.size(); i++) if (base[i] != moved[i]) { firstDiff = i; break; }
    INFO("control " << what << ": first difference " << (firstDiff - 24000) << " samples after the move");
    REQUIRE(firstDiff < 24000 + 2 * 2048);
  }
}

TEST_CASE("block-32, costed: step-count shrink under an armed head does not run late (F9)",
          "[cadence]") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.5f, 0.0f);
  CostedProducer p;
  uint32_t lateAt = 0;
  auto out = drive_blocks(seq, 48000 * 3, kBlock,
      [&](uint32_t n) { if (n == 48000 + 4800) { lateAt = seq.lateSamples(); seq.setSteps(2); } },
      [&](uint32_t n) { p.step(seq, n); });
  REQUIRE(silent_windows(out, first_audible(out)) == 0);
  REQUIRE(seq.lateSamples() - lateAt == 0);
}

TEST_CASE("block-32: the armed head's deadline never reads 'now' through a crossfade seam (F3)",
          "[cadence]") {
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 1.0f, 0.25f);
  int32_t minSlack = 0x7fffffff;
  auto out = drive_blocks(seq, 48000 * 3, kBlock, [&](uint32_t) {
    int32_t s = seq.takeMinSlack();
    if (s < minSlack) minSlack = s;
  });
  REQUIRE(silent_windows(out, first_audible(out)) == 0);
  INFO("min slack at block cadence: " << minSlack);
  REQUIRE(minSlack >= 0);
}

TEST_CASE("block-32, costed: a 0.25 s march under a full crossfade at 16384 stays fed",
          "[cadence]") {
  // The heaviest steady case (cur + inc both rendering 16384 through every
  // seam, plus the armed pair) at the real cadence.
  gTab.init();
  auto srcbuf = make_source(3.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq; make_seq(seq, &src, 48000, 50.0f, 0.25f, 0.5f);
  seq.setFrame(16384);
  CostedProducer p(0.0038);
  auto out = drive_blocks(seq, 48000 * 4, kBlock, [](uint32_t) {},
                          [&](uint32_t n) { p.step(seq, n); });
  REQUIRE(silent_windows(out, first_audible(out)) == 0);
  INFO("holds " << seq.holds() << " late " << seq.lateSamples());
  REQUIRE(seq.holds() == 0);
}
