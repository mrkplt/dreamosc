// seam_probe.cpp - ad-hoc host measurement (review aid, not a test).
// Renders the core at fade 0 and reports the short-window RMS envelope around
// every head seam, so the depth/length of any level dip at a raw cut can be
// read as numbers rather than argued about.
//   c++ -std=c++17 -O2 -I.. -I../test seam_probe.cpp -o seam_probe && ./seam_probe
#include <cstdio>
#include <cmath>
#include <vector>
#include "test_support.h"

using namespace testutil;

static std::vector<float> pool(SS_POOL_FLOATS);

static void probe(int frame, float duration, float fade, float ringout, int steps) {
  gTab.init();
  auto srcbuf = make_source(4.0f, 48000);
  Source src{srcbuf.data(), (uint32_t)srcbuf.size()};
  Sequencer seq;
  seq.init(&src, 48000.0f, pool.data());
  seq.stretch = 50.0f; seq.duration = duration; seq.fade = fade;
  seq.ringout = ringout; seq.setSteps(steps);
  seq.setFrame(frame);

  uint32_t dur = seq.durSamples();
  uint32_t total = dur * 4 + 4800;
  std::vector<float> out; out.reserve(total);
  for (uint32_t n = 0; n < total; n++) {
    for (int g = 0; g < 128 && seq.service(); g++) {}
    out.push_back(seq.next());
  }
  size_t s0 = first_audible(out);
  const uint32_t W = 240;   // 5 ms windows
  double ref = rms_range(out, s0 + dur + dur / 2, 4800);
  printf("frame=%d dur=%.2fs fade=%.2f ringout=%.1f steps=%d  ref_rms=%.4f  first audible at sample %zu\n",
         frame, duration, fade, ringout, steps, ref, s0);
  uint32_t hop = frame / 2;
  for (int k = 1; k <= 2; k++) {
    size_t seam = s0 + (size_t)k * dur;
    printf("  seam %d @ %zu: ", k, seam);
    int below3 = 0;
    double deepest = 0.0;
    for (int64_t s = (int64_t)seam - 480; s < (int64_t)seam + (int64_t)hop * 3 / 2; s += W) {
      double r = rms_range(out, (size_t)s, W);
      double db = 20.0 * std::log10(std::max(r, 1e-9) / ref);
      if (s >= (int64_t)seam && db < -3.0) below3++;
      if (s >= (int64_t)seam && db < deepest) deepest = db;
      if ((s - (int64_t)seam) % ((int64_t)W * 4) == 0) printf("%+.0f ", db);
    }
    printf("\n     -> %d x 5ms windows (%.0f ms) more than 3 dB below steady state after the cut; deepest %.1f dB\n",
           below3, below3 * 5.0, deepest);
  }
}

int main() {
  probe(4096, 1.0f, 0.0f, 0.0f, 8);
  probe(16384, 1.0f, 0.0f, 0.0f, 8);
  probe(4096, 1.0f, 0.0f, 0.0f, 1);   // single-step case
  return 0;
}
