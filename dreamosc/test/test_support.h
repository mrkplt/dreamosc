// Shared test helpers: the globals stretch_core.h externs, a synthetic source,
// and render-to-buffer drivers that mirror the audio callback + service()
// discipline as the device runs it (and a rate-limited one that does not).
#ifndef TEST_SUPPORT_H
#define TEST_SUPPORT_H

#include <cmath>
#include <cstdint>
#include <vector>

#include "stretch_core.h"

// The globals stretch_core.h externs. Defined once here for the test binary.
inline StretchTables gTab;
inline float         gWork[SS_W];
inline float         gSpec[SS_W];
inline float         gWindows[SS_WIN_FLOATS];
inline float         gBlendA[SS_HOP_FLOATS];
inline float         gBlendC[SS_HOP_FLOATS];
inline volatile uint32_t gUnderruns = 0;   // frame holds (see Head::tick)

namespace testutil {

// A deterministic, spectrally rich mono source of `seconds` at `sr`.
inline std::vector<float> make_source(float seconds, uint32_t sr) {
  std::vector<float> s((size_t)(seconds * sr));
  for (size_t i = 0; i < s.size(); i++) {
    float t = (float)i / sr;
    s[i] = 0.5f * sinf(2.0f * (float)M_PI * (220.0f + 60.0f * t) * t)
         + 0.3f * sinf(2.0f * (float)M_PI * 660.0f * t)
         + 0.2f * sinf(2.0f * (float)M_PI * 1500.0f * t);
  }
  return s;
}

// Render `passes` full patterns of a configured Sequencer into a flat buffer,
// draining service() fully between samples (an infinitely fast producer).
inline std::vector<float> render(Sequencer& seq, int passes = 1) {
  uint32_t total = seq.patternSamples() * passes;
  std::vector<float> out;
  out.reserve(total);
  for (uint32_t n = 0; n < total; n++) {
    for (int g = 0; g < 64 && seq.service(); g++) {}
    out.push_back(seq.next());
  }
  return out;
}

// Render with a COST-MODELLED producer: each render of window size w charges
// cost(w) samples of main-loop time, and the producer only gets `block`
// samples of time per `block` samples of playback. `costPerNLogN` is the
// per-(w*log2 w) cost in samples; 0.0038 reproduces ~18 ms at 16384 / 48 kHz
// (the bench number at the old clock). This is the harness that can see
// scheduling: holds, late go-lives, starvation under a fast march.
inline std::vector<float> render_costed(Sequencer& seq, uint32_t total,
                                        double costPerNLogN, int block = 32) {
  std::vector<float> out;
  out.reserve(total);
  double budget = 0.0;   // main-loop samples of time available
  int w = ssClampW(seq.frameSize);
  double cost = costPerNLogN * w * ssLog2(w);
  for (uint32_t n = 0; n < total; n++) {
    if (n % block == 0) {
      budget += block;
      while (budget >= cost && seq.service()) {
        w = ssClampW(seq.frameSize);
        cost = costPerNLogN * w * ssLog2(w);
        budget -= cost;
      }
      if (budget > 4.0 * cost) budget = 4.0 * cost;   // idle time is not banked
    }
    out.push_back(seq.next());
  }
  return out;
}

inline double rms(const std::vector<float>& x) {
  double acc = 0.0;
  for (float v : x) acc += (double)v * v;
  return x.empty() ? 0.0 : std::sqrt(acc / x.size());
}

inline double rms_range(const std::vector<float>& x, size_t s, size_t n) {
  double acc = 0.0;
  size_t e = s + n; if (e > x.size()) e = x.size();
  for (size_t i = s; i < e; i++) acc += (double)x[i] * x[i];
  return e > s ? std::sqrt(acc / (e - s)) : 0.0;
}

// First sample index at which the output becomes audible (|v| > thresh).
inline size_t first_audible(const std::vector<float>& x, float thresh = 1e-4f) {
  for (size_t i = 0; i < x.size(); i++) if (std::fabs(x[i]) > thresh) return i;
  return x.size();
}

}  // namespace testutil

#endif  // TEST_SUPPORT_H
