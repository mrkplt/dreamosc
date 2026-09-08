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
inline volatile uint32_t gClips = 0;       // +-1 clamp hits (see render output tail)
#ifdef SS_TEST_HOOKS
inline void (*gSsTestHook)(int id, void* ctx) = nullptr;
#endif

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

// COST-MODELLED producer: each FRAME rendered at window size w charges
// cost(w) = costPerNLogN * w * log2(w) samples of main-loop time, and the
// producer only gets `block` samples of time per `block` samples of playback.
// 0.0038 reproduces ~18 ms at 16384 / 48 kHz (the bench number at the old
// clock). The sequencer's own per-size cost estimate is PINNED to the same
// model (the host ISR clock cannot measure it), so the scheduler's slack and
// refresh rules run against the cost the harness actually charges. This is
// the harness that can see scheduling: holds, late go-lives, starvation.
struct CostedProducer {
  double costPerNLogN;
  int    block;
  double budget = 0.0;
  CostedProducer(double c = 0.0038, int b = 32) : costPerNLogN(c), block(b) {}
  double cost(int w) const { return costPerNLogN * w * ssLog2(w); }
  void pin(Sequencer& seq) const {
    for (int s = 0; s < SS_NSIZES; s++)
      seq.setCostEstimate(s, (uint32_t)cost(ssSizeW(s)));
  }
  // Call once per output sample BEFORE seq.next(); services the producer at
  // block boundaries within its time budget.
  void step(Sequencer& seq, uint32_t n) {
    if (n % (uint32_t)block != 0) return;
    pin(seq);
    budget += block;
    for (;;) {
      int w = ssClampW(seq.frameSize);
      double c = cost(w);
      if (budget < c) break;
      int frames = seq.service();
      if (frames <= 0) break;
      budget -= c * frames;                    // a pair costs two frames
    }
    double cap = 4.0 * cost(ssClampW(seq.frameSize));
    if (budget > cap) budget = cap;            // idle time is not banked
  }
};

inline std::vector<float> render_costed(Sequencer& seq, uint32_t total,
                                        double costPerNLogN, int block = 32) {
  std::vector<float> out;
  out.reserve(total);
  CostedProducer p(costPerNLogN, block);
  for (uint32_t n = 0; n < total; n++) {
    p.step(seq, n);
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

// Click detector: a sample-to-sample jump large RELATIVE TO ITS LOCAL CONTEXT
// (8x the mean absolute difference over +-64 samples, and at least 5e-3).
// Returns the count and the index of the first click; `skip(i)` excludes
// regions (seams) the caller accepts as spectral cuts.
template <typename F>
inline int count_clicks(const std::vector<float>& out, F skip, size_t* first = nullptr) {
  const int W = 64;
  int clicks = 0;
  for (size_t i = W + 1; i + W < out.size(); i++) {
    if (skip(i)) continue;
    float d = std::fabs(out[i] - out[i - 1]);
    if (d < 5e-3f) continue;
    float local = 0.0f;
    for (int k = -W; k < W; k++) local += std::fabs(out[i + k] - out[i + k - 1]);
    local /= (2 * W);
    if (d > 8.0f * local) { if (clicks == 0 && first) *first = i; clicks++; }
  }
  return clicks;
}

// Write a mono 16-bit WAV (for listening to a failing case on the host).
inline void write_wav(const char* path, const std::vector<float>& s, uint32_t sr = 48000) {
  FILE* f = fopen(path, "wb");
  if (!f) return;
  auto u32 = [&](uint32_t v) { fwrite(&v, 4, 1, f); };
  auto u16 = [&](uint16_t v) { fwrite(&v, 2, 1, f); };
  uint32_t bytes = (uint32_t)s.size() * 2;
  fwrite("RIFF", 1, 4, f); u32(36 + bytes); fwrite("WAVE", 1, 4, f);
  fwrite("fmt ", 1, 4, f); u32(16); u16(1); u16(1); u32(sr); u32(sr * 2); u16(2); u16(16);
  fwrite("data", 1, 4, f); u32(bytes);
  for (float v : s) { float x = v > 1 ? 1 : (v < -1 ? -1 : v); u16((uint16_t)(int16_t)(x * 32767.0f)); }
  fclose(f);
}

}  // namespace testutil

#endif  // TEST_SUPPORT_H
