// Shared test helpers: the three globals stretch_core.h externs, a synthetic
// source, and a render-to-buffer driver that mirrors the audio callback +
// service() discipline exactly as the device runs it.
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
inline float         gWindow[SS_W];   // analysis window curve (AXI SRAM on device)
inline volatile uint32_t gUnderruns = 0;   // diagnostic counter (see Voice::next)

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
// interleaving service() and next() the same way loop()/AudioCallback do.
inline std::vector<float> render(Sequencer& seq, int passes = 1) {
  uint32_t total = seq.patternSamples() * passes;
  std::vector<float> out;
  out.reserve(total);
  for (uint32_t n = 0; n < total; n++) {
    // Drain service() fully each sample: on-device the main loop spins service()
    // continuously between callback samples. A bounded cap guards against a
    // runaway while staying well above the worst case (8 heads * a few frames).
    for (int g = 0; g < 64 && seq.service(); g++) {}
    out.push_back(seq.next());
  }
  return out;
}

// Render mimicking the on-device producer/consumer race: service() (the worker)
// gets only `fftsPerBlock` FFT calls per `block` samples of playback, instead of
// being fully drained every sample. This reproduces a cold buffer at a head's
// onset — the failure the fully-drained render() cannot see. With a correct
// ready-gate, an unready voice stays silent (no click); without it, the callback
// reads an empty/partial buffer and the seam jumps.
inline std::vector<float> render_rate_limited(Sequencer& seq, int passes,
                                              int block, int fftsPerBlock) {
  uint32_t total = seq.patternSamples() * passes;
  std::vector<float> out;
  out.reserve(total);
  int budget = 0;
  for (uint32_t n = 0; n < total; n++) {
    if ((int)(n % block) == 0) budget = fftsPerBlock;   // refill worker budget
    while (budget > 0 && seq.service()) budget--;
    out.push_back(seq.next());
  }
  return out;
}

inline double rms(const std::vector<float>& x) {
  double acc = 0.0;
  for (float v : x) acc += (double)v * v;
  return x.empty() ? 0.0 : std::sqrt(acc / x.size());
}

}  // namespace testutil

#endif  // TEST_SUPPORT_H
