// host_main.cpp - compile stretch_core.h on a desktop compiler and render a
// WAV, so the C++ DSP can be diffed against stretchseq.py before it ever
// touches the Daisy. No Arduino/Daisy headers here; this is the whole point of
// stretch_core.h having no platform dependencies.
//
//   c++ -std=c++17 -O2 -I.. host_main.cpp -o stretchcore
//   ./stretchcore in.wav out.wav [--stretch 50] [--duration 4] [--spread 1]
//                 [--passes 1] [--seed 0x12345678]
//
// The step positions and the sample-by-sample drive loop mirror StretchSeq.ino
// and the Sequencer, so what renders here is the same code path the Pod runs.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "stretch_core.h"

// The three globals stretch_core.h externs. On the Pod these live in
// StretchSeq.ino / dreamosc.cpp; on the host they live here.
StretchTables gTab;
float         gWork[SS_W];
float         gSpec[SS_W];

// --- minimal 16-bit PCM WAV I/O (mono/stereo), matching stretchseq.py -------

struct Wav {
  uint32_t           sr = 48000;
  uint16_t           ch = 1;
  std::vector<float> samples;  // interleaved, [-1, 1]
};

static uint32_t rd_u32(const uint8_t* p) {
  return p[0] | (p[1] << 8) | (p[2] << 16) | (uint32_t(p[3]) << 24);
}
static uint16_t rd_u16(const uint8_t* p) { return p[0] | (p[1] << 8); }

static Wav load_wav(const char* path) {
  FILE* f = fopen(path, "rb");
  if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(1); }
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  fseek(f, 0, SEEK_SET);
  std::vector<uint8_t> b(n);
  if (fread(b.data(), 1, n, f) != (size_t)n) { fprintf(stderr, "short read\n"); exit(1); }
  fclose(f);

  Wav w;
  size_t i = 12;  // skip RIFF....WAVE
  const uint8_t* data = nullptr;
  uint32_t       data_len = 0;
  uint16_t       bits = 16;
  while (i + 8 <= (size_t)n) {
    uint32_t id  = rd_u32(&b[i]);
    uint32_t len = rd_u32(&b[i + 4]);
    if (id == 0x20746d66) {  // "fmt "
      w.ch   = rd_u16(&b[i + 10]);
      w.sr   = rd_u32(&b[i + 12]);
      bits   = rd_u16(&b[i + 22]);
    } else if (id == 0x61746164) {  // "data"
      data     = &b[i + 8];
      data_len = len;
    }
    i += 8 + len + (len & 1);
  }
  if (!data || bits != 16) { fprintf(stderr, "need 16-bit PCM WAV\n"); exit(1); }
  size_t count = data_len / 2;
  w.samples.resize(count);
  for (size_t k = 0; k < count; k++) {
    int16_t s = (int16_t)rd_u16(data + 2 * k);
    w.samples[k] = s / 32768.0f;
  }
  return w;
}

static void wr_u32(FILE* f, uint32_t v) {
  uint8_t p[4] = {uint8_t(v), uint8_t(v >> 8), uint8_t(v >> 16), uint8_t(v >> 24)};
  fwrite(p, 1, 4, f);
}
static void wr_u16(FILE* f, uint16_t v) {
  uint8_t p[2] = {uint8_t(v), uint8_t(v >> 8)};
  fwrite(p, 1, 2, f);
}

static void save_wav(const char* path, uint32_t sr, uint16_t ch,
                     const std::vector<float>& s) {
  // Normalize like stretchseq.py's save_wav (peak <= 0.999).
  float peak = 0.0f;
  for (float v : s) { float a = v < 0 ? -v : v; if (a > peak) peak = a; }
  float g = (peak > 0.999f) ? (0.999f / peak) : 1.0f;

  FILE* f = fopen(path, "wb");
  if (!f) { fprintf(stderr, "cannot write %s\n", path); exit(1); }
  uint32_t bytes = s.size() * 2;
  fwrite("RIFF", 1, 4, f); wr_u32(f, 36 + bytes); fwrite("WAVE", 1, 4, f);
  fwrite("fmt ", 1, 4, f); wr_u32(f, 16); wr_u16(f, 1); wr_u16(f, ch);
  wr_u32(f, sr); wr_u32(f, sr * ch * 2); wr_u16(f, ch * 2); wr_u16(f, 16);
  fwrite("data", 1, 4, f); wr_u32(f, bytes);
  for (float v : s) {
    float x = v * g;
    if (x > 1.0f) x = 1.0f; if (x < -1.0f) x = -1.0f;
    wr_u16(f, (uint16_t)(int16_t)lrintf(x * 32767.0f));
  }
  fclose(f);
}

// --- drive ------------------------------------------------------------------

int main(int argc, char** argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: %s in.wav out.wav [--stretch N] [--duration S] "
                    "[--spread S] [--passes N] [--seed H]\n", argv[0]);
    return 1;
  }
  const char* in_path  = argv[1];
  const char* out_path = argv[2];
  float    stretch = 50.0f, duration = 4.0f, spread = 1.0f;
  int      passes  = 1;
  uint32_t seed    = 0x12345678u;
  for (int a = 3; a + 1 < argc; a += 2) {
    std::string k = argv[a];
    if      (k == "--stretch")  stretch  = atof(argv[a + 1]);
    else if (k == "--duration") duration = atof(argv[a + 1]);
    else if (k == "--spread")   spread   = atof(argv[a + 1]);
    else if (k == "--passes")   passes   = atoi(argv[a + 1]);
    else if (k == "--seed")     seed     = strtoul(argv[a + 1], nullptr, 0);
  }

  Wav in = load_wav(in_path);
  // Fold to mono to match the single-channel Source the Pod records.
  std::vector<float> mono;
  if (in.ch == 1) {
    mono = in.samples;
  } else {
    mono.resize(in.samples.size() / in.ch);
    for (size_t k = 0; k < mono.size(); k++) {
      float acc = 0.0f;
      for (uint16_t c = 0; c < in.ch; c++) acc += in.samples[k * in.ch + c];
      mono[k] = acc / in.ch;
    }
  }

  gTab.init();

  Source src;
  src.data = mono.data();
  src.len  = (uint32_t)mono.size();

  Sequencer seq;
  seq.init(&src, (float)in.sr, seed);
  seq.stretch  = stretch;
  seq.duration = duration;
  seq.spread   = spread;
  // Default positions as in StretchSeq.ino; drift 0 for a deterministic diff.
  const float pos[SS_STEPS] = {0.10f, 0.13f, 0.16f, 0.19f,
                               0.22f, 0.25f, 0.28f, 0.31f};
  for (int i = 0; i < SS_STEPS; i++) { seq.position[i] = pos[i]; seq.drift[i] = 0.0f; }

  // Total length: passes * 8 steps on the even lattice.
  // One pass of all SS_STEPS heads is patternSamples(); render `passes` of them.
  uint32_t total = seq.patternSamples() * passes;

  std::vector<float> out;
  out.reserve(total);
  for (uint32_t n = 0; n < total; n++) {
    // Drain service() each sample, same discipline as the device main loop
    // (which spins service() continuously). Pre-roll is deferred into service()
    // now, so a freshly-triggered head is filled here before next() reads it.
    for (int g = 0; g < 64 && seq.service(); g++) {}
    out.push_back(seq.next());
  }

  save_wav(out_path, in.sr, 1, out);
  printf("rendered %u samples (%.2fs) stretch=%.1f dur=%.1f spread=%.1f "
         "passes=%d -> %s\n",
         total, total / (float)in.sr, stretch, duration, spread, passes, out_path);
  return 0;
}
