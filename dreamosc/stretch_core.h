// stretch_core.h - portable DSP core for the stretch sequencer.
// No Arduino or Daisy dependencies, so it can be compiled and verified on a
// host machine. StretchSeq.ino wraps this with audio I/O and pot reading.

#ifndef STRETCH_CORE_H
#define STRETCH_CORE_H

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>

// Emilie Gillet's real FFT, MIT licensed, as shipped with the Nimbus example in
// DaisyExamples and used by its phase vocoder. Real rather than complex-on-real
// input, so about half the arithmetic of a naive transform, and already tuned
// for this chip.
#include "shy_fft.h"

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

#ifndef SS_W
#define SS_W 4096            // analysis window, must be a power of two
#endif
#define SS_H (SS_W / 2)      // output hop, always half the window
// Per-voice lookahead FIFO, power of two. Must be several hops deep so the main
// loop can stay well ahead of the audio callback; at exactly SS_W (2 hops) the
// ring could drain to empty between refills and momentarily starve the callback.
// 2*SS_W = 4 hops gives a comfortable cushion.
#define SS_RING (2 * SS_W)
#define SS_STEPS 8
// Ceiling on simultaneously sounding heads. Heads start (duration * spread)
// apart and each lives `duration`, so the number in flight is 1/spread. At
// spread -> 0 all SS_STEPS heads sound at once, so the ceiling must be SS_STEPS.
#define SS_MAX_VOICES SS_STEPS
// Arm each head this many samples before its audible onset, giving the main loop
// time to pre-roll + fill the FIFO off the audio callback. ~85 ms at 48 kHz —
// far more than the handful of FFTs a head needs, with margin for the spread-0
// case where all SS_STEPS heads arm at once. Heads armed but unfilled wait
// silently (Voice::next), so this never shifts when a head actually sounds.
#define SS_LOOKAHEAD SS_W
// Short raw-edge fade (samples) applied at each head's start and end to mask the
// splice transient between successive uncorrelated heads. ~5 ms at 48 kHz.
#define SS_EDGE 240

// ---------------------------------------------------------------------------
// Shared scratch. Only one voice renders at a time, from the main loop, so a
// single set of work buffers serves all of them.
// ---------------------------------------------------------------------------

typedef ShyFFT<float, SS_W, RotationPhasor> SSFFT;

struct StretchTables {
  SSFFT fft;
  float window[SS_W];        // (1 - x^2)^1.25, Nasca's original curve
  float sinLut[1024];
  float olaGain;             // corrects the twice-windowed overlap-add sum

  void init() {
    fft.Init();
    for (int i = 0; i < SS_W; i++) {
      float x = -1.0f + 2.0f * i / (SS_W - 1);
      window[i] = powf(1.0f - x * x, 1.25f);
    }
    for (int i = 0; i < 1024; i++)
      sinLut[i] = sinf(2.0f * (float)M_PI * i / 1024.0f);
    // Mean of the summed squared window across one hop, times ShyFFT's inverse
    // scaling. Measured on a host: Direct followed by Inverse multiplies by
    // SS_W, so the 1/N belongs here rather than inside the transform.
    float s = 0.0f;
    for (int i = 0; i < SS_H; i++)
      s += window[i] * window[i] + window[i + SS_H] * window[i + SS_H];
    olaGain = (1.0f / (s / SS_H)) / (float)SS_W;
  }

  inline float sinAt(uint32_t idx) const { return sinLut[idx & 1023]; }
  inline float cosAt(uint32_t idx) const { return sinLut[(idx + 256) & 1023]; }
};

extern StretchTables gTab;
extern float gWork[SS_W];   // windowed frame; ShyFFT::Direct destroys its input
extern float gSpec[SS_W];   // split spectrum: real in [0,W/2), imag in [W/2,W)

// ---------------------------------------------------------------------------
// The source. A plain buffer plus its length; reads wrap, so every position is
// legal and no bounds check is ever needed at the call site.
// ---------------------------------------------------------------------------

struct Source {
  float* data = nullptr;
  uint32_t len = 0;
  inline float at(int32_t i) const {
    i %= (int32_t)len;
    if (i < 0) i += len;
    return data[i];
  }
};

// ---------------------------------------------------------------------------
// Voice - one sounding step.
// ---------------------------------------------------------------------------

class Voice {
 public:
  void reset() { active_ = false; wr_ = rr_ = 0; }
  bool active() const { return active_; }

  // position: 0..1 through the stretch. stretch: factor. lenSamples: duration.
  void start(const Source* src, float position, float stretch,
             uint32_t lenSamples, uint32_t seed) {
    src_ = src;
    if (position < 0.0f) position = 0.0f;
    if (position >= 1.0f) position = 0.999999f;
    srcPos_ = (double)position * (double)src->len;
    srcHop_ = (double)SS_H / (double)stretch;
    len_ = lenSamples;
    out_ = 0;
    produced_ = 0;
    wr_ = rr_ = 0;
    // Phase seed comes from the position, not from a counter, so the same
    // position always yields the same audio. That is what makes zero drift a
    // literal repeat.
    rng_ = seed ^ (uint32_t)(position * 4294967295.0);
    if (rng_ == 0) rng_ = 0x9E3779B9u;
    memset(accum_, 0, sizeof(accum_));
    frame_ = -1;
    preRolled_ = false;   // pre-roll happens in topUp() (main loop), NOT here
    active_ = true;
  }

  // Called from the main loop. Returns true if it did work. The FFT-heavy work
  // (pre-roll + frame rendering) lives here, off the audio callback, so
  // triggering a voice never blocks next(). start() only arms the voice; the
  // first topUp() does the one-frame pre-roll before emitting.
  bool topUp() {
    if (!active_) return false;
    if (!preRolled_) {
      // Pre-roll. Overlap-add means every output sample is the sum of two
      // frames; starting cold leaves the first half-window ~3.5 dB down and
      // spectrally thin. The dependency is exactly one frame deep, so render one
      // frame ahead of the start point and discard its output.
      renderFrame();
      discardHop();
      preRolled_ = true;
      return true;
    }
    // Keep the ring as full as it can go (leaving room for one more hop) rather
    // than topping up only when it drains below one hop. The old SS_H cap let the
    // ring run all the way to empty between refills, so the audio callback could
    // catch the producer and momentarily starve — audible as a gated/stuttering
    // signal. Filling to SS_RING - SS_H keeps a multi-hop cushion.
    if (fill() > SS_RING - SS_H) return false;
    if (produced_ >= len_ + SS_H) return false;
    renderFrame();
    emitHop();
    return true;
  }

  // Called from the audio callback. Writes the enveloped sample and its
  // squared envelope, for the loudness compensation.
  inline void next(float* sample, float* envSq) {
    if (!active_) { *sample = 0.0f; *envSq = 0.0f; return; }
    // Ring empty = main loop has not filled this head yet (its FFTs are still in
    // flight). Output silence and DO NOT advance the envelope: the head waits at
    // its onset rather than burning its lifetime playing an empty buffer. This
    // turns a would-be underrun into an inaudible brief wait that self-corrects
    // as soon as service() catches up. Arming heads LOOKAHEAD samples early (see
    // Sequencer) makes that wait zero in the normal case.
    if (rr_ == wr_) {
      extern volatile uint32_t gUnderruns;  // diagnostic: ring starved mid/at-onset
      gUnderruns++;
      *sample = 0.0f; *envSq = 0.0f; return;
    }
    float raw = ring_[rr_ & (SS_RING - 1)];
    rr_++;

    // Short raw-edge fade to mask the splice transient. The whole-step sine fade
    // (`e` below) is divided back out by the sequencer's power normalization when
    // only one head sounds, so it cannot hide the boundary discontinuity — a head
    // otherwise ends/begins at full amplitude and the handoff clicks. Fading the
    // RAW sample (before the power math sees it) forces the signal to zero across
    // the first/last SS_EDGE samples regardless of normalization, so successive
    // uncorrelated heads splice silently.
    uint32_t edge = SS_EDGE < len_ / 2 ? SS_EDGE : len_ / 2;
    if (edge > 0) {
      if (out_ < edge)
        raw *= (float)out_ / (float)edge;
      else if (out_ >= len_ - edge)
        raw *= (float)(len_ - 1 - out_) / (float)edge;
    }

    // Equal-power (sine) fade across the whole step.
    float e = gTab.sinAt((uint32_t)(1024.0f * 0.5f * out_ / (float)len_));
    *sample = raw * e;
    *envSq = e * e;
    if (++out_ >= len_) active_ = false;
  }

 private:
  inline uint32_t fill() const { return wr_ - rr_; }

  inline uint32_t rand32() {
    rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
    return rng_;
  }

  void renderFrame() {
    int32_t base = (int32_t)srcPos_;
    for (int i = 0; i < SS_W; i++)
      gWork[i] = src_->at(base + i) * gTab.window[i];

    gTab.fft.Direct(gWork, gSpec);

    // Split layout: real part in gSpec[0..W/2), imaginary in gSpec[W/2..W).
    // gSpec[0] is DC and gSpec[W/2] is Nyquist, both of which get zeroed, the
    // same convention Nimbus uses.
    float* re = &gSpec[0];
    float* im = &gSpec[SS_W / 2];
    for (int k = 1; k < SS_W / 2; k++) {
      float mag = sqrtf(re[k] * re[k] + im[k] * im[k]);
      uint32_t a = rand32() >> 22;              // 0..1023
      re[k] = mag * gTab.cosAt(a);              // keep magnitude, redraw phase
      im[k] = mag * gTab.sinAt(a);
    }
    gSpec[0] = 0.0f;                            // DC
    gSpec[SS_W / 2] = 0.0f;                     // Nyquist

    gTab.fft.Inverse(gSpec, gWork);
    for (int i = 0; i < SS_W; i++)
      accum_[i] += gWork[i] * gTab.window[i];

    srcPos_ += srcHop_;
    frame_++;
  }

  void slideAccum() {
    memmove(accum_, accum_ + SS_H, SS_H * sizeof(float));
    memset(accum_ + SS_H, 0, SS_H * sizeof(float));
  }

  void discardHop() { slideAccum(); }

  void emitHop() {
    for (int i = 0; i < SS_H; i++)
      ring_[(wr_ + i) & (SS_RING - 1)] = accum_[i] * gTab.olaGain;
    wr_ += SS_H;
    produced_ += SS_H;
    slideAccum();
  }

  const Source* src_ = nullptr;
  double srcPos_ = 0.0, srcHop_ = 0.0;   // double: position precision matters
  uint32_t len_ = 0, out_ = 0, produced_ = 0, rng_ = 1;
  int32_t frame_ = -1;
  bool active_ = false;
  bool preRolled_ = false;
  float accum_[SS_W];
  float ring_[SS_RING];
  volatile uint32_t wr_ = 0, rr_ = 0;
};

// ---------------------------------------------------------------------------
// Sequencer - eight steps on an even lattice.
// Per step: position, drift.  Global: stretch, duration, spread.
// ---------------------------------------------------------------------------

class Sequencer {
 public:
  float position[SS_STEPS] = {0.10f, 0.13f, 0.16f, 0.19f,
                              0.22f, 0.25f, 0.28f, 0.31f};
  float drift[SS_STEPS] = {0};
  float stretch = 50.0f;      // stretch factor
  float duration = 4.0f;      // step duration, seconds
  float spread = 1.0f;        // 0..1: 0 = all heads together, 1 = end-to-end

  void init(const Source* src, float sampleRate, uint32_t seed = 0x12345678u) {
    src_ = src;
    sr_ = sampleRate;
    seed_ = seed;
    rng_ = seed;
    for (int i = 0; i < SS_MAX_VOICES; i++) voice_[i].reset();
    step_ = 0;
    armClock_ = 0;   // arm the first head on the very first next()
  }

  // Samples between successive head starts. spread is 0..1 as a fraction of the
  // step duration: 0 fires all heads together, 1 spaces them end-to-end. The
  // whole 8-head pattern repeats every patternSamples().
  uint32_t intervalSamples() const {
    float s = spread < 0.0f ? 0.0f : (spread > 1.0f ? 1.0f : spread);
    return (uint32_t)(duration * sr_ * s);
  }

  // How long one full pass of all SS_STEPS heads takes: the last head starts at
  // (SS_STEPS-1)*interval and lasts one duration. At spread 0 that is just one
  // duration (all heads fire at t=0); at spread 1 it is SS_STEPS durations.
  uint32_t patternSamples() const {
    uint32_t dur = (uint32_t)(duration * sr_);
    return (SS_STEPS - 1) * intervalSamples() + dur;
  }

  // Audio callback. One sample of the whole sequence.
  inline float next() {
    // Arm each head LOOKAHEAD samples before its audible onset, so the main loop
    // has time to pre-roll + fill its FIFO before next() reads it (heads that are
    // armed but not yet filled wait silently — see Voice::next — so arming early
    // is safe and does not shift when a head actually sounds). With interval 0
    // (spread ~0) all SS_STEPS heads share onset 0; otherwise one interval apart.
    // Arming runs on a free-running countdown (armClock_), NOT a per-pattern
    // counter that resets, so the next pattern's first head arms while this
    // pattern's last head is still sounding its tail — the seam is continuous and
    // never drains dry. armClock_ counts samples until the next head is ARMED;
    // arming is intrinsically LOOKAHEAD-ahead of onset because a just-armed head
    // waits silently (Voice::next) until service() fills its FIFO.
    uint32_t interval = intervalSamples();
    while (armClock_ == 0) {
      trigger();
      // spread ~0 (interval 0): all heads share one onset — arm them in a burst
      // this sample, then wait one full pattern before the next burst.
      armClock_ = interval > 0 ? interval : patternSamples();
    }
    armClock_--;

    float sum = 0.0f, power = 0.0f;
    for (int i = 0; i < SS_MAX_VOICES; i++) {
      float s, e2;
      voice_[i].next(&s, &e2);
      sum += s;
      power += e2;
    }
    // Uncorrelated sources sum in power, so dividing by the square root of the
    // summed squared envelopes holds level flat however many steps are
    // stacked. Spread stays a character control and never a loudness one.
    return power > 1e-6f ? sum / sqrtf(power) : 0.0f;
  }

  // Main loop. Keeps every voice's FIFO ahead of the audio callback. Does at
  // most one FFT per call (to bound latency) and returns true if it did work, so
  // a caller can spin it until there is nothing left to do.
  bool service() {
    for (int i = 0; i < SS_MAX_VOICES; i++)
      if (voice_[i].topUp()) return true;
    return false;
  }

 private:
  inline uint32_t rand32() {
    rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
    return rng_;
  }

  void trigger() {
    int slot = -1;
    for (int i = 0; i < SS_MAX_VOICES; i++)
      if (!voice_[i].active()) { slot = i; break; }
    if (slot < 0) return;              // all busy: drop rather than steal

    float d = drift[step_];
    float p = position[step_];
    if (d > 0.0f) {
      float u = (float)(rand32() >> 8) / 16777216.0f;   // 0..1
      p += (u * 2.0f - 1.0f) * d;
      if (p < 0.0f) p = 0.0f;
      if (p > 1.0f) p = 1.0f;
    }
    voice_[slot].start(src_, p, stretch,
                       (uint32_t)(duration * sr_), seed_);
    step_ = (step_ + 1) % SS_STEPS;
  }

  const Source* src_ = nullptr;
  Voice voice_[SS_MAX_VOICES];
  float sr_ = 48000.0f;
  uint32_t armClock_ = 0, seed_ = 0, rng_ = 1;  // armClock_: samples to next arm
  int step_ = 0;
};

#endif  // STRETCH_CORE_H
