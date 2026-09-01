// stretch_core.h - portable DSP core for the stretch sequencer.
// No Arduino or Daisy dependencies, so it can be compiled and verified on a
// host machine. StretchSeq.ino wraps this with audio I/O and pot reading.

#ifndef STRETCH_CORE_H
#define STRETCH_CORE_H

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <atomic>

// Emilie Gillet's real FFT, MIT licensed, as shipped with the Nimbus example in
// DaisyExamples and used by its phase vocoder. Real rather than complex-on-real
// input, so about half the arithmetic of a naive transform, and already tuned
// for this chip.
// Vendored upstream, unmodified, pinned in vendor/manifest.txt
// (pichenettes/stmlib@d18def8). Do NOT edit it; `make vendor-check` fails if it
// drifts. Upstream wraps everything in namespace stmlib.
#include "vendor_stmlib/shy_fft.h"
using stmlib::ShyFFT;
using stmlib::RotationPhasor;

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
// Arm-request queue depth (power of two, >= SS_STEPS since spread 0 bursts all
// SS_STEPS at once). The ISR pushes arm requests; the main loop starts voices.
#define SS_ARMQ 16
// Output headroom. The phase-randomized signal's peaks exceed its RMS (~1.07
// measured); scale below 1.0 so the codec never clips (an over-range sample =
// an audible tick on hardware). ~2 dB.
#define SS_HEADROOM 0.8f
// There is NO outer envelope or crossfade in either variant. Two seam
// constructions, compile-time selectable for A/B listening:
//
// Default (butt-joint): each head pre-rolls frame -1 and discards its hop,
// consuming the natural first-hop rise, so every head is steady-state from its
// first audible sample and stops after its body: heads butt-joint at full
// level (the uncorrelated splice at the joint is the audible variable).
//
// -DSS_SEAM_OLA (window-mediated): no pre-roll; the first hop rises naturally
// and the last frame's tail rings out one hop, with adjacent heads overlapping
// by that hop — the seam is the same overlap-add construction as the stretch
// interior.
//
// Durations quantize to the hop grid (SS_H samples, ~43 ms) in both.

// ---------------------------------------------------------------------------
// Shared scratch. Only one voice renders at a time, from the main loop, so a
// single set of work buffers serves all of them.
// ---------------------------------------------------------------------------

typedef ShyFFT<float, SS_W, RotationPhasor> SSFFT;

struct StretchTables {
  SSFFT fft;
  float window[SS_W];        // (1 - x^2)^1.25, Nasca's original curve (analysis)
  float sinLut[1024];
  float synthGain;           // PaulXStretch synthesis output gain

  void init() {
    fft.Init();
    // Rectangular analysis window (constant 0.707), the canonical PaulXStretch
    // default. A shaped window smears a tone across bins, and per-frame phase
    // randomization of those bins beats at the bin spacing — measured 10.7 dB
    // windowed-RMS wobble on a pure sine vs 6.7 dB with rect. (Residual wobble
    // comes from leakage of non-bin-centered tones; the lever for that is a
    // larger SS_W, which shrinks the bin spacing.)
    for (int i = 0; i < SS_W; i++) window[i] = 0.707f;
    for (int i = 0; i < 1024; i++)
      sinLut[i] = sinf(2.0f * (float)M_PI * i / 1024.0f);
    // ShyFFT Direct+Inverse multiplies by SS_W (measured on host), so 1/SS_W
    // undoes the transform scaling. The analysis window attenuates the input by
    // its mean, so dividing by mean(window) restores unity-ish gain through the
    // whole pipeline (verified by the unit-gain test).
    float wsum = 0.0f;
    for (int i = 0; i < SS_W; i++) wsum += window[i];
    synthGain = 1.0f / ((wsum / SS_W) * (float)SS_W);
  }

  inline float sinAt(uint32_t idx) const { return sinLut[idx & 1023]; }
  inline float cosAt(uint32_t idx) const { return sinLut[(idx + 256) & 1023]; }
  // Linearly interpolated LUT reads for smooth per-sample curves (a 1024-entry
  // LUT read with truncation would staircase the crossfade coefficients).
  inline float sinAtF(float idx) const {
    uint32_t i0 = (uint32_t)idx;
    float f = idx - (float)i0;
    float s0 = sinLut[i0 & 1023], s1 = sinLut[(i0 + 1) & 1023];
    return s0 + (s1 - s0) * f;
  }
  inline float cosAtF(float idx) const { return sinAtF(idx + 256.0f); }
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
  void reset() {
    active_.store(false, std::memory_order_relaxed);
    wr_.store(0, std::memory_order_relaxed);
    rr_.store(0, std::memory_order_relaxed);
  }
  bool active() const { return active_.load(std::memory_order_acquire); }

  // position: 0..1 through the stretch. stretch: factor. lenSamples: duration in
  // samples, already quantized to a multiple of SS_H by the sequencer.
  // onsetDelay: samples to stay silent (filling) before becoming audible.
  // The audible span is lenSamples + SS_H: the first frame rises naturally (no
  // pre-roll) and the last frame's tail rings out for one hop — those natural
  // window edges ARE the head's fades, identical to the stretch interior.
  void start(const Source* src, float position, float stretch,
             uint32_t lenSamples, uint32_t seed, uint32_t onsetDelay = 0) {
    src_ = src;
    if (position < 0.0f) position = 0.0f;
    if (position >= 1.0f) position = 0.999999f;
    srcPos_ = (double)position * (double)src->len;
    srcHop_ = (double)SS_H / (double)stretch;
    len_ = lenSamples;
    out_ = 0;
    produced_ = 0;
    onsetDelay_ = onsetDelay;
    wr_.store(0, std::memory_order_relaxed);
    rr_.store(0, std::memory_order_relaxed);
    // Phase seed comes from the position, not from a counter, so the same
    // position always yields the same audio. That is what makes zero drift a
    // literal repeat.
    rng_ = seed ^ (uint32_t)(position * 4294967295.0);
    if (rng_ == 0) rng_ = 0x9E3779B9u;
    memset(old_, 0, sizeof(old_));
#ifdef SS_SEAM_OLA
    preRolled_ = true;    // no pre-roll: the natural first-hop rise is the seam
#else
    preRolled_ = false;   // frame -1 renders in topUp() (main loop), not here
#endif
    // Release-store LAST: every plain field above must be visible to the ISR
    // before it can observe active_ == true. Without the barrier the ISR could
    // see a half-initialized voice (ARM reorders plain stores).
    active_.store(true, std::memory_order_release);
  }

  // Called from the main loop. Returns true if it did work. The FFT-heavy
  // frame rendering lives here, off the audio callback, so triggering a voice
  // never blocks next(). In the butt-joint variant the first topUp() renders
  // frame -1 (pre-roll) straight into old_: the first emitted block then starts
  // 100% inside frame -1's waveform, so the head is at full level from its very
  // first audible sample. In the seam variant old_ starts silent and the first
  // block ramps in through the raised cosine.
  bool topUp() {
    if (!active_.load(std::memory_order_acquire)) return false;
    if (!preRolled_) {
      renderFrame();
      memcpy(old_, gWork, sizeof(old_));   // frame -1 becomes "old"
      preRolled_ = true;
      return true;
    }
    // Keep the ring as full as it can go (leaving room for one more hop);
    // letting it drain to empty between refills lets the callback catch the
    // producer and starve.
    if (fill() > SS_RING - SS_H) return false;
#ifdef SS_SEAM_OLA
    if (produced_ < len_) {              // body: render a frame, emit a hop
      renderFrame();
      emitHop();
      return true;
    }
    if (produced_ < len_ + SS_H) {       // natural tail: the last frame fades
      memset(gWork, 0, sizeof(float) * SS_W);   // out against silence
      emitHop();
      return true;
    }
    return false;
#else
    if (produced_ >= len_) return false;
    renderFrame();
    emitHop();
    return true;
#endif
  }

  // Called from the audio callback. Returns the enveloped sample. The sine fade
  // is applied here and SURVIVES to the output — the sequencer compensates
  // loudness with a constant per-spread gain, never an instantaneous divide (an
  // instantaneous sum/sqrt(power) cancels a lone head's fade entirely, splicing
  // full-amplitude uncorrelated heads at every boundary: the click).
  inline float next() {
    if (!active_.load(std::memory_order_acquire)) return 0.0f;
    // Onset delay: the head is armed but not yet audible. It sits silent while
    // the main loop pre-rolls and fills its buffer (the lookahead window), then
    // becomes audible primed — this is what prevents an onset underrun.
    if (onsetDelay_ > 0) { onsetDelay_--; return 0.0f; }
    // Acquire-load wr_: pairs with emitHop's release-store, guaranteeing every
    // ring_ sample the index covers is visible before we read it.
    uint32_t w = wr_.load(std::memory_order_acquire);
    uint32_t r = rr_.load(std::memory_order_relaxed);
    // Ring empty despite the onset delay = the worker fell behind even with the
    // lookahead (should not happen in normal load). Count it and output clean
    // silence without advancing the envelope, so it self-corrects.
    if (r == w) {
      extern volatile uint32_t gUnderruns;
      gUnderruns++;
      return 0.0f;
    }
    float raw = ring_[r & (SS_RING - 1)];
    rr_.store(r + 1, std::memory_order_release);

    // No envelope in either variant; only the audible span differs.
#ifdef SS_SEAM_OLA
    if (++out_ >= len_ + SS_H) active_.store(false, std::memory_order_release);
#else
    if (++out_ >= len_) active_.store(false, std::memory_order_release);
#endif
    return raw;
  }

 private:
  // Producer-side occupancy. A stale rr_ only underestimates how much has been
  // consumed, making the producer conservative — safe.
  inline uint32_t fill() const {
    return wr_.load(std::memory_order_relaxed)
         - rr_.load(std::memory_order_relaxed);
  }

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

    // Full periodic IFFT waveform — NO synthesis window, no overlap-add. The
    // canonical PaulXStretch synthesis (essej/paulxstretch Stretch.cpp) plays
    // whole frames back-to-back via emitHop()'s raised-cosine handoff instead.
    gTab.fft.Inverse(gSpec, gWork);

    srcPos_ += srcHop_;
  }

  // Canonical PaulXStretch output block (one hop). The block blends the current
  // frame's SECOND half against the previous frame's FIRST half with a raised
  // cosine (a: 1 -> 0). At every block boundary the output is 100% a single
  // frame at its circular wrap point — an IFFT is periodic, so sample 0 follows
  // sample SS_W-1 with perfect phase continuity: block joints are seamless by
  // construction, and uncorrelated-frame mixing is confined to mid-block. The
  // 0.853553... = (1+1/sqrt2)/2 curve corrects the expected amplitude dip of
  // that mid-block mix. This is what kills the hop-rate amplitude wobble the
  // double-window OLA synthesis had on tonal material.
  void emitHop() {
    const float h = 0.853553390593f;
    uint32_t w = wr_.load(std::memory_order_relaxed);
    for (int i = 0; i < SS_H; i++) {
      // a = 0.5 + 0.5*cos(pi*i/SS_H): fraction of 2*pi is i/(2*SS_H).
      float a = 0.5f + 0.5f * gTab.cosAtF(1024.0f * i / (2.0f * SS_H));
      float mixed = gWork[SS_H + i] * (1.0f - a) + old_[i] * a;
      // corr = h - (1-h)*cos(2*pi*i/SS_H): 1/sqrt2 at the (single-frame) edges,
      // 1.0 at the (mixed) middle.
      float corr = h - (1.0f - h) * gTab.cosAtF(1024.0f * i / (float)SS_H);
      ring_[(w + i) & (SS_RING - 1)] = mixed * corr * gTab.synthGain;
    }
    // Release-store publishes the hop: all ring_ writes above are guaranteed
    // visible to the ISR before it can observe the advanced wr_.
    wr_.store(w + SS_H, std::memory_order_release);
    produced_ += SS_H;
    memcpy(old_, gWork, sizeof(old_));   // current frame becomes "old"
  }

  const Source* src_ = nullptr;
  double srcPos_ = 0.0, srcHop_ = 0.0;   // double: position precision matters
  uint32_t len_ = 0, out_ = 0, produced_ = 0, rng_ = 1;
  uint32_t onsetDelay_ = 0;   // samples silent-and-filling before audible
  bool preRolled_ = false;    // frame -1 rendered (consumes the first-hop rise)
  float old_[SS_W];   // previous frame's full IFFT waveform (PXS handoff)
  float ring_[SS_RING];
  // Cross-thread state (main-loop producer / audio-ISR consumer). active_ is the
  // publication gate for a freshly start()ed voice; wr_/rr_ are the SPSC ring
  // indices. Release/acquire pairs make the data they guard visible in order.
  std::atomic<bool>     active_{false};
  std::atomic<uint32_t> wr_{0}, rr_{0};
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
    armHead_ = armTail_ = 0;
    // Start at SS_LOOKAHEAD so the first head arms on sample 0 and becomes
    // audible SS_LOOKAHEAD samples later (~85 ms) — primed, so no startup click.
    armClock_ = SS_LOOKAHEAD;
  }

  // Step duration in samples, quantized to the hop grid (multiples of SS_H).
  // The quantization is what keeps a head's natural tail exactly overlapping
  // the next head's natural rise at spread 1 — the seam only reconstructs the
  // interior OLA sum when the two land on the same grid.
  uint32_t lenSamples() const {
    uint32_t hops = (uint32_t)(duration * sr_ / SS_H + 0.5f);
    return (hops < 1 ? 1 : hops) * SS_H;
  }

  // Samples between successive head starts. spread is 0..1 as a fraction of the
  // (quantized) step duration: 0 fires all heads together, 1 end-to-end.
  uint32_t intervalSamples() const {
    float s = spread < 0.0f ? 0.0f : (spread > 1.0f ? 1.0f : spread);
    return (uint32_t)(lenSamples() * s);
  }

  // How long one full pass of all SS_STEPS heads takes: the last head starts at
  // (SS_STEPS-1)*interval; its audible span is its body (butt-joint) or body
  // plus one-hop tail (seam-OLA variant).
  uint32_t patternSamples() const {
#ifdef SS_SEAM_OLA
    return (SS_STEPS - 1) * intervalSamples() + lenSamples() + SS_H;
#else
    return (SS_STEPS - 1) * intervalSamples() + lenSamples();
#endif
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
    // armClock_ counts down to the next head's ONSET. We arm a head SS_LOOKAHEAD
    // samples early and hand trigger() that same lookahead as the head's
    // onsetDelay, so the worker has SS_LOOKAHEAD samples to pre-roll + fill the
    // buffer while the voice sits silent; the head becomes audible exactly at its
    // scheduled onset — primed, no underrun.
    uint32_t interval = intervalSamples();
    if (armClock_ == SS_LOOKAHEAD) {
      if (interval == 0) {
        // spread ~0: all SS_STEPS heads share this onset — request them together.
        for (int k = 0; k < SS_STEPS; k++) requestArm(SS_LOOKAHEAD);
        armClock_ = patternSamples() + SS_LOOKAHEAD;
      } else {
        requestArm(SS_LOOKAHEAD);
        armClock_ = interval + SS_LOOKAHEAD;   // next onset one interval later
      }
    }
    armClock_--;

    float sum = 0.0f;
    for (int i = 0; i < SS_MAX_VOICES; i++) sum += voice_[i].next();
    // Constant loudness = a CONSTANT gain per spread setting, never an
    // instantaneous divide. ~1/spread heads sound at once and uncorrelated
    // sources sum in power, so scaling by sqrt(spread) (floored at 1/SS_STEPS
    // heads) holds overall RMS flat across the spread range while leaving each
    // head's fade intact — heads meet at silence, so the lattice is transient-
    // free by construction. (The old sum/sqrt(instantaneous power) flattened a
    // lone head to full-amplitude raw and spliced uncorrelated heads at the
    // boundary: an audible click of random size at every step.)
    float s = spread < 1.0f / SS_STEPS ? 1.0f / SS_STEPS
                                       : (spread > 1.0f ? 1.0f : spread);
    float out = sum * sqrtf(s) * SS_HEADROOM;
    // Hard clamp: the codec must never see an over-range sample (a clip is an
    // audible tick on hardware even though the float host plays it silently).
    if (out > 1.0f) out = 1.0f;
    else if (out < -1.0f) out = -1.0f;
    return out;
  }

  // Main loop. Two jobs, both OFF the audio ISR: (1) drain pending arm requests
  // by actually starting voices here — all Voice-state mutation happens in this
  // one thread, so it never races the ISR's next() (the bug that clicked every
  // head onset was start()'s memset/reset running in the ISR while topUp() wrote
  // the same voice from the main loop); (2) keep FIFOs full. One unit of work per
  // call, returns true if it did work, so a caller can spin until idle.
  bool service() {
    uint32_t h = armHead_.load(std::memory_order_acquire);
    uint32_t t = armTail_.load(std::memory_order_relaxed);
    if (h != t) {                         // a pending arm request from the ISR
      startVoice(armReq_[t & (SS_ARMQ - 1)]);
      armTail_.store(t + 1, std::memory_order_release);
      return true;
    }
    for (int i = 0; i < SS_MAX_VOICES; i++)
      if (voice_[i].topUp()) return true;
    return false;
  }

 private:
  inline uint32_t rand32() {
    rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
    return rng_;
  }

  // ISR side: enqueue an arm request only. No Voice state is touched here — the
  // heavy start() runs in the main loop (startVoice) to avoid racing topUp().
  inline void requestArm(uint32_t onsetDelay) {
    uint32_t h = armHead_.load(std::memory_order_relaxed);
    uint32_t t = armTail_.load(std::memory_order_acquire);
    if (h - t >= SS_ARMQ) return;                 // queue full: drop
    armReq_[h & (SS_ARMQ - 1)] = onsetDelay;
    // Release publishes the request payload before the index moves.
    armHead_.store(h + 1, std::memory_order_release);
  }

  // Main-loop side: actually start a voice for the next step. All Voice-state
  // mutation and the step/rng/drift advance happen here, single-threaded.
  void startVoice(uint32_t onsetDelay) {
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
                       lenSamples(), seed_, onsetDelay);
    step_ = (step_ + 1) % SS_STEPS;
  }

  const Source* src_ = nullptr;
  Voice voice_[SS_MAX_VOICES];
  float sr_ = 48000.0f;
  uint32_t armClock_ = 0, seed_ = 0, rng_ = 1;  // armClock_: samples to next arm
  int step_ = 0;
  // SPSC arm queue: ISR (next) pushes at armHead_, main loop (service) pops at
  // armTail_. SS_ARMQ must be a power of two and >= SS_STEPS (spread 0 bursts 8).
  uint32_t armReq_[SS_ARMQ];
  std::atomic<uint32_t> armHead_{0}, armTail_{0};
};

#endif  // STRETCH_CORE_H
