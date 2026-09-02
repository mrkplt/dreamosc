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
// Voice slots. Must EXCEED SS_STEPS: a head's audible span is len + SS_H (its
// one-hop tail), which is longer than the re-fire period, so an incoming burst
// has to overlap the outgoing one's tails. At spread 0 all SS_STEPS heads arm
// together, and with only SS_STEPS slots every slot is still busy when the next
// burst arrives -- startVoice() finds none free, drops all of them, and the
// output goes silent until they expire (a measured 50% duty cycle: 1 s on,
// 1 s off at duration 1 s). Doubling gives every head's tail room to ring out
// while its replacement starts.
#define SS_MAX_VOICES (2 * SS_STEPS)
// Per-voice scratch: old_ (SS_W) + ring_ (SS_RING). The caller allocates
// SS_POOL_FLOATS floats and passes them to Sequencer::init(); at SS_W 4096 that
// is 8 voices * 48 KB = 384 KB, which must live in SDRAM, not internal SRAM.
#define SS_VOICE_FLOATS (SS_W + SS_RING)
#define SS_POOL_FLOATS  (SS_MAX_VOICES * SS_VOICE_FLOATS)
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
// SEAM CONSTRUCTION: window-mediated overlap, no envelope, no crossfade, no
// pre-roll. A head's first hop rises naturally (its overlap partner is absent)
// and its last frame's tail rings out for one hop; adjacent heads overlap by
// exactly that hop, so a head-to-head seam is the SAME overlap-add construction
// as every frame junction inside a continuous stretch — which is why it needs
// no added fade to be click-free.
//
// The alternative (pre-roll frame -1, discard its hop, butt-joint heads at full
// steady-state level) was built and A/B'd on hardware; its uncorrelated splice
// at the joint is audible, so it was dropped.
//
// Durations quantize to the hop grid (SS_H samples, ~43 ms) so the overlap
// lands aligned.

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
    // Nasca's original PaulStretch analysis window. A rectangular window (the
    // PaulXStretch *default*, which is selectable there for a reason) measured
    // lower amplitude wobble but leaked 0.6% of output energy out of band vs
    // 0.0% here — broadband junk, audible as scratchiness. Spectral purity wins;
    // the lever for wobble is frame size (see #136), not the window.
    for (int i = 0; i < SS_W; i++) {
      float x = -1.0f + 2.0f * i / (SS_W - 1);
      window[i] = powf(1.0f - x * x, 1.25f);
    }
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
  // Attach this voice's working buffers. They are supplied by the caller rather
  // than held inline so the big arrays (old_ SS_W + ring_ SS_RING = 48 KB per
  // voice at SS_W 4096) can live in SDRAM while the Voice OBJECT stays in normal
  // memory. Objects placed in .sdram_bss get neither their constructor run nor
  // their storage zeroed (NOLOAD section, SDRAM unpowered at static init), so
  // only plain data may live there — see CLAUDE.md.
  void setBuffers(float* old_buf, float* ring_buf) {
    old_  = old_buf;
    ring_ = ring_buf;
  }

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
  // stretch is taken by POINTER, not value: the read head advances by the LIVE
  // stretch each frame, so turning the control moves heads that are already
  // sounding. Latching it at start() meant a change was inaudible until the next
  // head fired -- up to a full pattern (tens of seconds) of lag.
  void start(const Source* src, float position, const float* stretch,
             uint32_t lenSamples, uint32_t seed, uint32_t onsetDelay = 0,
             uint32_t overlap = 0) {
    src_ = src;
    stretch_ = stretch;
    if (position < 0.0f) position = 0.0f;
    if (position >= 1.0f) position = 0.999999f;
    srcPos_ = (double)position * (double)src->len;
    len_ = lenSamples;
    overlap_ = overlap;        // equal-power fade length (0 = butt-joint)
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
    // old_ starts silent: the first emitted block therefore ramps in through the
    // raised cosine, which IS the head's natural rise (no pre-roll).
    memset(old_, 0, SS_W * sizeof(float));
    // Release-store LAST: every plain field above must be visible to the ISR
    // before it can observe active_ == true. Without the barrier the ISR could
    // see a half-initialized voice (ARM reorders plain stores).
    active_.store(true, std::memory_order_release);
  }

  // Called from the main loop. Returns true if it did work. The FFT-heavy frame
  // rendering lives here, off the audio callback, so triggering a voice never
  // blocks next(). No pre-roll: old_ starts silent, so the first block ramps in
  // through the raised cosine (the head's natural rise), and after the body a
  // final block blends the last frame out against silence (its natural tail).
  bool topUp() {
    if (!active_.load(std::memory_order_acquire)) return false;
    // Keep the ring as full as it can go (leaving room for one more hop);
    // letting it drain to empty between refills lets the callback catch the
    // producer and starve.
    if (fill() > SS_RING - SS_H) return false;
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

    // Equal-power crossfade envelope. With overlap 0 the head is flat across its
    // body (butt-joint, identical to the old no-envelope path); with overlap > 0
    // it fades IN over the first `overlap` samples and OUT over the last, using
    // a quarter-sine so fade_in^2 + fade_out^2 = 1 at the seam — adjacent heads,
    // uncorrelated, sum to constant POWER (constant loudness) through the
    // overlap. gTab.sinAtF indexes a 1024-entry sine LUT (full period = 1024).
    float env = 1.0f;
    if (overlap_ > 0) {
      if (out_ < overlap_) {
        // rising quarter-sine: sin(pi/2 * out_/overlap) via LUT index 0..256
        env = gTab.sinAtF(256.0f * (float)out_ / (float)overlap_);
      } else if (out_ >= len_ - overlap_ && out_ < len_) {
        // falling quarter-sine over the body's last `overlap` samples
        uint32_t k = out_ - (len_ - overlap_);
        env = gTab.sinAtF(256.0f + 256.0f * (float)k / (float)overlap_);
      }
    }
    // Audible span is the body plus the one-hop natural tail.
    if (++out_ >= len_ + SS_H) active_.store(false, std::memory_order_release);
    return raw * env;
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

    // Advance by the LIVE stretch (see start()); clamp so a control at or below
    // zero cannot divide by zero or run the head backwards.
    float st = (stretch_ && *stretch_ > 0.01f) ? *stretch_ : 0.01f;
    srcPos_ += (double)SS_H / (double)st;
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
    memcpy(old_, gWork, SS_W * sizeof(float));   // current frame becomes "old"
  }

  const Source* src_ = nullptr;
  double srcPos_ = 0.0;        // double: position precision matters
  const float* stretch_ = nullptr;   // live stretch control (not a snapshot)
  uint32_t len_ = 0, out_ = 0, produced_ = 0, rng_ = 1;
  uint32_t onsetDelay_ = 0;   // samples silent-and-filling before audible
  uint32_t overlap_ = 0;      // equal-power crossfade length in samples
  // Buffers live in SDRAM, supplied via setBuffers(); see the note there.
  // NOTE: pointers, not arrays -- sizeof() on these is a pointer size, so always
  // spell out the element count when memset/memcpy-ing them.
  float* old_  = nullptr;   // previous frame's full IFFT waveform (PXS handoff)
  float* ring_ = nullptr;   // SS_RING-sample output FIFO
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
  // Crossfade between consecutive heads. Heads are SEQUENTIAL (one at a time);
  // fade sets how much each head overlaps the next, as a fraction of the step
  // duration, 0..0.5. 0 (or xfade off) = butt-joint, hard cut. 0.5 = maximum
  // overlap: [mix][no clean middle][mix], still only TWO heads at once — the
  // ceiling that keeps this off the 8-head CPU cliff the old `spread` model hit
  // at its zero end. The overlap is an equal-power (constant-loudness) fade.
  bool  xfade = false;        // crossfade enable (Pod button 2)
  float fade  = 0.0f;         // 0..0.5 overlap fraction (encoder page 1)

  // pool: SS_MAX_VOICES * SS_VOICE_FLOATS floats of scratch for the voices'
  // old_/ring_ buffers, carved up here. Supplied by the caller so it can live in
  // SDRAM (plain data only — see Voice::setBuffers).
  void init(const Source* src, float sampleRate, float* pool,
            uint32_t seed = 0x12345678u) {
    src_ = src;
    sr_ = sampleRate;
    seed_ = seed;
    rng_ = seed;
    for (int i = 0; i < SS_MAX_VOICES; i++) {
      float* base = pool + (size_t)i * SS_VOICE_FLOATS;
      voice_[i].setBuffers(/*old_=*/base, /*ring_=*/base + SS_W);
      voice_[i].reset();
    }
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

  // Effective overlap fraction: 0 when crossfade is off, else fade clamped to
  // [0, 0.5]. 0.5 is the hard ceiling — beyond it a THIRD head would overlap,
  // which is exactly the multi-head CPU pileup this model exists to avoid.
  float overlapFrac() const {
    if (!xfade) return 0.0f;
    return fade < 0.0f ? 0.0f : (fade > 0.5f ? 0.5f : fade);
  }

  // Samples between successive head starts. Heads are sequential; they overlap
  // by overlapFrac() of the (quantized) duration, so the start interval is
  // (1 - overlap) of a duration. Overlap 0 -> interval = duration (butt-joint,
  // one head at a time). Overlap 0.5 -> interval = half a duration (two heads
  // overlap, the max). This REPLACES the old spread model; interval never falls
  // below half a duration, so at most two heads ever render at once.
  uint32_t intervalSamples() const {
    return (uint32_t)(lenSamples() * (1.0f - overlapFrac()));
  }

  // How long one full pass of all SS_STEPS heads takes: the last head starts at
  // (SS_STEPS-1)*interval; its audible span is its body plus a one-hop tail.
  uint32_t patternSamples() const {
    return (SS_STEPS - 1) * intervalSamples() + lenSamples() + SS_H;
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
    // The controls are live, so the lattice can change out from under a pending
    // countdown. Two guards:
    //  - clamp: if duration/spread just shrank, a countdown scheduled against the
    //    OLD (longer) lattice would leave a hole; pull it in to the new one.
    //  - '<=' not '==': if the lattice shrank past SS_LOOKAHEAD entirely, an
    //    equality test never fires and armClock_ runs down through zero and wraps
    //    (it is unsigned), silencing the sequence until it counts all the way
    //    back around. That was a measured ~0.09 s gap on an abrupt change.
    // Samples from one onset to the next. At spread 0 all SS_STEPS heads share an
    // onset, so the burst repeats when the heads END — one head span, NOT
    // patternSamples() (which at spread 0 already IS one span, so using it made
    // the burst wait a whole extra span: a measured 50% duty cycle, 1 s on / 1 s
    // off at duration 1 s). Above spread 0 the onsets are one interval apart.
    uint32_t period = (interval == 0) ? lenSamples() : interval;
    // Controls are live, so the lattice can shrink out from under a countdown
    // scheduled against the old one; pull it in so no hole opens up. '<=' rather
    // than '==' because a shrink can step the countdown past SS_LOOKAHEAD, and an
    // equality test would then miss and let armClock_ wrap (it is unsigned).
    uint32_t ceiling = period + SS_LOOKAHEAD;
    if (armClock_ > ceiling) armClock_ = ceiling;
    if (armClock_ <= SS_LOOKAHEAD) {
      uint32_t onset_delay = armClock_;
      if (interval == 0) {
        for (int k = 0; k < SS_STEPS; k++) requestArm(onset_delay);
      } else {
        requestArm(onset_delay);
      }
      armClock_ = period + SS_LOOKAHEAD;
    }
    armClock_--;

    float sum = 0.0f;
    for (int i = 0; i < SS_MAX_VOICES; i++) sum += voice_[i].next();
    // Constant loudness needs NO global gain now: heads are sequential and their
    // seams carry an equal-power crossfade envelope (Voice::next), so at every
    // instant the overlapping heads sum to constant power. Only fixed headroom
    // is applied. (The old sqrt(spread) gain compensated for ~1/spread stacked
    // heads — a model this crossfade design replaces, capping overlap at two.)
    float out = sum * SS_HEADROOM;
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

  // Diagnostic: how many voice slots are currently active. Each active() is an
  // acquire load; safe to call from the main loop.
  int activeVoices() const {
    int n = 0;
    for (int i = 0; i < SS_MAX_VOICES; i++)
      if (voice_[i].active()) n++;
    return n;
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
    // Overlap length in samples = overlap fraction of the step duration. This
    // is the head's equal-power fade in/out length; it equals the gap between
    // this head's start and the previous head's end, so the two crossfade
    // exactly. 0 when crossfade is off -> flat envelope, butt-joint.
    uint32_t overlap = (uint32_t)(lenSamples() * overlapFrac());
    voice_[slot].start(src_, p, &stretch,
                       lenSamples(), seed_, onsetDelay, overlap);
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
