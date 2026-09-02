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
#define SS_STEPS 8
// DEMAND-DRIVEN synthesis: the ring is a small cushion between each synthesis
// head and its reader, NOT a queue of committed audio. Everything in the ring
// is already-rendered future, so the fill IS the knob-to-ear latency; the
// producer therefore runs only SS_FILL_TARGET ahead (fill oscillates between
// SS_FILL_TARGET - SS_SLICE and SS_FILL_TARGET: ~11-21 ms at 48 kHz). The
// cushion is the time the main loop has to respond before an underrun — the
// worst case is all SS_STEPS heads crossing a frame boundary together (spread
// 0), a burst of 8 FFT renders that must fit inside it. Tighten after
// PROFILE=1 numbers put a real bound on per-render cost. Latency below one
// hop of CONTENT change is impossible at a given SS_W (a frame commits SS_H
// samples of spectrum); the fill governs how fast a change ENTERS the stream.
// Fill 512 / slice 128 -> fill oscillates in [384, 512]: ~8-10.7 ms knob-to-
// ear, floor chosen so the worst render burst (spread 0: all SS_STEPS heads
// transition on the same sample, 8 back-to-back FFT renders, est. 3-6 ms on
// the H750) still fits inside the cushion. PROFILE=1's avg_us data is what
// justifies tightening further. Nothing here is sacred: frames and cushion
// content are deterministic to regenerate, so the design principle is to keep
// the committed region too small to matter rather than to build rewind
// machinery for a fat one.
#define SS_FILL_TARGET 512
#define SS_SLICE 128         // emission quantum; a slice may cross one boundary
// Ring CAPACITY, power of two >= SS_FILL_TARGET + SS_SLICE.
#define SS_RING 1024
// One PERSISTENT head per step: a synthesis side that renders on demand and a
// reader side that consumes one sample per output tick, always. There is no
// voice allocator: a step re-firing is a LIFE HANDOFF inside its own head (see
// Head), so worst-case synthesis is SS_STEPS streams at hop rate, constant
// across the whole spread range — the original voice-pool design let sustained
// concurrency reach 2*SS_STEPS at low spread, which doubled the FFT load
// precisely where the retrigger rate peaked and drove the device over budget.
// Per-head scratch: 4 rotating frame buffers (two lives x old/cur) + the ring.
// The caller allocates SS_POOL_FLOATS floats and passes them to
// Sequencer::init(); at SS_W 4096 that is 8 heads * 72 KB = 576 KB, which must
// live in SDRAM, not internal SRAM.
#define SS_HEAD_FLOATS (4 * SS_W + SS_RING)
#define SS_POOL_FLOATS (SS_STEPS * SS_HEAD_FLOATS)
// Ring priming depth: init() pre-fills each ring to the fill target with
// silence, so the first onsets come up primed (no startup underrun). Onsets
// are scheduled at stream position SS_LOOKAHEAD + s*interval + m*period, so
// the audible lattice is offset by exactly this prime.
#define SS_LOOKAHEAD SS_FILL_TARGET
// Output headroom. The phase-randomized signal's peaks exceed its RMS (~1.07
// measured); scale below 1.0 so the codec never clips (an over-range sample =
// an audible tick on hardware). ~2 dB.
#define SS_HEADROOM 0.8f
// SEAM CONSTRUCTION: window-mediated overlap, no envelope, no crossfade, no
// pre-roll. A life's first hop rises naturally (its overlap partner is absent)
// and its last frame's tail rings out for one hop; adjacent lives overlap by
// exactly that construction, so a seam is the SAME overlap-add as every frame
// junction inside a continuous stretch — which is why it needs no added fade
// to be click-free.
//
// The alternative (pre-roll frame -1, discard its hop, butt-joint heads at full
// steady-state level) was built and A/B'd on hardware; its uncorrelated splice
// at the joint is audible, so it was dropped.
//
// Durations quantize to the hop grid (SS_H samples, ~43 ms). Onsets do NOT:
// a retrigger lands on its exact sample (the incoming life's phase starts
// there; see Head::transition).

// ---------------------------------------------------------------------------
// Shared scratch. Only one head renders at a time, from the main loop, so a
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
// Head - one step's persistent synthesis + read pair.
//
// The synthesis side models the step as LANES: the active life (in_) and the
// fading remnant of the previous life (out_). A lane is just two frame
// pointers and a phase; its per-sample value is the canonical PaulXStretch
// raised-cosine blend of the current frame's second half against the previous
// frame's first half, with a null old-frame meaning "rising from silence" and
// a null current-frame meaning "natural tail". A retrigger is a life HANDOFF:
// the active lane moves to the out slot and completes its hop + natural tail
// (no new renders needed), while a fresh lane rises sample-accurately at the
// onset. All parameters are read at render time, so a control change reaches
// the stream at the next rendered frame and the ear SS_FILL_TARGET samples
// later (~21 ms) — the ring holds a demand cushion, not a schedule.
//
// The read side (audio ISR) consumes one sample per tick, always — no gate,
// no arming, no lifetime.
// ---------------------------------------------------------------------------

class Head {
 public:
  // Attach this head's working slice: 4 rotating frame buffers + the ring.
  // They are supplied by the caller rather than held inline so the big arrays
  // can live in SDRAM while the Head OBJECT stays in normal memory. Objects
  // placed in .sdram_bss get neither their constructor run nor their storage
  // zeroed (NOLOAD section, SDRAM unpowered at static init), so only plain
  // data may live there — see CLAUDE.md.
  void setBuffers(float* slice) {
    for (int i = 0; i < 4; i++) buf_[i] = slice + (size_t)i * SS_W;
    ring_ = slice + 4 * SS_W;
  }

  // Full reset: zero the ring (SDRAM arrives holding garbage) and prime it
  // with SS_LOOKAHEAD samples of silence. Frame buffers need no clearing —
  // a lane never reads a buffer it has not rendered.
  void init(const Source* src, uint32_t seed, uint32_t index) {
    src_ = src;
    seed_ = seed;
    // Per-head drift RNG: draws depend only on this head's own retrigger
    // count, so output is deterministic under any service() scheduling.
    driftRng_ = seed ^ (0x9E3779B9u * (index + 1u));
    if (driftRng_ == 0) driftRng_ = 0x9E3779B9u;
    in_ = Lane();
    out_ = Lane();
    renders_ = 0;
    memset(ring_, 0, SS_RING * sizeof(float));
    wr_.store(SS_LOOKAHEAD, std::memory_order_relaxed);
    rr_.store(0, std::memory_order_relaxed);
    produced_ = SS_LOOKAHEAD;
  }

  // Producer's absolute stream position (samples written since init, including
  // the silence prime). Consumption runs 1:1 with output samples, so stream
  // position p is audible at output sample p.
  uint64_t streamPos() const { return produced_; }
  bool sounding() const { return in_.alive() || out_.alive(); }
  uint32_t renders() const { return renders_; }

  // Producer-side occupancy. A stale rr_ only underestimates how much has been
  // consumed, making the producer conservative — safe.
  inline uint32_t fill() const {
    return wr_.load(std::memory_order_relaxed)
         - rr_.load(std::memory_order_relaxed);
  }

  // Emit n samples of the head's stream (both lanes summed). n never exceeds
  // the ring room by construction (service() sizes it from fill()). A slice
  // that crosses a lane's hop boundary renders the next frame inline — that
  // is the demand-driven part: frames are rendered as late as the cushion
  // allows, from the LIVE controls.
  void emit(uint32_t n) {
    uint32_t w = wr_.load(std::memory_order_relaxed);
    if (!in_.alive() && !out_.alive()) {          // idle: plain silence
      for (uint32_t i = 0; i < n; i++)
        ring_[(w + i) & (SS_RING - 1)] = 0.0f;
    } else {
      for (uint32_t i = 0; i < n; i++) {
        float v = laneSample(in_) + laneSample(out_);
        ring_[(w + i) & (SS_RING - 1)] = v * gTab.synthGain;
        laneAdvance(in_);
        laneAdvance(out_);
      }
    }
    // Release-store publishes the slice: all ring_ writes above are guaranteed
    // visible to the ISR before it can observe the advanced wr_.
    wr_.store(w + n, std::memory_order_release);
    produced_ += n;
  }

  // Life handoff at the CURRENT stream position (service() emits up to the
  // onset first, so this is sample-accurate). The active lane moves to the
  // out slot, where it finishes its current hop and its one-hop natural tail
  // using the frames it already owns — no further renders. The fresh life
  // renders its first frame from the live controls and rises from silence.
  // If a previous out-lane is somehow still fading (possible only at
  // degenerate sub-hop retrigger spacing), it is forced straight to its tail.
  void transition(float position, float driftAmt, const float* stretch,
                  uint32_t lenSamples) {
    if (out_.alive()) {                 // degenerate spacing: hurry the old tail
      if (out_.cur) { out_.old = out_.cur; out_.cur = nullptr; }
    }
    if (in_.alive()) {
      out_ = in_;
      out_.bodyRemain = 0;   // outgoing life: finish this hop + natural tail,
      in_ = Lane();          // no further renders — it is dying, not playing on
    }
    if (driftAmt > 0.0f) {
      driftRng_ ^= driftRng_ << 13; driftRng_ ^= driftRng_ >> 17;
      driftRng_ ^= driftRng_ << 5;
      float u = (float)(driftRng_ >> 8) / 16777216.0f;   // 0..1
      position += (u * 2.0f - 1.0f) * driftAmt;
    }
    if (position < 0.0f) position = 0.0f;
    if (position >= 1.0f) position = 0.999999f;
    in_.srcPos = (double)position * (double)src_->len;
    // Phase seed comes from the position, not from a counter, so the same
    // position always yields the same audio. That is what makes zero drift a
    // literal repeat.
    in_.rng = seed_ ^ (uint32_t)(position * 4294967295.0);
    if (in_.rng == 0) in_.rng = 0x9E3779B9u;
    in_.stretch = stretch;
    in_.phase = 0;
    in_.old = nullptr;                  // first hop rises from silence
    in_.cur = freeBuf();
    renderFrame(in_, in_.cur, /*advance=*/false);
    in_.bodyRemain = lenSamples > SS_H ? lenSamples - SS_H : 0;
  }

  // Mid-life frame REFRESH: re-source the current life's spectrum from the
  // live parameters without retriggering it. The sounding pair fades out as an
  // out-lane while a fresh frame rises in its place — but srcPos, rng, and the
  // body clock carry over, so this is the SAME life continuing, not a new
  // step. This is the "rendered frames are disposable" principle made
  // concrete: a content-affecting control change reaches the ear in cushion
  // time (~10 ms) plus the blend ramp, instead of waiting out the hop. No-op
  // on a silent or already-tailing head. Today no panel control changes
  // current-frame content (stretch only alters the advance, which renderFrame
  // reads live), so nothing calls this yet; it is the entry point for
  // frame-size and other spectral controls (#136).
  void refresh(const float* stretch) {
    if (!in_.cur) return;
    Lane keep = in_;
    if (out_.alive() && out_.cur) {     // hurry any old remnant to its tail
      out_.old = out_.cur;
      out_.cur = nullptr;
    }
    out_ = in_;
    out_.bodyRemain = 0;                // outgoing pair: this hop + tail only
    in_ = Lane();
    in_.srcPos = keep.srcPos;
    in_.rng = keep.rng;
    in_.stretch = stretch ? stretch : keep.stretch;
    in_.phase = 0;
    in_.old = nullptr;
    in_.cur = freeBuf();
    renderFrame(in_, in_.cur, /*advance=*/false);
    in_.bodyRemain = keep.bodyRemain;   // same life, same clock
  }

  // Called from the audio callback. Always consumes: the stream is continuous
  // (sound, handoff, or silence), so there is no activity gate and no envelope.
  inline float next() {
    // Acquire-load wr_: pairs with the producer's release-store, guaranteeing
    // every ring_ sample the index covers is visible before we read it.
    uint32_t w = wr_.load(std::memory_order_acquire);
    uint32_t r = rr_.load(std::memory_order_relaxed);
    // Ring empty = the producer fell behind the whole cushion. Count it and
    // output clean silence; rr_ does not advance, so this head's stream slips
    // late by the starved amount (a one-time offset, inaudible as such).
    if (r == w) {
      extern volatile uint32_t gUnderruns;
      gUnderruns++;
      return 0.0f;
    }
    float raw = ring_[r & (SS_RING - 1)];
    rr_.store(r + 1, std::memory_order_release);
    return raw;
  }

 private:
  // One life of the step: two frame pointers plus a phase inside the current
  // output hop. old == null: rising from silence (first hop). cur == null with
  // old set: the natural tail (last frame blending out). Both null: dead.
  struct Lane {
    float* old = nullptr;
    float* cur = nullptr;
    uint32_t phase = 0;         // 0..SS_H-1 within the current hop
    uint32_t bodyRemain = 0;    // body samples left AFTER the current hop
    double srcPos = 0.0;        // double: position precision matters
    const float* stretch = nullptr;   // live stretch control (not a snapshot)
    uint32_t rng = 1;
    bool alive() const { return old != nullptr || cur != nullptr; }
  };

  // Canonical PaulXStretch blend, evaluated per sample. The hop blends the
  // current frame's SECOND half against the previous frame's FIRST half with a
  // raised cosine (a: 1 -> 0); at every hop boundary the output is 100% a
  // single frame at its circular wrap point — an IFFT is periodic, so block
  // joints are seamless by construction, and uncorrelated-frame mixing is
  // confined to mid-hop. The 0.853553... = (1+1/sqrt2)/2 curve corrects the
  // expected amplitude dip of that mid-hop mix. A null old/cur contributes
  // silence, which yields the natural rise and tail for free.
  inline float laneSample(const Lane& l) const {
    if (!l.alive()) return 0.0f;
    const float h = 0.853553390593f;
    float a = 0.5f + 0.5f * gTab.cosAtF(1024.0f * l.phase / (2.0f * SS_H));
    float corr = h - (1.0f - h) * gTab.cosAtF(1024.0f * l.phase / (float)SS_H);
    float mixed = 0.0f;
    if (l.cur) mixed += l.cur[SS_H + l.phase] * (1.0f - a);
    if (l.old) mixed += l.old[l.phase] * a;
    return mixed * corr;
  }

  // Step a lane one sample; at the hop boundary advance its life: next body
  // frame (rendered NOW, from live controls), else the natural tail, else
  // death. The render target reuses the frame being retired, so four buffers
  // cover both lanes in every reachable state.
  inline void laneAdvance(Lane& l) {
    if (!l.alive()) return;
    if (++l.phase < SS_H) return;
    l.phase = 0;
    if (l.cur && l.bodyRemain > 0) {
      float* target = l.old ? l.old : freeBuf();
      float* retiring = l.cur;
      renderFrame(l, target, /*advance=*/true);
      l.old = retiring;
      l.cur = target;
      l.bodyRemain = l.bodyRemain > SS_H ? l.bodyRemain - SS_H : 0;
    } else if (l.cur) {                 // body done: one-hop natural tail
      l.old = l.cur;
      l.cur = nullptr;
    } else {                            // tail done: dead
      l.old = nullptr;
    }
  }

  // A frame buffer no live lane references. Needed only when a lane has no
  // retiring buffer to reuse (its first boundary), at which point at most
  // three of the four are referenced.
  float* freeBuf() const {
    for (int i = 0; i < 4; i++) {
      float* b = buf_[i];
      if (b != in_.old && b != in_.cur && b != out_.old && b != out_.cur)
        return b;
    }
    return buf_[0];                     // unreachable by construction
  }

  inline uint32_t laneRand(Lane& l) {
    l.rng ^= l.rng << 13; l.rng ^= l.rng >> 17; l.rng ^= l.rng << 5;
    return l.rng;
  }

  // Render one phase-randomized frame for lane l into `target`. When
  // `advance` is set (every body frame after a life's first), the lane's read
  // head first steps by the LIVE stretch — read at THIS render, not banked at
  // the previous one, so a stretch turn reaches the very next frame rendered;
  // clamp so a control at or below zero cannot divide by zero or run the head
  // backwards.
  void renderFrame(Lane& l, float* target, bool advance) {
    if (advance) {
      float st = (l.stretch && *l.stretch > 0.01f) ? *l.stretch : 0.01f;
      l.srcPos += (double)SS_H / (double)st;
    }
    int32_t base = (int32_t)l.srcPos;
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
      uint32_t a = laneRand(l) >> 22;           // 0..1023
      re[k] = mag * gTab.cosAt(a);              // keep magnitude, redraw phase
      im[k] = mag * gTab.sinAt(a);
    }
    gSpec[0] = 0.0f;                            // DC
    gSpec[SS_W / 2] = 0.0f;                     // Nyquist

    // Full periodic IFFT waveform straight into the lane's buffer — NO
    // synthesis window, no overlap-add here. The canonical PaulXStretch
    // synthesis (essej/paulxstretch Stretch.cpp) plays whole frames
    // back-to-back via laneSample's raised-cosine handoff instead.
    gTab.fft.Inverse(gSpec, target);
    renders_++;
  }

  const Source* src_ = nullptr;
  uint32_t seed_ = 0, driftRng_ = 1;
  uint32_t renders_ = 0;      // diagnostic: frames rendered since init
  Lane in_, out_;
  uint64_t produced_ = 0;     // absolute stream position (never wraps)
  // Buffers live in SDRAM, supplied via setBuffers(); see the note there.
  float* buf_[4] = {nullptr, nullptr, nullptr, nullptr};
  float* ring_ = nullptr;     // SS_RING-sample demand cushion
  // Cross-thread state (main-loop producer / audio-ISR consumer): the SPSC ring
  // indices. Release/acquire pairs make the data they guard visible in order.
  std::atomic<uint32_t> wr_{0}, rr_{0};
};

// ---------------------------------------------------------------------------
// Sequencer - eight steps on an even lattice, one persistent Head per step.
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

  // pool: SS_STEPS * SS_HEAD_FLOATS floats of scratch for the heads' frame
  // buffers and rings, carved up here. Supplied by the caller so it can live
  // in SDRAM (plain data only — see Head::setBuffers). Head::init memsets its
  // ring, so NOLOAD SDRAM garbage never reaches the output.
  void init(const Source* src, float sampleRate, float* pool,
            uint32_t seed = 0x12345678u) {
    src_ = src;
    sr_ = sampleRate;
    for (int i = 0; i < SS_STEPS; i++) {
      head_[i].setBuffers(pool + (size_t)i * SS_HEAD_FLOATS);
      head_[i].init(src, seed, (uint32_t)i);
    }
    anchor_ = SS_LOOKAHEAD;
    lastPeriod_ = 0;
    for (int i = 0; i < SS_STEPS; i++) fired_[i] = false;
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
  // (SS_STEPS-1)*interval; its audible span is its body plus a one-hop tail.
  uint32_t patternSamples() const {
    return (SS_STEPS - 1) * intervalSamples() + lenSamples() + SS_H;
  }

  // Audio callback. One sample of the whole sequence: read every head's
  // continuous stream and apply the constant-loudness gain. All synthesis and
  // scheduling live on the producer side, so this stays trivially cheap.
  inline float next() {
    float sum = 0.0f;
    for (int i = 0; i < SS_STEPS; i++) sum += head_[i].next();
    // Constant loudness = a CONSTANT gain per spread setting, never an
    // instantaneous divide. ~1/spread heads sound at once (capped at SS_STEPS)
    // and uncorrelated sources sum in power, so scaling by sqrt(spread)
    // (floored at 1/SS_STEPS heads) holds overall RMS flat across the spread
    // range while leaving each head's fade intact — heads meet at silence, so
    // the lattice is transient-free by construction. (The old
    // sum/sqrt(instantaneous power) flattened a lone head to full-amplitude raw
    // and spliced uncorrelated heads at the boundary: an audible click at every
    // step.)
    float s = spread < 1.0f / SS_STEPS ? 1.0f / SS_STEPS
                                       : (spread > 1.0f ? 1.0f : spread);
    float out = sum * sqrtf(s) * SS_HEADROOM;
    // Hard clamp: the codec must never see an over-range sample (a clip is an
    // audible tick on hardware even though the float host plays it silently).
    if (out > 1.0f) out = 1.0f;
    else if (out < -1.0f) out = -1.0f;
    return out;
  }

  // Main loop. One unit of work per call, returns true if it did work, so a
  // caller can spin until idle. Picks the emptiest head below the fill target
  // and emits one slice of its stream, splitting the slice at a lattice onset
  // so the retrigger handoff lands sample-accurately. All Head-state mutation
  // happens in this one thread, so it never races the ISR's next().
  //
  // Scheduling is PATTERN-ANCHORED: one anchor timestamp marks the current
  // pattern's start, head s's onset candidate is anchor + s*interval with the
  // LIVE interval, and a per-head flag records that it fired this pattern. The
  // anchor advances by one (live) period once every producer has crossed the
  // pattern's end. This gives both properties a live control needs: phase
  // CONTINUITY (a spread change moves candidates by at most
  // (SS_STEPS-1)*Δinterval — an earlier boot-origin lattice re-derived onsets
  // as base + m*period, and the m*Δperiod term re-rolled the phase arbitrarily
  // on every control tick, which measurably silenced heads for entire ramps)
  // and SELF-HEALING spacing (candidates are re-derived from the anchor each
  // pattern, so heads can never drift into permanent coincidence). A candidate
  // that a shrink has already pushed behind a producer fires immediately
  // (o = 0), at most one slice late.
  bool service() {
    uint64_t interval = intervalSamples();
    uint64_t period = (interval == 0) ? lenSamples()
                                      : (uint64_t)SS_STEPS * interval;
    uint64_t minPos = head_[0].streamPos();
    for (int i = 1; i < SS_STEPS; i++) {
      uint64_t p = head_[i].streamPos();
      if (p < minPos) minPos = p;
    }
    // A period change rescales the anchor so the pattern-phase FRACTION is
    // preserved: head s fires at fraction s/SS_STEPS of the pattern, so the
    // fired flags stay meaningful, a grow stretches the remainder of the
    // current pattern instead of inserting a dead pattern (measured ~2 s of
    // silence on an upward spread ramp without this), and a shrink compresses
    // it instead of double-firing.
    if (period != lastPeriod_) {
      if (lastPeriod_ != 0 && minPos > anchor_) {
        uint64_t phase = minPos - anchor_;
        uint64_t newPhase =
            (uint64_t)((double)phase * (double)period / (double)lastPeriod_);
        anchor_ = minPos > newPhase ? minPos - newPhase : 0;
      }
      lastPeriod_ = period;
    }
    while (minPos >= anchor_ + period) {
      anchor_ += period;
      for (int i = 0; i < SS_STEPS; i++) fired_[i] = false;
    }

    // Emptiest head, with hysteresis: emit only once a full slice is owed, so
    // fill oscillates in [SS_FILL_TARGET - SS_SLICE, SS_FILL_TARGET] and a
    // spinning caller does not degrade to one-sample slices of pure
    // scheduling overhead. Ties break to the lowest index, keeping renders
    // deterministic under a fixed drive loop.
    int s = -1;
    uint32_t bestFill = SS_FILL_TARGET - SS_SLICE + 1;
    for (int i = 0; i < SS_STEPS; i++) {
      uint32_t f = head_[i].fill();
      if (f < bestFill) { bestFill = f; s = i; }
    }
    if (s < 0) return false;
    Head& h = head_[s];
    uint32_t n = SS_FILL_TARGET - bestFill;
    if (n > SS_SLICE) n = SS_SLICE;
    uint64_t now = h.streamPos();
    if (!fired_[s]) {
      uint64_t cand = anchor_ + (uint64_t)s * interval;
      if (cand < now + n) {
        if (cand > now) h.emit((uint32_t)(cand - now));   // up to the onset
        h.transition(position[s], drift[s], &stretch, lenSamples());
        fired_[s] = true;
        return true;
      }
    }
    h.emit(n);
    return true;
  }

  // Re-source every sounding head's spectrum from the live parameters (see
  // Head::refresh). The caller throttles — one wave per control gesture tick,
  // not per sample.
  void refreshSounding() {
    for (int i = 0; i < SS_STEPS; i++) head_[i].refresh(&stretch);
  }

  // Diagnostics (producer-side; read from the main loop).
  int soundingHeads() const {
    int n = 0;
    for (int i = 0; i < SS_STEPS; i++)
      if (head_[i].sounding()) n++;
    return n;
  }
  uint32_t framesRendered() const {
    uint32_t n = 0;
    for (int i = 0; i < SS_STEPS; i++) n += head_[i].renders();
    return n;
  }

 private:
  const Source* src_ = nullptr;
  Head head_[SS_STEPS];
  float sr_ = 48000.0f;
  // Pattern-anchored schedule (see service()). anchor_ starts at the prime
  // depth so the first audible lattice sits at SS_LOOKAHEAD + s*interval.
  uint64_t anchor_ = SS_LOOKAHEAD;
  uint64_t lastPeriod_ = 0;         // detects period changes for the rescale
  bool fired_[SS_STEPS] = {false};  // fired-this-pattern, cleared per anchor
};

#endif  // STRETCH_CORE_H
