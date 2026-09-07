// stretch_core.h - portable DSP core for the stretch sequencer.
// No Arduino or Daisy dependencies, so it can be compiled and verified on a
// host machine. dreamosc.cpp wraps this with audio I/O and pot reading.
//
// ARCHITECTURE (#155, persistent-head): two decoupled clocks with a buffer at
// the seam.  The SEQUENCER is a robotic READ-side clock that owns TIME: it moves
// a read head from step to step, dwelling exactly `duration` (round(duration*sr),
// NO hop quantization -- so frame size no longer bends step timing), and OWNS the
// seam crossfade envelope (fade).  Each SYNTHESIS HEAD is a persistent per-step
// WRITER that owns SUPPLY: it keeps its own demand-cushion ring full of phase-
// randomized PaulXStretch content from the LIVE controls, and knows nothing about
// time, seams, fade, or tails.  Reader owns time; writer owns supply.
//
// This replaces the transient voice-pool model.  A head is pre-warmed one cushion
// ahead of its onset (so it is primed on arrival), gated audible by a 1-bit edge,
// and read raw by the sequencer, which sums the <=3 gated heads under the equal-
// power fade envelope.  fade 0 = RAW CUT: no envelope, no tail -- the spectral
// discontinuity between two uncorrelated phase-randomized streams is the wanted
// character; amplitude is continuous by construction (zero-mean, RMS-tracks-
// source), so a CLICK would be a bug, HEARING THE CUTOVER is not.
//
// STAGE 1 (this file): persistent heads + robotic sequencer + pre-warm + seam
// crossfade + no hop quantization.  Ring-out remnants (the enjoyable lingering
// tail as a budgeted feature) are STAGE 2 and not built here.

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
// Analysis window MAX (must be a power of two). This is the chunk of source that
// gets FFT'd to extract the spectrum, so it sets frequency resolution: bigger =
// narrower bins = the phase randomization beats more slowly = the slow shimmer
// PaulXStretch has, instead of an audible per-hop wobble on tonal material.
// 16384 (~0.34 s at 48 kHz, ~2.9 Hz bins) reaches PaulXStretch's ~0.25 s regime
// (#136). The RUNTIME window is <= this (frame-size control); default is 4096,
// so untouched behavior is unchanged and larger windows are opt-in via the
// window page. Per-head frame buffers scale with SS_W (see SS_HEAD_FLOATS), so
// this is also a memory + FFT-cost knob -- verify max_us on the bench (PROFILE=1).
#define SS_W 16384
#endif
// Default RUNTIME window (must be a power of two, <= SS_W). SS_W is only the
// buffer MAX; the instrument boots at this size and the frame-size control moves
// off it. This is SEPARATE from SS_W on purpose: the host tests and the firmware
// must agree on the boot window, and when SS_W == default was true they were
// coupled by accident. Keep this in sync with FRAME_DEFAULT_IDX in dreamosc.cpp
// (a static_assert there guards it). 4096 = the settled default character.
#ifndef SS_W_DEFAULT
#define SS_W_DEFAULT 4096
#endif
#define SS_H (SS_W / 2)      // MAX output hop (SS_W/2); the RUNTIME hop is activeH
#define SS_STEPS 8

// DEMAND-CUSHION ring per head (#155). The ring is NOT committed audio like the
// old voice FIFO -- it is a small cushion of already-rendered future the ISR
// drains at 1:1, so the fill IS the knob-to-ear / reconfigure latency and the
// pre-warm lead. A control change reaches the ear in ~cushion time, not a whole
// hop. SS_RING is a small power of two; SS_FILL_TARGET is where the producer
// stops topping up (fill oscillates in [FILL_TARGET - SLICE, FILL_TARGET]). At
// 48 kHz 512 samples ~= 10.7 ms. THESE ARE THE TUNABLE (see the profiler): shrink
// toward the bench-measured worst-case single render at 16384, floored by it.
#define SS_RING        1024        // per-head cushion ring (power of two)
#define SS_FILL_TARGET 512         // producer top-up target within the ring
#define SS_SLICE       128         // max samples emitted per service() unit
// Pre-warm/onset lead: a head is armed this many samples before its audible
// onset so the main loop primes its cushion first (it sits silent-and-filling,
// gated off, until the sequencer opens its gate). ~85 ms at 48 kHz -- far more
// than the handful of FFTs a cushion needs. NOTE the true safe lead is
// LOOKAHEAD + worst-case single render; at 16384 confirm on the bench.
#define SS_LOOKAHEAD   SS_W
// Pre-warm request queue depth (power of two). The ISR pushes a pre-warm request
// (which step to arm); the main loop arms the head. Only ever one pending at a
// time in the sequential schedule, but keep slack.
#define SS_ARMQ 16
// Output headroom. The phase-randomized signal's peaks exceed its RMS (~1.07
// measured); scale below 1.0 so the codec never clips (an over-range sample =
// an audible tick on hardware). ~2 dB.
#define SS_HEADROOM 0.8f

// Per-head scratch: 4 rotating frame buffers (old/cur for each of the <=2 lanes)
// + ring_ (SS_RING) + win_ (SS_W, this head's snapshot of the analysis window so
// a live frame-size change can't corrupt a rendering head -- the fast-scroll
// bug). One head per step. The caller allocates SS_POOL_FLOATS and passes it to
// Sequencer::init(); at SS_W 16384 that is SS_STEPS * (4*16384 + 1024 + 16384)
// floats.
#define SS_HEAD_FLOATS (4 * SS_W + SS_RING + SS_W)
#define SS_POOL_FLOATS (SS_STEPS * SS_HEAD_FLOATS)

// ---------------------------------------------------------------------------
// Shared scratch. Only one head renders at a time, from the main loop, so a
// single set of FFT work buffers serves all of them. renderFrame stages the FFT
// in gWork/gSpec (AXI SRAM on device) and memcpys the finished frame out to the
// head's SDRAM buffer -- the FFT never targets SDRAM (d5e4108 placed the hot FFT
// scratch in AXI on purpose).
// ---------------------------------------------------------------------------

// The ShyFFT is sized to the MAX window (SS_W is the compile-time ceiling).
// ShyFFT provides runtime-length overloads Direct/Inverse(in, out, passes) that
// run a SHORTER transform in the same buffers -- so ONE instance covers every
// window size <= SS_W. `passes` is log2(size), NOT the sample count (a real
// gotcha: pass 11 for a 2048-pt transform, not 2048).
typedef ShyFFT<float, SS_W, RotationPhasor> SSFFT;

// Integer log2 of a power of two.
inline int ssLog2(int n) { int p = 0; while ((1 << p) < n) p++; return p; }

// The Nasca (1-x^2)^1.25 analysis window curve, over activeW points. A separate
// GLOBAL rather than a StretchTables member so the device can place it in AXI
// SRAM (fast + cacheable) via AXISRAM_DATA -- 64 KB at SS_W 16384, too big to
// keep crowding DTCM. StretchTables (a constructed object) stays in DTCM; only
// this plain array moves. Defined by the platform (dreamosc.cpp / test_support.h)
// alongside gWork/gSpec.
extern float gWindow[SS_W];

struct StretchTables {
  SSFFT fft;
  float sinLut[1024];
  float synthGain;           // PaulXStretch synthesis output gain (per active W)
  // Active analysis window, runtime-adjustable (#136). SS_W is the buffer max;
  // activeW is the window actually used, <= SS_W, power of two. The DSP reads
  // these, never the SS_W/SS_H macros, so frame size is a live control. Defaults
  // to SS_W_DEFAULT (the boot window), NOT SS_W (the buffer max) -- host and
  // firmware must boot at the same window.
  int   activeW = SS_W_DEFAULT;
  int   activeH = SS_W_DEFAULT / 2;
  int   activePasses = 0;    // log2(activeW), the arg ShyFFT's runtime path wants

  void init() {
    fft.Init();
    for (int i = 0; i < 1024; i++)
      sinLut[i] = sinf(2.0f * (float)M_PI * i / 1024.0f);
    setWindow(SS_W_DEFAULT);  // build the window + gain for the default size
  }

  // Set the active analysis window to `w` (power of two, 64..SS_W). Recomputes
  // Nasca's (1-x^2)^1.25 window over w points and the matching synthGain. Cheap
  // -- called only on a frame-size control change. A rectangular window leaked
  // 0.6% out-of-band energy (audibly scratchy); Nasca's is spectrally clean.
  void setWindow(int w) {
    if (w < 64) w = 64;
    if (w > SS_W) w = SS_W;
    activeW = w;
    activeH = w / 2;
    activePasses = ssLog2(w);
    for (int i = 0; i < w; i++) {
      float x = -1.0f + 2.0f * i / (w - 1);
      gWindow[i] = powf(1.0f - x * x, 1.25f);
    }
    // ShyFFT Direct+Inverse multiplies by w; 1/w undoes it. The window
    // attenuates the input by its mean, so dividing by mean(window) restores
    // unity-ish gain through the pipeline (the unit-gain test checks this).
    float wsum = 0.0f;
    for (int i = 0; i < w; i++) wsum += gWindow[i];
    synthGain = 1.0f / ((wsum / (float)w) * (float)w);
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
// gWindow declared above StretchTables (setWindow builds it); defined by the
// platform alongside gWork/gSpec, placed in AXI SRAM on device.

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
// seamPhase - PURE seam-decision logic (host-tested).  Given the sounding head's
// elapsed dwell in samples, the (unquantized) duration in samples, the fade
// fraction (0..0.5) and the pre-warm lead, decide what the sequencer should do
// THIS sample.  Kept a free function with no state so the timing rule is tested
// directly, not inferred through a render.
//
//   prewarm : elapsed has reached the pre-warm point -> arm the NEXT head now.
//   goLive  : elapsed has reached the incoming head's onset -> open its gate and
//             start the crossfade.
//   end     : elapsed has reached the outgoing head's end -> stop reading it.
//   fadeLen : the crossfade length in samples, frozen at goLive.
//
// fade 0 => onset == end == duration: the incoming head goes live exactly as the
// outgoing head ends (raw cut, no overlap).  fade > 0 => onset = (1-fade)*D, so
// the two overlap for fadeLen = fade*D and the sequencer equal-power-sums them.
// ---------------------------------------------------------------------------
struct SeamDecision {
  bool     prewarm;
  bool     goLive;
  bool     end;
  uint32_t fadeLen;
};
inline SeamDecision seamPhase(uint64_t elapsed, uint32_t durSamples,
                              float fade, uint32_t lookahead) {
  if (fade < 0.0f) fade = 0.0f;
  if (fade > 0.5f) fade = 0.5f;
  uint32_t fadeLen = (uint32_t)(durSamples * fade);
  uint32_t onset   = durSamples - fadeLen;          // incoming head's go-live
  uint32_t prewarmAt = onset > lookahead ? onset - lookahead : 0;
  SeamDecision d;
  d.prewarm = (elapsed == prewarmAt);
  d.goLive  = (elapsed == onset);
  d.end     = (elapsed >= durSamples);
  d.fadeLen = fadeLen;
  return d;
}

// ---------------------------------------------------------------------------
// Head - one persistent synthesis writer, one per step.  Keeps its own demand-
// cushion ring full of phase-randomized PaulXStretch content from the LIVE
// controls.  It knows nothing about time, seams, fade, or tails: the sequencer
// gates it audible and reads it raw.  Ported from the cf22704 lane kernel; the
// spread scheduler / refresh()/two-lane crossfade are NOT ported (the sequencer
// owns the seam now).
// ---------------------------------------------------------------------------

class Head {
 public:
  // Attach this head's working slice: 4 rotating frame buffers + the ring + this
  // head's window snapshot. Supplied by the caller so the big arrays live in
  // SDRAM while the Head OBJECT (atomics) stays in normal memory -- objects in
  // .sdram_bss get neither constructor nor zeroing (NOLOAD, SDRAM unpowered at
  // static init), so only plain data may live there (see CLAUDE.md #129).
  void setBuffers(float* slice) {
    for (int i = 0; i < 4; i++) buf_[i] = slice + (size_t)i * SS_W;
    ring_ = slice + 4 * SS_W;
    win_  = slice + 4 * SS_W + SS_RING;
  }

  // Full reset: idle, ring zeroed (SDRAM arrives holding garbage). No lanes are
  // alive, the gate is shut. seed/index feed the per-head drift RNG so draws are
  // deterministic under any service() scheduling.
  void init(const Source* src, uint32_t seed, uint32_t index) {
    src_ = src;
    seed_ = seed;
    driftRng_ = seed ^ (0x9E3779B9u * (index + 1u));
    if (driftRng_ == 0) driftRng_ = 0x9E3779B9u;
    in_ = Lane();
    out_ = Lane();
    gated_.store(false, std::memory_order_relaxed);
    memset(ring_, 0, SS_RING * sizeof(float));
    memset(win_, 0, SS_W * sizeof(float));
    wr_.store(0, std::memory_order_relaxed);
    rr_.store(0, std::memory_order_relaxed);
  }

  // Producer-side occupancy. A stale rr_ only underestimates how much has been
  // consumed, making the producer conservative -- safe.
  inline uint32_t fill() const {
    return wr_.load(std::memory_order_relaxed)
         - rr_.load(std::memory_order_relaxed);
  }
  bool sounding() const { return in_.alive() || out_.alive(); }
  bool gated() const { return gated_.load(std::memory_order_acquire); }

  // Cushion low-water since the last read of it (profiler: how close the cushion
  // came to starving). Sampled in next(); read-and-reset from the main loop.
  uint32_t takeMinFill() {
    uint32_t m = minFill_;
    minFill_ = fill();
    return m;
  }

  // ARM this head at `position` for a fresh life (main loop only). Snapshots the
  // active window CURVE + geometry for the life about to render -- so a live
  // frame-size change while this head is filling can't corrupt it (the per-voice
  // snapshot pattern from the old Voice, kept per head). Resets the ring so the
  // pre-warm fills from silence. Draws drift once, here, at pre-warm time (NOT at
  // the gate) -- this is the per-fire drift semantics, deterministic under the
  // sequencer's rng. Leaves the head GATED OFF: it fills silently until the
  // sequencer opens the gate at the onset.
  void arm(float position, float driftAmt, const float* stretch) {
    // Snapshot window curve + geometry for this life. renderFrame reads win_/w_/
    // h_/passes_, never the shared gWindow live -- the fast-scroll fix, per head.
    w_ = gTab.activeW;
    h_ = gTab.activeH;
    passes_ = gTab.activePasses;
    memcpy(win_, gWindow, w_ * sizeof(float));
    synthGain_ = gTab.synthGain;

    if (driftAmt > 0.0f) {
      driftRng_ ^= driftRng_ << 13; driftRng_ ^= driftRng_ >> 17;
      driftRng_ ^= driftRng_ << 5;
      float u = (float)(driftRng_ >> 8) / 16777216.0f;   // 0..1
      position += (u * 2.0f - 1.0f) * driftAmt;
    }
    if (position < 0.0f) position = 0.0f;
    if (position >= 1.0f) position = 0.999999f;

    // Fresh life. Any previous life is discarded outright -- the sequencer's
    // gate/read is what makes a head audible, and a re-arm means the reader is
    // (about to be) here again; there is no ring-out in stage 1.
    in_  = Lane();
    out_ = Lane();
    in_.srcPos = (double)position * (double)src_->len;
    // Phase seed comes from the position, not a counter, so the same position
    // always yields the same audio -- what makes zero drift a literal repeat.
    in_.rng = seed_ ^ (uint32_t)(position * 4294967295.0);
    if (in_.rng == 0) in_.rng = 0x9E3779B9u;
    in_.stretch = stretch;
    in_.phase = 0;
    in_.w = w_; in_.h = h_;      // this lane's frozen geometry
    in_.old = nullptr;           // first hop rises from silence (natural rise)
    in_.cur = freeBuf();
    renderFrame(in_, in_.cur, /*advance=*/false);

    // Reset the ring: pre-warm fills from empty. gate shut.
    gated_.store(false, std::memory_order_release);
    wr_.store(0, std::memory_order_release);
    rr_.store(0, std::memory_order_release);
  }

  // Open/close the audible gate (main loop side of the arm; the ISR reads it).
  void openGate()  { gated_.store(true,  std::memory_order_release); }
  void closeGate() { gated_.store(false, std::memory_order_release); }

  // Top up the cushion by one slice, rendering inline as lane hop boundaries are
  // crossed (the demand-driven part -- frames render as late as the cushion
  // allows, from the LIVE controls). Main loop only. Returns true if it emitted.
  bool topUp() {
    if (!in_.alive() && !out_.alive()) return false;   // idle: nothing to make
    uint32_t f = fill();
    if (f >= SS_FILL_TARGET) return false;             // cushion full enough
    uint32_t n = SS_FILL_TARGET - f;
    if (n > SS_SLICE) n = SS_SLICE;
    emit(n);
    return true;
  }

  // ISR read. Returns one raw stream sample, or 0 if not gated or starved. The
  // envelope is NOT applied here -- the sequencer owns the seam and applies the
  // fade when it sums heads. next() ALWAYS advances rr_ when gated so the head's
  // stream stays aligned to wall time even across a fade (the sequencer scales
  // the sample it reads; it never skips reads).
  inline float next() {
    if (!gated_.load(std::memory_order_acquire)) return 0.0f;
    uint32_t w = wr_.load(std::memory_order_acquire);
    uint32_t r = rr_.load(std::memory_order_relaxed);
    uint32_t f = w - r;
    if (f < minFill_) minFill_ = f;     // cushion low-water (profiler)
    if (r == w) {                       // cushion starved (cold arrival / stall)
      extern volatile uint32_t gUnderruns;
      gUnderruns++;
      return 0.0f;
    }
    float raw = ring_[r & (SS_RING - 1)];
    rr_.store(r + 1, std::memory_order_release);
    return raw;
  }

 private:
  // One life's lane: two frame pointers + a phase inside the current output hop,
  // plus the geometry (w/h) the frames in it were rendered at. old==null: rising
  // from silence (first hop). cur==null with old set: natural tail. Both null:
  // dead. Carrying w/h per-lane is what lets a live frame-size change be at most
  // one spectral-wrinkle hop instead of an out-of-bounds read or a gain error:
  // buffers already in flight keep the hop they were made with.
  struct Lane {
    float* old = nullptr;
    float* cur = nullptr;
    uint32_t phase = 0;               // 0..h-1 within the current hop
    double srcPos = 0.0;              // double: position precision matters
    const float* stretch = nullptr;   // live stretch control (not a snapshot)
    uint32_t rng = 1;
    int w = SS_W_DEFAULT;             // this lane's frozen window size
    int h = SS_W_DEFAULT / 2;         // this lane's frozen hop (= w/2)
    bool alive() const { return old != nullptr || cur != nullptr; }
  };

  // Emit n samples of the head's stream (both lanes summed) into the ring. The
  // per-frame synthGain is already baked into the frame buffers (renderFrame),
  // so laneSample does NOT re-apply it -- that is what makes a mixed-size blend
  // (old at one size, cur at another) gain-correct. n never exceeds ring room by
  // construction (topUp sizes it from fill()).
  void emit(uint32_t n) {
    uint32_t w = wr_.load(std::memory_order_relaxed);
    for (uint32_t i = 0; i < n; i++) {
      float v = laneSample(in_) + laneSample(out_);
      ring_[(w + i) & (SS_RING - 1)] = v;
      laneAdvance(in_);
      laneAdvance(out_);
    }
    // Release-store publishes the slice: all ring_ writes are visible to the ISR
    // before it can observe the advanced wr_.
    wr_.store(w + n, std::memory_order_release);
  }

  inline uint32_t laneRand(Lane& l) {
    l.rng ^= l.rng << 13; l.rng ^= l.rng >> 17; l.rng ^= l.rng << 5;
    return l.rng;
  }

  // Canonical PaulXStretch blend, per sample, at the LANE's own hop size. The hop
  // blends the current frame's SECOND half against the previous frame's FIRST
  // half with a raised cosine (a: 1->0); at every hop boundary the output is
  // 100% a single frame at its circular wrap point -- an IFFT is periodic, so
  // block joints are seamless by construction and uncorrelated-frame mixing is
  // confined to mid-hop. The 0.853553... = (1+1/sqrt2)/2 curve corrects the
  // expected amplitude dip of that mid-hop mix. A null old/cur contributes
  // silence -> natural rise and tail for free. synthGain is already baked into
  // the frames, so it is NOT applied here.
  inline float laneSample(const Lane& l) const {
    if (!l.alive()) return 0.0f;
    const float h = 0.853553390593f;
    int hop = l.h;
    float a = 0.5f + 0.5f * gTab.cosAtF(1024.0f * l.phase / (2.0f * hop));
    float corr = h - (1.0f - h) * gTab.cosAtF(1024.0f * l.phase / (float)hop);
    float mixed = 0.0f;
    if (l.cur) mixed += l.cur[hop + l.phase] * (1.0f - a);
    if (l.old) mixed += l.old[l.phase] * a;
    return mixed * corr;
  }

  // Step a lane one sample; at its hop boundary advance its life: next body frame
  // (rendered NOW, from live controls), else the natural tail, else death. The
  // render target reuses the frame being retired, so four buffers cover both
  // lanes. A life has NO body-length limit here (the sequencer, not the head,
  // decides when a head stops sounding, by closing its gate and re-arming) -- so
  // a gated, un-re-armed life renders body frames indefinitely to keep the
  // cushion full. That is the "keep the buffer full" contract.
  inline void laneAdvance(Lane& l) {
    if (!l.alive()) return;
    if (++l.phase < (uint32_t)l.h) return;
    l.phase = 0;
    if (l.cur) {                       // ongoing life: render the next body frame
      float* target = l.old ? l.old : freeBuf();
      float* retiring = l.cur;
      renderFrame(l, target, /*advance=*/true);
      l.old = retiring;
      l.cur = target;
    } else {                           // tail (cur==null): one hop then dead
      l.old = nullptr;
    }
  }

  // A frame buffer no live lane references. Needed only when a lane has no
  // retiring buffer to reuse (its first boundary), at which point at most three
  // of the four are referenced.
  float* freeBuf() const {
    for (int i = 0; i < 4; i++) {
      float* b = buf_[i];
      if (b != in_.old && b != in_.cur && b != out_.old && b != out_.cur)
        return b;
    }
    return buf_[0];                    // unreachable by construction
  }

  // Render one phase-randomized frame for lane l into `target`. When `advance`
  // is set (every body frame after a life's first), the read head first steps by
  // the LIVE stretch -- read at THIS render, so a stretch turn reaches the very
  // next frame; clamp so a control at or below zero cannot divide by zero or run
  // the head backwards. Uses the lane's OWN frozen window/geometry (l.w/l.h/this
  // head's win_ + passes_). The IFFT stages through gWork (AXI SRAM on device);
  // the finished frame is copied out to the SDRAM `target` MULTIPLYING BY the
  // frozen synthGain during the copy, so gain is baked per frame (a mixed-size
  // blend is then gain-correct) and the hot FFT never targets SDRAM.
  void renderFrame(Lane& l, float* target, bool advance) {
    if (advance) {
      float st = (l.stretch && *l.stretch > 0.01f) ? *l.stretch : 0.01f;
      l.srcPos += (double)l.h / (double)st;
    }
    int w = l.w, passes = ssLog2(w);
    int32_t base = (int32_t)l.srcPos;
    for (int i = 0; i < w; i++)
      gWork[i] = src_->at(base + i) * win_[i];   // this head's frozen window

    gTab.fft.Direct(gWork, gSpec, passes);       // runtime-length: passes=log2(w)

    // Split layout: real in gSpec[0..w/2), imaginary in gSpec[w/2..w). gSpec[0]
    // is DC and gSpec[w/2] is Nyquist, both zeroed (Nimbus convention).
    float* re = &gSpec[0];
    float* im = &gSpec[w / 2];
    for (int k = 1; k < w / 2; k++) {
      float mag = sqrtf(re[k] * re[k] + im[k] * im[k]);
      uint32_t a = laneRand(l) >> 22;            // 0..1023
      re[k] = mag * gTab.cosAt(a);               // keep magnitude, redraw phase
      im[k] = mag * gTab.sinAt(a);
    }
    gSpec[0] = 0.0f;                              // DC
    gSpec[w / 2] = 0.0f;                          // Nyquist

    // Full periodic IFFT waveform into the AXI work buffer, then copy out to the
    // head's SDRAM frame buffer scaling by the frozen synthGain (gain baked per
    // frame -- see the header note). NO synthesis window / no OLA here; the
    // per-sample laneSample raised-cosine handoff plays whole frames back-to-back
    // (canonical PaulXStretch, essej/paulxstretch Stretch.cpp).
    gTab.fft.Inverse(gSpec, gWork, passes);
    for (int i = 0; i < w; i++) target[i] = gWork[i] * synthGain_;
  }

  const Source* src_ = nullptr;
  uint32_t seed_ = 0, driftRng_ = 1;
  Lane in_, out_;
  // This head's frozen render geometry + window snapshot for the CURRENT arm.
  int   w_ = SS_W_DEFAULT, h_ = SS_W_DEFAULT / 2, passes_ = 0;
  float synthGain_ = 1.0f;
  // Buffers live in SDRAM, supplied via setBuffers(); pointers, not arrays, so
  // spell out the element count on every memset/memcpy.
  float* buf_[4] = {nullptr, nullptr, nullptr, nullptr};
  float* ring_ = nullptr;   // SS_RING-sample demand cushion
  float* win_  = nullptr;   // this head's snapshot of the analysis window curve
  uint32_t minFill_ = SS_RING;   // cushion low-water since last takeMinFill()
  // Cross-thread state (main-loop producer / audio-ISR consumer). gated_ is the
  // audibility gate the sequencer opens; wr_/rr_ are the SPSC ring indices.
  // Release/acquire pairs make the data they guard visible in order.
  std::atomic<bool>     gated_{false};
  std::atomic<uint32_t> wr_{0}, rr_{0};
};

// ---------------------------------------------------------------------------
// Sequencer - the robotic READ clock. Walks activeSteps positions, dwelling
// exactly `duration` on each (no hop quantization), pre-warms the next head one
// cushion ahead, gates heads audible, and sums the <=3 gated heads under the
// equal-power seam fade. Owns TIME and the seam; the heads own SUPPLY.
// ---------------------------------------------------------------------------

class Sequencer {
 public:
  float position[SS_STEPS] = {0.10f, 0.13f, 0.16f, 0.19f,
                              0.22f, 0.25f, 0.28f, 0.31f};
  float drift[SS_STEPS] = {0};
  float stretch = 50.0f;      // stretch factor
  float duration = 4.0f;      // step DWELL, seconds (live, unquantized)
  // Crossfade between consecutive heads, a fraction of the step duration 0..0.5.
  // fade 0 = RAW CUT (hard spectral cutover, amplitude continuous by
  // construction -- no envelope, no tail). fade > 0 = equal-power overlap: the
  // incoming head goes live fade*duration before the outgoing head ends, and the
  // sequencer sums them under a quarter-sine. 0.5 = max overlap (two heads the
  // whole step); the pre-warm of the next makes 3 the render ceiling.
  float fade  = 0.0f;

  // Frame/window size in samples (#136): a sound-CHARACTER control. Power of two,
  // 64..SS_W. Live now (not latched at fire) -- the head snapshots it per arm.
  // Default = SS_W_DEFAULT so host + firmware boot at the same window.
  int frameSize = SS_W_DEFAULT;

  // Push frameSize into the shared tables (recomputes window + gain for w).
  void setFrame(int w) {
    frameSize = w;
    gTab.setWindow(w);
  }

  // Number of ACTIVE sequence steps walked (#149). position[]/drift[] stay sized
  // to SS_STEPS; only [0, activeSteps) are used.
  int activeSteps = SS_STEPS;

  void setSteps(int n) {
    if (n < 1) n = 1;
    if (n > SS_STEPS) n = SS_STEPS;
    activeSteps = n;
    if (cur_ >= activeSteps) cur_ %= activeSteps;
    if (next_ >= activeSteps) next_ %= activeSteps;
  }

  // pool: SS_POOL_FLOATS floats (SS_STEPS heads' buffers), carved here. Supplied
  // by the caller so it can live in SDRAM (plain data only). Head::init memsets
  // its ring/win, so NOLOAD SDRAM garbage never reaches the output.
  void init(const Source* src, float sampleRate, float* pool,
            uint32_t seed = 0x12345678u) {
    src_ = src;
    sr_ = sampleRate;
    seed_ = seed;
    for (int i = 0; i < SS_STEPS; i++) {
      head_[i].setBuffers(pool + (size_t)i * SS_HEAD_FLOATS);
      head_[i].init(src, seed, (uint32_t)i);
    }
    cur_ = 0;
    next_ = activeSteps > 1 ? 1 : 0;
    inc_ = 0;
    elapsed_ = 0;
    started_ = false;
    fadeLen_ = 0;
    prewarmDone_ = false;
    armHead_ = armTail_ = 0;
    // Prime: arm the first head immediately (main loop will fill it) and schedule
    // its gate to open after the lookahead, so it is primed when it first sounds.
    startPending_ = true;
  }

  // Duration in samples -- LIVE, unquantized (round(duration*sr)). This is the
  // #155 fix: step timing no longer snaps to the hop grid, so frame size can't
  // bend it. Floor of 1 sample so a degenerate control can't divide by zero.
  uint32_t durSamples() const {
    uint32_t n = (uint32_t)(duration * sr_ + 0.5f);
    return n < 1 ? 1 : n;
  }

  // Nominal samples for ONE pass of all active steps: the start lookahead prime
  // plus activeSteps dwells. There is no fixed "pattern length" anymore (the
  // clock is live), but host/test drive loops need a bounded length to render;
  // this is that length. The seam overlap shortens real wall time slightly, so a
  // pass rendered to this length always covers at least one full walk.
  uint32_t patternSamples() const {
    return SS_LOOKAHEAD + (uint32_t)activeSteps * durSamples();
  }

  // Audio callback. One sample of the whole sequence. Runs the robotic clock,
  // pushes pre-warm/advance decisions to the main loop, and sums the gated heads
  // under the seam envelope. All heavy work (rendering) is on the main loop; this
  // stays cheap.
  //
  // DECIDE, then READ, then ADVANCE the clock -- in that order, so the sample we
  // emit uses state consistent for THIS tick. Read discipline: a gated head must
  // have next() called exactly once per sample (that is what keeps its rr_ in
  // lockstep with wall time). Here at most two heads are gated at once -- the
  // outgoing cur_ and, during a seam, the incoming inc_ -- so we read exactly
  // those, once each, applying the seam envelope.
  inline float next() {
    uint32_t dur = durSamples();

    // Kick the very first head: request its arm, open its gate LOOKAHEAD samples
    // later (primed). Until then the whole sequence is silent-and-filling.
    if (startPending_) {
      requestArm(cur_);
      startPending_ = false;
      startGateAt_ = SS_LOOKAHEAD;
    }
    if (!started_) {
      if (startGateAt_ > 0) startGateAt_--;
      if (startGateAt_ == 0) {
        head_[cur_].openGate();
        started_ = true;
        elapsed_ = 0;
        inc_ = cur_;              // no seam in progress: incoming == current
        prewarmDone_ = false;
      } else {
        return 0.0f;             // still priming: silence
      }
    }

    // --- DECIDE (seam phase for the current dwell) ---
    SeamDecision d = seamPhase(elapsed_, dur, fade, SS_LOOKAHEAD);
    bool single = (next_ == cur_);   // activeSteps==1: no distinct next head

    // Pre-warm the next head one cushion-lead ahead of its onset. SKIP for the
    // single-step case: next_==cur_ is the sounding head, and arm() resets its
    // ring -- which would race the ISR reading it. The single step instead hard
    // re-arms at end (below), when the head is briefly un-gated.
    if (d.prewarm && !prewarmDone_ && !single) {
      requestArm(next_);
      prewarmDone_ = true;
    }

    // Incoming head goes live: open its gate, begin the seam. At fade 0 this fires
    // the SAME sample as end (onset==dur), so the raw cut is: inc_ opens, cur_
    // closes, no overlap. Single-step case has no distinct incoming -- it re-arms
    // at end instead, so skip the seam here. HARD-DROP: only cur_ and inc_ are
    // ever gated, and end (below) closes the retiring one, so concurrency stays
    // <=2 gated (<=3 rendering counting the pre-warm).
    if (d.goLive && inc_ == cur_ && !single) {
      inc_ = next_;
      head_[inc_].openGate();
      fadeLen_ = d.fadeLen;
    }

    // --- READ (sum the gated heads under the seam envelope) ---
    float sum = 0.0f;
    bool seaming = (inc_ != cur_);
    if (seaming && fadeLen_ > 0) {
      // Equal-power quarter-sine over [onset, dur): incoming fades IN, outgoing
      // fades OUT. fade_in^2 + fade_out^2 = 1 -> constant power. Position in the
      // fade derives from elapsed_ (onset = dur - fadeLen). fadeLen_ is frozen at
      // go-live but dur is read live, so guard the (unsigned) subtraction against
      // a duration that shrank mid-seam.
      uint32_t onset = (dur > fadeLen_) ? dur - fadeLen_ : 0;
      uint32_t fp = (elapsed_ > onset) ? (uint32_t)(elapsed_ - onset) : 0;
      if (fp > fadeLen_) fp = fadeLen_;
      float fin  = gTab.sinAtF(256.0f * (float)fp / (float)fadeLen_);
      float fout = gTab.sinAtF(256.0f + 256.0f * (float)fp / (float)fadeLen_);
      sum = head_[inc_].next() * fin + head_[cur_].next() * fout;
    } else if (seaming) {
      // fade 0 raw cut: onset==dur, so this branch only runs the single end
      // sample; read both once (cur_ about to close, inc_ just opened).
      sum = head_[cur_].next() + head_[inc_].next();
    } else {
      sum = head_[cur_].next();
    }

    // --- ADVANCE the clock; handle end-of-dwell ---
    if (d.end) {
      if (single) {
        // Single active step: close the head, hard re-arm it (fresh life at its
        // position), and re-prime the LOOKAHEAD gate-open, exactly like startup.
        // No pre-warm was possible (arm resets the ring), so we accept one
        // lookahead of silence per dwell here -- the degenerate 1-step case.
        head_[cur_].closeGate();
        requestArm(cur_);
        started_ = false;
        startGateAt_ = SS_LOOKAHEAD;
        elapsed_ = 0;
        prewarmDone_ = false;
      } else {
        if (cur_ != inc_) head_[cur_].closeGate();   // retire the outgoing head
        cur_ = inc_;                                 // incoming becomes current
        inc_ = cur_;                                 // seam done
        next_ = (cur_ + 1) % activeSteps;
        elapsed_ = 0;
        prewarmDone_ = false;
      }
    } else {
      elapsed_++;
    }

    float out = sum * SS_HEADROOM;
    if (out > 1.0f) out = 1.0f;
    else if (out < -1.0f) out = -1.0f;
    return out;
  }

  // Main loop. Drains pre-warm requests (arm the head, all Head-state mutation in
  // this one thread so it never races the ISR) and tops up cushions. One unit of
  // work per call; returns true if it did work so a caller can spin until idle.
  bool service() {
    uint32_t h = armHead_.load(std::memory_order_acquire);
    uint32_t t = armTail_.load(std::memory_order_relaxed);
    if (h != t) {
      uint32_t step = armReq_[t & (SS_ARMQ - 1)];
      head_[step].arm(position[step], drift[step], &stretch);
      armTail_.store(t + 1, std::memory_order_release);
      return true;
    }
    for (int i = 0; i < SS_STEPS; i++)
      if (head_[i].topUp()) return true;
    return false;
  }

  // Diagnostic: how many heads are currently gated audible (the concurrency the
  // profiler's `act` reports). Producer-side acquire loads; safe from main loop.
  int activeVoices() const {
    int n = 0;
    for (int i = 0; i < SS_STEPS; i++)
      if (head_[i].gated()) n++;
    return n;
  }

  // Diagnostics for the profiler: live cushion fill of the sounding head, and the
  // low-water across ALL heads since the last call (read-and-reset), so cushion
  // depth is sized from data -- the number that says how close we came to
  // starving under the worst control sweep.
  uint32_t curFill() const { return head_[cur_].fill(); }
  uint32_t takeMinFill() {
    uint32_t m = SS_RING;
    for (int i = 0; i < SS_STEPS; i++) {
      uint32_t f = head_[i].takeMinFill();
      if (f < m) m = f;
    }
    return m;
  }

 private:
  // ISR side: enqueue a pre-warm request (which step to arm). No Head state is
  // touched here -- arm() (heavy) runs in the main loop to avoid racing next().
  inline void requestArm(int step) {
    uint32_t h = armHead_.load(std::memory_order_relaxed);
    uint32_t t = armTail_.load(std::memory_order_acquire);
    if (h - t >= SS_ARMQ) return;                 // queue full: drop
    armReq_[h & (SS_ARMQ - 1)] = (uint32_t)step;
    armHead_.store(h + 1, std::memory_order_release);
  }

  const Source* src_ = nullptr;
  Head head_[SS_STEPS];
  float sr_ = 48000.0f;
  uint32_t seed_ = 0;
  // Robotic clock state.
  uint64_t elapsed_ = 0;        // samples the current dwell has been sounding
  int cur_ = 0;                 // step currently sounding (outgoing during a seam)
  int next_ = 1;                // step to pre-warm + go live next
  int inc_ = 0;                 // head that went live this seam (== cur_ if none)
  bool started_ = false;        // first head has gone audible
  bool startPending_ = true;    // first-head arm request not yet pushed
  uint32_t startGateAt_ = 0;    // samples until the first head's gate opens
  bool prewarmDone_ = false;    // next head pre-warmed for this dwell
  uint32_t fadeLen_ = 0;        // seam crossfade length, frozen at go-live
  // SPSC pre-warm queue: ISR (next) pushes at armHead_, main loop (service) pops
  // at armTail_. SS_ARMQ is a power of two.
  uint32_t armReq_[SS_ARMQ];
  std::atomic<uint32_t> armHead_{0}, armTail_{0};
};

#endif  // STRETCH_CORE_H
