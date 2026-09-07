// stretch_core.h - portable DSP core for the stretch sequencer.
// No Arduino or Daisy dependencies, so it can be compiled and verified on a
// host machine. dreamosc.cpp wraps this with audio I/O and pot reading.
//
// ARCHITECTURE: frames are the unit. PaulXStretch output is, per hop, a raised-
// cosine blend of the previous frame's first half against the current frame's
// second half (times the 0.853553... AM-correction curve). So a sounding HEAD is
// just two frames (old, cur) and a phase, and the only thing the producer has to
// guarantee is "the NEXT frame is rendered before phase reaches the hop". There
// is no sample ring and no cushion: the ISR blends straight out of the frame
// buffers, the deadline of every render is known exactly (h - phase), and the
// main loop serves renders earliest-deadline-first.
//
// Two clocks, one seam (Fizzy #155): the SEQUENCER (ISR) owns TIME -- it moves
// from step to step, dwelling exactly round(duration*sr), owns the seam
// crossfade, and rotates frames at hop boundaries. HEADS (main loop) own SUPPLY:
// they render frames from the LIVE controls into staged buffers. Rendering is
// speculative and cheap to redo: a staged frame is RE-RENDERED when the
// controls it was made with have changed and there is slack before its
// deadline. That is what "clock is cheap, latency is not" buys -- the safety of
// an early render and the responsiveness of a late one.
//
// PRE-ROLL: a head goes live with BOTH frames rendered (the spec's one-frame
// pre-roll), so its first sample is mid-stream, full level. A raw cut (fade 0)
// is then a pure spectral cutover -- amplitude continuous, as #155 intended. An
// under-fed head repeats its current frame (spectrally the same, never silent).
//
// HEAD POOL: a head is a pool slot allocated at pre-warm (the moment the current
// step goes live, so the lead is the whole dwell) and freed when its step ends
// or its ring-out expires. Ring-out is just "do not free the departing head
// yet": no adoption, no copy, no remnant slots.

#ifndef STRETCH_CORE_H
#define STRETCH_CORE_H

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <atomic>

// Emilie Gillet's real FFT, MIT licensed (vendored, pinned in vendor/manifest.txt
// at pichenettes/stmlib@d18def8; do NOT edit, `make vendor-check` fails on drift).
#include "vendor_stmlib/shy_fft.h"
using stmlib::ShyFFT;
using stmlib::RotationPhasor;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

#ifndef SS_W
// Analysis window MAX (power of two). The runtime window is any power of two in
// [SS_W_MIN, SS_W]; 16384 (~0.34 s at 48 kHz) reaches PaulXStretch's shimmer
// regime (#136). Every frame buffer is allocated at this size.
#define SS_W 16384
#endif
#ifndef SS_W_DEFAULT
// Boot window. Host tests and firmware must agree (a static_assert in
// dreamosc.cpp pins FRAME_DEFAULT_IDX to this). 4096 = the settled default.
#define SS_W_DEFAULT 4096
#endif
#define SS_W_MIN 256
#define SS_STEPS 8
// Number of window sizes: SS_W, SS_W/2, ... SS_W_MIN. Tables are built for each.
#define SS_NSIZES 7
// HARD render ceiling: the most GATED heads (sounding + incoming + ringing) at
// once. 6 is the bench-proven number at the old clock; the sequencer ditches the
// ringing head with the least left to admit a new one.
#define SS_RENDER_CAP 6
// Pool: cap + one armed (pre-warmed) + slack, so allocation never has to ditch
// just to arm. Memory is cheap.
#define SS_HEADS 10
// Frame buffers per head: old + cur + a staged pair + a refresh pair.
#define SS_FRAME_BUFS 6
#define SS_HEAD_FLOATS (SS_FRAME_BUFS * SS_W)
#define SS_POOL_FLOATS (SS_HEADS * SS_HEAD_FLOATS)
// Staged-frame descriptor queue depth per head (power of two). At most a
// required render + one refresh are ever pending, plus one stale entry from a
// life change; 8 leaves room to spare.
#define SS_DESCQ 8
// Output headroom: the phase-randomized signal's peaks exceed its RMS (~1.07
// measured); ~2 dB below full scale so the codec never clips.
#define SS_HEADROOM 0.8f

// Shared table storage sizes. Windows: one curve per size, stacked
// (SS_W + SS_W/2 + ... + SS_W_MIN). Blend/correction curves: one per HOP size.
#define SS_WIN_FLOATS (2 * SS_W - SS_W_MIN)
#define SS_HOP_FLOATS (SS_W - SS_W_MIN / 2)

// ---------------------------------------------------------------------------
// Shared scratch and tables. Defined by the platform (dreamosc.cpp, host).
//   gWork/gSpec  : FFT scratch, hit hard per frame -> AXI SRAM on device.
//   gWindows     : the 7 analysis window curves (Nasca (1-x^2)^1.25), ~127 KB.
//   gBlendA/C    : per-hop raised-cosine (A) and AM-correction (C) curves,
//                  ~64 KB each, read sequentially by the ISR.
// All tables are written ONCE in StretchTables::init() and never again, so a
// live frame-size change cannot race a render (the old fast-scroll bug class is
// gone by construction). Plain arrays: on device they sit in NOLOAD sections and
// are filled after SDRAM is powered; never constructed objects (CLAUDE.md #129).
// ---------------------------------------------------------------------------

typedef ShyFFT<float, SS_W, RotationPhasor> SSFFT;

extern float gWork[SS_W];
extern float gSpec[SS_W];
extern float gWindows[SS_WIN_FLOATS];
extern float gBlendA[SS_HOP_FLOATS];
extern float gBlendC[SS_HOP_FLOATS];
extern volatile uint32_t gUnderruns;   // frame holds (a head repeated a frame)

inline int ssLog2(int n) { int p = 0; while ((1 << p) < n) p++; return p; }

// Size index: 0 = SS_W, 1 = SS_W/2, ... SS_NSIZES-1 = SS_W_MIN.
inline int ssSizeIdx(int w) { return ssLog2(SS_W) - ssLog2(w); }
inline int ssSizeW(int idx) { return SS_W >> idx; }
// Offsets of size idx's tables inside the stacked arrays.
inline int ssWinOff(int idx) { return 2 * SS_W - (2 * SS_W >> idx); }
inline int ssHopOff(int idx) { return SS_W - (SS_W >> idx); }
// Clamp any request to a legal window: power of two in [SS_W_MIN, SS_W].
inline int ssClampW(int w) {
  if (w < SS_W_MIN) w = SS_W_MIN;
  if (w > SS_W) w = SS_W;
  int p = 1; while (p * 2 <= w) p *= 2;
  return p;
}

// Integer hash (lowbias32) and a two-word combiner; the per-frame phase seed is
// hash(lifeSeed, frameIdx) so a re-render of the same frame with the same
// controls is bit-identical regardless of scheduling.
inline uint32_t ssMix(uint32_t x) {
  x ^= x >> 16; x *= 0x7feb352dU; x ^= x >> 15; x *= 0x846ca68bU; x ^= x >> 16;
  return x;
}
inline uint32_t ssHash2(uint32_t a, uint32_t b) {
  uint32_t h = ssMix(a ^ 0x9E3779B9u);
  h = ssMix(h ^ b);
  return h ? h : 0x9E3779B9u;
}

struct StretchTables {
  SSFFT fft;
  float sinLut[1024];
  float synthGain[SS_NSIZES];   // per-size gain: undoes ShyFFT's *w and the window's mean

  void init() {
    fft.Init();
    for (int i = 0; i < 1024; i++)
      sinLut[i] = sinf(2.0f * (float)M_PI * i / 1024.0f);
    for (int s = 0; s < SS_NSIZES; s++) {
      int w = ssSizeW(s), h = w / 2;
      float* win = &gWindows[ssWinOff(s)];
      double wsum = 0.0;
      for (int i = 0; i < w; i++) {
        double x = -1.0 + 2.0 * i / (w - 1);
        win[i] = (float)pow(1.0 - x * x, 1.25);
        wsum += win[i];
      }
      synthGain[s] = (float)(1.0 / ((wsum / w) * w));
      // Canonical PaulXStretch hop blend: a runs 1 -> 0 across the hop (old
      // fades out, cur fades in); corr = the (1+1/sqrt2)/2 curve that fills the
      // expected mid-hop dip of an uncorrelated mix.
      float* A = &gBlendA[ssHopOff(s)];
      float* C = &gBlendC[ssHopOff(s)];
      const double hc = 0.853553390593;
      for (int p = 0; p < h; p++) {
        A[p] = (float)(0.5 + 0.5 * cos(M_PI * p / h));
        C[p] = (float)(hc - (1.0 - hc) * cos(2.0 * M_PI * p / h));
      }
    }
  }

  inline float sinAt(uint32_t idx) const { return sinLut[idx & 1023]; }
  inline float cosAt(uint32_t idx) const { return sinLut[(idx + 256) & 1023]; }
  inline float sinAtF(float idx) const {
    uint32_t i0 = (uint32_t)idx;
    float f = idx - (float)i0;
    float s0 = sinLut[i0 & 1023], s1 = sinLut[(i0 + 1) & 1023];
    return s0 + (s1 - s0) * f;
  }
  inline float cosAtF(float idx) const { return sinAtF(idx + 256.0f); }
};

extern StretchTables gTab;

// ---------------------------------------------------------------------------
// The source. A plain buffer plus its length; reads wrap, so every position is
// legal. fillWindowed() is the hot path: source * window into dst with at most
// one wrap split, no per-sample modulo.
// ---------------------------------------------------------------------------

struct Source {
  float* data = nullptr;
  uint32_t len = 0;
  inline float at(int32_t i) const {
    i %= (int32_t)len;
    if (i < 0) i += len;
    return data[i];
  }
  void fillWindowed(int64_t start, int n, const float* win, float* dst) const {
    int64_t L = (int64_t)len;
    int64_t s = start % L;
    if (s < 0) s += L;
    int i = 0;
    while (i < n) {
      int run = (int)(L - s);
      if (run > n - i) run = n - i;
      const float* p = data + s;
      for (int k = 0; k < run; k++) dst[i + k] = p[k] * win[i + k];
      i += run;
      s = 0;
    }
  }
};

// ---------------------------------------------------------------------------
// seamPhase - PURE seam-decision geometry (host-tested). For a dwell of
// durSamples with crossfade fraction fade (0..0.5): the incoming head goes live
// at `onset` and the two overlap for `fadeLen` samples. fade 0 => onset ==
// duration (raw cut, no overlap).
// ---------------------------------------------------------------------------
struct SeamGeom {
  uint32_t onset;
  uint32_t fadeLen;
};
inline SeamGeom seamGeom(uint32_t durSamples, float fade) {
  if (fade < 0.0f) fade = 0.0f;
  if (fade > 0.5f) fade = 0.5f;
  SeamGeom g;
  g.fadeLen = (uint32_t)(durSamples * fade);
  g.onset   = durSamples - g.fadeLen;
  return g;
}

// ---------------------------------------------------------------------------
// Head - one life at one step. ISR side: state transitions, the per-sample
// blend, frame rotation at hop boundaries. Main-loop side: rendering staged
// frames from the live controls. Every field is single-writer; the two sides
// meet only through the descriptor queue and a few atomics.
// ---------------------------------------------------------------------------

class Head {
 public:
  enum State : uint8_t { FREE = 0, ARMED, READY, GATED, RINGING };
  enum Kind : int8_t { SINGLE = 0, PAIR = 1 };
  enum Want : uint8_t { NONE = 0, REQUIRED, REFRESH };

  // A staged render: what to apply at the next boundary (or at go-live).
  struct Staged {
    uint32_t life;
    uint32_t frameIdx;     // index of the frame that becomes `cur`
    int8_t   kind;         // SINGLE: old=cur, cur=a. PAIR: old=a, cur=b.
    int8_t   a, b;
    uint8_t  sizeIdx;
    double   travel;       // source-sample travel of the frame in `cur`
    float    stretchUsed;
    float    posUsed;
  };

  void setBuffers(float* slice) {
    for (int i = 0; i < SS_FRAME_BUFS; i++) buf_[i] = slice + (size_t)i * SS_W;
  }

  void reset(const Source* src) {
    src_ = src;
    state_.store(FREE, std::memory_order_relaxed);
    life_.store(0, std::memory_order_relaxed);
    oldIdx_.store(-1, std::memory_order_relaxed);
    curIdx_.store(-1, std::memory_order_relaxed);
    applied_.store(0, std::memory_order_relaxed);
    appliedSlot_.store(0, std::memory_order_relaxed);
    qHead_.store(0, std::memory_order_relaxed);
    qTail_.store(0, std::memory_order_relaxed);
    remain_.store(0, std::memory_order_relaxed);
    due_.store(0, std::memory_order_relaxed);
    phase_ = 0; h_ = 0; old_ = cur_ = nullptr; A_ = C_ = nullptr;
    seenLife_ = 0;
  }

  // ---- ISR side ----------------------------------------------------------

  // Main-loop reads use acquire (they pair with the ISR's release stores).
  State state() const { return (State)state_.load(std::memory_order_acquire); }
  // ISR-side reads: the ISR is the sole writer of every transition it acts on
  // (the main loop only writes the READY hint, which goLive() does not trust),
  // so a relaxed load is correct and avoids a DMB per head per sample.
  State stateIsr() const { return (State)state_.load(std::memory_order_relaxed); }
  bool  isFree() const { return state() == FREE; }
  bool  isArmed() const { State s = state(); return s == ARMED || s == READY; }
  int   step() const { return step_; }
  uint32_t remain() const { return remain_.load(std::memory_order_relaxed); }

  // FREE -> ARMED. Drains any stale descriptors (the ISR is the queue consumer,
  // so it may pop at any time) so a new life never sees the old life's frames.
  void alloc(int step, uint32_t lifeSeed, float driftOff, uint32_t due) {
    step_ = step;
    lifeSeed_ = lifeSeed;
    driftOff_ = driftOff;
    due_.store(due, std::memory_order_relaxed);
    qTail_.store(qHead_.load(std::memory_order_acquire), std::memory_order_release);
    applied_.store(0, std::memory_order_relaxed);
    oldIdx_.store(-1, std::memory_order_relaxed);
    curIdx_.store(-1, std::memory_order_relaxed);
    old_ = cur_ = nullptr;
    phase_ = 0; h_ = 0;
    remain_.store(0, std::memory_order_relaxed);
    life_.fetch_add(1, std::memory_order_release);
    state_.store(ARMED, std::memory_order_release);
  }

  void setDue(uint32_t due) { due_.store(due, std::memory_order_relaxed); }

  // READY -> GATED: apply the staged pre-roll pair and open the gate. Returns
  // false if no valid pair is staged yet (READY is only a hint; the queue is
  // the truth) -- the caller then keeps the outgoing head sounding.
  bool goLive() {
    if (!applyStaged(/*requirePair=*/true)) return false;
    state_.store(GATED, std::memory_order_release);
    return true;
  }

  // GATED -> RINGING for `remain` samples (ring-out), or -> FREE.
  void retire(uint32_t ringout) {
    if (ringout > 0) {
      remain_.store(ringout, std::memory_order_relaxed);
      state_.store(RINGING, std::memory_order_release);
    } else {
      free();
    }
  }
  void free() {
    state_.store(FREE, std::memory_order_release);
    old_ = cur_ = nullptr;
    oldIdx_.store(-1, std::memory_order_relaxed);
    curIdx_.store(-1, std::memory_order_relaxed);
  }

  // One output sample of this head's stream (GATED or RINGING), then advance.
  // At the hop boundary rotate frames from the staged queue, or hold (repeat
  // the current frame) if nothing is staged -- never silence.
  inline float tick() {
    uint32_t p = phase_;
    float a = A_[p];
    float s = (old_[p] * a + cur_[h_ + p] * (1.0f - a)) * C_[p];
    if (++phase_ >= (uint32_t)h_) {
      if (!applyStaged(false)) {         // hold: repeat cur as old
        old_ = cur_;
        oldIdx_.store(curIdx_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        holds_++;
        gUnderruns++;
      }
      phase_ = 0;
    }
    return s;
  }
  // RINGING bookkeeping: one sample consumed; frees itself when spent.
  inline void ringTick() {
    uint32_t r = remain_.load(std::memory_order_relaxed);
    if (r <= 1) { remain_.store(0, std::memory_order_relaxed); free(); }
    else        remain_.store(r - 1, std::memory_order_relaxed);
  }

  // ---- main-loop side ----------------------------------------------------

  // Decide what this head wants rendered. `clock` is the ISR sample counter;
  // `w` the live window; `stretch`/`pos` the live controls. Fills `deadline`
  // (absolute sample). Also syncs main-loop bookkeeping with what the ISR has
  // applied since the last call.
  Want plan(uint32_t clock, int w, float stretch, float pos, uint32_t& deadline,
            uint32_t minRefreshGap) {
    State st = state();
    if (st == FREE) return NONE;
    uint32_t life = life_.load(std::memory_order_acquire);
    if (life != seenLife_) {             // new life: reset our bookkeeping
      seenLife_ = life;
      stagedFrame_ = 0; lastApplied_ = 0;
      curTravel_ = 0.0; curSizeIdx_ = -1;
      refreshesPending_ = 0; lastRenderClock_ = clock - minRefreshGap;
    }
    uint32_t applied = applied_.load(std::memory_order_acquire);
    if (applied != lastApplied_) {       // the ISR consumed a staged frame
      const Staged& d = q_[appliedSlot_.load(std::memory_order_relaxed) & (SS_DESCQ - 1)];
      curTravel_ = d.travel;
      curSizeIdx_ = d.sizeIdx;
      lastApplied_ = applied;
      stagedFrame_ = applied;            // anything else queued is stale now
      refreshesPending_ = 0;
    }
    bool armed = (st == ARMED || st == READY);
    if (armed) deadline = due_.load(std::memory_order_relaxed);
    else       deadline = clock + (h_ - phase_);
    // Ringing head about to expire before its next boundary: nothing to make.
    if (st == RINGING && remain() <= (uint32_t)(h_ - phase_)) return NONE;
    bool pending = stagedFrame_ > applied;
    if (!pending) return REQUIRED;
    // Refresh: controls moved materially since this frame was staged.
    if (refreshesPending_ >= 1) return NONE;
    if ((int32_t)(clock - lastRenderClock_) < (int32_t)minRefreshGap) return NONE;
    bool moved = (ssSizeIdx(w) != stagedSizeIdx_)
              || (stretch != stagedStretch_)
              || (fabsf(pos - stagedPos_) > 0.001f);
    return moved ? REFRESH : NONE;
  }

  // Render what plan() asked for. Returns the size index rendered (for cost
  // accounting), or -1 if nothing was published (life changed under us).
  int render(uint32_t clock, int w, float stretch, float pos) {
    uint32_t life = life_.load(std::memory_order_acquire);
    if (life != seenLife_) return -1;
    State st = state();
    if (st == FREE) return -1;
    int sizeIdx = ssSizeIdx(w);
    int h = w / 2;
    float st_ = stretch > 0.01f ? stretch : 0.01f;
    double hop = (double)h / (double)st_;
    uint32_t applied = lastApplied_;
    bool pending = stagedFrame_ > applied;
    uint32_t frameIdx = pending ? stagedFrame_ : applied + 1;

    Staged d;
    d.life = life;
    d.frameIdx = frameIdx;
    d.sizeIdx = (uint8_t)sizeIdx;
    d.stretchUsed = stretch;
    d.posUsed = pos;
    bool armed = (st == ARMED || st == READY);
    if (armed) {                         // pre-roll pair: frame 0 at -hop, frame 1 at 0
      d.kind = PAIR;
      d.travel = 0.0;
    } else if (sizeIdx != curSizeIdx_) { // size change: re-render a pair at the new size
      d.kind = PAIR;
      d.travel = curTravel_ + hop;
    } else {
      d.kind = SINGLE;
      d.travel = curTravel_ + hop;
    }
    // Free buffers: exclude old/cur (ISR) and everything still queued. Read the
    // queue tail FIRST, then old/cur: a pop between the two reads moves a
    // buffer from the queue into cur, and this order keeps it excluded either way.
    uint32_t tail = qTail_.load(std::memory_order_acquire);
    uint32_t head = qHead_.load(std::memory_order_relaxed);
    if (head - tail >= SS_DESCQ - 1) return -1;   // no queue room (never in practice)
    bool used[SS_FRAME_BUFS] = {false};
    for (uint32_t i = tail; i != head; i++) {
      const Staged& q = q_[i & (SS_DESCQ - 1)];
      if (q.a >= 0) used[q.a] = true;
      if (q.b >= 0) used[q.b] = true;
    }
    int oi = oldIdx_.load(std::memory_order_relaxed);
    int ci = curIdx_.load(std::memory_order_relaxed);
    if (oi >= 0) used[oi] = true;
    if (ci >= 0) used[ci] = true;
    int need = d.kind == PAIR ? 2 : 1, got = 0;
    int8_t pick[2] = {-1, -1};
    for (int i = 0; i < SS_FRAME_BUFS && got < need; i++)
      if (!used[i]) pick[got++] = (int8_t)i;
    if (got < need) return -1;           // cannot happen by construction (6 buffers)

    double base = (double)pos * (double)src_->len;
    if (d.kind == PAIR) {
      renderFrame(sizeIdx, base + d.travel - hop, ssHash2(lifeSeed_, frameIdx - 1), buf_[pick[0]]);
      renderFrame(sizeIdx, base + d.travel,       ssHash2(lifeSeed_, frameIdx),     buf_[pick[1]]);
      d.a = pick[0]; d.b = pick[1];
    } else {
      renderFrame(sizeIdx, base + d.travel, ssHash2(lifeSeed_, frameIdx), buf_[pick[0]]);
      d.a = pick[0]; d.b = -1;
    }
    // The life may have ended (or changed) during the render: then this frame
    // is for a dead life. Do not publish (plan() resets on the next pass).
    if (life_.load(std::memory_order_acquire) != life) return -1;
    if (state() == FREE) return -1;
    q_[head & (SS_DESCQ - 1)] = d;
    qHead_.store(head + 1, std::memory_order_release);
    // READY is a hint for the ISR (goLive validates the queue itself). CAS from
    // ARMED so a head the ISR freed meanwhile can never be marked READY.
    if (st == ARMED) {
      uint8_t expect = ARMED;
      state_.compare_exchange_strong(expect, (uint8_t)READY,
                                     std::memory_order_release,
                                     std::memory_order_relaxed);
    }
    if (pending) refreshesPending_++;
    stagedFrame_ = frameIdx;
    stagedSizeIdx_ = sizeIdx;
    stagedStretch_ = stretch;
    stagedPos_ = pos;
    lastRenderClock_ = clock;
    return sizeIdx;
  }

  // Live base position for this life: the step's position plus the drift
  // offset drawn at arm, clamped.
  float basePos(float stepPos) const {
    float p = stepPos + driftOff_;
    if (p < 0.0f) p = 0.0f;
    if (p >= 1.0f) p = 0.999999f;
    return p;
  }

  uint32_t holds() const { return holds_; }
  int hop() const { return h_; }
  // Would the next render at window w be a pair (two frames)? Armed heads
  // always render a pre-roll pair; a gated head does when the size changes.
  bool nextIsPair(int w) const {
    State st = state();
    return st == ARMED || st == READY || ssSizeIdx(w) != curSizeIdx_;
  }

 private:
  // ISR: pop everything queued, apply the LAST valid descriptor for the next
  // frame (a refresh supersedes the original). Returns false if none.
  bool applyStaged(bool requirePair) {
    uint32_t life = life_.load(std::memory_order_relaxed);
    uint32_t applied = applied_.load(std::memory_order_relaxed);
    uint32_t tail = qTail_.load(std::memory_order_relaxed);
    uint32_t head = qHead_.load(std::memory_order_acquire);
    int pick = -1;
    for (uint32_t i = tail; i != head; i++) {
      const Staged& d = q_[i & (SS_DESCQ - 1)];
      if (d.life != life || d.frameIdx != applied + 1) continue;
      if (requirePair && d.kind != PAIR) continue;
      pick = (int)i;
    }
    if (pick < 0) {
      if (!requirePair) qTail_.store(head, std::memory_order_release);
      return false;
    }
    const Staged& d = q_[(uint32_t)pick & (SS_DESCQ - 1)];
    int sizeIdx = d.sizeIdx;
    if (d.kind == PAIR) {
      old_ = buf_[d.a]; cur_ = buf_[d.b];
      oldIdx_.store(d.a, std::memory_order_relaxed);
      curIdx_.store(d.b, std::memory_order_relaxed);
    } else {
      old_ = cur_; cur_ = buf_[d.a];
      oldIdx_.store(curIdx_.load(std::memory_order_relaxed), std::memory_order_relaxed);
      curIdx_.store(d.a, std::memory_order_relaxed);
    }
    h_ = ssSizeW(sizeIdx) / 2;
    A_ = &gBlendA[ssHopOff(sizeIdx)];
    C_ = &gBlendC[ssHopOff(sizeIdx)];
    phase_ = 0;
    appliedSlot_.store((uint32_t)pick, std::memory_order_relaxed);
    applied_.store(applied + 1, std::memory_order_release);
    qTail_.store(head, std::memory_order_release);
    return true;
  }

  // Render one phase-randomized frame of size idx at source position `srcPos`
  // with phase seed `seed` into `target`. Window from the immutable table;
  // FFT staged in gWork/gSpec (AXI on device); gain folded into the bins.
  void renderFrame(int sizeIdx, double srcPos, uint32_t seed, float* target) {
    int w = ssSizeW(sizeIdx), passes = ssLog2(w);
    const float* win = &gWindows[ssWinOff(sizeIdx)];
    src_->fillWindowed((int64_t)floor(srcPos), w, win, gWork);
    gTab.fft.Direct(gWork, gSpec, passes);
    float* re = &gSpec[0];
    float* im = &gSpec[w / 2];
    float g = gTab.synthGain[sizeIdx];
    uint32_t rng = seed;
    for (int k = 1; k < w / 2; k++) {
      float mag = sqrtf(re[k] * re[k] + im[k] * im[k]) * g;
      rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
      uint32_t a = rng >> 22;                    // 0..1023
      re[k] = mag * gTab.cosAt(a);
      im[k] = mag * gTab.sinAt(a);
    }
    gSpec[0] = 0.0f;                              // DC
    gSpec[w / 2] = 0.0f;                          // Nyquist
    gTab.fft.Inverse(gSpec, gWork, passes);
    memcpy(target, gWork, (size_t)w * sizeof(float));
  }

  const Source* src_ = nullptr;
  float* buf_[SS_FRAME_BUFS] = {nullptr};

  // ISR-owned.
  std::atomic<uint8_t>  state_{FREE};
  std::atomic<uint32_t> life_{0};
  std::atomic<int>      oldIdx_{-1}, curIdx_{-1};
  std::atomic<uint32_t> applied_{0}, appliedSlot_{0};
  std::atomic<uint32_t> qTail_{0};
  std::atomic<uint32_t> remain_{0};
  std::atomic<uint32_t> due_{0};
  int      step_ = 0;
  uint32_t lifeSeed_ = 0;
  float    driftOff_ = 0.0f;
  uint32_t phase_ = 0;
  int      h_ = 0;
  float*   old_ = nullptr;
  float*   cur_ = nullptr;
  const float* A_ = nullptr;
  const float* C_ = nullptr;
  uint32_t holds_ = 0;

  // Main-loop-owned.
  Staged q_[SS_DESCQ];
  std::atomic<uint32_t> qHead_{0};
  uint32_t seenLife_ = 0;
  uint32_t stagedFrame_ = 0, lastApplied_ = 0;
  double   curTravel_ = 0.0;
  int      curSizeIdx_ = -1;
  int      stagedSizeIdx_ = -1;
  float    stagedStretch_ = 0.0f, stagedPos_ = -1.0f;
  int      refreshesPending_ = 0;
  uint32_t lastRenderClock_ = 0;
};

// ---------------------------------------------------------------------------
// Sequencer - the READ clock (ISR) and the render scheduler (main loop).
// ---------------------------------------------------------------------------

class Sequencer {
 public:
  float position[SS_STEPS] = {0.10f, 0.13f, 0.16f, 0.19f,
                              0.22f, 0.25f, 0.28f, 0.31f};
  float drift[SS_STEPS] = {0};
  float stretch = 50.0f;      // live: read at every render
  float duration = 4.0f;      // live, unquantized dwell in seconds
  float fade  = 0.0f;         // seam overlap 0..0.5; 0 = raw cut
  float ringout = 0.0f;       // seconds a departing head keeps sounding (0 = off)
  int   frameSize = SS_W_DEFAULT;   // live: read at every render
  int   activeSteps = SS_STEPS;

  void setFrame(int w) { frameSize = ssClampW(w); }
  void setSteps(int n) {
    if (n < 1) n = 1;
    if (n > SS_STEPS) n = SS_STEPS;
    activeSteps = n;
  }

  // pool: SS_POOL_FLOATS floats (SDRAM on device; plain data only).
  void init(const Source* src, float sampleRate, float* pool,
            uint32_t seed = 0x12345678u) {
    src_ = src;
    sr_ = sampleRate;
    seed_ = seed;
    driftRng_ = seed ^ 0x9E3779B9u;
    if (driftRng_ == 0) driftRng_ = 0x9E3779B9u;
    for (int i = 0; i < SS_HEADS; i++) {
      head_[i].setBuffers(pool + (size_t)i * SS_HEAD_FLOATS);
      head_[i].reset(src);
    }
    cur_ = inc_ = nxt_ = -1;
    nxtStep_ = 0;
    elapsed_ = 0; seamStart_ = 0; fadeLen_ = 0;
    clock_.store(0, std::memory_order_relaxed);
    late_ = 0;
    for (int s = 0; s < SS_NSIZES; s++) {
      int w = ssSizeW(s);
      // Seed the cost model at ~18 ms for 16384 at 48 kHz (bench, old clock):
      // cost ~ c * w * log2(w) samples. Conservative until measured.
      costSamples_[s] = (uint32_t)(0.003766 * w * ssLog2(w)) + 16;
    }
    minSlack_ = 0x7fffffff;
    refreshes_ = 0;
  }

  uint32_t durSamples() const {
    uint32_t n = (uint32_t)(duration * sr_ + 0.5f);
    return n < 1 ? 1 : n;
  }
  uint32_t ringoutSamples() const {
    float r = ringout < 0.0f ? 0.0f : (ringout > 16.0f ? 16.0f : ringout);
    return (uint32_t)(r * sr_ + 0.5f);
  }
  // One pass of all active steps plus a startup allowance (the first pair
  // render); host drive loops use this as a bounded length.
  uint32_t patternSamples() const {
    return 4096u + (uint32_t)activeSteps * durSamples();
  }

  // ---- ISR: audio ----------------------------------------------------------

  // Render n mono samples. Per-block constants are hoisted; the per-sample
  // work is the seam decision, the gated heads' blends, and the ring-out ticks.
  void render(float* out, int n) {
    uint32_t dur = durSamples();
    SeamGeom g = seamGeom(dur, fade);
    uint32_t ro = ringoutSamples();
    // Keep the armed head's deadline honest against a live duration change.
    if (nxt_ >= 0) {
      uint32_t c = clock_.load(std::memory_order_relaxed);
      uint32_t until = g.onset > elapsed_ ? (uint32_t)(g.onset - elapsed_) : 0u;
      head_[nxt_].setDue(c + until);
    }
    for (int i = 0; i < n; i++) out[i] = tick(g, ro);
  }
  inline float next() { float s; render(&s, 1); return s; }

  // ---- main loop: rendering ------------------------------------------------

  // One unit of work: the most urgent render. Returns true if it rendered.
  bool service() {
    uint32_t clock = clock_.load(std::memory_order_relaxed);
    int w = ssClampW(frameSize);
    int bestReq = -1, bestRef = -1;
    uint32_t reqDl = 0, refDl = 0;
    for (int i = 0; i < SS_HEADS; i++) {
      Head& h = head_[i];
      if (h.isFree()) continue;
      float pos = h.basePos(position[h.step()]);
      uint32_t dl;
      uint32_t gap = 4 * costSamples_[ssSizeIdx(w)];
      if (gap < 480) gap = 480;
      Head::Want want = h.plan(clock, w, stretch, pos, dl, gap);
      if (want == Head::REQUIRED) {
        if (bestReq < 0 || (int32_t)(dl - reqDl) < 0) { bestReq = i; reqDl = dl; }
      } else if (want == Head::REFRESH) {
        if (bestRef < 0 || (int32_t)(dl - refDl) < 0) { bestRef = i; refDl = dl; }
      }
    }
    // A refresh may run only with slack: the worst recent cost for this size,
    // per frame rendered, with a 2x margin. A refresh that would miss its
    // boundary is worse than none (the original frame plays; the change lands
    // next hop).
    uint32_t cost = costSamples_[ssSizeIdx(w)];
    bool refOk = false;
    if (bestRef >= 0) {
      int32_t slack = (int32_t)(refDl - clock);
      uint32_t need = cost * (head_[bestRef].nextIsPair(w) ? 2u : 1u);
      refOk = slack > (int32_t)(2 * need);
    }
    int pick = -1;
    bool refresh = false;
    if (bestReq >= 0) {
      // A gated head's required frame always wins (its deadline is within a
      // hop). An ARMED head's pre-roll pair is due a whole dwell away; letting
      // it pre-empt a slack-valid refresh would cost the knob a hop for no
      // gain, so it yields unless its due is within a few pair-costs.
      bool farArmed = head_[bestReq].isArmed()
                   && (int32_t)(reqDl - clock) > (int32_t)(8 * cost);
      if (farArmed && refOk) { pick = bestRef; refresh = true; }
      else                    pick = bestReq;
    } else if (refOk) {
      pick = bestRef; refresh = true;
    }
    if (pick < 0) return false;
    Head& h = head_[pick];
    int32_t slack = (int32_t)((refresh ? refDl : reqDl) - clock);
    if (slack < minSlack_) minSlack_ = slack;
    uint32_t t0 = clock_.load(std::memory_order_relaxed);
    int s = h.render(clock, w, stretch, h.basePos(position[h.step()]));
    uint32_t took = clock_.load(std::memory_order_relaxed) - t0;
    if (s >= 0) {
      // Recent max with slow decay: an outlier does not poison refreshes forever.
      uint32_t c = costSamples_[s];
      c -= c >> 6;
      if (took > c) c = took;
      costSamples_[s] = c < 16 ? 16 : c;
      if (refresh) refreshes_++;
    }
    return true;
  }

  // ---- diagnostics -----------------------------------------------------------

  int activeVoices() const {      // gated STEP heads (sounding + incoming)
    return (cur_ >= 0 ? 1 : 0) + (inc_ >= 0 ? 1 : 0);
  }
  int activeRemnants() const {    // ringing heads
    int n = 0;
    for (int i = 0; i < SS_HEADS; i++) if (head_[i].state() == Head::RINGING) n++;
    return n;
  }
  int armedHeads() const {
    int n = 0;
    for (int i = 0; i < SS_HEADS; i++) {
      Head::State s = head_[i].state();
      if (s == Head::ARMED || s == Head::READY) n++;
    }
    return n;
  }
  uint32_t holds() const {
    uint32_t n = 0;
    for (int i = 0; i < SS_HEADS; i++) n += head_[i].holds();
    return n;
  }
  uint32_t lateSamples() const { return late_; }
  int curStep() const { return cur_ >= 0 ? head_[cur_].step() : -1; }
  int curHop() const { return cur_ >= 0 ? head_[cur_].hop() : 0; }
  int freeHeads() const {
    int n = 0;
    for (int i = 0; i < SS_HEADS; i++) if (head_[i].isFree()) n++;
    return n;
  }
  uint32_t refreshes() const { return refreshes_; }
  uint32_t costSamples(int sizeIdx) const { return costSamples_[sizeIdx]; }
  // Min slack (samples to deadline at render start) since last call; resets.
  int32_t takeMinSlack() { int32_t m = minSlack_; minSlack_ = 0x7fffffff; return m; }
  uint32_t clock() const { return clock_.load(std::memory_order_relaxed); }

 private:
  inline float tick(const SeamGeom& g, uint32_t ro) {
    float sum = 0.0f;
    uint32_t c = clock_.load(std::memory_order_relaxed);

    if (cur_ < 0) {                       // startup: first head not live yet
      if (nxt_ < 0) armNext(c, g.onset, 0);
      if (nxt_ >= 0 && admit() && head_[nxt_].goLive()) {
        cur_ = nxt_; nxt_ = -1;
        elapsed_ = 0;
        armNext(c, g.onset, (head_[cur_].step() + 1) % activeSteps);
      } else {
        late_++;
        clock_.store(c + 1, std::memory_order_relaxed);
        return 0.0f;
      }
    }

    // Seam: the incoming head goes live at onset, once it is ready and the cap
    // admits it. If it is not ready the outgoing head keeps sounding (late,
    // never silent).
    if (inc_ < 0 && elapsed_ >= g.onset) {
      if (nxt_ >= 0 && nxtStep_ >= activeSteps) {   // steps shrank under the arm
        head_[nxt_].free(); nxt_ = -1;
      }
      if (nxt_ < 0) armNext(c, g.onset, (head_[cur_].step() + 1) % activeSteps);
      if (nxt_ >= 0 && admit() && head_[nxt_].goLive()) {
        inc_ = nxt_; nxt_ = -1;
        fadeLen_ = g.fadeLen;
        seamStart_ = elapsed_;
        int after = (head_[inc_].step() + 1) % activeSteps;
        if (fadeLen_ == 0) {              // raw cut: outgoing stops now
          head_[cur_].retire(ro);
          enforceCap();
          cur_ = inc_; inc_ = -1;
          elapsed_ = 0;
        }
        armNext(c, g.onset, after);
      } else {
        late_++;
      }
    }

    // Read the seam.
    if (inc_ >= 0) {
      uint32_t fp = (uint32_t)(elapsed_ - seamStart_);
      if (fp >= fadeLen_) {               // seam complete: retire outgoing
        head_[cur_].retire(ro);
        enforceCap();
        cur_ = inc_; inc_ = -1;
        elapsed_ = 0;                     // this sample is the new dwell's first
        sum = head_[cur_].tick();
        elapsed_++;
      } else {
        float t = 256.0f * (float)fp / (float)fadeLen_;
        float fin  = gTab.sinAtF(t);
        float fout = gTab.sinAtF(256.0f + t);
        sum = head_[inc_].tick() * fin + head_[cur_].tick() * fout;
        elapsed_++;
      }
    } else {
      sum = head_[cur_].tick();
      elapsed_++;
    }

    // Ring-out heads at full volume.
    for (int i = 0; i < SS_HEADS; i++) {
      if (head_[i].stateIsr() != Head::RINGING) continue;
      sum += head_[i].tick();
      head_[i].ringTick();
    }

    clock_.store(c + 1, std::memory_order_relaxed);
    float out = sum * SS_HEADROOM;
    if (out > 1.0f) out = 1.0f;
    else if (out < -1.0f) out = -1.0f;
    return out;
  }

  // Allocate and arm the head for `step`. Called the moment a step goes live
  // (with the step after the one that just went live), so the pre-warm lead is
  // the whole dwell.
  void armNext(uint32_t clock, uint32_t onset, int step) {
    int h = allocHead();
    if (h < 0) return;                    // retry next tick
    // Drift: one draw per life, from the sequencer's rng (deterministic).
    float off = 0.0f;
    float d = drift[step];
    if (d > 0.0f) {
      driftRng_ ^= driftRng_ << 13; driftRng_ ^= driftRng_ >> 17; driftRng_ ^= driftRng_ << 5;
      float u = (float)(driftRng_ >> 8) / 16777216.0f;
      off = (u * 2.0f - 1.0f) * d;
    }
    // Life seed from the (drift-adjusted) position at arm, so the same position
    // always yields the same phases: zero drift is a literal repeat.
    float p = position[step] + off;
    if (p < 0.0f) p = 0.0f;
    if (p >= 1.0f) p = 0.999999f;
    uint32_t lifeSeed = seed_ ^ (uint32_t)(p * 4294967295.0);
    uint32_t until = onset > elapsed_ ? (uint32_t)(onset - elapsed_) : 0u;
    head_[h].alloc(step, lifeSeed, off, clock + until);
    nxt_ = h; nxtStep_ = step;
  }

  // ISR-side pool bookkeeping (relaxed state reads: the ISR is the writer).
  int allocHead() {
    for (int i = 0; i < SS_HEADS; i++) if (head_[i].stateIsr() == Head::FREE) return i;
    int o = oldestRinging();
    if (o >= 0) { head_[o].free(); return o; }
    return -1;
  }
  int oldestRinging() const {             // the ringing head with the least left
    int best = -1; uint32_t least = 0;
    for (int i = 0; i < SS_HEADS; i++) {
      if (head_[i].stateIsr() != Head::RINGING) continue;
      uint32_t r = head_[i].remain();
      if (best < 0 || r < least) { best = i; least = r; }
    }
    return best;
  }
  int gatedCount() const {
    int n = 0;
    for (int i = 0; i < SS_HEADS; i++) {
      Head::State s = head_[i].stateIsr();
      if (s == Head::GATED || s == Head::RINGING) n++;
    }
    return n;
  }
  // Make room for one more gated head under SS_RENDER_CAP.
  bool admit() {
    while (gatedCount() >= SS_RENDER_CAP) {
      int o = oldestRinging();
      if (o < 0) return false;
      head_[o].free();
    }
    return true;
  }
  void enforceCap() {
    while (gatedCount() > SS_RENDER_CAP) {
      int o = oldestRinging();
      if (o < 0) return;
      head_[o].free();
    }
  }

  const Source* src_ = nullptr;
  Head head_[SS_HEADS];
  float sr_ = 48000.0f;
  uint32_t seed_ = 0, driftRng_ = 1;
  // ISR-owned clock state.
  std::atomic<uint32_t> clock_{0};
  uint64_t elapsed_ = 0;         // samples the current dwell has sounded
  uint64_t seamStart_ = 0;       // elapsed_ at which the current seam began
  uint32_t fadeLen_ = 0;
  int cur_ = -1, inc_ = -1, nxt_ = -1;
  int nxtStep_ = 0;
  uint32_t late_ = 0;
  // Main-loop-owned scheduler state.
  uint32_t costSamples_[SS_NSIZES];
  int32_t  minSlack_ = 0x7fffffff;
  uint32_t refreshes_ = 0;
};

#endif  // STRETCH_CORE_H
