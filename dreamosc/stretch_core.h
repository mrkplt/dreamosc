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
// step goes live, so the lead is the whole dwell) and freed when its step ends.
// Heads are strictly sequential -- at most two sound at once, and only during a
// seam crossfade (cur + inc); no adoption, no copy, no lingering tails.

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
// Boot window. Host tests and firmware must agree: the firmware's frame index
// is DERIVED from this (controls_core.h FRAME_DEFAULT_IDX = ssSizeIdx of it),
// so they cannot drift apart. 4096 = the settled default.
#define SS_W_DEFAULT 4096
#endif
#define SS_W_MIN 256
static_assert(SS_W_DEFAULT >= SS_W_MIN && SS_W_DEFAULT <= SS_W
              && (SS_W_DEFAULT & (SS_W_DEFAULT - 1)) == 0,
              "SS_W_DEFAULT must be a power of two in [SS_W_MIN, SS_W]");
// Render cost model seed: cost(w) ~ SS_COST_COEFF * w * log2(w) ISR samples per
// frame. 0.003766 reproduces ~18 ms at 16384 / 48 kHz, the bench number at the
// old 400 MHz clock (the 480 MHz bench measures ~14.5 ms; the seed is
// deliberately conservative and is replaced by measurement as renders run).
// ONE constant: the firmware seeds from it and the test harness's costed
// producer charges from it, so host scheduling tests run against the cost the
// device starts with.
#define SS_COST_COEFF 0.003766
#define SS_STEPS 8
// Number of window sizes: SS_W, SS_W/2, ... SS_W_MIN. Tables are built for each.
#define SS_NSIZES 7
// Pool: at most three heads are live at once (current + incoming at a seam +
// one armed/pre-warmed for the next step); the rest is slack so allocation
// never has to steal. Memory is cheap. (static_assert below pins the relation.)
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

// Worst-case live occupancy is 3: current + incoming (during a seam) + one
// armed head pre-warmed for the next step. The pool is larger so allocHead()
// always finds a FREE slot immediately, with headroom to spare.
static_assert(SS_HEADS >= 4, "pool must cover cur + inc + armed with slack");
static_assert(SS_FRAME_BUFS >= 6, "old + cur + staged pair + refresh pair");

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
extern volatile uint32_t gClips;       // output samples the +-1 clamp caught
                                       // (the phase-randomized peaks of two
                                       // heads at a seam can overrun
                                       // SS_HEADROOM -- distortion, not density)

// Test-only preemption hooks. The host suite is single-threaded, so an ISR
// firing in the middle of a main-loop routine can only be simulated by calling
// into the ISR side at a named point. Compiled out unless SS_TEST_HOOKS is
// defined (the test build defines it; firmware and host_main do not).
//   1: Head::render(), between the oldIdx_ and curIdx_ loads (ctx = Head*)
//   2: Sequencer::service(), between plan() and render()   (ctx = Head*)
#ifdef SS_TEST_HOOKS
extern void (*gSsTestHook)(int id, void* ctx);
#define SS_HOOK(id, ctx) do { if (gSsTestHook) gSsTestHook((id), (void*)(ctx)); } while (0)
#else
#define SS_HOOK(id, ctx) do {} while (0)
#endif

// constexpr so the firmware can derive its frame-index constants from them.
constexpr int ssLog2(int n) { int p = 0; while ((1 << p) < n) p++; return p; }

// Size index: 0 = SS_W, 1 = SS_W/2, ... SS_NSIZES-1 = SS_W_MIN.
constexpr int ssSizeIdx(int w) { return ssLog2(SS_W) - ssLog2(w); }
constexpr int ssSizeW(int idx) { return SS_W >> idx; }
// Offsets of size idx's tables inside the stacked arrays.
constexpr int ssWinOff(int idx) { return 2 * SS_W - (2 * SS_W >> idx); }
constexpr int ssHopOff(int idx) { return SS_W - (SS_W >> idx); }
// Clamp any request to a legal window: power of two in [SS_W_MIN, SS_W].
inline int ssClampW(int w) {
  if (w < SS_W_MIN) w = SS_W_MIN;
  if (w > SS_W) w = SS_W;
  int p = 1; while (p * 2 <= w) p *= 2;
  return p;
}
// Clamp an active step count to [1, SS_STEPS] (#149). The one place this
// range lives; setSteps(), the panel nav and the steps LED all use it.
inline int ssClampSteps(int n) {
  return n < 1 ? 1 : (n > SS_STEPS ? SS_STEPS : n);
}
// A source position in [0, 1): the last representable float below 1 keeps
// `pos * len` inside the buffer.
inline float ssClampPos(float p) {
  return p < 0.0f ? 0.0f : (p >= 1.0f ? 0.999999f : p);
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
  enum State : uint8_t { FREE = 0, ARMED, READY, GATED };
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
  };

  static bool isArmedState(State s) { return s == ARMED || s == READY; }

  // What to render, given where the head is and the slack it has -- the
  // pure decision behind render(), including the F5 size-change policy:
  //   armed            -> the pre-roll PAIR at the wanted size (travel 0:
  //                       frame 0 sits at -hop, frame 1 at 0);
  //   size changed     -> a PAIR at the new size if it can make this boundary
  //                       (or nothing can: slack is already a full hop, so
  //                       waiting gains nothing), else a SINGLE at the OLD
  //                       size now (no hold) and the pair renders right after
  //                       the boundary with a whole hop of slack;
  //   otherwise        -> the next SINGLE at the current size.
  // `stretchSafe` is the live stretch floored away from zero; the travel is
  // half the chosen window (one hop of output) divided by it.
  struct Choice { int8_t kind; int sizeIdx; double travel; };
  static Choice chooseRender(bool armed, int wantSizeIdx, int curSizeIdx, double curTravel,
                             uint32_t slack, uint32_t costNew, int hop, float stretchSafe) {
    Choice c;
    if (armed) {
      c.kind = PAIR; c.sizeIdx = wantSizeIdx; c.travel = 0.0;
      return c;
    }
    if (wantSizeIdx != curSizeIdx) {
      uint32_t pairCost = 2 * costNew;
      bool fits = slack > pairCost;
      bool atMaxSlack = slack + 64 >= (uint32_t)hop;
      if (fits || atMaxSlack) { c.kind = PAIR;   c.sizeIdx = wantSizeIdx; }
      else                    { c.kind = SINGLE; c.sizeIdx = curSizeIdx; }
    } else {
      c.kind = SINGLE; c.sizeIdx = wantSizeIdx;
    }
    c.travel = curTravel + (double)(ssSizeW(c.sizeIdx) / 2) / (double)stretchSafe;
    return c;
  }

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
    phase_ = 0; h_ = 0; old_ = cur_ = nullptr; A_ = C_ = nullptr;
    holds_ = 0;
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
  bool  isArmed() const { return isArmedState(state()); }
  int   step() const { return step_; }

  // FREE -> ARMED. Drains any stale descriptors (the ISR is the queue consumer,
  // so it may pop at any time) so a new life never sees the old life's frames.
  void alloc(int step, uint32_t lifeSeed, float driftOff) {
    step_ = step;
    lifeSeed_ = lifeSeed;
    driftOff_ = driftOff;
    qTail_.store(qHead_.load(std::memory_order_acquire), std::memory_order_release);
    applied_.store(0, std::memory_order_relaxed);
    oldIdx_.store(-1, std::memory_order_relaxed);
    curIdx_.store(-1, std::memory_order_relaxed);
    old_ = cur_ = nullptr;
    phase_ = 0; h_ = 0;
    life_.fetch_add(1, std::memory_order_release);
    state_.store(ARMED, std::memory_order_release);
  }

  // READY -> GATED: apply the staged pre-roll pair and open the gate. Returns
  // false if no valid pair is staged yet (READY is only a hint; the queue is
  // the truth) -- the caller then keeps the outgoing head sounding.
  bool goLive() {
    if (!applyStaged(/*requirePair=*/true)) return false;
    state_.store(GATED, std::memory_order_release);
    return true;
  }

  void free() {
    state_.store(FREE, std::memory_order_release);
    old_ = cur_ = nullptr;
    oldIdx_.store(-1, std::memory_order_relaxed);
    curIdx_.store(-1, std::memory_order_relaxed);
  }

  // One output sample of this GATED head's stream, then advance. At the hop
  // boundary rotate frames from the staged queue, or hold (repeat the current
  // frame) if nothing is staged -- never silence.
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

  // ISR, once per block: drop every queued descriptor that a later one
  // supersedes (same next frame, newer render) and every stale one. The ISR
  // owns qTail_, so popping here is its right. This is what lets a head take
  // any number of control-driven refreshes while it waits (F4): the queue
  // never holds more than the one descriptor that will actually be applied,
  // so the buffer budget (old, cur, that descriptor's pair, one more pair)
  // always has room for the next refresh.
  void isrDrainSuperseded() {
    uint32_t tail = qTail_.load(std::memory_order_relaxed);
    uint32_t head = qHead_.load(std::memory_order_acquire);
    if (tail == head) return;
    uint32_t life = life_.load(std::memory_order_relaxed);
    uint32_t applied = applied_.load(std::memory_order_relaxed);
    uint32_t keep = head;                // default: nothing valid, drop all
    for (uint32_t i = tail; i != head; i++) {
      const Staged& d = qAt(i);
      if (d.life == life && d.frameIdx == applied + 1) keep = i;
    }
    if (keep != tail) qTail_.store(keep, std::memory_order_release);
  }

  // ---- main-loop side ----------------------------------------------------

  // Sync main-loop bookkeeping with what the ISR has applied. Called at the
  // top of plan() AND render() (F7): a boundary between the two must not let
  // render() stage a frame the ISR has already played.
  void syncApplied(uint32_t clock) {
    uint32_t life = life_.load(std::memory_order_acquire);
    if (life != seenLife_) {             // new life: reset our bookkeeping
      seenLife_ = life;
      stagedFrame_ = 0; lastApplied_ = 0;
      curTravel_ = 0.0; curSizeIdx_ = -1;
      stagedSizeIdx_ = -1;
      lastRefreshClock_ = clock - 0x10000u;
    }
    uint32_t applied = applied_.load(std::memory_order_acquire);
    if (applied != lastApplied_) {       // the ISR consumed a staged frame
      const Staged& d = qAt(appliedSlot_.load(std::memory_order_relaxed));
      curTravel_ = d.travel;
      curSizeIdx_ = d.sizeIdx;
      lastApplied_ = applied;
      stagedFrame_ = applied;            // anything else queued is stale now
    }
  }

  // Decide what this head wants rendered. `clock` is the ISR sample counter;
  // `w` the live window; `stretch`/`pos` the live controls. Fills `deadline`
  // (absolute sample): the next hop boundary for a gated head, `armedDue`
  // (the sequencer's next go-live) for an armed one. `minRefreshGap`: at most
  // one control-driven refresh per this many samples per head (the caller
  // passes one hop, so a creeping knob costs one render per hop, never more
  // -- F8).
  Want plan(uint32_t clock, int w, float stretch, float pos, uint32_t& deadline,
            uint32_t minRefreshGap, uint32_t armedDue) {
    State st = state();
    if (st == FREE) return NONE;
    syncApplied(clock);
    uint32_t applied = lastApplied_;
    bool armed = isArmedState(st);
    if (armed) deadline = armedDue;
    else       deadline = clock + (h_ - phase_);
    bool pending = stagedFrame_ > applied;
    if (!pending) return REQUIRED;
    // Refresh: controls moved materially since this frame was staged. Room:
    // the ISR drains superseded descriptors per block, so the queue holds at
    // most the live one plus what we add; wait if two are already there.
    if (qHead_.load(std::memory_order_relaxed) - qTail_.load(std::memory_order_acquire) >= 2)
      return NONE;
    // The gap is measured from the last REFRESH (not the required render that
    // starts every hop), so the first turn after a boundary lands right away.
    if ((int32_t)(clock - lastRefreshClock_) < (int32_t)minRefreshGap) return NONE;
    // Position threshold 0.002 of the source: below a quarter window at the
    // default size, inaudible in a smeared cloud; above ADC jitter.
    bool moved = (ssSizeIdx(w) != stagedSizeIdx_)
              || (stretch != stagedStretch_)
              || (fabsf(pos - stagedPos_) > 0.002f);
    return moved ? REFRESH : NONE;
  }

  // Render what plan() asked for. `slack` = samples to this head's deadline,
  // `costNew` = the per-frame render cost estimate at window `w`. Returns the
  // number of frames rendered and published (1 or 2), or 0 if nothing was
  // published (life changed, no room). `sizeIdxOut` = the size rendered.
  int render(uint32_t clock, int w, float stretch, float pos,
             uint32_t slack, uint32_t costNew, int& sizeIdxOut) {
    syncApplied(clock);                  // F7: re-sync across a boundary
    uint32_t life = seenLife_;
    State st = state();
    if (st == FREE) return 0;
    int sizeIdx = ssSizeIdx(w);
    uint32_t applied = lastApplied_;
    bool pending = stagedFrame_ > applied;
    uint32_t frameIdx = pending ? stagedFrame_ : applied + 1;

    float st_ = stretch > 0.01f ? stretch : 0.01f;
    Choice c = chooseRender(isArmedState(st), sizeIdx, curSizeIdx_, curTravel_,
                            slack, costNew, h_, st_);
    sizeIdx = c.sizeIdx;
    Staged d;
    d.life = life;
    d.frameIdx = frameIdx;
    d.kind = c.kind;
    d.travel = c.travel;
    d.sizeIdx = (uint8_t)sizeIdx;
    double hop = (double)(ssSizeW(sizeIdx) / 2) / (double)st_;

    // Free buffers: exclude everything still queued and the ISR's old/cur.
    // Read order matters (F1): queue tail first, then CUR, then OLD. The only
    // ISR transitions that claim a buffer are a SINGLE pop (queued -> cur,
    // cur -> old) and a PAIR pop (queued, queued -> old, cur); a hold claims
    // nothing (old = cur). Reading cur before old means a pop between the two
    // loads leaves the buffer that just became old already captured as cur.
    uint32_t tail = qTail_.load(std::memory_order_acquire);
    uint32_t head = qHead_.load(std::memory_order_relaxed);
    if (head - tail >= SS_DESCQ - 1) return 0;    // no queue room (never in practice)
    bool used[SS_FRAME_BUFS] = {false};
    for (uint32_t i = tail; i != head; i++) {
      const Staged& q = qAt(i);
      if (q.a >= 0) used[q.a] = true;
      if (q.b >= 0) used[q.b] = true;
    }
    int ci = curIdx_.load(std::memory_order_relaxed);
    SS_HOOK(1, this);
    int oi = oldIdx_.load(std::memory_order_relaxed);
    if (oi >= 0) used[oi] = true;
    if (ci >= 0) used[ci] = true;
    int need = d.kind == PAIR ? 2 : 1, got = 0;
    int8_t pick[2] = {-1, -1};
    for (int i = 0; i < SS_FRAME_BUFS && got < need; i++)
      if (!used[i]) pick[got++] = (int8_t)i;
    if (got < need) return 0;            // cannot happen by construction (6 buffers)
#ifdef SS_TEST_HOOKS
    dbgLastPick_ = pick[0];
#endif

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
    // is for a dead life. Do not publish (syncApplied resets on the next pass).
    if (life_.load(std::memory_order_acquire) != life) return 0;
    if (state() == FREE) return 0;
    qAt(head) = d;
    qHead_.store(head + 1, std::memory_order_release);
    // READY is a hint for the ISR (goLive validates the queue itself). CAS from
    // ARMED so a head the ISR freed meanwhile can never be marked READY.
    if (st == ARMED) {
      uint8_t expect = ARMED;
      state_.compare_exchange_strong(expect, (uint8_t)READY,
                                     std::memory_order_release,
                                     std::memory_order_relaxed);
    }
    stagedFrame_ = frameIdx;
    stagedSizeIdx_ = sizeIdx;
    stagedStretch_ = stretch;
    stagedPos_ = pos;
    if (pending) lastRefreshClock_ = clock;
    sizeIdxOut = sizeIdx;
    return d.kind == PAIR ? 2 : 1;
  }

  // Live base position for this life: the step's position plus the drift
  // offset drawn at arm, clamped.
  float basePos(float stepPos) const { return ssClampPos(stepPos + driftOff_); }

  uint32_t holds() const { return holds_; }
  int hop() const { return h_; }
#ifdef SS_TEST_HOOKS
  // Test-only introspection (see SS_HOOK).
  int  dbgOldIdx() const { return oldIdx_.load(std::memory_order_relaxed); }
  int  dbgCurIdx() const { return curIdx_.load(std::memory_order_relaxed); }
  int  dbgLastPick() const { return dbgLastPick_; }
  bool dbgPending() const { return stagedFrame_ > lastApplied_; }
  int  dbgPendingKind() const { return qAt(qHead_.load(std::memory_order_relaxed) - 1).kind; }
  uint32_t dbgQueued() const { return qHead_.load(std::memory_order_relaxed) - qTail_.load(std::memory_order_relaxed); }
  // Queued descriptors the ISR will discard (frame already applied, or a
  // stale life): each one was a wasted render.
  int dbgStaleQueued() const {
    int n = 0;
    uint32_t applied = applied_.load(std::memory_order_relaxed);
    uint32_t life = life_.load(std::memory_order_relaxed);
    for (uint32_t i = qTail_.load(std::memory_order_relaxed);
         i != qHead_.load(std::memory_order_relaxed); i++) {
      const Staged& d = qAt(i);
      if (d.life != life || d.frameIdx <= applied) n++;
    }
    return n;
  }
#endif
  // Could the next render at window w be a pair (two frames)? Armed heads
  // always render a pre-roll pair; a gated head does when the size changes
  // (the worst case for budgeting -- chooseRender() may still fall back to a
  // SINGLE at the old size when the slack is short).
  bool nextIsPair(int w) const {
    return isArmedState(state()) || ssSizeIdx(w) != curSizeIdx_;
  }

 private:
  // The descriptor ring is indexed by an unbounded counter.
  Staged&       qAt(uint32_t i)       { return q_[i & (SS_DESCQ - 1)]; }
  const Staged& qAt(uint32_t i) const { return q_[i & (SS_DESCQ - 1)]; }

  // ISR: pop everything queued, apply the LAST valid descriptor for the next
  // frame (a refresh supersedes the original). Returns false if none.
  bool applyStaged(bool requirePair) {
    uint32_t life = life_.load(std::memory_order_relaxed);
    uint32_t applied = applied_.load(std::memory_order_relaxed);
    uint32_t tail = qTail_.load(std::memory_order_relaxed);
    uint32_t head = qHead_.load(std::memory_order_acquire);
    int pick = -1;
    for (uint32_t i = tail; i != head; i++) {
      const Staged& d = qAt(i);
      if (d.life != life || d.frameIdx != applied + 1) continue;
      if (requirePair && d.kind != PAIR) continue;
      pick = (int)i;
    }
    if (pick < 0) {
      if (!requirePair) qTail_.store(head, std::memory_order_release);
      return false;
    }
    const Staged& d = qAt((uint32_t)pick);
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
  uint32_t lastRefreshClock_ = 0;
#ifdef SS_TEST_HOOKS
  int      dbgLastPick_ = -1;
#endif
};

// ---------------------------------------------------------------------------
// CostModel - per-size render cost, in ISR samples per frame. Seeded from
// SS_COST_COEFF, then tracked as a recent MAX with a slow decay (an outlier
// does not poison refreshes forever; a quiet stretch lets it relax). The
// scheduler's slack and refresh rules are judged against it. Main-loop-owned.
// ---------------------------------------------------------------------------

struct CostModel {
  uint32_t samples[SS_NSIZES];

  void seed() {
    for (int s = 0; s < SS_NSIZES; s++) {
      int w = ssSizeW(s);
      samples[s] = (uint32_t)(SS_COST_COEFF * w * ssLog2(w)) + 16;
    }
  }
  uint32_t at(int sizeIdx) const { return samples[sizeIdx]; }
  void set(int sizeIdx, uint32_t v) {
    if (sizeIdx < 0 || sizeIdx >= SS_NSIZES) return;
    samples[sizeIdx] = v < 16 ? 16 : v;
  }
  // A render of `frames` frames at `sizeIdx` took `took` ISR samples. took ==
  // 0 means the clock did not move, i.e. no information (the host harness):
  // leave the estimate alone.
  void observe(int sizeIdx, uint32_t took, int frames) {
    if (frames <= 0 || took == 0) return;
    uint32_t per = took / (uint32_t)frames;
    uint32_t c = samples[sizeIdx];
    c -= c >> 6;
    if (per > c) c = per;
    samples[sizeIdx] = c < 16 ? 16 : c;
  }
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
  int   frameSize = SS_W_DEFAULT;   // live: read at every render
  int   activeSteps = SS_STEPS;

  void setFrame(int w) { frameSize = ssClampW(w); }
  void setSteps(int n) { activeSteps = ssClampSteps(n); }

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
    nextOnset_.store(0, std::memory_order_relaxed);
    late_ = 0;
    cost_.seed();
    minSlack_ = 0x7fffffff;
    refreshes_ = 0;
  }

  uint32_t durSamples() const {
    uint32_t n = (uint32_t)(duration * sr_ + 0.5f);
    return n < 1 ? 1 : n;
  }
  // One pass of all active steps plus a startup allowance (the first pair
  // render); host drive loops use this as a bounded length.
  uint32_t patternSamples() const {
    return 4096u + (uint32_t)activeSteps * durSamples();
  }

  // ---- ISR: audio ----------------------------------------------------------

  // The live controls a block is rendered against. The main loop is the only
  // writer of duration/fade/activeSteps and cannot preempt the ISR, so they
  // are constant across one render() anyway; snapshotting them makes that a
  // property of the code rather than of the caller, and is why tick() needs
  // no per-sample re-check of the step count.
  struct Block { SeamGeom g; int steps; };

  // Render n mono samples. Per-block constants are hoisted; the per-sample
  // work is the seam decision and the gated heads' blends.
  void render(float* out, int n) {
    Block b{seamGeom(durSamples(), fade), activeSteps};
    uint32_t c = clock_.load(std::memory_order_relaxed);
    // Per-block ISR housekeeping: drop superseded staged frames (F4), keep the
    // armed head's deadline honest against live duration/fade changes (F3),
    // and re-arm now if the step count shrank under the armed head (F9) so
    // the seam is not late. armNext() always arms a step below b.steps, so
    // after this check nxtStep_ < b.steps holds for the whole block.
    for (int i = 0; i < SS_HEADS; i++)
      if (head_[i].stateIsr() != Head::FREE) head_[i].isrDrainSuperseded();
    if (nxt_ >= 0 && nxtStep_ >= b.steps) {
      head_[nxt_].free(); nxt_ = -1;
      int from = inc_ >= 0 ? inc_ : cur_;
      if (from >= 0) armAfter(c, b, from);
    }
    if (nxt_ >= 0) nextOnset_.store(c + samplesToNextOnset(b.g.onset), std::memory_order_relaxed);
    for (int i = 0; i < n; i++) out[i] = tick(b);
  }
  inline float next() { float s; render(&s, 1); return s; }

  // ---- main loop: rendering ------------------------------------------------

  // Seed/override the per-size render cost estimate (samples per frame). The
  // firmware may seed it from bench numbers; the costed host harness pins it
  // to the cost it charges (F2), since the host ISR clock cannot measure it.
  void setCostEstimate(int sizeIdx, uint32_t samples) { cost_.set(sizeIdx, samples); }

  // The most urgent REQUIRED and REFRESH candidates, earliest deadline first
  // (strict: on equal deadlines the lower pool index wins, which is what keeps
  // render order -- and so the cost-model updates -- deterministic).
  struct Work {
    int req = -1, ref = -1;
    uint32_t reqDl = 0, refDl = 0;
  };
  Work pickWork(uint32_t clock, int w, uint32_t gap) {
    Work k;
    uint32_t armedDue = nextOnset_.load(std::memory_order_relaxed);
    for (int i = 0; i < SS_HEADS; i++) {
      Head& h = head_[i];
      if (h.isFree()) continue;
      float pos = h.basePos(position[h.step()]);
      uint32_t dl;
      Head::Want want = h.plan(clock, w, stretch, pos, dl, gap, armedDue);
      if (want == Head::REQUIRED) {
        if (k.req < 0 || (int32_t)(dl - k.reqDl) < 0) { k.req = i; k.reqDl = dl; }
      } else if (want == Head::REFRESH) {
        if (k.ref < 0 || (int32_t)(dl - k.refDl) < 0) { k.ref = i; k.refDl = dl; }
      }
    }
    return k;
  }

  // One unit of work: the most urgent render. Returns the number of frames
  // rendered (0 = nothing to do or nothing published).
  int service() {
    uint32_t clock = clock_.load(std::memory_order_relaxed);
    int w = ssClampW(frameSize);
    uint32_t cost = cost_.at(ssSizeIdx(w));
    // Refresh gap: one per hop per head, and never faster than the render
    // itself can turn around (F8).
    uint32_t gap = 4 * cost;
    if (gap < (uint32_t)(w / 2)) gap = (uint32_t)(w / 2);
    if (gap < 480) gap = 480;
    Work k = pickWork(clock, w, gap);
    // A refresh may run only with slack: the worst recent cost for this size,
    // per frame rendered, with a 2x margin. A refresh that would miss its
    // boundary is worse than none (the original frame plays; the change lands
    // next hop).
    bool refOk = false;
    if (k.ref >= 0) {
      int32_t slack = (int32_t)(k.refDl - clock);
      uint32_t need = cost * (head_[k.ref].nextIsPair(w) ? 2u : 1u);
      refOk = slack > (int32_t)(2 * need);
    }
    int pick = -1;
    bool refresh = false;
    if (k.req >= 0) {
      // A gated head's required frame always wins (its deadline is within a
      // hop). An ARMED head's pre-roll pair is due a whole dwell away; letting
      // it pre-empt a slack-valid refresh would cost the knob a hop for no
      // gain, so it yields unless its due is within a few pair-costs.
      bool farArmed = head_[k.req].isArmed()
                   && (int32_t)(k.reqDl - clock) > (int32_t)(8 * cost);
      if (farArmed && refOk) { pick = k.ref; refresh = true; }
      else                    pick = k.req;
    } else if (refOk) {
      pick = k.ref; refresh = true;
    }
    if (pick < 0) return 0;
    Head& h = head_[pick];
    SS_HOOK(2, &h);
    int32_t slack = (int32_t)((refresh ? k.refDl : k.reqDl) - clock);
    if (slack < minSlack_) minSlack_ = slack;
    uint32_t t0 = clock_.load(std::memory_order_relaxed);
    int s = -1;
    // basePos is recomputed here rather than carried from pickWork(): the ISR
    // may free and re-arm this head (a new life, a new drift offset) between
    // the two, and render() then publishes for the NEW life. Reading it at
    // render time is what keeps a stale offset from ever being staged.
    int frames = h.render(clock, w, stretch, h.basePos(position[h.step()]),
                          slack < 0 ? 0u : (uint32_t)slack, cost, s);
    uint32_t took = clock_.load(std::memory_order_relaxed) - t0;
    cost_.observe(s, took, frames);
    if (frames > 0 && refresh) refreshes_++;
    return frames;
  }

  // ---- diagnostics -----------------------------------------------------------

  int activeVoices() const {      // gated STEP heads (sounding + incoming)
    return (cur_ >= 0 ? 1 : 0) + (inc_ >= 0 ? 1 : 0);
  }
  int armedHeads() const {
    int n = 0;
    for (int i = 0; i < SS_HEADS; i++) if (head_[i].isArmed()) n++;
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
  uint32_t costSamples(int sizeIdx) const { return cost_.at(sizeIdx); }
  // Min slack (samples to deadline at render start) since last call; resets.
  int32_t takeMinSlack() { int32_t m = minSlack_; minSlack_ = 0x7fffffff; return m; }
  uint32_t clock() const { return clock_.load(std::memory_order_relaxed); }
#ifdef SS_TEST_HOOKS
  Head& dbgHead(int i) { return head_[i]; }
  int   dbgCur() const { return cur_; }
  int   dbgNxt() const { return nxt_; }
  uint32_t dbgNextOnset() const { return nextOnset_.load(std::memory_order_relaxed); }
#endif

 private:
  inline float tick(const Block& b) {
    float sum = 0.0f;
    uint32_t c = clock_.load(std::memory_order_relaxed);

    if (cur_ < 0) {                       // startup: first head not live yet
      if (nxt_ < 0) armNext(c, b.g.onset, 0);
      if (nxt_ >= 0 && head_[nxt_].goLive()) {
        cur_ = nxt_; nxt_ = -1;
        elapsed_ = 0;
        armAfter(c, b, cur_);
      } else {
        late_++;
        clock_.store(c + 1, std::memory_order_relaxed);
        return 0.0f;
      }
    }

    // Seam: the incoming head goes live at onset once it is ready. If it is not
    // ready the outgoing head keeps sounding (late, never silent). (A step
    // count shrunk under the armed head was already re-armed at block top.)
    if (inc_ < 0 && elapsed_ >= b.g.onset) {
      if (nxt_ < 0) armAfter(c, b, cur_);
      if (nxt_ >= 0 && head_[nxt_].goLive()) {
        int live = nxt_;
        inc_ = live; nxt_ = -1;
        fadeLen_ = b.g.fadeLen;
        seamStart_ = elapsed_;
        if (fadeLen_ == 0) retireOutgoing();   // raw cut: outgoing stops now
        armAfter(c, b, live);
      } else {
        late_++;
      }
    }

    // Read the seam.
    if (inc_ >= 0) {
      uint32_t fp = (uint32_t)(elapsed_ - seamStart_);
      if (fp >= fadeLen_) {               // seam complete
        retireOutgoing();                 // this sample is the new dwell's first
        sum = head_[cur_].tick();
        elapsed_++;
      } else {
        float t = 256.0f * (float)fp / (float)fadeLen_;
        float fin  = gTab.sinAtF(t);      // equal-power: sin in, cos out
        float fout = gTab.cosAtF(t);
        sum = head_[inc_].tick() * fin + head_[cur_].tick() * fout;
        elapsed_++;
      }
    } else {
      sum = head_[cur_].tick();
      elapsed_++;
    }

    clock_.store(c + 1, std::memory_order_relaxed);
    float out = sum * SS_HEADROOM;
    if (out > 1.0f) { out = 1.0f; gClips++; }
    else if (out < -1.0f) { out = -1.0f; gClips++; }
    return out;
  }

  // The incoming head becomes the sounding one; the outgoing head stops (a
  // raw cut or the end of a crossfade -- spectral, not amplitude).
  void retireOutgoing() {
    head_[cur_].free();
    cur_ = inc_; inc_ = -1;
    elapsed_ = 0;
  }

  // Arm the step after head `h`'s (wrapping within the block's step count).
  void armAfter(uint32_t clock, const Block& b, int h) {
    armNext(clock, b.g.onset, (head_[h].step() + 1) % b.steps);
  }

  // Allocate and arm the head for `step`. Called the moment a step goes live
  // (with the step after the one that just went live), so the pre-warm lead is
  // the whole dwell. Stamps the sequencer's next go-live (the armed head's
  // deadline); render() restates it every block against the live controls.
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
    float p = ssClampPos(position[step] + off);
    uint32_t lifeSeed = seed_ ^ (uint32_t)(p * 4294967295.0);
    head_[h].alloc(step, lifeSeed, off);
    nextOnset_.store(clock + samplesToNextOnset(onset), std::memory_order_relaxed);
    nxt_ = h; nxtStep_ = step;
  }

  // Samples until the NEXT go-live (the armed head's deadline). Outside a
  // seam: the rest of this dwell up to onset. Inside a seam (F3): the rest of
  // the fade, then the incoming head's whole dwell up to its onset.
  uint32_t samplesToNextOnset(uint32_t onset) const {
    if (inc_ >= 0) {
      uint32_t fp = (uint32_t)(elapsed_ - seamStart_);
      uint32_t left = fadeLen_ > fp ? fadeLen_ - fp : 0u;
      return left + onset;
    }
    return onset > elapsed_ ? (uint32_t)(onset - elapsed_) : 0u;
  }

  // ISR-side pool bookkeeping (relaxed state reads: the ISR is the writer). The
  // pool always has a FREE slot: at most 3 heads are live (cur + inc + armed)
  // and SS_HEADS covers that with slack, so allocation never has to steal.
  int allocHead() {
    for (int i = 0; i < SS_HEADS; i++) if (head_[i].stateIsr() == Head::FREE) return i;
    return -1;
  }

  const Source* src_ = nullptr;
  Head head_[SS_HEADS];
  float sr_ = 48000.0f;
  uint32_t seed_ = 0, driftRng_ = 1;
  // ISR-owned clock state.
  std::atomic<uint32_t> clock_{0};
  // The armed head's deadline: absolute sample of the next go-live. There is
  // exactly one armed head at a time (nxt_), so this is sequencer state, not
  // head state. Written at arm and restated per block (F3); read by plan().
  std::atomic<uint32_t> nextOnset_{0};
  uint64_t elapsed_ = 0;         // samples the current dwell has sounded
  uint64_t seamStart_ = 0;       // elapsed_ at which the current seam began
  uint32_t fadeLen_ = 0;
  int cur_ = -1, inc_ = -1, nxt_ = -1;
  int nxtStep_ = 0;
  uint32_t late_ = 0;
  // Main-loop-owned scheduler state.
  CostModel cost_;
  int32_t  minSlack_ = 0x7fffffff;
  uint32_t refreshes_ = 0;
};

// ---------------------------------------------------------------------------
// Fingerprint: render `samples` of a configured Sequencer at the device
// cadence (service() drained before every `n`-sample render(), the audio
// callback's shape) and CRC-32 the output's bit patterns. Same build + same
// config -> same CRC. The firmware prints it (PROFILE `crc=`) so two firmware
// builds can be compared ON THE BOARD, where the host goldens do not apply
// (the M7 build fuses multiply-adds; the host build does not). Leaves the
// Sequencer mid-stream: init() it again afterwards.
// ---------------------------------------------------------------------------

inline uint32_t ssCrc32(uint32_t crc, const void* data, size_t n) {
  const uint8_t* p = (const uint8_t*)data;
  for (size_t i = 0; i < n; i++) {
    crc ^= p[i];
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return crc;
}

inline uint32_t renderFingerprint(Sequencer& seq, float* block, int n, uint32_t samples) {
  uint32_t crc = 0xFFFFFFFFu;
  for (uint32_t done = 0; done < samples; done += (uint32_t)n) {
    for (int g = 0; g < 64 && seq.service(); g++) {}
    int m = (int)(samples - done < (uint32_t)n ? samples - done : (uint32_t)n);
    seq.render(block, m);
    crc = ssCrc32(crc, block, (size_t)m * sizeof(float));
  }
  return ~crc;
}

#endif  // STRETCH_CORE_H
