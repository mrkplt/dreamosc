// dreamosc.cpp - Daisy Pod firmware for the Stretch Sequencer.
//
// Defines the globals stretch_core.h externs, initializes the tables, runs
// Sequencer::render() in the audio callback and Sequencer::service() in the
// main loop. Frame model: the ISR blends straight out of each head's frame
// buffers and rotates them at hop boundaries; the main loop renders staged
// frames earliest-deadline-first. The head pool (SS_POOL_FLOATS) and the
// immutable window/blend tables live in SDRAM; the Sequencer OBJECT (atomics)
// stays in internal SRAM.

#include "daisy_pod.h"
#include "stretch_core.h"
#include "controls_core.h"
#include "source_core.h"

using namespace daisy;

// Place a plain (no-constructor) global in the AXI SRAM (D1, fast + L1-cacheable)
// via our own linker script's .axisram_bss section (dreamosc.lds). This is the
// region ST designates for large hot working sets that outgrow DTCM -- see
// hardware_spec.md. NOLOAD, so it is runtime-zeroed by our own memset, NOT the
// C runtime; only plain data, never constructed C++ objects.
#define AXISRAM_DATA __attribute__((section(".axisram_bss")))

// --- the globals stretch_core.h externs ------------------------------------
StretchTables gTab;
// FFT scratch. At SS_W 16384 these are 64 KB each -- too big for DTCM. They
// are PLAIN float arrays hit HARD by the per-frame FFT, so they
// go in AXI SRAM (fast + cacheable), NOT external SDRAM: the AXI region is what ST
// intends for exactly this (large fast working set). ShyFFT zero-fills them each
// pass, so NOLOAD is fine. (Earlier they were in DSY_SDRAM_BSS as a stopgap when
// the stock linker script had no AXI-SRAM section; owning dreamosc.lds fixed that.)
float AXISRAM_DATA gWork[SS_W];   // windowed frame; ShyFFT::Direct destroys its input
float AXISRAM_DATA gSpec[SS_W];   // split spectrum: real [0,W/2), imag [W/2,W)
// Immutable tables: the seven analysis windows (~127 KB) and the per-hop blend
// and AM-correction curves (~64 KB each). Written once by gTab.init() after
// SDRAM is powered, read sequentially (cached) by renders and the ISR, and
// never rewritten -- so a live frame-size change cannot race a render. Plain
// arrays in NOLOAD SDRAM, never constructed objects (CLAUDE.md #129).
float DSY_SDRAM_BSS gWindows[SS_WIN_FLOATS];
float DSY_SDRAM_BSS gBlendA[SS_HOP_FLOATS];
float DSY_SDRAM_BSS gBlendC[SS_HOP_FLOATS];

// Frame HOLDS: a head reached its hop boundary with no staged frame and
// repeated its current frame (spectrally the same, never silent). The one
// number that says the producer fell behind. Printed as `du` in PROFILE.
volatile uint32_t gUnderruns = 0;
volatile uint32_t gClips = 0;

#ifdef PROFILE
// `make PROFILE=1`: per-second CPU accounting over USB serial. Read it with
//   screen /dev/tty.usbmodem<tab> 115200
// HLTH fields: act (gated step heads, at most 2), arm (armed heads), free (FREE
// pool slots), units (service() calls that rendered), svc_us (main-loop DSP us
// that second), avg_us (per unit), max_us (WORST single render), isr_us
// (audio-callback us that second), du (frame holds that second), late (samples a
// seam waited for its incoming head), rfr (control-driven re-renders), clip
// (+-1 clamp hits that second), slack (min samples to deadline at render start,
// signed). A separate COST line carries the per-size render cost estimate in
// samples (16384 down to 256) and stk (deepest stack use seen, bytes).
//
// Timing is done in raw TIM2 ticks (System::GetTick) and converted per delta,
// because System::GetUs() wraps every 2^32/200e6 = 21.5 s and a delta across
// the wrap reads ~4.27e9 (the "32-bit wrap" in the old OPEN_ISSUES.md).
// ISR time is a running total the ISR alone writes; the main loop prints the
// delta since its last print (a shared "add then reset" was a lost update).
#include <stdio.h>   // snprintf for the POS/DRF lines
static volatile uint32_t profIsrTicksTotal = 0;
static uint32_t profTicksPerUs = 1;
static inline uint32_t profUs(uint32_t ticks) { return ticks / profTicksPerUs; }

// Deepest stack use observed (bytes below _estack). The stack grows down from
// _estack (top of DTCM); keeping the minimum MSP ever seen gives a high-water
// mark. This catches the pre-mortem's "hang, not underrun" case: freeing DTCM
// (94% -> 44%) helped, but a deep 16384-path local could still collide with the
// heap/statics below -- and that shows as a crash, not a number, unless we
// watch it. Sampled from BOTH the main loop (between service() calls) and the
// audio ISR: the ISR preempts a render at a random depth, so ITS read is what
// actually sees the render path's stack; the main-loop sample alone never did
// (it always ran between renders and read the same ~176 B at every frame
// size). ISR-written, main-read: volatile. _estack comes from dreamosc.lds.
extern "C" uint32_t _estack;
static volatile uint32_t profMinSp = 0xFFFFFFFFu;   // lowest SP ever seen
static inline void profSampleStack() {
  uint32_t sp = __get_MSP();
  if (sp < profMinSp) profMinSp = sp;
}
// Bytes from _estack down to the deepest SP = max stack depth used so far.
static inline uint32_t profStackUsed() {
  uint32_t top = (uint32_t)&_estack;
  return (profMinSp == 0xFFFFFFFFu) ? 0 : (top - profMinSp);
}

// Per-second accounting the main loop owns (the ISR only writes
// profIsrTicksTotal and profMinSp above).
struct Prof {
  uint32_t busyTicks = 0, units = 0;
  // Peak single service() duration this window: an under-fed head comes from
  // the WORST single render, which the average hides. Watch max_us at 16384.
  uint32_t maxTicks = 0;
  uint32_t lastUnder = 0, lastClip = 0, lastLate = 0, lastRefresh = 0, lastIsr = 0;
  uint32_t lastPrint = 0;
  float r1 = 0, r2 = 0, k1 = 0, k2 = 0;   // last knob reads (raw + smoothed)
  // Peak per-poll knob speed since the last print (instantaneous speed is ~0
  // at any given print instant; the peak catches an actual turn). Used to
  // tune the potFast threshold from board data.
  float pk1 = 0, pk2 = 0;
};
static Prof prof;

// Scaled-integer formatting: nano-newlib printf can't do floats reliably, so
// every float on the serial line is printed as round(v * k).
static inline int scaled(float v, float k) { return (int)(v * k + 0.5f); }
// Append n floats, scaled by k, space-separated, to buf; returns the new length.
static int appendScaled(char* buf, size_t cap, int used, const float* v, int n, float k) {
  for (int i = 0; i < n && used < (int)cap; i++)
    used += snprintf(buf + used, cap - used, "%s%d", i ? " " : "", scaled(v[i], k));
  return used;
}
#endif

// --- storage ---------------------------------------------------------------
#define SOURCE_SECONDS 10
#define SAMPLE_RATE    48000
#define SOURCE_LEN     (SOURCE_SECONDS * SAMPLE_RATE)

// Source audio lives in SDRAM (~1.9 MB) — never fits internal SRAM. A plain
// array (no constructor), so NOLOAD SDRAM is fine; we memset it before use.
static float DSY_SDRAM_BSS sourceBuf[SOURCE_LEN];

// The Sequencer stays in internal SRAM. It is a C++ object with member
// initializers and Head sub-objects (atomics); objects in .sdram_bss get NEITHER
// their constructor run NOR their storage zeroed (the section is NOLOAD and SDRAM
// is not even powered until Init()), so a Sequencer placed there boots with
// garbage state and produces no sound (#129). The Head objects are small; only
// their big buffers (the pool below) go to SDRAM.
static Sequencer seq;

// Per-head frame buffers (SS_FRAME_BUFS per head, each SS_W floats) live in
// SDRAM: SS_HEADS * 6 * 16384 floats ~= 3.9 MB. A plain array, NOT an object --
// .sdram_bss is NOLOAD and SDRAM is unpowered at static-init time, so
// constructors never run and storage is not zeroed there. Sequencer::init()
// carves this up and hands each Head a slice; a frame buffer is always fully
// written by a render before the ISR can reference it, so NOLOAD garbage never
// reaches the output. The Head objects themselves stay in SRAM.
static float DSY_SDRAM_BSS voicePool[SS_POOL_FLOATS];

static DaisyPod pod;
static Source   src;

// --- source audio -----------------------------------------------------------
// Sample material is uploaded separately to QSPI flash (8 MB, memory-mapped at
// 0x90000000) rather than embedded in the firmware: internal flash is only
// 128 KB and a usable sample is hundreds of KB. Build the blob with
// tools/wav2raw.py and upload it with `make program-sample`. The blob format
// and its decode (validate, int16 -> float, wrap-pad) are source_core.h,
// host-tested; only the address and the cast are hardware.
//
// QSPI layout, read from the Daisy bootloader's own DFU descriptor:
//   0x90000000  64 x 4KB   (256 KB) bootloader-reserved
//   0x90040000  60 x 64KB  (3.75 MB) firmware images live here
//   0x90400000  60 x 64KB  (3.75 MB) free
// Sample data goes in the THIRD region so it can never collide with a firmware
// image, even under APP_TYPE=BOOT_QSPI. Must match SAMPLE_ADDR in the Makefile.
// (Not named QSPI_BASE: stm32h750xx.h already defines that as the peripheral's
// 0x90000000 base, and redefining it was a warning on every build.)
#define SAMPLE_QSPI_ADDR 0x90400000u

// Load the QSPI sample into the SDRAM source buffer as float, wrap-padded to
// SOURCE_LEN. Returns false if no valid blob is present (never uploaded, or
// erased), so the caller can fall back to a synthesized source rather than
// playing garbage. The blob's sample rate is decoded but NOT applied (see the
// NOTE in source_core.h and OPEN_ISSUES.md).
static bool load_qspi_sample() {
  uint32_t rate = 0;
  return decodeSampleBlob((const uint8_t*)SAMPLE_QSPI_ADDR, sourceBuf, SOURCE_LEN, rate) > 0;
}

// --- controls ---------------------------------------------------------------
// Panel (2026 Daisy Pod): 2 knobs, encoder (turn + click), 2 buttons, 2 RGB LEDs.
// Two KNOB MODES (button1 cycles GLOBAL -> step1..8 -> GLOBAL; button2 = GLOBAL):
//   GLOBAL mode (led1 OFF):  knob1 -> duration (0.25..60s), knob2 -> global drift
//   step mode  (led1 ROYGBIVW): knob1 -> that step's position, knob2 -> its drift
//   encoder turn   -> the current page's parameter (stretch / fade / frame / steps)
//   encoder click  -> cycle page: stretch(red)/steps(orange)/fade(yellow)/window(green)
//                     each page's led2 brightness encodes that page's level
//   led2           -> encoder page color; brightness = that page's level
//
// PICKUP (soft takeover) EVERYWHERE: landing on GLOBAL or a step does NOT snap
// its value to the pot -- a knob takes over only after it physically moves since
// arriving. So you can tour steps (and hop to global) without disturbing values
// you don't touch. All the mode/pickup logic is the host-tested PanelEditor.
//
// Read from the MAIN LOOP, not the audio callback: debouncing and smoothing do
// not belong in an interrupt, and the sequencer reads these values live anyway.
// The encoder's page + index state, the stretch detent table, the per-page
// dispatch (applyEncoder) and the led2 level (pageBrightness) all live in
// controls_core.h (host-tested); this is just the state instance.
static EncoderState enc;

// Global drift: a fun all-steps shimmer, the FLOOR under each step's own
// per-step drift (knob2). Effective drift per step = max(perStep, global),
// clamped -- see foldDrift() in controls_core.h.
static float globalDrift = 0.0f;

// Panel edit state (mode/pickup/shadow) lives in the platform-free PanelEditor
// so it is host-testable — see controls_core.h. GLOBAL mode: knobs = duration
// + global drift. Step mode: knobs = that step's position + per-step drift.
static PanelEditor panel;

// Audio block size. The hop (>= 128 samples, 2048 at the default window) sets
// the control-to-ear floor, so a tiny block buys nothing; 32 (0.67 ms) keeps
// ISR entry overhead and jitter low and gives the main loop longer uninterrupted
// runs for its FFTs.
static constexpr size_t AUDIO_BLOCK = 32;
static float monoBlock[AUDIO_BLOCK];

// Knob smoothing state (the smoothing math is smoothKnob() in controls_core.h).
static float knobSmooth[2] = {0.0f, 0.0f};
static bool  knobPrimed    = false;

// Milliseconds since the last encoder detent on the current page, for the
// shared fast/slow speed detection (encoderFast). Reset when the page changes
// so a page switch doesn't read as a fast spin.
static uint32_t lastDetentMs = 0;

static void processControls() {
  pod.ProcessAllControls();

  // --- encoder click: cycle page (stretch/steps/fade/window) ---
  if (pod.encoder.RisingEdge()) enc.page = nextPage(enc.page);

  // --- button1: advance panel mode (GLOBAL -> step1..N -> GLOBAL) where N is
  // the active step count (#149) -- the tour only visits active steps ---
  if (pod.button1.RisingEdge()) panel.advance(seq.activeSteps);
  // --- button2: jump back to GLOBAL ---
  if (pod.button2.RisingEdge()) panel.toGlobal();

  // --- encoder turn: the current page's parameter (stretch / frame / steps /
  // fade). Speed from the detent GAP (Increment is only +-1); the per-page
  // effect is applyEncoder() in controls_core.h (host-tested). ---
  int32_t inc = pod.encoder.Increment();
  if (inc != 0) {
    uint32_t tnow = System::GetNow();
    bool fast = encoderFast(tnow - lastDetentMs);
    lastDetentMs = tnow;
    applyEncoder(enc, seq, panel, (int)inc, fast);
  }

  // --- knobs: GLOBAL mode -> duration + global drift; step mode -> that step's
  // position + per-step drift. PICKUP everywhere (a knob takes over only after
  // it moves since arriving on a slot). All decision logic is host-tested
  // (controls_core.h). seq.duration is written by PanelEditor via &seq.duration.
  float r1 = pod.knob1.Value(), r2 = pod.knob2.Value();   // raw (move detect)
  float k1 = smoothKnob(knobSmooth[0], r1, knobPrimed);   // smoothed (value)
  float k2 = smoothKnob(knobSmooth[1], r2, knobPrimed);
  knobPrimed = true;
  panel.update(seq, &seq.duration, &globalDrift, r1, r2, k1, k2);
#ifdef PROFILE
  prof.r1 = r1; prof.r2 = r2; prof.k1 = k1; prof.k2 = k2;   // for the KNOB line
  if (panel.speed1() > prof.pk1) prof.pk1 = panel.speed1();   // peak since last print
  if (panel.speed2() > prof.pk2) prof.pk2 = panel.speed2();
#endif

  // --- led2: encoder page color (RoYG over stretch/steps/fade/window, same
  // ROYGBIVW palette as led1) at that page's LEVEL (pageBrightness), so a
  // bright LED always means "this parameter is turned up".
  Rgb c2 = pageColor(enc.page, pageBrightness(enc, seq));
  pod.led2.Set(c2.r, c2.g, c2.b);

  // --- led1: OFF in GLOBAL mode; ROYGBIVW for the selected step otherwise ---
  if (panel.inGlobal()) {
    pod.led1.Set(0.0f, 0.0f, 0.0f);
  } else {
    Rgb c1 = stepColor(panel.step());
    pod.led1.Set(c1.r, c1.g, c1.b);
  }

  pod.UpdateLeds();
}

#ifdef PROFILE
// The once-a-second serial dump. Every variable parameter appears here (the
// profiler directive in CLAUDE.md); the format is the bench's diagnostic
// contract, so keep the field names stable. Lines stay under libDaisy's
// 128-byte Logger buffer (HLTH/COST were split for exactly that reason).
static void profilePrint() {
  uint32_t isrTotal = profIsrTicksTotal;
  uint32_t isr = profUs(isrTotal - prof.lastIsr);
  prof.lastIsr = isrTotal;
  // SETTINGS line: globals + which step is selected. Integers *1000 (or *100
  // for stretch). gdrift_cc: global drift in units of 0.01% (hundredths of a
  // percent) -> full scale 0..300 == 0..3% drift, so the fine 0.03% grid is
  // visible. dur_ms IS the step length (live, unquantized). hop = the sounding
  // head's current hop in samples (frame size as actually applied), step = the
  // step it is playing, blk = audio block.
  pod.seed.PrintLine(
      "SET stretch_c=%d dur_ms=%d gdrift_cc=%d fade_m=%d frame=%d hop=%d step=%d steps=%d blk=%u page=%d slot=%d",
      scaled(seq.stretch, 100.0f), scaled(seq.duration, 1000.0f),
      scaled(globalDrift, 10000.0f), scaled(seq.fade, 1000.0f),
      seq.frameSize,                         // requested (live control)
      seq.curHop(),                          // applied on the sounding head
      seq.curStep(),
      seq.activeSteps,                       // active step count (#149)
      (unsigned)AUDIO_BLOCK,
      (int)enc.page,
      panel.slot());                         // 0 = GLOBAL, 1..N = step
  // KNOB line: raw (r1/r2) and smoothed (k1/k2) knob reads (*1000) and whether
  // pickup has engaged on the current slot (k1L/k2L = 1 once the pot has moved
  // past threshold). If you turn a knob and k1L stays 0, pickup isn't detecting
  // the move; if k1L=1 but the value doesn't change, the write is the bug.
  // pk1/pk2 = PEAK per-poll knob speed since last print (*1000, i.e. per-mil
  // of full travel per poll). potFast threshold is 10 in these units (0.01).
  // Turn a knob and read pk to see what "fast" actually measures -> tune the
  // threshold. f1/f2 = the fast verdict at print time.
  pod.seed.PrintLine(
      "KNOB r1=%d r2=%d k1=%d k2=%d k1L=%d k2L=%d pk1=%d pk2=%d f1=%d f2=%d b1=%d b2=%d",
      scaled(prof.r1, 1000.0f), scaled(prof.r2, 1000.0f),
      scaled(prof.k1, 1000.0f), scaled(prof.k2, 1000.0f),
      (int)panel.k1Live(), (int)panel.k2Live(),
      scaled(prof.pk1, 1000.0f), scaled(prof.pk2, 1000.0f),
      (int)panel.fast1(), (int)panel.fast2(),
      (int)pod.button1.Pressed(), (int)pod.button2.Pressed());
  prof.pk1 = prof.pk2 = 0.0f;   // reset peak for the next window
  // POS line: all 8 step positions (*1000). Homing every knob should make
  // these equal; if they differ, that's why steps sound different.
  char line[LOGGER_BUFFER];
  int  used = snprintf(line, sizeof(line), "POS ");
  appendScaled(line, sizeof(line), used, seq.position, SS_STEPS, 1000.0f);
  pod.seed.PrintLine("%s", line);
  // DRF line: per-step drift shadow (what the knob set), then the EFFECTIVE
  // drift the DSP reads = max(perStep, global) -- global is the FLOOR, not an
  // addend. So a step whose own drift is ABOVE the floor reads its own value
  // (eff == shadow); a step BELOW the floor reads the floor (eff > shadow).
  // Units are 0.01% (hundredths of a percent), *10000, so the 0..3% range
  // reads 0..300 and the fine 0.03% grid is visible.
  float shadow[SS_STEPS];
  for (int i = 0; i < SS_STEPS; i++) shadow[i] = panel.perStepDrift(i);
  used = snprintf(line, sizeof(line), "DRF s ");
  used = appendScaled(line, sizeof(line), used, shadow, SS_STEPS, 10000.0f);
  used += snprintf(line + used, sizeof(line) - used, " | eff ");
  appendScaled(line, sizeof(line), used, seq.drift, SS_STEPS, 10000.0f);
  pod.seed.PrintLine("%s", line);
  // HEALTH line: CPU and supply accounting for this second. act = gated
  // step heads (sounding + incoming, at most 2); arm = armed (pre-warmed)
  // heads. du = frame HOLDS (a head repeated a frame: the producer fell
  // behind); late = samples a seam waited for an incoming head that was not
  // ready (the step ran long, never silent); rfr = re-renders driven by
  // control changes; clip = output samples the +-1 clamp caught this second
  // (the phase-randomized peaks can overrun SS_HEADROOM -> distortion, not
  // loudness); slack = min samples to deadline at render start this second
  // (negative = a render started past its boundary). free = FREE pool slots
  // (a leak shows here as a steady decline). max_us = worst single render. A
  // separate COST line carries the per-size render cost estimates in samples
  // (16384..256) and stk (deepest stack use); it split off HLTH because the
  // combined string overran libDaisy's 128-byte Logger buffer and truncated
  // cost[1..6] + stk.
  uint32_t busyUs = profUs(prof.busyTicks);
  uint32_t late = seq.lateSamples(), rfr = seq.refreshes();
  pod.seed.PrintLine(
      "HLTH act=%d arm=%d free=%d units=%u svc_us=%u avg_us=%u max_us=%u isr_us=%u du=%u late=%u rfr=%u clip=%u slack=%d",
      seq.activeVoices(), seq.armedHeads(), seq.freeHeads(),
      (unsigned)prof.units, (unsigned)busyUs,
      (unsigned)(prof.units ? busyUs / prof.units : 0), (unsigned)profUs(prof.maxTicks),
      (unsigned)isr, (unsigned)(gUnderruns - prof.lastUnder),
      (unsigned)(late - prof.lastLate), (unsigned)(rfr - prof.lastRefresh),
      (unsigned)(gClips - prof.lastClip),
      (int)seq.takeMinSlack());
  pod.seed.PrintLine(
      "COST 16384=%u 8192=%u 4096=%u 2048=%u 1024=%u 512=%u 256=%u stk=%u",
      (unsigned)seq.costSamples(0), (unsigned)seq.costSamples(1),
      (unsigned)seq.costSamples(2), (unsigned)seq.costSamples(3),
      (unsigned)seq.costSamples(4), (unsigned)seq.costSamples(5),
      (unsigned)seq.costSamples(6),
      (unsigned)profStackUsed());
  prof.lastUnder = gUnderruns; prof.lastClip = gClips;
  prof.lastLate = late; prof.lastRefresh = rfr;
  prof.busyTicks = prof.units = 0;
  prof.maxTicks = 0;   // reset the per-window peak (stk is a running high-water)
}
#endif

// --- audio -----------------------------------------------------------------
static void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                          AudioHandle::InterleavingOutputBuffer out,
                          size_t                                size) {
#ifdef PROFILE
  uint32_t t0 = System::GetTick();
  profSampleStack();   // sees the render this ISR preempted (see profMinSp)
#endif
  // size is the INTERLEAVED sample count (2 * block). Render mono in chunks
  // of at most AUDIO_BLOCK (per-block constants hoisted inside), then
  // duplicate to L/R. Chunking means a larger-than-expected block can never
  // leave the tail of `out` unwritten.
  size_t n = size / 2;
  for (size_t off = 0; off < n; off += AUDIO_BLOCK) {
    size_t m = n - off;
    if (m > AUDIO_BLOCK) m = AUDIO_BLOCK;
    seq.render(monoBlock, (int)m);
    for (size_t i = 0; i < m; i++) {
      out[2 * (off + i)]     = monoBlock[i];   // left
      out[2 * (off + i) + 1] = monoBlock[i];   // right
    }
  }
#ifdef PROFILE
  profIsrTicksTotal += System::GetTick() - t0;   // ISR-only writer; main reads deltas
#endif
}

int main(void) {
  // boost = true: 480 MHz. libDaisy's default config is 400 MHz; every earlier
  // max_us on record was taken at 400. Clock is cheap; latency is not.
  pod.Init(true);
  pod.SetAudioBlockSize(AUDIO_BLOCK);
#ifdef PROFILE
  pod.seed.StartLog(false);   // USB CDC; non-blocking so boot never stalls
  profTicksPerUs = System::GetTickFreq() / 1000000u;
  if (profTicksPerUs == 0) profTicksPerUs = 1;
#endif

  gTab.init();                       // ShyFFT + window/blend tables (SDRAM is up)
  // TEMPORARY (#132 testing): real material from QSPI so the controls can be
  // judged on broadband audio -- a sine has no spectral variation across the
  // buffer, so moving a read head sounds identical everywhere. Reverts to the
  // SD-card path (#131) once the controls are sorted. Either way the buffer is
  // SOURCE_LEN of material (the blob is wrap-padded to fill it).
  if (!load_qspi_sample()) fillStubSource(sourceBuf, SOURCE_LEN, SAMPLE_RATE);
  src.data = sourceBuf;
  src.len  = SOURCE_LEN;

  seq.init(&src, pod.AudioSampleRate(), voicePool);
  // Starting values; the knobs/encoder take over from here (see processControls).
  // Pickup applies from boot: a knob takes over its parameter only after it has
  // physically moved (PanelEditor), so these hold until the pots are touched.
  encoderSync(enc, seq);  // stretch 50x, window 4096 (the core's SS_W_DEFAULT)
  seq.duration = 1.0f;
  seq.fade     = 0.0f;    // butt-joint by default; raise fade for crossfade

  pod.StartAdc();
  pod.StartAudio(AudioCallback);

  // Main loop: render staged frames (earliest deadline first) and read the
  // panel. Controls are polled here rather than in the audio ISR -- debouncing
  // and smoothing do not belong in an interrupt.
  // Controls poll on a WALL-CLOCK 1 ms tick, NOT every-Nth-service(): a single
  // service() call is one or two full FFTs, so gating controls on a service
  // count polled the encoder only a few times a second and dropped detents.
  uint32_t lastControlMs = System::GetNow();
#ifdef PROFILE
  prof.lastPrint = System::GetNow();
#endif
  while (1) {
#ifdef PROFILE
    uint32_t s0 = System::GetTick();
    if (seq.service()) {
      uint32_t d = System::GetTick() - s0;   // wrap-safe in raw ticks
      prof.busyTicks += d;
      if (d > prof.maxTicks) prof.maxTicks = d;
      prof.units++;
    }
    profSampleStack();   // stack high-water mark (deepest SP seen)
#else
    seq.service();
#endif
    // Poll the panel on the 1 ms wall clock (see lastControlMs above).
    uint32_t nowMs = System::GetNow();
    if (nowMs != lastControlMs) {
      lastControlMs = nowMs;
      processControls();
    }
#ifdef PROFILE
    uint32_t now = System::GetNow();
    if (now - prof.lastPrint >= 1000) {
      prof.lastPrint = now;
      profilePrint();
    }
#endif
  }
}
