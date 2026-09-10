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
#include "sd_source.h"
#include "codec_rate.h"
#include "axisram.h"      // AXISRAM_DATA: plain globals in D1 AXI SRAM (see the header)

using namespace daisy;

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
// Worst single audio callback since the last print (ISR writes the max, the
// main loop reads then zeroes it; a callback landing between those two is the
// one value that can be lost -- acceptable for a per-second diagnostic).
static volatile uint32_t profIsrMaxTicks = 0;
static uint32_t profTicksPerUs = 1;
// Boot fingerprint: CRC-32 of a fixed-config render taken before audio starts
// (renderFingerprint in stretch_core.h). Two firmware builds that print the
// same crc render the same samples on the board; the host goldens cannot say
// that (the M7 fuses multiply-adds). Fixed config, independent of the boot
// values: stretch 50x, dwell 0.5 s, fade 0.5, 3 steps, window 4096, 1.5 s.
static uint32_t profBootCrc = 0;
static constexpr uint32_t PROF_CRC_SAMPLES = 72000;
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
// The Sequencer stays in internal SRAM. It is a C++ object with member
// initializers and Head sub-objects (atomics); objects in .sdram_bss get NEITHER
// their constructor run NOR their storage zeroed (the section is NOLOAD and SDRAM
// is not even powered until Init()), so a Sequencer placed there boots with
// garbage state and produces no sound (#129). The Head objects are small; only
// their big buffers (the pool below) go to SDRAM.
static Sequencer seq;

// Per-head frame buffers (SS_FRAME_BUFS per head, each SS_W floats) and each
// head's window cache (4 * SS_W floats, the run of source it renders from)
// live in SDRAM: SS_HEADS * 10 * 16384 floats ~= 6.5 MB. A plain array, NOT
// an object -- .sdram_bss is NOLOAD and SDRAM is unpowered at static-init
// time, so constructors never run and storage is not zeroed there.
// Sequencer::init() carves this up and hands each Head a slice; a frame
// buffer is always fully written by a render before the ISR can reference
// it, and a cache run is fetched before it is read, so NOLOAD garbage never
// reaches the output. The Head objects themselves stay in SRAM.
static float DSY_SDRAM_BSS voicePool[SS_POOL_FLOATS];

static DaisyPod pod;

// --- source audio -----------------------------------------------------------
// The source is a WAV on the microSD, STREAMED: each head's window cache asks
// SdSource::read() for the run it is about to render, and the frames come
// off the card then (sector-aligned DMA into an AXI staging buffer, decoded
// to float; sd_source.h). No length limit, no copy of the file in RAM. The
// SdSource object is small (a handle, a WavInfo) and constructed normally
// in .bss; its DMA targets are AXI SRAM statics inside sd_source.h.
//
// There is NO fallback source. No card, no WAV, an unsupported format, an
// unreadable file, or a rate the codec cannot take is "no instrument":
// haltNoInstrument() below. A wrong rate or the wrong material is worse than
// silence with a reason on the SRC line.
static stretchsd::SdSource sdSrc;

// The open file: rate, length, format, the name, and why an open failed --
// printed as the PROFILE SRC line. The boot crc= renders this material, so
// crc is comparable only between builds whose SRC lines match.
static SourceInfo srcInfo;

// The codec's rate: the material's own (codec_rate.h applies it through PLL3
// and reads it back). `measured` is what the registers say the codec runs
// at; it is the sample rate the Sequencer is initialised with, and it is the
// only rate anything downstream ever sees.
static CodecRate codecRate;

static bool audioRunning = false;

#ifdef PROFILE
static void profilePrintSource();
#endif

// No instrument: the source would not open, or the codec would not take its
// rate. Audio never starts (or has been stopped); both LEDs blink red; under
// PROFILE the SRC line says why, once a second.
static void haltNoInstrument() {
  if (audioRunning) { pod.StopAudio(); pod.StopAdc(); audioRunning = false; }
  uint32_t last = System::GetNow();
  bool on = true;
  for (;;) {
    uint32_t now = System::GetNow();
    if (now - last >= 500) {
      last = now; on = !on;
#ifdef PROFILE
      if (on) profilePrintSource();
#endif
    }
    float r = on ? 1.0f : 0.0f;
    pod.led1.Set(r, 0.0f, 0.0f);
    pod.led2.Set(r, 0.0f, 0.0f);
    pod.UpdateLeds();
    System::Delay(1);   // the LEDs are software PWM; keep stepping them
  }
}

// Open a file as THE source and bring the codec to its rate. `name` is a
// root-directory file name, or nullptr for List[0] (the first WAV the
// directory lists). Re-entrant: audio and the ADC are stopped first if they
// run (a PLL relock under a running stream would glitch; PLL3 R clocks the
// ADC), the previous file is closed, and the Sequencer is re-initialised at
// the new rate -- every head's cache is reset with it, so nothing reads the
// old file. Public controls (stretch, duration, fade, window, steps,
// positions) persist across an open; the sequence restarts at step 0.
// Returns false with srcInfo.err / codecRate.err set; audio is then stopped
// and the caller halts (no fallback).
static bool openSource(const char* name) {
  if (audioRunning) { pod.StopAudio(); pod.StopAdc(); audioRunning = false; }
  bool ok = name ? sdSrc.open(name, srcInfo) : sdSrc.openFirst(srcInfo);
  if (!ok) return false;
  if (!setCodecRate(pod.seed, sdSrc.rate, codecRate)) return false;
  seq.init(&sdSrc, (float)codecRate.measured, voicePool);
  return true;
}

// Audio and the panel: the instrument is live from here.
static void AudioCallback(AudioHandle::InterleavingInputBuffer in,
                          AudioHandle::InterleavingOutputBuffer out, size_t size);
static void startInstrument() {
  pod.StartAdc();
  pod.StartAudio(AudioCallback);   // the panel is read in the callback from here on
  audioRunning = true;
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
// Read from the AUDIO CALLBACK (see processControls below), which then owns
// every control value the Sequencer reads. The encoder's page + index state,
// the stretch detent table, the per-page dispatch (applyDigitalControls /
// applyEncoder) and the led2 level (pageBrightness) all live in
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

// Timestamp of the last encoder detent, for the shared fast/slow speed
// detection (encoderFast). Carried across page changes (a click then a quick
// detent on the new page reads as fast).
static uint32_t lastDetentMs = 0;

// --- the panel is read from the AUDIO CALLBACK -------------------------------
// processControls() runs at the top of every audio callback (1.5 kHz at block
// 32), before render(). That is where Electrosmith's own Pod examples read the
// panel, and for this firmware it is the only placement that works: the main
// loop is busy with non-preemptible 1.2-29 ms FFT renders, and libDaisy's
// encoder debounce needs the A phase low on two CONSECUTIVE 1 ms samples, so a
// poll that can only run between renders drops or mis-signs detents (the
// Daisy forum has the same failure with an OLED refresh blocking the loop).
// A timer IRQ was the first fix; it needed a watchdog and a fallback to be
// safe, and reading the panel where the audio already runs needs neither:
// if the audio ISR is not running there is no instrument anyway.
//
// Consequence: the ISR owns EVERY control value the Sequencer reads
// (stretch, position[], duration, fade, frameSize, activeSteps), written here
// before render() so they are constant across a block; the main loop only
// reads them to render frames (THEORY_OF_OPERATION.md, section 6).
//
// Rates. The debouncers self-limit to one sample per GetNow() millisecond, so
// calling them at 1.5 kHz is exactly right (never a skipped ms, edges valid
// on this call only). The knobs are sampled once per ms too, on purpose: the
// smoother (smoothKnob) and the pickup speed detector (potFast) are per-call
// filters tuned at 1 kHz; sampling them at the callback rate would change how
// duration and drift feel under the hand. The LEDs are libDaisy SOFTWARE PWM
// (Led::Update steps a ramp per call) and want the steady callback rate.
// Cost: a handful of GPIO reads and float ops; isr_max on the COST line is
// the measurement.
static uint32_t lastKnobMs = 0;
#ifdef PROFILE
static volatile uint32_t profDetents = 0;   // detents applied since the last print
#endif

static void processControls() {
  // --- encoder click / turn, buttons: one debounce sample per ms; the edge
  // flags are valid only on this call, and the pure decision logic applies
  // them in the same order the old poll did (click before detent). ---
  pod.ProcessDigitalControls();
  uint32_t ms = System::GetNow();
  int det = applyDigitalControls(enc, seq, panel,
                                 pod.encoder.RisingEdge(), (int)pod.encoder.Increment(),
                                 pod.button1.RisingEdge(), pod.button2.RisingEdge(),
                                 ms, lastDetentMs);
#ifdef PROFILE
  profDetents += (uint32_t)det;
#else
  (void)det;
#endif

  // --- knobs, once per ms: GLOBAL mode -> duration + global drift; step mode
  // -> that step's position + per-step drift. PICKUP everywhere (a knob takes
  // over only after it moves since arriving on a slot). All decision logic is
  // host-tested (controls_core.h). seq.duration is written by PanelEditor via
  // &seq.duration. ---
  if (ms != lastKnobMs) {
    lastKnobMs = ms;
    pod.ProcessAnalogControls();
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
  }

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
// SRC line: the open file -- its own sample rate (rate), the codec's rate as
// READ BACK from the registers (fs; the two must agree: the codec runs at
// the material's rate), the codec-rate error code (codec_rate.h; 0 = ok),
// the board revision libDaisy detected (0 = Seed, 1 = Seed 1.1 / WM8731, 2 =
// Seed 2 DFM / PCM3060), the format (1 PCM / 3 float), container bits,
// channels in the file (channel 0 plays), the length in frames, the card
// clock that initialised (spd 0 = FAST 50 MHz, 1 = STANDARD 25 MHz), why the
// open failed (err: SourceErr codes in source_core.h; 0 = ok) and the file
// name. crc= is comparable only between builds with the same SRC line.
// Printed on its own from haltNoInstrument() too, so a refusal is visible.
static void profilePrintSource() {
  pod.seed.PrintLine("SRC rate=%u fs=%u fs_err=%d board=%d fmt=%u bits=%u ch=%u len=%u spd=%d err=%d file=%s",
                     (unsigned)srcInfo.rate,
                     (unsigned)codecRate.measured, (int)codecRate.err,
                     (int)pod.seed.CheckBoardVersion(),
                     (unsigned)srcInfo.format, (unsigned)srcInfo.bits, (unsigned)srcInfo.channels,
                     (unsigned)srcInfo.len, (int)srcInfo.speed, (int)srcInfo.err, srcInfo.name);
}

// FETCH line: the streaming source's traffic this second. miss = whole
// cache re-fetches (a fresh life, a position scrub), ext = look-ahead
// top-ups as heads travel, fail = reads the card refused (zero-filled:
// must stay 0), smp = ISR samples the main loop spent inside fetches (this
// time is SUBTRACTED from the render cost model), max = the worst single
// render's fetch in samples (a scrub at 16384 is the case to watch against
// the hop: 8192 samples at that size), rd = SdSource::read() calls, kb =
// frame bytes delivered. A head is starved (du on HLTH) only if smp + the
// render exceed the slack; this line says which half it was.
static uint32_t profLastMiss = 0, profLastExt = 0, profLastFail = 0, profLastFetch = 0, profLastReads = 0;
static uint64_t profLastBytes = 0;
static void profilePrintFetch() {
  Sequencer::CacheStats c = seq.cacheStats();
  uint32_t fetch = seq.fetchSamples();
  uint32_t reads = sdSrc.reads();
  uint64_t bytes = sdSrc.bytes();
  pod.seed.PrintLine("FETCH miss=%u ext=%u fail=%u smp=%u max=%u rd=%u kb=%u",
                     (unsigned)(c.misses - profLastMiss), (unsigned)(c.extends - profLastExt),
                     (unsigned)(c.failures - profLastFail), (unsigned)(fetch - profLastFetch),
                     (unsigned)seq.takeMaxFetch(),
                     (unsigned)(reads - profLastReads), (unsigned)((bytes - profLastBytes) / 1024));
  profLastMiss = c.misses; profLastExt = c.extends; profLastFail = c.failures;
  profLastFetch = fetch; profLastReads = reads; profLastBytes = bytes;
}

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
  profilePrintSource();
  // KNOB line: raw (r1/r2) and smoothed (k1/k2) knob reads (*1000) and whether
  // pickup has engaged on the current slot (k1L/k2L = 1 once the pot has moved
  // past threshold). If you turn a knob and k1L stays 0, pickup isn't detecting
  // the move; if k1L=1 but the value doesn't change, the write is the bug.
  // pk1/pk2 = PEAK per-poll knob speed since last print (*1000, i.e. per-mil
  // of full travel per poll). potFast threshold is 10 in these units (0.01).
  // Turn a knob and read pk to see what "fast" actually measures -> tune the
  // threshold. f1/f2 = the fast verdict at print time. det = encoder detents
  // applied this second (count them against the physical clicks: the L1
  // bench check), drop = panel events the IRQ queue refused, ever (must stay
  // bench check). b1/b2 = buttons held, as the last debounce sample saw them
  // (a single-byte read of ISR-owned state; diagnostics only).
  pod.seed.PrintLine(
      "KNOB r1=%d r2=%d k1=%d k2=%d k1L=%d k2L=%d pk1=%d pk2=%d f1=%d f2=%d b1=%d b2=%d det=%u",
      scaled(prof.r1, 1000.0f), scaled(prof.r2, 1000.0f),
      scaled(prof.k1, 1000.0f), scaled(prof.k2, 1000.0f),
      (int)panel.k1Live(), (int)panel.k2Live(),
      scaled(prof.pk1, 1000.0f), scaled(prof.pk2, 1000.0f),
      (int)panel.fast1(), (int)panel.fast2(),
      (int)pod.button1.Pressed(), (int)pod.button2.Pressed(),
      (unsigned)profDetents);
  prof.pk1 = prof.pk2 = 0.0f;   // reset peak for the next window
  profDetents = 0;
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
  // isr_max = worst single audio callback this second (the seam case: two
  // gated heads blending); crc = the boot fingerprint (see profBootCrc).
  uint32_t isrMax = profIsrMaxTicks;
  profIsrMaxTicks = 0;
  pod.seed.PrintLine(
      "COST 16384=%u 8192=%u 4096=%u 2048=%u 1024=%u 512=%u 256=%u stk=%u isr_max=%u crc=%08lx",
      (unsigned)seq.costSamples(0), (unsigned)seq.costSamples(1),
      (unsigned)seq.costSamples(2), (unsigned)seq.costSamples(3),
      (unsigned)seq.costSamples(4), (unsigned)seq.costSamples(5),
      (unsigned)seq.costSamples(6),
      (unsigned)profStackUsed(), (unsigned)profUs(isrMax),
      (unsigned long)profBootCrc);
  prof.lastUnder = gUnderruns; prof.lastClip = gClips;
  prof.lastLate = late; prof.lastRefresh = rfr;
  prof.busyTicks = prof.units = 0;
  prof.maxTicks = 0;   // reset the per-window peak (stk is a running high-water)
  profilePrintFetch();
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
  // The panel first, so every control value is settled before this block is
  // rendered against it (see processControls).
  processControls();
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
  uint32_t d = System::GetTick() - t0;
  profIsrTicksTotal += d;                        // ISR-only writer; main reads deltas
  if (d > profIsrMaxTicks) profIsrMaxTicks = d;
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

  // The source, before audio starts (#131): List[0] on the microSD, opened
  // and streamed (sd_source.h), and the codec brought to ITS rate
  // (codec_rate.h: PLL3 reprogrammed with audio and the ADC not yet running,
  // the rate read back from the registers). Card init, the mount and the
  // header parse are the boot delay. A refusal of any kind is no instrument.
  if (!openSource(nullptr)) haltNoInstrument();
  const float fs = (float)codecRate.measured;   // never pod.AudioSampleRate()'s nominal 48000

#ifdef PROFILE
  // Boot fingerprint (see profBootCrc): a fixed-config render with audio still
  // stopped, so service()/render() run strictly sequentially, exactly as the
  // host harness does. Then init() again so the audible state is untouched.
  seq.stretch = 50.0f; seq.duration = 0.5f; seq.fade = 0.5f;
  seq.setSteps(3); seq.setFrame(4096);
  profBootCrc = renderFingerprint(seq, monoBlock, (int)AUDIO_BLOCK, PROF_CRC_SAMPLES);
  seq.init(&sdSrc, fs, voicePool);
  gUnderruns = 0; gClips = 0;
#endif
  // Starting values; the knobs/encoder take over from here (see processControls).
  // Pickup applies from boot: a knob takes over its parameter only after it has
  // physically moved (PanelEditor), so these hold until the pots are touched.
  // EVERY public control is set here explicitly: init() leaves them alone by
  // design (they are the player's), so this list -- not init() -- is what
  // makes the boot state independent of the PROFILE fingerprint above (the
  // step count leaked once: df0af04 booted the PROFILE build with 3 steps).
  encoderSync(enc, seq);  // stretch 50x, window 4096 (the core's SS_W_DEFAULT)
  seq.duration = 1.0f;
  seq.fade     = 0.0f;    // butt-joint by default; raise fade for crossfade
  seq.setSteps(SS_STEPS);

  startInstrument();

  // Main loop: a pure producer. Render staged frames (earliest deadline
  // first), nothing else; the panel, the LEDs and every control value belong
  // to the audio callback (processControls), so a 29 ms render here never
  // delays a detent or a knob. A display, when it comes (#141), goes HERE:
  // its I2C frame is ~10 ms of bus time and cannot sit in the callback.
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
#ifdef PROFILE
    uint32_t now = System::GetNow();
    if (now - prof.lastPrint >= 1000) {
      prof.lastPrint = now;
      profilePrint();
    }
#endif
  }
}
