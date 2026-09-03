// dreamosc.cpp - Daisy Pod firmware for the Stretch Sequencer.
//
// Ticket #129: real audio path. Defines the globals stretch_core.h externs,
// initializes the FFT tables, runs Sequencer::next() in the audio callback and
// Sequencer::service() in the main loop. The source is a stubbed test tone until
// SD-card load lands (#131). Controls are not yet wired (#132) and voice-buffer
// SDRAM placement gets its proper budget pass in #130 — for now both the source
// and the Sequencer (which owns the 8 voice buffers, ~256 KB) live in SDRAM so
// the build fits.

#include "daisy_pod.h"
#include "stretch_core.h"
#include "controls_core.h"

using namespace daisy;

// --- the globals stretch_core.h externs ------------------------------------
StretchTables gTab;
float         gWork[SS_W];   // windowed frame; ShyFFT::Direct destroys its input
float         gSpec[SS_W];   // split spectrum: real [0,W/2), imag [W/2,W)

// #129 diagnostic: incremented in Voice::next() when a head's ring is empty
// (starved). led1 latches red if this ever moves — tells us on-device whether
// the boundary artifact is a buffer underrun vs. a DSP/splice issue.
volatile uint32_t gUnderruns = 0;

#ifdef PROFILE
// `make PROFILE=1`: per-second CPU accounting over USB serial. Read it with
//   screen /dev/tty.usbmodem<tab> 115200
// Fields: act (active voices), units (service() calls that did work), svc_us
// (us of main-loop DSP that second), avg_us (per unit), isr_us (us in the audio
// callback that second), du (underruns that second), fade_m (fade*1000),
// xf (crossfade on/off), page (0 stretch / 1 fade).
static volatile uint32_t profIsrUs = 0;
// Last knob reads (raw + smoothed) for the KNOB diagnostic line.
static float dbgR1 = 0, dbgR2 = 0, dbgK1 = 0, dbgK2 = 0;
// Peak per-poll knob speed since the last profiler print (instantaneous speed is
// ~0 at any given print instant; the peak catches an actual turn). *1000 in the
// line. Used to tune the potFast threshold from board data.
static float dbgPk1 = 0, dbgPk2 = 0;
#endif

// --- storage ---------------------------------------------------------------
#define SOURCE_SECONDS 10
#define SAMPLE_RATE    48000
#define SOURCE_LEN     (SOURCE_SECONDS * SAMPLE_RATE)

// Source audio lives in SDRAM (~1.9 MB) — never fits internal SRAM. A plain
// array (no constructor), so NOLOAD SDRAM is fine; we memset it before use.
static float DSY_SDRAM_BSS sourceBuf[SOURCE_LEN];

// The Sequencer stays in internal SRAM. It is a C++ object with member
// initializers and Voice sub-objects; objects in .sdram_bss get NEITHER their
// constructor run NOR their storage zeroed (the section is NOLOAD and SDRAM is
// not even powered until Init()), so a Sequencer placed there boots with garbage
// state and produces no sound. Its 8 voice buffers (~256 KB) fit in the 512 KB
// SRAM. #130 revisits placement/budget deliberately; #129 needs it to run.
static Sequencer seq;

// Voice working buffers (old_ + ring_ per voice, ~384 KB at SS_W 4096) live in
// SDRAM: far too big for the 512 KB internal SRAM once anything else is present.
// This is a plain array, NOT an object -- .sdram_bss is NOLOAD and SDRAM is
// unpowered at static-init time, so constructors never run and storage is not
// zeroed there. Sequencer::init() carves this up and hands each Voice a slice;
// the Voice objects themselves stay in SRAM where C++ works normally.
static float DSY_SDRAM_BSS voicePool[SS_POOL_FLOATS];

static DaisyPod pod;
static Source   src;

// --- source audio -----------------------------------------------------------
// Sample material is uploaded separately to QSPI flash (8 MB, memory-mapped at
// 0x90000000) rather than embedded in the firmware: internal flash is only
// 128 KB and a usable sample is hundreds of KB. Build the blob with
// tools/wav2raw.py and upload it with `make program-sample`.
//
// Blob layout, little-endian (see tools/wav2raw.py):
//   uint32 magic 'DRMO' | uint32 count | uint32 rate | uint32 reserved
//   int16  samples[count]
// QSPI layout, read from the Daisy bootloader's own DFU descriptor:
//   0x90000000  64 x 4KB   (256 KB) bootloader-reserved
//   0x90040000  60 x 64KB  (3.75 MB) firmware images live here
//   0x90400000  60 x 64KB  (3.75 MB) free
// Sample data goes in the THIRD region so it can never collide with a firmware
// image, even under APP_TYPE=BOOT_QSPI.
#define QSPI_BASE        0x90400000u
#define SAMPLE_MAGIC     0x4F4D5244u   // 'DRMO'

struct SampleHeader {
  uint32_t magic;
  uint32_t count;
  uint32_t rate;
  uint32_t reserved;
};

// Load the QSPI sample into the SDRAM source buffer as float. Returns false if
// no valid blob is present (never uploaded, or erased), so the caller can fall
// back to a synthesized source rather than playing garbage.
static bool load_qspi_sample() {
  const SampleHeader* h = (const SampleHeader*)QSPI_BASE;
  if (h->magic != SAMPLE_MAGIC) return false;
  if (h->count == 0 || h->count > 8u * 1024 * 1024) return false;

  const int16_t* pcm = (const int16_t*)(QSPI_BASE + sizeof(SampleHeader));
  uint32_t n = h->count > SOURCE_LEN ? SOURCE_LEN : h->count;
  for (uint32_t i = 0; i < n; i++) sourceBuf[i] = pcm[i] / 32768.0f;
  // Wrap-pad the remainder so the whole buffer is musical material rather than
  // a block of silence the read heads can wander into.
  for (uint32_t i = n; i < SOURCE_LEN; i++) sourceBuf[i] = sourceBuf[i % n];
  src.len = SOURCE_LEN;
  return true;
}

// Fallback when QSPI holds no sample: a spectrally-varied synthetic source, so
// the position/stretch controls still audibly do something.
static void fill_stub_source() {
  for (uint32_t i = 0; i < SOURCE_LEN; i++) {
    float t = (float)i / SAMPLE_RATE;
    sourceBuf[i] = 0.5f * sinf(2.0f * (float)M_PI * 220.0f * t)
                 + 0.3f * sinf(2.0f * (float)M_PI * 331.0f * t)
                 + 0.2f * sinf(2.0f * (float)M_PI * 554.0f * t);
  }
}

// --- controls ---------------------------------------------------------------
// Panel (2026 Daisy Pod): 2 knobs, encoder (turn + click), 2 buttons, 2 RGB LEDs.
// Two KNOB MODES (button1 cycles GLOBAL -> step1..8 -> GLOBAL; button2 = GLOBAL):
//   GLOBAL mode (led1 OFF):  knob1 -> duration (0.25..60s), knob2 -> global drift
//   step mode  (led1 ROYGBIVW): knob1 -> that step's position, knob2 -> its drift
//   encoder turn   -> the current page's parameter (stretch / fade / frame / steps)
//   encoder click  -> cycle page: stretch(red)/steps(orange)/fade(yellow)/window(green)
//                     each page's led2 brightness encodes that page's level
//   led2           -> encoder page color; brightness = crossfade active (fade>0)
//
// PICKUP (soft takeover) EVERYWHERE: landing on GLOBAL or a step does NOT snap
// its value to the pot -- a knob takes over only after it physically moves since
// arriving. So you can tour steps (and hop to global) without disturbing values
// you don't touch. All the mode/pickup logic is the host-tested PanelEditor.
//
// Read from the MAIN LOOP, not the audio callback: debouncing and smoothing do
// not belong in an interrupt, and the sequencer reads these values live anyway.
// EncoderPage enum + LED color helpers live in controls_core.h (host-tested).
static EncoderPage encPage = PAGE_STRETCH;

// Global drift: a fun all-steps shimmer, added on top of each step's own
// per-step drift (knob2). Effective drift per step = perStep + global, clamped.
// (How these two should ultimately combine is still open; additive is the
// simplest sensible first cut.)
static float globalDrift = 0.0f;

// Panel edit state (mode/pickup/shadow) lives in the platform-free PanelEditor
// so it is host-testable — see controls_core.h. GLOBAL mode: knobs = duration
// + global drift. Step mode: knobs = that step's position + per-step drift.
static PanelEditor panel;

// Stretch is a fixed, musically-spaced DETENT TABLE rather than a continuous
// range: PaulStretch factors are not perceptually linear, so what matters is the
// regime (scan / drift / freeze), not the exact number. Fine 1..10, then coarser
// as character stops changing: by 2 to 20, by 5 to 50, by 10 to 100, by 25 to
// 300, by 100 to 1000. The encoder moves an INDEX into this table.
static const float STRETCH_STOPS[] = {
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
  12, 14, 16, 18, 20,
  25, 30, 35, 40, 45, 50,
  60, 70, 80, 90, 100,
  125, 150, 175, 200, 225, 250, 275, 300,
  400, 500, 600, 700, 800, 900, 1000,
  // Above 1000x is transient-squelch territory: 1000x still lets a sharp hit
  // (a cymbal) punch through as a transient; ~10000x freezes it into sustained
  // wash. By 500s through the low thousands (where the freeze character still
  // changes), then by 1000s to 10000x.
  1500, 2000, 2500, 3000,
  4000, 5000, 6000, 7000, 8000, 9000, 10000,
};
static const int STRETCH_NSTOPS =
    (int)(sizeof(STRETCH_STOPS) / sizeof(STRETCH_STOPS[0]));   // 56
static int stretchIdx = 20;   // start at 50x (index into STRETCH_STOPS)

// Frame/window size stops (#136): powers of two up to SS_W (the compile-time
// buffer max). Smaller = grainier/more articulated, larger = glassy/frozen.
// Encoder page PAGE_FRAME indexes this; the value goes to seq.setFrame().
// LARGEST FIRST (idx 0 = SS_W, the default): a clockwise detent (inc +1) walks
// toward SMALLER windows, so turning right shrinks the frame.
static const int FRAME_STOPS[] = { 4096, 2048, 1024, 512, 256 };
static const int FRAME_NSTOPS =
    (int)(sizeof(FRAME_STOPS) / sizeof(FRAME_STOPS[0]));
static int frameIdx = 0;   // start at SS_W (4096), the default (largest window)

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
  if (pod.encoder.RisingEdge()) encPage = nextPage(encPage);

  // --- button1: advance panel mode (GLOBAL -> step1..N -> GLOBAL) where N is
  // the active step count (#149) -- the tour only visits active steps ---
  if (pod.button1.RisingEdge()) panel.advance(seq.activeSteps);
  // --- button2: jump back to GLOBAL ---
  if (pod.button2.RisingEdge()) panel.toGlobal();

  // --- encoder turn: stretch (index into detent table) or fade (additive) ---
  // Speed from the detent GAP (Increment is only +-1); stepping math is pure
  // and host-tested in controls_core.h.
  int32_t inc = pod.encoder.Increment();
  if (inc != 0) {
    uint32_t tnow = System::GetNow();
    bool fast = encoderFast(tnow - lastDetentMs);
    lastDetentMs = tnow;
    if (encPage == PAGE_STRETCH) {
      stretchIdx = stepIndex(stretchIdx, inc, fast ? 3 : 1, STRETCH_NSTOPS);
      seq.stretch = STRETCH_STOPS[stretchIdx];
    } else if (encPage == PAGE_FRAME) {
      frameIdx = stepIndex(frameIdx, inc, 1, FRAME_NSTOPS);   // 1 stop/detent
      seq.setFrame(FRAME_STOPS[frameIdx]);                    // recompute window
    } else if (encPage == PAGE_STEPS) {
      // Active step count 1..SS_STEPS (#149): one step per detent (small range,
      // no fast/coarse mode). Keep the panel nav on a valid slot if the count
      // shrank past the currently selected step.
      seq.setSteps(stepCount(seq.activeSteps, inc, 1, SS_STEPS));
      panel.clampToActive(seq.activeSteps);
    } else {   // PAGE_FADE
      seq.fade = stepAdditive(seq.fade, inc, fast ? 0.04f : 0.005f, 0.0f, 0.5f);
    }
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
  dbgR1 = r1; dbgR2 = r2; dbgK1 = k1; dbgK2 = k2;   // for the KNOB profiler line
  if (panel.speed1() > dbgPk1) dbgPk1 = panel.speed1();   // peak since last print
  if (panel.speed2() > dbgPk2) dbgPk2 = panel.speed2();
#endif

  // --- led2: encoder page color (RoYG over stretch/steps/fade/window, same
  // ROYGBIVW palette as led1). EVERY page's brightness tracks that page's
  // encoded LEVEL, so a bright LED always means "this parameter is turned up":
  //   stretch -> red    = stretch detent index
  //   steps   -> orange = active step count
  //   fade    -> yellow = crossfade amount (0..0.5)
  //   frame   -> green  = frame-size (window) index
  float b2;
  switch (encPage) {
    case PAGE_STRETCH: b2 = stretchBrightness(stretchIdx, STRETCH_NSTOPS); break;
    case PAGE_FADE:    b2 = fadeBrightness(seq.fade);                      break;
    case PAGE_FRAME:   b2 = frameBrightness(frameIdx, FRAME_NSTOPS);       break;
    case PAGE_STEPS:   b2 = stepBrightness(seq.activeSteps);               break;
    default:           b2 = 0.15f;                                         break;
  }
  Rgb c2 = pageColor(encPage, b2);
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

// --- audio -----------------------------------------------------------------
// #129 bisection: define DEBUG_PURE_TONE to bypass the sequencer entirely and
// emit a continuous 220 Hz sine. If THAT still clicks every second, the click is
// in the firmware/codec path, not the sequencer/DSP.
static void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                          AudioHandle::InterleavingOutputBuffer out,
                          size_t                                size) {
#ifdef DEBUG_PURE_TONE
  static float phase = 0.0f;
  const float  inc = 2.0f * (float)M_PI * 220.0f / SAMPLE_RATE;
  for (size_t i = 0; i < size; i += 2) {
    float s = 0.3f * sinf(phase);
    phase += inc;
    if (phase > 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
    out[i] = s; out[i + 1] = s;
  }
#else
#ifdef PROFILE
  uint32_t t0 = System::GetUs();
#endif
  for (size_t i = 0; i < size; i += 2) {
    float s     = seq.next();
    out[i]      = s;   // left
    out[i + 1]  = s;   // right
  }
#ifdef PROFILE
  profIsrUs += System::GetUs() - t0;
#endif
#endif
}

int main(void) {
  pod.Init();
  pod.SetAudioBlockSize(4);
#ifdef PROFILE
  pod.seed.StartLog(false);   // USB CDC; non-blocking so boot never stalls
#endif

  gTab.init();                       // ShyFFT tables + window; takes a moment
  src.data = sourceBuf;
  src.len  = SOURCE_LEN;
  // TEMPORARY (#132 testing): real material from QSPI so the controls can be
  // judged on broadband audio -- a sine has no spectral variation across the
  // buffer, so moving a read head sounds identical everywhere. Reverts to the
  // SD-card path (#131) once the controls are sorted.
  if (!load_qspi_sample()) fill_stub_source();

  seq.init(&src, pod.AudioSampleRate(), voicePool);
  // Starting values; the knobs/encoder take over from here (see processControls).
  // The knobs snap to their physical positions on the first read, so duration and
  // drift are whatever the pots are set to within a few ms of boot.
  seq.stretch  = STRETCH_STOPS[stretchIdx];   // 50x, matches stretchIdx default
  seq.duration = 1.0f;
  seq.fade     = 0.0f;    // butt-joint by default; raise fade for crossfade

  pod.StartAdc();
  pod.StartAudio(AudioCallback);

  // Main loop: keep every voice's FIFO ahead of the audio callback, and read the
  // panel. Controls are polled here rather than in the audio ISR -- debouncing
  // and smoothing do not belong in an interrupt.
  // Controls poll on a WALL-CLOCK 1 ms tick, NOT every-Nth-service(): a single
  // service() call is a full FFT (~2.4 ms at low stretch), so gating controls on
  // a service count polled the encoder only ~6x/sec -- it missed most detents of
  // a real spin and never saw the multi-detent bursts the fast-log branch needs.
  // System::GetNow() is milliseconds; poll every 1 ms so a fast spin registers.
  uint32_t lastControlMs = System::GetNow();
#ifdef PROFILE
  uint32_t profBusyUs = 0, profUnits = 0, profLastUnder = 0;
  uint32_t profLastPrint = System::GetNow();
#endif
  while (1) {
#ifdef PROFILE
    uint32_t s0 = System::GetUs();
    if (seq.service()) {
      profBusyUs += System::GetUs() - s0;
      profUnits++;
    }
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
    if (now - profLastPrint >= 1000) {
      profLastPrint = now;
      uint32_t isr = profIsrUs;
      profIsrUs = 0;
      // SETTINGS line: globals + which step is selected. Integers *1000 (or
      // *100 for stretch) since nano-newlib printf can't do floats reliably.
      pod.seed.PrintLine(
          "SET stretch_c=%d dur_ms=%d gdrift_m=%d fade_m=%d frame=%d steps=%d page=%d slot=%d",
          (int)(seq.stretch * 100.0f + 0.5f),
          (int)(seq.duration * 1000.0f + 0.5f),
          (int)(globalDrift * 1000.0f + 0.5f),
          (int)(seq.fade * 1000.0f + 0.5f),
          seq.frameSize,
          seq.activeSteps,   // active step count (#149)
          (int)encPage,
          panel.slot());   // 0 = GLOBAL, 1..N = step
      // KNOB line: raw + smoothed knob reads (*1000) and whether pickup has
      // engaged on the current slot (k1L/k2L = 1 once the pot has moved past
      // threshold). If you turn a knob and k1L stays 0, pickup isn't detecting
      // the move; if k1L=1 but the value doesn't change, the write is the bug.
      // pk1/pk2 = PEAK per-poll knob speed since last print (*1000, i.e. per-mil
      // of full travel per poll). potFast threshold is 10 in these units (0.01).
      // Turn a knob and read pk to see what "fast" actually measures -> tune the
      // threshold. f1/f2 = the fast verdict at print time.
      pod.seed.PrintLine(
          "KNOB r1=%d r2=%d k1L=%d k2L=%d pk1=%d pk2=%d f1=%d f2=%d b1=%d b2=%d",
          (int)(dbgR1 * 1000.0f + 0.5f), (int)(dbgR2 * 1000.0f + 0.5f),
          (int)panel.k1Live(), (int)panel.k2Live(),
          (int)(dbgPk1 * 1000.0f + 0.5f), (int)(dbgPk2 * 1000.0f + 0.5f),
          (int)panel.fast1(), (int)panel.fast2(),
          (int)pod.button1.Pressed(), (int)pod.button2.Pressed());
      dbgPk1 = dbgPk2 = 0.0f;   // reset peak for the next window
      // POS line: all 8 step positions (*1000). Homing every knob should make
      // these equal; if they differ, that's why steps sound different.
      pod.seed.PrintLine(
          "POS %d %d %d %d %d %d %d %d",
          (int)(seq.position[0] * 1000.0f + 0.5f), (int)(seq.position[1] * 1000.0f + 0.5f),
          (int)(seq.position[2] * 1000.0f + 0.5f), (int)(seq.position[3] * 1000.0f + 0.5f),
          (int)(seq.position[4] * 1000.0f + 0.5f), (int)(seq.position[5] * 1000.0f + 0.5f),
          (int)(seq.position[6] * 1000.0f + 0.5f), (int)(seq.position[7] * 1000.0f + 0.5f));
      // DRF line: per-step drift shadow (what the knob set), then the EFFECTIVE
      // drift the DSP reads (perStep + global). If effective differs from shadow
      // uniformly, that's global drift; if the shadow itself varies, that's the
      // per-step knob.
      pod.seed.PrintLine(
          "DRF s %d %d %d %d %d %d %d %d | eff %d %d %d %d %d %d %d %d",
          (int)(panel.perStepDrift(0)*1000+0.5f), (int)(panel.perStepDrift(1)*1000+0.5f),
          (int)(panel.perStepDrift(2)*1000+0.5f), (int)(panel.perStepDrift(3)*1000+0.5f),
          (int)(panel.perStepDrift(4)*1000+0.5f), (int)(panel.perStepDrift(5)*1000+0.5f),
          (int)(panel.perStepDrift(6)*1000+0.5f), (int)(panel.perStepDrift(7)*1000+0.5f),
          (int)(seq.drift[0]*1000+0.5f), (int)(seq.drift[1]*1000+0.5f),
          (int)(seq.drift[2]*1000+0.5f), (int)(seq.drift[3]*1000+0.5f),
          (int)(seq.drift[4]*1000+0.5f), (int)(seq.drift[5]*1000+0.5f),
          (int)(seq.drift[6]*1000+0.5f), (int)(seq.drift[7]*1000+0.5f));
      // HEALTH line: CPU and dropout accounting for this second.
      pod.seed.PrintLine(
          "HLTH act=%d units=%u svc_us=%u avg_us=%u isr_us=%u du=%u",
          seq.activeVoices(), (unsigned)profUnits, (unsigned)profBusyUs,
          (unsigned)(profUnits ? profBusyUs / profUnits : 0), (unsigned)isr,
          (unsigned)(gUnderruns - profLastUnder));
      profLastUnder = gUnderruns;
      profBusyUs = profUnits = 0;
    }
#endif
  }
}
