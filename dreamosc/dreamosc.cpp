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
//   knob1          -> current step's POSITION (0..1)
//   knob2          -> current step's DRIFT    (0..0.25)
//   button1        -> advance the SELECTED STEP (0..7, wraps)
//   encoder turn   -> the current page's global parameter
//   encoder click  -> cycle the encoder page: stretch / fade / duration / drift
//   button2        -> (free)
//   led1           -> selected step, ROYGBIVW (steps 1..8)
//   led2           -> encoder page color; brightness = crossfade active (fade>0)
//
// PER-STEP EDITING with PICKUP (soft takeover): the two knobs edit the CURRENTLY
// SELECTED step's position/drift. Selecting a new step (button1) does NOT snap
// the step to the pot position -- the step keeps its stored value UNTIL a knob
// is physically moved, at which point that knob takes over. This lets you tour
// the 8 steps and only change the ones you touch.
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

// Per-step edit state (pickup latches, step select, drift shadow) lives in the
// platform-free StepEditor so it is host-testable — see controls_core.h.
static StepEditor stepEd;

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

// Knob smoothing state (the smoothing math is smoothKnob() in controls_core.h).
static float knobSmooth[2] = {0.0f, 0.0f};
static bool  knobPrimed    = false;

// Milliseconds since the last encoder detent on the current page, for the
// shared fast/slow speed detection (encoderFast). Reset when the page changes
// so a page switch doesn't read as a fast spin.
static uint32_t lastDetentMs = 0;

static void processControls() {
  pod.ProcessAllControls();

  // --- encoder click: cycle page (stretch -> fade -> duration -> drift) ---
  if (pod.encoder.RisingEdge()) encPage = nextPage(encPage);

  // --- button1: advance the selected step (arm knob pickup for the new step) ---
  if (pod.button1.RisingEdge()) stepEd.advanceStep();

  // --- encoder turn: drives the current page's parameter ---
  // Speed comes from the GAP between detents (Increment is only +-1); the pure
  // stepping math is in controls_core.h so it is host-tested. Fast = coarse
  // step, slow = fine, per the values below (chosen so a page's full range is
  // ~1-2 turns fast / fine when clicked deliberately).
  int32_t inc = pod.encoder.Increment();
  if (inc != 0) {
    uint32_t tnow = System::GetNow();
    bool fast = encoderFast(tnow - lastDetentMs);
    lastDetentMs = tnow;
    switch (encPage) {
      case PAGE_DURATION:
        seq.duration = stepRatio(seq.duration, inc, fast ? 1.08f : 1.015f,
                                 0.25f, 60.0f);
        break;
      case PAGE_DRIFT:   // global drift, additive
        globalDrift = stepAdditive(globalDrift, inc, fast ? 0.02f : 0.0025f,
                                   0.0f, 0.25f);
        break;
      case PAGE_STRETCH: // index into the detent table; 3 stops/detent fast
        stretchIdx = stepIndex(stretchIdx, inc, fast ? 3 : 1, STRETCH_NSTOPS);
        seq.stretch = STRETCH_STOPS[stretchIdx];
        break;
      case PAGE_FADE:    // crossfade length, additive
        seq.fade = stepAdditive(seq.fade, inc, fast ? 0.04f : 0.005f, 0.0f, 0.5f);
        break;
      default: break;
    }
  }

  // --- knobs: edit the SELECTED step's position (k1) and per-step drift (k2),
  // with PICKUP. A knob takes over its parameter for the current step only once
  // it has physically MOVED since the step was selected; until then the step
  // holds its stored value. The move test is a small threshold on the raw read
  // so pot noise doesn't trip it.
  float k1 = smoothKnob(knobSmooth[0], pod.knob1.Value(), knobPrimed);
  float k2 = smoothKnob(knobSmooth[1], pod.knob2.Value(), knobPrimed);
  knobPrimed = true;
  // StepEditor applies pickup, writes the selected step's position/drift, and
  // folds per-step + global drift into seq.drift[] for every step. All the
  // decision logic is here and host-tested (controls_core.h).
  stepEd.update(seq, k1, k2, globalDrift);

  // --- led2: encoder page color; brightness = crossfade active (fade > 0) ---
  // (pageColor/stepColor are host-tested pure lookups in controls_core.h.)
  Rgb c2 = pageColor(encPage, (seq.fade > 0.0f) ? 0.6f : 0.15f);
  pod.led2.Set(c2.r, c2.g, c2.b);

  // --- led1: selected step, ROYGBIVW (steps 1..8) ---
  Rgb c1 = stepColor(stepEd.selected());
  pod.led1.Set(c1.r, c1.g, c1.b);

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
      // SETTINGS line: full state, integers (nano-newlib printf can't do floats
      // reliably). Globals: stretch*100, dur_ms, gdrift_m=globalDrift*1000,
      // fade_m=fade*1000, page. Selected step: sel (0..7), its pos*1000 and
      // per-step drift*1000.
      pod.seed.PrintLine(
          "SET stretch_c=%d dur_ms=%d gdrift_m=%d fade_m=%d page=%d sel=%d pos_m=%d sdrift_m=%d",
          (int)(seq.stretch * 100.0f + 0.5f),
          (int)(seq.duration * 1000.0f + 0.5f),
          (int)(globalDrift * 1000.0f + 0.5f),
          (int)(seq.fade * 1000.0f + 0.5f),
          (int)encPage,
          stepEd.selected(),
          (int)(seq.position[stepEd.selected()] * 1000.0f + 0.5f),
          (int)(stepEd.perStepDrift(stepEd.selected()) * 1000.0f + 0.5f));
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
