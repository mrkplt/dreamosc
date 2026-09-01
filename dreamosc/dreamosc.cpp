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

using namespace daisy;

// --- the globals stretch_core.h externs ------------------------------------
StretchTables gTab;
float         gWork[SS_W];   // windowed frame; ShyFFT::Direct destroys its input
float         gSpec[SS_W];   // split spectrum: real [0,W/2), imag [W/2,W)

// #129 diagnostic: incremented in Voice::next() when a head's ring is empty
// (starved). led1 latches red if this ever moves — tells us on-device whether
// the boundary artifact is a buffer underrun vs. a DSP/splice issue.
volatile uint32_t gUnderruns = 0;

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
// The Daisy bootloader reserves the first 256 KB of QSPI for firmware images
// (it loads apps from 0x90040000). Park sample data ABOVE that so the two can
// never collide, even if we later switch to APP_TYPE=BOOT_QSPI.
#define QSPI_BASE        0x90100000u   // 1 MB in: clear of the bootloader region
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
//   knob1          -> duration   0.25 .. 8 s
//   knob2          -> drift      0 .. 25 % (all steps share one value for now)
//   encoder turn   -> stretch (page 0) or spread (page 1), exponential/linear
//   encoder click  -> toggle which parameter the encoder drives
//   led1           -> underrun indicator (latches red if a voice ring starves)
//   led2           -> page indicator (dim blue = stretch, dim green = spread)
//
// Read from the MAIN LOOP, not the audio callback: debouncing and smoothing do
// not belong in an interrupt, and the sequencer reads these values live anyway.
enum EncoderPage { PAGE_STRETCH = 0, PAGE_SPREAD = 1 };
static EncoderPage encPage = PAGE_STRETCH;

// Knob smoothing: a one-pole on the raw ADC read, so a noisy pot does not
// dither the parameter. ~pot-speed, same idea as the original .ino.
static float knobSmooth[2] = {0.0f, 0.0f};
static bool  knobPrimed    = false;

static float readKnob(int idx, float raw) {
  if (!knobPrimed) return raw;            // jump to the real value on first read
  knobSmooth[idx] += 0.02f * (raw - knobSmooth[idx]);
  return knobSmooth[idx];
}

static void processControls() {
  pod.ProcessAllControls();

  // --- encoder click: flip page ---
  if (pod.encoder.RisingEdge())
    encPage = (encPage == PAGE_STRETCH) ? PAGE_SPREAD : PAGE_STRETCH;

  // --- encoder turn: drives the current page's parameter ---
  int32_t inc = pod.encoder.Increment();
  if (inc != 0) {
    if (encPage == PAGE_STRETCH) {
      // Exponential: a detent is a fixed RATIO, so the knob is equally useful
      // at 5x and at 500x. ~3% per detent.
      seq.stretch *= powf(1.03f, (float)inc);
      if (seq.stretch < 1.0f)   seq.stretch = 1.0f;
      if (seq.stretch > 500.0f) seq.stretch = 500.0f;
    } else {
      // Linear over the whole 0..1 range; 1% per detent.
      seq.spread += 0.01f * (float)inc;
      if (seq.spread < 0.0f) seq.spread = 0.0f;
      if (seq.spread > 1.0f) seq.spread = 1.0f;
    }
  }

  // --- knobs ---
  float k1 = readKnob(0, pod.knob1.Value());
  float k2 = readKnob(1, pod.knob2.Value());
  knobPrimed = true;
  seq.duration = 0.25f + 7.75f * k1;          // 0.25 .. 8 s
  float d = 0.25f * k2;                        // 0 .. 25 % of the stretch
  for (int i = 0; i < SS_STEPS; i++) seq.drift[i] = d;

  // --- led2: which page the encoder is on ---
  if (encPage == PAGE_STRETCH) pod.led2.Set(0.0f, 0.0f, 0.15f);   // blue
  else                         pod.led2.Set(0.0f, 0.15f, 0.0f);   // green

  // --- led1: underrun indicator. Latches red if a voice ring ever starves, so
  // a dropout is visible rather than something to guess at. Kept from #129.
  static uint32_t last_underruns = 0;
  if (gUnderruns != last_underruns) {
    last_underruns = gUnderruns;
    pod.led1.Set(0.5f, 0.0f, 0.0f);   // red = underrun happened
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
  for (size_t i = 0; i < size; i += 2) {
    float s     = seq.next();
    out[i]      = s;   // left
    out[i + 1]  = s;   // right
  }
#endif
}

int main(void) {
  pod.Init();
  pod.SetAudioBlockSize(4);

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
  seq.stretch  = 50.0f;
  seq.duration = 1.0f;
  seq.spread   = 1.0f;

  pod.StartAdc();
  pod.StartAudio(AudioCallback);

  // Main loop: keep every voice's FIFO ahead of the audio callback, and read the
  // panel. Controls are polled here rather than in the audio ISR -- debouncing
  // and smoothing do not belong in an interrupt.
  uint32_t control_div = 0;
  while (1) {
    seq.service();
    // Poll the panel ~1 kHz-ish rather than every service() pass: encoder
    // debouncing expects a steady-ish rate, and there is no reason to burn FFT
    // time on ADC reads.
    if (++control_div >= 64) {
      control_div = 0;
      processControls();
    }
  }
}
