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

#ifdef PROFILE
// #132 diagnostic build (`make PROFILE=1`): measure where the CPU actually goes
// so the low-spread noise threshold can be attributed with numbers, not
// estimates. Prints one line per second over USB serial — read it with
//   screen /dev/tty.usbmodem<tab> 115200
// Fields: act (sounding heads), units (service() calls that did work: emitted
// slices + transitions), rnd (FFT frames rendered that second — the load
// metric), svc_us (us of main-loop DSP work that second), avg_us (per unit),
// isr_us (us inside the audio callback that second), du (underruns that
// second), spread_m (spread * 1000), page (0 stretch / 1 spread).
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
// initializers and Head sub-objects (atomics); objects in .sdram_bss get
// NEITHER their constructor run NOR their storage zeroed (the section is NOLOAD
// and SDRAM is not even powered until Init()), so a Sequencer placed there
// boots with garbage state and produces no sound. The heads' big buffers live
// in headPool below; the Sequencer object itself is small.
static Sequencer seq;

// Head working buffers (old_ + ring_ per head, 8 x 48 KB = 384 KB at SS_W 4096)
// live in SDRAM: far too big for the 512 KB internal SRAM once anything else is
// present. This is a plain array, NOT an object -- .sdram_bss is NOLOAD and
// SDRAM is unpowered at static-init time, so constructors never run and storage
// is not zeroed there. Sequencer::init() carves this up and hands each Head a
// slice (memsetting it); the Head objects themselves stay in SRAM where C++
// works normally.
static float DSY_SDRAM_BSS headPool[SS_POOL_FLOATS];

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

  seq.init(&src, pod.AudioSampleRate(), headPool);
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
#ifdef PROFILE
  uint32_t profBusyUs = 0, profUnits = 0, profLastUnder = 0, profLastRnd = 0;
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
    // Poll the panel ~1 kHz-ish rather than every service() pass: encoder
    // debouncing expects a steady-ish rate, and there is no reason to burn FFT
    // time on ADC reads.
    if (++control_div >= 64) {
      control_div = 0;
      processControls();
    }
#ifdef PROFILE
    uint32_t now = System::GetNow();
    if (now - profLastPrint >= 1000) {
      profLastPrint = now;
      uint32_t isr = profIsrUs;
      profIsrUs = 0;
      uint32_t rnd = seq.framesRendered();
      pod.seed.PrintLine(
          "act=%d units=%u rnd=%u svc_us=%u avg_us=%u isr_us=%u du=%u spread_m=%d page=%d",
          seq.soundingHeads(), (unsigned)profUnits,
          (unsigned)(rnd - profLastRnd), (unsigned)profBusyUs,
          (unsigned)(profUnits ? profBusyUs / profUnits : 0), (unsigned)isr,
          (unsigned)(gUnderruns - profLastUnder), (int)(seq.spread * 1000.0f),
          (int)encPage);
      profLastUnder = gUnderruns;
      profLastRnd = rnd;
      profBusyUs = profUnits = 0;
    }
#endif
  }
}
