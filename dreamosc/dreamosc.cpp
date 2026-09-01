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

static DaisyPod pod;
static Source   src;

// --- stub source: a test tone so the instrument makes sound before SD (#131) -
// A couple of detuned partials give the stretch something with spectral content
// to smear. Replaced by stretchsd::load_source() when a card is present.
static void fill_stub_source() {
  for (uint32_t i = 0; i < SOURCE_LEN; i++) {
    float t = (float)i / SAMPLE_RATE;
    sourceBuf[i] = 0.5f * sinf(2.0f * (float)M_PI * 220.0f * t)
                 + 0.3f * sinf(2.0f * (float)M_PI * 331.0f * t)
                 + 0.2f * sinf(2.0f * (float)M_PI * 554.0f * t);
  }
}

// --- audio -----------------------------------------------------------------
// #129 bisection: define DEBUG_PURE_TONE to bypass the sequencer entirely and
// emit a continuous 220 Hz sine. If THAT still clicks every second, the click is
// in the firmware/codec path, not the sequencer/DSP.
static void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                          AudioHandle::InterleavingOutputBuffer out,
                          size_t                                size) {
  pod.ProcessAllControls();
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
  fill_stub_source();
  src.data = sourceBuf;
  src.len  = SOURCE_LEN;

  seq.init(&src, pod.AudioSampleRate());
  // Defaults from the spec; controls (#132) will drive these live later.
  seq.stretch  = 50.0f;
  seq.duration = 1.0f;   // #129 debug: 1s steps -> 8s pattern, faster to hear
  seq.spread   = 1.0f;   // #129 debug: no head overlap, one voice at a time

  pod.StartAdc();
  pod.StartAudio(AudioCallback);

  // Keep every voice's FIFO ahead of the audio callback. Latch led1 red if a
  // ring ever starves (gUnderruns moves) — diagnostic for the boundary click.
  uint32_t last_underruns = gUnderruns;
  while (1) {
    seq.service();
    if (gUnderruns != last_underruns) {
      last_underruns = gUnderruns;
      pod.led1.Set(1.0f, 0.0f, 0.0f);   // red = underrun happened
      pod.UpdateLeds();
    }
  }
}
