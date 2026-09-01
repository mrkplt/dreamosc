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
static void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                          AudioHandle::InterleavingOutputBuffer out,
                          size_t                                size) {
  pod.ProcessAllControls();
  for (size_t i = 0; i < size; i += 2) {
    float s     = seq.next();
    out[i]      = s;   // left
    out[i + 1]  = s;   // right
  }
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
  seq.duration = 4.0f;
  seq.spread   = 0.5f;

  pod.StartAdc();
  pod.StartAudio(AudioCallback);

  // Keep every voice's FIFO ahead of the audio callback.
  while (1) {
    seq.service();
  }
}
