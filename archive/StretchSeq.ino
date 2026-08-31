// StretchSeq.ino - a step sequencer that addresses positions inside a virtual
// PaulStretch. Arduino IDE sketch for the Daisy Seed (DaisyDuino library).
//
// Board:   Daisy Seed / Seed2 DFM / Seed3  (Tools > Board > Daisy)
// Library: DaisyDuino  (Library Manager)
//
// The DSP lives in stretch_core.h, which has no Arduino dependencies and has
// been verified on a host compiler. This file is only wiring: audio in and
// out, pots, and keeping the render loop ahead of the callback.

#include "DaisyDuino.h"
#include "stretch_core.h"   // pulls in shy_fft.h

// ---------------------------------------------------------------------------
// Storage
// ---------------------------------------------------------------------------

#define SOURCE_SECONDS 10
#define SAMPLE_RATE 48000
#define SOURCE_LEN (SOURCE_SECONDS * SAMPLE_RATE)

// The source lives in SDRAM; everything the DSP touches per sample stays in
// internal SRAM, which is what keeps the render cheap.
static float DSY_SDRAM_BSS sourceBuf[SOURCE_LEN];

StretchTables gTab;                  // FFT tables, window, sine LUT
float gWork[SS_W];              // windowed frame; ShyFFT::Direct destroys it
float gSpec[SS_W];              // split spectrum: real [0,W/2), imag [W/2,W)

static Source    src;
static Sequencer seq;
static DaisyHardware hw;
static size_t numChannels;

// ---------------------------------------------------------------------------
// The eight steps. Positions are percentages through the stretch, and their
// arrangement is the composition: clustered for eight views of one moment,
// spread for a tour of the whole source. Edit freely.
// ---------------------------------------------------------------------------

static const float kPositions[SS_STEPS] = {
  0.10f, 0.13f, 0.16f, 0.19f, 0.22f, 0.25f, 0.28f, 0.31f
};

// ---------------------------------------------------------------------------
// Panel. Four pots for the globals; drift is shared here because the Seed has
// the ADC channels for eight more but a first build rarely has the pots.
// ---------------------------------------------------------------------------

#define PIN_STRETCH  A0
#define PIN_DURATION A1
#define PIN_SPREAD   A2
#define PIN_DRIFT    A3
#define PIN_RECORD   D0    // momentary to ground, re-arms recording

static float smoothed[4] = {0.5f, 0.5f, 0.5f, 0.0f};

static float readPot(int pin, int slot) {
  float v = analogRead(pin) / 1023.0f;
  smoothed[slot] += 0.02f * (v - smoothed[slot]);   // one-pole, ~pot-speed
  return smoothed[slot];
}

// ---------------------------------------------------------------------------
// Recording
// ---------------------------------------------------------------------------

static volatile bool     recording = true;
static volatile uint32_t recPos = 0;

static void armRecording() {
  recPos = 0;
  recording = true;
}

// ---------------------------------------------------------------------------
// Audio
// ---------------------------------------------------------------------------

static void AudioCallback(float **in, float **out, size_t size) {
  if (recording) {
    for (size_t i = 0; i < size; i++) {
      if (recPos < SOURCE_LEN) sourceBuf[recPos++] = in[0][i];
      out[0][i] = in[0][i];                  // monitor while recording
      if (numChannels > 1) out[1][i] = in[0][i];
    }
    if (recPos >= SOURCE_LEN) recording = false;
    return;
  }
  for (size_t i = 0; i < size; i++) {
    float s = seq.next();
    out[0][i] = s;
    if (numChannels > 1) out[1][i] = s;
  }
}

// ---------------------------------------------------------------------------

void setup() {
  hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  numChannels = hw.num_channels;

  pinMode(PIN_RECORD, INPUT_PULLUP);
  analogReadResolution(10);

  for (uint32_t i = 0; i < SOURCE_LEN; i++) sourceBuf[i] = 0.0f;

  gTab.init();                     // ShyFFT tables and window; takes a moment
  src.data = sourceBuf;
  src.len  = SOURCE_LEN;

  seq.init(&src, (float)SAMPLE_RATE, 0x12345678u);
  for (int i = 0; i < SS_STEPS; i++) {
    seq.position[i] = kPositions[i];
    seq.drift[i]    = 0.0f;
  }
  seq.stretch  = 50.0f;
  seq.duration = 4.0f;
  seq.spread   = 1.0f;

  armRecording();
  DAISY.begin(AudioCallback);
}

void loop() {
  if (digitalRead(PIN_RECORD) == LOW) armRecording();

  if (!recording) {
    // Stretch factor, exponential so the knob is useful across its travel.
    // It does not move the steps: it sets how far each read head travels
    // during its step, so low is a scan and high is a held chord.
    seq.stretch = 5.0f * powf(100.0f, readPot(PIN_STRETCH, 0));   // 5..500

    // Step duration, global. The lattice is even in time and uneven only in
    // the source.
    seq.duration = 0.5f + 7.5f * readPot(PIN_DURATION, 1);        // 0.5..8 s

    // Spread: 0 butt-joined and articulated, 1 two steps always sounding,
    // 2 three, above that a chord of positions.
    seq.spread = 3.0f * readPot(PIN_SPREAD, 2);                   // 0..3

    // Drift: how far a step wanders from its position, redrawn each pass with
    // no memory. Zero is literal repetition.
    float d = 0.02f * readPot(PIN_DRIFT, 3);                      // 0..2 %
    for (int i = 0; i < SS_STEPS; i++) seq.drift[i] = d;
  }

  // Keep every voice's FIFO ahead of the callback. One frame per pass bounds
  // how long a single loop iteration can block.
  seq.service();
}
