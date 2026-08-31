// dreamosc.cpp - Daisy Pod firmware for the Stretch Sequencer.
//
// STATUS: PLACEHOLDER. This is NOT the finished instrument yet. It currently
// emits a test oscillator so the toolchain/build/flash path is proven end to
// end. The real firmware still has to be written — see the checklist below and
// CLAUDE.md "State of play".
//
// TODO (the actual instrument):
//   [ ] Define the three globals stretch_core.h externs:
//         StretchTables gTab;  float gWork[SS_W];  float gSpec[SS_W];
//   [ ] Source + voice buffers in SDRAM (DSY_SDRAM_BSS). ~196 KB of voices.
//   [ ] load_source() from sd_source.h at boot; stub with a test tone until a
//       microSD card with a WAV is present.
//   [ ] Sequencer::next() in the audio callback; Sequencer::service() in main().
//   [ ] Pod controls: encoder -> stretch, knob1 -> duration, knob2 -> spread,
//       drift on a second page. See CLAUDE.md.

#include "daisy_pod.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisysp;

DaisyPod   pod;
Oscillator osc;

void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{
    pod.ProcessAllControls();
    osc.SetFreq(110.0f + pod.knob1.Value() * 330.0f);
    osc.SetAmp(pod.knob2.Value());
    for(size_t i = 0; i < size; i += 2)
    {
        float sig  = osc.Process();
        out[i]     = sig;
        out[i + 1] = sig;
    }
}

int main(void)
{
    pod.Init();
    pod.SetAudioBlockSize(4);
    osc.Init(pod.AudioSampleRate());
    osc.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
    pod.StartAdc();
    pod.StartAudio(AudioCallback);
    while(1) {}
}
