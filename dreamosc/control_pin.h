// control_pin.h -- an AnalogInput over one of libDaisy's AnalogControls
// (DEVICE EDGE). The Pod's two knobs are these: DaisyPod::Init() builds
// pod.knob1 / pod.knob2 (AnalogControl on ADC1, KNOB_1 = D21, KNOB_2 = D15),
// and this reads them through libDaisy's own object rather than reaching
// past the Pod into ADC channel numbers.
//
// It reads GetRawFloat() (count / 65535), NOT Value(): Value() is
// AnalogControl's one-pole filter, and every filter a control wants is the
// Knob's (controls_core.h) -- in seconds, not per poll, and host-tested -- so a
// pot behaves the same whatever input it is wired to. Process() is therefore
// never called. Note GetRawFloat() also skips AnalogControl's flip / invert /
// offset / scale (those apply in Process()); the Pod's knobs use none of them.
#pragma once

#include "hid/ctrl.h"
#include "analog_input.h"

class ControlPin : public AnalogInput {
 public:
  explicit ControlPin(daisy::AnalogControl& ctl) : ctl_(ctl) {}
  float read() const override { return ctl_.GetRawFloat(); }
  // The ADC's DMA buffer is live from StartAdc(), which runs before the audio
  // callback (where controls are polled) starts: startInstrument().

 private:
  daisy::AnalogControl& ctl_;
};
