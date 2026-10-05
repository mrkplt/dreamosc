// analog_input.h -- the hardware-reader interface every analog control reads
// through. Platform-free: the implementations live at the device edge
// (ControlPin in control_pin.h: one of libDaisy's AnalogControls, e.g. a Pod
// knob; MuxPin in mux_adc.h: one channel of the 4051, a "virtual pin"), and
// the host tests supply their own. A Knob (controls_core.h) is injected
// with one and never knows which kind it has.
#pragma once

class AnalogInput {
 public:
  virtual ~AnalogInput() = default;

  // The latest reading, normalised 0..1, with any per-pin correction
  // (wiring order, inversion, calibration) already applied. Called from the
  // audio callback: it must be cheap and must never block -- it returns a
  // value the hardware has already sampled.
  virtual float read() const = 0;

  // True once read() returns real readings. Until then a consumer must not
  // treat read() as a position (a mux reads 0 before its first full scan),
  // or a control would see its first real read as a move.
  virtual bool ready() const { return true; }
};
