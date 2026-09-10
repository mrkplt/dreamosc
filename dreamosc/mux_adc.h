// mux_adc.h - the CD4051 analog mux on A7, scanned by ADC2 (device edge).
//
// WHY NOT libDaisy's AdcHandle mux mode: DaisyPod::Init() has already run
// AdcHandle::Init() for the two panel knobs by the time we could add the mux,
// and the ADC's DMA mode (circular for plain channels, one-shot for a mux)
// is chosen inside HAL_ADC_MspInit, which the HAL runs only on the FIRST
// HAL_ADC_Init (state RESET). Re-initialising with a mux channel therefore
// left the DMA circular under a one-shot ADC: the conversion-complete
// callback restarted a DMA that never stopped, the select lines stepped
// against stale data (every mux channel read the same floating value), and
// the restart storm took the board down. AdcHandle has no DeInit, so the
// Pod's ADC1 cannot be reset from here without editing libDaisy.
//
// So the mux gets its own converter. ADC2 is unused by libDaisy and shares
// the ADC12 clock (already enabled) and the A7 input (PA5 = ADC12_INP19)
// with ADC1, which no longer samples it. ADC2 runs CONTINUOUS on that one
// channel, no DMA, no interrupt, overrun = overwrite: the data register
// always holds the latest completed conversion. poll() -- once per
// millisecond from processControls() -- reads it for the channel the select
// lines have pointed at for the last millisecond, then steps the selects to
// the next channel. Eight channels scan in 8 ms; the 4051 settles in well
// under a microsecond and a conversion (16-bit, 32x oversampled, 64.5-cycle
// sampling for the pot + mux source impedance) takes ~0.2 ms, so the value
// read is always a conversion that both started and finished after the
// switch.
//
// Init AFTER pod.Init() (ADC12 clock, PLL3) and BEFORE pod.StartAdc(): the
// ADC12 common prescaler is written only while neither ADC is enabled, so
// ADC2 must come up with the same prescaler as ADC1 (ASYNC_DIV2) and before
// ADC1 starts.

#ifndef MUX_ADC_H
#define MUX_ADC_H

#include <stdint.h>

#include "daisy_seed.h"

class MuxAdc {
 public:
  static constexpr int CHANNELS = 8;

  // adcPin: the mux common (an ADC12 input); sel0..2: the 4051's A, B, C.
  bool init(daisy::Pin adcPin, uint32_t adcChannel, daisy::Pin sel0, daisy::Pin sel1, daisy::Pin sel2) {
    using namespace daisy;
    GPIO::Config a;
    a.pin = adcPin; a.mode = GPIO::Mode::ANALOG; a.pull = GPIO::Pull::NOPULL;
    pin_.Init(a);
    const Pin sels[3] = {sel0, sel1, sel2};
    for (int i = 0; i < 3; i++) {
      GPIO::Config s;
      s.pin = sels[i]; s.mode = GPIO::Mode::OUTPUT; s.pull = GPIO::Pull::NOPULL; s.speed = GPIO::Speed::LOW;
      sel_[i].Init(s);
    }
    select(0);

    __HAL_RCC_ADC12_CLK_ENABLE();
    hadc_ = ADC_HandleTypeDef();
    hadc_.Instance                      = ADC2;
    hadc_.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV2;    // same as libDaisy's ADC1 (shared common)
    hadc_.Init.Resolution               = ADC_RESOLUTION_16B;
    hadc_.Init.ScanConvMode             = ADC_SCAN_DISABLE;
    hadc_.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    hadc_.Init.LowPowerAutoWait         = DISABLE;
    hadc_.Init.ContinuousConvMode       = ENABLE;
    hadc_.Init.NbrOfConversion          = 1;
    hadc_.Init.DiscontinuousConvMode    = DISABLE;
    hadc_.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    hadc_.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc_.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc_.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
    hadc_.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
    hadc_.Init.OversamplingMode         = ENABLE;
    hadc_.Init.Oversampling.Ratio                 = 32;
    hadc_.Init.Oversampling.RightBitShift         = ADC_RIGHTBITSHIFT_5;
    hadc_.Init.Oversampling.TriggeredMode         = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
    hadc_.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
    if (HAL_ADC_Init(&hadc_) != HAL_OK) return false;

    ADC_ChannelConfTypeDef ch = {};
    ch.Channel      = adcChannel;
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
    ch.SingleDiff   = ADC_SINGLE_ENDED;
    ch.OffsetNumber = ADC_OFFSET_NONE;
    ch.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc_, &ch) != HAL_OK) return false;
    if (HAL_ADCEx_Calibration_Start(&hadc_, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK) return false;
    if (HAL_ADC_Start(&hadc_) != HAL_OK) return false;
    ok_ = true;
    return true;
  }

  // Once per millisecond: latch the current channel's latest conversion,
  // then point the mux at the next channel for the next millisecond.
  void poll() {
    if (!ok_) return;
    raw_[cur_] = (uint16_t)HAL_ADC_GetValue(&hadc_);
    cur_ = (cur_ + 1) & (CHANNELS - 1);
    select(cur_);
  }

  float value(int ch) const { return raw_[ch & (CHANNELS - 1)] / 65535.0f; }
  bool  ok() const { return ok_; }

 private:
  void select(int ch) {
    sel_[0].Write((ch & 1) != 0);
    sel_[1].Write((ch & 2) != 0);
    sel_[2].Write((ch & 4) != 0);
  }

  ADC_HandleTypeDef hadc_ = {};
  daisy::GPIO pin_, sel_[3];
  uint16_t raw_[CHANNELS] = {0};
  int  cur_ = 0;
  bool ok_ = false;
};

#endif  // MUX_ADC_H
