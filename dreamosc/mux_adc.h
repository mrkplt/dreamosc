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
// with ADC1, which no longer samples it.
//
// THE CONVERTER PACES THE SCAN, not the control poll. ADC2 does one single
// conversion per channel; its end-of-conversion interrupt (isr(), wired to
// ADC_IRQHandler in dreamosc.cpp) latches the result, points the 4051 at the
// next channel and starts the next conversion. So a full 8-channel scan takes
// 8 conversion times, and every pot has a fresh read on every 1 ms control
// poll -- the same cadence as the Pod's own knobs, which is what the tuned
// knob feel (speed-adaptive grid, ownership threshold) is measured in.
//
// Timing. ADC kernel = pll3_r (24.58 MHz stock; the codec planner keeps it
// in 24.13..25.06 MHz across 16..96 kHz, clock_core.h) / 2 (ASYNC_DIV2) / 2
// (the fixed internal divider on silicon rev V, which the Seed's H750 is --
// HAL ADC_ConfigureBoostMode: "divider by 2 for Rev.V") = 6.03..6.26 MHz.
// One conversion = 16 x (32.5 sampling + 8.5 conversion) = 656 cycles =
// 105..109 us -> a full scan every 0.84..0.87 ms, inside the 1 ms poll at
// every codec rate. 16x oversampling (not 32x) and 32.5-cycle sampling (not
// 64.5) are what buy that; 16x costs ~sqrt(2) in noise vs the one-pot bench
// test's 32x (to confirm on the MUX line).
//
// Settling. The next conversion starts right after the select lines change.
// The 4051 settles in about a microsecond at 3.3 V; the ADC holds the input
// at the END of each sampling window, and the first window is 32.5 cycles
// (~5.3 us), so the switch transient is over before anything is held. The
// pot + mux source impedance needs far less than that.
//
// Readiness. raw_[] reads 0 until a channel's first conversion, so nothing
// may treat the reads as pot positions before scans() >= 1 (every channel
// converted once) -- see StepKnobs in controls_core.h.
//
// Interrupt. ADC1 and ADC2 share ADC_IRQn. libDaisy never enables that IRQ
// (ADC1 runs on DMA), but HAL_ADC_Start_DMA does enable ADC1's OVERRUN
// interrupt, so once ADC_IRQn is on, an ADC1 overrun would land here and
// re-fire forever unless cleared: isr() clears and counts it (ovr1()).
// (ADC2 cannot overrun: each conversion is started only after the previous
// one's result has been read.) The
// priority is BELOW the audio DMA's (0): the audio callback preempts the
// scan, never the reverse; a delayed EOC only delays the next conversion.
//
// Init AFTER pod.Init() (ADC12 clock, PLL3) and BEFORE pod.StartAdc(): the
// ADC12 common prescaler is written only while neither ADC is enabled, so
// ADC2 must come up with the same prescaler as ADC1 (ASYNC_DIV2) and before
// ADC1 starts. stop()/start() bracket a PLL3 re-plan (codec_rate.h: the ADC
// kernel clock is PLL3's R output), alongside pod.StopAdc()/StartAdc().

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
    cur_ = 0;
    select(cur_);

    __HAL_RCC_ADC12_CLK_ENABLE();
    hadc_ = ADC_HandleTypeDef();
    hadc_.Instance                      = ADC2;
    hadc_.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV2;    // same as libDaisy's ADC1 (shared common)
    hadc_.Init.Resolution               = ADC_RESOLUTION_16B;
    hadc_.Init.ScanConvMode             = ADC_SCAN_DISABLE;
    hadc_.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    hadc_.Init.LowPowerAutoWait         = DISABLE;
    hadc_.Init.ContinuousConvMode       = DISABLE;                 // one conversion per start: isr() restarts
    hadc_.Init.NbrOfConversion          = 1;
    hadc_.Init.DiscontinuousConvMode    = DISABLE;
    hadc_.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    hadc_.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc_.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc_.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
    hadc_.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
    hadc_.Init.OversamplingMode         = ENABLE;
    hadc_.Init.Oversampling.Ratio                 = 16;         // scan < 1 ms (see the header comment)
    hadc_.Init.Oversampling.RightBitShift         = ADC_RIGHTBITSHIFT_4;
    hadc_.Init.Oversampling.TriggeredMode         = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
    hadc_.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
    if (HAL_ADC_Init(&hadc_) != HAL_OK) return false;

    ADC_ChannelConfTypeDef ch = {};
    ch.Channel      = adcChannel;
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;               // ~5.3 us: mux settles (see the header comment)
    ch.SingleDiff   = ADC_SINGLE_ENDED;
    ch.OffsetNumber = ADC_OFFSET_NONE;
    ch.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc_, &ch) != HAL_OK) return false;
    if (HAL_ADCEx_Calibration_Start(&hadc_, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK) return false;
    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);   // below the audio DMA (0)
    ok_ = true;
    return true;   // configured; start() begins the scan (with the instrument)
  }

  // Start / stop the scan (a PLL3 re-plan must happen between them). The IRQ
  // is masked across the HAL call in both directions, so isr() can never
  // re-arm ADSTART half-way through a stop.
  bool start() {
    if (!ok_) return false;
    const bool started = HAL_ADC_Start_IT(&hadc_) == HAL_OK;   // EOC (+OVR) interrupt, one conversion
    if (started) HAL_NVIC_EnableIRQ(ADC_IRQn);
    return started;
  }
  void stop() {
    if (!ok_) return;
    HAL_NVIC_DisableIRQ(ADC_IRQn);
    HAL_ADC_Stop_IT(&hadc_);
  }

  // ADC_IRQHandler body. ADC2 end of conversion: latch, step the mux, start
  // the next conversion (reading DR clears EOC). Register-level, not
  // HAL_ADC_IRQHandler: that would dispatch ADC2 into libDaisy's
  // HAL_ADC_ConvCpltCallback, which is ADC1's DMA handler.
  void isr() {
    if (ADC2->ISR & ADC_ISR_EOC) {
      raw_[cur_] = (uint16_t)ADC2->DR;
      if (cur_ == CHANNELS - 1) scans_++;
      cur_ = (cur_ + 1) & (CHANNELS - 1);
      select(cur_);
      LL_ADC_REG_StartConversion(ADC2);   // masks CR's set-only command bits (RM0433)
    }
    if (ADC1->ISR & ADC_ISR_OVR) { ADC1->ISR = ADC_ISR_OVR; ovr1_++; }
  }

  // The latest completed read of channel `ch`, 0..1.
  float value(int ch) const { return raw_[ch & (CHANNELS - 1)] / 65535.0f; }
  bool  ok() const { return ok_; }
  // Completed 8-channel scans since boot (>= 1: every channel is a real
  // read), and ADC1 overruns (the Pod knobs; cleared here), ever.
  uint32_t scans() const { return scans_; }
  uint32_t ovr1()  const { return ovr1_; }

 private:
  void select(int ch) {
    sel_[0].Write((ch & 1) != 0);
    sel_[1].Write((ch & 2) != 0);
    sel_[2].Write((ch & 4) != 0);
  }

  ADC_HandleTypeDef hadc_ = {};
  daisy::GPIO pin_, sel_[3];
  // Written by isr(), read by the audio callback: single aligned stores and
  // loads, and isr() never runs inside the callback (lower priority).
  volatile uint16_t raw_[CHANNELS] = {0};
  volatile uint32_t scans_ = 0, ovr1_ = 0;
  int  cur_ = 0;
  bool ok_ = false;
};

#endif  // MUX_ADC_H
