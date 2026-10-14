// mux_adc.h - the CD4051 analog muxes, scanned by ADC2 (device edge).
//
// Up to two 4051s (BANKS) share the three select lines (D7/D8/D9); each has
// its own common on its own ADC12 input (row 2 on A7, row 3 on A1). At every
// select position ADC2 converts each bank's common in turn, then steps the
// select lines.
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
// So the muxes get their own converter. ADC2 is unused by libDaisy and
// shares the ADC12 clock (already enabled) and the ADC12 inputs with ADC1,
// which samples neither A7 (PA5 = INP19) nor A1 (PA3 = INP15).
//
// THE CONVERTER PACES THE SCAN. ADC2 does one single conversion per start;
// its end-of-conversion interrupt (isr(), wired to ADC_IRQHandler in
// dreamosc.cpp) latches the result, points ADC2 at the next bank's common
// or the 4051s at the next channel, and starts the next conversion. Each
// conversion is started only after the previous one's result has been read,
// so a delayed interrupt (the audio callback preempts it) delays the scan
// and can never overrun ADC2 or attribute a result to the wrong pot -- safe
// by construction, which a hardware scan sequence (several ranks per start)
// would not be: there a late interrupt loses a rank and every later read
// shifts by one.
//
// WHY NOT a timer + DMA driving the select lines (Mutable Instruments Beads,
// beads/drivers/pots_adc.cc): Beads writes all three selects with ONE DMA
// store to one GPIO port's BSRR. Ours are split across two ports (D7/D8 =
// PG10/PG11, D9 = PB4), so that needs two DMA streams on one timer plus the
// ADC's own -- three DMA paths for what the interrupt does in a few hundred
// ns per conversion (16 x ~2300 scans/s, ~0.5% of the CPU). The scan rate
// is set by the conversion time either way, not by who steps the mux.
//
// Timing. ADC kernel = pll3_r (24.58 MHz stock; the codec planner keeps it
// in 24.13..25.06 MHz across 16..96 kHz, clock_core.h) / 2 (ASYNC_DIV2) / 2
// (the fixed internal divider on silicon rev V, which the Seed's H750 is --
// HAL ADC_ConfigureBoostMode: "divider by 2 for Rev.V") = 6.03..6.26 MHz.
// One conversion = 4 x (32.5 sampling + 8.5 conversion) = 164 cycles =
// 26.2..27.2 us. Two banks: 16 conversions = 0.42..0.44 ms a scan, ~2300
// scans/s: a fresh read of every pot for every audio block at block 32 up
// to fs ~= 74 kHz (above it a block sometimes sees the previous scan, and
// the Knobs, polled per scan with the time that elapsed, are unaffected).
// 4x oversampling is Beads' choice for its muxed pots (pots_adc.cc, ADC3);
// it costs 2x the noise of 16x, which the Knob's one-pole and hysteresis
// (controls_core.h) are sized for. 32.5-cycle sampling is Beads' too.
//
// Settling. A new select position's first conversion starts right after
// the select lines change. The 4051 settles in about a microsecond at 3.3
// V; the ADC holds the input at the END of each sampling window (32.5 cycles,
// ~5.3 us), so the switch transient is over before anything is held. The
// second bank's conversion starts ~27 us later.
//
// Readiness. raw_[][] reads 0 until a channel's first conversion, so nothing
// may treat the reads as pot positions before scans() >= 1 (every channel
// of every bank converted once): MuxPin::ready().
//
// Interrupt. ADC1 and ADC2 share ADC_IRQn. libDaisy never enables that IRQ
// (ADC1 runs on DMA), but HAL_ADC_Start_DMA does enable ADC1's OVERRUN
// interrupt, so once ADC_IRQn is on, an ADC1 overrun would land here and
// re-fire forever unless cleared: isr() clears and counts it (ovr1()).
// The priority is BELOW the audio DMA's (0): the audio callback preempts the
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
#include "analog_input.h"

class MuxAdc {
 public:
  static constexpr int CHANNELS = 8;   // per 4051
  static constexpr int MAX_BANKS = 2;  // 4051s sharing the select lines

  // One 4051's common: its pin (an ADC12 input) and that input's channel.
  struct Common {
    daisy::Pin pin;
    uint32_t   channel;
  };

  // `commons`: `banks` (1..MAX_BANKS) commons, bank 0 first; sel0..2: the
  // 4051s' A, B, C (shared by every bank).
  bool init(const Common* commons, int banks, daisy::Pin sel0, daisy::Pin sel1, daisy::Pin sel2) {
    using namespace daisy;
    if (banks < 1 || banks > MAX_BANKS) return false;
    banks_ = banks;
    for (int b = 0; b < banks_; b++) {
      GPIO::Config a;
      a.pin = commons[b].pin; a.mode = GPIO::Mode::ANALOG; a.pull = GPIO::Pull::NOPULL;
      pin_[b].Init(a);
      chan_[b] = commons[b].channel;
    }
    const Pin sels[3] = {sel0, sel1, sel2};
    for (int i = 0; i < 3; i++) {
      GPIO::Config s;
      s.pin = sels[i]; s.mode = GPIO::Mode::OUTPUT; s.pull = GPIO::Pull::NOPULL; s.speed = GPIO::Speed::LOW;
      sel_[i].Init(s);
    }
    cur_ = 0;
    bank_ = 0;
    select(cur_);

    __HAL_RCC_ADC12_CLK_ENABLE();
    hadc_ = ADC_HandleTypeDef();
    hadc_.Instance                      = ADC2;
    hadc_.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV2;    // same as libDaisy's ADC1 (shared common)
    hadc_.Init.Resolution               = ADC_RESOLUTION_16B;
    hadc_.Init.ScanConvMode             = ADC_SCAN_DISABLE;        // one channel per start (see the header)
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
    hadc_.Init.Oversampling.Ratio                 = 4;          // Beads' muxed-pot ratio (see the header)
    hadc_.Init.Oversampling.RightBitShift         = ADC_RIGHTBITSHIFT_2;
    hadc_.Init.Oversampling.TriggeredMode         = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
    hadc_.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
    if (HAL_ADC_Init(&hadc_) != HAL_OK) return false;

    // Configure every bank's channel (sampling time, and on the H7 the
    // input's PCSEL preselection bit), last bank first so bank 0's is left in
    // rank 1 for the first conversion. isr() then re-points rank 1 between
    // conversions (setRank), which the other banks' PCSEL bits allow.
    for (int b = banks_ - 1; b >= 0; b--) {
      ADC_ChannelConfTypeDef ch = {};
      ch.Channel      = chan_[b];
      ch.Rank         = ADC_REGULAR_RANK_1;
      ch.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;             // ~5.3 us: mux settles (see the header)
      ch.SingleDiff   = ADC_SINGLE_ENDED;
      ch.OffsetNumber = ADC_OFFSET_NONE;
      ch.Offset       = 0;
      if (HAL_ADC_ConfigChannel(&hadc_, &ch) != HAL_OK) return false;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc_, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK) return false;
    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);   // below the audio DMA (0)
    ok_ = true;
    return true;   // configured; start() begins the scan (with the instrument)
  }

  // Start / stop the scan (a PLL3 re-plan must happen between them). The IRQ
  // is masked across the HAL call in both directions, so isr() can never
  // re-arm ADSTART half-way through a stop. A restart begins a fresh select
  // position at bank 0.
  bool start() {
    if (!ok_) return false;
    bank_ = 0;
    setRank(chan_[0]);
    const bool started = HAL_ADC_Start_IT(&hadc_) == HAL_OK;   // EOC (+OVR) interrupt, one conversion
    if (started) HAL_NVIC_EnableIRQ(ADC_IRQn);
    return started;
  }
  void stop() {
    if (!ok_) return;
    HAL_NVIC_DisableIRQ(ADC_IRQn);
    HAL_ADC_Stop_IT(&hadc_);
  }

  // ADC_IRQHandler body. ADC2 end of conversion: latch, point at the next
  // bank's common (or step the muxes and go back to bank 0), start the next
  // conversion (reading DR clears EOC; ADSTART is already clear, so SQR1 may
  // be written). Register-level, not HAL_ADC_IRQHandler: that would dispatch
  // ADC2 into libDaisy's HAL_ADC_ConvCpltCallback, which is ADC1's DMA
  // handler.
  void isr() {
    if (ADC2->ISR & ADC_ISR_EOC) {
      raw_[bank_][cur_] = (uint16_t)ADC2->DR;
      if (++bank_ == banks_) {
        bank_ = 0;
        if (cur_ == CHANNELS - 1) scans_++;
        cur_ = (cur_ + 1) & (CHANNELS - 1);
        select(cur_);
      }
      if (banks_ > 1) setRank(chan_[bank_]);
      LL_ADC_REG_StartConversion(ADC2);   // masks CR's set-only command bits (RM0433)
    }
    if (ADC1->ISR & ADC_ISR_OVR) { ADC1->ISR = ADC_ISR_OVR; ovr1_++; }
  }

  // The latest completed read of bank `bank`, channel `ch`, 0..1.
  float value(int bank, int ch) const {
    return raw_[bank & (MAX_BANKS - 1)][ch & (CHANNELS - 1)] / 65535.0f;
  }
  bool  ok() const { return ok_; }
  int   banks() const { return banks_; }
  // Completed scans (every channel of every bank) since boot (>= 1: every
  // read is real), and ADC1 overruns (the Pod knobs; cleared here), ever.
  uint32_t scans() const { return scans_; }
  uint32_t ovr1()  const { return ovr1_; }

 private:
  static_assert((MAX_BANKS & (MAX_BANKS - 1)) == 0, "value() masks the bank index");

  void select(int ch) {
    sel_[0].Write((ch & 1) != 0);
    sel_[1].Write((ch & 2) != 0);
    sel_[2].Write((ch & 4) != 0);
  }
  // Rank 1 of the regular sequence (L = 0: one conversion) = `channel`.
  // HAL ADC_CHANNEL_x values are the LL encodings on the H7.
  static void setRank(uint32_t channel) {
    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, channel);
  }

  ADC_HandleTypeDef hadc_ = {};
  daisy::GPIO pin_[MAX_BANKS], sel_[3];
  uint32_t chan_[MAX_BANKS] = {0};
  // Written by isr(), read by the audio callback: single aligned stores and
  // loads, and isr() never runs inside the callback (lower priority).
  volatile uint16_t raw_[MAX_BANKS][CHANNELS] = {{0}};
  volatile uint32_t scans_ = 0, ovr1_ = 0;
  int  banks_ = 1, bank_ = 0, cur_ = 0;
  bool ok_ = false;
};

// One 4051 channel as an AnalogInput: a "virtual pin". A control wired
// through a mux reads exactly like one wired straight to an ADC (ControlPin);
// it never sees the mux. Not ready until the muxes have converted every
// channel once: before that its reads are 0, not a pot position.
class MuxPin : public AnalogInput {
 public:
  MuxPin(const MuxAdc& mux, int bank, int channel) : mux_(mux), bank_(bank), ch_(channel) {}
  float read() const override { return mux_.value(bank_, ch_); }
  bool  ready() const override { return mux_.scans() >= 1; }

 private:
  const MuxAdc& mux_;
  int           bank_;
  int           ch_;
};

#endif  // MUX_ADC_H
