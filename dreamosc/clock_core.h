// clock_core.h - platform-free planning for a NATIVE codec sample rate.
//
// The codec runs at the material's own rate: a 44.1 kHz file is played at
// 44.1 kHz, not resampled and not pitched up at 48 kHz. On the Daisy that is
// a PLL setting, nothing more: libDaisy's SAI1 is clocked from PLL3's P
// output and generates MCLK = pll3_p / MCKDIV, with the codec's frame clock at
// MCLK / 256, so
//
//     fs = pll3_p / (MCKDIV * 256)          (libDaisy leaves MCKDIV = 4)
//
// and choosing pll3_p = fs * 1024 sets the codec's rate exactly. PLL3 is
//
//     vco   = HSE / M * (N + FRACN / 8192)     HSE = 16 MHz, ref 2..4 MHz,
//     pll3_p = vco / P                          vco 192..836 MHz (wide)
//
// so with the fractional divider the achievable grid at 44.1 kHz is 0.02 Hz.
// This header decides the M/N/FRACN/P/Q/R for a rate and predicts the result;
// the firmware (codec_rate.h) applies it through the HAL and VERIFIES the
// rate it got from the registers before audio starts. Everything here is
// pure and host-tested (test_clock_core.cpp).
//
// PLL3 also clocks the ADC (knobs) and I2C4 from its R output, so the planner
// keeps pll3_r as close as it can to libDaisy's stock 24.58 MHz: the knob ADC
// never notices a rate change. (Q feeds nothing the Pod uses; kept near stock
// anyway.)

#ifndef CLOCK_CORE_H
#define CLOCK_CORE_H

#include <math.h>
#include <stdint.h>

// The Daisy Seed's crystal and libDaisy's PLL3 / SAI geometry (system.cpp,
// daisy_seed.cpp): M 6, N 295, P 16, Q 4, R 32, no fraction; SAI MCKDIV 4.
#define CLK_HSE_HZ         16000000.0
#define CLK_SAI_MCKDIV     4
#define CLK_PLL3_STOCK_M   6
#define CLK_PLL3_STOCK_N   295
#define CLK_PLL3_STOCK_P   16
#define CLK_PLL3_STOCK_Q   4
#define CLK_PLL3_STOCK_R   32
// Stock VCO (786.67 MHz) and its Q/R outputs (196.67 / 24.58 MHz): the targets
// the planner keeps Q and R near so the ADC clock stays put.
#define CLK_PLL3_STOCK_VCO_HZ (CLK_HSE_HZ / CLK_PLL3_STOCK_M * CLK_PLL3_STOCK_N)
#define CLK_PLL3_STOCK_Q_HZ   (CLK_PLL3_STOCK_VCO_HZ / CLK_PLL3_STOCK_Q)
#define CLK_PLL3_STOCK_R_HZ   (CLK_PLL3_STOCK_VCO_HZ / CLK_PLL3_STOCK_R)

// STM32H750 PLL3 limits (RM0433 / the HAL's IS_RCC_PLL3* asserts): ref
// 2..4 MHz for VCI range 1, wide VCO 192..836 MHz, N 4..512, FRACN 0..8191,
// P/Q/R 1..128 (P kept >= 2 so the P output is never the raw VCO).
#define CLK_REF_MIN_HZ   2000000.0
#define CLK_REF_MAX_HZ   4000000.0
#define CLK_VCO_MIN_HZ   192000000.0
#define CLK_VCO_MAX_HZ   836000000.0
#define CLK_PLL_N_MIN    4
#define CLK_PLL_N_MAX    512
#define CLK_PLL_FRAC_DEN 8192
#define CLK_PLL_DIV_MAX  128

// Rates the codec accepts at 256fs. The PCM3060 (Seed 2 DFM) is the tightest
// of the Seed codecs: 16..96 kHz. A file outside this is refused (no
// fallback: the instrument reports it and stays silent).
#define CODEC_FS_MIN 16000
#define CODEC_FS_MAX 96000

struct Pll3Plan {
  uint32_t m = 0, n = 0, fracn = 0, p = 0, q = 0, r = 0;
  double refHz = 0, vcoHz = 0, pHz = 0, qHz = 0, rHz = 0;
  double fsHz = 0;        // the rate the codec will actually run at
  double errPpm = 0;      // |fsHz - requested| / requested * 1e6
};

inline bool codecRateInRange(uint32_t fs) {
  return fs >= CODEC_FS_MIN && fs <= CODEC_FS_MAX;
}

// The SAI kernel clock that yields `fs` with libDaisy's MCKDIV.
inline double pll3TargetHz(uint32_t fs) { return (double)fs * 256.0 * CLK_SAI_MCKDIV; }

// The rate a PLL3 setting produces (the same arithmetic the HAL's
// HAL_RCCEx_GetPLL3ClockFreq uses, so the on-device verification and this
// prediction agree).
inline double pll3RateHz(uint32_t m, uint32_t n, uint32_t fracn, uint32_t p) {
  double vco = CLK_HSE_HZ / (double)m * ((double)n + (double)fracn / CLK_PLL_FRAC_DEN);
  return vco / (double)p / (256.0 * CLK_SAI_MCKDIV);
}

// Choose PLL3 for `fs`. Every M with a legal reference is tried; for each,
// P is the divider that lands the VCO nearest the stock 786.67 MHz (so Q and
// R can stay near stock), plus its neighbours; N + FRACN/8192 is the nearest
// fraction. The candidate with the smallest rate error wins, ties to the VCO
// nearest stock. Returns false if fs is outside the codec's range or nothing
// legal reaches it (never, for any rate in range -- tested).
inline bool planPll3(uint32_t fs, Pll3Plan& out) {
  if (!codecRateInRange(fs)) return false;
  const double target = pll3TargetHz(fs);
  bool found = false;
  Pll3Plan best;
  for (uint32_t m = 1; m <= 63; m++) {
    double ref = CLK_HSE_HZ / (double)m;
    if (ref < CLK_REF_MIN_HZ || ref > CLK_REF_MAX_HZ) continue;
    long pMid = lround(CLK_PLL3_STOCK_VCO_HZ / target);
    for (long p = pMid - 1; p <= pMid + 1; p++) {
      if (p < 2 || p > CLK_PLL_DIV_MAX) continue;
      double vco = target * (double)p;
      if (vco < CLK_VCO_MIN_HZ || vco > CLK_VCO_MAX_HZ) continue;
      double ratio = vco / ref;
      double nWhole = floor(ratio);
      long frac = lround((ratio - nWhole) * CLK_PLL_FRAC_DEN);
      if (frac >= CLK_PLL_FRAC_DEN) { nWhole += 1; frac = 0; }
      if (nWhole < CLK_PLL_N_MIN || nWhole > CLK_PLL_N_MAX) continue;
      Pll3Plan c;
      c.m = m; c.n = (uint32_t)nWhole; c.fracn = (uint32_t)frac; c.p = (uint32_t)p;
      c.refHz = ref;
      c.vcoHz = ref * ((double)c.n + (double)c.fracn / CLK_PLL_FRAC_DEN);
      c.pHz = c.vcoHz / (double)c.p;
      c.fsHz = c.pHz / (256.0 * CLK_SAI_MCKDIV);
      c.errPpm = fabs(c.fsHz - (double)fs) / (double)fs * 1e6;
      bool better = !found || c.errPpm < best.errPpm
                    || (c.errPpm == best.errPpm
                        && fabs(c.vcoHz - CLK_PLL3_STOCK_VCO_HZ) < fabs(best.vcoHz - CLK_PLL3_STOCK_VCO_HZ));
      if (better) { best = c; found = true; }
    }
  }
  if (!found) return false;
  // Q and R: nearest to stock, clamped to the divider range.
  long q = lround(best.vcoHz / CLK_PLL3_STOCK_Q_HZ);
  long r = lround(best.vcoHz / CLK_PLL3_STOCK_R_HZ);
  if (q < 1) q = 1;
  if (q > CLK_PLL_DIV_MAX) q = CLK_PLL_DIV_MAX;
  if (r < 1) r = 1;
  if (r > CLK_PLL_DIV_MAX) r = CLK_PLL_DIV_MAX;
  best.q = (uint32_t)q; best.r = (uint32_t)r;
  best.qHz = best.vcoHz / (double)best.q;
  best.rHz = best.vcoHz / (double)best.r;
  out = best;
  return true;
}

// The rate the SAI is actually producing, from the numbers the firmware reads
// back after applying a plan: the SAI1 kernel clock the HAL reports and the
// MCKDIV field of SAI1 block A's CR1. MCKDIV 0 means "divide by 1".
inline uint32_t saiRateFromRegisters(uint32_t saiKernelHz, uint32_t mckdivField) {
  uint32_t div = mckdivField == 0 ? 1 : mckdivField;
  return saiKernelHz / (div * 256u);
}

// Did the codec land on `fs`? The HAL reports the kernel clock as an integer
// and MCKDIV*256 divides it, so a correct setting can read 1 Hz low; 2 Hz is
// the acceptance band (a wrong plan is off by whole percent, not hertz).
inline bool codecRateMatches(uint32_t fs, uint32_t measured) {
  uint32_t d = fs > measured ? fs - measured : measured - fs;
  return d <= 2;
}

// WM8731 (Daisy Seed 1.1 only): the codec's sampling-control register
// (0x08) must name the MCLK/fs family it is being driven with; the AK4556
// (original Seed) and PCM3060 (Seed 2 DFM) follow MCLK on their own. With
// MCLK = 256fs the WM8731 has exactly two legal families -- 12.288 MHz
// (48 kHz, SR 0000) and 11.2896 MHz (44.1 kHz, SR 1000), normal mode, BOSR 0
// -- so on that board every other rate is refused. Returns false for a rate
// the WM8731 cannot be driven at through MCLK alone.
inline bool wm8731SamplingReg(uint32_t fs, uint8_t& reg) {
  switch (fs) {
    case 48000: reg = 0x00; return true;   // SR=0000, BOSR=0, USB=0
    case 44100: reg = 0x20; return true;   // SR=1000, BOSR=0, USB=0
    default:    return false;
  }
}

#endif  // CLOCK_CORE_H
