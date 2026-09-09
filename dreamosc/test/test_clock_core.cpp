// Unit tests for clock_core.h: the PLL3 plan behind a native codec sample
// rate. The device applies the plan through the HAL and verifies from the
// registers; the host proves the plan is legal and lands on the rate.
//
// No CATCH_CONFIG_MAIN here -- test_stretch_core.cpp defines it.

#include "catch_amalgamated.hpp"

#include "clock_core.h"

namespace {

void require_legal(const Pll3Plan& p) {
  REQUIRE(p.refHz >= CLK_REF_MIN_HZ); REQUIRE(p.refHz <= CLK_REF_MAX_HZ);
  REQUIRE(p.vcoHz >= CLK_VCO_MIN_HZ); REQUIRE(p.vcoHz <= CLK_VCO_MAX_HZ);
  REQUIRE(p.n >= (uint32_t)CLK_PLL_N_MIN); REQUIRE(p.n <= (uint32_t)CLK_PLL_N_MAX);
  REQUIRE(p.fracn < (uint32_t)CLK_PLL_FRAC_DEN);
  REQUIRE(p.p >= 2u); REQUIRE(p.p <= (uint32_t)CLK_PLL_DIV_MAX);
  REQUIRE(p.q >= 1u); REQUIRE(p.q <= (uint32_t)CLK_PLL_DIV_MAX);
  REQUIRE(p.r >= 1u); REQUIRE(p.r <= (uint32_t)CLK_PLL_DIV_MAX);
}

}  // namespace

TEST_CASE("planPll3: libDaisy's stock 48 kHz geometry is +298 ppm; the plan is exact") {
  // The stock PLL3 (M 6, N 295, P 16) gives 49.1667 MHz / 1024 = 48014.3 Hz:
  // what every bench session through alpha5 actually ran at.
  double stock = pll3RateHz(CLK_PLL3_STOCK_M, CLK_PLL3_STOCK_N, 0, CLK_PLL3_STOCK_P);
  REQUIRE(stock == Approx(48014.32).margin(0.05));
  Pll3Plan p;
  REQUIRE(planPll3(48000, p));
  require_legal(p);
  REQUIRE(p.errPpm < 1.0);
  REQUIRE(p.fsHz == Approx(48000.0).margin(0.05));
}

TEST_CASE("planPll3: every standard rate lands within 1 ppm with a legal PLL") {
  const uint32_t rates[] = {16000, 22050, 24000, 32000, 44100, 48000, 88200, 96000};
  for (uint32_t fs : rates) {
    INFO("fs = " << fs);
    Pll3Plan p;
    REQUIRE(planPll3(fs, p));
    require_legal(p);
    REQUIRE(p.errPpm < 1.0);
    // The prediction is the HAL's own arithmetic, so the device check agrees.
    REQUIRE(pll3RateHz(p.m, p.n, p.fracn, p.p) == Approx(p.fsHz));
    // The ADC/I2C4 kernel (pll3_r) stays within 15% of stock: the knob ADC
    // keeps its timing across a rate change.
    REQUIRE(p.rHz == Approx(CLK_PLL3_STOCK_R_HZ).epsilon(0.15));
  }
}

TEST_CASE("planPll3: 44.1 kHz is the fractional plan the bench will read back") {
  Pll3Plan p;
  REQUIRE(planPll3(44100, p));
  REQUIRE(p.fracn != 0);                      // needs the fractional divider
  REQUIRE(p.fsHz == Approx(44100.0).margin(0.01));
  // The register readback: kernel clock is an integer Hz; MCKDIV 4.
  uint32_t kernel = (uint32_t)p.pHz;
  REQUIRE(codecRateMatches(44100, saiRateFromRegisters(kernel, CLK_SAI_MCKDIV)));
  // And the mismatch that a NOT-applied plan would show: stock PLL3 at
  // MCKDIV 4 reads 48014, which must not pass for 44100.
  REQUIRE_FALSE(codecRateMatches(44100, saiRateFromRegisters(49166666u, CLK_SAI_MCKDIV)));
}

TEST_CASE("planPll3: every integer rate in the codec range is reachable; outside is refused") {
  Pll3Plan p;
  REQUIRE_FALSE(planPll3(CODEC_FS_MIN - 1, p));
  REQUIRE_FALSE(planPll3(CODEC_FS_MAX + 1, p));
  REQUIRE_FALSE(planPll3(0, p));
  REQUIRE_FALSE(codecRateInRange(8000));
  // Sweep the whole range in 1 Hz steps: legal and under 2 ppm everywhere
  // (the fractional grid is ~0.02 Hz at 44.1 kHz, coarser at low rates).
  double worst = 0.0;
  for (uint32_t fs = CODEC_FS_MIN; fs <= CODEC_FS_MAX; fs += 1) {
    Pll3Plan q;
    if (!planPll3(fs, q)) { FAIL("no plan for " << fs); }
    if (q.refHz < CLK_REF_MIN_HZ || q.refHz > CLK_REF_MAX_HZ
        || q.vcoHz < CLK_VCO_MIN_HZ || q.vcoHz > CLK_VCO_MAX_HZ
        || q.n < (uint32_t)CLK_PLL_N_MIN || q.n > (uint32_t)CLK_PLL_N_MAX
        || q.p < 2 || q.p > (uint32_t)CLK_PLL_DIV_MAX) {
      FAIL("illegal plan for " << fs);
    }
    if (q.errPpm > worst) worst = q.errPpm;
  }
  WARN("worst rate error across the codec range: " << worst << " ppm");
  REQUIRE(worst < 2.0);
}

TEST_CASE("saiRateFromRegisters: MCKDIV 0 means divide by one") {
  REQUIRE(saiRateFromRegisters(12288000u, 0) == 48000u);
  REQUIRE(saiRateFromRegisters(49152000u, 4) == 48000u);
}

TEST_CASE("wm8731SamplingReg: only the two MCLK families the WM8731 accepts at 256fs") {
  uint8_t reg = 0xFF;
  REQUIRE(wm8731SamplingReg(48000, reg)); REQUIRE(reg == 0x00);
  REQUIRE(wm8731SamplingReg(44100, reg)); REQUIRE(reg == 0x20);
  REQUIRE_FALSE(wm8731SamplingReg(96000, reg));
  REQUIRE_FALSE(wm8731SamplingReg(32000, reg));
}
