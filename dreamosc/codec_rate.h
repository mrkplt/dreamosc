// codec_rate.h - run the codec at the material's own sample rate (device edge).
//
// The decision (which PLL3 setting yields fs, what the registers must read
// back) is clock_core.h, host-tested. This header is only the HAL calls:
// reprogram PLL3 and the SAI1 kernel-clock mux, read the achieved rate back
// from the RCC and the SAI, and on a Seed 1.1 tell the WM8731 which MCLK
// family it is being driven with. It never touches the SAI block itself:
// libDaisy computed MCKDIV once at Init() (4, from the stock 49.17 MHz), the
// SAI keeps it, and fs = pll3_p / (4 * 256) follows the PLL. AudioHandle's
// Start/Stop only start and stop the DMA stream (audio.cpp), so the SAI
// config survives a stop/start around a rate change.
//
// Preconditions the caller owns: audio STOPPED (the DMA stream is off; a PLL
// relock under a running stream would glitch) and the ADC STOPPED (PLL3's R
// output is the ADC kernel clock; the HAL disables PLL3 to reprogram it).
//
// Verified, not assumed: the rate is read back from the registers and a
// mismatch is an error the caller must treat as "no instrument" -- the
// codec's rate is the one number every duration and hop is measured in.

#ifndef CODEC_RATE_H
#define CODEC_RATE_H

#include <stdint.h>

#include "daisy_seed.h"
#include "clock_core.h"

enum CodecRateErr : uint8_t {
  CR_OK = 0,
  CR_RANGE,       // fs outside the codec's range (clock_core.h)
  CR_PLL,         // HAL refused the PLL3 configuration
  CR_VERIFY,      // registers read back a different rate
  CR_WM8731,      // Seed 1.1: the WM8731 cannot be driven at fs through MCLK
  CR_I2C,         // Seed 1.1: the WM8731 register write failed
};

struct CodecRate {
  uint32_t     requested = 0;
  uint32_t     measured  = 0;      // from the registers, after the change
  Pll3Plan     plan;
  CodecRateErr err = CR_OK;
};

// Seed 1.1 only: the WM8731's sampling-control register over the same I2C2
// pins libDaisy used to bring the codec up (daisy_seed.cpp). The AK4556
// (original Seed) and the PCM3060 (Seed 2 DFM) need nothing: they follow MCLK.
inline CodecRateErr codecTellWm8731(uint32_t fs) {
  using namespace daisy;
  uint8_t reg;
  if (!wm8731SamplingReg(fs, reg)) return CR_WM8731;
  I2CHandle::Config cfg;
  cfg.mode           = I2CHandle::Config::Mode::I2C_MASTER;
  cfg.periph         = I2CHandle::Config::Peripheral::I2C_2;
  cfg.speed          = I2CHandle::Config::Speed::I2C_400KHZ;
  cfg.pin_config.scl = Pin(PORTH, 4);
  cfg.pin_config.sda = Pin(PORTB, 11);
  I2CHandle i2c;
  if (i2c.Init(cfg) != I2CHandle::Result::OK) return CR_I2C;
  // WM8731 control word: 7-bit register address, 9-bit data (codec_wm8731.cpp).
  const uint8_t  addr = 0x08;                       // CODEC_REG_SAMPLE_RATE
  const uint16_t data = reg;
  uint8_t buf[2] = {(uint8_t)(((addr << 1) & 0xfe) | ((data >> 8) & 0x01)),
                    (uint8_t)(data & 0xff)};
  if (i2c.TransmitBlocking(0x1A, buf, 2, 250) != I2CHandle::Result::OK) return CR_I2C;
  System::Delay(10);
  return CR_OK;
}

// Set the codec to `fs`. Returns true when the registers confirm it.
inline bool setCodecRate(daisy::DaisySeed& seed, uint32_t fs, CodecRate& out) {
  using namespace daisy;
  out = CodecRate();
  out.requested = fs;
  if (!planPll3(fs, out.plan)) { out.err = CR_RANGE; return false; }

  RCC_PeriphCLKInitTypeDef pc = {};
  pc.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  pc.Sai1ClockSelection   = RCC_SAI1CLKSOURCE_PLL3;
  pc.PLL3.PLL3M           = out.plan.m;
  pc.PLL3.PLL3N           = out.plan.n;
  pc.PLL3.PLL3P           = out.plan.p;
  pc.PLL3.PLL3Q           = out.plan.q;
  pc.PLL3.PLL3R           = out.plan.r;
  pc.PLL3.PLL3RGE         = RCC_PLL3VCIRANGE_1;    // ref 2..4 MHz (planPll3 guarantees)
  pc.PLL3.PLL3VCOSEL      = RCC_PLL3VCOWIDE;       // 192..836 MHz
  pc.PLL3.PLL3FRACN       = out.plan.fracn;
  if (HAL_RCCEx_PeriphCLKConfig(&pc) != HAL_OK) { out.err = CR_PLL; return false; }

  uint32_t kernel = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SAI1);
  uint32_t mckdiv = (SAI1_Block_A->CR1 & SAI_xCR1_MCKDIV_Msk) >> SAI_xCR1_MCKDIV_Pos;
  out.measured = saiRateFromRegisters(kernel, mckdiv);
  if (!codecRateMatches(fs, out.measured)) { out.err = CR_VERIFY; return false; }

  if (seed.CheckBoardVersion() == DaisySeed::BoardVersion::DAISY_SEED_1_1) {
    out.err = codecTellWm8731(fs);
    if (out.err != CR_OK) return false;
  }
  return true;
}

#endif  // CODEC_RATE_H
