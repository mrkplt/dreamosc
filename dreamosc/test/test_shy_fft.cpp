// Direct correctness tests for shy_fft.h (Emilie Gillet's real FFT), vendored
// third-party code. No distinction from our own code for coverage purposes: it
// runs on-device and a bug in it is a bug in the instrument. stretch_core.h only
// ever instantiates ShyFFT<float, 4096>, which never exercises: the small-size
// (<=256) bit-reversal LUT path (num_passes <= 8 uses bit_rev_ instead of the
// static 256-entry table), the DirectTransform<0/1/2> and InverseTransform<0/1/2>
// base cases, or the Math<double> specialization. Exercise them directly here.

#include "catch_amalgamated.hpp"

#include <cmath>
#include <vector>

#include "shy_fft.h"

TEST_CASE("ShyFFT<float,64> round-trips (small size, bit_rev_ LUT path)") {
  // size=64 -> num_passes=6 <= 8, so Init() builds and Direct/Inverse use the
  // small bit_rev_[] LUT (shy_fft.h lines ~830-848), not the static 256 table.
  ShyFFT<float, 64, RotationPhasor> fft;
  fft.Init();

  float in[64], spec[64];
  for (int i = 0; i < 64; i++)
    in[i] = sinf(2.0f * (float)M_PI * 5.0f * i / 64.0f);

  float work[64];
  std::copy(in, in + 64, work);
  fft.Direct(work, spec);
  fft.Inverse(spec, work);

  // Direct+Inverse scales by N (matches stretch_core.h's own measured
  // convention), so compare up to that factor.
  for (int i = 0; i < 64; i++)
    REQUIRE(work[i] / 64.0f == Approx(in[i]).margin(1e-4));
}

TEST_CASE("ShyFFT<float,16> round-trips (exercises small DirectTransform bases)") {
  // size=16 -> num_passes=4, still <=8 (bit_rev_ path), and small enough that
  // different butterfly-pass-count parity is exercised across sizes.
  ShyFFT<float, 16, RotationPhasor> fft;
  fft.Init();

  float in[16], spec[16];
  for (int i = 0; i < 16; i++) in[i] = (i % 3 == 0) ? 1.0f : -0.5f;

  float work[16];
  std::copy(in, in + 16, work);
  fft.Direct(work, spec);
  fft.Inverse(spec, work);
  for (int i = 0; i < 16; i++)
    REQUIRE(work[i] / 16.0f == Approx(in[i]).margin(1e-4));
}

TEST_CASE("ShyFFT<float,32> round-trips (odd pass count hits the post-loop "
         "copy-back branch)") {
  // InverseTransform's post-loop "copy data if necessary" branch (d == output)
  // fires or not depending on num_passes parity from the ping-pong across
  // passes. size=32 -> num_passes=5 lands the ping-pong on the side that needs
  // the copy, which sizes 16/64/128 tested above did not exercise.
  ShyFFT<float, 32, RotationPhasor> fft;
  fft.Init();

  float in[32], spec[32];
  for (int i = 0; i < 32; i++) in[i] = (i % 2) ? 1.0f : -1.0f;

  float work[32];
  std::copy(in, in + 32, work);
  fft.Direct(work, spec);
  fft.Inverse(spec, work);
  for (int i = 0; i < 32; i++)
    REQUIRE(work[i] / 32.0f == Approx(in[i]).margin(1e-4));
}

TEST_CASE("ShyFFT<double,64> round-trips (exercises Math<double> specialization)") {
  // stretch_core.h only ever uses float; instantiate double directly so
  // Math<double>::pi/sqrt_2_div_2/cos/sin (shy_fft.h lines ~142-149) run.
  ShyFFT<double, 64, RotationPhasor> fft;
  fft.Init();

  double in[64], spec[64];
  for (int i = 0; i < 64; i++)
    in[i] = std::sin(2.0 * M_PI * 3.0 * i / 64.0);

  double work[64];
  std::copy(in, in + 64, work);
  fft.Direct(work, spec);
  fft.Inverse(spec, work);
  for (int i = 0; i < 64; i++)
    REQUIRE(work[i] / 64.0 == Approx(in[i]).margin(1e-8));
}

TEST_CASE("ShyFFT magnitude spectrum locates a pure tone in the right bin") {
  // Correctness beyond round-trip: a pure sine at bin k should show its energy
  // concentrated at bin k (and its mirror), not smeared or misplaced.
  const int N = 128;
  ShyFFT<float, N, RotationPhasor> fft;
  fft.Init();

  const int kBin = 10;
  float in[N], spec[N];
  for (int i = 0; i < N; i++)
    in[i] = sinf(2.0f * (float)M_PI * kBin * i / N);

  float work[N];
  std::copy(in, in + N, work);
  fft.Direct(work, spec);

  // Split layout: real in [0,N/2), imag in [N/2,N) — same convention
  // stretch_core.h relies on.
  float* re = &spec[0];
  float* im = &spec[N / 2];
  float peak_mag = 0.0f;
  int peak_bin = -1;
  for (int k = 1; k < N / 2; k++) {
    float mag = std::sqrt(re[k] * re[k] + im[k] * im[k]);
    if (mag > peak_mag) { peak_mag = mag; peak_bin = k; }
  }
  REQUIRE(peak_bin == kBin);
}
