// Unit tests for source_core.h: the QSPI sample-blob decode that used to live
// inline in dreamosc.cpp (un-host-compilable). Header validation, int16 ->
// float conversion, truncation to the buffer, and the wrap-pad that fills the
// rest of the buffer with material instead of silence.
//
// No CATCH_CONFIG_MAIN here -- test_stretch_core.cpp defines it.

#include "catch_amalgamated.hpp"

#include <cstring>
#include <vector>

#include "source_core.h"

namespace {

// Build a blob: header + int16 samples.
std::vector<uint8_t> make_blob(uint32_t magic, uint32_t count, uint32_t rate,
                               const std::vector<int16_t>& pcm) {
  std::vector<uint8_t> b(sizeof(SampleHeader) + pcm.size() * 2);
  SampleHeader h{magic, count, rate, 0};
  memcpy(b.data(), &h, sizeof(h));
  memcpy(b.data() + sizeof(h), pcm.data(), pcm.size() * 2);
  return b;
}

}  // namespace

TEST_CASE("sampleBlobValid: magic and count bounds") {
  SampleHeader ok{SAMPLE_MAGIC, 100, 48000, 0};
  REQUIRE(sampleBlobValid(&ok));
  SampleHeader badMagic{0x12345678u, 100, 48000, 0};
  REQUIRE_FALSE(sampleBlobValid(&badMagic));
  SampleHeader zero{SAMPLE_MAGIC, 0, 48000, 0};
  REQUIRE_FALSE(sampleBlobValid(&zero));          // empty is not a sample
  SampleHeader huge{SAMPLE_MAGIC, 9u * 1024 * 1024, 48000, 0};
  REQUIRE_FALSE(sampleBlobValid(&huge));          // past the 8 MB part: garbage
  REQUIRE(sampleBlobValid(&huge, 16u * 1024 * 1024));   // unless the cap says so
  // Erased flash reads 0xFF: must be rejected, never played.
  SampleHeader erased{0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};
  REQUIRE_FALSE(sampleBlobValid(&erased));
}

TEST_CASE("decodeSampleBlob: rejects an invalid header and leaves dst alone") {
  std::vector<float> dst(16, 7.0f);
  auto blob = make_blob(0xDEADBEEFu, 4, 48000, {1, 2, 3, 4});
  uint32_t rate = 0;
  REQUIRE(decodeSampleBlob(blob.data(), dst.data(), 16, rate) == 0);
  for (float v : dst) REQUIRE(v == 7.0f);
}

TEST_CASE("decodeSampleBlob: int16 -> float exactly as the firmware did (/32768)") {
  auto blob = make_blob(SAMPLE_MAGIC, 4, 44100, {0, 16384, -32768, 32767});
  std::vector<float> dst(4, 0.0f);
  uint32_t rate = 0;
  REQUIRE(decodeSampleBlob(blob.data(), dst.data(), 4, rate) == 4);
  REQUIRE(dst[0] == 0.0f);
  REQUIRE(dst[1] == 0.5f);
  REQUIRE(dst[2] == -1.0f);
  REQUIRE(dst[3] == Approx(32767.0f / 32768.0f));
  REQUIRE(rate == 44100);          // returned (not applied -- see the header NOTE)
}

TEST_CASE("decodeSampleBlob: a short blob is wrap-padded to fill the buffer") {
  // 3 samples into a 10-float buffer: dst[i] = dst[i % 3] for the remainder,
  // exactly the inline `sourceBuf[i % n]` this replaces.
  auto blob = make_blob(SAMPLE_MAGIC, 3, 48000, {100, 200, 300});
  std::vector<float> dst(10, 0.0f);
  uint32_t rate = 0;
  REQUIRE(decodeSampleBlob(blob.data(), dst.data(), 10, rate) == 3);
  for (uint32_t i = 0; i < 10; i++) REQUIRE(dst[i] == dst[i % 3]);
  REQUIRE(dst[0] == Approx(100.0f / 32768.0f));
  REQUIRE(dst[9] == dst[0]);                         // 9 % 3 == 0
  REQUIRE(dst[7] == Approx(200.0f / 32768.0f));      // 7 % 3 == 1
}

TEST_CASE("decodeSampleBlob: a long blob is truncated to the buffer, no padding") {
  std::vector<int16_t> pcm(20);
  for (int i = 0; i < 20; i++) pcm[i] = (int16_t)(i * 1000);
  auto blob = make_blob(SAMPLE_MAGIC, 20, 48000, pcm);
  std::vector<float> dst(8, 0.0f);
  uint32_t rate = 0;
  REQUIRE(decodeSampleBlob(blob.data(), dst.data(), 8, rate) == 8);
  for (int i = 0; i < 8; i++) REQUIRE(dst[i] == Approx(pcm[i] / 32768.0f));
}

TEST_CASE("fillStubSource: bounded, non-silent, spectrally varied material") {
  std::vector<float> dst(48000, 0.0f);
  fillStubSource(dst.data(), 48000, 48000.0f);
  float peak = 0.0f; double acc = 0.0;
  for (float v : dst) { if (fabsf(v) > peak) peak = fabsf(v); acc += (double)v * v; }
  REQUIRE(peak <= 1.0f);              // 0.5 + 0.3 + 0.2 = 1.0 worst case
  REQUIRE(peak > 0.5f);
  REQUIRE(sqrt(acc / 48000) > 0.3);   // not silence
  REQUIRE(dst[0] == 0.0f);            // sines start at zero
}
