// Unit tests for source_core.h: the QSPI sample-blob decode that used to live
// inline in dreamosc.cpp (un-host-compilable). Header validation, int16 ->
// float conversion, truncation to the buffer, and the wrap-pad that fills the
// rest of the buffer with material instead of silence.
//
// No CATCH_CONFIG_MAIN here -- test_stretch_core.cpp defines it.

#include "catch_amalgamated.hpp"

#include <cstring>
#include <string>
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

// --- "the first WAV on the card" ---------------------------------------------

TEST_CASE("isCandidateWav: plain .wav files only, any case, never dotfiles or directories") {
  REQUIRE(isCandidateWav("cw_amen13_173.wav", false));
  REQUIRE(isCandidateWav("LOOP.WAV", false));
  REQUIRE(isCandidateWav("a.Wav", false));
  REQUIRE_FALSE(isCandidateWav("._cw_amen13_173.wav", false));   // macOS AppleDouble twin
  REQUIRE_FALSE(isCandidateWav(".hidden.wav", false));
  REQUIRE_FALSE(isCandidateWav("samples.wav", true));            // a directory
  REQUIRE_FALSE(isCandidateWav("notes.txt", false));
  REQUIRE_FALSE(isCandidateWav("wav", false));
  REQUIRE_FALSE(isCandidateWav(".wav", false));
  REQUIRE_FALSE(isCandidateWav("", false));
  REQUIRE_FALSE(isCandidateWav(nullptr, false));
}

TEST_CASE("pickFirstWav: alphabetical, case-insensitive, over a FAT-ordered walk") {
  char best[32];
  best[0] = '\0';
  // The card was written in this order (FAT order); "first" must not mean that.
  REQUIRE(pickFirstWav("zebra.wav", false, best, sizeof(best)));
  REQUIRE(std::string(best) == "zebra.wav");
  REQUIRE_FALSE(pickFirstWav("._Amen.wav", false, best, sizeof(best)));  // skipped
  REQUIRE(pickFirstWav("Amen.wav", false, best, sizeof(best)));
  REQUIRE(std::string(best) == "Amen.wav");
  REQUIRE_FALSE(pickFirstWav("break.wav", false, best, sizeof(best)));  // b > a
  REQUIRE(pickFirstWav("amen.wav", false, best, sizeof(best)) == false); // equal: keeps the first seen
  REQUIRE(std::string(best) == "Amen.wav");
  REQUIRE(pickFirstWav("ame.wav", false, best, sizeof(best)));           // prefix sorts first
  REQUIRE(std::string(best) == "ame.wav");
  REQUIRE_FALSE(pickFirstWav("aaa", false, best, sizeof(best)));         // not a wav
  // A long name is truncated to the buffer, never overrun.
  char tiny[6]; tiny[0] = '\0';
  REQUIRE(pickFirstWav("abcdefgh.wav", false, tiny, sizeof(tiny)));
  REQUIRE(std::string(tiny) == "abcde");
}

// --- WAV parse + read over the reader interface ------------------------------

namespace {

// Build a WAV in memory: optional extra chunks before data, given fmt fields.
struct WavBuild {
  uint16_t format = 1, channels = 1, bits = 16; uint32_t rate = 48000;
  std::vector<std::vector<uint8_t>> before;   // chunks placed before `data`
  uint32_t fmtExtra = 0;                      // extra bytes in the fmt chunk (18-byte fmt etc.)
  bool lieDataLen = false;                    // write 0xFFFFFFFF as the data length
};
void put16(std::vector<uint8_t>& b, uint16_t v) { b.push_back(v & 0xff); b.push_back(v >> 8); }
void put32(std::vector<uint8_t>& b, uint32_t v) { for (int i = 0; i < 4; i++) b.push_back((v >> (8 * i)) & 0xff); }
std::vector<uint8_t> chunk(const char* id, const std::vector<uint8_t>& body) {
  std::vector<uint8_t> c(id, id + 4);
  put32(c, (uint32_t)body.size());
  c.insert(c.end(), body.begin(), body.end());
  if (body.size() & 1) c.push_back(0);        // pad byte
  return c;
}
std::vector<uint8_t> make_wav(const WavBuild& w, const std::vector<int16_t>& pcm) {
  std::vector<uint8_t> fmt;
  put16(fmt, w.format); put16(fmt, w.channels); put32(fmt, w.rate);
  put32(fmt, w.rate * w.channels * w.bits / 8); put16(fmt, (uint16_t)(w.channels * w.bits / 8)); put16(fmt, w.bits);
  for (uint32_t i = 0; i < w.fmtExtra; i++) fmt.push_back(0);
  std::vector<uint8_t> data;
  for (int16_t s : pcm) put16(data, (uint16_t)s);
  std::vector<uint8_t> body(4, 0); memcpy(body.data(), "WAVE", 4);
  auto f = chunk("fmt ", fmt); body.insert(body.end(), f.begin(), f.end());
  for (auto& c : w.before) body.insert(body.end(), c.begin(), c.end());
  auto d = chunk("data", data);
  if (w.lieDataLen) { d[4] = d[5] = d[6] = d[7] = 0xff; }
  body.insert(body.end(), d.begin(), d.end());
  std::vector<uint8_t> out(4, 0); memcpy(out.data(), "RIFF", 4);
  put32(out, (uint32_t)body.size());
  out.insert(out.end(), body.begin(), body.end());
  return out;
}
struct Loaded { SourceErr err; WavInfo w; std::vector<float> s; };
Loaded load(const std::vector<uint8_t>& bytes, uint32_t cap = 1 << 16) {
  MemReader r(bytes.data(), (uint32_t)bytes.size());
  Loaded L; L.err = parseWav(r, L.w);
  if (L.err != SE_OK) return L;
  L.s.assign(cap, 0.0f);
  uint8_t block[64];                          // deliberately tiny, not a frame multiple of 6
  SourceErr rerr;
  uint32_t n = readWavMono(r, L.w, L.s.data(), cap, block, sizeof(block), rerr);
  L.err = rerr; L.s.resize(n);
  return L;
}

}  // namespace

TEST_CASE("parseWav/readWavMono: a canonical 44-byte mono file") {
  std::vector<int16_t> pcm = {0, 16384, -16384, 32767, -32768};
  auto L = load(make_wav(WavBuild{}, pcm));
  REQUIRE(L.err == SE_OK);
  REQUIRE(L.w.rate == 48000); REQUIRE(L.w.channels == 1); REQUIRE(L.w.bits == 16);
  REQUIRE(L.w.dataBytes == 10);
  REQUIRE(L.s.size() == 5);
  REQUIRE(L.s[1] == 16384 / 32768.0f);
  REQUIRE(L.s[3] == 32767 / 32768.0f);
  REQUIRE(L.s[4] == -1.0f);
}

TEST_CASE("parseWav: LIST and odd-length chunks before data are skipped with their pad byte") {
  WavBuild w;
  w.before.push_back(chunk("LIST", std::vector<uint8_t>(37, 'x')));   // odd: pad byte follows
  w.before.push_back(chunk("fact", std::vector<uint8_t>(4, 0)));
  w.fmtExtra = 2;                                                     // an 18-byte fmt chunk
  w.rate = 44100;
  std::vector<int16_t> pcm = {100, 200, 300};
  auto L = load(make_wav(w, pcm));
  REQUIRE(L.err == SE_OK);
  REQUIRE(L.w.rate == 44100);
  REQUIRE(L.s.size() == 3);
  REQUIRE(L.s[2] == 300 / 32768.0f);
}

TEST_CASE("readWavMono: stereo folds to the mean of the channels (as wav2raw.py does)") {
  WavBuild w; w.channels = 2;
  std::vector<int16_t> pcm = {1000, 3000,  -2000, 2000,  32767, 32767};   // L R pairs
  auto L = load(make_wav(w, pcm));
  REQUIRE(L.err == SE_OK);
  REQUIRE(L.s.size() == 3);
  REQUIRE(L.s[0] == 4000 / (2 * 32768.0f));
  REQUIRE(L.s[1] == 0.0f);
  REQUIRE(L.s[2] == 65534 / (2 * 32768.0f));
}

TEST_CASE("parseWav: 24-bit, float and EXTENSIBLE files are refused with SE_BAD_FORMAT") {
  WavBuild b24; b24.bits = 24;
  REQUIRE(load(make_wav(b24, {})).err == SE_BAD_FORMAT);
  WavBuild f32; f32.format = 3; f32.bits = 32;
  REQUIRE(load(make_wav(f32, {})).err == SE_BAD_FORMAT);
  WavBuild ext; ext.format = 0xFFFE; ext.fmtExtra = 24;
  REQUIRE(load(make_wav(ext, {})).err == SE_BAD_FORMAT);
}

TEST_CASE("parseWav: not a WAV, missing fmt, missing data") {
  std::vector<uint8_t> junk(64, 'J');
  REQUIRE(load(junk).err == SE_NOT_RIFF);
  std::vector<uint8_t> riffOnly = {'R','I','F','F', 4,0,0,0, 'W','A','V','E'};
  REQUIRE(load(riffOnly).err == SE_NO_FMT);
  // fmt then nothing: no data chunk.
  auto full = make_wav(WavBuild{}, {1, 2});
  std::vector<uint8_t> noData(full.begin(), full.begin() + 12 + 8 + 16);
  REQUIRE(load(noData).err == SE_NO_DATA);
}

TEST_CASE("parseWav: a lying data length is clamped to what the file holds; cap truncates") {
  WavBuild w; w.lieDataLen = true;
  std::vector<int16_t> pcm(1000, 7);
  auto bytes = make_wav(w, pcm);
  auto L = load(bytes);
  REQUIRE(L.err == SE_OK);
  REQUIRE(L.w.dataBytes == 2000);
  REQUIRE(L.s.size() == 1000);
  auto T = load(bytes, 300);                  // buffer smaller than the file
  REQUIRE(T.s.size() == 300);
  auto E = load(make_wav(WavBuild{}, {}));    // a data chunk with no samples
  REQUIRE(E.err == SE_EMPTY);
  REQUIRE(E.s.empty());
}
