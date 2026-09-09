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
// `raw` is the data chunk's bytes, already laid out at `bits` per sample.
struct WavBuild {
  uint16_t format = 1, channels = 1, bits = 16; uint32_t rate = 48000;
  uint16_t blockAlign = 0;                    // 0 = channels * bits / 8
  std::vector<std::vector<uint8_t>> before;   // chunks placed before `data`
  uint32_t fmtExtra = 0;                      // extra bytes in the fmt chunk (18-byte fmt etc.)
  uint16_t subFormat = 0;                     // EXTENSIBLE: the wrapped tag (writes a 40-byte fmt)
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
std::vector<uint8_t> make_wav_raw(const WavBuild& w, const std::vector<uint8_t>& data) {
  uint16_t ba = w.blockAlign ? w.blockAlign : (uint16_t)(w.channels * w.bits / 8);
  std::vector<uint8_t> fmt;
  put16(fmt, w.format); put16(fmt, w.channels); put32(fmt, w.rate);
  put32(fmt, w.rate * ba); put16(fmt, ba); put16(fmt, w.bits);
  if (w.format == 0xFFFE) {
    put16(fmt, 22); put16(fmt, w.bits); put32(fmt, 0);        // cbSize, validBits, channelMask
    put16(fmt, w.subFormat);                                  // SubFormat GUID: tag first
    static const uint8_t guidTail[14] = {0x00,0x00,0x00,0x00,0x10,0x00,0x80,0x00,0x00,0xAA,0x00,0x38,0x9B,0x71};
    fmt.insert(fmt.end(), guidTail, guidTail + 14);
  }
  for (uint32_t i = 0; i < w.fmtExtra; i++) fmt.push_back(0);
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
std::vector<uint8_t> make_wav(const WavBuild& w, const std::vector<int16_t>& pcm) {
  std::vector<uint8_t> data;
  for (int16_t s : pcm) put16(data, (uint16_t)s);
  return make_wav_raw(w, data);
}
struct Loaded { SourceErr err; WavInfo w; std::vector<float> s; };
// Parse, then read every frame through a deliberately tiny stage with the
// given alignment (1 = byte-exact; 512 = the device's sector rounding).
Loaded load(const std::vector<uint8_t>& bytes, uint32_t align = 1, uint32_t stageBytes = 64) {
  MemReader r(bytes.data(), (uint32_t)bytes.size());
  Loaded L; L.err = parseWav(r, L.w);
  if (L.err != SE_OK) return L;
  L.s.assign(L.w.frames, 0.0f);
  std::vector<uint8_t> stage(stageBytes < align + 8 ? align + 8 : stageBytes);
  SourceErr rerr;
  uint32_t n = readWavFrames(r, L.w, 0, L.w.frames, L.s.data(), stage.data(), (uint32_t)stage.size(), align, rerr);
  L.err = rerr; L.s.resize(n);
  if (n == 0 && rerr == SE_OK) L.err = SE_EMPTY;
  return L;
}

}  // namespace

TEST_CASE("parseWav/readWavFrames: a canonical 44-byte mono file") {
  std::vector<int16_t> pcm = {0, 16384, -16384, 32767, -32768};
  auto L = load(make_wav(WavBuild{}, pcm));
  REQUIRE(L.err == SE_OK);
  REQUIRE(L.w.rate == 48000); REQUIRE(L.w.channels == 1); REQUIRE(L.w.bits == 16);
  REQUIRE(L.w.blockAlign == 2); REQUIRE(L.w.dataOffset == 44);
  REQUIRE(L.w.dataBytes == 10); REQUIRE(L.w.frames == 5);
  REQUIRE(L.w.codec == WAV_S16);
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

TEST_CASE("readWavFrames: stereo takes the LEFT channel only (no fold)") {
  WavBuild w; w.channels = 2;
  std::vector<int16_t> pcm = {1000, 3000,  -2000, 2000,  32767, -32768};   // L R pairs
  auto L = load(make_wav(w, pcm));
  REQUIRE(L.err == SE_OK);
  REQUIRE(L.w.blockAlign == 4);
  REQUIRE(L.s.size() == 3);
  REQUIRE(L.s[0] == 1000 / 32768.0f);
  REQUIRE(L.s[1] == -2000 / 32768.0f);
  REQUIRE(L.s[2] == 32767 / 32768.0f);
  // Six channels: still channel 0.
  WavBuild six; six.channels = 6;
  std::vector<int16_t> pcm6 = {7, 1, 2, 3, 4, 5,  -7, 1, 2, 3, 4, 5};
  auto S = load(make_wav(six, pcm6));
  REQUIRE(S.s.size() == 2);
  REQUIRE(S.s[0] == 7 / 32768.0f); REQUIRE(S.s[1] == -7 / 32768.0f);
}

TEST_CASE("wavDecodeSample: every supported depth scales by its container width") {
  // 8-bit is UNSIGNED: 0x80 is silence, 0x00 is -1, 0xFF is +127/128.
  const uint8_t u8[3] = {0x80, 0x00, 0xFF};
  REQUIRE(wavDecodeSample(u8 + 0, WAV_U8) == 0.0f);
  REQUIRE(wavDecodeSample(u8 + 1, WAV_U8) == -1.0f);
  REQUIRE(wavDecodeSample(u8 + 2, WAV_U8) == 127 / 128.0f);
  // 24-bit packed little-endian, sign-extended: 0x800000 = -1, 0x7FFFFF = max, 0xFFFFFF = -1 LSB.
  const uint8_t s24[12] = {0x00,0x00,0x80,  0xFF,0xFF,0x7F,  0xFF,0xFF,0xFF,  0x00,0x00,0x40};
  REQUIRE(wavDecodeSample(s24 + 0, WAV_S24) == -1.0f);
  REQUIRE(wavDecodeSample(s24 + 3, WAV_S24) == 8388607 / 8388608.0f);
  REQUIRE(wavDecodeSample(s24 + 6, WAV_S24) == -1 / 8388608.0f);
  REQUIRE(wavDecodeSample(s24 + 9, WAV_S24) == 0.5f);
  // 32-bit int.
  const uint8_t s32[8] = {0x00,0x00,0x00,0x80,  0x00,0x00,0x00,0x40};
  REQUIRE(wavDecodeSample(s32 + 0, WAV_S32) == -1.0f);
  REQUIRE(wavDecodeSample(s32 + 4, WAV_S32) == 0.5f);
  // float32 straight through, including values past full scale (the synth clamps).
  float f[2] = {0.25f, -1.5f}; uint8_t fb[8]; memcpy(fb, f, 8);
  REQUIRE(wavDecodeSample(fb + 0, WAV_F32) == 0.25f);
  REQUIRE(wavDecodeSample(fb + 4, WAV_F32) == -1.5f);
}

TEST_CASE("parseWav: 8/24/32-bit PCM and float files read end to end") {
  WavBuild b8; b8.bits = 8;
  auto L8 = load(make_wav_raw(b8, {0x80, 0xC0, 0x40}));
  REQUIRE(L8.err == SE_OK); REQUIRE(L8.w.codec == WAV_U8); REQUIRE(L8.w.blockAlign == 1);
  REQUIRE(L8.s.size() == 3); REQUIRE(L8.s[1] == 0.5f); REQUIRE(L8.s[2] == -0.5f);

  WavBuild b24; b24.bits = 24; b24.channels = 2;                 // 6-byte frames
  std::vector<uint8_t> d24 = {0x00,0x00,0x40, 0,0,0,   0x00,0x00,0xC0, 0,0,0};
  auto L24 = load(make_wav_raw(b24, d24));
  REQUIRE(L24.err == SE_OK); REQUIRE(L24.w.codec == WAV_S24); REQUIRE(L24.w.blockAlign == 6);
  REQUIRE(L24.s.size() == 2); REQUIRE(L24.s[0] == 0.5f); REQUIRE(L24.s[1] == -0.5f);

  WavBuild b32; b32.bits = 32;
  auto L32 = load(make_wav_raw(b32, {0x00,0x00,0x00,0x40}));
  REQUIRE(L32.err == SE_OK); REQUIRE(L32.w.codec == WAV_S32); REQUIRE(L32.s[0] == 0.5f);

  WavBuild f32; f32.format = 3; f32.bits = 32;
  float v = 0.75f; std::vector<uint8_t> df(4); memcpy(df.data(), &v, 4);
  auto LF = load(make_wav_raw(f32, df));
  REQUIRE(LF.err == SE_OK); REQUIRE(LF.w.codec == WAV_F32); REQUIRE(LF.s[0] == 0.75f);
}

TEST_CASE("parseWav: WAVE_FORMAT_EXTENSIBLE resolves to the wrapped PCM or float tag") {
  WavBuild ext; ext.format = 0xFFFE; ext.subFormat = 1; ext.bits = 24;
  auto L = load(make_wav_raw(ext, {0x00,0x00,0x40}));
  REQUIRE(L.err == SE_OK); REQUIRE(L.w.format == 1); REQUIRE(L.w.codec == WAV_S24);
  REQUIRE(L.s.size() == 1); REQUIRE(L.s[0] == 0.5f);
  WavBuild extF; extF.format = 0xFFFE; extF.subFormat = 3; extF.bits = 32;
  float v = -0.5f; std::vector<uint8_t> d(4); memcpy(d.data(), &v, 4);
  auto F = load(make_wav_raw(extF, d));
  REQUIRE(F.err == SE_OK); REQUIRE(F.w.format == 3); REQUIRE(F.s[0] == -0.5f);
  // EXTENSIBLE wrapping something else (ADPCM tag 2) is refused.
  WavBuild bad; bad.format = 0xFFFE; bad.subFormat = 2;
  REQUIRE(load(make_wav(bad, {1})).err == SE_BAD_FORMAT);
}

TEST_CASE("parseWav: unsupported formats are refused with SE_BAD_FORMAT") {
  WavBuild b12; b12.bits = 12;
  REQUIRE(load(make_wav(b12, {})).err == SE_BAD_FORMAT);
  WavBuild f64; f64.format = 3; f64.bits = 64;
  REQUIRE(load(make_wav(f64, {})).err == SE_BAD_FORMAT);
  WavBuild adpcm; adpcm.format = 2;
  REQUIRE(load(make_wav(adpcm, {})).err == SE_BAD_FORMAT);
  WavBuild none; none.channels = 0;
  REQUIRE(load(make_wav(none, {})).err == SE_BAD_FORMAT);
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

TEST_CASE("parseWav: a lying data length is clamped to what the file holds; a broken blockAlign recovers") {
  WavBuild w; w.lieDataLen = true;
  std::vector<int16_t> pcm(1000, 7);
  auto bytes = make_wav(w, pcm);
  auto L = load(bytes);
  REQUIRE(L.err == SE_OK);
  REQUIRE(L.w.dataBytes == 2000);
  REQUIRE(L.w.frames == 1000);
  REQUIRE(L.s.size() == 1000);
  auto E = load(make_wav(WavBuild{}, {}));    // a data chunk with no samples
  REQUIRE(E.err == SE_EMPTY);
  REQUIRE(E.s.empty());
  WavBuild zeroAlign; zeroAlign.channels = 2; zeroAlign.blockAlign = 1;   // fmt lies: 1 < 4
  auto Z = load(make_wav(zeroAlign, {5, 6, 7, 8}));
  REQUIRE(Z.err == SE_OK); REQUIRE(Z.w.blockAlign == 4); REQUIRE(Z.s.size() == 2);
}

TEST_CASE("readWavFrames: sector-aligned staging equals byte-exact reads for any frame size and range") {
  // 6-byte frames (24-bit stereo) never divide a 512-byte sector or a stage;
  // every frame that straddles a staged read must be re-read, not dropped.
  WavBuild b24; b24.bits = 24; b24.channels = 2;
  b24.before.push_back(chunk("LIST", std::vector<uint8_t>(101, 'x')));   // shifts dataOffset off alignment
  std::vector<uint8_t> data;
  for (uint32_t f = 0; f < 3000; f++) {
    int32_t v = (int32_t)(f * 2000) - 3000000;                // channel 0
    data.push_back(v & 0xff); data.push_back((v >> 8) & 0xff); data.push_back((v >> 16) & 0xff);
    data.push_back(0x11); data.push_back(0x22); data.push_back(0x33);   // channel 1: ignored
  }
  auto bytes = make_wav_raw(b24, data);
  auto exact = load(bytes, 1, 4096);
  auto sect  = load(bytes, 512, 4096);
  REQUIRE(exact.err == SE_OK); REQUIRE(sect.err == SE_OK);
  REQUIRE(exact.s.size() == 3000); REQUIRE(sect.s == exact.s);
  REQUIRE(exact.s[1500] == Approx((1500 * 2000 - 3000000) / 8388608.0f));
  // A ranged read from the middle, through a stage barely bigger than a sector.
  MemReader r(bytes.data(), (uint32_t)bytes.size());
  WavInfo w; REQUIRE(parseWav(r, w) == SE_OK);
  std::vector<float> mid(700); std::vector<uint8_t> stage(520);
  SourceErr err;
  REQUIRE(readWavFrames(r, w, 1234, 700, mid.data(), stage.data(), 520, 512, err) == 700);
  REQUIRE(err == SE_OK);
  for (uint32_t i = 0; i < 700; i++) REQUIRE(mid[i] == exact.s[1234 + i]);
}

TEST_CASE("readWavFrames: an I/O failure mid-read reports SE_IO with the frames it got") {
  struct FailingReader {
    MemReader m; uint32_t failAt;
    FailingReader(const uint8_t* p, uint32_t n, uint32_t at) : m(p, n), failAt(at) {}
    bool read(void* d, uint32_t n, uint32_t& got) { if (m.pos >= failAt) return false; return m.read(d, n, got); }
    bool seek(uint32_t to) { return m.seek(to); }
    uint32_t tell() const { return m.tell(); }
    uint32_t size() const { return m.size(); }
  };
  std::vector<int16_t> pcm(2000, 3);
  auto bytes = make_wav(WavBuild{}, pcm);
  FailingReader r(bytes.data(), (uint32_t)bytes.size(), 44 + 1024);   // dies after 512 frames' bytes
  WavInfo w; REQUIRE(parseWav(r, w) == SE_OK);
  std::vector<float> out(2000); std::vector<uint8_t> stage(512);
  SourceErr err;
  uint32_t n = readWavFrames(r, w, 0, 2000, out.data(), stage.data(), 512, 1, err);
  REQUIRE(err == SE_IO);
  REQUIRE(n == 512);
}
