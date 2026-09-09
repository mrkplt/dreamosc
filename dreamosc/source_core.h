// source_core.h - platform-free source-loading logic for the Pod firmware.
//
// The QSPI sample path in dreamosc.cpp used to validate the blob header,
// convert int16 -> float and wrap-pad the buffer inline, all of which is pure
// over a byte pointer and a float buffer -- only the QSPI base address and the
// cast to it are hardware. That decision logic lives here so it is
// host-compiled and tested (CLAUDE.md: as little as sensibly possible is left
// in the un-host-compilable edge). dreamosc.cpp calls decodeSampleBlob() with
// the memory-mapped QSPI pointer.
//
// Blob layout, little-endian (see tools/wav2raw.py):
//   uint32 magic 'DRMO' | uint32 count | uint32 rate | uint32 reserved
//   int16  samples[count]
//
// NOTE (surfaced, not fixed -- OPEN_ISSUES.md): `rate` is decoded and
// returned but the firmware does not apply it. wav2raw.py's docstring promises
// "the firmware reads the header and scales playback accordingly"; a blob at a
// rate other than the codec's plays pitch-shifted. Applying it is a sound
// change and a bench item, not a cleanup.

#ifndef SOURCE_CORE_H
#define SOURCE_CORE_H

#include <math.h>
#include <stdint.h>
#include <string.h>

#define SAMPLE_MAGIC 0x4F4D5244u   // 'DRMO'

struct SampleHeader {
  uint32_t magic;
  uint32_t count;
  uint32_t rate;
  uint32_t reserved;
};

// Is `h` a plausible sample blob header? count must be 1..maxCount (the QSPI
// part is 8 MB, so a count past that is garbage, not a sample).
inline bool sampleBlobValid(const SampleHeader* h, uint32_t maxCount = 8u * 1024 * 1024) {
  if (h->magic != SAMPLE_MAGIC) return false;
  if (h->count == 0 || h->count > maxCount) return false;
  return true;
}

// Decode a sample blob at `blob` into dst[0..cap): int16 PCM -> float in
// [-1, 1), at most `cap` samples, then WRAP-PAD the remainder of dst so the
// whole buffer is musical material rather than a block of silence the read
// heads can wander into. Returns the number of samples decoded from the blob
// (before padding), or 0 if the header is invalid (dst untouched). `rate` gets
// the blob's sample rate (unused by the caller today -- see the NOTE above).
// Byte-exact with the inline version this replaces: dst[i] = dst[i % n].
inline uint32_t decodeSampleBlob(const uint8_t* blob, float* dst, uint32_t cap,
                                 uint32_t& rate) {
  SampleHeader h;
  memcpy(&h, blob, sizeof(h));
  if (!sampleBlobValid(&h)) return 0;
  const int16_t* pcm = (const int16_t*)(blob + sizeof(SampleHeader));
  uint32_t n = h.count > cap ? cap : h.count;
  for (uint32_t i = 0; i < n; i++) dst[i] = pcm[i] / 32768.0f;
  uint32_t j = 0;                       // running i % n, no per-sample modulo
  for (uint32_t i = n; i < cap; i++) {
    dst[i] = dst[j];
    if (++j == n) j = 0;
  }
  rate = h.rate;
  return n;
}

// Fallback when no blob is present: a spectrally-varied synthetic source, so
// the position/stretch controls still audibly do something. Fills dst[0..n) at
// sample rate `sr`.
inline void fillStubSource(float* dst, uint32_t n, float sr) {
  for (uint32_t i = 0; i < n; i++) {
    float t = (float)i / sr;
    dst[i] = 0.5f * sinf(2.0f * (float)M_PI * 220.0f * t)
           + 0.3f * sinf(2.0f * (float)M_PI * 331.0f * t)
           + 0.2f * sinf(2.0f * (float)M_PI * 554.0f * t);
  }
}

// ---------------------------------------------------------------------------
// Which source the instrument booted from, for the PROFILE `SRC` line. The
// boot fingerprint (`crc=`) renders whatever the source holds, so a crc is
// comparable only between builds whose SRC lines match.
// ---------------------------------------------------------------------------

enum SourceKind : uint8_t { SRC_STUB = 0, SRC_QSPI = 1, SRC_SD = 2 };

// Why a loader fell through (SD stage, then QSPI stage); 0 = it succeeded.
enum SourceErr : uint8_t {
  SE_OK = 0,
  SE_NO_CARD,        // SDMMC init failed (no card, or not readable)
  SE_NO_FS,          // no FAT filesystem / mount failed
  SE_NO_WAV,         // no candidate *.wav in the root directory
  SE_OPEN,           // the chosen file would not open
  SE_NOT_RIFF,       // not a RIFF/WAVE file
  SE_NO_FMT,         // no fmt chunk before the data chunk
  SE_BAD_FORMAT,     // not 16-bit PCM (24-bit, float, EXTENSIBLE ...)
  SE_NO_DATA,        // no data chunk
  SE_IO,             // a read/seek failed mid-file
  SE_EMPTY,          // the file had no samples
  SE_NO_BLOB,        // (QSPI) no valid blob header
};

struct SourceInfo {
  SourceKind kind = SRC_STUB;
  uint32_t   rate = 0;         // the material's own rate; NOT applied (see NOTE)
  uint32_t   len  = 0;         // samples in the Source
  SourceErr  sdErr   = SE_OK;  // why the SD stage fell through, if it did
  SourceErr  qspiErr = SE_OK;  // why the QSPI stage fell through, if it did
  char       name[32] = {0};   // the SD file, or "" (13 chars if LFN is off)
};

// ---------------------------------------------------------------------------
// "The first WAV on the card": among the ROOT directory's entries, the
// candidate whose name sorts first case-insensitively (FAT directory order is
// write order, so alphabetical is the only stable meaning of "first").
// ---------------------------------------------------------------------------

inline char ssLower(char c) { return (c >= 'A' && c <= 'Z') ? (char)(c + 32) : c; }

// A directory entry we would load: a plain file, not a dotfile (macOS writes
// an AppleDouble "._name.wav" twin next to every file on a FAT card, and it
// is not audio), with a .wav extension in any case.
inline bool isCandidateWav(const char* name, bool isDir) {
  if (isDir || !name || name[0] == '\0' || name[0] == '.') return false;
  size_t n = strlen(name);
  if (n < 5) return false;
  const char* ext = name + n - 4;
  return ext[0] == '.' && ssLower(ext[1]) == 'w' && ssLower(ext[2]) == 'a' && ssLower(ext[3]) == 'v';
}

// Does `a` sort before `b` (case-insensitive; a prefix sorts first)?
inline bool wavNameBefore(const char* a, const char* b) {
  for (;; a++, b++) {
    char ca = ssLower(*a), cb = ssLower(*b);
    if (ca != cb) return (unsigned char)ca < (unsigned char)cb;
    if (ca == '\0') return false;
  }
}

// Fold one entry into the running best. Returns true if it became the best.
inline bool pickFirstWav(const char* name, bool isDir, char* best, size_t cap) {
  if (!isCandidateWav(name, isDir)) return false;
  if (best[0] != '\0' && !wavNameBefore(name, best)) return false;
  size_t n = strlen(name);
  if (n >= cap) n = cap - 1;
  memcpy(best, name, n);
  best[n] = '\0';
  return true;
}

// ---------------------------------------------------------------------------
// WAV: chunk-walking header parse and 16-bit PCM read with stereo->mono fold,
// over a minimal reader interface so the same code runs on a FatFs FIL (the
// device) and a memory buffer (the tests):
//   bool     read(void* dst, uint32_t n, uint32_t& got);   // false = I/O error
//   bool     seek(uint32_t pos);
//   uint32_t tell() const;
//   uint32_t size() const;
// Real files interleave LIST/fact/etc. before `data`, pad odd chunks, and
// (streaming writers) lie about the data length, so the walk skips unknown
// chunks with their pad byte and clamps the data length to what the file
// actually holds.
// ---------------------------------------------------------------------------

struct WavInfo {
  uint16_t format   = 0;       // 1 = PCM
  uint16_t channels = 0;
  uint32_t rate     = 0;
  uint16_t bits     = 0;
  uint32_t dataBytes = 0;      // clamped to the file
};

inline uint16_t wavU16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline uint32_t wavU32(const uint8_t* p) {
  return (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24));
}

// Parse the header, leaving the reader at the first sample of `data`.
template <class R>
inline SourceErr parseWav(R& r, WavInfo& w) {
  uint8_t hdr[12]; uint32_t got = 0;
  if (!r.read(hdr, 12, got)) return SE_IO;
  if (got != 12 || memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) return SE_NOT_RIFF;
  bool haveFmt = false;
  for (;;) {
    uint8_t ch[8];
    if (!r.read(ch, 8, got)) return SE_IO;
    if (got != 8) return haveFmt ? SE_NO_DATA : SE_NO_FMT;
    uint32_t id = wavU32(ch), len = wavU32(ch + 4);
    if (id == 0x20746d66u) {                                  // "fmt "
      uint8_t f[16]; uint32_t take = len < 16 ? len : 16;
      if (!r.read(f, take, got)) return SE_IO;
      if (got != take || take < 16) return SE_NO_FMT;
      w.format = wavU16(f); w.channels = wavU16(f + 2);
      w.rate = wavU32(f + 4); w.bits = wavU16(f + 14);
      haveFmt = true;
      uint32_t rest = (len - 16) + (len & 1);
      if (rest && !r.seek(r.tell() + rest)) return SE_IO;
    } else if (id == 0x61746164u) {                           // "data"
      if (!haveFmt) return SE_NO_FMT;
      if (w.format != 1 || w.bits != 16 || w.channels == 0) return SE_BAD_FORMAT;
      uint32_t avail = r.size() > r.tell() ? r.size() - r.tell() : 0;
      w.dataBytes = len < avail ? len : avail;                // streaming writers lie here
      return SE_OK;
    } else {
      if (!r.seek(r.tell() + len + (len & 1))) return SE_IO;
    }
  }
}

// Read up to `cap` mono samples after parseWav() into dst, folding channels
// by their mean (byte-exact with tools/wav2raw.py and the old sd_source.h).
// `block` is scratch of `blockBytes` (a multiple of the frame size is not
// required). Returns the sample count; 0 with err = SE_IO on a failed read.
template <class R>
inline uint32_t readWavMono(R& r, const WavInfo& w, float* dst, uint32_t cap,
                            uint8_t* block, uint32_t blockBytes, SourceErr& err) {
  const uint32_t ch = w.channels ? w.channels : 1;
  const uint32_t frameBytes = 2u * ch;
  const uint32_t total = w.dataBytes / frameBytes;
  const uint32_t want = total < cap ? total : cap;
  const uint32_t perBlock = blockBytes / frameBytes;
  uint32_t written = 0;
  err = SE_OK;
  while (written < want && perBlock > 0) {
    uint32_t frames = want - written;
    if (frames > perBlock) frames = perBlock;
    uint32_t got = 0;
    if (!r.read(block, frames * frameBytes, got)) { err = SE_IO; return 0; }
    uint32_t gotFrames = got / frameBytes;
    for (uint32_t f = 0; f < gotFrames; f++) {
      const uint8_t* p = block + f * frameBytes;
      if (ch == 1) {
        dst[written] = (int16_t)wavU16(p) / 32768.0f;
      } else {
        int32_t acc = 0;
        for (uint32_t c = 0; c < ch; c++) acc += (int16_t)wavU16(p + 2 * c);
        dst[written] = (float)acc / (ch * 32768.0f);
      }
      written++;
    }
    if (gotFrames < frames) break;                            // short read = EOF
  }
  if (written == 0) err = SE_EMPTY;
  return written;
}

// A reader over a memory buffer (tests; also any memory-mapped WAV).
struct MemReader {
  const uint8_t* p; uint32_t n; uint32_t pos = 0;
  MemReader(const uint8_t* data, uint32_t size) : p(data), n(size) {}
  bool read(void* dst, uint32_t want, uint32_t& got) {
    got = pos + want <= n ? want : (n > pos ? n - pos : 0);
    memcpy(dst, p + pos, got); pos += got; return true;
  }
  bool seek(uint32_t to) { if (to > n) return false; pos = to; return true; }
  uint32_t tell() const { return pos; }
  uint32_t size() const { return n; }
};

#endif  // SOURCE_CORE_H
