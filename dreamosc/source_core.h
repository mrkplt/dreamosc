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
  SE_BAD_FORMAT,     // not PCM 8/16/24/32 or float 32 (compressed, ADPCM, 64-bit float ...)
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
// WAV: chunk-walking header parse and PCM decode over a minimal reader
// interface, so the same code runs on a FatFs FIL (the device) and a memory
// buffer (the tests):
//   bool     read(void* dst, uint32_t n, uint32_t& got);   // false = I/O error
//   bool     seek(uint32_t pos);
//   uint32_t tell() const;
//   uint32_t size() const;
// Real files interleave LIST/fact/etc. before `data`, pad odd chunks, and
// (streaming writers) lie about the data length, so the walk skips unknown
// chunks with their pad byte and clamps the data length to what the file
// actually holds.
//
// Formats: PCM 8 (unsigned), 16, 24 (packed), 32-bit; IEEE float 32; either
// plain or wrapped in WAVE_FORMAT_EXTENSIBLE. Channel 0 (left) only, by
// design for now -- the instrument is mono and a fold was a choice nobody
// asked for. Scaling is by the CONTAINER width (a 20-in-24 file scales by
// 2^23 like any 24-bit file), so full scale is always +-1.
// ---------------------------------------------------------------------------

enum WavCodec : uint8_t { WAV_U8 = 0, WAV_S16, WAV_S24, WAV_S32, WAV_F32 };

struct WavInfo {
  uint16_t format     = 0;     // 1 = PCM, 3 = IEEE float (EXTENSIBLE resolved to these)
  uint16_t channels   = 0;
  uint32_t rate       = 0;
  uint16_t bits       = 0;     // container bits per sample
  uint16_t blockAlign = 0;     // bytes per frame (all channels)
  uint32_t dataOffset = 0;     // absolute byte offset of the first frame
  uint32_t dataBytes  = 0;     // clamped to the file
  uint32_t frames     = 0;     // dataBytes / blockAlign
  WavCodec codec      = WAV_S16;
};

inline uint16_t wavU16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline uint32_t wavU32(const uint8_t* p) {
  return (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24));
}

// The decoder for a (format, bits) pair, or false if unsupported.
inline bool wavCodecFor(uint16_t format, uint16_t bits, WavCodec& c) {
  if (format == 1) {
    switch (bits) {
      case 8:  c = WAV_U8;  return true;
      case 16: c = WAV_S16; return true;
      case 24: c = WAV_S24; return true;
      case 32: c = WAV_S32; return true;
      default: return false;
    }
  }
  if (format == 3 && bits == 32) { c = WAV_F32; return true; }
  return false;
}

// Parse the header. On success the reader is positioned at the first frame
// and w.dataOffset records where that is.
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
      uint8_t f[40]; uint32_t take = len < sizeof(f) ? len : (uint32_t)sizeof(f);
      if (!r.read(f, take, got)) return SE_IO;
      if (got != take || take < 16) return SE_NO_FMT;
      w.format = wavU16(f); w.channels = wavU16(f + 2);
      w.rate = wavU32(f + 4); w.blockAlign = wavU16(f + 12); w.bits = wavU16(f + 14);
      if (w.format == 0xFFFE) {                               // WAVE_FORMAT_EXTENSIBLE
        // cbSize at 16, validBits at 18, channelMask at 20, SubFormat GUID
        // at 24: its first two bytes are the wrapped format tag.
        if (take < 26) return SE_BAD_FORMAT;
        w.format = wavU16(f + 24);
      }
      haveFmt = true;
      uint32_t rest = (len - take) + (len & 1);
      if (rest && !r.seek(r.tell() + rest)) return SE_IO;
    } else if (id == 0x61746164u) {                           // "data"
      if (!haveFmt) return SE_NO_FMT;
      if (w.channels == 0 || !wavCodecFor(w.format, w.bits, w.codec)) return SE_BAD_FORMAT;
      uint16_t minAlign = (uint16_t)(w.channels * (w.bits / 8));
      if (w.blockAlign < minAlign) w.blockAlign = minAlign;    // a broken writer; recover
      w.dataOffset = r.tell();
      uint32_t avail = r.size() > r.tell() ? r.size() - r.tell() : 0;
      w.dataBytes = len < avail ? len : avail;                // streaming writers lie here
      w.frames = w.dataBytes / w.blockAlign;
      return SE_OK;
    } else {
      if (!r.seek(r.tell() + len + (len & 1))) return SE_IO;
    }
  }
}

// Decode one sample at p.
inline float wavDecodeSample(const uint8_t* p, WavCodec c) {
  switch (c) {
    case WAV_U8:  return ((int)p[0] - 128) / 128.0f;
    case WAV_S16: return (int16_t)wavU16(p) / 32768.0f;
    case WAV_S24: {
      int32_t v = (int32_t)((uint32_t)p[0] << 8 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 24);
      return (float)(v >> 8) / 8388608.0f;                    // sign-extend via the shift
    }
    case WAV_S32: return (float)((int32_t)wavU32(p)) / 2147483648.0f;
    case WAV_F32: { float v; uint32_t u = wavU32(p); memcpy(&v, &u, 4); return v; }
  }
  return 0.0f;
}

// Decode `frames` consecutive frames at `bytes` into dst: channel 0 only.
inline void wavDecodeFrames(const uint8_t* bytes, uint32_t frames, const WavInfo& w, float* dst) {
  for (uint32_t f = 0; f < frames; f++)
    dst[f] = wavDecodeSample(bytes + (size_t)f * w.blockAlign, w.codec);
}

// Read frames [first, first + n) of a parsed file into dst, through a scratch
// `stage` of `stageBytes`. Reads are issued at file offsets rounded DOWN to
// `align` (a sector size on the device: FatFs then moves whole sectors
// straight into `stage` by DMA instead of through the FIL's private window;
// 1 = byte-exact, for a memory reader). Frames that straddle the end of one
// staged read are re-read at the head of the next, so any blockAlign works
// with any stage size >= align + blockAlign. Returns the frames decoded; a
// short count with err = SE_IO is a failed read, with err = SE_OK it is the
// end of the data. `first + n` must not exceed w.frames.
template <class R>
inline uint32_t readWavFrames(R& r, const WavInfo& w, uint32_t first, uint32_t n, float* dst,
                              uint8_t* stage, uint32_t stageBytes, uint32_t align, SourceErr& err) {
  err = SE_OK;
  if (align == 0) align = 1;
  const uint32_t ba = w.blockAlign;
  uint32_t done = 0;
  while (done < n) {
    uint32_t byte = w.dataOffset + (first + done) * ba;
    uint32_t at = byte - byte % align;
    uint32_t want = stageBytes;
    uint32_t end = w.dataOffset + w.dataBytes;
    if (at + want > end) want = end - at;
    if (!r.seek(at)) { err = SE_IO; return done; }
    uint32_t got = 0;
    if (!r.read(stage, want, got)) { err = SE_IO; return done; }
    uint32_t availEnd = at + got;
    uint32_t off = byte - at;
    uint32_t fit = availEnd > byte ? (availEnd - byte) / ba : 0;
    if (fit == 0) return done;                                // short read: end of data
    uint32_t take = n - done < fit ? n - done : fit;
    wavDecodeFrames(stage + off, take, w, dst + done);
    done += take;
  }
  return done;
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
