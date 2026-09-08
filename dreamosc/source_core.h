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

#endif  // SOURCE_CORE_H
