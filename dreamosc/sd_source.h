// sd_source.h - load a WAV file off the Pod's microSD into an SDRAM buffer and
// hand it to the DSP core as a Source.
//
// This is the one place that knows where source audio comes from. stretch_core.h
// stays source-agnostic (Source is just a float* + length); everything upstream
// of load_source() is finished and only the body here changes if the origin ever
// does. The signature is the seam:
//
//     bool load_source(Source& src, float* dst, uint32_t dst_capacity,
//                      const char* path, uint32_t& out_samplerate);
//
// Reads 16-bit PCM WAV (mono or stereo), folds stereo to mono to match the
// single-channel Source the host harness was verified against, and normalizes
// nothing (the sequencer's power-sum compensation handles level). Returns false
// on any failure so the caller can fall back to a stub/test tone.

#ifndef SD_SOURCE_H
#define SD_SOURCE_H

#include <cstdint>
#include <cstring>

// daisy_seed.h pulls in SdmmcHandler, FatFSInterface, and FatFs (ff.h).
#include "daisy_seed.h"
#include "stretch_core.h"

namespace stretchsd {

// Minimal canonical-WAV header fields we care about. We parse chunks rather than
// assuming a fixed 44-byte header, because real files interleave LIST/fact/etc.
struct WavFmt {
  uint16_t audio_format   = 1;   // 1 = PCM
  uint16_t num_channels   = 1;
  uint32_t sample_rate    = 48000;
  uint16_t bits_per_sample = 16;
};

// Read a little-endian u16/u32 from a 4-byte-aligned-agnostic byte pointer.
static inline uint16_t rd_u16(const uint8_t* p) {
  return (uint16_t)(p[0] | (p[1] << 8));
}
static inline uint32_t rd_u32(const uint8_t* p) {
  return (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24));
}

// Parse the WAV header from an open FatFs file, leaving the read cursor at the
// first sample of the data chunk. Fills fmt and data_bytes. Returns false if the
// file is not a 16-bit PCM WAV we can read.
static bool parse_wav_header(FIL* fp, WavFmt& fmt, uint32_t& data_bytes) {
  uint8_t hdr[12];
  UINT    br = 0;
  if (f_read(fp, hdr, 12, &br) != FR_OK || br != 12) return false;
  if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) return false;

  bool have_fmt = false;
  for (;;) {
    uint8_t ch[8];
    if (f_read(fp, ch, 8, &br) != FR_OK || br != 8) return false;  // ran out
    uint32_t id  = rd_u32(ch);
    uint32_t len = rd_u32(ch + 4);

    if (id == 0x20746d66u) {  // "fmt "
      uint8_t f[16];
      uint32_t take = len < 16 ? len : 16;
      if (f_read(fp, f, take, &br) != FR_OK || br != take) return false;
      fmt.audio_format    = rd_u16(f + 0);
      fmt.num_channels    = rd_u16(f + 2);
      fmt.sample_rate     = rd_u32(f + 4);
      fmt.bits_per_sample = rd_u16(f + 14);
      have_fmt = true;
      // Skip any bytes of the fmt chunk beyond the 16 we consumed, plus pad.
      uint32_t rest = (len > 16 ? len - 16 : 0) + (len & 1);
      if (rest) f_lseek(fp, f_tell(fp) + rest);
    } else if (id == 0x61746164u) {  // "data"
      if (!have_fmt) return false;
      if (fmt.audio_format != 1 || fmt.bits_per_sample != 16) return false;
      data_bytes = len;
      return true;  // cursor is now at the first sample
    } else {
      // Skip an unknown chunk (LIST, fact, etc.), honoring the pad byte.
      if (f_lseek(fp, f_tell(fp) + len + (len & 1)) != FR_OK) return false;
    }
  }
}

// Fill dst[0..out_count) with mono float samples from the WAV. Reads in blocks
// so we never need a second big buffer; stereo is folded to mono on the fly.
// dst_capacity is in samples (floats), sized for SOURCE_SECONDS * sample_rate.
static bool read_wav_mono(FIL* fp, const WavFmt& fmt, uint32_t data_bytes,
                          float* dst, uint32_t dst_capacity, uint32_t& out_count) {
  const uint16_t ch          = fmt.num_channels ? fmt.num_channels : 1;
  const uint32_t frame_bytes = 2u * ch;                 // 16-bit per channel
  const uint32_t total_frames = data_bytes / frame_bytes;
  const uint32_t want         = total_frames < dst_capacity ? total_frames
                                                            : dst_capacity;

  static uint8_t block[4096];
  const uint32_t frames_per_block = sizeof(block) / frame_bytes;

  uint32_t written = 0;
  while (written < want) {
    uint32_t frames_now = want - written;
    if (frames_now > frames_per_block) frames_now = frames_per_block;
    uint32_t bytes_now = frames_now * frame_bytes;

    UINT br = 0;
    if (f_read(fp, block, bytes_now, &br) != FR_OK) return false;
    if (br == 0) break;
    uint32_t got_frames = br / frame_bytes;

    for (uint32_t f = 0; f < got_frames; f++) {
      const uint8_t* p = block + f * frame_bytes;
      if (ch == 1) {
        dst[written] = (int16_t)rd_u16(p) / 32768.0f;
      } else {
        int32_t acc = 0;
        for (uint16_t c = 0; c < ch; c++) acc += (int16_t)rd_u16(p + 2 * c);
        dst[written] = (float)acc / (ch * 32768.0f);
      }
      written++;
    }
    if (got_frames < frames_now) break;  // short read = EOF
  }
  out_count = written;
  return written > 0;
}

// The seam. Mounts the SD card, opens `path`, loads it into dst (an SDRAM
// buffer of dst_capacity floats), and points src at it. On any failure returns
// false and leaves src untouched so the caller can substitute a stub.
//
// hw is the already-initialized DaisySeed (we need it for the SDMMC pins /
// clock config libDaisy sets up).
inline bool load_source(daisy::DaisySeed& hw, Source& src, float* dst,
                        uint32_t dst_capacity, const char* path,
                        uint32_t& out_samplerate) {
  using namespace daisy;

  // Bring up the SD peripheral in 4-bit, standard-speed mode. FAST is available
  // but standard is the safe default for the first bring-up.
  static SdmmcHandler  sdmmc;
  SdmmcHandler::Config sd_cfg;
  sd_cfg.Defaults();
  sd_cfg.speed = SdmmcHandler::Speed::STANDARD;
  sd_cfg.width = SdmmcHandler::BusWidth::BITS_4;
  if (sdmmc.Init(sd_cfg) != SdmmcHandler::Result::OK) return false;

  // Bind FatFs to the SD media and mount at "/", as the WavParser example does.
  static FatFSInterface fsi;
  if (fsi.Init(FatFSInterface::Config::Media::MEDIA_SD) != FatFSInterface::Result::OK)
    return false;
  if (f_mount(&fsi.GetSDFileSystem(), "/", 1) != FR_OK) return false;

  // Build a full path against the "/" mount (e.g. "/source.wav").
  char full[128];
  {
    size_t n = 0;
    full[n++] = '/';
    for (const char* q = path; *q && n < sizeof(full) - 1; ++q) full[n++] = *q;
    full[n] = '\0';
  }

  FIL fil;
  if (f_open(&fil, full, FA_OPEN_EXISTING | FA_READ) != FR_OK) {
    f_mount(nullptr, "/", 0);
    return false;
  }

  WavFmt   fmt;
  uint32_t data_bytes = 0;
  bool     ok = parse_wav_header(&fil, fmt, data_bytes);
  uint32_t count = 0;
  if (ok) ok = read_wav_mono(&fil, fmt, data_bytes, dst, dst_capacity, count);

  f_close(&fil);
  f_mount(nullptr, "/", 0);

  if (!ok || count == 0) return false;

  src.data        = dst;
  src.len         = count;
  out_samplerate  = fmt.sample_rate;
  return true;
}

}  // namespace stretchsd

#endif  // SD_SOURCE_H
