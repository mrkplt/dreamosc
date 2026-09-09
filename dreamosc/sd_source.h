// sd_source.h - load the first WAV off the Pod's microSD into an SDRAM buffer
// and hand it to the DSP core as a Source (Fizzy #131).
//
// This is the hardware edge of the source path and nothing more: SDMMC bring-
// up, FatFs mount, a root-directory walk, and a FIL adapter for the reader
// interface. Every decision -- which entry counts as "the first WAV", the
// chunk walk, the format check, the stereo fold -- is in source_core.h and
// host-tested (CLAUDE.md: as little as sensibly possible in the un-host-
// compilable edge). `make sd-check` compiles this header against the real
// libDaisy/FatFs API with the device toolchain.
//
// Returns false on any failure and says why in `info` (the PROFILE SRC line),
// leaving `src` untouched so the caller falls back (QSPI blob, then stub).

#ifndef SD_SOURCE_H
#define SD_SOURCE_H

#include <cstdint>
#include <cstring>

// daisy_seed.h pulls in SdmmcHandler, FatFSInterface and FatFs (ff.h).
#include "daisy_seed.h"
#include "stretch_core.h"
#include "source_core.h"

namespace stretchsd {

// The reader interface (source_core.h) over an open FatFs file.
struct FilReader {
  FIL* f;
  explicit FilReader(FIL* fil) : f(fil) {}
  bool read(void* dst, uint32_t n, uint32_t& got) {
    UINT br = 0;
    FRESULT r = f_read(f, dst, n, &br);
    got = br;
    return r == FR_OK;
  }
  bool seek(uint32_t pos) { return f_lseek(f, pos) == FR_OK; }
  uint32_t tell() const { return (uint32_t)f_tell(f); }
  uint32_t size() const { return (uint32_t)f_size(f); }
};

// Walk the root directory once and keep the candidate that sorts first
// (pickFirstWav). Returns false if there is none.
inline bool findFirstWav(char* best, size_t cap) {
  best[0] = '\0';
  DIR dir;
  if (f_opendir(&dir, "/") != FR_OK) return false;
  FILINFO fno;
  for (;;) {
    if (f_readdir(&dir, &fno) != FR_OK || fno.fname[0] == '\0') break;
    pickFirstWav(fno.fname, (fno.fattrib & AM_DIR) != 0, best, cap);
  }
  f_closedir(&dir);
  return best[0] != '\0';
}

// The seam. Mounts the card, loads the first WAV into dst (an SDRAM buffer of
// `cap` floats, so a file longer than that is truncated) and points src at
// it: src.len is the MATERIAL length, so position 0..1 spans the file.
inline bool load_source(Source& src, float* dst, uint32_t cap, SourceInfo& info) {
  using namespace daisy;
  info.sdErr = SE_OK;
  info.name[0] = '\0';

  // 4-bit, standard speed (25 MHz): the safe first-bring-up setting.
  static SdmmcHandler  sdmmc;
  SdmmcHandler::Config sd_cfg;
  sd_cfg.Defaults();
  sd_cfg.speed = SdmmcHandler::Speed::STANDARD;
  sd_cfg.width = SdmmcHandler::BusWidth::BITS_4;
  if (sdmmc.Init(sd_cfg) != SdmmcHandler::Result::OK) { info.sdErr = SE_NO_CARD; return false; }

  static FatFSInterface fsi;
  if (fsi.Init(FatFSInterface::Config::Media::MEDIA_SD) != FatFSInterface::Result::OK
      || f_mount(&fsi.GetSDFileSystem(), "/", 1) != FR_OK) {
    info.sdErr = SE_NO_FS; return false;
  }

  char name[sizeof(info.name)];
  if (!findFirstWav(name, sizeof(name))) { f_mount(nullptr, "/", 0); info.sdErr = SE_NO_WAV; return false; }

  char full[sizeof(name) + 1];
  full[0] = '/';
  memcpy(full + 1, name, strlen(name) + 1);

  FIL fil;
  if (f_open(&fil, full, FA_OPEN_EXISTING | FA_READ) != FR_OK) {
    f_mount(nullptr, "/", 0); info.sdErr = SE_OPEN; return false;
  }

  FilReader rd(&fil);
  WavInfo   w;
  SourceErr err = parseWav(rd, w);
  uint32_t  count = 0;
  if (err == SE_OK) {
    static uint8_t block[4096];
    uint32_t want = w.frames < cap ? w.frames : cap;
    count = readWavFrames(rd, w, 0, want, dst, block, sizeof(block), 512, err);
  }
  f_close(&fil);
  f_mount(nullptr, "/", 0);

  memcpy(info.name, name, strlen(name) + 1);
  info.rate = w.rate;
  if (err != SE_OK || count == 0) { info.sdErr = err == SE_OK ? SE_EMPTY : err; return false; }

  src.data  = dst;
  src.len   = count;
  info.kind = SRC_SD;
  info.len  = count;
  return true;
}

}  // namespace stretchsd

#endif  // SD_SOURCE_H
