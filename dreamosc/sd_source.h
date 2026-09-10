// sd_source.h - a WAV on the Pod's microSD, streamed to the DSP core as a
// Source (Fizzy #131).
//
// This is the hardware edge of the source path and nothing more: SDMMC
// bring-up, the FatFs mount, a root-directory walk, an open file, and a
// reader adapter over it. Every decision -- which entry counts as the first
// WAV, the chunk walk, the format check, the sector-aligned ranged read and
// the decode -- is source_core.h, host-tested. `make sd-check` compiles this
// header against the real libDaisy/FatFs API with the device toolchain.
//
// Streaming, not loading: SdSource::read() is called by a head's WindowCache
// (stretch_core.h) from the main loop, for the run of samples that head is
// about to render, and reads exactly those frames off the card (sector-
// aligned, whole sectors by DMA into the staging buffer). There is no
// length limit; the whole cost of a read is on the main loop, measured by
// the sequencer as fetch samples (PROFILE FETCH line) and kept out of the
// render cost model.
//
// MEMORY. FatFs reads sectors by SDMMC IDMA into FIL::buf, FATFS::win and
// the caller's buffer. Under APP_TYPE=BOOT_SRAM, .bss is DTCM, which no DMA
// master can reach (libDaisy #508): a FIL on the stack or a static staging
// buffer here would fail every read on hardware. So every DMA target lives
// in AXI SRAM (axisram.h), NOLOAD: no constructor, not zeroed. Each is
// memset / placement-new'd in mount()/open() before any use, and never
// touched by the ISR.
//
// FAILURE. No fallback source, by design: an unreadable card, no WAV, an
// unsupported format or an unreadable file is "no instrument" (the caller
// halts with both LEDs red and the PROFILE SRC line saying why). The one
// retry is the card CLOCK: FAST (50 MHz) first, STANDARD (25 MHz) if the
// card would not initialise at FAST -- same card, same file.

#ifndef SD_SOURCE_H
#define SD_SOURCE_H

#include <new>
#include <stdint.h>
#include <string.h>

// daisy_seed.h pulls in SdmmcHandler, FatFSInterface and FatFs (ff.h).
#include "daisy_seed.h"
#include "axisram.h"
#include "stretch_core.h"
#include "source_core.h"

namespace stretchsd {

// Staging: whole sectors land here by DMA; the decoder walks it. 32 KB is
// 64 sectors per f_read (one SDMMC transfer at 4-bit/50 MHz ~ 0.8 ms) and
// holds 1365 6-byte frames, so a 16384-window fetch at 24-bit stereo is a
// dozen reads. Multiple of 32 (D-cache line) and 32-aligned.
static constexpr uint32_t SD_STAGE_BYTES = 32768;
static_assert(SD_STAGE_BYTES % 32 == 0, "stage must be whole cache lines");
// FatFs cluster link map (fast seek): N entries hold (N-1)/2 fragments. A
// file written to a fresh card is 1-3 fragments; 511 is generous. A file
// too fragmented for the table is refused (SE_FRAGMENTED) rather than seeked
// the slow way: a seek in every fetch would be a FAT walk per read.
static constexpr uint32_t SD_CLMT_ENTRIES = 1024;

// DMA targets -> AXI SRAM (see MEMORY above). Plain storage only; FIL and
// FATFS are C structs, FatFSInterface is placement-new'd into raw bytes.
static AXISRAM_DATA alignas(32) uint8_t gFsiStore[sizeof(daisy::FatFSInterface)];
static AXISRAM_DATA alignas(32) FIL     gFil;
static AXISRAM_DATA alignas(32) uint8_t gStage[SD_STAGE_BYTES];
static AXISRAM_DATA alignas(32) DWORD   gClmt[SD_CLMT_ENTRIES];

// The reader interface (source_core.h) over the open FatFs file.
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

class SdSource : public Source {
 public:
  // Bring the card up and mount it. Idempotent. False with info.err set.
  bool mount(SourceInfo& info) {
    using namespace daisy;
    if (mounted_) return true;
    SdmmcHandler::Config cfg;
    cfg.Defaults();
    cfg.width = SdmmcHandler::BusWidth::BITS_4;
    cfg.speed = SdmmcHandler::Speed::FAST;
    speed_ = SRC_SPEED_FAST;
    if (sdmmc_.Init(cfg) != SdmmcHandler::Result::OK) {
      cfg.speed = SdmmcHandler::Speed::STANDARD;    // the one retry: clock only
      speed_ = SRC_SPEED_STANDARD;
      if (sdmmc_.Init(cfg) != SdmmcHandler::Result::OK) { info.err = SE_NO_CARD; return false; }
    }
    info.speed = speed_;
    // FatFSInterface holds the FATFS (its sector window is a DMA target):
    // constructed in place in AXI SRAM, once.
    fsi_ = new (gFsiStore) FatFSInterface();
    if (fsi_->Init(FatFSInterface::Config::Media::MEDIA_SD) != FatFSInterface::Result::OK
        || f_mount(&fsi_->GetSDFileSystem(), "/", 1) != FR_OK) {
      info.err = SE_NO_FS; return false;
    }
    mounted_ = true;
    return true;
  }

  // List[0]: the first entry of the root directory, in the order the
  // directory lists it, that is a plain file with a .wav extension and not a
  // dotfile (isCandidateWav). No sorting. Returns false with info.err set.
  bool firstWav(char* name, size_t cap, SourceInfo& info) {
    if (!mount(info)) return false;
    DIR dir;                                        // CPU-only (reads FATFS::win)
    if (f_opendir(&dir, "/") != FR_OK) { info.err = SE_NO_FS; return false; }
    static FILINFO fno;                             // 256-byte LFN; CPU-only
    bool found = false;
    for (;;) {
      if (f_readdir(&dir, &fno) != FR_OK || fno.fname[0] == '\0') break;
      if (isCandidateWav(fno.fname, (fno.fattrib & AM_DIR) != 0)) {
        size_t n = strlen(fno.fname);
        if (n >= cap) n = cap - 1;
        memcpy(name, fno.fname, n); name[n] = '\0';
        found = true;
        break;
      }
    }
    f_closedir(&dir);
    if (!found) info.err = SE_NO_WAV;
    return found;
  }

  // Open `name` (root directory) as the source: parse, build the cluster
  // map, set len/rate. Closes any file open before. False with info.err set;
  // the source is then closed (len 0) -- there is no previous file to fall
  // back to and no caller should read it.
  bool open(const char* name, SourceInfo& info) {
    info = SourceInfo();
    info.speed = speed_;
    if (!mount(info)) return false;
    close();
    size_t n = strlen(name);
    if (n >= sizeof(info.name)) n = sizeof(info.name) - 1;
    memcpy(info.name, name, n); info.name[n] = '\0';
    char path[_MAX_LFN + 2];
    path[0] = '/';
    size_t full = strlen(name);
    if (full > _MAX_LFN) full = _MAX_LFN;
    memcpy(path + 1, name, full); path[1 + full] = '\0';

    // NOLOAD FIL: f_open fills every field it reads, but the table pointer
    // (cltbl) it only clears on success and the object may hold anything
    // from the last life; zero is the one safe start.
    memset(&gFil, 0, sizeof(gFil));
    if (f_open(&gFil, path, FA_OPEN_EXISTING | FA_READ) != FR_OK) { info.err = SE_OPEN; return false; }
    open_ = true;
    FilReader rd(&gFil);
    SourceErr err = parseWav(rd, wav_);
    if (err != SE_OK) { info.err = err; close(); return false; }
    if (wav_.frames == 0) { info.err = SE_EMPTY; close(); return false; }
    // Cluster link map so every fetch's seek is a table lookup, not a FAT
    // walk. Entry 0 holds the table size; FR_NOT_ENOUGH_CORE = too fragmented.
    memset(gClmt, 0, sizeof(gClmt));
    gClmt[0] = SD_CLMT_ENTRIES;
    gFil.cltbl = gClmt;
    if (f_lseek(&gFil, CREATE_LINKMAP) != FR_OK) { info.err = SE_FRAGMENTED; close(); return false; }

    len  = wav_.frames;
    rate = wav_.rate;
    info.rate = wav_.rate; info.len = wav_.frames;
    info.format = wav_.format; info.channels = wav_.channels; info.bits = wav_.bits;
    info.err = SE_OK;
    return true;
  }

  // The first WAV on the card (List[0]), opened.
  bool openFirst(SourceInfo& info) {
    char name[_MAX_LFN + 1];
    if (!firstWav(name, sizeof(name), info)) return false;
    return open(name, info);
  }

  void close() {
    if (open_) { f_close(&gFil); open_ = false; }
    len = 0; rate = 0;
  }

  // Source: frames [first, first + n) -> dst, channel 0, decoded. Main loop
  // only. False if the card did not deliver every frame (the cache
  // zero-fills and counts it).
  bool read(uint32_t first, uint32_t n, float* dst) override {
    if (!open_) return false;
    FilReader rd(&gFil);
    SourceErr err;
    uint32_t got = readWavFrames(rd, wav_, first, n, dst, gStage, SD_STAGE_BYTES, 512, err);
    reads_++;
    bytes_ += (uint64_t)n * wav_.blockAlign;
    return got == n && err == SE_OK;
  }

  const WavInfo& wav() const { return wav_; }
  bool isOpen() const { return open_; }
  uint32_t reads() const { return reads_; }         // read() calls, ever
  uint64_t bytes() const { return bytes_; }         // frame bytes delivered, ever

 private:
  daisy::SdmmcHandler    sdmmc_;                    // a handle, not a DMA target
  daisy::FatFSInterface* fsi_ = nullptr;            // lives in gFsiStore (AXI)
  WavInfo     wav_;
  SourceSpeed speed_ = SRC_SPEED_FAST;
  bool        mounted_ = false, open_ = false;
  uint32_t    reads_ = 0;
  uint64_t    bytes_ = 0;
};

}  // namespace stretchsd

#endif  // SD_SOURCE_H
