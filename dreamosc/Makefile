# Project Name
TARGET = dreamosc

# Sources
CPP_SOURCES = dreamosc.cpp

# Library Locations (sibling clones under ~/src/dreamosc)
LIBDAISY_DIR = ../libDaisy
DAISYSP_DIR  = ../DaisySP

# Built for the Daisy bootloader, which we need because the STM32 ROM bootloader
# exposes no QSPI DFU target and sample audio has to live in QSPI (see CLAUDE.md).
# BOOT_SRAM runs from internal SRAM: comparable speed to internal flash, 480 KB
# limit (we use ~90 KB). Do NOT use BOOT_QSPI -- executing from QSPI is
# "more cache-dependent" i.e. slower, a real risk for the audio callback.
APP_TYPE = BOOT_SRAM

# --- Vendored third-party sources -------------------------------------------
# Pinned to exact upstream commits in ../vendor/manifest.txt and fetched by
# ../vendor/fetch.sh. Vendored files are NEVER edited in place: `make
# vendor-check` re-fetches from the pinned SHA and fails if what is on disk
# differs, so a local edit (or a stale/missing fetch) is caught rather than
# silently carried. To take a newer upstream, bump the SHA in the manifest.
VENDOR_DIR   = ../vendor
VENDOR_STAMP = vendor_stmlib/shy_fft.h

# libDaisy's Makefile defines the real default goal (all); declare it here too
# so these vendor rules -- which appear before that include -- do not steal it.
.DEFAULT_GOAL := all

# `make PROFILE=1`: diagnostic build that prints per-second CPU accounting
# (active voices, service us, per-unit us, ISR us, underruns) over USB serial.
# See the PROFILE blocks in dreamosc.cpp. Run `make clean` when toggling it;
# command-line defines are not tracked by the dependency files.
ifdef PROFILE
C_DEFS += -DPROFILE
endif

# Linker script: OURS, not libDaisy's. dreamosc.lds began as a copy of
# libDaisy/core/STM32H750IB_sram.lds (a starter template -- linker scripts are
# project-owned on STM32) and adds an .axisram_bss section. Set LDSCRIPT before
# the include below: libDaisy's core Makefile uses `LDSCRIPT ?=`, so this wins.
# (If APP_TYPE ever changes off BOOT_SRAM, re-base this from the matching stock
# .lds.)
LDSCRIPT = dreamosc.lds

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

.PHONY: vendor vendor-check deps deps-check
vendor:
	$(VENDOR_DIR)/fetch.sh

vendor-check:
	$(VENDOR_DIR)/fetch.sh --check

# Pinned build dependencies (whole repos: libDaisy, DaisySP). deps/manifest.txt
# records the exact SHA; `make deps` clones/checks-them-out, `make deps-check`
# gates that the working clones haven't drifted (used by test/run.sh).
DEPS_DIR = ../deps
deps:
	$(DEPS_DIR)/setup.sh

deps-check:
	$(DEPS_DIR)/setup.sh --check

# Fetch vendored sources automatically if missing, so a fresh clone builds
# without a separate manual step.
$(VENDOR_STAMP):
	$(VENDOR_DIR)/fetch.sh

build/$(TARGET).o: $(VENDOR_STAMP)

# --- Sample upload to QSPI ---------------------------------------------------
# TEMPORARY scaffolding for #132 control testing: real broadband material has to
# live somewhere, and internal flash (128 KB) is far too small. QSPI is 8 MB and
# memory-mapped at 0x90000000, so the firmware reads it in place. Reverts to the
# SD-card path (#131) once controls are sorted.
#
#   make sample SAMPLE_SRC=/path/to.wav   convert a WAV to the blob
#   make program-all                      upload sample + firmware in one go
#   make program-sample                   upload just the blob to QSPI
#   make program-dfu                      upload just the firmware
#
# NOTE: QSPI DFU requires Electrosmith's Daisy bootloader (the STM32 ROM
# bootloader exposes only Internal Flash + Option Bytes). Install it once with
# `make program-boot`, which libDaisy's core Makefile provides.
SAMPLE_BIN ?= ../sample/amen.bin
SAMPLE_SRC ?=

# 0x90400000 is the third QSPI region per the bootloader's DFU descriptor
# (0x90000000 reserved, 0x90040000 firmware images, 0x90400000 free).
# Must match QSPI_BASE in dreamosc.cpp.
SAMPLE_ADDR = 0x90400000

.PHONY: sample program-sample program-all
sample:
	@test -n "$(SAMPLE_SRC)" || { echo "usage: make sample SAMPLE_SRC=/path/to.wav"; exit 2; }
	../tools/wav2raw.py "$(SAMPLE_SRC)" "$(SAMPLE_BIN)"

program-sample:
	@test -f "$(SAMPLE_BIN)" || { echo "no $(SAMPLE_BIN) -- run: make sample SAMPLE_SRC=..."; exit 2; }
	dfu-util -a 0 -s $(SAMPLE_ADDR):leave -D "$(SAMPLE_BIN)" -d ,0483:df11

# Both images in one pass. The sample is written WITHOUT ':leave' so the board
# stays in DFU for the firmware upload that follows -- ':leave' would boot it out
# and the second dfu-util call would find no device. Only the firmware (last)
# leaves DFU and runs.
#
# dfu-util exits non-zero on the harmless "Submitting leave request" error on the
# STM32H750 (the chip resets and drops USB before dfu-util gets its ack), so the
# firmware step tolerates that; the sample step must genuinely succeed.
program-all: $(BUILD_DIR)/$(TARGET_BIN)
	@test -f "$(SAMPLE_BIN)" || { echo "no $(SAMPLE_BIN) -- run: make sample SAMPLE_SRC=..."; exit 2; }
	@echo "==> 1/2 sample -> QSPI $(SAMPLE_ADDR)"
	dfu-util -a 0 -s $(SAMPLE_ADDR) -D "$(SAMPLE_BIN)" -d ,0483:df11
	@echo "==> 2/2 firmware -> $(FLASH_ADDRESS)"
	-dfu-util -a 0 -s $(FLASH_ADDRESS):leave -D $(BUILD_DIR)/$(TARGET_BIN) -d ,0483:df11
	@echo "==> done (ignore any 'Error during download get_status' above)"
