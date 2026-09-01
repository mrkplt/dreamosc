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

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

.PHONY: vendor vendor-check
vendor:
	$(VENDOR_DIR)/fetch.sh

vendor-check:
	$(VENDOR_DIR)/fetch.sh --check

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
#   make sample SRC=/path/to.wav   convert a WAV to the blob
#   make program-sample            upload the blob to QSPI over DFU
#
# NOTE: QSPI DFU requires Electrosmith's Daisy bootloader (the STM32 ROM
# bootloader exposes only Internal Flash + Option Bytes). Install it once with
# `make program-boot`, which libDaisy's core Makefile provides.
SAMPLE_BIN ?= ../sample/amen.bin
SAMPLE_SRC ?=

.PHONY: sample program-sample
sample:
	@test -n "$(SAMPLE_SRC)" || { echo "usage: make sample SAMPLE_SRC=/path/to.wav"; exit 2; }
	../tools/wav2raw.py "$(SAMPLE_SRC)" "$(SAMPLE_BIN)"

# 0x90400000 is the third QSPI region per the bootloader's DFU descriptor
# (0x90000000 reserved, 0x90040000 firmware images, 0x90400000 free).
# Must match QSPI_BASE in dreamosc.cpp.
program-sample:
	@test -f "$(SAMPLE_BIN)" || { echo "no $(SAMPLE_BIN) -- run: make sample SAMPLE_SRC=..."; exit 2; }
	dfu-util -a 0 -s 0x90400000:leave -D "$(SAMPLE_BIN)" -d ,0483:df11
