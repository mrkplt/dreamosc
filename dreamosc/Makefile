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
# (service-loop us, per-hop us, ISR us, underruns) over USB serial. See the
# PROFILE blocks in dreamosc.cpp.
ifdef PROFILE
C_DEFS += -DPROFILE
endif

# `make FILL=1024`: override the per-head demand-cushion depth (samples,
# power of two; default 512 in stretch_core.h). The cushion IS the knob-to-ear
# latency (~fill/48 ms) AND the budget the worst render burst must fit inside —
# raise it if LED 1 latches underruns, lower it once PROFILE numbers show
# headroom. Flag changes are tracked (cflags.stamp), so no `make clean` is
# needed between differing-flag builds.
ifdef FILL
C_DEFS += -DSS_FILL_TARGET=$(FILL)
endif

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

# --- Flag tracking -----------------------------------------------------------
# The core Makefile's dependency files track headers but not command-line
# defines (PROFILE, FILL), so an incremental `make PROFILE=1` after a plain
# `make` reuses stale objects. A stamp file whose CONTENT is the current C_DEFS
# fixes this without cleaning the world every build: it is a prerequisite of
# every object and is only rewritten (moving its mtime) when the defines
# actually change, so objects rebuild exactly on a flag change and are left
# alone otherwise. The comparison happens at parse time via $(file <...), and
# the stamp is written at parse time too — both before the object rules run, so
# by the time make checks prerequisites the mtimes are settled. mkdir guards a
# clean tree; the stamp is recreated (and thus objects rebuilt) after a clean,
# which is correct — a clean tree has no objects anyway.
FLAGSTAMP := $(BUILD_DIR)/cflags.stamp
$(shell mkdir -p $(BUILD_DIR))
ifneq ($(strip $(file < $(FLAGSTAMP))),$(strip $(C_DEFS)))
$(shell printf '%s' '$(strip $(C_DEFS))' > $(FLAGSTAMP))
endif
$(OBJECTS): $(FLAGSTAMP)

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
