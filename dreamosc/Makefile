# Project Name
TARGET = dreamosc

# Sources
CPP_SOURCES = dreamosc.cpp

# Library Locations (sibling clones under ~/src/dreamosc)
LIBDAISY_DIR = ../libDaisy
DAISYSP_DIR  = ../DaisySP

# Built for the Daisy bootloader. BOOT_SRAM runs from internal SRAM:
# comparable speed to internal flash, 480 KB limit (we use ~130 KB). Do NOT
# use BOOT_QSPI -- executing from QSPI is "more cache-dependent" i.e. slower,
# a real risk for the audio callback. Consequence that matters (sd_source.h):
# under BOOT_SRAM .bss/.data/stack are DTCM, which the SDMMC DMA cannot
# reach, so every FatFs DMA target is placed in AXI SRAM by hand.
APP_TYPE = BOOT_SRAM

# FatFs for the microSD source (#131, sd_source.h). libdaisy.a already carries
# ff.c/diskio.c, but NOT option/ccsbcs.c (ff_convert/ff_wtoupper, needed with
# long-file-name support on); USE_FATFS=1 makes libDaisy's core Makefile add
# the FatFs sources to this build -- the same wiring the DaisyExamples use.
# Note the bootloader is still required (`make program-boot`, once): with it
# installed programs cannot use internal flash, and it is what makes the
# 480 KB SRAM image possible.
USE_FATFS = 1

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

# `make PROFILE=1`: diagnostic build that prints the full instrument state once
# a second over USB serial (SET/KNOB/POS/DRF/HLTH/COST lines: controls, per-step
# state, CPU and supply accounting, per-size render cost, stack high-water).
# See profilePrint() in dreamosc.cpp. Run `make clean` when toggling it;
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
# gates that the working clones haven't drifted (run it before a device build
# after touching the clones; test/run.sh runs only the vendor check).
DEPS_DIR = ../deps
deps:
	$(DEPS_DIR)/setup.sh

deps-check:
	$(DEPS_DIR)/setup.sh --check

# Compile-check sd_source.h on its own against the real libDaisy/FatFs API
# with the device toolchain and flags (it is also compiled as part of
# dreamosc.cpp; this catches a header that only builds because of what the
# firmware happened to include before it). Syntax/semantics only.
.PHONY: sd-check
sd-check:
	@mkdir -p $(BUILD_DIR)
	@printf '#include "sd_source.h"\n' > $(BUILD_DIR)/sd_check.cpp
	$(CXX) $(MCU) $(C_DEFS) $(C_INCLUDES) $(CPP_STANDARD) -fno-exceptions -fno-rtti \
	    -fsyntax-only -I. $(BUILD_DIR)/sd_check.cpp
	@echo "sd_source.h compiles against libDaisy $(shell git -C $(LIBDAISY_DIR) rev-parse --short HEAD)"

# Fetch vendored sources automatically if missing, so a fresh clone builds
# without a separate manual step.
$(VENDOR_STAMP):
	$(VENDOR_DIR)/fetch.sh

build/$(TARGET).o: $(VENDOR_STAMP)
