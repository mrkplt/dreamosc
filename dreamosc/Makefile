# Project Name
TARGET = dreamosc

# Sources
CPP_SOURCES = dreamosc.cpp

# Library Locations (sibling clones under ~/src/dreamosc)
LIBDAISY_DIR = ../libDaisy
DAISYSP_DIR  = ../DaisySP

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
