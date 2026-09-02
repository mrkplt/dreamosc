# dreamosc — Stretch Sequencer for Daisy Pod

A PaulStretch-based texture instrument for the **Electrosmith Daisy Pod** (Seed3,
STM32H750). A step sequencer whose 8 steps address positions inside a *virtual*
PaulStretch that is never fully rendered — each step pulls a fixed duration of
phase-randomized sound from a named point in the stretch and fades into the next.

Read `archive/stretchsequencerspec.md` for the full design. It is the source of
truth for behavior.

## Mental model (read this before reasoning about voices)

**8 read heads sweeping through the stretch, with spread setting how much their
sweeps overlap.** Not one scanning cursor, not 8 fixed taps, and NOT polyphony.

- Each of the 8 steps is a **read head** starting at its Position in the virtual
  stretch.
- While a step sounds (its **duration**), its head **travels** — the distance set
  by the **stretch factor**. Low stretch: the head travels far, scanning through
  material. High stretch: it barely moves, a frozen/held sound.
- **Spread** (0..1) sets the time between head starts as a fraction of duration:
  `interval = duration * spread`. **0% = all 8 heads fire together** (maximum
  overlap, pattern lasts one duration); **100% = end-to-end**, each head starts as
  the previous ends (pattern lasts 8 durations). 50% at a 4 s duration = heads 2 s
  apart. So spread runs from fully-stacked (0) to fully-sequential (1) — note this
  is the INVERSE of the original spec/`.ino` comment ("0 butt-joined"), which was
  wrong; the code now implements the correct direction.
- **The 8 heads are PERSISTENT objects, not allocated voices — and synthesis is
  DEMAND-DRIVEN, not queued.** Each step owns one `Head` forever, modeled as
  LANES: the active life plus the fading remnant of the previous one, each just
  (old frame, cur frame, phase); the per-sample value is the PaulXStretch
  raised-cosine blend, with null-old = natural rise and null-cur = natural
  tail. A step re-firing is a sample-accurate life HANDOFF inside its own head;
  a life re-fired while sounding finishes only its hop + tail (no extra
  renders). The ring between synthesis and the ISR is a SMALL CUSHION
  (`SS_FILL_TARGET` 512 / `SS_SLICE` 128: fill oscillates 384-512, ~8-11 ms
  knob-to-ear), not committed audio: frames render as late as the cushion
  allows, from LIVE controls — the fill IS the knob-to-ear latency (the first
  build filled a 4-hop ring: ~170 ms, audibly rubbery; do not regress this).
  **Nothing rendered is sacred**: `SS_W` is the analysis INPUT, not an output
  quantum — `Head::refresh()` re-sources a sounding life's spectrum mid-hop
  from live params (same srcPos/rng/body clock, so it is the same life, not a
  retrigger), the hook for frame-size and other spectral controls. Worst-case
  synthesis is 8 streams at hop rate, constant across the spread range. (The
  original voice-pool design let sustained concurrency hit 16 at low spread —
  the #132 "noise near spread 0" bug.) The spec's "polyphonic voice allocator"
  phrasing describes rendering machinery, not chords: one source, one
  timeline, one sequence.

## Toolchain (already installed on this machine)

- `arm-none-eabi-gcc` 15.3.1 — ARM cross-compiler (`brew install --cask gcc-arm-embedded`)
- `dfu-util` 0.11 — flashes the H750 over USB DFU (`brew install dfu-util`)
- `arduino-cli` 1.5.1 — present but NOT used here (we chose the native libDaisy path)
- Host verification uses a Python venv at `dreamosc/host/.venv` (numpy)

**Path chosen: native libDaisy + Makefile, NOT DaisyDuino/Arduino.** The uploaded
`StretchSeq.ino` was a DaisyDuino wrapper; we are porting its wiring to libDaisy.
The DSP core (`stretch_core.h`, `shy_fft.h`) has no platform dependencies and is
shared verbatim between host and device.

## Dependencies (cloned, gitignored — re-fetch if missing)

```
cd ~/src/dreamosc
git clone --recursive https://github.com/electro-smith/libDaisy.git
git clone --recursive https://github.com/electro-smith/DaisySP.git
make -C libDaisy && make -C DaisySP        # build the static libs once
```

## Layout

```
dreamosc/
  stretch_core.h      DSP core: Source, Voice, Sequencer (portable, verified)
  shy_fft.h           Emilie Gillet's embedded real FFT (MIT)
  sd_source.h         Load a WAV off microSD -> SDRAM -> Source. THE source seam.
  dreamosc.cpp        (WIP) Pod firmware — currently a placeholder oscillator
  Makefile            libDaisy build; targets ../libDaisy and ../DaisySP
  host/
    host_main.cpp     Host harness: WAV -> Sequencer -> WAV (defines the globals)
    stretchseq.py     Python reference (ground truth for the DSP)
archive/              The original reference files, as delivered (see below)
```

## Build & flash

```
# Device firmware
cd dreamosc && make                 # -> build/dreamosc.bin
make program-dfu                    # BOOT+RESET the Seed into DFU first

# Host tests (run before committing a DSP change)
cd dreamosc/test && ./run.sh    # Catch2 unit tests + golden C++-vs-Python regression

# Ad-hoc render (manual listening / debugging)
cd dreamosc/host
c++ -std=c++17 -O2 -I.. host_main.cpp -o stretchcore
./stretchcore in.wav out.wav --stretch 50 --duration 4 --spread 1
```

**Testing culture: the DSP core is platform-free so it can be tested on the host —
run `dreamosc/test/run.sh` and keep it green. A change to `stretch_core.h` /
`shy_fft.h` is not done until the suite passes. Add a test with new behavior; set
tolerances from measurement with a written reason, never loosen one to hide a
regression.** Details in `dreamosc/test/README.md`.

### Vendored dependencies

Third-party sources are **pinned to an exact upstream commit** in
`vendor/manifest.txt` (`<repo> <sha> <src-path> <dest-path>`) and fetched by
`vendor/fetch.sh`, wired into the build:

```
make vendor          # fetch/refresh vendored sources at their pinned SHAs
make vendor-check    # verify on-disk files still match upstream (build gate)
```

`vendor-check` also runs at the top of `dreamosc/test/run.sh`. A fresh clone
fetches automatically on first build. Vendored files land under
`dreamosc/vendor_stmlib/` and are **byte-identical to upstream** — no local
edits, ever.

**Rules:**
1. **Never edit a vendored file in place.** `vendor-check` re-fetches from the
   pinned SHA and fails on any difference, so an edit is caught rather than
   silently carried. If upstream is broken, either bump the SHA to a fixed
   upstream commit or work around it from *our* code.
2. **Check upstream before writing tests or fixes.** Look for existing upstream
   tests before writing your own, and an existing upstream fix before writing a
   local one.
3. **Pin the source of truth, not the most convenient copy.** Choose the
   longest-lived / best-supported repo that is actually authoritative for the
   file. `shy_fft.h` comes from `pichenettes/stmlib` (its canonical home) rather
   than Electro-Smith's DaisyExamples copy, which pins stmlib at a June-2021
   commit predating the `Math<double>` cos/sin infinite-recursion fix — pinning
   there would have baked in a known-buggy snapshot.

**Why this exists:** a self-recursive `cos`/`sin` bug in `shy_fft.h` was found by
writing a test against it, then patched *in place* — without first checking
whether upstream already had tests (it has none) or had already fixed the bug (it
had, in PR #7, 2021). A hand-copied file with a silent local edit has no
provenance, no pinning, and no rollback path, and the next re-vendor wipes the fix
with no signal. Pinning at a SHA plus a drift check gives all three.

### Flashing the Pod (DFU)

`make program-dfu` uploads over USB DFU. The board must be in the bootloader first:
on the Seed there are two unlabeled tactile buttons — **hold one, tap the other,
release the first** (empirically on this board: hold the button [POSITION TBD —
fill in relative to the USB connector], tap the other). Confirm with `dfu-util -l`
showing `[0483:df11] ... @Internal Flash /0x08000000` before flashing.
`make program-dfu` ends with `dfu-util: Error during download get_status` /
`Error 74` on the "Submitting leave request" step — this is **harmless** on the
STM32H750 (the chip resets and drops USB before dfu-util gets its ack); the
`File downloaded successfully` line above it means the flash landed.

### Sample audio: QSPI (temporary scaffolding)

**Why:** internal flash is 128 KB and the firmware already uses ~89 KB, so real
sample material (hundreds of KB) cannot be embedded. QSPI is 8 MB and
memory-mapped, so the firmware reads samples in place with no copy.

**This is temporary.** It exists so the #132 controls can be judged on broadband
material — a sine has no spectral variation across the buffer, so moving a read
head sounds identical everywhere and the controls appear to do nothing. The
intended long-term source is the SD card path (#131, `sd_source.h`, already
written). Revert this once controls are settled.

```
make sample SAMPLE_SRC=/path/to.wav   # WAV -> tools/wav2raw.py blob
make program-boot                     # ONE TIME: install the Daisy bootloader
make program-sample                   # upload the blob to QSPI
```

**The Daisy bootloader is required** because the STM32 ROM bootloader exposes
only Internal Flash + Option Bytes over DFU — no QSPI target. Notes on it:

- **Use `APP_TYPE = BOOT_SRAM`**, never `BOOT_QSPI`. SRAM execution is
  "comparable speed to internal flash" with a 480 KB limit (we use ~89 KB);
  QSPI execution is "more cache-dependent" i.e. slower, which is a real risk
  for a real-time audio callback.
- The bootloader **reserves the first 256 KB of QSPI** (firmware images load at
  `0x90040000`). Sample data therefore lives at **`0x90100000`** (1 MB in) —
  `QSPI_BASE` in `dreamosc.cpp` and the address in `make program-sample` must
  agree.
- With the bootloader installed, programs **cannot use internal flash**.
- The blob is self-describing (magic `DRMO`, count, rate) so the firmware
  validates it and falls back to a synthesized source if QSPI is empty or
  erased — it never plays garbage.

## State of play (what's done, what's next)

- **Synthesis: canonical PaulXStretch, settled by listening on hardware.** Each
  frame's IFFT is a full periodic waveform; each output block raised-cosine-blends
  the current frame's second half against the previous frame's first half, times
  the `0.853553…` = `(1+1/√2)/2` AM-correction curve (ported against the real
  `essej/paulxstretch` `Stretch.cpp`, not a summary of it). Seams between heads
  are **window-mediated overlap** — no envelope, no crossfade, no pre-roll — which
  beat the butt-joint alternative in an A/B on hardware. Analysis window is
  Nasca's `(1-x²)^1.25`; a rectangular window leaked 0.6% out-of-band energy and
  was audibly scratchy.
- **DSP core: host-tested.** Run `dreamosc/test/run.sh` — vendored-source drift
  check plus Catch2 unit tests over `stretch_core.h` (determinism, NaN/bounds, the
  spread model, constant-loudness, drift, multi-samplerate, click detection,
  bounded-work-at-any-spread, live-spread-change recovery, sub-hop onsets) and
  `shy_fft.h` (round-trips across sizes/types). 23 cases; gates on exit code. See
  `dreamosc/test/README.md`.
- **Persistent-head / lane rework (#132).** The voice allocator (arm queue,
  `SS_MAX_VOICES` slots, per-start memset/re-prime) is gone; see the
  mental-model bullet above for the lane model and the demand cushion.
  Scheduling is pattern-anchored on the producer side (`Sequencer::service`):
  candidates at `anchor + s*interval` with the LIVE interval, per-head fired
  flags, and a phase-fraction rescale on period change so live spread/duration
  sweeps neither insert dead patterns nor double-fire (both failure modes were
  measured on ramps before the rescale). The audio ISR is just 8 ring reads +
  gain. There is NO one-hop content-latency floor: `renderFrame` reads stretch
  at each render (advance-at-use, not banked at the previous render), and
  `Head::refresh()` re-sources a sounding frame mid-hop — content latency is
  cushion + render + blend ramp, at any `SS_W`. Frame size as a live control
  (#136) rides the same refresh/handoff machinery.
  `make PROFILE=1` (a cflags stamp tracks flag changes, so no `make clean`
  needed when toggling PROFILE / FILL) prints
  per-second CPU accounting over USB serial (`rnd` = frames rendered/s is the
  load metric) for on-device verification, and its numbers are what should
  justify tightening `SS_FILL_TARGET` further.
- **SD reader: written and compile-checked** against the real libDaisy API
  (`SdmmcHandler` + `FatFSInterface`, mount at `"/"`, chunk-walking WAV parser,
  stereo->mono fold). Not yet run on hardware (no card yet).
- **Pod firmware (`dreamosc.cpp`): NOT DONE.** Still the throwaway oscillator.
  Needs: voices/source buffers in SDRAM (`DSY_SDRAM_BSS`), `Sequencer::next()` in
  the audio callback, `service()` in the main loop, Pod controls mapped.

## Key facts a future agent needs

- **Source is source-agnostic by design.** `Source { float* data; uint32_t len; }`
  with wrapping reads. `sd_source.h::load_source()` is the ONE seam that decides
  where audio comes from. Today: SD card at boot (card not present yet — stub with
  a test tone until one is). This SD path is reused in later projects, so it was
  built for real, not faked.
- **Controls = Daisy Pod, not a bare Seed.** 2 knobs + encoder (turn+click) +
  2 buttons. Plan: encoder turn -> stretch (wide exponential range), knob1 ->
  duration, knob2 -> spread, drift on a second page / encoder-click. Four globals
  do not fit two knobs directly. **This mapping is a first-cut constraint of the
  Pod's panel, NOT the intended final control surface** — the design wants a knob
  per parameter (and per-step, that is the spec's 16 numbers), so the
  encoder-juggling is provisional. Add knobs / dedicated controls as the hardware
  allows; don't treat two-knobs-plus-encoder as the design.
- **Memory.** `SS_W = 4096`. Per Head: 4 rotating frame buffers (two lives x
  old/cur) + a 1024-sample ring = 68 KB; 8 heads -> `headPool` = 544 KB, a
  **plain float array in SDRAM** (`Sequencer::init` carves it up; rings are
  memset so NOLOAD garbage never plays; frame buffers need no clearing — a
  lane never reads a buffer it hasn't rendered). The **source buffer (~1.9 MB)
  is also in SDRAM**; the `Sequencer` OBJECT (small: lane bookkeeping +
  atomics) stays in internal SRAM (~19% full). #130 is the deliberate
  budget/placement pass.
- **DO NOT put a C++ object with a constructor in `DSY_SDRAM_BSS`.** `.sdram_bss`
  is `NOLOAD` and SDRAM is not powered until `Init()`, so objects placed there get
  NEITHER their constructor run NOR their storage zeroed — they boot with garbage
  state. This cost us a silent-firmware bug in #129 (a `Sequencer` in SDRAM never
  activated its voices → no sound). SDRAM is fine for **plain arrays you memset
  yourself** (like the source buffer); keep constructed objects in SRAM, or if a
  voice pool must go to SDRAM later, placement-new it after `Init()`.
- **libDaisy + GCC 15.3 wrinkle:** `WavPlayer.h` throws a `[-Wtemplate-body]`
  error (`FileReader` vs `IReader`) when transitively included. It is upstream, not
  ours. Avoid pulling that header, or pin/patch it when building `dreamosc.cpp`.
- **Two Pythons, one truth:** `paulstretch.py` is the older origin convention;
  `stretchseq.py` + `stretch_core.h` are the current, matching pair. Verify against
  `stretchseq.py`.

## archive/

Original reference files as delivered, kept verbatim for provenance:
`paulstretch.py`, `stretchseq.py`, `stretchsequencerspec.md`, `stretch_core.h`,
`shy_fft.h`, `StretchSeq.ino`. The working copies of `stretch_core.h` / `shy_fft.h`
under `dreamosc/` are the ones the build uses; the archive copies are the untouched
originals.
