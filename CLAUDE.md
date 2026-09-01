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
- `SS_MAX_VOICES` (6) is NOT musical polyphony — it is the ceiling on how many
  overlapping traveling heads / fade tails can render simultaneously. The spec's
  "polyphonic voice allocator" phrasing describes that rendering machinery, not
  chords. There is one source, one timeline, one sequence.

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

## State of play (what's done, what's next)

- **DSP core: host-tested.** Run `dreamosc/test/run.sh` — Catch2 unit tests over
  `stretch_core.h` (determinism, NaN/bounds, the spread model, constant-loudness,
  multi-samplerate) plus a golden regression that diffs the C++ core against the
  Python reference (length / RMS / spectral shape, tolerances documented). Both
  gate on exit code. See `dreamosc/test/README.md`. The port is faithful to
  `stretchseq.py` (phase differs by design — different RNGs — so tests assert
  invariants, not bit-equality).
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
- **Memory.** `SS_W = 4096`. Per Voice: `accum_[4096]` + `ring_[4096]` = 32 KB;
  `SS_MAX_VOICES = SS_STEPS = 8` -> ~256 KB (raised from 6 because spread 0 fires
  all 8 heads at once — the ceiling must be 8 or 0% silently drops heads). As built
  in #129: the **source buffer (~1.9 MB) is in SDRAM** (it must be — too big for
  SRAM), and the **`Sequencer` (with its 8 voice buffers, ~256 KB) is in internal
  SRAM** (SRAM lands ~70% full). #130 is the deliberate budget/placement pass.
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
