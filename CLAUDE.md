# dreamosc — Stretch Sequencer for Daisy Pod

A PaulStretch-based texture instrument for the **Electrosmith Daisy Pod** (Seed3,
STM32H750). A step sequencer whose 8 steps address positions inside a *virtual*
PaulStretch that is never fully rendered — each step pulls a fixed duration of
phase-randomized sound from a named point in the stretch and fades into the next.

Read `archive/stretchsequencerspec.md` for the full design. It is the source of
truth for behavior.

## Mental model (read this before reasoning about voices)

**8 read heads stepping through the stretch SEQUENTIALLY, one at a time, with a
crossfade at each seam.** Not one scanning cursor, not 8 fixed taps, and NOT
polyphony.

- Each of the 8 steps is a **read head** starting at its Position in the virtual
  stretch.
- While a step sounds (its **duration**), its head **travels** — the distance set
  by the **stretch factor**. Low stretch: the head travels far, scanning through
  material. High stretch: it barely moves, a frozen/held sound. NOTE the stretch
  ↔ duration coupling: stretch controls how FAR a head travels, duration how LONG
  it has to travel, so at very short durations stretch is nearly inaudible (the
  head barely moves either way) — it comes alive at multi-second durations.
- **Heads are sequential.** Head N ends, head N+1 begins. The only thing that
  varies is the SEAM between them, set by the **crossfade** (button 2 on/off):
  - **Crossfade OFF:** butt-joint — a hard cut from one head to the next.
  - **Crossfade ON:** the two heads overlap and equal-power (constant-loudness)
    crossfade. The overlap length is variable 0..50% of the duration (`fade`, on
    the encoder's fade page). At 50% each step is [mix ¼][clean ½... shrinking to
    0][mix ¼]; **50% is the hard ceiling because beyond it a THIRD head would
    overlap** — the whole point is that AT MOST TWO heads ever sound at once.
- **This REPLACES the old `spread` model** (which let all 8 heads stack at
  spread 0 — a ~2.3× CPU overload on the H750 that caused constant dropouts,
  measured via `make PROFILE=1`). The crossfade design caps concurrency at two
  heads, so `avg_us` stays in the comfortable ~49 µs / du=0 regime at every
  setting. The origin of "spread" was always these seam crossfades; this restores
  that intent and drops the fast-overlap extreme that never fit the chip.
- `SS_MAX_VOICES` is the ceiling on how many overlapping traveling heads / fade
  tails can render simultaneously (never more than two now). The spec's
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
  spread model, constant-loudness, drift, multi-samplerate, click detection) and
  `shy_fft.h` (round-trips across sizes/types). 20 cases; gates on exit code. See
  `dreamosc/test/README.md`.
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
  2 buttons + 2 RGB LEDs + a PERMANENTLY attached SSD1309 OLED (I²C). Current map:
  - **knob1** → duration (0.25..60 s), **knob2** → drift.
  - **encoder turn** → stretch (page 0) or crossfade length (page 1);
    **encoder click** toggles page.
  - **stretch mapping is speed-sensitive:** slow single detents are LINEAR trim
    (±0.5×, easy to land a value); a fast spin (≤40 ms between detents) switches
    to LOGARITHMIC ratio steps so the full 1×..500× range is ~2 turns. This
    matters because `Encoder::Increment()` only ever returns ±1 — speed must be
    inferred from the detent GAP, not the step magnitude.
  - **button2** → crossfade on/off. **led2** → page hue (blue=stretch,
    green=fade) with brightness = crossfade state. **led1** → underrun latch.
  - **Controls poll on a 1 ms WALL-CLOCK tick**, not every-Nth-`service()`: a
    `service()` call is a full FFT (~2.4 ms), so the old service-count gate polled
    the encoder only ~6×/s and dropped most detents. Poll on `System::GetNow()`.
  **This mapping is a first-cut constraint of the Pod's panel, NOT the intended
  final control surface** — the design wants a knob per parameter. Add dedicated
  controls as the hardware allows.
- **`make PROFILE=1`** builds a diagnostic firmware that prints two lines/sec over
  USB serial (`screen /dev/tty.usbmodem* 115200`): a `SET` line (full instrument
  state — stretch, duration, drift, fade, xfade, page, as scaled integers) and an
  `HLTH` line (active voices, service µs, per-render `avg_us`, ISR µs, underruns).
  This is how the spread-0 CPU overload was measured and how control/DSP behavior
  is confirmed on-device. Gated behind `-DPROFILE`; costs nothing in normal builds.
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
