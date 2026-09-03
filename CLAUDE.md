# dreamosc — Stretch Sequencer for Daisy Pod

A PaulStretch-based texture instrument for the **Electrosmith Daisy Pod** (Seed3,
STM32H750). A step sequencer whose 8 steps address positions inside a *virtual*
PaulStretch that is never fully rendered — each step pulls a fixed duration of
phase-randomized sound from a named point in the stretch and fades into the next.

Read `archive/stretchsequencerspec.md` for the full design. It is the source of
truth for behavior.

## Issue tracking (Fizzy)

dreamosc has its OWN board on the self-hosted Fizzy instance (PiHost, LAN-only) —
**do NOT create dreamosc cards on the default Flail Whale board** (that's the
play-by-post backlog). Use the `fizzy` CLI (see the `fizzy-cards` skill) with the
dreamosc board id explicitly:

```
fizzy boards                                            # list boards (id + name)
fizzy cards --all --board 03gsa1lwpx0m5f4k5f2p16toh     # dreamosc board
fizzy create "<title>" "<body>" --board 03gsa1lwpx0m5f4k5f2p16toh
fizzy move <card#> 03gsa1lwpx0m5f4k5f2p16toh            # if it landed on the wrong board
```

- **dreamosc board id: `03gsa1lwpx0m5f4k5f2p16toh`** (Flail Whale is the CLI
  default at `03gmw8zdlnigtxsgs2hknc3bx` — always pass `--board` for dreamosc).
- Columns are by nature of work: Now / Next / Sound / Controls / Infra / Ideas.
- **Fizzy is the live source of truth for what's open, done, and next** — query
  it (`fizzy cards --all --board …`) rather than trusting a card list frozen in
  this file. Don't maintain a card snapshot here; it rots.

## Architecture: hardware glue vs testable cores

The load-bearing structural rule of this codebase:

- **`dreamosc.cpp` holds ONLY what is exclusive to the hardware** — the audio
  callback, ADC/knob/encoder/button reads, `led.Set`, `PrintLine`, DFU/QSPI
  wiring, `System::GetNow`. It `#include`s `daisy_pod.h`, so it **cannot be
  compiled on the host** and cannot be unit-tested. Keep it as thin as possible:
  it reads the panel, calls into a core, and writes the result to the hardware.
- **Everything else — all real logic — lives in platform-free `*_core.h`
  headers** (`stretch_core.h` = DSP, `controls_core.h` = control-surface logic).
  These have no Daisy/Arduino dependency, so they compile and run on a desktop
  and are exhaustively host-tested (`dreamosc/test/run.sh`).
- **When you write behavior in `dreamosc.cpp`** — a mapping, a clamp, a state
  machine, a stepping rule, a decision — **EXTRACT it to a core header and test
  it.** The bar is that as little as sensibly possible is left in the untestable
  edge. This is not optional polish: real bugs shipped precisely because they
  lived in the un-host-compilable firmware (a drift double-add; a pickup
  reference that chased the pot). Extracting them is what made them testable and
  caught the next regression.

The testing-culture note under "Build & flash" restates the discipline; this is
the *why* and the *where*.

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
  varies is the SEAM between them, set by **`fade`** (the crossfade overlap, on
  the encoder's fade page):
  - **fade 0:** butt-joint — a hard cut from one head to the next.
  - **fade > 0:** the two heads overlap and equal-power (constant-loudness)
    crossfade. Overlap is 0..50% of the duration. At 50% each step is
    [mix ¼][clean ½ → 0][mix ¼]; **50% is the hard ceiling because beyond it a
    THIRD head would overlap** — the whole point is that AT MOST TWO heads ever
    sound at once. (There is no separate on/off toggle: fade 0 IS butt-joint.)
- **Frame size (`SS_W`) is a sound-CHARACTER axis** (Fizzy #136), a live control:
  small window = grainy/articulated, large = glassy/frozen. `SS_W` is the
  compile-time buffer MAX; the active window is runtime-adjustable (one ShyFFT
  instance, its runtime-length overload). Stretch and frame size together set
  "how frozen" a head is.
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
  stretch_core.h      DSP core: Source, Voice, Sequencer, StretchTables (portable)
  controls_core.h     Control-surface logic: PanelEditor, encoder stepping, LED
                      colors (portable, host-tested — see CONTROLS.md)
  shy_fft.h           Emilie Gillet's embedded real FFT (MIT; vendored)
  sd_source.h         Load a WAV off microSD -> SDRAM -> Source. THE source seam.
  dreamosc.cpp        Pod firmware: hardware glue (audio callback, controls, LEDs)
  CONTROLS.md         The current control mapping (progressive disclosure)
  Makefile            libDaisy build; targets ../libDaisy and ../DaisySP
  host/
    host_main.cpp     Host harness: WAV -> Sequencer -> WAV (defines the globals)
    stretchseq.py     Python reference (origin convention; see note below)
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

### Tag discipline (hardware checkpoints)

Tags mark **hardware-verified** vs **experimental** states, because host tests
passing does NOT mean it sounds right on the Pod — this project's whole history
is bugs that only showed on hardware.

- **`alphaN`** (`alpha1`, `alpha2`, …): a state **confirmed working on hardware**
  by ear. Bump to the next `alphaN` only after a bench session validated it.
  These are the safe fall-back points.
- **`experimental`**: the latest change that is **host-tested but NOT yet heard
  on hardware**. Moves forward freely; graduates to the next `alphaN` once
  confirmed on the bench, or gets abandoned if it doesn't hold up. Always
  re-pointable (`git tag -f experimental <commit>` + `git push -f origin
  experimental`).
- When the user asks to "tag alphaN" / "tag experimental", create an **annotated**
  tag (`git tag -a`) on the right commit and **push the tag to origin** (a plain
  `git push` does NOT push tags — `git push origin <tag>`). Verify with
  `git ls-remote --tags origin <tag>`.
- Reference the driving Fizzy card (dreamosc board `03gsa1lwpx0m5f4k5f2p16toh`)
  in the commit message when a change implements one.

**Testing culture: push high coverage, test all the time.** The bar is that as
little as sensibly possible is left untested. The mechanism is architectural:
**pure decision logic lives in platform-free `*_core.h` headers so it compiles
and is tested on the host; `dreamosc.cpp` holds ONLY hardware glue** (ADC reads,
`led.Set`, `PrintLine`, `RisingEdge`, `System::GetNow`) — the thin, untestable
edge. When you write logic in `dreamosc.cpp` that has behavior worth checking
(a mapping, a clamp, a state machine, a stepping rule), EXTRACT it to a core
header and test it rather than leaving it in the firmware. This is not optional
polish — a real bug (a drift double-add) shipped precisely because it lived in
the un-host-compilable `dreamosc.cpp`; extracting it (`controls_core.h`) is what
made it testable. Cores today: `stretch_core.h` (DSP), `controls_core.h`
(control-surface logic). Run `dreamosc/test/run.sh` and keep it green; a change
to any core is not done until the suite passes. Add a test with new behavior;
set tolerances from measurement with a written reason, never loosen one to hide
a regression. Details in `dreamosc/test/README.md`.

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
  `essej/paulxstretch` `Stretch.cpp`, not a summary of it). Within a head, block
  junctions are window-mediated overlap. BETWEEN heads the seam is the **`fade`
  crossfade**: fade 0 keeps the natural one-hop tail (butt-joint); fade > 0
  applies an equal-power (quarter-sine) fade so ≤2 heads sum to constant power
  (a full-volume-tail bug that clicked at every crossfaded seam is fixed and
  guarded by a test). Analysis window is Nasca's `(1-x²)^1.25`; a rectangular
  window leaked 0.6% out-of-band energy and was audibly scratchy.
- **DSP + control cores: host-tested.** Run `dreamosc/test/run.sh` — vendored-
  source drift check plus Catch2 unit tests over `stretch_core.h` (determinism,
  bounds, the crossfade/interval model, constant-loudness, drift, multi-
  samplerate, click detection, per-frame-size rendering), `controls_core.h`
  (pickup, mode navigation, drift-fold, encoder stepping, LED colors), and
  `shy_fft.h` (round-trips). Gates on exit code; the count grows with behavior —
  read it from the run, don't hardcode it here. See `dreamosc/test/README.md`.
- **Pod firmware: WORKING on hardware through `alpha2`** (crossfade model,
  detented stretch to 10000×, two-mode panel controls, PROFILE diagnostics).
  `experimental` adds live frame size on top (host-tested, not yet heard on the
  bench). See the tag discipline above and Fizzy for what's next.
- **SD reader: written and compile-checked** against the real libDaisy API
  (`SdmmcHandler` + `FatFSInterface`, mount at `"/"`, chunk-walking WAV parser,
  stereo->mono fold). Not yet run on hardware (no card yet — Fizzy #131). Source
  today is the QSPI sample scaffolding (see below).

## Key facts a future agent needs

- **Source is source-agnostic by design.** `Source { float* data; uint32_t len; }`
  with wrapping reads. `sd_source.h::load_source()` is the ONE seam that decides
  where audio comes from. Today: SD card at boot (card not present yet — stub with
  a test tone until one is). This SD path is reused in later projects, so it was
  built for real, not faked.
- **Controls → see `dreamosc/CONTROLS.md`** for the full, current mapping (two
  knob modes global/step via button1, encoder pages stretch/fade/frame, pickup,
  the ROYGBIVW step LED, the speed model, and the hard-won pickup/polling facts).
  The one-line summary: it's a DEVELOPMENT surface (reach every parameter), not
  the intended performance interface — the design wants a knob per parameter.
  All the pure control logic is host-tested in `controls_core.h`; `dreamosc.cpp`
  is only glue. **Keep CONTROLS.md updated when the mapping changes** — don't let
  a control snapshot rot in this file.
- **`make PROFILE=1`** builds a diagnostic firmware that prints per-second lines
  over USB serial (`screen /dev/tty.usbmodem* 115200`): a `SET` line (full
  instrument state), `KNOB`/`POS`/`DRF` lines (per-step + control state), and an
  `HLTH` line (active voices, service µs, per-render `avg_us`, ISR µs, underruns),
  all as scaled integers (nano-newlib printf can't do floats). This is how the
  spread-0 CPU overload was measured, how the pickup and "steps go silent" bugs
  were diagnosed from the board instead of by conjecture, and how any control/DSP
  behavior is confirmed on-device. Gated behind `-DPROFILE`; costs nothing in
  normal builds.
  **DIRECTIVE: every variable parameter goes in the profiler.** When you add a
  control or a piece of runtime state (a new global, a per-step value, a mode,
  frame size, an engagement flag), add it to the relevant PROFILE line in the
  same change. The recurring lesson this project keeps teaching is that
  bench debugging is only fast when the board can *show* its state — chasing
  "is this working?" by ear or by theory wasted whole sessions; a value on the
  serial line settles it in one glance. No new parameter is done until it's
  visible in `make PROFILE=1`.
- **Memory.** `SS_W = 4096` is the compile-time buffer MAX (the runtime window is
  ≤ this — frame-size control). Per Voice: `old_[SS_W]` + `ring_[2*SS_W]` = 48 KB;
  `SS_MAX_VOICES = 2*SS_STEPS = 16` slots (a voice's tail outlives the re-fire
  period, so slots must exceed SS_STEPS or steps drop) → the voice pool (~768 KB)
  lives in **SDRAM** (`headPool`, a plain float array `init()` carves + memsets),
  as does the **source buffer (~1.9 MB)**. The `Sequencer` OBJECT stays in
  internal SRAM (~19% full). The crossfade model caps CONCURRENT sounding heads at
  2, so CPU is comfortable even though 16 slots exist.
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
