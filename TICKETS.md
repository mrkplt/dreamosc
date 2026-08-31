# dreamosc — remaining work

Ticket breakdown for the Stretch Sequencer Daisy Pod build. Ready to file onto a
Fizzy `dreamosc` board once it exists (Fizzy has no board-creation API — create
the board in the UI, then these go up with `fizzy create ... --board <id>`).

Status at time of writing: DSP core host-verified; SD reader written and
compile-checked; Pod firmware is still a placeholder oscillator.

---

## 1. Pod firmware skeleton — globals + audio path
**Type:** Feature
Replace the placeholder oscillator in `dreamosc/dreamosc.cpp` with the real
wiring: define the three globals `stretch_core.h` externs (`StretchTables gTab;
float gWork[SS_W]; float gSpec[SS_W];`), call `gTab.init()` at boot, run
`Sequencer::next()` in the audio callback and `Sequencer::service()` in the main
loop.
**Acceptance:** builds via `make`; flashes; produces sequencer output (using a
stub/test-tone source until SD lands) audible on the Pod.

## 2. SDRAM buffer placement
**Type:** Toil / Guards and Quality
Place the source buffer and the six voice buffers (`accum_[4096]` + `ring_[4096]`
per voice, ~196 KB total) in SDRAM (`DSY_SDRAM_BSS`). Confirm against the linker
map that internal SRAM is not overrun.
**Acceptance:** `build/dreamosc.map` shows voices + source in SDRAM; SRAM within
budget; no hard fault at boot.

## 3. SD card source load at boot
**Type:** Feature
Call `stretchsd::load_source()` (already written in `sd_source.h`) at boot to
fill the SDRAM source buffer from a WAV on microSD. Fall back to a synthesized
test tone if no card / no file (returns false).
**Acceptance:** with a card + WAV present, the instrument stretches that audio;
with no card, it falls back cleanly to the test tone.
**Depends on:** #1, #2. Needs a physical microSD with a 16-bit PCM WAV (not yet
on hand — easy to remedy).

## 4. Pod controls mapping
**Type:** Feature
Map the Pod's 2 knobs + encoder + 2 buttons onto the four globals: encoder turn
-> stretch (wide exponential range, ~5–500), knob1 -> duration (0.5–8 s),
knob2 -> spread (0–3). Drift on a second page (encoder click to switch), or fixed
for a first cut. Smooth pot reads (one-pole) as the .ino did.
**Acceptance:** each control audibly does what the spec describes; no zipper
noise; stretch is usable across the encoder's travel.

## 5. Host regression harness in CI-able form
**Type:** Guards and Quality
Turn the ad-hoc host comparison into a repeatable check: render fixed params
through both `host_main.cpp` and `stretchseq.py`, assert length identical, RMS
within tolerance, spectral correlation above threshold. A script under
`dreamosc/host/` that exits non-zero on regression.
**Acceptance:** one command renders + compares + passes/fails; documented in
CLAUDE.md.

## 6. Per-step position editing (stretch goal)
**Type:** Feature
The spec's compositional act is arranging the 8 step positions. Currently
hard-coded. Add a way to edit positions on the Pod (e.g. encoder-select a step,
knob sets its position) or load them from a preset line (the spec notes a preset
is "one line of text").
**Acceptance:** step positions changeable without a rebuild.

## 7. libDaisy + GCC 15.3 WavPlayer.h wrinkle
**Type:** Guards and Quality
`WavPlayer.h` throws `[-Wtemplate-body]` (`FileReader` vs `IReader`) under GCC
15.3 when transitively included. Confirm whether the real firmware pulls it in;
if so, pin/patch libDaisy or avoid the header. Document the resolution.
**Acceptance:** clean `make` with no template-body errors; note in CLAUDE.md.
