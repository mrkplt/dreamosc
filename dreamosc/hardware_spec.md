# Hardware spec — Daisy Seed / Pod (STM32H750)

Real, unchanging hardware facts for the target board, with citations. This is a
reference doc: it records what the silicon **is** and what each memory region is
**designed for**, independent of how dreamosc currently uses it. Correctness
discussions (are *we* placing data well?) belong elsewhere; this is the ground
truth they argue against.

Sources are ST's datasheet + system-architecture app note and Electrosmith's
docs, linked at the bottom. Where a number is board-observed rather than
datasheet-stated, it says so.

## The MCU: STM32H750IB

- **Core:** single Arm Cortex-M7, 32-bit, with FPU (IEEE-754 **single AND
  double** precision) and the full DSP instruction set + MPU.
- **Clock:** up to **480 MHz** (the Daisy runs it at 480 MHz).
- **Performance:** **2424 CoreMark / 1027 DMIPS** from flash at 0-wait-state,
  thanks to the L1 cache.
- **L1 cache:** **16 KB I-cache + 16 KB D-cache.** This is the key to why
  "slow" external memory is often not slow in practice — cached, sequential
  access to AXI SRAM / SDRAM is served largely from L1 (see below).
- **On-chip flash:** only **128 KB** (this is the "value line" H750 — it has the
  full H74x/75x silicon but a tiny mask-ROM-class internal flash, which is why
  real code/samples must live in QSPI/SDRAM).

## On-chip RAM regions — what each is DESIGNED for

The H7 is a multi-bus, multi-domain part. RAM is not one flat pool; it is several
physically distinct banks on different buses, with different latency, and ST
assigns each an intended role. From the CPU's five interfaces (AXIM, ITCM, DTCM,
AHBS, AHBP):

| Region        | Size   | Bus / domain      | CPU access                       | DMA?  | Designed for |
|---------------|--------|-------------------|----------------------------------|-------|--------------|
| **ITCM-RAM**  | 64 KB  | Instruction TCM, D1 | **0-wait, full CPU clock**, no shared matrix | MDMA only | Hot **code** (ISRs, tight loops) that must never stall on a bus. |
| **DTCM-RAM**  | 128 KB | Data TCM, D1 (2×32-bit) | **0-wait, full CPU clock**, no shared matrix | **No DMA** | The **most performance-critical DATA**: stacks, main heap, hot static/bss. ST: "put all performance-sensitive data in DTCM." Reserve it for critical data; move data to AXI SRAM as it grows too large. |
| **AXI SRAM**  | 512 KB (this LD carves 480 KB usable) | AXI bus matrix, D1 | Fast; **cacheable** via L1 D-cache; goes through the shared matrix (can contend) | Yes | The **big general-purpose fast RAM**: large buffers/working sets that outgrow DTCM. Standard home for audio buffers. |
| **SRAM1**     | 128 KB | AHB, **D2** domain | Slower than D1 (domain crossing has latency/bandwidth cost) | Yes | Peripheral/**DMA buffers** (D2 is the peripheral/DMA domain). |
| **SRAM2**     | 128 KB | AHB, **D2** domain | as SRAM1 | Yes | as SRAM1. |
| **SRAM3**     | 32 KB  | AHB, **D2** domain | as SRAM1 | Yes | DMA scratch / inter-processor (multicore parts). |
| **SRAM4**     | 64 KB  | AHB, **D3** domain | D3 low-power domain | Yes (most) | Data kept alive / used while D1/D2 are in low-power; D3 peripheral DMA. |
| **Backup SRAM** | 4 KB | D3 (backup)       | battery-domain                   | —     | Data that must survive standby / power-down. |

Key design principles ST states:
- **TCM (I/D) never crosses a shared bus matrix**, so it is 0-wait and immune to
  contention from DMA/peripherals — that is *the* reason it is the fast tier.
- **Anything not TCM (AXI SRAM, D2/D3 SRAM, external) is cacheable** and served by
  the L1 caches when cache is on. Only "normal memory type" is cacheable.
- **DMA cannot reach ITCM/DTCM.** DMA-facing buffers must be in AXI/D2/D3 SRAM.
- **Crossing power domains (D1↔D2↔D3) costs latency/bandwidth.** Keep a working
  set within one domain where it matters.
- The intended *placement strategy*: performance-critical data in **DTCM**;
  overflow and large working sets in **AXI SRAM**; DMA/peripheral buffers in
  **D2 SRAM**; low-power-retained data in **D3/backup**.

## External memory on the Daisy board

- **SDRAM: 64 MB**, on the FMC (flexible memory controller), memory-mapped at
  `0xC000_0000`. Electrosmith: "64 MB of RAM (up to 10-minute audio buffers)."
  - **Speed:** the FMC SDRAM interface tops out at **~100 MHz** (FMC kernel clock
    ÷2; pin drivers aren't rated past 100 MHz), i.e. roughly **CPU/4.8**. So a
    *cache-missing* SDRAM access is far slower than DTCM — BUT it is **cacheable**
    through the 16 KB L1 D-cache, so **sequential / streaming access (like FFT
    butterflies over a contiguous buffer) is largely served from cache** and the
    100 MHz wall is mostly hidden. Random-stride access is where SDRAM actually
    hurts. ("Slow" is workload-relative, and measurable — see the CLAUDE.md note
    on profiling `avg_us`.)
- **QSPI NOR flash: 8 MB**, memory-mapped at `0x9000_0000` (execute/read in
  place). Slower than internal flash and more cache-dependent; ST/ES advise SRAM
  execution over QSPI execution for real-time code.
- **Audio codec:** **96 kHz / 24-bit** stereo hardware. The specific codec part
  depends on board revision (see below).
- **Converters on the MCU:** 12× **16-bit ADC**, 2× **12-bit DAC**, 31× GPIO.

## Board revision / codec caveat (get this right per unit)

The "Seed" name covers several revisions with **different audio codecs** — this
matters because codec, not MCU, is what changed:

- **Daisy Seed (Rev4), "the original":** AK4556 / AK4430-class codec.
- **Daisy Seed 1.1 (Rev5):** pin-compatible, **WM8731** codec.
- **Daisy Seed 2 DFM:** **PCM3060** codec ("improved").
- libDaisy enumerates these as `DAISY_SEED`, `DAISY_SEED_1_1`, `DAISY_SEED_2_DFM`
  and still carries a deprecated `SEED_REV2` define.

**"Seed3" as used in this repo's notes is a shorthand, not an official ST/ES
SKU** — confirm the actual codec on the physical unit (via the libDaisy board
version or the silkscreen) before relying on codec-specific behavior. The MCU
(STM32H750) and the memory map are identical across these; only the codec and
minor analog details differ.

## Sources

- ST STM32H750xB datasheet (DS12556): <https://www.st.com/resource/en/datasheet/stm32h750ib.pdf>
- ST AN4891 — STM32H74x/75x system architecture & performance (the definitive memory-placement guidance): <https://www.st.com/resource/en/application_note/an4891-stm32h72x-stm32h73x-and-singlecore-stm32h74x75x-system-architecture-and-performance-stmicroelectronics.pdf>
- STM32H750VB product page (CoreMark/DMIPS, L1 cache): <https://www.st.com/en/microcontrollers-microprocessors/stm32h750-value-line.html>
- Electrosmith Daisy Seed hardware docs: <https://docs.daisy.audio/hardware/Seed/>
- libDaisy `daisy_seed.h` (board-revision / codec enum): <https://github.com/electro-smith/libDaisy/blob/master/src/daisy_seed.h>
