// axisram.h - place a plain global in AXI SRAM (device only).
//
// Tags a variable for our linker script's .axisram_bss section (dreamosc.lds):
// D1 AXI SRAM, fast and L1-cacheable, the region ST designates for large hot
// working sets that outgrow DTCM -- and, under APP_TYPE=BOOT_SRAM, the ONLY
// on-chip RAM the SDMMC's IDMA can reach: .bss/.data/stack are DTCM at
// 0x20000000, which no DMA master sees (libDaisy #508). So both the FFT
// scratch (hot) and every FatFs DMA target (FIL, FATFS, the staging buffer)
// live here.
//
// NOLOAD: the section is neither zeroed nor constructed by the C runtime.
// Only plain data goes here, memset or placement-new'd by the code that
// owns it before first use -- and each owner says why that is safe.

#ifndef AXISRAM_H
#define AXISRAM_H

#define AXISRAM_DATA __attribute__((section(".axisram_bss")))

#endif  // AXISRAM_H
