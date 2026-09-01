#!/usr/bin/env python3
"""Convert a WAV to the raw mono int16 blob the firmware reads from QSPI.

Output layout (little-endian), so the firmware needs no parser:
    uint32  magic  'DRMO' (0x4F4D5244)
    uint32  sample_count
    uint32  sample_rate
    uint32  reserved (0)
    int16   samples[sample_count]

Stereo is folded to mono; the rate is recorded but NOT resampled -- the firmware
reads the header and scales playback accordingly.
"""
import struct, sys, wave

def main():
    if len(sys.argv) != 3:
        print("usage: wav2raw.py in.wav out.bin", file=sys.stderr); return 2
    src, dst = sys.argv[1], sys.argv[2]
    with wave.open(src, 'rb') as w:
        if w.getsampwidth() != 2:
            print("only 16-bit PCM WAV supported", file=sys.stderr); return 1
        ch, rate, n = w.getnchannels(), w.getframerate(), w.getnframes()
        raw = w.readframes(n)
    vals = struct.unpack('<%dh' % (len(raw)//2), raw)
    if ch > 1:
        mono = [int(sum(vals[i*ch:(i+1)*ch]) / ch) for i in range(len(vals)//ch)]
    else:
        mono = list(vals)
    with open(dst, 'wb') as f:
        f.write(struct.pack('<IIII', 0x4F4D5244, len(mono), rate, 0))
        f.write(struct.pack('<%dh' % len(mono), *mono))
    print(f"{src}: {ch}ch {rate}Hz {n} frames -> {dst}: {len(mono)} mono samples, "
          f"{16 + len(mono)*2} bytes ({(16+len(mono)*2)/1024:.1f} KB)")
    return 0

sys.exit(main())
