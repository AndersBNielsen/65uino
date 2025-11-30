#!/usr/bin/env python3
"""
Generate an interleaved unsigned 8-bit IQ binary file containing a single complex tone.

Usage:
  python3 tools/gen_iq_tone.py --sample-rate 1000000 --freq 10000 --seconds 5 --outfile iq_1msps_10khz_5s.bin

Format:
  Output file consists of 2*N bytes (I,Q) for N = sample_rate * seconds.
  I = cos(2*pi*f*t), Q = sin(2*pi*f*t) scaled to unsigned 8-bit.

Scaling:
  Value v in [-1,1] mapped to byte: round(127.5 + 127.5 * amp * v) with clamp to [0,255].
  Use --amplitude to set amp (default 0.95 to avoid edge clipping).

Performance:
  Streams in chunks to avoid high memory usage for large durations.
"""
from __future__ import annotations
import math, argparse, sys, os

def gen_chunk(start_idx: int, count: int, freq: float, fs: float, amp: float) -> bytes:
    out = bytearray()
    two_pi_f_div_fs = 2.0 * math.pi * freq / fs
    for n in range(start_idx, start_idx + count):
        phase = two_pi_f_div_fs * n
        i_val = math.cos(phase)
        q_val = math.sin(phase)
        i_b = int(round(127.5 + 127.5 * amp * i_val))
        q_b = int(round(127.5 + 127.5 * amp * q_val))
        if i_b < 0: i_b = 0
        elif i_b > 255: i_b = 255
        if q_b < 0: q_b = 0
        elif q_b > 255: q_b = 255
        out.append(i_b & 0xFF)
        out.append(q_b & 0xFF)
    return bytes(out)

def main(argv):
    ap = argparse.ArgumentParser(description="Generate unsigned 8-bit IQ tone file")
    ap.add_argument('--sample-rate', type=float, required=True, help='Samples per second (Hz)')
    ap.add_argument('--freq', type=float, required=True, help='Tone frequency (Hz)')
    ap.add_argument('--seconds', type=float, required=True, help='Duration in seconds')
    ap.add_argument('--outfile', required=True, help='Output filename')
    ap.add_argument('--amplitude', type=float, default=0.95, help='Amplitude scale (0..1, default 0.95)')
    ap.add_argument('--chunk', type=int, default=200000, help='Samples per chunk (IQ pairs) streaming size')
    args = ap.parse_args(argv)

    fs = args.sample_rate
    f = args.freq
    duration = args.seconds
    amp = args.amplitude
    if amp <= 0 or amp > 1.2:
        print('Amplitude should be (0,1.2]', file=sys.stderr); return 1
    total_pairs = int(round(fs * duration))
    if total_pairs <= 0:
        print('Total samples <= 0', file=sys.stderr); return 1

    # Warn if frequency is above Nyquist
    if f >= fs / 2:
        print('Warning: freq >= Nyquist; tone will alias', file=sys.stderr)

    # Try to avoid accidental extremely large files
    total_bytes = total_pairs * 2
    if total_bytes > 500_000_000:  # 500 MB safety
        print('Refusing to create file larger than 500MB', file=sys.stderr); return 1

    print(f'Generating {total_pairs} IQ pairs ({total_bytes/1_000_000:.2f} MB) tone {f} Hz @ {fs} Hz, amp={amp}')
    # Stream to file
    try:
        with open(args.outfile, 'wb') as fo:
            written = 0
            start = 0
            while start < total_pairs:
                n_chunk = min(args.chunk, total_pairs - start)
                buf = gen_chunk(start, n_chunk, f, fs, amp)
                fo.write(buf)
                start += n_chunk
                written += n_chunk
                if written % (args.chunk * 10) == 0:
                    print(f'  Progress: {written}/{total_pairs} pairs ({written/total_pairs*100:.1f}%)')
    except OSError as e:
        print(f'File write error: {e}', file=sys.stderr); return 1

    print(f'Finished: {args.outfile}')
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))