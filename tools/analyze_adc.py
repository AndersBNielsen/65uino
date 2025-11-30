#!/usr/bin/env python3
"""
Analyze interleaved 8-bit ADC samples (binary) and output per-channel statistics plus an FFT summary (peak frequencies).

Input format:
    Binary file of unsigned 8-bit samples. If `--iq` is used, input is interleaved I,Q bytes.

Interleaving:
    For N channels the byte order is:
        ch0_sample0 ch1_sample0 ... ch(N-1)_sample0 ch0_sample1 ch1_sample1 ...

CLI Usage:
    analyze_adc.py --file iq.bin --format bin --iq --sample-rate 48000 --peaks 10

Options:
    --file           Path to input file (if omitted, read stdin for ASCII)
    --channels       Number of interleaved channels (binary)
    --iq             Treat binary input as interleaved unsigned 8-bit I,Q pairs (overrides --channels=2)
    --sample-rate    Sample rate in Hz (for FFT frequency axis; default 1.0)
    --max-samples    Cap total samples (per channel) for analysis (truncate)
    --offset-mode    How to treat DC offset before FFT: raw|center (default center)
    --window         Optional window function: none|hann (default none)
    --peaks          Number of spectral peaks to report (default 5)
    --csv            Output peaks in CSV format (frequency,amplitude) after text summary
    --cf32-out       Write complex float32 (real,imag) interleaved samples (IQ) to this binary file
    --cf32-scale     Scaling strategy for cf32: unit|signed (default unit)

FFT Implementation:
  Uses numpy. If numpy is unavailable, falls back to a naive O(N^2) DFT (slower).

Output:
  Per channel stats (count, min, max, mean, rms, std, DC offset)
  Peak list: frequency (Hz) and amplitude (linear magnitude)

Limitations:
  Naive DFT fallback may be slow for large N; use --max-samples to cap size.
"""
from __future__ import annotations
import sys, math, argparse
from typing import List, Tuple
import struct

try:
    import numpy as np
except ImportError:  # noqa: E402
    np = None


# ASCII hex parsing removed — analyzer now expects binary input files only


def split_channels(raw: List[int], channels: int) -> List[List[int]]:
    if channels <= 1:
        return [raw]
    out = [[] for _ in range(channels)]
    for idx, val in enumerate(raw):
        ch = idx % channels
        out[ch].append(val)
    return out


def apply_window(data: List[float], name: str) -> List[float]:
    n = len(data)
    if name == 'hann':
        return [x * (0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1))) for i, x in enumerate(data)]
    return data


def fft_real(data: List[float]) -> Tuple[List[float], List[float]]:
    """Return (freq_bins_normalized, magnitudes) for real-valued input.
    freq_bins_normalized are in cycles per sample; caller scales by sample_rate.
    """
    n = len(data)
    if n == 0:
        return [], []
    if np is not None:
        arr = np.array(data, dtype=float)
        spec = np.fft.rfft(arr)
        mags = np.abs(spec) / n
        freqs = np.fft.rfftfreq(n, d=1.0)  # frequency in cycles/sample
        return freqs.tolist(), mags.tolist()
    # Naive DFT fallback
    half = n // 2
    mags = []
    freqs = []
    for k in range(half + 1):
        re = 0.0
        im = 0.0
        for t, x in enumerate(data):
            angle = 2 * math.pi * k * t / n
            re += x * math.cos(angle)
            im -= x * math.sin(angle)
        mag = math.sqrt(re * re + im * im) / n
        mags.append(mag)
        freqs.append(k / n)
    return freqs, mags


def fft_complex(I: List[float], Q: List[float]) -> Tuple[List[float], List[float]]:
    """Compute FFT on complex samples (I + jQ). Returns (freqs_normalized, magnitudes).
    freqs_normalized in cycles/sample.
    """
    n = min(len(I), len(Q))
    if n == 0:
        return [], []
    if np is not None:
        arr = np.array(I[:n], dtype=np.float64) + 1j * np.array(Q[:n], dtype=np.float64)
        spec = np.fft.fft(arr)
        mags = np.abs(spec) / n
        freqs = np.fft.fftfreq(n, d=1.0)
        # Return positive and negative freq bins (full FFT)
        return freqs.tolist(), mags.tolist()
    # naive complex DFT fallback
    freqs = []
    mags = []
    for k in range(n):
        re = 0.0
        im = 0.0
        for t in range(n):
            angle = 2 * math.pi * k * t / n
            c = math.cos(angle)
            s = math.sin(angle)
            x = I[t]
            y = Q[t]
            # complex multiply: (x + j y) * exp(-j angle)
            re += x * c + y * s
            im += -x * s + y * c
        mag = math.sqrt(re * re + im * im) / n
        mags.append(mag)
        freqs.append(k / n if k <= n//2 else (k - n) / n)
    return freqs, mags


def compute_stats(samples: List[int]) -> dict:
    n = len(samples)
    if n == 0:
        return {'count': 0}
    s_min = min(samples)
    s_max = max(samples)
    mean = sum(samples) / n
    rms = math.sqrt(sum((x * x) for x in samples) / n)
    std = math.sqrt(sum((x - mean) ** 2 for x in samples) / n)
    return {
        'count': n,
        'min': s_min,
        'max': s_max,
        'mean': mean,
        'rms': rms,
        'std': std,
        'dc_offset': mean,
    }


def pick_peaks(freqs: List[float], mags: List[float], count: int) -> List[Tuple[float, float]]:
    pairs = list(zip(freqs, mags))
    pairs.sort(key=lambda p: p[1], reverse=True)
    return pairs[:count]


def main(argv: List[str]) -> int:
    ap = argparse.ArgumentParser(description="Analyze interleaved 8-bit ADC hex samples and compute FFT peaks")
    ap.add_argument('--file', required=True, help='Input binary file path')
    ap.add_argument('--channels', type=int, default=1, help='Number of interleaved channels (ignored if --iq)')
    ap.add_argument('--iq', action='store_true', help='Binary input is unsigned 8-bit interleaved I,Q')
    ap.add_argument('--sample-rate', type=float, default=1.0, help='Sample rate (Hz) for frequency axis')
    ap.add_argument('--max-samples', type=int, default=0, help='Max samples per channel (truncate; 0 = no limit)')
    ap.add_argument('--offset-mode', choices=['raw', 'center'], default='center', help='Center subtract mean before FFT (center) or leave raw')
    ap.add_argument('--window', choices=['none', 'hann'], default='none', help='Optional window function')
    ap.add_argument('--peaks', type=int, default=5, help='Number of FFT peaks to show')
    ap.add_argument('--csv', action='store_true', help='Emit CSV of peaks after summary')
    ap.add_argument('--cf32-out', help='Output path for complex float32 IQ samples (requires --iq)')
    ap.add_argument('--cf32-scale', choices=['unit','signed'], default='unit', help='Scale: unit => (x-128)/128, signed => (x-127.5)/127.5')
    ap.add_argument('--iq-diagnose', action='store_true', help='Run IQ diagnostics: print sample pairs, correlation, phase')
    args = ap.parse_args(argv)

    # Read binary input
    if not args.file:
        print("--file required for binary input", file=sys.stderr)
        return 1
    try:
        with open(args.file, 'rb') as fb:
            raw_bytes = fb.read()
    except OSError as e:
        print(f"Error reading binary file: {e}", file=sys.stderr)
        return 1
    raw_vals = list(raw_bytes)

    # Determine channels
    if args.iq:
        if len(raw_vals) < 2:
            print("Not enough data for IQ pairs", file=sys.stderr)
            return 1
        if len(raw_vals) % 2 != 0:
            print("Warning: odd number of bytes; last byte ignored for IQ")
            raw_vals = raw_vals[:-1]
        args.channels = 2  # enforce

    chans = split_channels(raw_vals, args.channels)
    print(f"Parsed {len(raw_vals)} bytes total across {args.channels} channel(s). IQ={'yes' if args.iq else 'no'}")

    # Stats per channel
    for idx, ch in enumerate(chans):
        if args.max_samples and len(ch) > args.max_samples:
            ch = ch[:args.max_samples]
        stats = compute_stats(ch)
        if stats['count'] == 0:
            print(f"Channel {idx}: no samples")
            continue
        print(("Channel {i}: count={count} min={min} max={max} mean={mean:.3f} rms={rms:.3f} std={std:.3f} dc={dc_offset:.3f}")
              .format(i=idx, **stats))

    # FFT and peaks per channel
    for idx, ch in enumerate(chans):
        if args.max_samples and len(ch) > args.max_samples:
            ch = ch[:args.max_samples]
        if not ch:
            continue
        # Offset handling
        if args.offset_mode == 'center':
            mean = sum(ch) / len(ch)
            centered = [x - mean for x in ch]
        else:
            centered = ch[:]
        # Window
        windowed = apply_window(centered, args.window)
        freqs_norm, mags = fft_real(windowed)
        if not freqs_norm:
            print(f"Channel {idx}: FFT skipped (no data)")
            continue
        scale = args.sample_rate
        freqs_hz = [f * scale for f in freqs_norm]
        peaks = pick_peaks(freqs_hz, mags, args.peaks)
        print(f"Channel {idx} FFT peaks (freq Hz : amplitude):")
        for f, m in peaks:
            print(f"  {f:.2f} : {m:.5f}")
        if args.csv:
            print(f"channel,{idx},freq_hz,amplitude")
            for f, m in peaks:
                print(f"{idx},{f:.6f},{m:.6f}")

    # If IQ, also run complex FFT and report peaks (handles negative freqs)
    if args.iq and len(chans) >= 2:
        I = chans[0]
        Q = chans[1]
        if args.max_samples and len(I) > args.max_samples:
            I = I[:args.max_samples]
            Q = Q[:args.max_samples]
        # Center
        if args.offset_mode == 'center':
            meanI = sum(I) / len(I)
            meanQ = sum(Q) / len(Q)
            I = [x - meanI for x in I]
            Q = [x - meanQ for x in Q]
        freqs_norm, mags = fft_complex(I, Q)
        if freqs_norm:
            # Convert normalized freq bins to Hz
            freqs_hz = [f * args.sample_rate for f in freqs_norm]
            # pick magnitude peaks
            pairs = list(zip(freqs_hz, mags))
            pairs.sort(key=lambda p: p[1], reverse=True)
            peaks = pairs[:args.peaks]
            print("Complex IQ FFT peaks (freq Hz : amplitude) — negative freqs shown as negative:")
            for f, m in peaks:
                print(f"  {f:.2f} : {m:.5f}")
            if args.csv:
                print("channel,complex,freq_hz,amplitude")
                for f, m in peaks:
                    print(f"complex,{f:.6f},{m:.6f}")
        # IQ diagnostics
        if args.iq_diagnose:
            # print first few pairs
            nprint = min(16, len(I))
            print('\nIQ sample pairs (I,Q) first %d:' % nprint)
            for i in range(nprint):
                print(f"  {i}: {I[i]} , {Q[i]}")
            # compute Pearson correlation between I and Q
            def pearson(x, y):
                n = len(x)
                if n == 0:
                    return 0.0
                mx = sum(x) / n
                my = sum(y) / n
                num = sum((a - mx) * (b - my) for a, b in zip(x, y))
                denx = math.sqrt(sum((a - mx) ** 2 for a in x))
                deny = math.sqrt(sum((b - my) ** 2 for b in y))
                if denx * deny == 0:
                    return 0.0
                return num / (denx * deny)
            corr = pearson(I, Q)
            print(f"IQ Pearson correlation (I vs Q): {corr:.6f}")
            # Compute phase difference at main complex peak
            # Find index of max magnitude
            idx_max = max(range(len(mags)), key=lambda k: mags[k]) if mags else None
            if idx_max is not None:
                if np is not None:
                    arr = np.array(I[:len(I)], dtype=np.float64) + 1j * np.array(Q[:len(Q)], dtype=np.float64)
                    spec = np.fft.fft(arr)
                    ph = np.angle(spec[idx_max])
                    print(f"Phase of main complex bin (radians): {ph:.4f}")
                else:
                    print("Phase diagnostic requires numpy for convenience.")

    # Optional cf32 export for IQ data
    if args.iq and args.cf32_out:
        if np is not None:
            I = np.array(chans[0], dtype=np.float32)
            Q = np.array(chans[1], dtype=np.float32)
            if args.cf32_scale == 'unit':
                I = (I - 128.0) / 128.0
                Q = (Q - 128.0) / 128.0
            else:  # signed
                I = (I - 127.5) / 127.5
                Q = (Q - 127.5) / 127.5
            cf32 = np.empty(I.size * 2, dtype=np.float32)
            cf32[0::2] = I
            cf32[1::2] = Q
            try:
                with open(args.cf32_out, 'wb') as fo:
                    cf32.tofile(fo)
                print(f"Wrote cf32 IQ samples to {args.cf32_out} ({cf32.size * 4} bytes)")
            except OSError as e:
                print(f"Error writing cf32 file: {e}", file=sys.stderr)
        else:
            # Fallback without numpy
            def scale_byte(b: int) -> float:
                if args.cf32_scale == 'unit':
                    return (b - 128.0) / 128.0
                return (b - 127.5) / 127.5
            try:
                with open(args.cf32_out, 'wb') as fo:
                    for i_val, q_val in zip(chans[0], chans[1]):
                        fo.write(struct.pack('<ff', scale_byte(i_val), scale_byte(q_val)))
                total_pairs = min(len(chans[0]), len(chans[1]))
                print(f"Wrote cf32 IQ samples to {args.cf32_out} ({total_pairs * 8} bytes)")
            except OSError as e:
                print(f"Error writing cf32 file: {e}", file=sys.stderr)

    if np is None:
        print("(numpy not found; used naive DFT and naive cf32 writer)")
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
