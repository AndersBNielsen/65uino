#!/usr/bin/env python3
import math
import sys
from pathlib import Path

FS = 27778.0
N = 64
# Bin set used in ROM (approx 1 kHz steps at FS≈27778)
BINS = [2, 5, 7, 9, 12, 14, 16, 18]

# Q0.7 coefficients (signed int8): round(cos(2*pi*k/64)*128)
# Correct values for k in BINS
COEFF_Q07 = {2: 126, 5: 113, 7: 99, 9: 81, 12: 49, 14: 25, 16: 0, 18: -25}
SIN_Q07 = {2: 25, 5: 60, 7: 81, 9: 100, 12: 118, 14: 126, 16: 127, 18: 126}

# Q0.6 coefficients (signed int8): round(cos(2*pi*k/64)*64)
COEFF_Q06 = {k: int(round((v/128.0)*64.0)) for k, v in COEFF_Q07.items()}
SIN_Q06 = {k: int(round((v/128.0)*64.0)) for k, v in SIN_Q07.items()}

# Q1.14 coefficients: cos(w) scaled by 2^14 (for higher precision)
COEFF_Q14 = {}
SIN_Q14 = {}
for k in BINS:
    w = 2.0 * math.pi * k / N
    COEFF_Q14[k] = int(round(math.cos(w) * (1 << 14)))
    SIN_Q14[k] = int(round(math.sin(w) * (1 << 14)))

def i8(v):
    """Convert to signed 8-bit integer with two's complement semantics."""
    v &= 0xFF
    return v - 256 if v & 0x80 else v

def i16(v):
    """Convert to signed 16-bit integer with two's complement semantics."""
    v &= 0xFFFF
    return v - 65536 if v & 0x8000 else v

def mul8s(a, b):
    """Signed 8x8 -> 16 product (Python int), result masked to 16-bit."""
    return (i8(a) * i8(b)) & 0xFFFF

def asr7(val):
    """Arithmetic right shift by 7 on 16-bit value, sign-propagating per step."""
    val &= 0xFFFF
    for _ in range(7):
        if val & 0x8000:
            val = ((val >> 1) | 0x8000)
        else:
            val = (val >> 1)
    return val & 0xFFFF

def asr1_8(v):
    """Arithmetic right shift by 1 for signed 8-bit, return int in [-128,127]."""
    v = i8(v)
    if v & 0x80:
        return i8((v >> 1) | 0x80)
    return i8(v >> 1)

def compute_t(coeff_q07, s_prev):
    """Compute t = (coeff * (s_prev << 1)) >> 7 using split partials (ROM-style)."""
    S2 = (i16(s_prev) << 1) & 0xFFFF
    S2_lo = i8(S2 & 0xFF)
    S2_hi = i8((S2 >> 8) & 0xFF)
    prod0 = mul8s(coeff_q07, S2_lo)
    prod1 = mul8s(coeff_q07, S2_hi)
    tA = asr7(prod0)
    tB = (prod1 << 1) & 0xFFFF
    return (tA + tB) & 0xFFFF

def energy_fixed(coeff_q07, s_prev, s_prev2):
    """Fixed-point energy: E = s_prev^2 + s_prev2^2 - ((2*coeff * s_prev * s_prev2) >> 7)."""
    sp = i16(s_prev)
    sp2 = i16(s_prev2)
    termA = (sp * sp) & 0xFFFFFFFF
    termB = (sp2 * sp2) & 0xFFFFFFFF
    termC = ((2 * i8(coeff_q07)) * sp * sp2) >> 7
    E = termA + termB - termC
    return E if E >= 0 else 0

def energy_fixed16_romlike(coeff_q07, s_prev, s_prev2):
    """ROM-like 16-bit constrained energy using split partials and masking.
    Mirrors td_energy_fixed: masks after multiplies and adds, uses asr7.
    """
    sp = i16(s_prev) & 0xFFFF
    sp2 = i16(s_prev2) & 0xFFFF
    # Signed 8-bit parts (no extra masking to 0xFF which breaks sign)
    sp_lo = i8(sp & 0xFF)
    sp_hi = i8((sp >> 8) & 0xFF)
    sp2_lo = i8(sp2 & 0xFF)
    sp2_hi = i8((sp2 >> 8) & 0xFF)
    # termA = s_prev^2
    tlo = (mul8s(sp_lo, sp_lo)) & 0xFFFF
    thi = (mul8s(sp_hi, sp_hi)) & 0xFFFF
    mix = (mul8s(sp_lo, sp_hi)) & 0xFFFF
    termA = (tlo + thi + ((mix << 1) & 0xFFFF)) & 0xFFFF
    # termB = s_prev2^2
    tlo = (mul8s(sp2_lo, sp2_lo)) & 0xFFFF
    thi = (mul8s(sp2_hi, sp2_hi)) & 0xFFFF
    mix = (mul8s(sp2_lo, sp2_hi)) & 0xFFFF
    termB = (tlo + thi + ((mix << 1) & 0xFFFF)) & 0xFFFF
    # P = s_prev * s_prev2
    P0 = (mul8s(sp_lo, sp2_lo)) & 0xFFFF
    P1 = (mul8s(sp_hi, sp2_lo)) & 0xFFFF
    P2 = (mul8s(sp_lo, sp2_hi)) & 0xFFFF
    P3 = (mul8s(sp_hi, sp2_hi)) & 0xFFFF
    P = (P0 + P1 + P2 + P3) & 0xFFFF
    P = (P << 1) & 0xFFFF
    # termC via split partials
    prod0 = mul8s(coeff_q07, i8(P & 0xFF))
    tA = asr7(prod0)
    prod1 = mul8s(coeff_q07, i8((P >> 8) & 0xFF))
    tB = (prod1 << 1) & 0xFFFF
    termC = (tA + tB) & 0xFFFF
    E = (termA + termB - termC) & 0xFFFF
    return E

def energy_reim_q07_romlike(coeff_q07, sin_q07, s_prev, s_prev2):
    """ROM-like Re/Im energy using Q0.7 cos/sin with split partials and >>7.
    re = s_prev - cos*s_prev2; im = sin*s_prev2; E = re^2 + im^2
    """
    sp = i16(s_prev) & 0xFFFF
    sp2 = i16(s_prev2) & 0xFFFF
    # re = s_prev - (cos * s_prev2) with cos in Q0.7 and >>7 scaling
    sp2_lo = i8(sp2 & 0xFF)
    sp2_hi = i8((sp2 >> 8) & 0xFF)
    prod0 = mul8s(coeff_q07, sp2_lo)
    prod1 = mul8s(coeff_q07, sp2_hi)
    tA = asr7(prod0)
    tB = (prod1 << 1) & 0xFFFF
    cos_sp2 = (tA + tB) & 0xFFFF
    re = (sp - cos_sp2) & 0xFFFF
    # im = (sin * s_prev2) >> 7
    prod0 = mul8s(sin_q07, sp2_lo)
    prod1 = mul8s(sin_q07, sp2_hi)
    tA = asr7(prod0)
    tB = (prod1 << 1) & 0xFFFF
    im = (tA + tB) & 0xFFFF
    # E = re^2 + im^2 (truncate to 16-bit like ROM sums)
    re_lo = i8(re & 0xFF)
    re_hi = i8((re >> 8) & 0xFF)
    im_lo = i8(im & 0xFF)
    im_hi = i8((im >> 8) & 0xFF)
    # re^2
    tlo = (mul8s(re_lo, re_lo)) & 0xFFFF
    thi = (mul8s(re_hi, re_hi)) & 0xFFFF
    mix = (mul8s(re_lo, re_hi)) & 0xFFFF
    termRe = (tlo + thi + ((mix << 1) & 0xFFFF)) & 0xFFFF
    # im^2
    tlo = (mul8s(im_lo, im_lo)) & 0xFFFF
    thi = (mul8s(im_hi, im_hi)) & 0xFFFF
    mix = (mul8s(im_lo, im_hi)) & 0xFFFF
    termIm = (tlo + thi + ((mix << 1) & 0xFFFF)) & 0xFFFF
    return (termRe + termIm) & 0xFFFF

def detect_bins_q07_rom(samples_i):
    """ROM emulation: recurrence with td_compute_t and ROM-like 16-bit energy."""
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            t = compute_t(coeff, s_prev)
            s = i16(x + i16(t) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        sinv = i8(SIN_Q07[k])
        E = energy_reim_q07_romlike(coeff, sinv, s_prev, s_prev2)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_q06_rom(samples_i):
    """Detect using Q0.6 stored coefficients and adjusted shifts (>>6).
    This keeps the ROM-like split-partial approach but uses one fewer fractional
    bit so multiplies and shifts align differently.
    """
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q06[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            # compute t: coeff*(s_prev<<1) >> 6 (Q0.6)
            # we'll reuse compute_t but adapt by shifting result left 1 (equivalent to >>6)
            # So compute_t(coeff_q07, s_prev) expects Q0.7 coeff; emulate directly here.
            S2 = (i16(s_prev) << 1) & 0xFFFF
            S2_lo = i8(S2 & 0xFF)
            S2_hi = i8((S2 >> 8) & 0xFF)
            prod0 = mul8s(coeff, S2_lo)
            prod1 = mul8s(coeff, S2_hi)
            # arithmetic right shift by 6 instead of 7: do asr7(prod0) then <<1
            tA = (asr7(prod0) << 1) & 0xFFFF
            tB = (prod1 << 2) & 0xFFFF  # prod1<<1 used in Q0.7; shift one more
            t = (tA + tB) & 0xFFFF
            s = i16(x + i16(t) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        # compute energy using ROM-like re/im with Q0.6 sin/cos
        sinv = i8(SIN_Q06[k])
        E = energy_reim_q07_romlike(coeff, sinv, s_prev, s_prev2)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_reim24_end(samples_i):
    """Keep ROM-like 16-bit recurrence, but compute final Re/Im as signed 24-bit
    values (no downshift) and square them. This tests whether widening only
    the end-stage Re/Im precision recovers parity.
    """
    MASK24 = (1 << 24) - 1
    SIGN24 = 1 << 23
    def to_i24(v):
        v &= MASK24
        return v - (1 << 24) if v & SIGN24 else v

    powers = []
    for k in BINS:
        coeff = COEFF_Q07[k]
        sinv = SIN_Q07[k]
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            t = compute_t(i8(coeff), s_prev)
            s = i16(x + i16(t) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        # compute re/im in wider intermediates then fold to 24-bit signed
        re_full = int(i16(s_prev)) - ((int(i8(coeff)) * int(i16(s_prev2))) >> 7)
        im_full = (int(i8(sinv)) * int(i16(s_prev2))) >> 7
        re24 = to_i24(int(re_full))
        im24 = to_i24(int(im_full))
        E = (re24 * re24) + (im24 * im24)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_q14_16bit(samples_i):
    """Detect using Q1.14 cosine/sine and 16-bit sprev/sprev2 states.
    t = (coeff_q14 * (s_prev << 1)) >> 14
    final re = s_prev - (coeff_q14 * s_prev2) >> 14
    im = (sin_q14 * s_prev2) >> 14
    """
    powers = []
    powers_abs = []
    for k in BINS:
        coeff = COEFF_Q14[k]
        sinv = SIN_Q14[k]
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            # compute t using 16-bit sprev
            sp = int(i16(s_prev))
            t = (coeff * (sp << 1)) >> 14
            # update s (16-bit)
            s = i16(int(x) + int(i16(t)) - int(i16(s_prev2)))
            s_prev2 = s_prev
            s_prev = s
        # final re/im using Q1.14 scaled coeffs
        sp = int(i16(s_prev))
        sp2 = int(i16(s_prev2))
        re = sp - ((coeff * sp2) >> 14)
        im = (sinv * sp2) >> 14
        E = (re * re) + (im * im)
        E_abs = abs(int(i16(re))) + abs(int(i16(im)))
        powers.append(E)
        powers_abs.append(E_abs)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers, powers_abs


def detect_bins_q14_16bit_parity(samples_i):
    """Parity checker: compute per-bin re/im and E_abs exactly like ROM.
    Steps:
    - x = (sample - 128) then arithmetic >>1 using sign->carry then ROR emulation
    - t = (coeff_q14 * (s_prev << 1)) >> 14
    - s = x + t - s_prev2 (16-bit signed wrap)
    - re = s_prev - ((coeff_q14 * s_prev2) >> 14)
    - im = (sin_q14 * s_prev2) >> 14
    Returns list of dicts with re_hex, im_hex, re, im, e_abs for bins in BINS order.
    """
    out = []
    for k in BINS:
        coeff = COEFF_Q14[k]
        sinv = SIN_Q14[k]
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            # center, then arithmetic >>1 like 6502: set carry from sign then ROR
            x = (samples_i[n] - 0x80) & 0xFF
            # emulate: asl sample (carry gets sign), then ror original
            sign = (x & 0x80) != 0
            x_ror = x
            if sign:
                # carry=1: ROR with carry produces 1 in MSB
                x_ror = ((x >> 1) | 0x80) & 0xFF
            else:
                x_ror = (x >> 1) & 0xFF
            x_i8 = i8(x_ror)
            # compute t using 16-bit sprev
            sp = int(i16(s_prev))
            t = (coeff * (sp << 1)) >> 14
            # update s
            s = i16(int(x_i8) + int(i16(t)) - int(i16(s_prev2)))
            s_prev2 = s_prev
            s_prev = s
        sp = int(i16(s_prev))
        sp2 = int(i16(s_prev2))
        re = sp - ((coeff * sp2) >> 14)
        im = (sinv * sp2) >> 14
        # format hex like ROM prints (signed 16 => two bytes)
        re_hex = f"{(i16(re) >> 8) & 0xFF:02X}{i16(re) & 0xFF:02X}"
        im_hex = f"{(i16(im) >> 8) & 0xFF:02X}{i16(im) & 0xFF:02X}"
        e_abs = abs(int(i16(re))) + abs(int(i16(im)))
        out.append({
            "bin": k,
            "re": int(i16(re)),
            "im": int(i16(im)),
            "re_hex": re_hex,
            "im_hex": im_hex,
            "e_abs": int(e_abs),
        })
    return out


def detect_bins_clamped16(samples_i, S_MAX=16384, T_MAX=None, R_MAX=None, downshift=None):
    """Emulate a 16-bit recurrence but apply saturating clamps to reduce wrap/truncation.
    - S_MAX: clamp for s (signed) after each sample
    - T_MAX: optional clamp applied to s_prev when computing t
    - R_MAX: optional clamp applied to re/im before squaring
    - downshift: optional right-shift applied to re/im before squaring
    Returns (best_idx, powers) like other detectors.
    """
    def clamp_signed(v, lim):
        if lim is None:
            return v
        if v > lim:
            return lim
        if v < -lim:
            return -lim
        return v

    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        sinv = i8(SIN_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            # choose s_for_t possibly clamped
            s_for_t = i16(s_prev)
            if T_MAX is not None:
                s_for_t = clamp_signed(s_for_t, T_MAX)
            t = compute_t(coeff, s_for_t & 0xFFFF)
            s = i16(x + i16(t) - i16(s_prev2))
            # clamp s before storing
            s = clamp_signed(i16(s), S_MAX) & 0xFFFF
            s_prev2 = s_prev
            s_prev = s
        # final re/im using 32-bit intermediates then optional clamp/downshift
        sp = i16(s_prev)
        sp2 = i16(s_prev2)
        re = int(sp) - ((int(coeff) * int(sp2)) >> 7)
        im = (int(sinv) * int(sp2)) >> 7
        if R_MAX is not None:
            re = clamp_signed(re, R_MAX)
            im = clamp_signed(im, R_MAX)
        if downshift is not None:
            re = int(re) >> downshift
            im = int(im) >> downshift
        E = (re * re) + (im * im)
        powers.append(E & 0xFFFFFFFF)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_hybrid_reim24_acc(samples_i, downshift=4):
    """Hybrid: use ROM-like 16-bit recurrence (compute s each sample),
    but accumulate re/im partials into signed 24-bit accumulators across the
    window. At the end, right-shift by `downshift` then square.
    This tries to combine cheap inner-loop recurrence with wide end-stage
    accumulation like the recur24 approach.
    """
    MASK24 = (1 << 24) - 1
    SIGN24 = 1 << 23
    def to_i24(v):
        v &= MASK24
        return v - (1 << 24) if v & SIGN24 else v

    powers = []
    for k in BINS:
        coeff = COEFF_Q07[k]
        sinv = SIN_Q07[k]
        s_prev = 0
        s_prev2 = 0
        re_acc = 0
        im_acc = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            t = compute_t(i8(coeff), s_prev)
            s = i16(x + i16(t) - i16(s_prev2))
            # accumulate partials in wider space (use Python ints then fold to 24-bit)
            sp = i16(s_prev)
            # re partial = s^2 - sp^2
            re_part = (int(i16(s)) * int(i16(s))) - (int(sp) * int(sp))
            im_part = int(i16(s)) * int(sp)
            re_acc = to_i24(re_acc + (re_part & MASK24))
            im_acc = to_i24(im_acc + (im_part & MASK24))
            s_prev2 = s_prev
            s_prev = s

        # downshift accumulators before squaring
        re_ds = int(re_acc) >> downshift
        im_ds = int(im_acc) >> downshift
        E = (re_ds * re_ds) + (im_ds * im_ds)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers

def energy_from_states(sp, sp2, coeff):
    """Compute fixed-point energy from 16-bit signed states using ROM-like partials."""
    def i8s(v):
        return i8(v)
    def asr7(val):
        val &= 0xFFFF
        for _ in range(7):
            if val & 0x8000:
                val = ((val >> 1) | 0x8000)
            else:
                val = (val >> 1)
        return val & 0xFFFF
    sp = i16(sp)
    sp2 = i16(sp2)
    sp_lo = i8s(sp & 0xFF)
    sp_hi = i8s((sp >> 8) & 0xFF)
    sp2_lo = i8s(sp2 & 0xFF)
    sp2_hi = i8s((sp2 >> 8) & 0xFF)
    # termA
    tlo = (sp_lo * sp_lo) & 0xFFFF
    thi = (sp_hi * sp_hi) & 0xFFFF
    mix = (sp_lo * sp_hi) & 0xFFFF
    termA = (tlo + thi + ((mix << 1) & 0xFFFF)) & 0xFFFF
    # termB
    tlo = (sp2_lo * sp2_lo) & 0xFFFF
    thi = (sp2_hi * sp2_hi) & 0xFFFF
    mix = (sp2_lo * sp2_hi) & 0xFFFF
    termB = (tlo + thi + ((mix << 1) & 0xFFFF)) & 0xFFFF
    # P
    P0 = (sp_lo * sp2_lo) & 0xFFFF
    P1 = (sp_hi * sp2_lo) & 0xFFFF
    P2 = (sp_lo * sp2_hi) & 0xFFFF
    P3 = (sp_hi * sp2_hi) & 0xFFFF
    P = (P0 + P1 + P2 + P3) & 0xFFFF
    P = (P << 1) & 0xFFFF
    prod0 = (coeff * i8s(P & 0xFF)) & 0xFFFF
    prod1 = (coeff * i8s((P >> 8) & 0xFF)) & 0xFFFF
    tA = asr7(prod0)
    tB = (prod1 << 1) & 0xFFFF
    termC = (tA + tB) & 0xFFFF
    E = (termA + termB - termC) & 0xFFFF
    return 0 if E < 0 else E

def detect_bins_reference(samples_i):
    """Accurate Goertzel power using float math for ground truth."""
    powers = []
    for k in BINS:
        w = 2.0 * math.pi * k / N
        c = math.cos(w)
        s_prev = 0.0
        s_prev2 = 0.0
        for n in range(N):
            x = float(i8(samples_i[n] - 0x80))
            s = x + 2.0 * c * s_prev - s_prev2
            s_prev2 = s_prev
            s_prev = s
        # Re/Im form for exact non-negative power
        re_ = s_prev - c * s_prev2
        im_ = s_prev2 * math.sin(w)
        E = re_ * re_ + im_ * im_
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers

def detect_bins_fixed_q07(samples_i):
    """Fixed-point Goertzel using Q0.7 coeffs, ROM-like t-scaling (>>8) and Re/Im energy."""
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)  # match ROM scaling
            t = compute_t(coeff, s_prev)
            s = i16(x + i16(t) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        sinv = i8(SIN_Q07[k])
        E = energy_reim_q07_romlike(coeff, sinv, s_prev, s_prev2)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_variant_xhalved(samples_i):
    """Prototype: halve input `x` before recurrence to reduce dynamic range."""
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            # halve input (arithmetic shift) before feeding
            if x & 0x80:
                x = i8((x >> 1) | 0x80)
            else:
                x = i8(x >> 1)
            x = asr1_8(x)  # ROM scaling still applied
            t = compute_t(coeff, s_prev)
            s = i16(x + i16(t) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        sinv = i8(SIN_Q07[k])
        E = energy_reim_q07_romlike(coeff, sinv, s_prev, s_prev2)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_variant_thalved(samples_i):
    """Prototype: halve feedback `t` to reduce recurrence growth."""
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            t = compute_t(coeff, s_prev)
            # halve t (arithmetic >>1) emulating smaller feedback
            t = asr7(t)  # reuse asr7 to perform sign-preserving >>7 then shift — approximate halving
            t = (t >> 1) & 0xFFFF
            s = i16(x + i16(t) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        sinv = i8(SIN_Q07[k])
        E = energy_reim_q07_romlike(coeff, sinv, s_prev, s_prev2)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_option_b(samples_i):
    """Option B: perform exact arithmetic halving of feedback `t` (signed) each sample
    and compute Re/Im energy using 32-bit intermediates. This mirrors reducing recurrence
    gain while keeping higher-precision energy evaluation.
    """
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        sinv = i8(SIN_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            # compute t in ROM-style
            t = compute_t(coeff, s_prev)
            # treat t as signed 16 and arithmetic halve it (exact >>1)
            t_signed = i16(t)
            t_half = (t_signed >> 1) & 0xFFFF
            s = i16(x + i16(t_half) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        # compute re/im in 32-bit precision
        sp = i16(s_prev)
        sp2 = i16(s_prev2)
        cos_mul = (i8(coeff) * sp2)
        cos_sp2 = (cos_mul >> 7)
        re = sp - cos_sp2
        im = (i8(sinv) * sp2) >> 7
        E = (re * re) + (im * im)
        if E < 0:
            E = 0
        powers.append(E & 0xFFFFFFFF)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_variant_s_scaled(samples_i):
    """Prototype: store s_prev and s_prev2 halved (arithmetic >>1) to save headroom.
    When computing t = coeff * (s_prev<<1) >>7, we must account for the stored scale.
    We'll store s_prev_s = s_prev >> 1, s_prev2_s = s_prev2 >> 1, and adapt t computation accordingly.
    """
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        sinv = i8(SIN_Q07[k])
        # stored scaled states
        s_prev_s = 0
        s_prev2_s = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            # reconstruct s_prev (unscaled) for compute_t: s_prev = s_prev_s << 1
            s_prev = (i16(s_prev_s) << 1) & 0xFFFF
            t = compute_t(coeff, s_prev)
            # compute real s = x + t - s_prev2
            s_unscaled_prev2 = (i16(s_prev2_s) << 1) & 0xFFFF
            s = i16(x + i16(t) - i16(s_unscaled_prev2))
            # store scaled versions
            s_prev2_s = s_prev_s
            # arithmetic >>1 to store
            sval = i16(s)
            s_prev_s = (sval >> 1) & 0xFFFF
        # at end, reconstruct true s_prev and s_prev2
        s_prev_final = i16((s_prev_s << 1) & 0xFFFF)
        s_prev2_final = i16((s_prev2_s << 1) & 0xFFFF)
        # compute energy with 32-bit re/im to be forgiving
        cos_mul = (i8(coeff) * s_prev2_final)
        cos_sp2 = (cos_mul >> 7)
        re = s_prev_final - cos_sp2
        im = (i8(sinv) * s_prev2_final) >> 7
        E = (re * re) + (im * im)
        powers.append(E & 0xFFFFFFFF)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_variant_recur_wide(samples_i):
    """Prototype: keep recurrence in Python using full Python ints (no 16-bit wraps)
    to observe float parity when only energy truncation is the issue.
    """
    powers = []
    for k in BINS:
        coeff = COEFF_Q07[k]
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = (samples_i[n] - 128)
            x = int((x >> 1) if x < 0 else (x >> 1))
            # t compute in full precision: coeff * (s_prev<<1) >> 7
            S2 = (s_prev << 1)
            t = ((int(coeff) * S2) >> 7)
            s = x + t - s_prev2
            s_prev2 = s_prev
            s_prev = s
        # compute high-precision re/im
        w = 2.0 * math.pi * k / N
        cosv = int(round(math.cos(w) * 128))
        sinv = int(round(math.sin(w) * 128))
        re = s_prev - ((cosv * s_prev2) >> 7)
        im = (sinv * s_prev2) >> 7
        E = re * re + im * im
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_recur32(samples_i):
    """Run recurrence and energy using full Python ints (32-bit intermediates).
    This avoids 16-bit masking in the recurrence and computes Re/Im energy
    using 32-bit-style signed arithmetic before truncation.
    """
    powers = []
    for k in BINS:
        coeff = COEFF_Q07[k]
        sinv = SIN_Q07[k]
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            # use full-precision t: coeff * (s_prev<<1) >> 7
            t = (int(coeff) * (int(s_prev) << 1)) >> 7
            s = int(x) + int(t) - int(s_prev2)
            s_prev2 = s_prev
            s_prev = s
        # compute Re/Im using 32-bit intermediates
        re = int(s_prev) - ((int(coeff) * int(s_prev2)) >> 7)
        im = (int(sinv) * int(s_prev2)) >> 7
        E = (re * re) + (im * im)
        powers.append(E & 0xFFFFFFFF)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_recur24_24bit(samples_i):
    """Goertzel recurrence with 24-bit signed wrapping for energy intermediates.

    This keeps the recurrence in 16-bit-like semantics but accumulates Re/Im
    using 24-bit signed wrapping to test intermediate precision effects.
    """
    MASK24 = (1 << 24) - 1
    SIGN24 = 1 << 23

    def to_i24(v):
        v &= MASK24
        return v - (1 << 24) if v & SIGN24 else v

    powers = []
    for k in BINS:
        coeff = COEFF_Q07[k]
        sinv = SIN_Q07[k]
        s_prev = 0
        s_prev2 = 0
        re_acc = 0
        im_acc = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            # compute t in full precision then apply signed semantics
            t = (int(coeff) * (int(s_prev) << 1)) >> 7
            s = int(x) + int(t) - int(s_prev2)
            # partials for re/im (use 32-bit-like partials, fold into 24-bit acc)
            re_part = (int(s) * int(s)) - (int(s_prev) * int(s_prev))
            im_part = int(s) * int(s_prev)
            # accumulate with 24-bit wrapping
            re_acc = to_i24(re_acc + (re_part & MASK24))
            im_acc = to_i24(im_acc + (im_part & MASK24))
            s_prev2 = s_prev
            s_prev = s

        # final energy using expanded 24-bit accumulators
        E = (int(re_acc) * int(re_acc)) + (int(im_acc) * int(im_acc))
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_recur24_halved(samples_i):
    """24-bit recurrence with arithmetic half-window halving of the 24-bit states.

    This tests if combining wider intermediates with a half-window scale keeps
    parity with float while controlling growth.
    """
    MASK24 = (1 << 24) - 1
    SIGN24 = 1 << 23

    def to_i24(v):
        v &= MASK24
        return v - (1 << 24) if v & SIGN24 else v

    powers = []
    for k in BINS:
        coeff = COEFF_Q07[k]
        sinv = SIN_Q07[k]
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            # center sample and half-scale like ROM
            x = (samples_i[n] - 128) >> 1
            # reconstruct signed 24
            sp = to_i24(s_prev)
            # S2 = sp << 1 (within 24-bit wrap)
            S2 = (sp << 1) & MASK24
            t = (int(i8(coeff)) * to_i24(S2)) >> 7
            s = (x + t - to_i24(s_prev2)) & MASK24
            # half-window halving at N//2
            if n == N//2:
                s_prev = (s_prev >> 1) & MASK24
                s_prev2 = (s_prev2 >> 1) & MASK24
            s_prev2 = s_prev
            s_prev = s
        sp = to_i24(s_prev)
        sp2 = to_i24(s_prev2)
        re = sp - ((i8(coeff) * sp2) >> 7)
        im = (i8(sinv) * sp2) >> 7
        E = re * re + im * im
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_recur24_downshift(samples_i, downshift=4):
    """24-bit recurrence but downshift Re/Im by `downshift` bits before squaring.

    This reduces squared growth while keeping 24-bit dynamics for re/im formation.
    """
    MASK24 = (1 << 24) - 1
    SIGN24 = 1 << 23

    def to_i24(v):
        v &= MASK24
        return v - (1 << 24) if v & SIGN24 else v

    powers = []
    for k in BINS:
        coeff = COEFF_Q07[k]
        sinv = SIN_Q07[k]
        s_prev = 0
        s_prev2 = 0
        re_acc = 0
        im_acc = 0
        for n in range(N):
            x = (samples_i[n] - 128) >> 1
            sp = to_i24(s_prev)
            S2 = (sp << 1) & MASK24
            t = (int(i8(coeff)) * to_i24(S2)) >> 7
            s = (x + t - to_i24(s_prev2)) & MASK24
            # accumulate wide partials
            re_part = (int(s) * int(s)) - (int(sp) * int(sp))
            im_part = int(s) * int(sp)
            re_acc = to_i24(re_acc + (re_part & MASK24))
            im_acc = to_i24(im_acc + (im_part & MASK24))
            s_prev2 = s_prev
            s_prev = s

        # downshift re/im before squaring
        re_ds = int(re_acc) >> downshift
        im_ds = int(im_acc) >> downshift
        E = (re_ds * re_ds) + (im_ds * im_ds)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_recur24_best(samples_i, downshift=None, coeff_scale=1.0, half_window=False, signed_round_t=True):
    """Flexible 24-bit recurrence variant.
    Options:
      - downshift: None or integer bits to right-shift final re/im before squaring
      - coeff_scale: multiply stored coefficient by this factor (close to 1.0)
      - half_window: if True, arithmetic >>1 on s_prev/s_prev2 at N//2
      - signed_round_t: apply sign-aware rounding bias on prod0>>7

    This combines the successful ideas (24-bit accumulators, rounding, half-window)
    to search for the best 24-bit-only behavior.
    """
    MASK24 = (1 << 24) - 1
    SIGN24 = 1 << 23

    def to_i24(v):
        v &= MASK24
        return v - (1 << 24) if v & SIGN24 else v

    powers = []
    for k in BINS:
        orig_coeff = COEFF_Q07[k]
        coeff_val = int(round(orig_coeff * coeff_scale))
        coeff = coeff_val
        sinv = SIN_Q07[k]
        s_prev = 0
        s_prev2 = 0
        re_acc = 0
        im_acc = 0
        for n in range(N):
            # center + half-scale like ROM
            x = (samples_i[n] - 128) >> 1
            # reconstruct signed 24 prev
            sp = to_i24(s_prev)
            S2 = (sp << 1) & MASK24
            # compute t in signed 24 then >>7 emulating Q0.7 with rounding
            S2_lo = (S2 & 0xFF) - (256 if (S2 & 0x80) else 0)
            S2_hi = ((S2 >> 8) & 0xFF) - (256 if ((S2 >> 8) & 0x80) else 0)
            prod0 = (int(i8(coeff)) * S2_lo) & MASK24
            prod1 = (int(i8(coeff)) * S2_hi) & MASK24
            # prod0 signed value
            prod0_signed = prod0 - (1 << 24) if (prod0 & SIGN24) else prod0
            if signed_round_t:
                if prod0_signed >= 0:
                    tA = (prod0_signed + 0x40) >> 7
                else:
                    tA = -(((-prod0_signed) + 0x40) >> 7)
            else:
                tA = prod0_signed >> 7
            prod1_signed = prod1 - (1 << 24) if (prod1 & SIGN24) else prod1
            tB = (prod1_signed << 1) & 0xFFFFFF
            t = (tA + tB) & MASK24
            # combine into 24-bit s (with wrap semantics)
            s = (x + t - to_i24(s_prev2)) & MASK24
            # optionally half-window scale
            if half_window and n == N//2:
                s_prev = (s_prev >> 1) & MASK24
                s_prev2 = (s_prev2 >> 1) & MASK24
            # accumulate re/im partials (24-bit)
            re_part = (int(to_i24(s)) * int(to_i24(s))) - (int(sp) * int(sp))
            im_part = int(to_i24(s)) * int(sp)
            re_acc = to_i24(re_acc + (re_part & MASK24))
            im_acc = to_i24(im_acc + (im_part & MASK24))
            s_prev2 = s_prev
            s_prev = s

        # final re/im from accumulators or reconstructed states
        if downshift is None:
            re_final = re_acc
            im_final = im_acc
        else:
            re_final = int(re_acc) >> downshift
            im_final = int(im_acc) >> downshift
        E = (int(re_final) * int(re_final)) + (int(im_final) * int(im_final))
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_recur24_working(samples_i):
    """Convenience wrapper: use the tuned parameters that matched float best in experiments.
    This provides a stable 'working' recurrence to fall back to for comparison.
    """
    # These parameters were found to often match the float reference on the test chunk
    # (downshift=4, coeff_scale=1.0, half_window=False, signed_round_t=True)
    return detect_bins_recur24_best(samples_i, downshift=4, coeff_scale=1.0, half_window=False, signed_round_t=True)


def detect_bins_combo(samples_i, downshift=4, coeff_scale=1.0, adaptive_threshold=None):
    """Combined cheap fixes:
    - optional small coefficient scaling (coeff_scale)
    - signed rounding when shifting prod0>>7 for t compute
    - optional adaptive halving of s used for t when |s_prev|>adaptive_threshold
    - compute re/im in full precision then right-shift by `downshift` before squaring
    """
    powers = []
    for k in BINS:
        orig = COEFF_Q07[k]
        coeff_val = int(round(orig * coeff_scale))
        coeff = i8(coeff_val)
        sinv = i8(SIN_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            # choose s_for_t optionally halved when magnitude exceeds threshold
            if adaptive_threshold is not None and abs(i16(s_prev)) > adaptive_threshold:
                s_for_t = (i16(s_prev) >> 1) & 0xFFFF
            else:
                s_for_t = i16(s_prev) & 0xFFFF
            # compute t with signed rounding on prod0>>7 (ROM-like split partials)
            S2 = (i16(s_for_t) << 1) & 0xFFFF
            S2_lo = i8(S2 & 0xFF)
            S2_hi = i8((S2 >> 8) & 0xFF)
            prod0 = i16(mul8s(coeff, S2_lo))
            # signed rounding bias
            if prod0 >= 0:
                tA = (prod0 + 0x40) >> 7
            else:
                tA = -(((-prod0) + 0x40) >> 7)
            prod1 = i16(mul8s(coeff, S2_hi))
            tB = (prod1 << 1) & 0xFFFF
            t = (tA + tB) & 0xFFFF
            s = i16(x + i16(t) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        # compute re/im in full precision then downshift before squaring
        re = int(i16(s_prev)) - ((int(i8(coeff)) * int(i16(s_prev2))) >> 7)
        im = (int(sinv) * int(i16(s_prev2))) >> 7
        re_ds = int(re) >> downshift
        im_ds = int(im) >> downshift
        E = (re_ds * re_ds) + (im_ds * im_ds)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers





def detect_bins_halved16(samples_i):
    """Run recurrence with 16-bit signed states but apply arithmetic halving
    of s_prev and s_prev2 every sample (or optionally at half-window).

    We'll implement 'halve each sample' mode here to test the effect of
    aggressive scaling on recurrence growth while keeping 16-bit wrap semantics.
    """
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        sinv = i8(SIN_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            t = compute_t(coeff, s_prev)
            s = i16(x + i16(t) - i16(s_prev2))
            # after computing s, store halved versions (arithmetic >>1)
            s_prev2 = i16(s_prev) >> 1
            s_prev = i16(s) >> 1
        # compute energy using ROM-like re/im but with these halved final states
        E = energy_reim_q07_romlike(i8(COEFF_Q07[k]), sinv, s_prev, s_prev2)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_halved16_preserve_t(samples_i):
    """Store s_prev and s_prev2 halved each sample (arithmetic >>1) to limit growth,
    but reconstruct the full-scale values when computing t so the recurrence
    feedback magnitude is preserved. Final energy is computed from the
    reconstructed full-scale states.
    This mirrors the 's_scaled' idea but implements the simple per-sample
    halving policy for easier ROM mapping.
    """
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        sinv = i8(SIN_Q07[k])
        # stored scaled (halved) states
        s_prev_s = 0
        s_prev2_s = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            # reconstruct full-scale s_prev for compute_t: s_prev = s_prev_s << 1
            s_prev_full = (i16(s_prev_s) << 1) & 0xFFFF
            t = compute_t(coeff, s_prev_full)
            # reconstruct full-scale s_prev2 for subtraction
            s_prev2_full = (i16(s_prev2_s) << 1) & 0xFFFF
            s = i16(x + i16(t) - i16(s_prev2_full))
            # store halved versions (arithmetic >>1)
            s_prev2_s = i16(s_prev_s) >> 1
            s_prev_s = i16(s) >> 1
        # reconstruct final full-scale states for energy computation
        s_prev_final = i16((i16(s_prev_s) << 1) & 0xFFFF)
        s_prev2_final = i16((i16(s_prev2_s) << 1) & 0xFFFF)
        E = energy_reim_q07_romlike(i8(COEFF_Q07[k]), sinv, s_prev_final, s_prev2_final)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_halved16_downshift(samples_i, downshift=4):
    """Keep ROM-like 16-bit recurrence (optionally halved storage per-sample),
    but compute Re/Im as signed 16-bit values and right-shift them by
    `downshift` bits before squaring. This is the pure-16-bit-friendly
    attempt to limit squaring width while preserving recurrence semantics.
    """
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        sinv = i8(SIN_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            t = compute_t(coeff, s_prev)
            s = i16(x + i16(t) - i16(s_prev2))
            # store halved to limit growth (same as detect_bins_halved16)
            s_prev2 = i16(s_prev) >> 1
            s_prev = i16(s) >> 1
        # reconstruct signed 16-bit final states for re/im computation
        sp = i16(s_prev)
        sp2 = i16(s_prev2)
        # compute re/im using signed 16-bit multiplies then downshift
        cos_mul = (i8(coeff) * sp2)
        cos_sp2 = (cos_mul >> 7)
        re = sp - cos_sp2
        im = (i8(sinv) * sp2) >> 7
        re_ds = int(re) >> downshift
        im_ds = int(im) >> downshift
        E = (re_ds * re_ds) + (im_ds * im_ds)
        powers.append(E & 0xFFFFFFFF)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_quarter_scaled(samples_i):
    """Variant: halve s_prev and s_prev2 arithmetically at 25%,50%,75% of the window.
    This mirrors applying extra scaling more frequently to keep states small.
    """
    powers = []
    quarters = {int(N*0.25), int(N*0.5), int(N*0.75)}
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        sinv = i8(SIN_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            t = compute_t(coeff, s_prev)
            s = i16(x + i16(t) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
            if n+1 in quarters:
                # arithmetic >>1 on 16-bit signed values
                s_prev = i16((i16(s_prev) >> 1) & 0xFFFF)
                s_prev2 = i16((i16(s_prev2) >> 1) & 0xFFFF)
        E = energy_reim_q07_romlike(coeff, sinv, s_prev, s_prev2)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_variant_recur24(samples_i):
    """Prototype: 24-bit wrapped recurrence. States stored as signed 24-bit.
    Implement wrapping semantics to emulate a 24-bit accumulator in ROM.
    """
    MASK24 = (1 << 24) - 1
    SIGN24 = 1 << 23
    def to_i24(v):
        v &= MASK24
        return v - (1 << 24) if v & SIGN24 else v

    powers = []
    for k in BINS:
        coeff = COEFF_Q07[k]
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = (samples_i[n] - 128)
            x = (x >> 1)  # centering + half-scale like ROM
            # reconstruct s_prev as signed 24
            s_prev_signed = to_i24(s_prev)
            S2 = (s_prev_signed << 1) & MASK24
            # t = (coeff * S2) >> 7 (apply signed mul and signed shift)
            t = (int(i8(coeff)) * to_i24(S2)) >> 7
            # combine with wrap: compute s = x + t - s_prev2 (all in 24-bit wrap)
            s = (x + t - to_i24(s_prev2)) & MASK24
            s_prev2 = s_prev
            s_prev = s
        # final signed 24-bit states
        sp = to_i24(s_prev)
        sp2 = to_i24(s_prev2)
        # compute re/im using wider intermediates
        cosv = i8(COEFF_Q07[k])
        sinv = i8(SIN_Q07[k])
        re = sp - ((cosv * sp2) >> 7)
        im = (sinv * sp2) >> 7
        E = re * re + im * im
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_recur24_romlike(samples_i):
    """Emulate ROM's 24-bit accumulators folded into low 16-bit and 8x8 squaring.
    This mirrors `td_update_recur24`/`td_done_recur24` approximations for direct comparison.
    Returns list of tuples (re_acc_bytes, im_acc_bytes, E_low16) per bin.
    """
    MASK24 = (1 << 24) - 1
    SIGN24 = 1 << 23

    def to_i24(v):
        v &= MASK24
        return v - (1 << 24) if v & SIGN24 else v

    results = []
    for k in BINS:
        coeff = COEFF_Q07[k]
        sinv = SIN_Q07[k]
        s_prev = 0
        s_prev2 = 0
        re_acc0 = 0
        re_acc1 = 0
        re_acc2 = 0
        im_acc0 = 0
        im_acc1 = 0
        im_acc2 = 0
        for n in range(N):
            x = (samples_i[n] - 128) >> 1
            # compute t in full precision (approx like ROM does via compute_t)
            t = compute_t(coeff, s_prev & 0xFFFF)
            s = (x + t - (s_prev2 & 0xFFFFFFFF)) & MASK24
            # partials: low-byte squares
            s0 = s & 0xFF
            sp0 = s_prev & 0xFF
            # s0*s0
            prod = (i8(s0) * i8(s0)) & 0xFFFF
            # accumulate into 24-bit re_acc (only low two bytes used in ROM folding)
            # add to re_acc0/1
            lo = prod & 0xFF
            hi = (prod >> 8) & 0xFF
            re_acc0 = (re_acc0 + lo) & 0xFF
            carry = 1 if re_acc0 >= 0x100 else 0
            re_acc0 &= 0xFF
            re_acc1 = (re_acc1 + hi + carry) & 0xFF
            # subtract sp0*sp0
            prod2 = (i8(sp0) * i8(sp0)) & 0xFFFF
            lo2 = prod2 & 0xFF
            hi2 = (prod2 >> 8) & 0xFF
            # re_acc -= prod2
            v = (re_acc0 | (re_acc1 << 8) | (re_acc2 << 16))
            v = (v - (prod2 & 0xFFFFFF)) & MASK24
            re_acc0 = v & 0xFF
            re_acc1 = (v >> 8) & 0xFF
            re_acc2 = (v >> 16) & 0xFF
            # im_acc += s0*sp0
            prod3 = (i8(s0) * i8(sp0)) & 0xFFFF
            v2 = (im_acc0 | (im_acc1 << 8) | (im_acc2 << 16))
            v2 = (v2 + (prod3 & 0xFFFFFF)) & MASK24
            im_acc0 = v2 & 0xFF
            im_acc1 = (v2 >> 8) & 0xFF
            im_acc2 = (v2 >> 16) & 0xFF
            s_prev2 = s_prev
            s_prev = s

        # fold to 16-bit like td_done_recur24: re_low = re_acc0 + (re_acc1<<8)
        re_low = (re_acc0 | (re_acc1 << 8)) & 0xFFFF
        im_low = (im_acc0 | (im_acc1 << 8)) & 0xFFFF
        # square re_low and im_low using 8x8 muls (approx)
        re_lo = i8(re_low & 0xFF)
        re_hi = i8((re_low >> 8) & 0xFF)
        im_lo = i8(im_low & 0xFF)
        im_hi = i8((im_low >> 8) & 0xFF)
        # re^2
        rprod_lo = mul8s(re_lo, re_lo) & 0xFFFF
        rprod_hi = 0
        # im^2
        iprod_lo = mul8s(im_lo, im_lo) & 0xFFFF
        iprod_hi = 0
        E = (rprod_lo + iprod_lo) & 0xFFFF
        results.append(((re_acc2, re_acc1, re_acc0), (im_acc2, im_acc1, im_acc0), E))
    return results


def detect_bins_variant_coeffscale(samples_i, scale=0.9):
    """Prototype: scale coefficients by `scale` (0..1) to reduce loop gain."""
    powers = []
    for k in BINS:
        orig = COEFF_Q07[k]
        coeff = int(round(orig * scale))
        coeff = i8(coeff)
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            t = compute_t(coeff, s_prev)
            s = i16(x + i16(t) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        sinv = i8(SIN_Q07[k])
        E = energy_reim_q07_romlike(coeff, sinv, s_prev, s_prev2)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_variant_t_rounding(samples_i):
    """Prototype: perform signed rounding when shifting prod0>>7 in t computation."""
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            # compute t with signed rounding on prod0 >>7
            S2 = (i16(s_prev) << 1) & 0xFFFF
            S2_lo = i8(S2 & 0xFF)
            S2_hi = i8((S2 >> 8) & 0xFF)
            prod0 = i16(mul8s(coeff, S2_lo))
            # signed rounding: add sign-aware bias
            if prod0 >= 0:
                tA = (prod0 + 0x40) >> 7
            else:
                tA = -(((-prod0) + 0x40) >> 7)
            prod1 = i16(mul8s(coeff, S2_hi))
            tB = (prod1 << 1) & 0xFFFF
            t = (tA + tB) & 0xFFFF
            s = i16(x + i16(t) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        sinv = i8(SIN_Q07[k])
        E = energy_reim_q07_romlike(coeff, sinv, s_prev, s_prev2)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_variant_adaptive(samples_i, threshold=4096):
    """Prototype: when |s_prev| exceeds threshold, temporarily halve s_prev for t computation only."""
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            s_prev_signed = i16(s_prev)
            if abs(s_prev_signed) > threshold:
                s_for_t = i16(s_prev_signed >> 1)
            else:
                s_for_t = s_prev_signed
            t = compute_t(coeff, s_for_t)
            s = i16(x + i16(t) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        sinv = i8(SIN_Q07[k])
        E = energy_reim_q07_romlike(coeff, sinv, s_prev, s_prev2)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_variant_halfscale(samples_i):
    """Prototype: scale down s_prev and s_prev2 by >>1 after half the window samples.
    This gives extra headroom for the latter half of the window without changing early response.
    """
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            # at halfway, scale stored states down by 1 (arithmetic)
            if n == N//2:
                s_prev = i16(s_prev) >> 1
                s_prev2 = i16(s_prev2) >> 1
            t = compute_t(coeff, s_prev)
            s = i16(x + i16(t) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        sinv = i8(SIN_Q07[k])
        E = energy_reim_q07_romlike(coeff, sinv, s_prev, s_prev2)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_variant_bigenergy(samples_i):
    """Prototype: keep recurrence as ROM but compute energy with full 32-bit intermediates (less truncation)."""
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            t = compute_t(coeff, s_prev)
            s = i16(x + i16(t) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        # compute energy using full precision like energy_fixed
        E = energy_fixed(coeff, i16(s_prev), i16(s_prev2))
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_variant_reim32(samples_i):
    """Prototype: Re/Im computed using 32-bit intermediates and proper sign expansion before squaring."""
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        sinv = i8(SIN_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            t = compute_t(coeff, s_prev)
            s = i16(x + i16(t) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        # re = s_prev - cos*s_prev2 (use 32-bit signed multiply by Q0.7 then >>7)
        sp = i16(s_prev)
        sp2 = i16(s_prev2)
        # compute cos*s_prev2 using full 32-bit multiply
        cos_mul = (i8(coeff) * sp2)
        cos_sp2 = (cos_mul >> 7)
        re = sp - cos_sp2
        im = (i8(sinv) * sp2) >> 7
        E = (re * re) + (im * im)
        if E < 0:
            E = 0
        powers.append(E & 0xFFFFFFFF)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_t_noshift(samples_i):
    """Compute t = coeff * s_prev >> 7 (no left shift). Tests effect of removing S<<1.

    This reduces feedback by ~2x compared to standard t, but keeps the rest
    of the recurrence the same (s = x + t - s_prev2) using 16-bit states.
    """
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        sinv = i8(SIN_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            # compute t without shifting s_prev left
            t = (int(coeff) * int(i16(s_prev))) >> 7
            t &= 0xFFFF
            s = i16(x + i16(t) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        E = energy_reim_q07_romlike(i8(COEFF_Q07[k]), sinv, s_prev, s_prev2)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers


def detect_bins_t_halved(samples_i):
    """Compute t = ((coeff * (s_prev<<1)) >> 7) then arithmetic halve t before use.

    This preserves the left-shift alignment but reduces feedback by ~2x.
    """
    powers = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        sinv = i8(SIN_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            t = compute_t(coeff, s_prev)
            # arithmetic halve signed t
            t_signed = i16(t)
            t_half = (t_signed >> 1) & 0xFFFF
            s = i16(x + i16(t_half) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        E = energy_reim_q07_romlike(i8(COEFF_Q07[k]), sinv, s_prev, s_prev2)
        powers.append(E)
    max_i = max(range(len(powers)), key=lambda i: powers[i])
    return max_i, powers



def trace_bins_rom(samples_i):
    """Trace final states and ROM-like energy per bin; returns list of dicts."""
    traces = []
    for k in BINS:
        coeff = i8(COEFF_Q07[k])
        s_prev = 0
        s_prev2 = 0
        for n in range(N):
            x = i8(samples_i[n] - 0x80)
            x = asr1_8(x)
            t = compute_t(coeff, s_prev)
            s = i16(x + i16(t) - i16(s_prev2))
            s_prev2 = s_prev
            s_prev = s
        E = energy_fixed16_romlike(coeff, s_prev, s_prev2)
        traces.append({
            'k': k,
            'coeff': coeff,
            's_prev': i16(s_prev),
            's_prev2': i16(s_prev2),
            'energy': E,
        })
    return traces


def emulate_assembly(samples_i):
    """Bit-accurate emulator of the ROM's detect loop per bin.
    Returns traces similar to trace_bins_rom but computed using exact ZP-sized
    signed 8-bit/16-bit semantics, as the 6502 code does.
    """
    def to_u8(v):
        return v & 0xFF
    def to_i8(v):
        v &= 0xFF
        return v - 256 if v & 0x80 else v
    def to_u16(v):
        return v & 0xFFFF
    def to_i16(v):
        v &= 0xFFFF
        return v - 65536 if v & 0x8000 else v

    traces = []
    for k in BINS:
        coeff = COEFF_Q07[k] & 0xFF
        # zero states
        s_prev_lo = 0
        s_prev_hi = 0
        s_prev2_lo = 0
        s_prev2_hi = 0
        for n in range(N):
            # load sample and center
            x = (samples_i[n] - 128) & 0xFF
            # arithmetic right shift x by 1 (as in ROM)
            if x & 0x80:
                x = ((x >> 1) | 0x80) & 0xFF
            else:
                x = (x >> 1) & 0xFF
            # S2 = s_prev << 1 (16-bit)
            S2_lo = to_u8((s_prev_lo << 1) & 0xFF)
            carry = (s_prev_lo >> 7) & 1
            S2_hi = to_u8(((s_prev_hi << 1) | carry) & 0xFF)
            # prod0 = coeff * S2_lo (signed 8x8)
            a = to_i8(coeff)
            b = to_i8(S2_lo)
            prod0 = (a * b) & 0xFFFF
            # arithmetic >>7 on prod0
            val = prod0
            for _ in range(7):
                if val & 0x8000:
                    val = ((val >> 1) | 0x8000) & 0xFFFF
                else:
                    val = (val >> 1) & 0xFFFF
            tA = val
            # prod1 = coeff * S2_hi
            a = to_i8(coeff)
            b = to_i8(S2_hi)
            prod1 = (a * b) & 0xFFFF
            # tB = prod1 << 1
            tB = ((prod1 << 1) & 0xFFFF)
            # t = tA + tB (16-bit)
            t = (tA + tB) & 0xFFFF
            # s = x + t - s_prev2  (all aligned to 16-bit signed)
            # expand x to 16-bit signed
            x16 = to_i16(x)
            t16 = to_i16(t)
            s_prev2_16 = to_i16((s_prev2_hi << 8) | s_prev2_lo)
            s = (x16 + t16 - s_prev2_16) & 0xFFFF
            s_lo = s & 0xFF
            s_hi = (s >> 8) & 0xFF
            # rotate states: s_prev2 = s_prev; s_prev = s
            s_prev2_lo, s_prev2_hi = s_prev_lo, s_prev_hi
            s_prev_lo, s_prev_hi = s_lo, s_hi
        # final signed 16-bit states
        s_prev = to_i16((s_prev_hi << 8) | s_prev_lo)
        s_prev2 = to_i16((s_prev2_hi << 8) | s_prev2_lo)
        # compute ROM-like Re/Im energy using 16-bit constrained ops
        sinv = SIN_Q07[k]
        E = energy_reim_q07_romlike(i8(coeff), i8(sinv), s_prev, s_prev2)
        traces.append({'k':k, 's_prev':s_prev, 's_prev2':s_prev2, 'energy':E})
    return traces


def emulate_rom_combined(samples_i):
    """Emulate current ROM behaviour: bit-accurate recurrence + half-window scaling
    + 32-bit Re/Im energy (td_energy_reim32 semantics). Returns per-bin traces.
    """
    def to_i16(v):
        v &= 0xFFFF
        return v - 65536 if v & 0x8000 else v
    def to_u8(v):
        return v & 0xFF

    traces = []
    for k in BINS:
        coeff = COEFF_Q07[k] & 0xFF
        sinv = SIN_Q07[k]
        s_prev_lo = 0
        s_prev_hi = 0
        s_prev2_lo = 0
        s_prev2_hi = 0
        for n in range(N):
            # sample
            raw = samples_i[n]
            x = (raw - 128) & 0xFF
            # arithmetic >>1 on 8-bit
            if x & 0x80:
                x = ((x >> 1) | 0x80) & 0xFF
            else:
                x = (x >> 1) & 0xFF
            # half-window scaling at n == N//2
            if n == N//2:
                # arithmetic >>1 on s_prev
                lo = s_prev_lo
                hi = s_prev_hi
                # shift right 16-bit arith by 1
                combined = (hi << 8) | lo
                combined = to_i16(combined) >> 1
                s_prev_lo = combined & 0xFF
                s_prev_hi = (combined >> 8) & 0xFF
                # same for s_prev2
                lo = s_prev2_lo
                hi = s_prev2_hi
                combined = (hi << 8) | lo
                combined = to_i16(combined) >> 1
                s_prev2_lo = combined & 0xFF
                s_prev2_hi = (combined >> 8) & 0xFF
            # compute t using split partials (signed 8x8)
            # S2 = s_prev << 1
            S2_lo = (s_prev_lo << 1) & 0xFF
            carry = (s_prev_lo >> 7) & 1
            S2_hi = ((s_prev_hi << 1) | carry) & 0xFF
            a = i8(coeff)
            b0 = i8(S2_lo)
            prod0 = (a * b0) & 0xFFFF
            # arithmetic >>7 on prod0
            val = prod0
            for _ in range(7):
                if val & 0x8000:
                    val = ((val >> 1) | 0x8000) & 0xFFFF
                else:
                    val = (val >> 1) & 0xFFFF
            tA = val
            b1 = i8(S2_hi)
            prod1 = (a * b1) & 0xFFFF
            tB = ((prod1 << 1) & 0xFFFF)
            t = (tA + tB) & 0xFFFF
            # s = x + t - s_prev2 (16-bit signed)
            x16 = to_i16(x)
            t16 = to_i16(t)
            sp2 = to_i16((s_prev2_hi << 8) | s_prev2_lo)
            s = (x16 + t16 - sp2) & 0xFFFF
            s_lo = s & 0xFF
            s_hi = (s >> 8) & 0xFF
            s_prev2_lo, s_prev2_hi = s_prev_lo, s_prev_hi
            s_prev_lo, s_prev_hi = s_lo, s_hi
        # reconstruct signed 16-bit states
        sp = to_i16((s_prev_hi << 8) | s_prev_lo)
        sp2 = to_i16((s_prev2_hi << 8) | s_prev2_lo)
        # compute 32-bit re = sp - (coeff*sp2 >>7)
        cos_mul = i8(coeff) * sp2
        cos_sp2 = int(cos_mul >> 7)
        re = sp - cos_sp2
        im = (i8(sinv) * sp2) >> 7
        E = (re * re) + (im * im)
        traces.append({'k': k, 's_prev': sp, 's_prev2': sp2, 'energy': E & 0xFFFFFFFF})
    return traces


def micro_trace_bin(samples_i, k):
    """Produce per-sample micro-trace for bin k comparing float and bit-accurate ROM steps.
    Returns a list of dicts with fields: n, x, x_shr, S2_lo, S2_hi, prod0, prod1, tA, tB, t, s_prev, s_prev2, s_prev_f, s_prev2_f
    """
    coeff = COEFF_Q07[k] & 0xFF
    sinv = SIN_Q07[k]
    # asm-like state (bytes)
    s_prev_lo = 0
    s_prev_hi = 0
    s_prev2_lo = 0
    s_prev2_hi = 0
    # float reference states
    s_prev_f = 0.0
    s_prev2_f = 0.0
    out = []
    for n in range(N):
        raw = samples_i[n]
        x_u8 = raw & 0xFF
        # centered
        x_center = (x_u8 - 128) & 0xFF
        # asm arithmetic >>1 on 8-bit
        if x_center & 0x80:
            x_shr = ((x_center >> 1) | 0x80) & 0xFF
        else:
            x_shr = (x_center >> 1) & 0xFF
        # S2 = s_prev << 1
        S2_lo = (s_prev_lo << 1) & 0xFF
        carry = (s_prev_lo >> 7) & 1
        S2_hi = ((s_prev_hi << 1) | carry) & 0xFF
        # signed 8x8 partials
        a = (coeff - 256) if (coeff & 0x80) else coeff
        b0 = (S2_lo - 256) if (S2_lo & 0x80) else S2_lo
        b1 = (S2_hi - 256) if (S2_hi & 0x80) else S2_hi
        prod0 = (a * b0) & 0xFFFF
        prod1 = (a * b1) & 0xFFFF
        # asr7 prod0
        val = prod0
        for _ in range(7):
            if val & 0x8000:
                val = ((val >> 1) | 0x8000) & 0xFFFF
            else:
                val = (val >> 1) & 0xFFFF
        tA = val
        tB = (prod1 << 1) & 0xFFFF
        t = (tA + tB) & 0xFFFF
        # s = x + t - s_prev2
        x16 = (x_shr - 256) if (x_shr & 0x80) else x_shr
        # make 16-bit signed for s_prev2
        s_prev2_16 = (((s_prev2_hi << 8) | s_prev2_lo) & 0xFFFF)
        s_prev2_16 = s_prev2_16 - 65536 if (s_prev2_16 & 0x8000) else s_prev2_16
        t16 = t - 65536 if (t & 0x8000) else t
        s_val = (x16 + t16 - s_prev2_16) & 0xFFFF
        s_lo = s_val & 0xFF
        s_hi = (s_val >> 8) & 0xFF
        # rotate
        s_prev2_lo, s_prev2_hi = s_prev_lo, s_prev_hi
        s_prev_lo, s_prev_hi = s_lo, s_hi

        # float update
        w = 2.0 * math.pi * k / N
        c = math.cos(w)
        x_f = float((raw - 128))
        s_f = x_f + 2.0 * c * s_prev_f - s_prev2_f
        s_prev2_f = s_prev_f
        s_prev_f = s_f

        out.append({
            'n': n,
            'x_u8': x_u8,
            'x_shr': x_shr,
            'S2_lo': S2_lo,
            'S2_hi': S2_hi,
            'prod0': prod0,
            'prod1': prod1,
            'tA': tA,
            'tB': tB,
            't': t,
            's_prev': ((s_prev_hi << 8) | s_prev_lo),
            's_prev2': ((s_prev2_hi << 8) | s_prev2_lo),
            's_prev_f': s_prev_f,
            's_prev2_f': s_prev2_f,
        })
    return out


def compare_emus(samples_i, k):
    """Compare micro-trace from bit-accurate emulator with trace_bins_rom step outputs.
    Returns tuple (match, mismatch_info). If mismatch found, mismatch_info contains
    the sample index and both records.
    """
    asm_trace = micro_trace_bin(samples_i, k)
    # generate rom-style micro steps using the compute_t path for each sample
    # build rom-like sequence of s_prev/s_prev2 and t values
    coeff = i8(COEFF_Q07[k])
    s_prev = 0
    s_prev2 = 0
    rom_seq = []
    for n in range(N):
        raw = samples_i[n]
        x = i8(raw - 0x80)
        x = asr1_8(x)
        t = compute_t(coeff, s_prev)
        s = i16(x + i16(t) - i16(s_prev2))
        rom_seq.append({'n': n, 'x': x & 0xFF, 't': t & 0xFFFF, 's_prev': i16(s), 's_prev2': i16(s_prev2)})
        s_prev2 = s_prev
        s_prev = s

    # compare
    for a, r in zip(asm_trace, rom_seq):
        # compare raw 16-bit words (unsigned) to avoid signed/unsigned false positives
        asm_t_raw = a['t'] & 0xFFFF
        asm_s_prev_raw = a['s_prev'] & 0xFFFF
        rom_t_raw = r['t'] & 0xFFFF
        rom_s_prev_raw = r['s_prev'] & 0xFFFF
        if asm_t_raw != rom_t_raw or asm_s_prev_raw != rom_s_prev_raw:
            # include both raw and signed views for debugging
            def signed16(v):
                return v - 65536 if v & 0x8000 else v
            return False, {
                'n': r['n'],
                'asm': a,
                'rom': r,
                'asm_s_prev_raw': asm_s_prev_raw,
                'rom_s_prev_raw': rom_s_prev_raw,
                'asm_s_prev_signed': signed16(asm_s_prev_raw),
                'rom_s_prev_signed': signed16(rom_s_prev_raw),
                'asm_t_raw': asm_t_raw,
                'rom_t_raw': rom_t_raw,
                'asm_t_signed': signed16(asm_t_raw),
                'rom_t_signed': signed16(rom_t_raw),
            }
    return True, None

def mul8_signed(a, b):
    a = i8(a)
    b = i8(b)
    return (a * b) & 0xFFFF


def detect_bins_over_windows(samples_i, window_len=N, step=N):
    best = None
    best_info = (0, 0, [])
    for start in range(0, max(0, len(samples_i) - window_len + 1), step):
        window = samples_i[start:start+window_len]
        idx, powers = detect_bins_q07_rom(window)
        peak = powers[idx]
        if best is None or peak > best:
            best = peak
            best_info = (start, idx, powers)
    return best_info

def gen_tone(freq, n=N, phase=0.0):
    return [math.sin(2*math.pi*freq/FS * i + phase) for i in range(n)]

def read_chunk(path):
    data = Path(path).read_bytes()
    # I-only: take every other byte raw 0..255, leave centering to detector
    samples_i = []
    for i in range(0, min(len(data), 2*N), 2):
        samples_i.append(data[i])
    return samples_i

def main():
    default_path = Path('received_chunk_0.bin')
    # Decide mode: explicit .bin path, explicit frequency, or default chunk if present
    if len(sys.argv) > 1 and sys.argv[1].endswith('.bin'):
        path = Path(sys.argv[1])
        samples_i = read_chunk(path)
        idx_rom, powers_rom, powers_rom_abs = detect_bins_q14_16bit(samples_i[:N])
        idx_ref, powers_ref = detect_bins_reference(samples_i[:N])
        print(f"File: {path}")
        print("ROM emu (Q1.14 16-bit):")
        for i, k in enumerate(BINS):
            fk = k * FS / N
            print(f"k={k:2d} (~{fk/1000:.2f} kHz): power_sq={powers_rom[i]:.3f} power_abs={powers_rom_abs[i]:.3f}")
        print(f"ROM best bin = {BINS[idx_rom]} (~{BINS[idx_rom]*FS/N/1000:.2f} kHz)")
        print("Reference (float):")
        for i, k in enumerate(BINS):
            fk = k * FS / N
            print(f"k={k:2d} (~{fk/1000:.2f} kHz): power={powers_ref[i]:.3f}")
        print(f"Ref best bin = {BINS[idx_ref]} (~{BINS[idx_ref]*FS/N/1000:.2f} kHz)")
        start, idx_b, powers_b = detect_bins_over_windows(samples_i)
        if powers_b:
            fk_b = BINS[idx_b] * FS / N
            print(f"Best window start={start}, max bin={BINS[idx_b]} (~{fk_b/1000:.2f} kHz)")
    elif len(sys.argv) > 1:
        # Treat argument as frequency (Hz)
        freq = float(sys.argv[1])
        samples = gen_tone(freq)
        samples_i = []
        for x in samples:
            val = int(round(128 + 60 * x))
            samples_i.append(max(0, min(255, val)))
        idx_rom, powers_rom, powers_rom_abs = detect_bins_q14_16bit(samples_i[:N])
        idx_ref, powers_ref = detect_bins_reference(samples_i[:N])
        print(f"Synthetic tone {freq} Hz")
        print("ROM emu (Q1.14 16-bit):")
        for i, k in enumerate(BINS):
            fk = k * FS / N
            print(f"k={k:2d} (~{fk/1000:.2f} kHz): power_sq={powers_rom[i]:.3f} power_abs={powers_rom_abs[i]:.3f}")
        print(f"ROM best bin = {BINS[idx_rom]} (~{BINS[idx_rom]*FS/N/1000:.2f} kHz)")
        print("Reference (float):")
        for i, k in enumerate(BINS):
            fk = k * FS / N
            print(f"k={k:2d} (~{fk/1000:.2f} kHz): power={powers_ref[i]:.3f}")
        print(f"Ref best bin = {BINS[idx_ref]} (~{BINS[idx_ref]*FS/N/1000:.2f} kHz)")
    elif default_path.exists():
        path = default_path
        samples_i = read_chunk(path)
        idx_rom, powers_rom, powers_rom_abs = detect_bins_q14_16bit(samples_i[:N])
        idx_ref, powers_ref = detect_bins_reference(samples_i[:N])
        print(f"File: {path}")
        print("ROM emu (Q1.14 16-bit):")
        for i, k in enumerate(BINS):
            fk = k * FS / N
            print(f"k={k:2d} (~{fk/1000:.2f} kHz): power_sq={powers_rom[i]:.3f} power_abs={powers_rom_abs[i]:.3f}")
        print(f"ROM best bin = {BINS[idx_rom]} (~{BINS[idx_rom]*FS/N/1000:.2f} kHz)")
        print("Reference (float):")
        for i, k in enumerate(BINS):
            fk = k * FS / N
            print(f"k={k:2d} (~{fk/1000:.2f} kHz): power={powers_ref[i]:.3f}")
        print(f"Ref best bin = {BINS[idx_ref]} (~{BINS[idx_ref]*FS/N/1000:.2f} kHz)")
        start, idx_b, powers_b = detect_bins_over_windows(samples_i)
        if powers_b:
            fk_b = BINS[idx_b] * FS / N
            print(f"Best window start={start}, max bin={BINS[idx_b]} (~{fk_b/1000:.2f} kHz)")
    else:
        # Default synthetic run at 3800 Hz
        freq = 3800.0
        samples = gen_tone(freq)
        samples_i = []
        for x in samples:
            val = int(round(128 + 60 * x))
            samples_i.append(max(0, min(255, val)))
        # Use the same estimator as in sdr.s: Q1.14 coeffs with 16-bit sprev recurrence
        idx_rom, powers_rom, powers_rom_abs = detect_bins_q14_16bit(samples_i[:N])
        idx_ref, powers_ref = detect_bins_reference(samples_i[:N])
        print(f"Synthetic tone {freq} Hz (default)")
        print("ROM emu (Q1.14 16-bit):")
        for i, k in enumerate(BINS):
            fk = k * FS / N
            print(f"k={k:2d} (~{fk/1000:.2f} kHz): power_sq={powers_rom[i]:.3f} power_abs={powers_rom_abs[i]:.3f}")
        print(f"ROM best bin = {BINS[idx_rom]} (~{BINS[idx_rom]*FS/N/1000:.2f} kHz)")
        print("Reference (float):")
        for i, k in enumerate(BINS):
            fk = k * FS / N
            print(f"k={k:2d} (~{fk/1000:.2f} kHz): power={powers_ref[i]:.3f}")
        print(f"Ref best bin = {BINS[idx_ref]} (~{BINS[idx_ref]*FS/N/1000:.2f} kHz)")

if __name__ == "__main__":
    main()
