#!/usr/bin/env python3
"""
int_sqrt_accuracy.py -- Verify integer int_sqrt matches float reference.
The real KCC code uses int_sqrt() for fixed-point sqrt, not float math.
This test compares the integer implementation against the mathematical reference.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

KCC_R_POWER_FRAC = 20
BASE_R = 400
J50 = 200


def int_sqrt(n):
    """Kernel int_sqrt -- integer square root (floor)."""
    if n <= 1:
        return n
    x = n
    y = (x + 1) // 2
    while y < x:
        x = y
        y = (x + n // x) // 2
    return x


def compute_r_int(jitter_excess):
    """Power-law R using integer int_sqrt (matches KCC code exactly)."""
    if jitter_excess <= 0:
        return BASE_R
    ratio = (jitter_excess << KCC_R_POWER_FRAC) // J50  # Q20 fixed-point
    # ratio^(3/2) = ratio * sqrt(ratio) in fixed point
    sqrt_ratio = int_sqrt(ratio << KCC_R_POWER_FRAC)  # sqrt(ratio) * 2^20
    # ratio * sqrt_ratio = ratio^(3/2) * 2^40
    # >> FRAC => ratio^(3/2) * 2^20
    scale = (ratio * sqrt_ratio) >> KCC_R_POWER_FRAC
    r_new = (BASE_R * scale) >> KCC_R_POWER_FRAC
    return max(BASE_R, min(r_new, BASE_R * 256))


def compute_r_float(jitter_excess):
    """Power-law R using float math (reference)."""
    if jitter_excess <= 0:
        return BASE_R
    ratio = jitter_excess / J50
    r = BASE_R * (ratio**1.5)
    return max(BASE_R, min(int(r), BASE_R * 256))


print("=" * 90)
print("INT_SQRT ACCURACY: Integer vs Float R computation")
print(f"  KCC_R_POWER_FRAC={KCC_R_POWER_FRAC}, BASE_R={BASE_R}, J50={J50}")
print("=" * 90)

# Sweep jitter from 1 to 500000us
max_error = 0
max_error_je = 0
errors = []
print(f"\n{'je(us)':>8} {'R_int':>8} {'R_float':>8} {'error':>8} {'err%':>8}")
for je in [
    1,
    10,
    50,
    100,
    200,
    317,
    500,
    800,
    1000,
    2000,
    5000,
    10000,
    50000,
    100000,
    500000,
]:
    ri = compute_r_int(je)
    rf = compute_r_float(je)
    err = abs(ri - rf)
    err_pct = err / rf * 100 if rf > 0 else 0
    if err > max_error:
        max_error = err
        max_error_je = je
    errors.append(err_pct)
    # Only print if error > 0.5%
    mark = " <<<" if err_pct > 0.5 else ""
    print(f"{je:>8} {ri:>8} {rf:>8} {err:>8} {err_pct:>7.2f}%{mark}")

print(f"\nMax error: {max_error} at je={max_error_je}us")
print(f"Mean error: {sum(errors) / len(errors):.2f}%")
print(f"Accuracy: {'PASS' if max_error <= 1 else 'FAIL'} (max error <= 1 unit)")

# Overflow test: verify the ratio*ratio product stays within u64
print("\n--- Overflow guard verification ---")
u64_max = 2**64 - 1
for je in [500, 5000, 50000, 100000, 500000, 1000000]:
    ratio = (je << KCC_R_POWER_FRAC) // J50
    sqrt_ratio = int_sqrt(ratio << KCC_R_POWER_FRAC)
    product = ratio * sqrt_ratio
    overflows = product > u64_max
    status = "OVERFLOW" if overflows else "OK"
    if overflows:
        print(f"  je={je}us: product={product:,} > U64_MAX -- NEEDS GUARD {status}")
    else:
        u64_used = product / u64_max * 100
        print(f"  je={je}us: product={product:,} ({u64_used:.2f}% of U64_MAX) {status}")

print("\nWorst case (J50=1, je=500ms):")
je_max = 500000
ratio_max = (je_max << KCC_R_POWER_FRAC) // 1
sqrt_max = int_sqrt(ratio_max << KCC_R_POWER_FRAC)
prod_max = ratio_max * sqrt_max
print(f"  ratio={ratio_max:,}, sqrt={sqrt_max:,}, product={prod_max:,}")
print(f"  U64_MAX={u64_max:,} ({prod_max / u64_max * 100:.1f}%)")
print(f"  Guard needed: {prod_max > u64_max}")
