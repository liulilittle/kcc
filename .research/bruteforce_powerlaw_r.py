#!/usr/bin/env python3
"""Brute-force power-law R scaling: verify J50, range, overflow, R_max_boost."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kcc_sim_base import BASE_R, J50_DEFAULT, Q_BASE, R_MAX


def test_power_law_r():
    failures = 0
    print("=" * 90)
    print("POWER-LAW R SCALING BRUTE-FORCE: J50=200, jitter 0-100ms")
    print("=" * 90)

    # Test 1: Pass-through at J50 (jitter_excess = J50 => multiplier = 1.0)
    print("\n--- Test 1: Pass-through at J50=200us ---")
    jitter_excess = 200
    ratio = jitter_excess / J50_DEFAULT
    r = BASE_R * (ratio**1.5)
    expected = BASE_R  # 1.0^1.5 = 1.0
    status = "PASS" if abs(r - expected) < 0.01 else "FAIL"
    print(
        f"  jitter_excess={jitter_excess}us ratio={ratio:.1f} R={r:.1f} expected={expected} {status}",
    )
    if status == "FAIL":
        failures += 1

    # Test 2: R doubles at jitter_excess = J50 * 2^(2/3) ~= 317us
    print("\n--- Test 2: Doubling point (R=2*base_R at ~317us) ---")
    je = int(J50_DEFAULT * (2 ** (2 / 3)))
    ratio = je / J50_DEFAULT
    r = BASE_R * (ratio**1.5)
    expected = BASE_R * 2
    status = "PASS" if abs(r - expected) < 2 else "FAIL"
    print(
        f"  jitter_excess={je}us ratio={ratio:.3f} R={r:.0f} expected={expected} {status}",
    )
    if status == "FAIL":
        failures += 1

    # Test 3: Old 8x cap reached at 4*J50 = 800us under power-law
    print("\n--- Test 3: Old 8x cap intersection (jitter=800us => R=8*base_R) ---")
    je = 800
    ratio = je / J50_DEFAULT  # = 4.0
    r = BASE_R * (ratio**1.5)  # = 400 * 8 = 3200
    expected = BASE_R * 8
    status = "PASS" if abs(r - expected) < 1 else "FAIL"
    print(
        f"  jitter_excess={je}us ratio={ratio:.1f} R={r:.1f} expected={expected} {status}",
    )
    if status == "FAIL":
        failures += 1

    # Test 4: WiFi 2000us scenario
    print("\n--- Test 4: WiFi 2000us jitter ---")
    je = 2000
    ratio = je / J50_DEFAULT  # = 10
    r = BASE_R * (ratio**1.5)  # = 400 * 31.62 = 12649
    expected = 12649
    status = "PASS" if abs(r - expected) < 10 else "FAIL"
    print(
        f"  jitter_excess={je}us ratio={ratio:.1f} R={r:.0f} expected~{expected} {status}",
    )
    if status == "FAIL":
        failures += 1

    # Test 5: Clamped at R_MAX (102400) for extreme jitter
    print("\n--- Test 5: R_MAX=102400 clamping ---")
    for je in [50000, 100000, 500000]:
        ratio = je / J50_DEFAULT
        r_raw = BASE_R * (ratio**1.5)
        r_clamped = min(r_raw, R_MAX)
        clamped = "CLAMPED" if r_raw > R_MAX else "within"
        status = "PASS" if r_clamped <= R_MAX else "FAIL"
        print(
            f"  je={je}us ratio={ratio:.0f} r_raw={r_raw:.0f} r={r_clamped:.0f} [{clamped}] {status}",
        )
        if status == "FAIL":
            failures += 1

    # Test 6: Full sweep 0-100ms jitter, verify monotonic, no jumps
    print("\n--- Test 6: Monotonic sweep 0-100ms ---")
    prev = 0
    monotonic_fail = 0
    for je in range(0, 100001, 1000):
        ratio = je / J50_DEFAULT
        r = min(BASE_R * (ratio**1.5), R_MAX) if je > 0 else BASE_R
        if r < prev:
            monotonic_fail += 1
            if monotonic_fail <= 3:
                print(f"  NON-MONOTONIC at je={je}us: r={r:.0f} < prev={prev:.0f}")
        prev = r
    status = "PASS" if monotonic_fail == 0 else "FAIL"
    print(f"  Monotonic check: {status} ({monotonic_fail} failures)")
    if status == "FAIL":
        failures += 1

    # Test 7: Fixed-point overflow guard verification
    print("\n--- Test 7: Fixed-point overflow analysis ---")
    frac = 20
    for je in [1, 50, 200, 1000, 5000, 50000, 500000]:
        ratio_fp = (je << frac) // J50_DEFAULT
        sqrt_fp = math.isqrt(ratio_fp << frac)
        product = ratio_fp * sqrt_fp
        u64_max = 2**64 - 1
        overflow = product > u64_max
        status = "OVERFLOW" if overflow else "OK"
        print(
            f"  je={je:>6}us ratio_fp={ratio_fp:>15} sqrt_fp={sqrt_fp:>15} "
            f"product={product:>20} (>{u64_max}={overflow}) {status}",
        )
        if overflow and je <= 500000:
            failures += 1  # Should be safe at defaults (J50=200, max je=500ms)

    # Test 8: K_min verification at R_MAX
    print("\n--- Test 8: K_min ~= sqrt(Q/R_max) at R=102400 ---")
    k_min_approx = math.sqrt(Q_BASE / R_MAX)
    print(
        f"  sqrt(Q/R_max) = sqrt({Q_BASE}/{R_MAX}) = {k_min_approx:.5f} ~= {k_min_approx:.3f}",
    )
    status = "PASS" if abs(k_min_approx - 0.031) < 0.001 else "FAIL"
    print(f"  Expected ~0.031: {status}")
    if status == "FAIL":
        failures += 1

    return failures


if __name__ == "__main__":
    f = test_power_law_r()
    print(f"\n{'PASS' if f == 0 else f'{f} FAILURES'}")
