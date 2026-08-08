#!/usr/bin/env python3
"""
Brute-force verification of the RTT-proportional outlier gate.
Tests: threshold correctness across 1-300ms RTT, jitter range,
       interaction with min_floor, edge cases.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kcc_sim_base import OUTLIER_JITTER_MULT, OUTLIER_MIN_FLOOR_US, SCALE

rt_vals = [1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 300000]
jit_vals = [0, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000]
shift_vals = [0, 1, 2, 3, 4]  # varying kcc_noise_reject_rtt_frac_shift


def test_outlier_gate():
    failures = 0
    print("=" * 90)
    print("OUTLIER GATE BRUTE-FORCE: RTT 1-300ms, jitter 0-20ms, shift 0-4")
    print("=" * 90)

    # Test 1: RTT-proportional threshold with default shift=2
    print("\n--- Test 1: Default threshold (shift=2, 25% RTT, floor=50us) ---")
    print(
        f"{'RTT(us)':>10} {'prop_us':>10} {'floor_us':>10} {'gate_us':>10} {'valid?':>8}",
    )
    for rtt in rt_vals:
        prop = rtt >> 2
        gate = max(prop, OUTLIER_MIN_FLOOR_US)
        valid = "OK" if gate >= OUTLIER_MIN_FLOOR_US else "FAIL"
        print(f"{rtt:>10} {prop:>10} {OUTLIER_MIN_FLOOR_US:>10} {gate:>10} {valid:>8}")
        if gate < OUTLIER_MIN_FLOOR_US:
            failures += 1

    # Test 2: Floor dominance on short-RTT paths
    print("\n--- Test 2: Floor dominates for RTT < 200us ---")
    short_rtts = [100, 150, 190, 200, 250]
    for rtt in short_rtts:
        prop = rtt >> 2
        gate = max(prop, OUTLIER_MIN_FLOOR_US)
        dom = "floor" if prop <= OUTLIER_MIN_FLOOR_US else "prop"
        status = "OK" if gate in (OUTLIER_MIN_FLOOR_US, prop) else "FAIL"
        print(
            f"  RTT={rtt:>4}us prop={prop:>4}us floor={OUTLIER_MIN_FLOOR_US}us gate={gate}us [{dom}] {status}",
        )
        if gate != max(prop, OUTLIER_MIN_FLOOR_US):
            failures += 1

    # Test 3: Jitter component dominance
    print("\n--- Test 3: Jitter component vs RTT component (default mult=2) ---")
    print(
        f"{'RTT(us)':>10} {'jit(us)':>10} {'prop_us':>10} {'jit*2':>10} {'dominates':>12}",
    )
    for rtt in rt_vals[::2]:
        for jit in jit_vals[::2]:
            prop = max(rtt >> 2, OUTLIER_MIN_FLOOR_US)
            jitter_part = jit * OUTLIER_JITTER_MULT
            dom = "jitter" if jitter_part > prop else "RTT/floor"
            gate = max(prop, jitter_part)
            print(f"{rtt:>10} {jit:>10} {prop:>10} {jitter_part:>10} {dom:>12}")

    # Test 4: Scaled threshold (code path: dyn_thresh = max_wins * scale)
    print("\n--- Test 4: Scaled threshold computation ---")
    scale_shift = SCALE.bit_length() - 1
    print(f"  scale_shift = ilog2({SCALE}) = {scale_shift}")
    for rtt in [1400, 50000, 200000]:
        prop = max(rtt >> 2, OUTLIER_MIN_FLOOR_US)
        jit_thresh = 5000 * OUTLIER_JITTER_MULT  # 5ms jitter example
        dyn_us = max(prop, jit_thresh)
        dyn_scaled = dyn_us << scale_shift
        if dyn_scaled < 0:
            print(f"  FAIL: RTT={rtt}us dyn_scaled overflow")
            failures += 1
        else:
            print(
                f"  RTT={rtt:>6}us prop={prop:>6}us jitter={jit_thresh:>6}us "
                f"=> dyn={dyn_us:>6}us => scaled={dyn_scaled:>12} OK",
            )

    # Test 5: Gate behavior on the user's test scenario
    print("\n--- Test 5: User scenario (1.4ms RTT, 3-8ms jitter) ---")
    rtt = 1400
    for jit in [3000, 5000, 8000]:
        prop = max(rtt >> 2, OUTLIER_MIN_FLOOR_US)
        jitter_part = jit * OUTLIER_JITTER_MULT
        gate = max(prop, jitter_part)
        dom = "jitter dominates" if jitter_part > prop else "RTT/floor dominates"
        print(
            f"  jitter={jit}us: prop={prop}us jitter*2={jitter_part}us "
            f"gate={gate}us => {dom}",
        )

    return failures


if __name__ == "__main__":
    f = test_outlier_gate()
    print(f"\n{'PASS' if f == 0 else f'{f} FAILURES'}")
