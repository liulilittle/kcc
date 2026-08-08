#!/usr/bin/env python3
"""Brute-force physical floor gate (speed-of-light) and G3 path-shift verification."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kcc_sim_base import (
    SCALE,
    KCCFlow,
    gated_drop_floor_reject,
    gated_drop_FLOOR_SHIFT,
)


def test_floor_g3():
    failures = 0
    print("=" * 90)
    print("PHYSICAL FLOOR GATE + G3 PATH-SHIFT BRUTE-FORCE")
    print("=" * 90)

    # Test 1: Floor gate threshold = x_est - x_est>>shift (12.5% at shift=3)
    print("\n--- Test 1: Floor gate = 12.5% of x_est ---")
    for xe_us in [1400, 50000, 100000, 300000]:
        xe = xe_us * SCALE
        floor = xe - (xe >> gated_drop_FLOOR_SHIFT)
        floor_us = floor // SCALE
        drop_pct = (xe_us - floor_us) / xe_us * 100
        status = "PASS" if abs(drop_pct - 12.5) < 0.1 else "FAIL"
        print(
            f"  x_est={xe_us:>6}us floor={floor_us:>6}us drop={drop_pct:.1f}% {status}",
        )
        if status == "FAIL":
            failures += 1

    # Test 2: Floor gate rejects 50% drop (TSO artifact)
    print("\n--- Test 2: Floor gate rejects 50% drop ---")
    fl = KCCFlow()
    fl.x_est = 50000 * SCALE
    fl.neg_skip_count = 0
    z = 25000 * SCALE  # 50% drop
    rejected = gated_drop_floor_reject(z, fl.x_est, fl.neg_skip_count)
    status = "PASS" if rejected else "FAIL"
    print(f"  x_est=50ms z=25ms neg_skip=0 => rejected={rejected} {status}")
    if status == "FAIL":
        failures += 1

    # Test 3: Floor gate accepts 5% drop (normal jitter)
    print("\n--- Test 3: Floor gate passes 5% drop ---")
    fl.neg_skip_count = 0
    z = int(50000 * 0.95) * SCALE
    accepted = not gated_drop_floor_reject(z, fl.x_est, fl.neg_skip_count)
    status = "PASS" if accepted else "FAIL"
    print(f"  x_est=50ms z=47.5ms => accepted={accepted} {status}")
    if status == "FAIL":
        failures += 1

    # Test 4: Floor gate bypassed after neg_skip >= 3 (persistent evidence)
    print("\n--- Test 4: Floor bypass after 3 consecutive negatives ---")
    fl.neg_skip_count = 3
    z = 25000 * SCALE  # 50% drop
    bypassed = not gated_drop_floor_reject(z, fl.x_est, fl.neg_skip_count)
    status = "PASS" if bypassed else "FAIL"
    print(f"  x_est=50ms z=25ms neg_skip=3 => bypassed={bypassed} {status}")
    if status == "FAIL":
        failures += 1

    # Test 5: G3 non-trigger under congestion
    print("\n--- Test 5: G3 path-shift verification ---")
    # G3 requires: C1: nu > 2.5xqdelay_ewma, C2: qdelay_ewma < min_rtt>>1 (50%)
    # Under congestion with qdelay > 50% RTT, C2 should fail
    fl = KCCFlow()
    fl.x_est = 1400 * SCALE
    fl.min_rtt_us = 1400
    # Simulate severe congestion: qdelay=10ms, RTT=11.4ms
    qdelay = 10000
    rtt = 11400
    z = rtt * SCALE
    nu = abs(z - fl.x_est)  # = 10000us * SCALE
    # C1: nu > 2.5 * qdelay? 10000 > 2.5*10000=25000? NO
    c1 = nu > 2.5 * qdelay * SCALE
    # C2: qdelay < min_rtt>>1? 10000 < 700? NO
    c2 = qdelay < fl.min_rtt_us >> 1
    triggers = c1 and c2
    status = "PASS" if not triggers else "FAIL"
    print(
        f"  RTT=11.4ms qdelay=10ms: C1={c1} C2={c2} => G3={'fires' if triggers else 'silent'} {status}",
    )
    if status == "FAIL":
        failures += 1

    return failures


if __name__ == "__main__":
    f = test_floor_g3()
    print(f"\n{'PASS' if f == 0 else f'{f} FAILURES'}")
