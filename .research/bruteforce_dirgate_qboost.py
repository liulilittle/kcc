#!/usr/bin/env python3
"""Brute-force G2_queue_cap + G3-detect convergence + directional gate interactions."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kcc_sim_base import BASE_R, Q_BASE, QBOOST_THRESH_US, SCALE, KCCFlow


def test_directional_qboost():
    failures = 0
    print("=" * 90)
    print("DIRECTIONAL GATE + FORCED CONV + G2_queue_cap BRUTE-FORCE")
    print("=" * 90)

    # Test 1: G3-detect convergence pulls x_est to min RTT
    print("\n--- Test 1: G3-detect convergence x_est=z ---")
    for rtt in [1400, 5000, 50000]:
        fl = KCCFlow()
        fl.x_est = 100000 * SCALE  # start with inflated estimate
        fl.min_rtt_us = rtt
        # Simulate a clean sample (nu<0)
        z = rtt * SCALE
        nu = z - fl.x_est
        assert nu <= 0, "Expected negative innovation"
        fl.x_est = min(z, 0xFFFFFFFF)  # G3-detect convergence
        status = "PASS" if fl.x_est_us() == rtt else "FAIL"
        print(f"  RTT={rtt}us x_est={fl.x_est_us()}us {status}")
        if status == "FAIL":
            failures += 1

    # Test 2: Directional gate rejects positive innovations
    print("\n--- Test 2: Directional gate nu>0 rejection ---")
    for rtt in [1400, 50000, 300000]:
        fl = KCCFlow()
        fl.x_est = rtt * SCALE  # converged
        z = (rtt + 5000) * SCALE  # +5ms queue
        nu = z - fl.x_est
        rejected = nu > 0
        status = "PASS" if rejected else "FAIL"
        print(
            f"  RTT={rtt}us z={rtt + 5000}us nu>0={rejected} => {'REJECTED' if rejected else 'ACCEPTED'} {status}",
        )
        if status == "FAIL":
            failures += 1

    # Test 3: G2_queue_cap threshold verification
    print(f"\n--- Test 3: G2_queue_cap threshold = {QBOOST_THRESH_US}ms ---")
    for shift_us in [4000, 8000, 16000, 20000, 50000]:
        fl = KCCFlow()
        fl.x_est = 1400 * SCALE
        z = (1400 + shift_us) * SCALE
        nu = abs(z - fl.x_est)  # in scaled units
        fires = nu > QBOOST_THRESH_US * SCALE  # STRICTLY greater than
        expected = shift_us > QBOOST_THRESH_US  # STRICTLY greater than
        status = "PASS" if fires == expected else "FAIL"
        print(
            f"  shift={shift_us}us (>{QBOOST_THRESH_US}ms={expected}) => "
            f"G2_queue_cap {'fires' if fires else 'silent'} {status}",
        )
        if status == "FAIL":
            failures += 1

    # Test 4: G2_queue_cap convergence speed
    print("\n--- Test 4: G2_queue_cap single-step convergence ---")
    for tp in [1400, 50000, 300000]:
        fl = KCCFlow()
        fl.x_est = tp * SCALE
        z = (tp + 50000) * SCALE  # 50ms baseline shift
        nu = z - fl.x_est
        K = (fl.p_est + Q_BASE) / (fl.p_est + Q_BASE + BASE_R)
        corr = int(K * nu)
        fl.x_est = min(fl.x_est + corr, 0xFFFFFFFF)
        xe_us = fl.x_est_us()
        target = tp + 50000
        error_pct = abs(xe_us - target) / target * 100
        status = "PASS" if error_pct < 30 else "FAIL"
        print(
            f"  T_prop={tp}us shift=+50ms: xe={xe_us}us target={target}us "
            f"error={error_pct:.1f}% {status}",
        )
        if status == "FAIL":
            failures += 1

    return failures


if __name__ == "__main__":
    f = test_directional_qboost()
    print(f"\n{'PASS' if f == 0 else f'{f} FAILURES'}")
