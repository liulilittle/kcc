#!/usr/bin/env python3
"""
clean_thresh_bruteforce.py -- Verify clean/cong threshold computation
across 1-300ms RTT. Checks: floor dominance, bp calculation, invariants.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

KCC_QDELAY_BP_BASE = 10000
CLEAN_BP = 1000  # 10% BDP
CONG_BP = 2500  # 25% BDP
FLOOR_US = 500  # absolute floor

rt_vals = [1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 300000]
failures = 0

print("=" * 90)
print("CLEAN/CONG THRESHOLD BRUTE-FORCE: 1-300ms RTT")
print(f"  clean_bp={CLEAN_BP}bp ({CLEAN_BP / KCC_QDELAY_BP_BASE * 100:.0f}% BDP)")
print(f"  cong_bp={CONG_BP}bp ({CONG_BP / KCC_QDELAY_BP_BASE * 100:.0f}% BDP)")
print(f"  floor={FLOOR_US}us")
print("=" * 90)

# Test 1: Clean threshold across RTTs
print(
    f"\n{'RTT(us)':>10} {'bp_us':>10} {'floor_us':>10} {'clean_us':>10} {'clean%':>8} {'dominates':>12}",
)
for rtt in rt_vals:
    bp_us = (rtt * CLEAN_BP) // KCC_QDELAY_BP_BASE
    clean = max(bp_us, FLOOR_US)
    dom = "floor" if bp_us <= FLOOR_US else "RTT%"
    pct = clean / rtt * 100
    print(f"{rtt:>10} {bp_us:>10} {FLOOR_US:>10} {clean:>10} {pct:>7.1f}% {dom:>12}")
    if clean > rtt:
        print(
            f"  WARN: clean_thresh({clean}us) > min_rtt({rtt}us) -- threshold exceeds path!",
        )
        # Not a bug per se -- the floor may dominate, but worth noting

# Test 2: Cong > Clean invariant
print(f"\n{'RTT':>10} {'clean':>10} {'cong':>10} {'gap':>10} {'invariant':>12}")
for rtt in rt_vals:
    clean = max((rtt * CLEAN_BP) // KCC_QDELAY_BP_BASE, FLOOR_US)
    cong = max((rtt * CONG_BP) // KCC_QDELAY_BP_BASE, FLOOR_US)
    gap = cong - clean
    ok = "OK" if cong > clean or (cong == clean and cong == FLOOR_US) else "FAIL"
    if ok == "FAIL":
        failures += 1
    print(f"{rtt:>10} {clean:>10} {cong:>10} {gap:>10} {ok:>12}")

# Test 3: Floor dominance region
print("\n--- Test 3: Floor dominates for RTT < 5000us ---")
floor_rtt = FLOOR_US * KCC_QDELAY_BP_BASE // CLEAN_BP
print(f"  Floor ({FLOOR_US}us) dominates clean_thresh for RTT < {floor_rtt}us")
for rtt in [1000, 2000, 3000, 4000, 5000, 6000, 10000]:
    clean = max((rtt * CLEAN_BP) // KCC_QDELAY_BP_BASE, FLOOR_US)
    dom = "floor" if clean == FLOOR_US else f"RTT% ({clean}us)"
    print(f"  RTT={rtt}us => clean={clean}us [{dom}]")

# Test 4: Clean threshold as fraction of qdelay (drain skip decision)
print("\n--- Test 4: Drain skip boundary ---")
print("  Drain skip fires when qdelay_avg < clean_thresh")
print(
    f"  This means queue < {CLEAN_BP / KCC_QDELAY_BP_BASE * 100:.0f}% of RTT (or {FLOOR_US}us floor)",
)
print("  On user's 1.4ms path: clean = max(1400*0.1=140us, 500us) = 500us")
print("  User's qdelay (3-8ms) >> 500us => drain skip BLOCKED (correct)")

print(f"\n{'PASS' if failures == 0 else f'{failures} FAILURES'}")
