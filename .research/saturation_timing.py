#!/usr/bin/env python3
"""
saturation_timing.py -- Analyze saturation response timing.
Key question: is 1M rounds (~1400s at 1.4ms) too slow?
Should the saturation threshold be lowered for short-RTT paths?
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kcc_sim_base import P_EST_INIT, P_EST_MAX, Q_BASE

print("=" * 90)
print("SATURATION TIMING ANALYSIS: How fast does p_est reach P_MAX?")
print("=" * 90)

# Base calculation
p_init = P_EST_INIT
p_max = P_EST_MAX
q = Q_BASE

rounds_to_max = (p_max - p_init) / q
print("\n--- Baseline ---")
print(f"  p_est: {p_init} -> {p_max}")
print(f"  Q = {q} per skip round")
print(f"  Rounds to max: {rounds_to_max:,.0f}")
print(f"  Time at 1.4ms RTT: {rounds_to_max * 1.4 / 1000:,.0f} seconds")
print(f"  Time at 50ms RTT:  {rounds_to_max * 50 / 1000:,.0f} seconds")
print(f"  Time at 300ms RTT: {rounds_to_max * 300 / 1000:,.0f} seconds")

# Saturation fires at pos_skip = 55, but waits for p_est >= P_MAX
# pos_skip goes 0->55 in ~77ms (1.4ms * 55)
# Then goes 55->254 (saturation at 254 pos_skip)
# But can't fire until p_est >= P_MAX which takes ~1M rounds
# pos_skip saturates at 254 for ~1M rounds
print("\n--- Timing breakdown at 1.4ms RTT ---")
print(f"  pos_skip: 0->55 in {55 * 1.4:.0f}us = 77us  (first saturation window)")
print("  But p_est = 1000 + 55*100 = 6500 << 100M")
print(f"  pos_skip: 55->254 in {(254 - 55) * 1.4:.0f}us = 279us")
print("  pos_skip saturates at 254")
print(f"  p_est reaches P_MAX after {rounds_to_max:,.0f} rounds")
print("  At that point, saturation FINALLY fires")

# Alternative: adaptive saturation threshold
print("\n--- Adaptive saturation threshold analysis ---")
for fraction in [0.1, 0.25, 0.5, 1.0]:
    target = int(p_max * fraction)
    rounds = (target - p_init) / q
    time_1ms = rounds * 1 / 1000
    time_50ms = rounds * 50 / 1000
    print(
        f"  p_est_max * {fraction:.0%} = {target:,}: {rounds:,.0f} rounds "
        f"({time_1ms:.1f}s at 1ms, {time_50ms:.1f}s at 50ms)",
    )

print("\n--- Recommendation ---")
print("  Current saturation requires ~1M rounds (16 minutes at 1ms)")
print("  This is SLOWER than the PROBE_RTT interval (10s)")
print("  For short-RTT paths, saturation provides no timely protection")
print("  Consider: scale saturation threshold with RTT or use count-based trigger")

# Verify saturation cap protects BDP
print("\n--- BDP protection ---")
for rtt in [1400, 50000, 300000]:
    p_est_time = rounds_to_max * rtt / 1e6
    print(f"  RTT={rtt}us: saturation after {p_est_time:.0f}s, BDP capped at min_rtt")
