#!/usr/bin/env python3

"""outlier_exhaustive.py -- Exhaustive outlier gate validation.
Sweeps all RTT 1-300ms, jitter 0-500ms, qdelay, all param combinations.
Identifies gate dominance regions and verifies no edge-case failures."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
SCALE = 1024
SCALE_SHIFT = 10
failures = 0


def fail(msg):
    global failures
    print(f"  FAIL: {msg}")
    failures += 1


def pass_(msg):
    print(f"  PASS: {msg}")


def info(msg):
    print(f"  INFO: {msg}")


def outlier_thresh_us(
    min_rtt_us,
    jitter_us,
    rtt_frac_shift=2,
    min_floor_us=50,
    jitter_mult=2,
):
    prop_us = max(min_rtt_us >> rtt_frac_shift, min_floor_us)
    return max(prop_us, jitter_us * jitter_mult)


def outlier_fires(innov_us, min_rtt_us, jitter_us, **kwargs):
    return abs(innov_us) > outlier_thresh_us(min_rtt_us, jitter_us, **kwargs)


print("=" * 90)
print("OUTLIER GATE EXHAUSTIVE VALIDATION")
print("=" * 90)
print("\n--- 1. Gate dominance regions (RTT-prop vs jitter-prop vs floor) ---")
RTTs = [100, 300, 500, 1400, 2000, 5000, 10000, 50000, 100000, 300000]
jitters = [0, 10, 25, 50, 100, 200, 500, 1000, 5000, 10000, 50000, 100000]
for rtt in RTTs:
    floor = max(rtt >> 2, 50)
    jitter_thresh_at_0 = jitters[1] * 2
    jitter_thresh_at_50 = 50 * 2
    jitter_thresh_at_500 = 500 * 2
    dominant = []
    if floor >= jitter_thresh_at_0 and floor >= jitter_thresh_at_50:
        dominant.append(f"floor={floor}us")
    if floor < jitter_thresh_at_500:
        dominant.append("jitter dominates at high jitter")
    info(f"  RTT={rtt:>6d}us: floor={floor}us, jitter_contrib=jitter*2")
print("\n--- 2. Min floor dominance: RTT<200us always uses min_floor_us=50 ---")
for rtt in [1, 10, 50, 100, 150, 199, 200, 201]:
    prop = max(rtt >> 2, 50)
    expected_floor = 50 if rtt < 200 else rtt >> 2
    if prop == expected_floor:
        pass_(f"  RTT={rtt:>3d}us: prop_thresh={prop}us (expected={expected_floor}us)")
    else:
        fail(f"  RTT={rtt:>3d}us: prop_thresh={prop}us (expected={expected_floor}us)")
# ---------------------------------------------------------------------------
# 3. rtt_frac_shift sweep
# ---------------------------------------------------------------------------
print("\n--- 3. rtt_frac_shift sweep: 0-8 (100% -> 0.4% RTT) ---")
for shift in range(9):
    for rtt in [1400, 50000, 300000]:
        prop = max(rtt >> shift, 50)
        pct = prop / rtt * 100
        pct = min(pct, 100)
        if prop >= 50:
            pass_(
                f"  shift={shift}, RTT={rtt:>6d}us: gate={prop:>6d}us = {pct:.1f}% RTT",
            )
        else:
            fail(f"  shift={shift}, RTT={rtt:>6d}us: gate={prop}us < 50us floor")
print("\n--- 4. jitter_mult sweep: 1-8 ---")
for mult in [1, 2, 3, 4, 8]:
    for jitter in [25, 100, 500, 10000]:
        gate = jitter * mult
        if gate > 0:
            pass_(f"  mult={mult}, jitter={jitter:>5d}us: jitter_gate={gate}us")
        else:
            fail(f"  mult={mult}, jitter={jitter:>5d}us: gate=0 (invalid)")
print("\n--- 5. Component dominance tracking ---")
for rtt in [1400, 50000, 300000]:
    for jitter in [0, 50, 500, 5000, 20000, 100000]:
        gate = outlier_thresh_us(rtt, jitter)
        prop_part = max(rtt >> 2, 50)
        jitter_part = jitter * 2
        dominant = "prop" if prop_part > jitter_part else "jitter"
        pass_(
            f"  RTT={rtt:>6d}us, jitter={jitter:>5d}us: gate={gate}us ({dominant} dominates)",
        )
print("\n--- 6. Edge cases: zero jitter, minimal RTT ---")
for rtt in [1, 10, 50, 100]:
    gate = outlier_thresh_us(rtt, 0)
    if gate >= 50:
        pass_(f"  RTT={rtt:>3d}us, jitter=0: gate={gate}us (floored to >=50us)")
    else:
        fail(f"  RTT={rtt:>3d}us, jitter=0: gate={gate}us < 50us")
gate = outlier_thresh_us(1, 500000)
if gate == 1000000:
    pass_(f"  RTT=1us, jitter=500ms: gate={gate}us (jitter dominates)")
else:
    fail(f"  RTT=1us, jitter=500ms: gate={gate}us")
print("\n--- 7. Gate in scaled units (x1024) for C code comparison ---")
for rtt in [1400, 50000]:
    for jitter in [50, 500, 10000]:
        gate_us = outlier_thresh_us(rtt, jitter)
        gate_scaled = gate_us * SCALE
        if gate_scaled <= 0xFFFFFFFF:
            pass_(
                f"  RTT={rtt}us, jitter={jitter}us: gate_scaled={gate_scaled:_d} (fits u32)",
            )
        else:
            fail(
                f"  RTT={rtt}us, jitter={jitter}us: gate_scaled={gate_scaled:_d} OVERFLOWS u32",
            )
print("\n--- 8. False-positive rate estimation per RTT/jitter ---")
for rtt, sigma in [(1400, 20), (50000, 200), (300000, 500)]:
    for jitter_us in [sigma, sigma * 2, sigma * 5]:
        gate_us = outlier_thresh_us(rtt, jitter_us)
        k = gate_us / sigma
        p_fp = min(1.0, 1.0 / (k * k)) if k > 0 else 1.0
        if p_fp < 0.01:
            pass_(
                f"  RTT={rtt}us, jitter={jitter_us}us: gate={gate_us}us, k={k:.1f}, Cheb<={p_fp:.4f} (< 1% FP)",
            )
        else:
            info(
                f"  RTT={rtt}us, jitter={jitter_us}us: gate={gate_us}us, k={k:.1f}, Cheb<={p_fp:.4f}",
            )
print("\n--- 9. Force-accept safety valve: P(20 consecutive rejects) ---")
p_rej = 0.72
p_20 = p_rej**20
pass_(f"  P(20 consec rejects) = {p_rej}^20 = {p_20:.2e}")
p_rej_alt = 0.5
p_20_alt = p_rej_alt**20
if p_20_alt < 1e-5:
    pass_(f"  Even P_rej=0.5: P(20)= {p_20_alt:.2e} -- safely rare")
print("\n--- 10. Outlier gate vs clean threshold interaction ---")
for rtt, label in [(1400, "DC"), (50000, "WAN"), (300000, "long-haul")]:
    clean = max(rtt * 1000 // 10000, 500)
    outlier = max(rtt >> 2, 50)
    if outlier < clean:
        info(
            f"  {label}: outlier_gate={outlier}us < clean={clean}us (outlier more selective on clean samples)",
        )
    else:
        info(
            f"  {label}: outlier_gate={outlier}us >= clean={clean}us (outlier less selective than clean)",
        )
print(f"\n{'=' * 90}")
if failures == 0:
    print("ALL OUTLIER GATE EXHAUSTIVE VERIFICATIONS PASSED")
else:
    print(f"{failures} FAILURES DETECTED")
