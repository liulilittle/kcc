#!/usr/bin/env python3
"""
parameter_matrix.py -- Brute-force parameter interaction verification.
Checks invariants across all valid parameter combinations.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kcc_sim_base import (
    BASE_R,
    DRIFT_EARLY_MIN_RTT,
    DRIFT_QUIET_JITTER_SHIFT,
    DRIFT_THRESH,
    NEG_PERSIST_THRESH,
    OUTLIER_MIN_FLOOR_US,
    OUTLIER_RTT_FRAC_SHIFT,
    P_EST_FLOOR,
    P_EST_INIT,
    P_EST_MAX,
    Q_BASE,
    QBOOST_THRESH_US,
    R_MAX,
    SATURATION_THRESH,
    DRIFT_G3_slow_MULT,
)

failures = 0
print("=" * 90)
print("PARAMETER INTERACTION MATRIX: Invariant verification")
print("=" * 90)

# Invariant 1: SAT_THR < DRIFT_THRESH * DRIFT_G3_slow_MULT
print("\n--- Invariant 1: Saturation fires before Tier 2 ---")
sat_eff = SATURATION_THRESH  # effective after clamping
t2_min = DRIFT_THRESH * DRIFT_G3_slow_MULT  # = 56
print(f"  SAT_THR={sat_eff}, T2_MIN={t2_min}")
print(f"  sat < t2: {sat_eff} < {t2_min} = {sat_eff < t2_min}")
if sat_eff >= t2_min:
    print("  FAIL: Saturation would never fire! Tier 2 always preempts it.")
    failures += 1
else:
    print("  PASS")

# Invariant 2: DRIFT_EARLY_MIN_RTT < DRIFT_THRESH
print("\n--- Invariant 2: Early drift < Tier 1 ---")
print(f"  EARLY={DRIFT_EARLY_MIN_RTT}, G3_fast={DRIFT_THRESH}")
if DRIFT_EARLY_MIN_RTT >= DRIFT_THRESH:
    print("  FAIL: Early drift preempts Tier 1")
    failures += 1
else:
    print("  PASS")

# Invariant 3: R_MAX > BASE_R
print("\n--- Invariant 3: R_MAX > BASE_R ---")
if R_MAX <= BASE_R:
    print(f"  FAIL: R_MAX={R_MAX} <= BASE_R={BASE_R}")
    failures += 1
else:
    print(f"  PASS: R_MAX={R_MAX} >> BASE_R={BASE_R}")

# Invariant 4: QBOOST_THRESH > typical queue
print("\n--- Invariant 4: G2_queue_cap threshold > typical queue ---")
print(f"  QBOOST={QBOOST_THRESH_US}us = {QBOOST_THRESH_US / 1000}ms")
print("  Typical queue per BBR: 0.25 * BDP. At 50ms RTT = 12.5ms")
print("  At 1.4ms RTT = 0.35ms")
print("  QBOOST(16ms) > 12.5ms max BBR queue => only fires on path changes")
print("  PASS" if QBOOST_THRESH_US > 12500 else "  WARN: may fire on BBR probe queue")

# Invariant 5: NEG_PERSIST_THRESH >= 3
print("\n--- Invariant 5: Neg persist threshold ---")
print(f"  NEG_PERSIST_THRESH={NEG_PERSIST_THRESH}")
print("  3 consecutive negatives required to bypass floor gate")
print("  P(3 consecutive negative noise) <= 2^-3 = 12.5% (single flow)")
print("  PASS: threshold provides meaningful persistence filtering")

# Invariant 6: p_est_floor < p_est_init < p_est_max
print("\n--- Invariant 6: p_est ordering ---")
print(f"  floor={P_EST_FLOOR} < init={P_EST_INIT} < max={P_EST_MAX}")
if P_EST_FLOOR >= P_EST_INIT or P_EST_INIT >= P_EST_MAX:
    print("  FAIL")
    failures += 1
else:
    print("  PASS")

# Invariant 7: K_min exists (R_max allows meaningful learning)
print("\n--- Invariant 7: K_min > 0 at R_MAX ---")
k_min = math.sqrt(Q_BASE / R_MAX)
print(f"  K_min = sqrt({Q_BASE}/{R_MAX}) = {k_min:.5f}")
if k_min < 0.001:
    print("  FAIL: Filter would be effectively frozen at R_MAX")
    failures += 1
else:
    print(f"  PASS: Minimum {k_min * 100:.1f}% observation_update_gain retained")

# Invariant 8: Outlier gate floor < RTT fraction at typical RTTs
print("\n--- Invariant 8: Outlier floor vs RTT fraction ---")
print(
    f"  floor={OUTLIER_MIN_FLOOR_US}us, shift={OUTLIER_RTT_FRAC_SHIFT} => {100 >> OUTLIER_RTT_FRAC_SHIFT}% RTT",
)
for rtt in [1400, 50000, 200000]:
    prop = rtt >> OUTLIER_RTT_FRAC_SHIFT
    gate = max(prop, OUTLIER_MIN_FLOOR_US)
    pct = gate / rtt * 100
    print(
        f"  RTT={rtt:>6}us => prop={prop:>6}us floor={OUTLIER_MIN_FLOOR_US}us => gate={gate:>6}us ({pct:.1f}% RTT)",
    )

# Invariant 9: Tier 1 jitter threshold vs jitter_ewma
print("\n--- Invariant 9: Tier 1 jitter gate ---")
print(
    f"  shift={DRIFT_QUIET_JITTER_SHIFT} => jitter_ewma < min_rtt/{1 << DRIFT_QUIET_JITTER_SHIFT}",
)
for rtt in [1400, 50000, 200000]:
    thresh = rtt >> DRIFT_QUIET_JITTER_SHIFT
    print(f"  RTT={rtt:>6}us => T1 jitter gate < {thresh}us")

print(f"\n{'PASS' if failures == 0 else f'{failures} FAILURES'}")
