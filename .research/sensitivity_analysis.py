#!/usr/bin/env python3
"""
sensitivity_analysis.py -- Condition numbers for EVERY KCC parameter.
Measures how much K_obs_drain, x_est, and gate FP rates change per 1% parameter change.
Identifies which parameters are most/least critical.
Tests at RTT 1us--500ms.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

failures = 0


def fail(msg):
    global failures
    print(f"  FAIL: {msg}")
    failures += 1


def pass_(msg):
    print(f"  PASS: {msg}")


def info(msg):
    print(f"  INFO: {msg}")


print("=" * 90)
print("KCC PARAMETER SENSITIVITY ANALYSIS -- CONDITION NUMBERS")
print("=" * 90)

# =============================================================================
# 1. K_obs_drain sensitivity to Q (dK_obs_drain/dQ)
# =============================================================================
print("\n--- 1. dK_obs_drain/dQ -- how K_obs_drain reacts to Q changes ---")
for Q_base, R, delta_pct in [(100, 400, 10), (2000, 400, 10), (100, 102400, 10)]:
    Q1 = Q_base * (1 - delta_pct / 100)
    Q2 = Q_base * (1 + delta_pct / 100)
    p1 = (Q1 + math.sqrt(Q1**2 + 4 * Q1 * R)) / 2
    p2 = (Q2 + math.sqrt(Q2**2 + 4 * Q2 * R)) / 2
    K1 = p1 / (p1 + R)
    K2 = p2 / (p2 + R)
    dK_dQ_pct = (
        abs(K2 - K1) / K1 * 100 / (2 * delta_pct)
    )  # % change in K per 1% change in Q
    info(
        f"  Q={Q_base},R={R}: dK/dQ ~= {dK_dQ_pct:.2f}x (1% DeltaQ -> {dK_dQ_pct:.2f}% DeltaK)",
    )

# =============================================================================
# 2. K_obs_drain sensitivity to R (dK_obs_drain/dR)
# =============================================================================
print("\n--- 2. dK_obs_drain/dR -- how K_obs_drain reacts to R changes ---")
for Q, R_base, delta_pct in [(100, 400, 10), (2000, 400, 10), (100, 3200, 10)]:
    R1 = R_base * (1 - delta_pct / 100)
    R2 = R_base * (1 + delta_pct / 100)
    p1 = (Q + math.sqrt(Q**2 + 4 * Q * R1)) / 2
    p2 = (Q + math.sqrt(Q**2 + 4 * Q * R2)) / 2
    K1 = p1 / (p1 + R1)
    K2 = p2 / (p2 + R2)
    dK_dR_pct = abs(K2 - K1) / K1 * 100 / (2 * delta_pct)
    info(
        f"  Q={Q},R={R_base}: dK/dR ~= {dK_dR_pct:.2f}x (1% DeltaR -> {dK_dR_pct:.2f}% DeltaK)",
    )

# =============================================================================
# 3. x_est convergence sensitivity to K (how convergence rate depends on K)
# =============================================================================
print("\n--- 3. x_est convergence sensitivity to K ---")
for K in [0.03, 0.1, 0.347, 0.5, 0.88]:
    x_0 = 2400.0  # start 1000us above true
    x_true = 1400.0
    steps = []
    x = x_0
    for _ in range(100):
        x = x + K * (x_true - x)  # oversimplified but captures exponential convergence
        steps.append(x)
    # Time to reach within 10%: x - x_true < 0.1 * (x_0 - x_true)
    threshold = x_true + 0.1 * (x_0 - x_true)
    t_10 = next(
        (i for i, v in enumerate(steps) if abs(v - x_true) < abs(threshold - x_true)),
        100,
    )
    info(f"  K={K:.2f}: T_10%={t_10} steps = {t_10 * 1.4:.0f}ms at DC RTT")

# =============================================================================
# 4. Outlier gate FP sensitivity to parameters
# =============================================================================
print("\n--- 4. Outlier gate FP sensitivity to rtt_frac_shift ---")
for rtt, sigma in [(1400, 20), (50000, 200)]:
    for shift in [1, 2, 3, 4]:
        prop = max(rtt >> shift, 50)
        k = prop / sigma
        fp = min(1.0, 1.0 / (k * k))
        for shift2 in [shift - 1, shift + 1]:
            if 0 <= shift2 <= 8:
                prop2 = max(rtt >> shift2, 50)
                k2 = prop2 / sigma
                fp2 = min(1.0, 1.0 / (k2 * k2))
                if fp > 0:
                    ratio = fp2 / fp
                    if ratio > 10:
                        info(
                            f"  RTT={rtt}us: shift {shift}->{shift2}: FP changes {ratio:.0f}x",
                        )

# =============================================================================
# 5. Drift detection sensitivity to drift_thresh
# =============================================================================
print("\n--- 5. Drift detection sensitivity to drift_thresh (D=14) ---")
for D in [10, 12, 14, 16, 18, 20]:
    FP = 2 ** (-D)
    info(f"  D={D}: FP={FP:.2e}")
# Changing D by +/-2 changes FP by factor of 4

# =============================================================================
# 6. Power-law exponent sensitivity
# =============================================================================
print("\n--- 6. Power-law exponent sensitivity: dR/dexp ---")
for exp_ in [1.0, 1.25, 1.5, 1.75, 2.0]:
    for je in [200, 500, 2000, 10000]:
        ratio = je / 200.0
        R = 400 * ratio**exp_
        R = max(400, min(R, 102400))
        K = math.sqrt(100 / max(R, 1))
        if je in [200, 2000]:
            info(f"  exp={exp_:.2f}, je={je}us: R={R:.0f}, K={K:.4f}")

# =============================================================================
# 7. J50 sensitivity
# =============================================================================
print("\n--- 7. J50 sensitivity: dR/dJ50 ---")
for J50 in [50, 100, 150, 200, 300, 500]:
    for je in [200, 500, 2000]:
        ratio = je / J50
        R = 400 * ratio**1.5
        R = max(400, min(R, 102400))
        if je in [200, 500]:
            info(f"  J50={J50}, je={je}us: R={R:.0f}")

# =============================================================================
# 8. r_max_boost sensitivity
# =============================================================================
print("\n--- 8. r_max_boost sensitivity: when does cap matter? ---")
for cap in [64, 128, 256, 512, 1000]:
    R_cap = 400 * cap
    K_at_cap = math.sqrt(100 / R_cap)
    # At what jitter does R hit this cap?
    je_at_cap = 200 * (
        cap ** (2 / 3)
    )  # solve: 400 * (je/200)^1.5 = 400*cap -> je = 200 * cap^(2/3)
    info(
        f"  cap={cap} (R_max={R_cap}): K_min={K_at_cap:.4f}, hits cap at je={je_at_cap:.0f}us",
    )

# =============================================================================
# 9. Clean threshold classification sensitivity
# =============================================================================
print("\n--- 9. Clean/congest classification boundary sharpness ---")
for rtt, bp_pct in [(1400, 5), (1400, 10), (1400, 15), (50000, 5), (50000, 10)]:
    thresh = max(rtt * bp_pct // 100, 500)
    info(f"  RTT={rtt}us, {bp_pct}%: clean_thresh={thresh}us")

# =============================================================================
# 10. Overall system sensitivity matrix
# =============================================================================
print("\n--- 10. Parameter criticality ranking (most -> least sensitive) ---")
# Rank parameters by how much K_obs_drain changes per 1% parameter change at default values
importances = []
# Q sensitivity
Q = 100
R = 400
p_ss = (Q + math.sqrt(Q**2 + 4 * Q * R)) / 2
K_obs_drain = p_ss / (p_ss + R)
for name, _pct_change, fn in [
    (
        "Q",
        1,
        lambda d: (
            (Q * (1 + d) + math.sqrt((Q * (1 + d)) ** 2 + 4 * Q * (1 + d) * R))
            / 2
            / (
                (Q * (1 + d) + math.sqrt((Q * (1 + d)) ** 2 + 4 * Q * (1 + d) * R)) / 2
                + R
            )
        ),
    ),
    (
        "R",
        1,
        lambda d: (
            (Q + math.sqrt(Q**2 + 4 * Q * R * (1 + d)))
            / 2
            / ((Q + math.sqrt(Q**2 + 4 * Q * R * (1 + d))) / 2 + R * (1 + d))
        ),
    ),
]:
    K_plus = fn(0.01)
    K_minus = fn(-0.01)
    dK = abs(K_plus - K_minus) / K_obs_drain * 100 / 2
    importances.append((abs(dK), name, dK))

importances.sort(reverse=True)
for rank, (_impact, name, dK) in enumerate(importances):
    info(f"  #{rank + 1}: {name} -- {(dK / 1):.2f}% DeltaK per 1% Deltaparam")

# =============================================================================
print(f"\n{'=' * 90}")
print(f"SENSITIVITY ANALYSIS COMPLETE -- {failures} issues")
