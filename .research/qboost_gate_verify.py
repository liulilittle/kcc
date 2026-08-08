#!/usr/bin/env python3
"""Verify G2_queue_cap qdelay gate: confirm it suppresses noise-triggered G2_queue_caps."""

import os
import random
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kcc_sim_base import (
    BASE_R,
    J50_DEFAULT,
    P_EST_FLOOR,
    P_EST_INIT,
    P_EST_MAX,
    Q_BASE,
    QBOOST_THRESH_US,
    R_MAX,
    SCALE,
    KCCFlow,
)

SEEDS = list(range(5))


def qb_count(nf, tp, qd, jh, sh, seed, n_rounds=1000):
    """Count G2_queue_cap events with full gate simulation."""
    random.seed(seed)
    fl = KCCFlow()
    fl.x_est = tp * SCALE
    fl.min_rtt_us = tp
    qb = 0
    cdwn = 0
    qewma = 0
    for _ in range(n_rounds):
        jt = jh + int(random.gauss(0, jh * 0.5))
        jt = max(0, jt)
        fl.jitter_ewma = (
            int(fl.jitter_ewma * 0.875 + jt * 0.125) if fl.jitter_ewma else jt
        )
        qewma = qewma * 0.875 + qd * 0.125
        fl.qdelay_ewma = int(qewma)
        rtt = tp + qd + jt
        z = rtt * SCALE
        nu = z - fl.x_est
        je = max(0, jt - jt // 4)
        ratio = je / J50_DEFAULT if je > 0 else 0
        r = (
            max(BASE_R, max(BASE_R, min(int(BASE_R * ratio**1.5), R_MAX)))
            if ratio > 0
            else BASE_R
        )
        ppred = min(fl.p_est + Q_BASE, P_EST_MAX)
        K = ppred / (ppred + r)
        corr_abs = int(K * abs(nu))
        if nu <= 0:
            fl.x_est = min(z, 0xFFFFFFFF)
            fl.p_est = max(r, P_EST_FLOOR)
            fl.pos_skip_cnt = 0
        else:
            fl.pos_skip_cnt = min(fl.pos_skip_cnt + 1, 254)
            fl.p_est = ppred
            if cdwn > 0:
                cdwn -= 1
            if (
                cdwn == 0
                and fl.pos_skip_cnt < 5
                and abs(nu) > QBOOST_THRESH_US * SCALE
                and fl.p_est <= 150
            ):
                thresh = max(fl.x_est >> (10 + sh), 1)
                if fl.qdelay_ewma < thresh:
                    fl.p_est = P_EST_INIT
                    fl.x_est = min(fl.x_est + corr_abs, 0xFFFFFFFF)
                    fl.pos_skip_cnt = 0
                    cdwn = 6
                    qb += 1
    return qb


print("=" * 90)
print("G2_queue_cap QDELAY GATE VERIFICATION: shift=0 vs shift=1")
print("=" * 90)

# Test: 1.4ms path, 5ms qdelay, 5ms jitter (matches drift test scenario)
print("\n--- User scenario: 1.4ms, qdelay=5ms, jitter=5ms ---")
print(f"{'shift':>6} {'QB mean':>10} {'QB std':>10} {'Reduction':>12}")
for sh in [0, 1]:
    counts = [qb_count(1, 1400, 5000, 5000, sh, s) for s in SEEDS]
    mean_qb = statistics.mean(counts)
    std_qb = statistics.stdev(counts) if len(counts) > 1 else 0
    print(f"{sh:>6} {mean_qb:>10.1f} {std_qb:>10.1f} {'':>12}")

sh0_mean = statistics.mean([qb_count(1, 1400, 5000, 5000, 0, s) for s in SEEDS])
sh1_mean = statistics.mean([qb_count(1, 1400, 5000, 5000, 1, s) for s in SEEDS])
reduction = (sh0_mean - sh1_mean) / sh0_mean * 100 if sh0_mean > 0 else 0
print(
    f"\n  Reduction: {sh0_mean:.1f} -> {sh1_mean:.1f} ({reduction:.0f}% fewer G2_queue_caps)",
)
print(
    f"  {'PASS: gate eliminates noise-triggered G2_queue_caps' if sh1_mean < 1 else 'WARN: some G2_queue_caps still occur'}",
)

# Test: baseline shift (qdelay=0) -- gate should NOT block genuine path change
print("\n--- Baseline shift: 1.4ms->50ms, qdelay=0 ---")
for sh in [0, 1]:
    # Simulate a sudden shift
    random.seed(42)
    fl = KCCFlow()
    fl.x_est = 1400 * SCALE
    fl.min_rtt_us = 1400
    fl.p_est = 10  # converged
    qb_fired = False
    for rd in range(10):
        tp = 50000 if rd >= 0 else 1400
        rtt = tp
        z = rtt * SCALE
        nu = abs(z - fl.x_est)
        thresh = max(fl.x_est >> (10 + sh), 1)
        if nu > QBOOST_THRESH_US * SCALE and fl.qdelay_ewma < thresh:
            qb_fired = True
            fl.x_est = z
            break
    print(
        f"  shift={sh}: G2_queue_cap {'fired' if qb_fired else 'BLOCKED'} "
        f"{'PASS' if qb_fired else 'FAIL (deadlock!)'}",
    )

print("\nDone.")
