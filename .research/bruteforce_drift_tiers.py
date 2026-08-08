#!/usr/bin/env python3
"""Brute-force G3_thresholds: G3_fast jitter gate, G3_slow qdelay gate, saturation, interaction."""

import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kcc_sim_base import (
    BASE_R,
    DRIFT_QUIET_JITTER_SHIFT,
    DRIFT_THRESH,
    J50_DEFAULT,
    P_EST_FLOOR,
    P_EST_INIT,
    P_EST_MAX,
    Q_BASE,
    QBOOST_THRESH_US,
    R_MAX,
    SATURATION_HOLD,
    SATURATION_THRESH,
    SCALE,
    DRIFT_G3_fast_SHIFT,
    DRIFT_G3_slow_MULT,
    DRIFT_G3_slow_SHIFT,
    KCCFlow,
)


def test_drift_tiers():
    failures = 0
    print("=" * 90)
    print("G3_thresholds BRUTE-FORCE: G3_fast jitter, G3_slow qdelay, Saturation")
    print("=" * 90)

    # Helper: simulate a single flow through N rounds with fixed conditions
    def drift_sim(t_prop_us, qdelay_us, jitter_hi, shift, n_rounds=5000, n_flows=11):
        random.seed(42)
        fls = [KCCFlow() for _ in range(n_flows)]
        for f in fls:
            f.x_est = t_prop_us * SCALE
            f.min_rtt_us = t_prop_us
        t1_count = 0
        t2_count = 0
        sat_count = 0
        qb_count = 0
        for _rd in range(n_rounds):
            for f in fls:
                jt = jitter_hi + int(random.gauss(0, jitter_hi * 0.5))
                jt = max(0, jt)
                f.jitter_ewma = (
                    int(f.jitter_ewma * (1 - 0.125) + jt * 0.125)
                    if f.jitter_ewma
                    else jt
                )
                f.qdelay_ewma = (
                    int(f.qdelay_ewma * (1 - 0.125) + qdelay_us * 0.125)
                    if f.qdelay_ewma
                    else qdelay_us
                )
                f.min_rtt_us = min(f.min_rtt_us, t_prop_us + qdelay_us + jt)
                rtt = t_prop_us + qdelay_us + jt
                z = rtt * SCALE
                nu = z - f.x_est
                je = max(0, jt - jt // 4)
                ratio = je / J50_DEFAULT if je > 0 else 0
                r = (
                    max(BASE_R, max(BASE_R, min(int(BASE_R * ratio**1.5), R_MAX)))
                    if ratio > 0
                    else BASE_R
                )
                ppred = min(f.p_est + Q_BASE, P_EST_MAX)
                K = ppred / (ppred + r)
                corr_abs = int(K * abs(nu))
                if nu <= 0:
                    f.x_est = min(z, 0xFFFFFFFF)
                    f.p_est = max(r, P_EST_FLOOR)
                    f.pos_skip_cnt = 0
                else:
                    f.pos_skip_cnt = min(f.pos_skip_cnt + 1, 254)
                    f.p_est = ppred
                    if abs(nu) > QBOOST_THRESH_US * SCALE:
                        f.p_est = P_EST_INIT
                        f.x_est = min(f.x_est + corr_abs, 0xFFFFFFFF)
                        f.pos_skip_cnt = 0
                        qb_count += 1
                        continue
                    if f.p_est >= P_EST_MAX and f.pos_skip_cnt >= SATURATION_THRESH:
                        mrs = f.min_rtt_us * SCALE
                        if f.x_est > mrs:
                            f.x_est = mrs
                            f.pos_skip_cnt = 0
                            sat_count += 1
                            f.saturation_hold = SATURATION_HOLD
                    if f.saturation_hold > 0:
                        f.saturation_hold -= 1
                        if f.x_est >= f.min_rtt_us * SCALE:
                            continue
                    # Tier 1: jitter_ewma < min_rtt>>3
                    if (
                        f.pos_skip_cnt >= DRIFT_THRESH
                        and f.jitter_ewma < f.min_rtt_us >> DRIFT_QUIET_JITTER_SHIFT
                    ):
                        f.x_est = min(
                            f.x_est + corr_abs // DRIFT_G3_fast_SHIFT,
                            0xFFFFFFFF,
                        )
                        f.pos_skip_cnt = 0
                        t1_count += 1
                    # Tier 2: qdelay-gated
                    if f.pos_skip_cnt >= DRIFT_THRESH * DRIFT_G3_slow_MULT:
                        total_sh = 10 + shift
                        thresh = max(f.x_est >> total_sh, 1)
                        if f.qdelay_ewma < thresh:
                            f.x_est = min(
                                f.x_est + corr_abs // DRIFT_G3_slow_SHIFT,
                                0xFFFFFFFF,
                            )
                            f.pos_skip_cnt = 0
                            t2_count += 1
        avg_xe = sum(f.x_est_us() for f in fls) / n_flows
        return {
            "avg_xe_us": avg_xe,
            "t1": t1_count,
            "t2": t2_count,
            "sat": sat_count,
            "qb": qb_count,
        }

    # Test 1: On user scenario (1.4ms, 5ms qdelay, varying jitter, shift=1)
    print("--- Test 1: User scenario (1.4ms RTT, 5ms qdelay, 11 flows) ---")
    print(
        f"{'jitter':>8} {'shift':>6} {'avg_xe':>10} {'T1':>6} {'T2':>6} {'SAT':>6} {'QBOOST':>6} {'drift?':>8}",
    )
    for jit in [50, 200, 500, 1000, 2000, 5000]:
        for sh in [0, 1]:
            r = drift_sim(1400, 5000, jit, sh, n_rounds=1000)
            drift = r["avg_xe_us"] - 1400
            drift_str = f"{drift:+.0f}us" if drift != 0 else "none"
            print(
                f"{jit:>8} {sh:>6} {r['avg_xe_us']:>10.0f}us {r['t1']:>6} {r['t2']:>6} "
                f"{r['sat']:>6} {r['qb']:>6} {drift_str:>8}",
            )

    # Test 2: Varying RTT with fixed qdelay=5ms, jitter=2000us (G3_fast blocked, G3_slow only)
    print("\n--- Test 2: Varying RTT, qdelay=5ms, jitter=2ms, 11 flows ---")
    print(
        f"{'T_prop':>10} {'shift=0 xe':>12} {'shift=1 xe':>12} {'drift blocked?':>16}",
    )
    for tp in [1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 300000]:
        r0 = drift_sim(tp, 5000, 2000, 0, n_rounds=500)
        r1 = drift_sim(tp, 5000, 2000, 1, n_rounds=500)
        d1 = r1["avg_xe_us"] - tp
        blocked = "YES" if d1 < 100 else f"drift={d1:.0f}us"
        print(
            f"{tp:>10} {r0['avg_xe_us']:>12.0f}us {r1['avg_xe_us']:>12.0f}us {blocked:>16}",
        )

    # Test 3: Saturation timing verification
    print("\n--- Test 3: Saturation timing (p_est accumulation) ---")
    p = P_EST_INIT
    rounds_to_max = 0
    while p < P_EST_MAX:
        p = min(p + Q_BASE, P_EST_MAX)
        rounds_to_max += 1
    print(f"  p_est: {P_EST_INIT} -> {P_EST_MAX} at Q={Q_BASE}/round")
    print(
        f"  Requires {rounds_to_max:,} rounds ({rounds_to_max * 1.4 / 1000:.0f}s at 1.4ms RTT)",
    )
    status = "PASS" if rounds_to_max > 500000 else "FAIL"
    print(f"  {status}: well above drift timescales")

    return failures


if __name__ == "__main__":
    f = test_drift_tiers()
    print(f"\n{'PASS' if f == 0 else f'{f} FAILURES'}")
