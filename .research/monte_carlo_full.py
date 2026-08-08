#!/usr/bin/env python3

"""monte_carlo_full.py -- Full multi-seed Monte Carlo verification.
10 random seeds per scenario, 1-300ms RTT, 1-16 flows.
Statistical confidence in drift suppression with shift=1."""

import os
import random
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kcc_sim_base import (
    BASE_R,
    CWND_GAIN,
    DRIFT_THRESH,
    J50_DEFAULT,
    P_EST_FLOOR,
    P_EST_INIT,
    P_EST_MAX,
    Q_BASE,
    QBOOST_THRESH_US,
    R_MAX,
    SATURATION_THRESH,
    SCALE,
    DRIFT_G3_slow_MULT,
    DRIFT_G3_slow_SHIFT,
    KCCFlow,
    gated_drop_floor_reject,
    outlier_gate_reject,
)

RT_VALS = [1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 300000]
FLOW_VALS = [1, 4, 16]
SEEDS = list(range(10))
LINK_CAP = 1000000000
MSS_BITS = 12000


def mc_sim(nf, tp, sh, seed, sim_s=5):
    random.seed(seed)
    fls = [KCCFlow() for _ in range(nf)]
    for f in fls:
        f.x_est = tp * SCALE
        f.min_rtt_us = tp
    rtt_s = tp / 1e6
    rds = int(1 / rtt_s * sim_s)
    pkts_sec = LINK_CAP / MSS_BITS
    qpkt = 0.0
    qewma = 0.0
    t2_count = 0
    for _ in range(rds):
        inflight = sum(
            max(f.x_est_us() / 1e6 * f.bw_est / MSS_BITS * CWND_GAIN, 2) for f in fls
        )
        inflight = min(inflight, 8000 * nf)
        cap = rtt_s * pkts_sec
        qpkt = max(0.0, qpkt + inflight - min(qpkt + inflight, cap))
        qd = (qpkt / pkts_sec) * 1e6
        qewma = qewma * (1 - 0.125) + qd * 0.125
        for f in fls:
            f.min_rtt_us = min(f.min_rtt_us, tp + int(qd))
            jit = 50 + int(qd * 0.2) + (300 if random.random() < 0.02 else 0)
            jit += int(random.gauss(0, jit * 0.3))
            jit = max(0, min(jit, 50000))
            f.jitter_ewma = (
                int(f.jitter_ewma * (1 - 0.125) + jit * 0.125) if f.jitter_ewma else jit
            )
            f.qdelay_ewma = (
                int(f.qdelay_ewma * (1 - 0.125) + qd * 0.125) if f.qdelay_ewma else qd
            )
            rtt = tp + int(qd) + jit
            z = rtt * SCALE
            nu = z - f.x_est
            je = max(0, jit - jit // 4)
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
                if gated_drop_floor_reject(z, f.x_est, f.neg_skip_count):
                    continue
                if outlier_gate_reject(abs(nu), f.min_rtt_us, f.jitter_ewma):
                    continue
                f.x_est = min(z, 0xFFFFFFFF)
                f.p_est = max(r, P_EST_FLOOR)
                f.pos_skip_cnt = 0
                f.neg_skip_count += 1
            else:
                f.pos_skip_cnt = min(f.pos_skip_cnt + 1, 254)
                f.p_est = ppred
                if abs(nu) > QBOOST_THRESH_US * SCALE:
                    f.p_est = P_EST_INIT
                    f.x_est = min(f.x_est + corr_abs, 0xFFFFFFFF)
                    f.pos_skip_cnt = 0
                    continue
                if f.p_est >= P_EST_MAX and f.pos_skip_cnt >= SATURATION_THRESH:
                    mrs = f.min_rtt_us * SCALE
                    if f.x_est > mrs:
                        f.x_est = mrs
                        f.pos_skip_cnt = 0
                if f.pos_skip_cnt >= DRIFT_THRESH * DRIFT_G3_slow_MULT:
                    thresh = max(f.x_est >> (10 + sh), 1)
                    if f.qdelay_ewma < thresh:
                        f.x_est = min(
                            f.x_est + corr_abs // DRIFT_G3_slow_SHIFT,
                            0xFFFFFFFF,
                        )
                        f.pos_skip_cnt = 0
                        t2_count += 1
            f.bw_est = (
                f.bw_est * 0.9 + (min(cap / nf, inflight / nf) * MSS_BITS / rtt_s) * 0.1
            )
    return {"t2_count": t2_count, "avg_xe": sum(f.x_est_us() for f in fls) / nf}


print("=" * 90)
print("MONTE CARLO: 10 seeds, 1-300ms RTT, 1-16 flows, shift=0 vs shift=1")
print("=" * 90)
t0 = time.time()
all_pass = True
for nf in FLOW_VALS:
    print(f"\n--- {nf} flows ---")
    print(
        "{:>8} {:>12} {:>12} {:>12}".format(
            "RTT",
            "shift=0 T2",
            "shift=1 T2",
            "T2 blocked?",
        ),
    )
    for rt in RT_VALS:
        t2_0 = statistics.mean([mc_sim(nf, rt, 0, s)["t2_count"] for s in SEEDS])
        t2_1 = statistics.mean([mc_sim(nf, rt, 1, s)["t2_count"] for s in SEEDS])
        blocked = "YES" if t2_1 < 0.5 else f"NO({t2_1:.0f})"
        ok = "OK" if t2_1 < 0.5 or nf == 1 else "FAIL"
        if ok == "FAIL":
            all_pass = False
        print(f"{rt:>8} {t2_0:>12.1f} {t2_1:>12.1f} {blocked:>12} {ok:>4}")
print(
    "\n{}: Monte Carlo in {:.0f}s".format(
        "PASS" if all_pass else "FAIL",
        time.time() - t0,
    ),
)
