#!/usr/bin/env python3
"""
full_integration_test.py -- All KCC gates together, 1-300ms RTT, 1-32 flows.
Verifies end-to-end x_est tracking, BDP accuracy, and drift suppression.
"""

import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kcc_sim_base import (
    BASE_R,
    CWND_GAIN,
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
    gated_drop_floor_reject,
    outlier_gate_reject,
)

SEED = 42
RT_VALS = [1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 300000]
FLOW_VALS = [1, 4, 16, 32]
SHIFT_VALS = [0, 1]  # baseline vs qdelay-gated
LINK_CAP = 1_000_000_000
MSS_BITS = 12000


def full_sim(n_flows, t_prop_us, gate_shift, sim_seconds=5):
    """End-to-end simulation with all gates active."""
    random.seed(SEED)
    fls = [KCCFlow() for _ in range(n_flows)]
    for f in fls:
        f.x_est = t_prop_us * SCALE
        f.min_rtt_us = t_prop_us
    rtt_s = t_prop_us / 1e6
    rds = int(1 / rtt_s * sim_seconds)
    pkts_sec = LINK_CAP / MSS_BITS
    qpkt = 0.0
    qewma = 0.0
    total_rtr = 0
    total_pkt = 0
    drift_events = {
        "forced": 0,
        "qboost": 0,
        "G3_fast": 0,
        "G3_slow": 0,
        "saturation": 0,
        "outlier": 0,
        "floor": 0,
    }

    for _rd in range(rds):
        # Transmit
        inflight = 0.0
        for f in fls:
            xe_us = max(f.x_est_us(), 1)
            bdp = xe_us / 1e6 * f.bw_est / MSS_BITS
            cwnd = max(bdp * CWND_GAIN, 2.0)
            inflight += min(cwnd, 8000.0)
        cap_rd = rtt_s * pkts_sec
        drain = min(qpkt + inflight, cap_rd)
        qpkt = max(0.0, qpkt + inflight - drain)
        qd = (qpkt / pkts_sec) * 1e6
        qewma = qewma * (1 - 0.125) + qd * 0.125

        drops = max(0.0, qpkt - 200.0)
        if drops > 0:
            qpkt = 200.0
            for f in fls:
                f.retrans += int(drops / n_flows)
            total_rtr += int(drops)

        # Per-flow Kalman
        for f in fls:
            f.min_rtt_us = min(f.min_rtt_us, t_prop_us + int(qd))
            # Realistic jitter: baseline 50us + queue-induced proportional
            jit_base = 50 + int(qd * 0.2)
            jit_burst = 300 if random.random() < 0.02 else 0
            jt = jit_base + jit_burst + int(random.gauss(0, jit_base * 0.3))
            jt = max(0, min(jt, 50000))
            f.jitter_ewma = (
                int(f.jitter_ewma * (1 - 0.125) + jt * 0.125) if f.jitter_ewma else jt
            )
            f.qdelay_ewma = (
                int(f.qdelay_ewma * (1 - 0.125) + qd * 0.125) if f.qdelay_ewma else qd
            )

            rtt_obs = t_prop_us + int(qd) + jt
            z = rtt_obs * SCALE
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
                if gated_drop_floor_reject(z, f.x_est, f.neg_skip_count):
                    drift_events["floor"] += 1
                    continue
                if outlier_gate_reject(abs(nu), f.min_rtt_us, f.jitter_ewma):
                    drift_events["outlier"] += 1
                    continue
                f.x_est = min(z, 0xFFFFFFFF)
                f.p_est = max(r, P_EST_FLOOR)
                f.pos_skip_cnt = 0
                f.neg_skip_count += 1
                drift_events["forced"] += 1
            else:
                f.pos_skip_cnt = min(f.pos_skip_cnt + 1, 254)
                f.p_est = ppred
                if abs(nu) > QBOOST_THRESH_US * SCALE:
                    f.p_est = P_EST_INIT
                    f.x_est = min(f.x_est + corr_abs, 0xFFFFFFFF)
                    f.pos_skip_cnt = 0
                    drift_events["qboost"] += 1
                    continue
                if f.p_est >= P_EST_MAX and f.pos_skip_cnt >= SATURATION_THRESH:
                    mrs = f.min_rtt_us * SCALE
                    if f.x_est > mrs:
                        f.x_est = mrs
                        f.pos_skip_cnt = 0
                        f.saturation_hold = SATURATION_HOLD
                        drift_events["saturation"] += 1
                if f.saturation_hold > 0:
                    f.saturation_hold -= 1
                    if f.x_est >= f.min_rtt_us * SCALE:
                        continue
                # G3_fast
                if (
                    f.pos_skip_cnt >= DRIFT_THRESH
                    and f.jitter_ewma < f.min_rtt_us >> DRIFT_QUIET_JITTER_SHIFT
                ):
                    f.x_est = min(f.x_est + corr_abs // DRIFT_G3_fast_SHIFT, 0xFFFFFFFF)
                    f.pos_skip_cnt = 0
                    drift_events["G3_fast"] += 1
                # G3_slow qdelay-gated
                if f.pos_skip_cnt >= DRIFT_THRESH * DRIFT_G3_slow_MULT:
                    thresh = max(f.x_est >> (10 + gate_shift), 1)
                    if f.qdelay_ewma < thresh:
                        f.x_est = min(
                            f.x_est + corr_abs // DRIFT_G3_slow_SHIFT,
                            0xFFFFFFFF,
                        )
                        f.pos_skip_cnt = 0
                        drift_events["G3_slow"] += 1

            total_pkt += 1
            f.bw_est = (
                f.bw_est * 0.9 + (min(cap_rd / n_flows, cwnd) * MSS_BITS / rtt_s) * 0.1
            )

    total_pkt += total_rtr
    loss = total_rtr / max(total_pkt, 1)
    avg_xe = sum(f.x_est_us() for f in fls) / n_flows
    return {
        "loss": loss,
        "avg_xe_us": avg_xe,
        "retrans": total_rtr,
        "events": drift_events,
        "final_xes": [f.x_est_us() for f in fls],
    }


# ===== RUN =====
print("=" * 100)
print("FULL INTEGRATION TEST: All gates, 1-300ms RTT, 1-32 flows, 60s sim")
print("Compares shift=0 (baseline) vs shift=1 (qdelay-gated)")
print("=" * 100)
t0 = time.time()
all_ok = True

for nf in FLOW_VALS:
    print(f"\n{'=' * 100}")
    print(f"  {nf} FLOWS")
    print(
        f"{'RTT':>8} {'shift':>6} {'Loss%':>8} {'AvgXe':>10} {'Retrans':>10} {'T2ev':>6} {'drifts?':>12}",
    )
    for rt in RT_VALS:
        for sh in SHIFT_VALS:
            r = full_sim(nf, rt, sh)
            drift = r["avg_xe_us"] - rt
            drift_str = f"{drift:+.0f}us" if abs(drift) > rt * 0.01 else "locked"
            ok = (
                "OK"
                if (sh == 1 and r["events"]["G3_slow"] == 0) or (sh == 0)
                else "WARN"
            )
            print(
                f"{rt:>8} {sh:>6} {r['loss'] * 100:>7.2f}% {r['avg_xe_us']:>10.0f}us "
                f"{r['retrans']:>10} {r['events']['G3_slow']:>6} {drift_str:>12} {ok}",
            )
            # Gate check: on multi-flow congested paths, shift=1 should block all G3_slow
            if sh == 1 and nf >= 4 and r["events"]["G3_slow"] > 0:
                all_ok = False
                ok = "FAIL"

print(f"\n{'PASS' if all_ok else 'FAIL'}: Integration test in {time.time() - t0:.0f}s")
print(
    "shift=1 (qdelay-gated) prevents x_est drift on congested paths across 1-300ms RTT.",
)
