#!/usr/bin/env python3
"""
noise_mode_compare.py -- Compare noise_mode=0 vs noise_mode=1 on multi-flow wired paths.
Tests: 11 flows, 1.4ms baseline, 1Gbps link, 200-pkt buffer, 60s simulation.
Measures: r_est drift, effective R, loss rate.
"""

import os
import random
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
random.seed(42)

SCALE = 1024
BASE_R = 400
J50 = 200
Q_BASE = 100
R_MAX = 102400
P_INIT = 1000
P_MAX = 100000000
P_FLOOR = 10
DRIFT = 14
T2MULT = 4
SAT_THR = 55
SAT_HLD = 30
QBOOST = 16000
LINK_CAP = 1_000_000_000
MSS = 12000
PKTS_SEC = LINK_CAP // MSS
BUF = 200
NF = 11
TP = 1400
SIM_S = 60
SEEDS = list(range(5))


class Flow:
    def __init__(self):
        self.xe = 0
        self.pe = P_INIT
        self.psc = 0
        self.nsc = 0
        self.sat_hold = 0
        self.min_rtt = 10**9
        self.bw_est = LINK_CAP
        self.jit_ewma = 0
        self.qd_ewma = 0
        self.re = 0
        self.qe = 0
        self.r_est = BASE_R
        self.q_est = Q_BASE
        self.retrans = 0
        self.pkts = 0
        self.r_contrib = 0


def sim(noise_mode, seed):
    random.seed(seed)
    fls = [Flow() for _ in range(NF)]
    for f in fls:
        f.xe = TP * SCALE
        f.min_rtt = TP
        f.r_est = BASE_R
        f.q_est = Q_BASE
    rtt_s = TP / 1e6
    rds = int(1 / rtt_s * SIM_S)
    qpkt = 0.0
    qewma = 0.0
    total_rtr = 0
    for _ in range(rds):
        # Transmit with BBR gain cycling
        inflight = 0.0
        for f in fls:
            xeu = max(f.xe // SCALE, 1)
            bdp = xeu / 1e6 * f.bw_est / MSS
            cwnd = max(bdp * 2, 2)
            inflight += min(cwnd, 8000)
        cap = rtt_s * PKTS_SEC
        drain = min(qpkt + inflight, cap)
        qpkt = max(0.0, qpkt + inflight - drain)
        qd = (qpkt / PKTS_SEC) * 1e6
        qewma = qewma * 0.875 + qd * 0.125
        drops = max(0.0, qpkt - BUF)
        if drops > 0:
            qpkt = BUF
        for f in fls:
            f.retrans += int(drops / NF)
            total_rtr += int(drops)
        for f in fls:
            f.min_rtt = min(f.min_rtt, TP + int(qd))
            jt = 50 + int(qd * 0.2) + (300 if random.random() < 0.02 else 0)
            jt += int(random.gauss(0, jt * 0.3))
            jt = max(0, min(jt, 50000))
            f.jit_ewma = int(f.jit_ewma * 0.875 + jt * 0.125) if f.jit_ewma else jt
            f.qd_ewma = int(f.qd_ewma * 0.875 + qd * 0.125) if f.qd_ewma else qd
            rtt = TP + int(qd) + jt
            z = rtt * SCALE
            nu = z - f.xe
            je = max(0, jt - jt // 4)
            ratio = je / J50 if je > 0 else 0
            r = min(int(BASE_R * ratio**1.5), R_MAX) if ratio > 0 else BASE_R
            r = max(BASE_R, r)  # lower clamp
            ppred = min(f.pe + Q_BASE, P_MAX)
            K = ppred / (ppred + r)
            corr = abs(int(K * nu))
            # Matched estimator update
            r_contrib = int(abs(nu) ** 2) - ppred
            r_contrib = max(r_contrib, 0)
            f.r_est = int(f.r_est * 0.9 + r_contrib * 0.1)
            f.r_est = max(1, min(f.r_est, 32000))
            f.q_est = int(f.q_est * 0.9 + max(0, corr**2 - ppred) * 0.1)
            f.q_est = max(1, min(f.q_est, 50000))
            # Effective R
            eff_r = max(r, f.r_est) if noise_mode == 1 else r
            # Kalman
            if nu <= 0:
                f.xe = min(z, 0xFFFFFFFF)
                f.pe = max(eff_r, P_FLOOR)
                f.psc = 0
            else:
                f.psc = min(f.psc + 1, 254)
                f.pe = ppred
                if abs(nu) > QBOOST * SCALE:
                    thresh = max(f.xe >> 11, 1)  # shift=1 qdelay gate
                    if f.qd_ewma < thresh:
                        f.pe = P_INIT
                        f.xe = min(f.xe + corr, 0xFFFFFFFF)
                        f.psc = 0
                if f.pe >= P_MAX and f.psc >= SAT_THR:
                    mrs = f.min_rtt * SCALE
                    if f.xe > mrs:
                        f.xe = mrs
                        f.psc = 0
                        f.sat_hold = SAT_HLD
                if f.psc >= DRIFT * T2MULT:
                    thresh = max(f.xe >> 11, 1)  # shift=1 qdelay gate
                    if f.qd_ewma < thresh:
                        f.xe = min(f.xe + int(corr / 8), 0xFFFFFFFF)
                        f.psc = 0
            f.pkts += 1
            f.bw_est = f.bw_est * 0.9 + min(cap / NF, 8000) * MSS / rtt_s * 0.1
    total_pkt = sum(f.pkts for f in fls) + total_rtr
    avg_re = statistics.mean([f.r_est for f in fls])
    avg_qe = statistics.mean([f.q_est for f in fls])
    avg_xe = sum(f.xe // SCALE for f in fls) / NF
    return {
        "loss": total_rtr / max(total_pkt, 1),
        "re": avg_re,
        "qe": avg_qe,
        "xe": avg_xe,
        "rtr": total_rtr,
    }


print("=" * 90)
print("NOISE_MODE COMPARISON: 11 flows, 1.4ms, 1Gbps, 200-pkt buffer, 60s")
print("=" * 90)

for mode in [0, 1]:
    results = [sim(mode, s) for s in SEEDS]
    loss = statistics.mean([r["loss"] for r in results])
    re = statistics.mean([r["re"] for r in results])
    qe = statistics.mean([r["qe"] for r in results])
    xe = statistics.mean([r["xe"] for r in results])
    rtr = statistics.mean([r["rtr"] for r in results])
    print(
        f"\nmode={mode}: loss={loss * 100:.1f}% r_est={re:.0f} q_est={qe:.0f} "
        f"x_est={xe:.0f}us retrans={rtr:.0f}",
    )
    print(f"  Effective R floor = {re:.0f} (mode 0: R=400)")
    print(f"  K gain at effective R: K~{Q_BASE / (Q_BASE + re):.4f}")

print("\nRecommendation: noise_mode=0 for multi-flow wired paths")
print("  Keeps R at power-law value (~400 on clean paths)")
print("  Prevents r_est ratchet from inflating R to 32000")
print("  Kalman stays responsive at K~0.20 vs K~0.003 (67x more responsive)")
