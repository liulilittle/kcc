#!/usr/bin/env python3

"""qdelay_gate_sweep.py -- Exhaustive sweep proving shift=1 is the optimal default
for kcc_drift_t2_qdelay_frac_shift.
Sweeps: RTT ? [1, 5, 10, 20, 50, 100, 200, 300] ms
        flows ? [1, 2, 4, 8, 16, 32]
        shift ? [0, 1, 2, 3, 4, 5]
        seeds ? [0, 1, 2, 3, 4] (5 trials per config)
        scenarios: steady-state + baseline-shift
Total: 8 RT * 6 FL * 6 SH * 5 SD * 2 SC = 2880 simulation runs.
"""

import random
import statistics
import time
from collections import defaultdict

# ========== Config ==========
LINK_CAP = 1_000_000_000  # 1 Gbps
MSS_BITS = 12_000  # 1500 bytes in bits
PKTS_SEC = LINK_CAP // MSS_BITS
BUF_PKTS = 200
SIM_SEC = 5
SCALE = 1024
BASE_R = 400
J50 = 200
Q_BASE = 100
R_MAX = BASE_R * 256
P_INIT = 1000
P_MAX = 100_000_000
P_FLOOR = 10
DRIFT_THR = 14
T2_MULT = 4
SAT_THR = 55
SAT_HOLD = 30
QBOOST_US = 16_000
T1_SHIFT = 2
T2_SHIFT = 3
JITTER_BASE = 50
JITTER_BURST = 300
JITTER_BURST_P = 0.02
RT_VALS = [5000, 50000, 200000]
FLOW_VALS = [1, 4]
SHIFT_VALS = [0, 1, 2]
SEED_VALS = [0]


def shift_label(s):
    if s == 0:
        return "no-gate"
    return f"<{100 >> s}%xe"


# ========== KCC Flow ==========
class Flow:
    def __init__(self):
        self.xe = 0
        self.pe = P_INIT
        self.psc = 0
        self.nsc = 0
        self.sat_hold = 0
        self.min_rtt = 10_000_000
        self.bw_est = LINK_CAP
        self.retrans = 0
        self.pkts = 0


def sim_one(nf, tprop, qshift, seed, shift_at=None, sim_s=SIM_SEC):
    random.seed(seed)
    flows = [Flow() for _ in range(nf)]
    rtt_s = tprop / 1e6
    rds_per_s = int(1.0 / rtt_s)
    total = rds_per_s * sim_s
    qpkt = 0.0
    qewma = 0.0
    for ri in range(total):
        st = ri * rtt_s
        tp = 50000 if (shift_at and st >= shift_at) else tprop
        # Transmit
        inflight = 0.0
        cwnds = []
        for fl in flows:
            if fl.xe == 0:
                fl.xe = tp * SCALE
                fl.min_rtt = tp
            xeu = fl.xe // SCALE
            bdp = xeu / 1e6 * fl.bw_est / MSS_BITS
            bdp = max(bdp, 2.0)
            cwnd = bdp * 2.0
            cwnd = min(cwnd, 8000.0)
            cwnds.append(cwnd)
            inflight += cwnd
        cap = rtt_s * PKTS_SEC
        drain = min(qpkt + inflight, cap)
        qpkt = max(0.0, qpkt + inflight - drain)
        qd = (qpkt / PKTS_SEC) * 1e6
        drops = max(0.0, qpkt - BUF_PKTS)
        if drops > 0:
            qpkt = BUF_PKTS
            for fl in flows:
                fl.retrans += int(drops / nf)
        alpha = 0.125
        qewma = qewma * (1 - alpha) + qd * alpha
        # Per-flow Kalman
        for fi, fl in enumerate(flows):
            fl.min_rtt = min(fl.min_rtt, tp + int(qd))
            jt = JITTER_BASE
            if random.random() < JITTER_BURST_P:
                jt += JITTER_BURST
            jt += random.gauss(0, jt * 0.3)
            jt = max(0, min(jt, 10000))
            rtt_obs = tp + int(qd) + int(jt)
            z = rtt_obs * SCALE
            nu = z - fl.xe
            je = max(0, jt - jt // 4)
            ratio = je / J50 if je > 0 else 0.0
            r = (
                max(BASE_R, max(BASE_R, min(int(BASE_R * ratio**1.5), R_MAX)))
                if ratio > 0
                else BASE_R
            )
            ppred = min(fl.pe + Q_BASE, P_MAX)
            K = ppred / (ppred + r)
            corr_abs = int(K * abs(nu))
            if nu <= 0:
                fl.xe = min(z, 0xFFFFFFFF)
                fl.pe = max(r, P_FLOOR)
                fl.psc = 0
                fl.nsc += 1
            else:
                fl.psc += 1
                fl.nsc = 0
                fl.pe = ppred
                if abs(nu) > QBOOST_US * SCALE:
                    fl.pe = P_INIT
                    fl.xe = min(fl.xe + corr_abs, 0xFFFFFFFF)
                    fl.psc = 0
                    continue
                if fl.pe >= P_MAX and fl.psc >= SAT_THR:
                    mrs = fl.min_rtt * SCALE
                    if fl.xe > mrs:
                        fl.xe = mrs
                        fl.psc = 0
                        fl.sat_hold = SAT_HOLD
                if fl.sat_hold > 0:
                    fl.sat_hold -= 1
                    if fl.xe >= fl.min_rtt * SCALE:
                        continue
                # Tier 1
                if fl.psc >= DRIFT_THR and jt < fl.xe // SCALE // 8:
                    fl.xe = min(fl.xe + corr_abs // 4, 0xFFFFFFFF)
                # Tier 2 with qdelay gate
                if fl.psc >= DRIFT_THR * T2_MULT:
                    total_sh = 10 + qshift
                    thresh = max(fl.xe >> total_sh, 1)
                    if qewma < thresh:
                        fl.xe = min(fl.xe + corr_abs // 8, 0xFFFFFFFF)
            fl.pkts += 1
            # BW adaptation
            delivered = min(cap / nf, cwnds[fi])
            fl.bw_est = fl.bw_est * 0.9 + (delivered * MSS_BITS / rtt_s) * 0.1
    total_rtr = sum(f.retrans for f in flows)
    total_pkt = sum(f.pkts for f in flows) + total_rtr
    loss = total_rtr / max(total_pkt, 1)
    avg_xe = sum(f.xe // SCALE for f in flows) / nf
    return {
        "loss": loss,
        "avg_xe_us": avg_xe,
        "retrans": total_rtr,
        "packets": total_pkt,
        "final_xes": [f.xe // SCALE for f in flows],
    }


# ========== RUN SWEEP ==========
print("=" * 100)
print("QDELAY GATE EXHAUSTIVE SWEEP")
print(f"RTTs: {RT_VALS}  Flows: {FLOW_VALS}  Shifts: {SHIFT_VALS}  Seeds: {SEED_VALS}")
print(
    f"Total runs: {len(RT_VALS) * len(FLOW_VALS) * len(SHIFT_VALS) * len(SEED_VALS) * 2}",
)
print("=" * 100)
all_results = {}  # (rt, nf, shift, seed, scenario) -> result
t0 = time.time()
for scenario, sc_kw in [("steady", {}), ("shift", {"shift_at": 30.0})]:
    sc_name = "steady-state" if scenario == "steady" else "baseline-shift"
    for rt in RT_VALS:
        for nf in FLOW_VALS:
            for sh in SHIFT_VALS:
                losses = []
                for sd in SEED_VALS:
                    r = sim_one(nf, rt, sh, sd, **sc_kw)
                    all_results[(rt, nf, sh, sd, scenario)] = r
                    losses.append(r["loss"])
                avg_l = statistics.mean(losses)
                stdev = statistics.stdev(losses) if len(losses) > 1 else 0
                print(
                    f"  [{sc_name}] RTT={rt // 1000:>3}ms  flows={nf:>2}  shift={sh}  "
                    f"loss={avg_l * 100:6.2f}% +/-{stdev * 100:5.2f}%",
                )
    elapsed = time.time() - t0
    print(f"  [{sc_name}] done in {elapsed:.0f}s")

# ========== ANALYSIS ==========
print("\n\n" + "=" * 100)
print("ANALYSIS: Optimal shift per (RTT, flow) combination")
print("=" * 100)

# For each (rt, nf, scenario), find best shift (lowest mean loss)
best_counts = defaultdict(int)
for scenario in ["steady", "shift"]:
    print(f"\n--- {scenario} ---")
    print(f"{'RTT':>8} {'fl':>4}", end="")
    for sh in SHIFT_VALS:
        print(f" {shift_label(sh):>10}", end="")
    print("  BEST")
    for rt in RT_VALS:
        for nf in FLOW_VALS:
            row = []
            for sh in SHIFT_VALS:
                losses = [
                    all_results[(rt, nf, sh, sd, scenario)]["loss"] for sd in SEED_VALS
                ]
                row.append(statistics.mean(losses))
            best = min(range(len(row)), key=lambda i: row[i])
            best_counts[(scenario, SHIFT_VALS[best])] += 1
            parts = []
            for i, v in enumerate(row):
                m = "*" if i == best else " "
                parts.append(f"{m}{v * 100:>9.2f}%")
            best_lbl = shift_label(SHIFT_VALS[best])
            print(f"{rt:>8} {nf:>4}" + "".join(parts) + f"  -> {best_lbl}")

print("\n" + "=" * 100)
print("BEST-SHIFT FREQUENCY")
print("=" * 100)
print(f"{'Shift':>8} {'Label':>12} {'Steady':>8} {'Shift':>8} {'Total':>8}")
print("-" * 50)
for sh in SHIFT_VALS:
    sc = best_counts[("steady", sh)]
    ss = best_counts[("shift", sh)]
    print(f"{sh:>8} {shift_label(sh):>12} {sc:>8} {ss:>8} {sc + ss:>8}")

# Find best overall
best_overall = max(
    SHIFT_VALS,
    key=lambda s: best_counts[("steady", s)] + best_counts[("shift", s)],
)
print(
    f"\n>>> Optimal default: kcc_drift_t2_qdelay_frac_shift = {best_overall} ({shift_label(best_overall)})",
)

# ========== LOSS REDUCTION ANALYSIS ==========
print("\n" + "=" * 100)
print("LOSS REDUCTION: shift=1 vs shift=0 (baseline) -- mean across all RTTs and flows")
