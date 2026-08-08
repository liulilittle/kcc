#!/usr/bin/env python3
"""Exhaustive drift-fix simulation V2 -- KCC on shared-bottleneck wired path.
Models G2_queue_cap, adaptive BW, realistic queue, 12 candidate fixes, 4 scenarios.
"""

import random

# Config
T_PROP_US = 1400
LINK_CAP = 1000000000  # 1 Gbps
MSS = 12000  # bits (1500 bytes)
PKTS_SEC = LINK_CAP // MSS
BUF_PKTS = 200
NUM_FLOWS = 11
SIM_SEC = 5
SEED = 42
SCALE = 1024
BASE_R = 400
J50 = 200
Q_BASE = 100
R_MAX = BASE_R * 256
PEST_INIT = 1000
PEST_MAX = 100000000
PEST_FLOOR = 10
CWND_GAIN = 2
DRIFT_THR = 14
T2_MULT = 4
T2_SHIFT = 3
SAT_THR = 55
SAT_HOLD = 30
QBOOST_THR = 16 * 1000 * SCALE  # 16ms scaled
JITTER_BASE = 50
JITTER_BURST = 300
JITTER_BURST_P = 0.02
random.seed(SEED)

# ---------- Fix functions ----------
FIXES = []


def reg(name, fn, **kw):
    FIXES.append((name, fn, kw))


def fix_none(f, qd, xe, jit, ecn, t2c, rtt):
    return True


reg("0_BASELINE", fix_none)


def fix_qdelay50(f, qd, xe, jit, ecn, t2c, rtt):
    return qd < xe // 2


reg("1_qdelay<50%", fix_qdelay50)


def fix_jitter12(f, qd, xe, jit, ecn, t2c, rtt):
    return jit < xe // 8


reg("2_jitter<12.5%", fix_jitter12)


def fix_both(f, qd, xe, jit, ecn, t2c, rtt):
    return qd < xe // 2 and jit < xe // 8


reg("3_qdelay<50%+jitter<12.5%", fix_both)


def fix_no_t2(f, qd, xe, jit, ecn, t2c, rtt):
    return False


reg("4_Remove-G3_slow", fix_no_t2)


def fix_qdelay_or_jit(f, qd, xe, jit, ecn, t2c, rtt):
    return qd < xe // 2 or jit < xe // 8


reg("5_qdelay<50%_OR_jitter<12.5%", fix_qdelay_or_jit)


def fix_qdelay25(f, qd, xe, jit, ecn, t2c, rtt):
    return qd < xe // 4


reg("6_qdelay<25%", fix_qdelay25)


def fix_qdelay_ecn(f, qd, xe, jit, ecn, t2c, rtt):
    return qd < xe // 2 and not ecn


reg("7_qdelay<50%+ECN", fix_qdelay_ecn)


def fix_qdelay50_t1style(f, qd, xe, jit, ecn, t2c, rtt):
    # Tier-2 fires with qdelay gate; Tier-1 uses jitter gate (already in sim)
    # This just adds qdelay to T2 like the other qdelay gates
    return qd < xe // 2


reg("8_qdelayT2_jitterT1", fix_qdelay50_t1style)


def fix_t2_rate_limit(f, qd, xe, jit, ecn, t2c, rtt):
    return True  # handled via caller time-check


reg("9_rate-limit_200ms", fix_t2_rate_limit)


def fix_ecn(f, qd, xe, jit, ecn, t2c, rtt):
    return not ecn


reg("10_ECN-gate", fix_ecn)


def fix_hybrid(f, qd, xe, jit, ecn, t2c, rtt):
    # Allow T2 if qdelay<25% (strict) OR if jitter>12.5% AND qdelay<50% (noisy+moderate)
    if qd < xe // 4:
        return True
    return bool(jit > xe // 8 and qd < xe // 2)


reg("11_hybrid_strict25_or_noisy50", fix_hybrid)


def fix_qdelay25_loose(f, qd, xe, jit, ecn, t2c, rtt):
    # Progressive: qdelay < 25% always; 25-50% only if pos_skip >> 56
    if qd < xe // 4:
        return True
    return bool(qd < xe // 2 and t2c > 3)


reg("12_progressive_qdelay", fix_qdelay25_loose)


# ---------- Simulation ----------
class Flow:
    def __init__(self):
        self.xe = 0
        self.pe = PEST_INIT
        self.psc = 0
        self.nsc = 0
        self.drift_sum = 0
        self.sat_hold = 0
        self.min_rtt_us = 10000000
        self.bw_est = LINK_CAP  # initial estimate
        self.retrans = 0
        self.pkts = 0
        self.last_t2_ms = 0.0
        self.qdelay_ewma = 0.0


def run_sim(name, n_flow, t_prop, shift_at=None, sim_s=60):
    results = {}
    for fix_name, fix_fn, fix_opts in FIXES:
        flows = [Flow() for _ in range(n_flow)]
        rate_ms = fix_opts.get("rate_limit_ms", 0)
        q_pkts = 0
        q_ewma = 0.0
        rtt_base = t_prop / 1e6
        rds_per_s = int(1.0 / rtt_base)
        total_rds = rds_per_s * sim_s
        for ri in range(total_rds):
            st_ms = ri * rtt_base * 1000.0
            st_s = st_ms / 1000.0
            # Baseline shift
            tp = t_prop
            if shift_at is not None and st_s >= shift_at:
                tp = 50000
            # Each flow sends
            total_inflight = 0
            total_send = 0
            for fl in flows:
                if fl.xe == 0:
                    fl.xe = tp * SCALE
                    fl.min_rtt_us = tp
                # Adaptive BW estimate (simple EWMA of delivery rate)
                xe_us = fl.xe // SCALE
                bdp = int(xe_us / 1e6 * fl.bw_est / MSS)
                bdp = max(bdp, 2)
                cwnd = bdp * CWND_GAIN
                cwnd = min(cwnd, 2000)  # prevent blowup
                total_inflight += cwnd
                total_send += cwnd
            # Queue: drain capacity_per_rtt, add inflight
            cap_per_rd = rtt_base * PKTS_SEC
            drained = min(q_pkts + total_send, cap_per_rd)
            q_pkts = max(0, q_pkts + total_send - drained)
            qd_us = (q_pkts / PKTS_SEC) * 1e6
            # Loss
            drops = 0
            if q_pkts > BUF_PKTS:
                drops = int(q_pkts - BUF_PKTS)
                q_pkts = BUF_PKTS
            if drops > 0:
                for fl in flows:
                    fl_drop = drops // n_flow
                    fl.retrans += fl_drop
                    total_send += fl_drop
            # EWMA qdelay
            alpha = 0.125
            q_ewma = q_ewma * (1 - alpha) + qd_us * alpha
            # Per-flow Kalman
            for _fi, fl in enumerate(flows):
                fl.min_rtt_us = min(fl.min_rtt_us, tp + int(qd_us))
                jit = JITTER_BASE
                if random.random() < JITTER_BURST_P:
                    jit += JITTER_BURST
                jit += int(random.gauss(0, jit * 0.3))
                jit = max(0, min(jit, 10000))
                rtt = tp + int(qd_us) + jit
                z = rtt * SCALE
                nu = z - fl.xe
                # Adaptive R
                je = max(0, jit - jit // 4)
                if je > 0:
                    ratio = je / J50
                    r = max(BASE_R, min(int(BASE_R * ratio**1.5), R_MAX))
                else:
                    r = BASE_R
                p_pred = min(fl.pe + Q_BASE, PEST_MAX)
                K = p_pred / (p_pred + r)
                if nu <= 0:
                    fl.xe = min(z, 0xFFFFFFFF)
                    fl.pe = max(r, PEST_FLOOR)
                    fl.psc = 0
                    fl.nsc += 1
                    fl.drift_sum = 0
                else:
                    fl.psc += 1
                    fl.nsc = 0
                    fl.pe = p_pred
                    # G2_queue_cap
                    if abs(nu) > QBOOST_THR:
                        fl.pe = PEST_INIT
                        corr = int(K * nu)
                        nu_x = int(fl.xe + corr)
                        fl.xe = min(nu_x, 0xFFFFFFFF)
                        fl.psc = 0
                        fl.drift_sum = 0
                        continue
                    # Saturation
                    if fl.pe >= PEST_MAX and fl.psc >= SAT_THR:
                        mrs = fl.min_rtt_us * SCALE
                        if fl.xe > mrs:
                            fl.xe = mrs
                            fl.psc = 0
                            fl.drift_sum = 0
                            fl.sat_hold = SAT_HOLD
                    # Tier-2 (with gates)
                    eff_min = DRIFT_THR * T2_MULT
                    if fl.sat_hold > 0:
                        fl.sat_hold -= 1
                        if fl.xe >= fl.min_rtt_us * SCALE:
                            continue
                    if fl.psc >= eff_min:
                        t2c = fl.psc // eff_min
                        xe_us = fl.xe // SCALE
                        ecn_flag = q_pkts > BUF_PKTS * 0.4
                        # Rate limit
                        if rate_ms > 0 and st_ms - fl.last_t2_ms < rate_ms:
                            continue
                        # Gate check
                        if not fix_fn(fl, int(q_ewma), xe_us, jit, ecn_flag, t2c, rtt):
                            continue
                        corr = K * nu
                        dc = int(corr / 8)
                        dc = max(dc, 1)
                        fl.xe = min(fl.xe + dc, 0xFFFFFFFF)
                        if rate_ms > 0:
                            fl.last_t2_ms = st_ms
                fl.pkts += 1
                # Update BW estimate (EWMA delivery rate)
                delivered = min(cap_per_rd / n_flow, cwnd)
                fl.bw_est = fl.bw_est * 0.9 + (delivered * MSS / rtt_base) * 0.1
        total_rtr = sum(f.retrans for f in flows)
        total_pkt = sum(f.pkts for f in flows) + total_rtr
        loss = total_rtr / max(total_pkt, 1)
        avg_xe = sum(f.xe // SCALE for f in flows) / n_flow
        results[fix_name] = {
            "loss": loss,
            "retrans": total_rtr,
            "avg_xe_us": avg_xe,
            "final_xes": [f.xe // SCALE for f in flows],
        }
    return results


def fmt_table(res, sc):
    print(f"\n{'=' * 90}")
    print(f"  {sc}")
    print(f"{'=' * 90}")
    print(f"{'Fix':<38} {'Loss%':>8} {'Retrans':>10} {'AvgXest':>10}")
    print("-" * 72)
    for name, r in sorted(res.items(), key=lambda x: x[1]["loss"]):
        print(
            f"{name:<38} {r['loss'] * 100:>7.2f}% {r['retrans']:>10} {r['avg_xe_us']:>9.0f}us",
        )
    return res


# ---- Run ----
print("Scenario A: 11 flows, 1.4ms baseline, persistent queue (matches iperf3)")
ra = run_sim("A", 11, T_PROP_US, shift_at=None)
fmt_table(ra, "A: 11 flows, 1.4ms wired")

print("\nScenario B: 1 flow, clean path")
rb = run_sim("B", 1, T_PROP_US, shift_at=None)
fmt_table(rb, "B: 1 flow, clean 1.4ms")

print("\nScenario C: 1 flow, baseline shift 1.4ms->50ms at t=10s")
rc = run_sim("C", 1, T_PROP_US, shift_at=10)
fmt_table(rc, "C: Baseline shift 1.4ms->50ms (BGP reroute)")

print("\nScenario D: 3 flows, shift at 30s")
rd = run_sim("D", 3, T_PROP_US, shift_at=30)
fmt_table(rd, "D: 3 flows baseline shift 1.4ms->50ms at 30s")

print("\n\n======= RANKING (avg loss across all 4 scenarios) =======")
names = sorted(ra.keys())
for i, n in enumerate(
    sorted(
        names,
        key=lambda n: (
            (ra[n]["loss"] + rb[n]["loss"] + rc[n]["loss"] + rd[n]["loss"]) / 4
        ),
    ),
):
    avg = (ra[n]["loss"] + rb[n]["loss"] + rc[n]["loss"] + rd[n]["loss"]) / 4
    print(f"  {i + 1:2d}. {n:<38} avg_loss={avg * 100:.2f}%")
