#!/usr/bin/env python3
"""Focused deadlock analysis: qdelay gate on Tier 2.
Tests: baseline shift during persistent queue (the deadlock concern).
"""

import random

SCALE = 1024
BASE_R = 400
J50 = 200
Q_BASE = 100
R_MAX = 102400
PEST_INIT = 1000
PEST_MAX = 100000000
DRIFT_THR = 14
T2_MULT = 4
SAT_THR = 55
SAT_HOLD = 30
QBOOST_THR_SCALED = 16 * 1000 * SCALE
T1_SHIFT = 2  # corr/4
T2_SHIFT = 3  # corr/8
T2_EARLY = 3  # amplitude


class Flow:
    def __init__(self):
        self.xe = 0
        self.pe = PEST_INIT
        self.psc = 0
        self.nsc = 0
        self.min_rtt = 10000000
        self.sat_hold = 0


def compute_r(jitter_us):
    je = max(0, jitter_us - jitter_us // 4)
    if je > 0:
        ratio = je / J50
        r = max(BASE_R, min(int(BASE_R * ratio**1.5), R_MAX))
    else:
        r = BASE_R
    return r


def sim_deadlock(
    t_prop_before,
    t_prop_after,
    qdelay_us,
    jitter_base_us,
    t2_gate_fn,
    n_rounds=5000,
    verbose=False,
):
    """
    Simulate a single flow experiencing a baseline shift while queue is present.
    Returns: x_est_us over time, convergence time to within 10% of target.
    """
    fl = Flow()
    fl.xe = t_prop_before * SCALE
    fl.min_rtt = t_prop_before
    history = []
    for rd in range(n_rounds):
        tp = t_prop_after if rd >= 1000 else t_prop_before
        jit = jitter_base_us + int(random.gauss(0, jitter_base_us * 0.2))
        jit = max(0, jit)
        rtt_us = tp + qdelay_us + jit
        z = rtt_us * SCALE
        nu = z - fl.xe
        jitter_ewma = jit
        r = compute_r(jitter_ewma)
        p_pred = min(fl.pe + Q_BASE, PEST_MAX)
        K = p_pred / (p_pred + r)
        if nu <= 0:
            fl.xe = min(z, 0xFFFFFFFF)
            fl.pe = max(r, 10)
            fl.psc = 0
            fl.nsc += 1
        else:
            fl.psc += 1
            fl.nsc = 0
            fl.pe = p_pred
            if abs(nu) > QBOOST_THR_SCALED:
                fl.pe = PEST_INIT
                fl.xe = min(fl.xe + int(K * nu), 0xFFFFFFFF)
                fl.psc = 0
                if verbose and rd >= 990 and rd <= 1010:
                    print(
                        f"  rd={rd} QBOOST! nu={nu // SCALE}us x_est->{fl.xe // SCALE}us",
                    )
                continue
            if fl.pe >= PEST_MAX and fl.psc >= SAT_THR:
                mrs = fl.min_rtt * SCALE
                if fl.xe > mrs:
                    fl.xe = mrs
                    fl.psc = 0
                    fl.sat_hold = SAT_HOLD
            if fl.sat_hold > 0:
                fl.sat_hold -= 1
                if fl.xe >= fl.min_rtt * SCALE:
                    continue
            if fl.psc >= DRIFT_THR and jit < fl.xe // SCALE // 8:
                corr = K * nu
                fl.xe = min(fl.xe + int(corr / 4), 0xFFFFFFFF)
            if fl.psc >= DRIFT_THR * T2_MULT:
                xe_us = fl.xe // SCALE
                if t2_gate_fn(qdelay_us, xe_us, jit):
                    corr = K * nu
                    fl.xe = min(fl.xe + int(corr / 8), 0xFFFFFFFF)
        history.append(fl.xe // SCALE)
    target = t_prop_after
    conv_round = None
    for i in range(1000, len(history)):
        if abs(history[i] - target) < target * 0.10:
            conv_round = i - 1000
            break
    post_shift = history[1000:]
    mean_post = sum(post_shift) / len(post_shift)
    return {
        "conv_round": conv_round,
        "mean_post_us": mean_post,
        "final_xe": history[-1],
        "pre_shift_mean": sum(history[:1000]) / 1000,
    }


def no_gate(qd, xe, jit):
    return True


def qdelay_gate(qd, xe, jit):
    return qd < xe // 2


def jitter_gate(qd, xe, jit):
    return jit < xe // 8


def both_gate(qd, xe, jit):
    return qd < xe // 2 and jit < xe // 8


print("=" * 85)
print("DEADLOCK ANALYSIS: qdelay gate on Tier 2")
print("=" * 85)

# Test 1: small baseline shift during persistent queue
print("\n--- Case 1: Small baseline shift (1.4->5ms) with qdelay=3ms ---")
print(f"{'Gate':<25} {'Converge@':>10} {'MeanPost':>10} {'FinalXe':>10}")
for name, fn in [
    ("BASELINE (no gate)", no_gate),
    ("qdelay<50%", qdelay_gate),
    ("jitter<12.5%", jitter_gate),
    ("qdelay+jitter", both_gate),
]:
    r = sim_deadlock(1400, 5000, 3000, 50, fn)
    conv = f"{r['conv_round']}rds" if r["conv_round"] else "NEVER"
    print(f"{name:<25} {conv:>10} {r['mean_post_us']:>9.0f}us {r['final_xe']:>9.0f}us")

# Test 2: large baseline shift with light queue
print("\n--- Case 2: Large baseline shift (1.4->50ms) with qdelay=1ms ---")
for name, fn in [
    ("BASELINE (no gate)", no_gate),
    ("qdelay<50%", qdelay_gate),
    ("jitter<12.5%", jitter_gate),
    ("qdelay+jitter", both_gate),
]:
    r = sim_deadlock(1400, 50000, 1000, 50, fn)
    conv = f"{r['conv_round']}rds" if r["conv_round"] else "NEVER"
    print(f"{name:<25} {conv:>10} {r['mean_post_us']:>9.0f}us {r['final_xe']:>9.0f}us")

# Test 3: baseline shift with zero queue
print("\n--- Case 3: Baseline shift (1.4->50ms) with qdelay=0 (clean) ---")
for name, fn in [
    ("BASELINE (no gate)", no_gate),
    ("qdelay<50%", qdelay_gate),
    ("jitter<12.5%", jitter_gate),
    ("qdelay+jitter", both_gate),
]:
    r = sim_deadlock(1400, 50000, 0, 50, fn)
    conv = f"{r['conv_round']}rds" if r["conv_round"] else "NEVER"
    print(f"{name:<25} {conv:>10} {r['mean_post_us']:>9.0f}us {r['final_xe']:>9.0f}us")

# Test 4: qdelay varying
print("\n--- Case 4: Convergence time vs qdelay (1.4->5ms shift, qdelay gate) ---")
print(f"{'qdelay(us)':<12} {'Converge@':<12} {'FinalXe':<12}")
for qd in [0, 500, 1000, 2000, 3000, 4000, 5000, 8000]:
    r = sim_deadlock(1400, 5000, qd, 50, qdelay_gate)
    conv = f"{r['conv_round']}rds" if r["conv_round"] else "NEVER"
    print(f"{qd:<12} {conv:<12} {r['final_xe']:<12.0f}us")
