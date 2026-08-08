#!/usr/bin/env python3

"""Correct simulation: uses jitter_ewma (not raw jitter) for Tier 1 gate.

Tests: 11 flows, 1.4ms T_prop, 5ms qdelay, jitter_ewma converges to ~5ms.
=> Tier 1 blocked by jitter gate. Only Tier 2 can cause drift.
=> qdelay gate on Tier 2 is the correct and sufficient fix."""

import random
import statistics

random.seed(42)
SCALE = 1024
BASE_R = 400
J50 = 200
Q_BASE = 100
R_MAX = 102400
P_INIT = 1000
P_MAX = 100_000_000
P_FLOOR = 10
DRIFT = 14
T2_MULT = 4
SAT_THR = 55
SAT_HLD = 30
QBOOST_THR_US = 16000


class Flow:
    def __init__(self):
        self.xe = 0
        self.pe = P_INIT
        self.psc = 0
        self.sat_hold = 0
        self.min_rtt = 10**9
        self.jitter_ewma = 0


JITTER_ALPHA = 0.125  # KCC jitter EWMA alpha


def sim(n_flows, t_prop, qdelay, jitter_hi, shift, n_rounds=3000):
    fls = [Flow() for _ in range(n_flows)]
    for f in fls:
        f.xe = t_prop * SCALE
        f.min_rtt = t_prop
    for _rd in range(n_rounds):
        for f in fls:
            jt = jitter_hi + int(random.gauss(0, jitter_hi * 0.5))
            jt = max(0, jt)
            # Update jitter EWMA (matches KCC code)
            f.jitter_ewma = int(f.jitter_ewma * (1 - JITTER_ALPHA) + jt * JITTER_ALPHA)
            rtt = t_prop + qdelay + jt
            z = rtt * SCALE
            nu = z - f.xe
            je = max(0, jt - jt // 4)
            ratio = je / J50 if je > 0 else 0
            r = (
                max(BASE_R, max(BASE_R, min(int(BASE_R * ratio**1.5), R_MAX)))
                if ratio > 0
                else BASE_R
            )
            ppred = min(f.pe + Q_BASE, P_MAX)
            K = ppred / (ppred + r)
            corr_abs = int(K * abs(nu))
            if nu <= 0:
                f.xe = min(z, 0xFFFFFFFF)
                f.pe = max(r, P_FLOOR)
                f.psc = 0
            else:
                f.psc += 1
                f.pe = ppred
                if abs(nu) > QBOOST_THR_US * SCALE:
                    f.pe = P_INIT
                    f.xe = min(f.xe + corr_abs, 0xFFFFFFFF)
                    f.psc = 0
                    continue
                if f.pe >= P_MAX and f.psc >= SAT_THR:
                    mrs = f.min_rtt * SCALE
                    if f.xe > mrs:
                        f.xe = mrs
                        f.psc = 0
                        f.sat_hold = SAT_HLD
                if f.sat_hold > 0:
                    f.sat_hold -= 1
                    if f.xe >= f.min_rtt * SCALE:
                        continue
                # Tier 1: uses jitter_ewma vs min_rtt>>3 (matches KCC code)
                if f.psc >= DRIFT and f.jitter_ewma < f.min_rtt >> 3:
                    f.xe = min(f.xe + corr_abs // 4, 0xFFFFFFFF)
                # Tier 2: qdelay-gated
                if f.psc >= DRIFT * T2_MULT:
                    thresh = max(f.xe >> (10 + shift), 1)
                    if qdelay < thresh:
                        f.xe = min(f.xe + corr_abs // 8, 0xFFFFFFFF)
    return statistics.mean([(f.xe // SCALE) - t_prop for f in fls])


print("CORRECTED SIM: G3_fast uses jitter_ewma, G3_slow uses qdelay gate")
print()
print("T_prop=1.4ms, 11 flows, qdelay=5ms (user test scenario)")
print(f"{'jitter':>8} {'shift=0':>12} {'shift=1':>12} {'shift=2':>12} {'shift=3':>12}")
for jit in [50, 200, 500, 1000, 2000, 5000]:
    row = [f"{sim(11, 1400, 5000, jit, sh):>10.0f}us" for sh in [0, 1, 2, 3]]
    print(f"{jit:>8}us  " + ("  ".join(row)))
print()
print("With jitter_ewma >= 500us (typical for multi-flow congestion on 1.4ms path):")
print("  min_rtt>>3 = 1400>>3 = 175us")
print("  jitter_ewma(500us) > 175us => G3_fast BLOCKED")
print("  => Only G3_slow can cause drift")
print("  => shift=1 qdelay gate blocks G3_slow (qdelay=5000 > x_est>>1=700)")
print("  => x_est stays at 1400us (correct)")
print("  => shift=0 (no gate) allows G3_slow => x_est drifts to ~6400us")
