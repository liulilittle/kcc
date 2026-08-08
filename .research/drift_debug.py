#!/usr/bin/env python3
"""Debug script: trace x_est drift on 1.4ms path with jitter=5000us, qdelay=5000us."""

import random

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


for shift in [0, 1]:
    fl = Flow()
    fl.xe = 1400 * SCALE
    fl.min_rtt = 1400
    tp = 1400
    qd = 5000
    jit_hi = 5000
    trace = []
    for rd in range(500):
        jt = jit_hi + int(random.gauss(0, jit_hi * 0.5))
        jt = max(0, jt)
        rtt = tp + qd + jt
        z = rtt * SCALE
        nu = z - fl.xe
        je = max(0, jt - jt // 4)
        ratio = je / J50 if je > 0 else 0
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
        else:
            fl.psc += 1
            fl.pe = ppred
            if abs(nu) > QBOOST_THR_US * SCALE:
                fl.pe = P_INIT
                fl.xe = min(fl.xe + corr_abs, 0xFFFFFFFF)
                fl.psc = 0
                trace.append(
                    f"rd={rd} QBOOST: nu={nu // SCALE}us -> xe={fl.xe // SCALE}us",
                )
            elif fl.pe >= P_MAX and fl.psc >= SAT_THR:
                mrs = fl.min_rtt * SCALE
                if fl.xe > mrs:
                    fl.xe = mrs
                    fl.psc = 0
                    fl.sat_hold = SAT_HLD
                    trace.append(f"rd={rd} SAT: xe capped to {fl.xe // SCALE}us")
            elif fl.sat_hold > 0:
                fl.sat_hold -= 1
            elif fl.psc >= DRIFT and jt < fl.xe // SCALE // 8:
                fl.xe = min(fl.xe + corr_abs // 4, 0xFFFFFFFF)
                trace.append(
                    f"rd={rd} T1: corr={corr_abs // 4 // SCALE}us -> xe={fl.xe // SCALE}us",
                )
            elif fl.psc >= DRIFT * T2_MULT:
                thresh = max(fl.xe >> (10 + shift), 1)
                if qd < thresh:
                    fl.xe = min(fl.xe + corr_abs // 8, 0xFFFFFFFF)
                    trace.append(
                        f"rd={rd} T2(shift={shift}): corr={corr_abs // 8 // SCALE}us -> xe={fl.xe // SCALE}us",
                    )
            if fl.psc > 250:
                trace.append(
                    f"rd={rd} pos_skip saturated at 254, xe={fl.xe // SCALE}us, pe={fl.pe}",
                )
                break  # Nothing more interesting will happen
    print(f"shift={shift} final xe={fl.xe // SCALE}us")
    for t in trace[:20]:
        print(f"  {t}")
    print()
