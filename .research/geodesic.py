#!/usr/bin/env python3
"""GEODESIC: x_est geometric up, min down. Windowed confirm."""

import random
import sys
import time

sys.stdout.reconfigure(encoding="utf-8")
_f = 0
SCALE = 1024
SHIFT = 10
GROWTH_NUM = 122
GROWTH_DEN = 1000


def F(m):
    global _f
    print(m)
    _f += 1


def P(m):
    print(m)


class K:
    def __init__(self, T, sig):
        self.x = T * SCALE
        self.mr = T
        self.conf = 0
        self.conf_slow = 0
        self.pd_cnt = 0
        self.T = T
        self.sig = sig

    def step(self, rtt, ql=0):
        z = rtt * SCALE
        v = z - self.x
        if v <= 0:
            self.x = min(self.x, z)
        else:
            growth = self.x * GROWTH_NUM // GROWTH_DEN
            self.x = min(self.x + growth, z)
        if self.x >= self.mr * SCALE * 11 // 10:
            self.conf += 1
            self.conf_slow += 1
        elif self.x >= self.mr * SCALE * 21 // 20:
            self.conf = 0
            self.conf_slow += 1
        else:
            self.conf = 0
        if self.x <= self.mr * SCALE:
            self.conf = 0
            self.conf_slow = 0
        if self.conf >= 4 or self.conf_slow >= 5:
            self.mr = self.x >> SHIFT
            self.conf = 0
            self.conf_slow = 0
        if self.conf == 0 and self.conf_slow == 0:
            x_us = self.x >> SHIFT
            if x_us < self.mr:
                self.pd_cnt += 1
                if self.pd_cnt >= 3:
                    self.mr = x_us
                    self.pd_cnt = 0
            else:
                self.pd_cnt = 0

    def bdp(self):
        x_us = self.x >> SHIFT
        return min(self.mr, x_us)


t0 = time.time()
P("=" * 70)
P("GEODESIC: geom up + min down + confirm=3")
P("=" * 70)

P("\n--- PATH INCREASE ---")
for T, sig in [(1400, 14), (50000, 500), (300000, 3000), (1000000, 10000)]:
    for sp in [5, 10, 25, 50, 100, 200]:
        Tn = T + int(T * sp / 100)
        d = []
        m = 0
        if Tn == T:
            continue
        for seed in range(50):
            rng = random.Random(T * sp + seed)
            k = K(T, sig)
            for _ in range(2000):
                k.step(max(1, T + int(rng.gauss(0, sig))))
            for s in range(1, 500):
                k.step(max(1, Tn + int(rng.gauss(0, sig))))
                if k.bdp() > T * 1.02:
                    d.append(s)
                    break
            else:
                m += 1
        if m > 5:
            F(f"  FAIL T={T} +{sp}%: {m}/50")
    P(f"  T={T:>7d}us: ok") if _f == 0 else None

P("\n--- CONGESTED (BDP bounded by T+q — no divergence) ---")
for T, s, q, ln in [
    (1400, 20, 400, "DC"),
    (50000, 200, 5000, "WAN"),
    (300000, 500, 20000, "LH"),
]:
    ok = 0
    for seed in range(10):
        rng = random.Random(T + seed)
        k = K(T, s)
        for _ in range(50000):
            k.step(max(1, T + q + int(rng.gauss(0, s))), q)
        if k.bdp() < T + q:
            ok += 1
    P(f"  {ln}: {ok}/10 bounded") if ok >= 8 else F(f"  FAIL {ln}: {ok}/10")

P("\n--- DEADLOCK ---")
for T, s in [(1400, 20), (50000, 200)]:
    ok = 0
    for seed in range(50):
        rng = random.Random(T + seed * 9999)
        k = K(T, s)
        k.x = int(T * 5.5) * SCALE
        for _ in range(5000):
            k.step(max(1, T + int(rng.gauss(0, s))))
        if k.bdp() < T * 1.1:
            ok += 1
    P(f"  T={T}us: {ok}/50") if ok >= 40 else F(f"  FAIL T={T}: {ok}/50")

dt = time.time() - t0
if _f == 0:
    P(f"\nVERIFIED {int(dt)}s. ZERO FAILURES.")
else:
    P(f"\nFAILURES: {_f}")
