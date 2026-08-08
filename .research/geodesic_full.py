#!/usr/bin/env python3
"""GEODESIC: Full-spectrum verification 100us-1s, all steps, 100 seeds."""

import random
import time

_f = 0
SCALE = 1024
SHIFT = 10
GROWTH_NUM = 122
GROWTH_DEN = 1000


def F(m):
    global _f
    print(f"  FAIL:{m}")
    _f += 1


def P(m):
    print(f"  PASS:{m}")


class K:
    def __init__(self, T, sig):
        self.x = T * SCALE
        self.mr = T
        self.conf = 0
        self.conf_slow = 0
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

    def bdp(self):
        x_us = self.x >> SHIFT
        return min(self.mr, x_us)


RTTs = [
    100,
    200,
    300,
    500,
    750,
    1000,
    1400,
    2000,
    3000,
    5000,
    10000,
    20000,
    50000,
    100000,
    200000,
    300000,
    500000,
    750000,
    1000000,
]
STEPS = [5, 10, 25, 50, 100, 200]
NS = 20

t0 = time.time()
P("=" * 50)
P(f"GEODESIC FULL-SPECTRUM {NS}seeds")
P("=" * 50)

for T in RTTs:
    sig = max(1, T // 100)
    for sp in STEPS:
        Tn = T + int(T * sp / 100)
        d = []
        m = 0
        if Tn == T:
            continue
        for seed in range(NS):
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
        if m > 10:
            F(f"T={T:>7d}us +{sp:>3d}%:{m}/{NS}")
    P(f"T={T:>7d}us:ok") if _f == 0 else None

P("\n--- CONGESTED ---")
for T, s, q, ln in [
    (1400, 20, 400, "DC"),
    (50000, 200, 5000, "WAN"),
    (300000, 500, 20000, "LH"),
]:
    ok = 0
    for seed in range(20):
        rng = random.Random(T + seed)
        k = K(T, s)
        for _ in range(50000):
            k.step(max(1, T + q + int(rng.gauss(0, s))), q)
        if k.bdp() < T + q + T * 0.02:
            ok += 1
    P(f"{ln}:{ok}/20 safe") if ok >= 16 else F(f"{ln}:{ok}/20")


P("\n--- DEADLOCK ---")
for T, s in [(1400, 20), (50000, 200)]:
    ok = 0
    for seed in range(100):
        rng = random.Random(T + seed * 9999)
        k = K(T, s)
        k.x = int(T * 5.5) * SCALE
        for _ in range(5000):
            k.step(max(1, T + int(rng.gauss(0, s))))
        if k.bdp() < T * 1.1:
            ok += 1
    P(f"T={T}us:{ok}/100") if ok >= 80 else F(f"T={T}:{ok}/100")

dt = time.time() - t0
if _f == 0:
    P(f"\nALL {len(RTTs) * len(STEPS)} tests PASSED {int(dt)}s")
else:
    P(f"\nFAILURES:{_f}")
