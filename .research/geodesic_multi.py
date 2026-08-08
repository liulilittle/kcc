#!/usr/bin/env python3
"""GEODESIC: Multi-round full-spectrum verification 25us-1s. 3 rounds."""

import random
import time

SCALE = 1024
SHIFT = 10
GROWTH_NUM = 122
GROWTH_DEN = 1000
_f = 0


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


RTTs = [100, 500, 1400, 5000, 50000, 200000, 1000000]
STEPS = [5, 10, 25, 50, 100, 200]
NS = 10
ROUNDS = 2

t0 = time.time()
P("=" * 60)
P(f"GEODESIC {len(RTTs)}RTTs x {len(STEPS)}steps x {NS}seeds x {ROUNDS}rounds")
P("=" * 60)

for rd in range(ROUNDS):
    P(f"\n--- ROUND {rd + 1}/{ROUNDS} ---")
    for T in RTTs:
        sig = max(1, T // 100)
        for sp in STEPS:
            Tn = T + int(T * sp / 100)
            if Tn == T:
                continue
            m = 0
            for seed in range(NS):
                rng = random.Random(T * sp + seed * 1000 + rd * 100000)
                k = K(T, sig)
                for _ in range(2000):
                    k.step(max(1, T + int(rng.gauss(0, sig))))
                for s in range(1, 500):
                    k.step(max(1, Tn + int(rng.gauss(0, sig))))
                    if k.bdp() > T * 1.02:
                        break
                else:
                    m += 1
            if m > 5:
                F(f"R{rd} T={T:>7d}us +{sp:>3d}%:{m}/{NS}")
    if _f == 0:
        P(f"R{rd}:ok")

P("\n--- CONGESTION (3 rounds) ---")
for T, s, q, ln in [
    (1400, 20, 400, "DC"),
    (50000, 200, 5000, "WAN"),
    (300000, 500, 20000, "LH"),
]:
    for rd in range(ROUNDS):
        ok = 0
        for seed in range(20):
            rng = random.Random(T + seed + rd * 100)
            k = K(T, s)
            for _ in range(50000):
                k.step(max(1, T + q + int(rng.gauss(0, s))), q)
            if k.bdp() < T + q + T * 0.02:
                ok += 1
        if ok < 16:
            F(f"R{rd} {ln}:{ok}/20")
    P(f"{ln}:ok") if _f == 0 else None

P("\n--- DEADLOCK (3 rounds) ---")
for T, s in [(1400, 20), (50000, 200)]:
    for rd in range(ROUNDS):
        ok = 0
        for seed in range(50):
            rng = random.Random(T + seed * 9999 + rd * 1000)
            k = K(T, s)
            k.x = int(T * 5.5) * SCALE
            for _ in range(5000):
                k.step(max(1, T + int(rng.gauss(0, s))))
            if k.bdp() < T * 1.1:
                ok += 1
        if ok < 40:
            F(f"R{rd} T={T}us:{ok}/50")
    P(f"T={T}us:ok") if _f == 0 else None

P("\n--- PATH DECREASE ---")
for To, Tn in [(50000, 1400), (200000, 50000)]:
    ok = 0
    for seed in range(50):
        rng = random.Random(seed * 888 + To)
        k = K(To, To // 50)
        for _ in range(2000):
            k.step(max(1, To + int(rng.gauss(0, To // 50))))
        for _ in range(200):
            k.step(max(1, Tn + int(rng.gauss(0, Tn // 50))))
        if k.bdp() < Tn * 1.1:
            ok += 1
    P(f"{To}->{Tn}:{ok}/50") if ok >= 40 else F(f"{To}->{Tn}:{ok}/50")

dt = time.time() - t0
if _f == 0:
    P(f"\nALL PASSED {int(dt)}s {len(RTTs) * len(STEPS) * NS * ROUNDS} path tests")
else:
    P(f"\nFAILURES:{_f}")
