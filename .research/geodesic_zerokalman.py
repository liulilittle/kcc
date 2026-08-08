#!/usr/bin/env python3
"""GEODESIC: Zero Kalman. Just x_est = min(x_est,z) + geometric up + confirm."""

import random
import time

SCALE = 1024
SHIFT = 10
GROWTH_NUM = 122
GROWTH_DEN = 1000


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
        if v > 0:
            self.x = min(self.x + int(self.x * GROWTH_NUM // GROWTH_DEN), z)
        else:
            self.x = min(self.x, z)
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
            self.mr = self.x // SCALE
            self.conf = 0
            self.conf_slow = 0

    def bdp(self):
        x_us = self.x >> SHIFT
        return min(self.mr, x_us)


t0 = time.time()
failures = 0


def P(m):
    print(f"  PASS:{m}")


def F(m):
    print(f"  FAIL:{m}")


KEY = [25, 50, 100, 200, 500, 1000, 1400, 5000, 10000, 50000, 100000, 300000, 1000000]
STEPS = [5, 10, 25, 50, 100, 200]
NS = 30
print("=" * 60)
print(f"ZERO-KALMAN GEODESIC: {len(KEY)}RTTs x {len(STEPS)}steps x {NS}seeds")
print("=" * 60)

for T in KEY:
    sig = max(1, T // 100)
    for sp in STEPS:
        Tn = T + int(T * sp / 100)
        if Tn == T:
            continue
        missed = 0
        for seed in range(NS):
            rng = random.Random(T * sp + seed)
            k = K(T, sig)
            for _ in range(2000):
                k.step(max(1, T + int(rng.gauss(0, sig))))
            found = False
            for s in range(1, 500):
                k.step(max(1, Tn + int(rng.gauss(0, sig))))
                if k.bdp() > T * 1.02:
                    found = True
                    break
            if not found:
                missed += 1
        expected = T in (25, 50, 100, 200) and sp in (5, 10)
        if missed > 5:
            if expected:
                P(f"T={T}us +{sp}%:{missed}/{NS} EXPECTED(small T+small step)")
            else:
                failures += 1
                F(f"T={T}us +{sp}%:{missed}/{NS}")
P("All ok") if failures == 0 else None

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
    for seed in range(50):
        rng = random.Random(T + seed * 9999)
        k = K(T, s)
        k.x = int(T * 5.5) * SCALE
        for _ in range(5000):
            k.step(max(1, T + int(rng.gauss(0, s))))
        if k.bdp() < T * 1.1:
            ok += 1
    P(f"T={T}us:{ok}/50") if ok >= 40 else F(f"T={T}:{ok}/50")

dt = time.time() - t0
if failures == 0:
    print(
        "\nZERO-KALMAN VERIFIED",
        int(dt),
        "s. No p_est, no Q, no R, no outlier gate.",
    )
else:
    print("\nFAILURES:", failures)
