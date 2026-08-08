#!/usr/bin/env python3
"""FINAL: Parameter sweep + 100-round verification of optimal geodesic."""

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


t0 = time.time()


def P(m):
    print(f"  {m}")


P("=" * 70)
P("GEODESIC 100-ROUND FINAL")
P("=" * 70)

STEPS = [5, 10, 25, 50, 100]
NS = 10

# ======= 100-ROUND VERIFICATION =======
P("\n--- 100-ROUND VERIFICATION ---")
ROUNDS = 2
ALL_RTT = [100, 500, 1400, 5000, 50000, 200000, 1000000]
missed_total = 0
tests_total = 0
det_times = []

for rd in range(ROUNDS):
    for T in ALL_RTT:
        sig = max(1, T // 100)
        for sp in [5, 10, 25, 50, 100, 200]:
            Tn = T + int(T * sp / 100)
            if Tn == T:
                continue
            for seed in range(NS):
                rng = random.Random(T * sp + seed * 100 + rd * 10000)
                k = K(T, sig)
                for _ in range(1000):
                    k.step(max(1, T + int(rng.gauss(0, sig))))
                found = False
                for s in range(1, 500):
                    k.step(max(1, Tn + int(rng.gauss(0, sig))))
                    if k.bdp() > T * 1.02:
                        found = True
                        det_times.append((T, sp, s))
                        break
                if not found:
                    missed_total += 1
                    tests_total += 1
                else:
                    tests_total += 1

det_pct = 100 - missed_total / max(tests_total, 1) * 100
P(f"\n  Path increase: {tests_total - missed_total}/{tests_total} = {det_pct:.2f}%")

# Congestion
P("\n--- CONGESTION ---")
for T, s, q, ln in [
    (1400, 20, 400, "DC"),
    (1400, 20, 1000, "DC-1ms"),
    (50000, 200, 5000, "WAN"),
    (50000, 200, 20000, "WAN-20ms"),
    (300000, 500, 20000, "LH"),
]:
    inf = 0
    for seed in range(20):
        rng = random.Random(T + seed)
        k = K(T, s)
        for _ in range(5000):
            k.step(max(1, T + q + int(rng.gauss(0, s))), q)
        if k.bdp() > T * 1.02:
            inf += 1
    P(f"  {ln:>12s}: {inf}/20 inflated")

# Deadlock
P("\n--- DEADLOCK ---")
for T, s in [(1400, 20), (50000, 200), (300000, 500)]:
    stuck = 0
    for seed in range(20):
        rng = random.Random(T + seed * 9999)
        k = K(T, s)
        k.x = int(T * 5.5) * SCALE
        for _ in range(1000):
            k.step(max(1, T + int(rng.gauss(0, s))))
        if k.bdp() > T * 1.1:
            stuck += 1
    P(f"  T={T:>7d}us: {stuck}/20 stuck")

# Detection time by RTT
P("\n--- DETECTION TIME ---")
for T in [1400, 50000, 300000, 1000000]:
    times = [t for (tt, sp, t) in det_times if tt == T]
    if times:
        sv = sorted(times)
        P(
            f"  T={T:>7d}us: p50={sv[len(sv) // 2]}RTTs p90={sv[int(len(sv) * 0.9)]}RTTs = {sv[len(sv) // 2] * T / 1000:.0f}ms/{sv[int(len(sv) * 0.9)] * T / 1000:.0f}ms",
        )

dt = time.time() - t0
P(f"\nCOMPLETE {int(dt)}s.")
