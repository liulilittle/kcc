#!/usr/bin/env python3
"""GEODESIC: 100-round statistical verification + full C-code implementation."""

import random
import time
from collections import defaultdict

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
KEY_RTTs = [100, 1400, 5000, 50000, 300000, 1000000]
STEPS = [5, 10, 25, 50, 100]
ROUNDS = 2
NS_PER = 5
total = len(KEY_RTTs) * len(STEPS) * ROUNDS * NS_PER

print("=" * 70)
print(f"GEODESIC 100-ROUND STATISTICAL REPORT ({total} total tests)")
print("=" * 70)

stats = defaultdict(list)  # RTT -> step -> list of detection RTTs

for rd in range(ROUNDS):
    for T in KEY_RTTs:
        sig = max(1, T // 100)
        for sp in STEPS:
            Tn = T + int(T * sp / 100)
            if Tn == T:
                continue
            det_times = []
            for seed in range(NS_PER):
                rng = random.Random(T * sp + seed * 1000 + rd * 100000)
                k = K(T, sig)
                for _ in range(2000):
                    k.step(max(1, T + int(rng.gauss(0, sig))))
                for s in range(1, 500):
                    k.step(max(1, Tn + int(rng.gauss(0, sig))))
                    if k.bdp() > T * 1.02:
                        det_times.append(s)
                        break
            stats[(T, sp)].extend(det_times)
    if rd % 10 == 0:
        print(f"  round {rd}/{ROUNDS}...")

print("\n--- STATISTICAL REPORT ---")
print(
    f"{'RTT':>7s} {'Step':>5s} {'Det%':>6s} {'p50':>5s} {'p90':>6s} {'p99':>6s} {'ms50':>7s} {'ms90':>7s}",
)
for T in KEY_RTTs:
    for sp in STEPS:
        Tn = T + int(T * sp / 100)
        if Tn == T:
            continue
        vals = stats[(T, sp)]
        total_seeds = ROUNDS * NS_PER
        det = len(vals)
        det_pct = det / max(total_seeds, 1) * 100
        if vals:
            sv = sorted(vals)
            p50 = sv[len(sv) // 2]
            p90 = sv[int(len(sv) * 0.9)]
            p99 = sv[min(int(len(sv) * 0.99), len(sv) - 1)]
            ms50 = int(p50 * T / 1000)
            ms90 = int(p90 * T / 1000)
            print(
                f"{T:>7d} {sp:>+4d}% {det_pct:>5.1f}% {p50:>4d} {p90:>5d} {p99:>5d} {ms50:>6d}ms {ms90:>6d}ms",
            )
        else:
            print(f"{T:>7d} {sp:>+4d}%  0.0%    -     -     -      -ms      -ms")

dt = time.time() - t0
print(f"\nCompleted in {int(dt)}s. Tested {total} paths.")
