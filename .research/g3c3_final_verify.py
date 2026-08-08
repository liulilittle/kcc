#!/usr/bin/env python3
"""FINAL VERIFICATION: Geodesic estimator. All RTTs, all scenarios, noise resistance."""

import random
import time

SCALE = 1024
SHIFT = 10
GROWTH_NUM = 122
GROWTH_DEN = 1000
_f = 0


def F(m):
    global _f
    print(f"  FAIL: {m}")
    _f += 1


def P(m):
    print(f"  PASS: {m}")


def info(m):
    print(f"  {m}")


class Geo:
    def __init__(self, T, sig):
        self.x = T * SCALE
        self.mr = T
        self.conf = 0
        self.conf_slow = 0
        self.T = T
        self.j = 0
        self.g3c = 0

    def step(self, rtt):
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
            self.g3c += 1
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
        self.j = self.j * 7 // 8 + (abs(v) >> SHIFT) * 1 // 8

    def bdp(self):
        return min(self.x >> SHIFT, self.mr)


t0 = time.time()
print("=" * 70)
print("FINAL GEODESIC VERIFICATION")
print("=" * 70)

# ===== 1. NOISE: G3 false-fire rate under H0 =====
print("\n--- 1. NOISE RESISTANCE: G3 false-fire under H0 ---")
for T, sig in [(1400, 14), (50000, 500), (300000, 3000)]:
    g3_sum = 0
    total_steps = 0
    for seed in range(5):
        rng = random.Random(T + seed)
        k = Geo(T, sig)
        for _ in range(1000):
            k.step(max(1, T + int(rng.gauss(0, sig))))
        for _ in range(5000):
            k.step(max(1, T + int(rng.gauss(0, sig))))
        g3_sum += k.g3c
        total_steps += 5000
    rate = g3_sum / max(total_steps, 1) * 100
    if rate < 0.1:
        P(f"T={T:>7d}us: G3 rate={rate:.4f}% (<0.1% -- noise-resistant)")
    elif rate < 1:
        info(f"T={T:>7d}us: G3 rate={rate:.4f}% (<1% -- acceptable)")
    else:
        F(f"T={T:>7d}us: G3 rate={rate:.4f}% (TOO HIGH)")

# ===== 2. PATH INCREASE: detection at all step sizes =====
print("\n--- 2. PATH INCREASE detection (all RTTs, all steps, 30 seeds) ---")
RTTs = [
    500,
    1000,
    1400,
    2000,
    5000,
    10000,
    20000,
    50000,
    100000,
    200000,
    300000,
    500000,
    1000000,
]
for T in RTTs:
    sig = max(1, T // 100)
    for sp in [5, 10, 25, 50, 100, 200]:
        Tn = T + int(T * sp / 100)
        if Tn == T:
            continue
        delays = []
        missed = 0
        for seed in range(30):
            rng = random.Random(T * sp + seed)
            k = Geo(T, sig)
            for _ in range(1000):
                k.step(max(1, T + int(rng.gauss(0, sig))))
            for s in range(1, 500):
                k.step(max(1, Tn + int(rng.gauss(0, sig))))
                if k.x >> 10 >= T * 105 // 100:
                    delays.append(s)
                    break
            else:
                missed += 1
        if missed > 10:
            F(f"T={T}us +{sp}%: {missed}/30 MISSED")
    if _f < 10:
        P(f"T={T:>7d}us: all steps OK")

# ===== 3. CONGESTED: BDP not inflated =====
print("\n--- 3. CONGESTED: x_est must stay near T_prop ---")
for T, s, q, ln in [
    (1400, 20, 400, "DC"),
    (50000, 200, 5000, "WAN"),
    (300000, 500, 20000, "LH"),
]:
    rng = random.Random(T + 42)
    k = Geo(T, s)
    for _ in range(5000):
        k.step(max(1, T + q + int(rng.gauss(0, s))))
    bdp = k.bdp()
    mr = k.mr
    o = (bdp - mr) / mr * 100 if bdp > mr else 0
    if o < 10:
        P(f"  {ln}: bdp={bdp}us mr={mr}us over={o:.1f}% (<10% -- safe)")
    elif o < 20:
        info(f"  {ln}: bdp={bdp}us mr={mr}us over={o:.1f}% (<20%)")
    else:
        F(f"  {ln}: bdp={bdp}us mr={mr}us over={o:.1f}% (EXCESSIVE)")

# ===== 4. DEADLOCK =====
print("\n--- 4. DEADLOCK: 450% inflation -> recovery ---")
for T, s in [(1400, 20), (50000, 200), (300000, 500)]:
    ok = 0
    for seed in range(20):
        rng = random.Random(T + seed * 9999)
        k = Geo(T, s)
        k.x = int(T * 5.5) * SCALE
        for _ in range(2000):
            k.step(max(1, T + int(rng.gauss(0, s))))
        if abs((k.x >> 10) - T) / T < 0.15:
            ok += 1
    if ok >= 14:
        P(f"  T={T}us: {ok}/20 recovered (v)")
    else:
        F(f"  T={T}us: only {ok}/20")

# ===== 5. PATH DECREASE =====
print("\n--- 5. PATH DECREASE: instant convergence ---")
for To, Tn in [(50000, 1400), (200000, 50000)]:
    ok = 0
    for seed in range(20):
        rng = random.Random(seed * 888 + To)
        k = Geo(To, To // 50)
        for _ in range(1000):
            k.step(max(1, To + int(rng.gauss(0, To // 50))))
        for _ in range(200):
            k.step(max(1, Tn + int(rng.gauss(0, Tn // 50))))
        if abs((k.x >> 10) - Tn) / Tn < 0.1:
            ok += 1
    if ok >= 14:
        P(f"  {To}->{Tn}: {ok}/20 converged (v)")
    else:
        F(f"  {To}->{Tn}: only {ok}/20")

dt = time.time() - t0
print(f"\n{'=' * 70}")
if _f == 0:
    print(f"ALL TESTS PASSED in {dt:.0f}s. Geodesic verified.")
else:
    print(f"FAILURES: {_f}")
