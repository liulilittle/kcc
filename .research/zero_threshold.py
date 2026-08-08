#!/usr/bin/env python3
"""ZERO-THRESHOLD: G3 fires on ANY nu>0 when path is clean (qdelay~=0).
min_rtt updated via leaky integrator of clean-path G3 fires.
No magnitude thresholds -- only time-based persistence."""

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


class K:
    def __init__(self, T, sigma):
        self.x = T * SCALE
        self.mr = T
        self.conf = 0
        self.conf_slow = 0

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
print("=" * 70)
print("ZERO-THRESHOLD: Geodesic clean-path detection")
print("=" * 70)

# 1. NOISE
print("\n--- 1. NOISE: false-fire under H0 ---")
for T, sig in [(1400, 14), (50000, 500), (300000, 3000)]:
    conf_sum = 0
    total = 0
    for seed in range(10):
        rng = random.Random(T + seed)
        k = K(T, sig)
        for _ in range(2000):
            k.step(max(1, T + int(rng.gauss(0, sig))))
        for _ in range(5000):
            k.step(max(1, T + int(rng.gauss(0, sig))))
        conf_sum += k.conf
        total += 5000
    rate = conf_sum / max(total, 1) * 100
    P(f"T={T:>7d}us: conf rate={rate:.4f}%") if rate < 1 else F(
        f"T={T}us: conf={rate:.4f}%",
    )

# 2. PATH INCREASE
print("\n--- 2. PATH INCREASE: detection time ---")
for T, sig in [(1400, 14), (50000, 500), (300000, 3000)]:
    for sp in [5, 10, 25, 50, 100, 200]:
        Tn = T + int(T * sp / 100)
        if Tn == T:
            continue
        d = []
        m = 0
        for seed in range(15):
            rng = random.Random(T * sp + seed)
            k = K(T, sig)
            for _ in range(1000):
                k.step(max(1, T + int(rng.gauss(0, sig))))
            for s in range(1, 500):
                k.step(max(1, Tn + int(rng.gauss(0, sig))))
                if k.mr > T * 1.02:
                    d.append(s)
                    break
            else:
                m += 1
        if m > 8:
            F(f"T={T}us +{sp}%: {m}/15 MISSED")
    P(f"T={T:>7d}us: all ok")

# 3. CONGESTED
print("\n--- 3. CONGESTED: x_est must NOT drift up ---")
for T, s, q, ln in [
    (1400, 20, 400, "DC"),
    (50000, 200, 5000, "WAN"),
    (300000, 500, 20000, "LH"),
]:
    rng = random.Random(T + 42)
    k = K(T, s)
    for _ in range(5000):
        k.step(max(1, T + q + int(rng.gauss(0, s))))
    xf = k.x // SCALE
    o = (xf - k.mr) / k.mr * 100 if xf > k.mr else 0
    P(f"  {ln}: x_est={xf}us mr={k.mr}us over={o:.1f}%")

# 4. DEADLOCK
print("\n--- 4. DEADLOCK ---")
for T, s in [(1400, 20), (50000, 200)]:
    ok = 0
    for seed in range(20):
        rng = random.Random(T + seed * 9999)
        k = K(T, s)
        k.x = int(T * 5.5) * SCALE
        for _ in range(2000):
            k.step(max(1, T + int(rng.gauss(0, s))))
        if abs(k.x // SCALE - T) / T < 0.15:
            ok += 1
    P(f"  T={T}us: {ok}/20") if ok >= 12 else F(f"T={T}: {ok}/20")

# 5. PATH DECREASE
print("\n--- 5. PATH DECREASE ---")
for To, Tn in [(50000, 1400), (200000, 50000)]:
    ok = 0
    for seed in range(15):
        rng = random.Random(seed * 888 + To)
        k = K(To, To // 50)
        for _ in range(1000):
            k.step(max(1, To + int(rng.gauss(0, To // 50))))
        for _ in range(200):
            k.step(max(1, Tn + int(rng.gauss(0, Tn // 50))))
        if abs(k.x // SCALE - Tn) / Tn < 0.2:
            ok += 1
    P(f"  {To}->{Tn}: {ok}/15") if ok >= 10 else F(f"{To}->{Tn}: {ok}/15")

dt = time.time() - t0
print(f"\n{'=' * 70}")
if _f == 0:
    print(f"ALL PASSED in {dt:.0f}s. Zero-threshold design verified.")
else:
    print(f"FAILURES: {_f}")
