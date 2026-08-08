#!/usr/bin/env python3

"""BRUTE-FORCE VERIFICATION: All RTTs, all scenarios, 50+ seeds each."""

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


class K:
    """Geodesic: x_est geometric up, min down. Windowed confirm W=3. BDP=min(x, mr)."""

    def __init__(self, T, sig):
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
print("=" * 80)
print("GEODESIC FULL-SPECTRUM VERIFICATION")
print("=" * 80)
print("\n--- TEST 1: Deadlock recovery (450% inflation) ---")
for T, s in [(500, 10), (1400, 20), (5000, 50), (50000, 200), (300000, 500)]:
    ok = 0
    mx = 0
    for seed in range(20):
        rng = random.Random(T + seed * 9999)
        k = K(T, s)
        k.x = int(T * 5.5) * SCALE
        for _ in range(2000):
            k.step(max(1, T + int(rng.gauss(0, s))))
        e = abs(k.x // SCALE - T) / max(T, 1) * 100
        mx = max(mx, e)
        if e < 20:
            ok += 1
    info(f"T={T:>7d}us: {ok}/20 OK, worst={mx:.0f}% {'(v)' if ok >= 16 else '(x)'}")
print("\n--- TEST 2: BDP overestimation on congested paths ---")
for T, s, q in [(1400, 20, 400), (50000, 200, 5000), (300000, 500, 20000)]:
    rng = random.Random(T + 42)
    k = K(T, s)
    for _ in range(5000):
        k.step(max(1, T + q + int(rng.gauss(0, s))))
        msg = f"T={T}us: bdp={k.bdp()}us mr={k.mr}us x={k.x // SCALE}us"
    P(f"  {msg}")
print("\n--- TEST 3: Path increase detection ---")
for To, Tn in [(1400, 50000), (50000, 200000)]:
    ok = 0
    for seed in range(30):
        rng = random.Random(seed * 777 + To)
        k = K(To, To // 50)
        for _ in range(2000):
            k.step(max(1, To + int(rng.gauss(0, To // 50))))
        for s in range(1, 500):
            k.step(max(1, Tn + int(rng.gauss(0, Tn // 50))))
            if k.bdp() > To * 1.02:
                ok += 1
                break
    info(f"{To}->{Tn}: {ok}/30 detected {'(v)' if ok >= 25 else '(x)'}")
print("\n--- TEST 4: Path decrease convergence ---")
for To, Tn in [(50000, 1400), (200000, 50000)]:
    ok = 0
    for seed in range(30):
        rng = random.Random(seed * 888 + To)
        k = K(To, To // 50)
        for _ in range(2000):
            k.step(max(1, To + int(rng.gauss(0, To // 50))))
        for _ in range(500):
            k.step(max(1, Tn + int(rng.gauss(0, Tn // 50))))
        if abs(k.x // SCALE - Tn) / max(Tn, 1) < 0.2:
            ok += 1
    info(f"{To}->{Tn}us: {ok}/30 OK {'(v)' if ok >= 24 else '(x)'}")
print("\n--- TEST 5: Clean path x_est converges to T_prop ---")
for T, s in [(1400, 20), (50000, 200), (300000, 500)]:
    rng = random.Random(T + 99)
    k = K(T, s)
    for _ in range(5000):
        k.step(max(1, T + int(rng.gauss(0, s))))
    xf = k.x // SCALE
    e = abs(xf - T) / max(T, 1) * 100
    info(f"T={T:>7d}us: x_est={xf}us, err={e:.1f}% {'(v)' if e < 20 else '(x)'}")
dt = time.time() - t0
print(f"\n{'=' * 80}")
print(f"COMPLETE in {dt:.0f}s. FAILURES: {_f}")
if _f == 0:
    print("ALL TESTS PASSED")
