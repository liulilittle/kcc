#!/usr/bin/env python3
"""FINAL: sticky mr (5% threshold) + G3 C1=qdelay C2=qdelay C3=2 + slow upward"""

import random
import time

SCALE = 1024
PINIT = 1000
PMAX = int(1e8)
SHIFT = 10
_f = 0


def F(m):
    global _f
    print(f"  FAIL: {m}")
    _f += 1


def P(m):
    print(f"  PASS: {m}")


class K:
    def __init__(self, T, sigma):
        self.Tp = T
        self.x = T * SCALE
        self.mr = T
        self.p = PINIT
        self.j = 0
        self.q = 0
        self.ps = 0
        self.cr = 0
        self.g3c = 0

    def step(self, rtt):
        # sticky mr update: only go down if >5% below (noise-immune)
        if rtt < int(self.mr * 0.95):
            self.mr = rtt
        z = rtt * SCALE
        v = z - self.x
        av = v if v >= 0 else -v
        pp = min(self.p + 100, PMAX)
        # G3: N=2 (optimal), C1=|nu|>2.5*qdelay, C2=qdelay<mr/2
        if (
            v > 0
            and av > (int(self.q * SCALE) * 5) // 2
            and self.q < self.mr >> 1
            and self.ps >= 2
        ):
            self.x = min(z, 0xFFFFFFFF)
            self.p = max(400, 10)
            self.ps = 0
            self.mr = max(self.mr, rtt)
            self.g3c += 1
            return
        if v <= 0:
            self.x = z
            self.p = max(400, 10)
            self.ps = 0
        else:
            dt = max(max(self.mr >> 2, 50) * SCALE, max(1, int(self.j * 2)) * SCALE)
            if self.p <= 33 and av > dt and self.cr < 20:
                self.cr += 1
                self.ps += 1
                self.p = pp
            else:
                self.cr = 0
                gd = pp + 400
                self.p = max(pp - (pp * pp) // gd if gd else pp, 10)
                self.ps += 1
        self.j = self.j * 0.875 + (av >> SHIFT) * 0.125
        self.q = self.q * 0.875 + max(0, rtt - self.mr) * 0.125


t0 = time.time()
print("=" * 70)
print("FINAL: sticky mr(5%) + G3 N=2")
print("=" * 70)

print("\n--- 1. NOISE ---")
for T, sig in [(1400, 14), (50000, 500), (300000, 3000), (1000000, 10000)]:
    g3_sum = 0
    total = 0
    for seed in range(10):
        rng = random.Random(T + seed)
        k = K(T, sig)
        for _ in range(2000):
            k.step(max(1, T + int(rng.gauss(0, sig))))
        for _ in range(50000):
            k.step(max(1, T + int(rng.gauss(0, sig))))
        g3_sum += k.g3c
        total += 50000
    P(f"T={T:>7d}us: G3 rate={g3_sum / max(total, 1) * 100:.4f}%")

print("\n--- 2. PATH INCREASE ---")
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
    750000,
    1000000,
]
for T in RTTs:
    sig = max(1, T // 100)
    for sp in [5, 10, 25, 50, 100, 200]:
        Tn = T + int(T * sp / 100)
        if Tn == T:
            continue
        d = []
        m = 0
        for seed in range(50):
            rng = random.Random(T * sp + seed)
            k = K(T, sig)
            for _ in range(2000):
                k.step(max(1, T + int(rng.gauss(0, sig))))
            for s in range(1, 500):
                k.step(max(1, Tn + int(rng.gauss(0, sig))))
                if k.mr > T * 1.02:
                    d.append(s)
                    break
            else:
                m += 1
        if m > 5:
            F(f"T={T}us +{sp}%: {m}/50 MISSED")
    if _f == 0:
        P(f"T={T:>7d}us: all ok")

print("\n--- 3. CONGESTED ---")
for T, s, q, ln in [
    (1400, 20, 400, "DC"),
    (50000, 200, 5000, "WAN"),
    (300000, 500, 20000, "LH"),
]:
    rng = random.Random(T + 42)
    k = K(T, s)
    for _ in range(50000):
        k.step(max(1, T + q + int(rng.gauss(0, s))))
    xf = k.x // SCALE
    o = (xf - k.mr) / k.mr * 100 if xf > k.mr else 0
    P(f"  {ln}: x_est={xf}us mr={k.mr}us over={o:.1f}%") if o < 10 else F(
        f"  {ln}: over={o:.1f}%",
    )

print("\n--- 4. DEADLOCK ---")
for T, s in [(1400, 20), (50000, 200)]:
    ok = 0
    for seed in range(50):
        rng = random.Random(T + seed * 9999)
        k = K(T, s)
        k.x = int(T * 5.5) * SCALE
        for _ in range(5000):
            k.step(max(1, T + int(rng.gauss(0, s))))
        if abs(k.x // SCALE - T) / T < 0.15:
            ok += 1
    P(f"  T={T}us: {ok}/50") if ok >= 40 else F(f"T={T}us: {ok}/50")

print("\n--- 5. PATH DECREASE ---")
for To, Tn in [(50000, 1400), (200000, 50000)]:
    ok = 0
    for seed in range(50):
        rng = random.Random(seed * 888 + To)
        k = K(To, To // 50)
        for _ in range(2000):
            k.step(max(1, To + int(rng.gauss(0, To // 50))))
        for _ in range(200):
            k.step(max(1, Tn + int(rng.gauss(0, Tn // 50))))
        if abs(k.x // SCALE - Tn) / Tn < 0.2:
            ok += 1
    P(f"  {To}->{Tn}: {ok}/50") if ok >= 40 else F(f"{To}->{Tn}: {ok}/50")

dt = time.time() - t0
print(f"\n{'=' * 70}")
if _f == 0:
    print(f"ALL PASSED in {dt:.0f}s")
else:
    print(f"FAILURES: {_f}")
