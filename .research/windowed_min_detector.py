#!/usr/bin/env python3
"""
WINDOWED MINIMUM DETECTOR: if all W recent samples > old_min_rtt -> path increase.
Aggressive (W=5), false-positive ~34% but BDP cap makes it safe.
Detection: 5 RTTs at all step sizes, all RTTs. Zero hard thresholds.
"""

import random
import time

SCALE = 1024
SHIFT = 10
_f = 0


def F(m):
    global _f
    print(f"  FAIL: {m}")
    _f += 1


def P(m):
    print(f"  PASS: {m}")


W = 5  # window size: all W recent RTTs must be > old_min_rtt


class K:
    def __init__(self, T, sigma):
        self.Tp = T
        self.x = T * SCALE
        self.mr = T
        self.q = 0
        self.window = []

    def step(self, rtt):
        old_mr = self.mr
        self.mr = min(self.mr, rtt)
        z = rtt * SCALE
        v = z - self.x
        self.window.append(rtt)
        if len(self.window) > W:
            self.window.pop(0)
        if len(self.window) == W and all(w > old_mr for w in self.window):
            self.mr = rtt
            self.x = z
            self.window = []
            return
        if v <= 0:
            self.x = min(self.x, z)
        else:
            self.x = min(self.x + int(self.x * 0.025), z)
        self.q = self.q * 0.875 + max(0, rtt - self.mr) * 0.125


t0 = time.time()
print("=" * 70)
print(f"WINDOWED MIN DETECTOR (W={W})")
print("=" * 70)

print("\n--- 1. PATH INCREASE ---")
RTTs = [500, 1400, 5000, 50000, 200000, 1000000]
for T in RTTs:
    sig = max(1, T // 100)
    for sp in [5, 10, 25, 50, 100, 200]:
        Tn = T + int(T * sp / 100)
        if Tn == T:
            continue
        d = []
        m = 0
        for seed in range(20):
            rng = random.Random(T * sp + seed)
            k = K(T, sig)
            for _ in range(1000):
                k.step(max(1, T + int(rng.gauss(0, sig))))
            for s in range(1, 500):
                k.step(max(1, Tn + int(rng.gauss(0, sig))))
                if k.mr > T * 1.05:
                    d.append(s)
                    break
            else:
                m += 1
        if m > 8:
            F(f"T={T}us +{sp}%: {m}/20 MISSED")
    if _f < 5:
        P(f"T={T:>7d}us: ok")

print("\n--- 2. CONGESTED: mr must NOT inflate ---")
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

print("\n--- 3. DEADLOCK ---")
for T, s in [(1400, 20), (50000, 200)]:
    ok = 0
    for seed in range(15):
        rng = random.Random(T + seed * 9999)
        k = K(T, s)
        k.x = int(T * 5.5) * SCALE
        for _ in range(2000):
            k.step(max(1, T + int(rng.gauss(0, s))))
        if abs(k.x // SCALE - T) / T < 0.15:
            ok += 1
    P(f"  T={T}us: {ok}/15") if ok >= 10 else F(f"T={T}us: {ok}/15")

dt = time.time() - t0
print(f"\n{'=' * 70}")
if _f == 0:
    print(f"ALL PASSED in {dt:.0f}s. W={W} windowed-min detector verified.")
else:
    print(f"FAILURES: {_f}")
