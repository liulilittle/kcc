#!/usr/bin/env python3
import random as rnd

SCALE = 1024
GROWTH_NUM = 122
GROWTH_DEN = 1000


class Est:
    # Simplified test of fast path (10%/4) only; actual code uses dual-threshold (10%/4 + 5%/5)
    def __init__(self, T):
        self.x = T * SCALE
        self.mr = T
        self.conf = 0

    def stepA(self, rtt):
        z = rtt * SCALE
        v = z - self.x
        if v <= 0:
            self.x = min(self.x, z)
        else:
            self.x = min(self.x + (self.x * GROWTH_NUM) // GROWTH_DEN, z)
        t = (self.mr * 11 * SCALE) // 10
        if self.x >= t:
            self.conf += 1
        elif self.x <= self.mr * SCALE:
            self.conf = 0
        if self.conf >= 4:
            self.mr = self.x // SCALE
            self.conf = 0
        return self.x // SCALE, min(self.x // SCALE, self.mr)

    def stepB(self, rtt):
        z = rtt * SCALE
        v = z - self.x
        if v <= 0:
            self.x = min(self.x, z)
        else:
            self.x = min(self.x + (self.x * GROWTH_NUM) // GROWTH_DEN, z)
        return self.x // SCALE


T = 100000
sigma = T // 100
seeds = 100
Tnew = T + T * 5 // 100
print(f"Path increase: {T} -> {Tnew} us (+5%, below 10% G3 threshold)")
print()
print(f"{'Step':>6} {'A:bdp=min(x,mr)':>18} {'B:bdp=x_est':>16} {'ideal(Tnew)':>14}")
step_markers = [1, 5, 10, 20, 30, 50, 70, 100]
for marker in step_markers:
    avgA = 0
    avgB = 0
    for s in range(seeds):
        rnd.seed(s * 1000 + marker)
        a = Est(T)
        b = Est(T)
        for _ in range(500):
            n = rnd.gauss(0, sigma)
            z = max(0, int(T + n))
            a.stepA(z)
            b.stepB(z)
        for _step in range(1, marker + 1):
            n = rnd.gauss(0, sigma)
            z = max(0, int(Tnew + n))
            xa, bdpa = a.stepA(z)
            xb = b.stepB(z)
        avgA += bdpa
        avgB += xb
    avgA //= seeds
    avgB //= seeds
    print(f"{marker:>5} {avgA:>17} {avgB:>16} {Tnew:>14}")

print()
print(f"A: permanent {-((Tnew - avgA) * 100 // Tnew)}% BDP underestimation")
print(f"B: x_est converges to within {((avgB - Tnew) * 100 // Tnew)}% of Tnew")
