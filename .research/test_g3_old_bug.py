#!/usr/bin/env python3
import random as rnd

SCALE = 1024
GROWTH_NUM = 122
GROWTH_DEN = 1000


# Original buggy code
class EstOld:
    def __init__(self, T):
        self.x = T * SCALE
        self.mr = T
        self.conf = 0

    def step(self, rtt):
        self.mr = min(self.mr, rtt)  # BUG: noise pollutes mr
        z = rtt * SCALE
        v = z - self.x
        if v <= 0:
            self.x = min(self.x, z)  # G1
        else:
            self.x = min(self.x + (self.x * GROWTH_NUM) // GROWTH_DEN, z)  # G2
        t = (self.mr * 11 * SCALE) // 10
        if self.x > t:
            self.conf += 1  # G3 not gated by innovation sign
        elif self.x < self.mr * SCALE:
            self.conf = 0
        if self.conf >= 4:
            self.mr = self.x // SCALE
            self.conf = 0
        return min(self.x // SCALE, self.mr)


T = 100000
sigma = T // 100
seeds = 100

print("=== Original code: mr baseline drift during 500-RTT warmup ===")
avg_mr = 0
for s in range(seeds):
    rnd.seed(s)
    est = EstOld(T)
    for _ in range(500):
        n = rnd.gauss(0, sigma)
        z = max(0, int(T + n))
        est.step(z)
    avg_mr += est.mr
avg_mr //= seeds
print(f"  True T_prop: {T} us")
print(
    f"  mr after 500 RTTs of noise: {avg_mr} us ({-(T - avg_mr) * 100 / T:.1f}% below T_prop)",
)
print(f"  G3 threshold: 1.1 * {avg_mr} = {int(avg_mr * 1.1)} us")
print(
    f"  A +5% path increase (T_new={T * 105 // 100} us) would need z > {int(avg_mr * 1.1)}",
)
print(
    f"  z = {T * 105 // 100} + noise, need noise > {int(avg_mr * 1.1 - T * 105 // 100)} us = {int(avg_mr * 1.1 - T * 105 // 100)} sigma",
)
print("  => detection via CORRUPTED baseline, not actual 10% threshold")
print()

print("=== What mr actually represents ===")
print(f"  Correct mr:  {T} us (true T_prop)")
print(f"  Actual mr:   {avg_mr} us (noise-corrupted)")
print(f"  BDP understatement: {-(T - avg_mr) * 100 / T:.1f}% (permanent)")
print(
    "  The 'detection' of small changes was buying sensitivity at the cost of accuracy.",
)
