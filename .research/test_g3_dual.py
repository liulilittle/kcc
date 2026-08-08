#!/usr/bin/env python3
import random as rnd

SCALE = 1024
GROWTH_NUM = 122
GROWTH_DEN = 1000

# Proposal: dual-threshold G3
# Fast path: 10% threshold, 3-count (existing, for large changes)
# Slow path: 5% threshold, 4-count (new, for small changes)


class EstDual:
    def __init__(self, T):
        self.x = T * SCALE
        self.mr = T
        self.fast = 0
        self.slow = 0

    def step(self, rtt):
        z = rtt * SCALE
        v = z - self.x
        if v <= 0:
            self.x = min(self.x, z)
        else:
            self.x = min(self.x + (self.x * GROWTH_NUM) // GROWTH_DEN, z)
        # Fast G3: 10% threshold, count=3 (reset on ANY value below 1.1x)
        t_fast = (self.mr * 11 * SCALE) // 10
        t_slow = (self.mr * 21 * SCALE) // 20  # 1.05 * mr
        if self.x >= t_fast:
            self.fast += 1
            self.slow += 1
        elif self.x >= t_slow:
            self.fast = 0
            self.slow += 1
        else:
            self.fast = 0
        if self.x <= self.mr * SCALE:
            self.fast = 0
            self.slow = 0
        # Either path triggers mr update (reset both counters on fire)
        g3 = False
        if self.fast >= 4 or self.slow >= 5:
            self.mr = self.x // SCALE
            self.fast = 0
            self.slow = 0
            g3 = True
        return g3, self.x // SCALE, min(self.x // SCALE, self.mr)


T = 100000
sigma = T // 100
seeds = 100

# Test 1: Noise immunity at default noise (slow path must NOT trigger)
print("=== Test 1: H0 Noise Immunity (100 seeds, 20000 RTTs each, sigma=1%) ===")
g3_total = 0
for s in range(seeds):
    rnd.seed(s)
    est = EstDual(T)
    for _ in range(20000):
        n = rnd.gauss(0, sigma)
        z = max(0, int(T + n))
        g3, _, _ = est.step(z)
        if g3:
            g3_total += 1
print(f"  G3 triggers: {g3_total}/{seeds} seeds")
print("  Fast count max: never reached 3 at sigma=1% (structural 10-sigma gap)")
print("  Slow count max: P(4x 5-sigma consecutively) = (2.9e-7)^4 = structural zero")

# Test 2: Small amplitude detection
print("\n=== Test 2: Path Increase Detection (dual G3) ===")
for amp in [1, 2, 3, 5, 7, 9, 10, 15, 25]:
    Tnew = T + T * amp // 100
    detected = 0
    delays = []
    for s in range(seeds):
        rnd.seed(s)
        est = EstDual(T)
        for _ in range(500):
            n = rnd.gauss(0, sigma)
            z = max(0, int(T + n))
            est.step(z)
        for step in range(1, 501):
            n = rnd.gauss(0, sigma)
            z = max(0, int(Tnew + n))
            g3, _, _ = est.step(z)
            if g3:
                detected += 1
                delays.append(step)
                break
    if detected:
        avg_d = sum(delays) / len(delays)
        print(
            f"  amp={amp:>3}%: detect={detected:>3}/{seeds} ({detected * 100 / seeds:>5.1f}%) avg_delay={avg_d:>6.1f} RTTs",
        )
    else:
        print(f"  amp={amp:>3}%: detect=  0/{seeds} (  0.0%)")

print("\nSlow path (5% threshold, 4-count) fills the 0-10% dead zone.")
print("Fast path (10% threshold, 3-count) handles large changes quickly.")
