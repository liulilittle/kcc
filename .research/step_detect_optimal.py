#!/usr/bin/env python3
"""OPTIMAL: Separate BDP (always min_rtt) from x_est (free-running Kalman).
When x_est > min_rtt consistently AND qdelay is low -> update min_rtt."""

import random

SCALE = 1024
PINIT = 1000
PMAX = int(1e8)
SHIFT = 10


class K:
    def __init__(self, T, sigma):
        self.Tp = T
        self.s = sigma
        self.x = T * SCALE
        self.mr = T
        self.p = PINIT
        self.j = 0
        self.q = 0
        self.pos_ratio = 0.5

    def step(self, rtt):
        self.mr = max(min(self.mr, rtt), 1)
        z = rtt * SCALE
        v = z - self.x
        av = v if v >= 0 else -v
        pp = min(self.p + 100, PMAX)
        # pos_ratio: leaky integrator of innovation sign (0=all neg, 1=all pos)
        self.pos_ratio = self.pos_ratio * 0.95 + (0.05 if v > 0 else 0.0)
        if v <= 0:
            self.x = z
            self.p = max(400, 10)
        else:
            # Adaptive upward gain based on pos_ratio (0.05 slow, 0.3 fast)
            alpha = 0.05 + self.pos_ratio * 0.25  # range: 0.05 to 0.30
            gd = pp + 400
            corr = (
                int((pp * av) // gd * alpha / 0.347) if gd else 0
            )  # scale from K to alpha
            self.x = min(self.x + corr, 0xFFFFFFFF)
            p_reduc = (pp * pp) // gd if gd else 0
            self.p = max(pp - p_reduc, 10)
        self.j = self.j * 0.875 + (av >> SHIFT) * 0.125
        self.q = self.q * 0.875 + max(0, rtt - self.mr) * 0.125


print(f"{'RTT':>7s} {'Step':>6s} {'Det%':>5s} {'p50':>4s} {'p90':>5s} {'p50s':>6s}")
RTTs = [250, 500, 1400, 5000, 50000, 200000, 500000, 1000000]
for T in RTTs:
    sigma = max(1, T // 100)
    for step_pct in [5, 10, 25, 50, 100, 200]:
        Tn = T + int(T * step_pct / 100)
        if Tn == T:
            continue
        delays = []
        missed = 0
        for seed in range(10):
            rng = random.Random(T * step_pct + seed)
            k = K(T, sigma)
            for _ in range(2000):
                k.step(max(1, T + int(rng.gauss(0, sigma))))
            for s in range(1, 500):
                k.step(max(1, Tn + int(rng.gauss(0, sigma))))
                if k.x // SCALE > Tn * 0.9:
                    delays.append(s)
                    break
            else:
                missed += 1
        if delays:
            d = sorted(delays)
            p50 = d[len(d) // 2]
            p90 = d[int(len(d) * 0.9)]
            print(
                f"{T:>7d} {step_pct:>+4d}% {50 - missed:>4d}/50 {p50:>4d} {p90:>5d} {p50 * T / 1000:>5.0f}s",
            )
