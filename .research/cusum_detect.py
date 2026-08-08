#!/usr/bin/env python3
"""CUSUM path-change detection + adaptive x_est + BDP=min_rtt."""

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
        self.cusum = 0

    def step(self, rtt):
        self.mr = max(min(self.mr, rtt), 1)
        z = rtt * SCALE
        v = z - self.x
        av = v if v >= 0 else -v
        pp = min(self.p + 100, PMAX)
        # CUSUM: accumulate positive innovations above noise floor
        delta = max(1, int(self.j)) * SCALE  # one sigma of jitter as noise floor
        if v > 0:
            self.cusum = max(0, self.cusum + v - delta)
        else:
            self.cusum = 0
        # Detection threshold: 10 sigma cumulative excess
        threshold = 10 * max(1, int(self.j)) * SCALE
        detected = self.cusum > threshold

        if detected:
            self.mr = rtt  # path increase: set directly, old mr is stale
            self.cusum = 0

        if v <= 0:
            self.x = z
            self.p = max(400, 10)
        else:
            # Slow upward tracking: adaptive alpha from cusum confidence
            confidence = min(1.0, self.cusum / max(threshold, 1))
            alpha = 0.02 + confidence * 0.20  # 0.02 slow, 0.22 fast when confident
            gd = pp + 400
            corr = int((pp * av) // gd * (alpha / 0.347)) if gd else 0
            self.x = min(self.x + corr, 0xFFFFFFFF)
            p_reduc = (pp * pp) // gd if gd else 0
            self.p = max(pp - p_reduc, 10)
        self.j = self.j * 0.875 + (av >> SHIFT) * 0.125


print(
    f"{'RTT':>7s} {'Step':>6s} {'Det%':>5s} {'p50':>4s} {'p90':>5s} {'p50s':>6s} {'p90s':>6s}",
)
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
    sigma = max(1, T // 100)
    for step in [5, 10, 25, 50, 100, 200]:
        Tn = T + int(T * step / 100)
        if Tn == T:
            continue
        delays = []
        missed = 0
        for seed in range(10):
            rng = random.Random(T * step + seed)
            k = K(T, sigma)
            for _ in range(2000):
                k.step(max(1, T + int(rng.gauss(0, sigma))))
            for s in range(1, 500):
                k.step(max(1, Tn + int(rng.gauss(0, sigma))))
                if k.mr > T * 1.5 or k.x // SCALE > Tn * 0.9:
                    delays.append(s)
                    break
            else:
                missed += 1
        if delays:
            d = sorted(delays)
            p50 = d[len(d) // 2]
            p90 = d[int(len(d) * 0.9)]
            det = 50 - missed
            print(
                f"{T:>7d} {step:>+4d}% {det:>4d}/{50} {p50:>4d} {p90:>5d} {p50 * T / 1000:>5.0f}s {p90 * T / 1000:>5.0f}s",
            )
