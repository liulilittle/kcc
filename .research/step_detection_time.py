#!/usr/bin/env python3
"""STEP DETECTION TIME: 250us--1000ms RTT, all step magnitudes."""

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
        self.ps = 0
        self.ds = 0
        self.cr = 0
        self.cd = 0

    def step(self, rtt):
        self.mr = max(min(self.mr, rtt), 1)
        z = rtt * SCALE
        v = z - self.x
        av = v if v >= 0 else -v
        pp = min(self.p + 100, PMAX)
        if self.cd > 0:
            self.cd -= 1
        if self.cd == 0 and v > 0 and av > 16384000 and self.ps < 5:
            self.p = PINIT
            self.cd = 6
            self.ps = 0
            self.x = min(z, 0xFFFFFFFF)
            self.mr = min(self.mr, rtt)
            return
        if (
            v > 0
            and av > (int(self.q * SCALE) * 5) // 2
            and self.q < self.mr >> 1
            and self.ps >= 2
        ):
            self.x = min(z, 0xFFFFFFFF)
            self.p = max(400, 10)
            self.ps = 0
            self.mr = min(self.mr, rtt)
            return
        if v <= 0:
            self.x = z
            self.p = max(400, 10)
            self.ps = self.cr = self.ds = 0
        else:
            dt = max(max(self.mr >> 2, 50) * SCALE, max(1, int(self.j * 2)) * SCALE)
            if self.p <= 33 and av > dt and self.cr < 20:
                self.cr += 1
                self.ps += 1
                self.p = pp
            else:
                if self.cr >= 20:
                    self.cr = 0
                self.cr = 0
                gd = pp + 400
                self.p = max(pp - (pp * pp) // gd if gd else pp, 10)
                self.ps += 1
                self.ds += av >> SHIFT
                if self.ps >= 3 and self.j < self.mr >> 3 and self.ds > self.mr >> 5:
                    self.x = min(self.x + max(av >> 2, 1), 0xFFFFFFFF)
                    self.p = max(pp >> 2, 10)
                    self.ps = self.ds = 0
                elif self.ps >= 14 and self.j < self.mr >> 3:
                    ca = (pp * av) // (pp + 400) if pp + 400 else 0
                    self.x = min(self.x + max(ca >> 2, 1), 0xFFFFFFFF)
                    self.p = max(pp >> 2, 10)
                    self.ps = self.ds = 0
                elif self.ps >= 56:
                    ca = (pp * av) // (pp + 400) if pp + 400 else 0
                    self.x = min(self.x + max(ca >> 3, 1), 0xFFFFFFFF)
                    self.p = max(pp >> 3, 10)
                    self.ps = self.ds = 0
        self.j = self.j * 0.875 + (av >> SHIFT) * 0.125
        self.q = self.q * 0.875 + max(0, rtt - self.mr) * 0.125


RTTs = [250, 500, 1400, 5000, 50000, 200000, 500000, 1000000]
NS = 10
print(
    f"{'RTT':>7s} {'Step':>6s} {'Det%':>5s} {'p50':>5s} {'p90':>6s} {'p99':>6s} {'p50ms':>8s} {'p90ms':>8s}",
)

for T in RTTs:
    sigma = max(1, T // 100)
    for step_pct in [5, 10, 25, 50, 100, 200]:
        Tn = T + int(T * step_pct / 100)
        if Tn == T:
            continue
        delays = []
        missed = 0
        for seed in range(NS):
            rng = random.Random(T * step_pct + seed * 777)
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
            p99 = d[min(int(len(d) * 0.99), len(d) - 1)]
            det = NS - missed
            step_us = Tn - T
            print(
                f"{T:>7d} {step_pct:>+4d}% {det:>4d}/{NS} {p50:>4d} {p90:>5d} {p99:>5d} {p50 * T / 1000:>7.0f}ms {p90 * T / 1000:>7.0f}ms",
            )
