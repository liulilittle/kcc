#!/usr/bin/env python3
"""Verify key boundary conditions with actual simulation data."""

import math
import random

SCALE = 1024
GROWTH_NUM = 122
GROWTH_DEN = 1000


def gauss(mean=0, std=1):
    u1, u2 = random.random(), random.random()
    return mean + std * math.sqrt(-2 * math.log(u1)) * math.cos(2 * math.pi * u2)


class GeodesicEstimator:
    def __init__(self, T_prop, sigma):
        self.x = T_prop * SCALE
        self.mr = T_prop
        self.conf = 0
        self.conf_slow = 0
        self.T = T_prop
        self.sigma = sigma
        self.pathGrown = False
        self.g3_events = 0

    def set_path_changed(self, newT):
        self.T = newT
        self.pathGrown = True

    def step(self, rtt, ql=0):
        z = rtt * SCALE
        v = z - self.x
        if v <= 0:
            self.x = min(self.x, z)
        else:
            g = (self.x * GROWTH_NUM) // GROWTH_DEN
            self.x = min(self.x + g, z)
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
            self.g3_events += 1
            om = self.mr
            self.mr = self.x // SCALE
            self.conf = 0
            self.conf_slow = 0
            if self.pathGrown and self.mr > om:
                self.T = self.mr

    def bdp(self):
        xu = self.x // SCALE
        return min(xu, self.mr)


# ═══════════════════════════════════════════════════
# B2: Pure Noise Path - False Positive Rate
# ═══════════════════════════════════════════════════
print("--- B2: False Positive Rate ---")
for T in [500, 10000, 100000, 500000, 10000000]:
    sigma = max(1, T // 100)
    fp = 0
    seeds = 100
    steps = 10000
    for s in range(seeds):
        random.seed(T + s)
        est = GeodesicEstimator(T, sigma)
        fired = False
        for _ in range(steps):
            rtt = max(1, T + round(gauss(0, sigma)))
            est.step(rtt)
            if est.g3_events > 0:
                fired = True
                break
        if fired:
            fp += 1
    print(f"  T={T}us: {fp}/{seeds} false positives ({fp / seeds * 100:.1f}%)")

# ═══════════════════════════════════════════════════
# B4: Path Increase Detection
# ═══════════════════════════════════════════════════
print("\n--- B4: Path Increase Detection ---")
for T in [100, 1400, 50000]:
    sigma = max(1, T // 100)
    for amp in [5, 10, 25, 50, 100, 200]:
        Tn = T + int(T * amp / 100)
        if Tn == T:
            continue
        det = 0
        delays = []
        seeds = 20
        for s in range(seeds):
            random.seed(T * 1000 + amp + s)
            est = GeodesicEstimator(T, sigma)
            for _ in range(2000):
                est.step(max(1, T + round(gauss(0, sigma))))
            est.set_path_changed(Tn)
            for i in range(1, 501):
                est.step(max(1, Tn + round(gauss(0, sigma))))
                if est.bdp() > T + int(T * 0.02):
                    delays.append(i)
                    det += 1
                    break
        med = sum(delays) / len(delays) if delays else float("inf")
        print(f"  T={T}us +{amp}%: {det}/20 det, med={med:.1f} RTTs")

# ═══════════════════════════════════════════════════
# B3: Congested Path - BDP Safety
# ═══════════════════════════════════════════════════
print("\n--- B3: BDP Safety Under Congestion ---")
for T, qmax, drain, label in [
    (1400, 400, 30, "DC"),
    (50000, 5000, 80, "WAN"),
    (300000, 20000, 150, "LH"),
]:
    sigma = max(1, T // 100)
    safe = 0
    for s in range(20):
        random.seed(T + s)
        est = GeodesicEstimator(T, sigma)
        q = 0
        for i in range(50000):
            if i % drain == 0:
                q = 0
            else:
                q += round(gauss(0, sigma * 0.5))
                q = max(0, min(qmax, q))
            est.step(max(1, T + q + round(gauss(0, sigma))), q)
        if est.bdp() < T + int(T * 0.02):
            safe += 1
    print(f"  {label}: {safe}/20 safe (BDP inflation<2%)")

# ═══════════════════════════════════════════════════
# B5: Path Decrease
# ═══════════════════════════════════════════════════
print("\n--- B5: Path Decrease Recovery ---")
for T in [1400, 50000]:
    sigma = max(1, T // 100)
    Tn = T // 2
    rec = 0
    delays = []
    for s in range(20):
        random.seed(T + s)
        est = GeodesicEstimator(T, sigma)
        for _ in range(2000):
            est.step(max(1, T + round(gauss(0, sigma))))
        old_bdp = est.bdp()
        for i in range(1, 501):
            est.step(max(1, Tn + round(gauss(0, sigma))))
            if est.bdp() <= Tn + int(Tn * 0.05):
                delays.append(i)
                rec += 1
                break
    med = sum(delays) / len(delays) if delays else float("inf")
    print(f"  T={T}->{Tn}us: {rec}/20 recovered, med={med:.1f} steps")
print("\nDone.")
