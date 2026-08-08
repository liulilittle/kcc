#!/usr/bin/env python3
"""Geodesic EXACT verification - PRECISE match of tcp_kcc.c."""

import random
import sys
import time

sys.stdout.reconfigure(encoding="utf-8")
SCALE = 1024
SHIFT = 10
GROWTH_NUM = 122
GROWTH_DEN = 1000
CONFIRM_W = 4
THRESH_NUM = 11
THRESH_DEN = 10


class Geo:
    """EXACT replica of tcp_kcc.c geodesic estimator."""

    def __init__(self, T, sig):
        self.x = T * SCALE
        self.mr = T
        self.conf = 0
        self.conf_slow = 0
        self.pd_cnt = 0
        self.T = T
        self.sig = sig

    def step(self, rtt):
        z = rtt * SCALE
        v = z - self.x
        if v <= 0:
            self.x = min(self.x, z)  # G1: TOBIT min
        else:
            growth = self.x * GROWTH_NUM // GROWTH_DEN
            self.x = min(self.x + growth, z)  # G2: 12.2% growth, capped at z
        if self.x >= self.mr * SCALE * THRESH_NUM // THRESH_DEN:
            self.conf += 1  # G3: confirm_cnt++
            self.conf_slow += 1
        elif self.x >= self.mr * SCALE * 21 // 20:
            self.conf = 0
            self.conf_slow += 1
        else:
            self.conf = 0
        if self.x <= self.mr * SCALE:
            self.conf = 0
            self.conf_slow = 0
        if self.conf >= CONFIRM_W:
            self.mr = self.x >> SHIFT  # G3: min_rtt = x_est
            self.conf = 0
            self.conf_slow = 0
        elif self.conf_slow >= 5:
            self.mr = self.x >> SHIFT
            self.conf = 0
            self.conf_slow = 0
        if self.conf == 0 and self.conf_slow == 0:
            x_us = self.x >> SHIFT
            if x_us < self.mr:
                self.pd_cnt += 1
                if self.pd_cnt >= 3:
                    self.mr = x_us
                    self.pd_cnt = 0
            else:
                self.pd_cnt = 0

    def bdp(self):
        x_us = self.x >> SHIFT
        return min(self.mr, x_us)  # G4: min(x_est, min_rtt)


RTTs = [
    100,
    200,
    300,
    500,
    750,
    1000,
    1400,
    2000,
    3000,
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
STEPS = [5, 10, 25, 50, 100, 200]
SEEDS = 30

_f = 0


def fail(m):
    global _f
    print(f"  FAIL: {m}")
    _f += 1


def ok(m):
    print(f"  PASS: {m}")


t0 = time.time()
print("=" * 66)
print(
    f"GEODESIC EXACT (12.2% growth, conf=4, theta=1.1): {len(RTTs)}RTTx{len(STEPS)}stepsx{SEEDS}seeds",
)
print("=" * 66)

total = 0
detected = 0
latencies = []
results = {}

for T in RTTs:
    sig = max(1, T // 100)
    for sp in STEPS:
        Tn = T + int(T * sp / 100)
        if Tn == T:
            continue
        det = 0
        lats = []
        for seed in range(SEEDS):
            rng = random.Random(T * sp + seed * 100 + int(T * 0.1))
            geo = Geo(T, sig)

            # converge to old T
            for _ in range(3000):
                geo.step(max(1, T + int(rng.gauss(0, sig))))

            # detect path increase
            found = False
            for s in range(1, 1000):
                geo.step(max(1, Tn + int(rng.gauss(0, sig))))
                if geo.bdp() > T * 1.02:
                    det += 1
                    lats.append(s)
                    found = True
                    break
            if not found:
                geo_lat = None

        rate = det * 100.0 / SEEDS
        avg = sum(lats) / max(1, len(lats))
        key = f"T={T:>7d}us +{sp:>3d}%"
        results[key] = rate
        total += SEEDS
        detected += det

        if rate >= 95:
            status = "OK"
        elif rate >= 80:
            status = f"OK ({rate:.0f}%)"
        else:
            status = f"LOW {rate:.0f}%"
        # Only show non-100% entries
        if rate < 100:
            print(f"  {key}: {det}/{SEEDS} ({rate:.0f}%), avg {avg:.1f} RTTs")

print("\n--- Overall ---")
overall = detected * 100.0 / total
print(f"  Detection: {detected}/{total} = {overall:.1f}%")

# Breakdown by T_prop range
print("\n--- By T_prop range ---")
ranges = [
    ("DC (<2ms)", [t for t in RTTs if t < 2000]),
    ("Campus (2-20ms)", [t for t in RTTs if 2000 <= t < 20000]),
    ("WAN (20-200ms)", [t for t in RTTs if 20000 <= t < 200000]),
    ("LH (>200ms)", [t for t in RTTs if t >= 200000]),
]
for label, ts in ranges:
    d = sum(
        1
        for T in ts
        for sp in STEPS
        for seed in range(SEEDS)
        if T + int(T * sp / 100) > T and results.get(f"T={T:>7d}us +{sp:>3d}%", 0) > 0
    )
    t = sum(SEEDS for T in ts for sp in STEPS if T + int(T * sp / 100) > T)
    # Actually compute from results
    matches = [(T, sp) for T in ts for sp in STEPS if T + int(T * sp / 100) > T]
    if matches:
        d = sum(
            results.get(f"T={T:>7d}us +{sp:>3d}%", 0) * SEEDS / 100.0
            for T, sp in matches
        )
        t = len(matches) * SEEDS
        pct = d * 100.0 / t
        print(f"  {label:>15s}: {d:.0f}/{t} = {pct:.1f}%")

# Breakdown by step size
print("\n--- By step size ---")
for sp in STEPS:
    matches = [(T, sp) for T in RTTs if T + int(T * sp / 100) > T]
    if matches:
        d = sum(
            results.get(f"T={T:>7d}us +{sp:>3d}%", 0) * SEEDS / 100.0
            for T, sp2 in matches
        )
        t = len(matches) * SEEDS
        pct = d * 100.0 / t
        print(f"  {'+' + str(sp) + '%':>7s}: {d:.0f}/{t} = {pct:.1f}%")

# Where do 5% increases fail?
print("\n--- 5% increases (cap-at-z analysis) ---")
for T in RTTs:
    sp = 5
    Tn = T + int(T * sp / 100)
    rate = results.get(f"T={T:>7d}us +{sp:>3d}%", 0)
    # Theoretical cap analysis
    h = Tn / T
    capped = h < 1.1  # cap at z prevents reaching threshold
    print(f"  T={T:>7d}us: {rate:.0f}% detected, h={h:.3f}, capped={capped}")

# False positive test
print("\n--- FALSE POSITIVE (H0, pure noise) ---")
fp_total = 0
for T in [500, 1400, 5000, 50000, 300000]:
    for seed in range(100):
        sig = max(1, T // 100)
        rng = random.Random(T * 100000 + seed * 9999)
        geo = Geo(T, sig)
        fp = False
        for _ in range(10000):
            geo.step(max(1, T + int(rng.gauss(0, sig))))
            if geo.bdp() > T * 1.1:
                fp = True
                break
        if fp:
            fp_total += 1
fp_rate = fp_total * 100.0 / (5 * 100)
print(f"  False positives: {fp_total}/{5 * 100} = {fp_rate:.2f}%")

# Congestion test
print("\n--- CONGESTION BDP SAFETY ---")
cong_configs = [
    (1400, 20, 400, "DC"),
    (50000, 200, 5000, "WAN"),
    (300000, 500, 20000, "LH"),
]
cong_ok = 0
cong_total = 0
for T, sig, Q, label in cong_configs:
    for seed in range(20):
        rng = random.Random(T * 100 + seed * 9999)
        geo = Geo(T, sig)
        for _ in range(50000):
            geo.step(max(1, T + Q + int(rng.gauss(0, sig))))
        b = geo.bdp()
        infl = (b - T) * 100.0 / T if b > T else 0
        cong_total += 1
        if infl < 2:
            cong_ok += 1
    print(f"  {label}: {cong_ok}/{cong_total} safe (<2% inflation)")

# Deadlock test
print("\n--- DEADLOCK RECOVERY (5.5x overestimate) ---")
dl_total = 0
dl_ok = 0
for T, sig in [(1400, 20), (50000, 200), (300000, 500), (1000000, 1000)]:
    for seed in range(100):
        rng = random.Random(T * 100000 + seed * 9999)
        geo = Geo(T, sig)
        geo.x = int(T * 5.5 * SCALE)
        for _ in range(500):
            geo.step(max(1, T + int(rng.gauss(0, sig))))
            if geo.bdp() < T * 1.05:
                dl_ok += 1
                break
        dl_total += 1
dl_rate = dl_ok * 100.0 / dl_total
print(f"  Recovered: {dl_ok}/{dl_total} = {dl_rate:.1f}%")

print(f"\n=== COMPLETE ({int(time.time() - t0)}s) ===")
