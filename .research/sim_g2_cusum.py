#!/usr/bin/env python3
import contextlib
import math
import sys

with contextlib.suppress(Exception):
    sys.stdout.reconfigure(encoding="utf-8")

SCALE = 1024
GROWTH_NUM = 122
GROWTH_DEN = 1000


def seed_random(seed):
    state = seed

    def rng():
        nonlocal state
        state = (state * 1664525 + 1013904223) & 0xFFFFFFFF
        return (state & 0xFFFFFFFF) / 0xFFFFFFFF

    return rng


def gauss(rng, mean=0, std=1):
    u1, u2 = rng(), rng()
    return mean + std * math.sqrt(-2 * math.log(u1)) * math.cos(2 * math.pi * u2)


def median(arr):
    if not arr:
        return float("nan")
    s = sorted(arr)
    m = len(s) // 2
    return s[m] if len(s) % 2 != 0 else (s[m - 1] + s[m]) / 2


def percentile(arr, p):
    if not arr:
        return float("nan")
    s = sorted(arr)
    idx = max(0, math.ceil(len(s) * p / 100) - 1)
    return s[idx]


class CUSUMDetector:
    def __init__(self, T_prop, sigma, delta=None, h=None):
        self.x = T_prop * SCALE
        self.mr = T_prop
        self.sigma = sigma
        self.delta = delta or int(T_prop * 0.05)
        self.h = h or int(T_prop * SCALE * 0.15)
        self.S_pos = 0
        self.detected = False
        self.currentT = T_prop

    def set_path_changed(self, newT):
        self.currentT = newT
        self.S_pos = 0
        self.detected = False

    def step(self, rng, rtt, ql=0):
        actual = rtt
        self.mr = min(self.mr, actual)
        z = actual * SCALE
        v = z - self.x
        if v <= 0:
            self.x = min(self.x, z)
        else:
            drift = self.delta * SCALE
            if v > drift:
                self.S_pos = max(0, self.S_pos + (v - drift))
        if self.S_pos >= self.h and not self.detected:
            self.detected = True
            self.x = max(self.x, z)
            self.mr = max(self.mr, actual)

    def bdp(self):
        x_us = self.x // SCALE
        return min(x_us, self.mr)


class GeodesicEstimator:
    def __init__(self, T_prop, sigma):
        self.x = T_prop * SCALE
        self.mr = T_prop
        self.conf = 0
        self.conf_slow = 0
        self.T = T_prop
        self.sigma = sigma
        self.pathGrown = False

    def set_path_changed(self, newT):
        self.T = newT
        self.pathGrown = True
        self.conf = 0
        self.conf_slow = 0

    def step(self, rng, rtt, ql=0):
        actual = rtt
        z = actual * SCALE
        v = z - self.x
        if v <= 0:
            self.x = min(self.x, z)
        else:
            growth = (self.x * GROWTH_NUM) // GROWTH_DEN
            self.x = min(self.x + growth, z)
        thresh_fast = (self.mr * 11 * SCALE) // 10
        thresh_slow = (self.mr * 21 * SCALE) // 20
        if self.x >= thresh_fast:
            self.conf += 1
            self.conf_slow += 1
        elif self.x >= thresh_slow:
            self.conf = 0
            self.conf_slow += 1
        else:
            self.conf = 0
        if self.x <= self.mr * SCALE:
            self.conf = 0
            self.conf_slow = 0
        if self.conf >= 4 or self.conf_slow >= 5:
            oldMr = self.mr
            self.mr = self.x // SCALE
            self.conf = 0
            self.conf_slow = 0
            if self.pathGrown and self.mr > oldMr:
                self.T = self.mr

    def bdp(self):
        x_us = self.x // SCALE
        return min(x_us, self.mr)


def compare():
    RTTs = [
        25,
        50,
        100,
        200,
        500,
        1000,
        1400,
        2000,
        5000,
        10000,
        50000,
        100000,
        300000,
        500000,
        1000000,
    ]
    GROWTHS = [5, 10, 25, 50, 100, 200]
    SEEDS = 20
    MAX_STEPS = 500

    sep = "=" * 180
    print(sep)
    print("G2 (12.2% GEOMETRIC) vs CUSUM (δ=5% T_prop, h=15% T_prop)")
    print(sep)
    header = (
        f"{'RTT(µs)':>9}{'Amp%':>6}{'G2_Det%':>9}{'G2_Med':>8}{'G2_P90':>8}"
        f"{'CUSUM_Det%':>11}{'CUSUM_Med':>10}{'CUSUM_P90':>10}{'Winner':>8}{'Note':>15}"
    )
    print(header)
    print("-" * 180)

    for T in RTTs:
        sigma = max(1, T // 100)
        for amp in GROWTHS:
            Tnew = T + int(T * amp / 100)
            if Tnew == T:
                continue

            g2_delays = []
            for seed in range(SEEDS):
                rng = seed_random(T * 1000 + amp + seed)
                est = GeodesicEstimator(T, sigma)
                for _ in range(2000):
                    est.step(rng, max(1, T + round(gauss(rng, 0, sigma))))
                est.set_path_changed(Tnew)
                for s in range(1, MAX_STEPS + 1):
                    est.step(rng, max(1, Tnew + round(gauss(rng, 0, sigma))))
                    if est.bdp() > T + int(T * 0.02):
                        g2_delays.append(s)
                        break
            cusum_delays = []

            for seed in range(SEEDS):
                rng = seed_random(T * 1000 + amp + seed + 100000)
                est = CUSUMDetector(T, sigma)
                for _ in range(2000):
                    est.step(rng, max(1, T + round(gauss(rng, 0, sigma))))
                est.set_path_changed(Tnew)
                for s in range(1, MAX_STEPS + 1):
                    est.step(rng, max(1, Tnew + round(gauss(rng, 0, sigma))))
                    if est.bdp() > T + int(T * 0.02):
                        cusum_delays.append(s)
                        break
            g2DetPct = f"{len(g2_delays) / SEEDS * 100:.1f}"
            g2Med = f"{median(g2_delays):.1f}" if g2_delays else "-"
            g2P90 = str(percentile(g2_delays, 90)) if g2_delays else "-"

            csDetPct = f"{len(cusum_delays) / SEEDS * 100:.1f}"
            csMed = f"{median(cusum_delays):.1f}" if cusum_delays else "-"
            csP90 = str(percentile(cusum_delays, 90)) if cusum_delays else "-"

            g2mv = median(g2_delays) if g2_delays else float("inf")
            csmv = median(cusum_delays) if cusum_delays else float("inf")
            if g2mv < csmv:
                winner = "G2"
            elif csmv < g2mv:
                winner = "CUSUM"
            else:
                winner = "TIE"

            note = "CUSUM MISS" if len(cusum_delays) < SEEDS else ""
            line = (
                f"{T!s:>9}{(str(amp) + '%'):>6}{(g2DetPct + '%'):>9}{g2Med!s:>8}{g2P90!s:>8}"
                f"{(csDetPct + '%'):>11}{csMed!s:>10}{csP90!s:>10}{winner:>8}{note:>15}"
            )
            print(line)


if __name__ == "__main__":
    print("\nCUSUM vs G2 COMPARISON\n")
    print("Notes:")
    print("1. CUSUM applies running-min update (v<=0: x_est = min(x_est, z))")
    print("2. setPathChanged resets S_pos and detected flag")
    print("Ensures bdp() = min(x_est, min_rtt) exceeds old baseline immediately\n")
    compare()
