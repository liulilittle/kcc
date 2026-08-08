#!/usr/bin/env python3
"""
noise_resistance.py -- Quantify geodesic estimator's natural noise immunity.

Tests G1/G2/G3 steady-state behavior under Gaussian noise with no path change (H0).

Components:
  G1: instant downward convergence (x = min(x, z) when v <= 0)
  G2: 12.2% geometric growth capped at observation
  G3: dual-threshold — fast 10%/4-count + slow 5%/5-count
"""

import math
import sys

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


class GeodesicEstimator:
    """Exact port from cusum_vs_g2.js — three-mechanism geodesic estimator."""

    def __init__(self, T_prop, sigma):
        self.x = T_prop * SCALE
        self.mr = T_prop
        self.conf = 0
        self.conf_slow = 0
        self.T = T_prop
        self.sigma = sigma

    def step(self, rtt):
        z = rtt * SCALE
        v = z - self.x
        g3_fired = False
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
            self.mr = self.x // SCALE
            self.conf = 0
            self.conf_slow = 0
            g3_fired = True
        x_us = self.x // SCALE
        return g3_fired, x_us, min(x_us, self.mr)


def run_test(T_prop_us, noise_sigma_us, num_seeds=50, num_steps=2000):
    g3_total = 0
    g3_seeds = 0
    abs_deviations = []
    max_dev = 0
    minrtt_correct = 0

    for seed in range(num_seeds):
        rng = seed_random(T_prop_us * 1000 + int(noise_sigma_us * 100) + seed)
        est = GeodesicEstimator(T_prop_us, noise_sigma_us)
        seed_g3 = 0
        for _ in range(num_steps):
            rtt = max(1, T_prop_us + round(gauss(rng, 0, noise_sigma_us)))
            g3, x_us, _bdp = est.step(rtt)
            if g3:
                g3_total += 1
                seed_g3 += 1
            dev = abs(x_us - T_prop_us)
            abs_deviations.append(dev)
            max_dev = max(max_dev, dev)
            if est.mr == T_prop_us:
                minrtt_correct += 1
        if seed_g3 > 0:
            g3_seeds += 1

    total = num_seeds * num_steps
    mad = sum(abs_deviations) / total
    fp_rate = (g3_seeds / num_seeds) * 100
    minrtt_acc = (minrtt_correct / total) * 100
    return fp_rate, mad, max_dev, minrtt_acc


def main():
    RTTs = [1000, 10000, 100000, 500000, 1000000]
    NOISE_PCTS = [1, 5, 10, 25, 50]
    SEEDS = 50
    STEPS = 2000

    sep = "=" * 130
    print(sep)
    print("  GEODESIC ESTIMATOR  —  NOISE RESISTANCE ANALYSIS  (H0: no path change)")
    print(
        "  G1: instant downward (x = min(x,z) when v<=0)  |  G2: 12.2% geometric growth capped at z",
    )
    print(
        "  G3: dual-threshold (10%%/4-count + 5%%/5-count)  |  Seeds=%d, Steps/run=%d"
        % (SEEDS, STEPS),
    )
    print(sep)
    hdr = "  %10s  %8s  %10s  %10s  %12s  %14s" % (
        "RTT(us)",
        "Noise%",
        "FP_Rate%",
        "MAD(us)",
        "MaxDev(us)",
        "MinRTT_Acc%",
    )
    print(hdr)
    print("  " + "-" * 126)

    for T in RTTs:
        for npct in NOISE_PCTS:
            sigma = T * npct / 100.0
            fp, mad, maxdev, minacc = run_test(T, sigma, SEEDS, STEPS)
            print(
                "  %10d  %7d%%  %9.2f%%  %9.2f  %10d  %13.2f%%"
                % (T, npct, fp, mad, maxdev, minacc),
            )

    print("  " + "-" * 126)
    print(
        "  FP_Rate%%   = %s of seeds (out of %d) where G3 triggered at least once"
        % ("%", SEEDS),
    )
    print("  MAD(us)    = mean absolute deviation of x_est from true T_prop")
    print("  MaxDev(us) = peak |x_est - T_prop| across all steps and seeds")
    print(
        "  MinRTT_Acc% = {} of time min_rtt == true T_prop (not inflated by false G3)".format(
            "%",
        ),
    )


if __name__ == "__main__":
    main()
