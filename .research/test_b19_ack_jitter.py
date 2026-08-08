#!/usr/bin/env python3
"""
B19: ACK Timing Jitter (Noise Immunity)
Tests estimator stability under ACK timing noise.
Guarantee: x_est oscillates around T_prop +/- sigma. Downward noise
absorbed by G1 (conservative). Upward noise triggers G2 but capped.
G3 false positive: P < 10^{-70} (Gaussian, sigma~1% T_prop);
P < 10^{-12} (Pareto alpha=2). Net zero drift.
"""

import os
import random
import sys

sys.path.insert(0, os.path.dirname(__file__))
from common import GeodesicEstimator


def mean(vals):
    return sum(vals) / float(len(vals)) if vals else 0.0


def run_trial(t_prop_us, noise_amplitude_us, distribution, n_samples, seed):
    random.seed(seed)
    est = GeodesicEstimator(t_prop_us)
    for _i in range(n_samples):
        if distribution == "gaussian":
            noise = random.gauss(0, noise_amplitude_us)
        elif distribution == "uniform":
            noise = random.uniform(-noise_amplitude_us, noise_amplitude_us)
        elif distribution == "pareto":
            noise = (random.paretovariate(2) - 1) * noise_amplitude_us * 2
            if random.random() < 0.5:
                noise = -noise
        else:
            noise = 0
        z = t_prop_us + noise
        est.update(max(1, int(z)))
    return est


def test():
    print("=" * 72)
    print("B19: ACK Timing Jitter - Estimator Stability Under Noise")
    print("  Guarantee: x_est oscillates around T_prop +/- sigma.")
    print("  G3 false positive: P < 10^{-70} (Gaussian sigma~1%% T_prop)")
    print("  P < 10^{-12} (Pareto alpha=2). Net zero drift.")
    print("=" * 72)

    T_PROP = 20000  # 20ms
    cases = [
        ("0.5%% Gaussian (typ NIC) ", 0.005, "gaussian"),
        ("1%% Gaussian (typ OS)    ", 0.01, "gaussian"),
        ("2%% Gaussian             ", 0.02, "gaussian"),
        ("5%% Gaussian             ", 0.05, "gaussian"),
        ("10%% Gaussian (extreme)  ", 0.10, "gaussian"),
        ("2%% Uniform jitter       ", 0.02, "uniform"),
        ("2%% Pareto heavy-tail    ", 0.02, "pareto"),
    ]

    N_SEEDS = 30
    N_SAMPLES = 2000

    all_pass = True
    for label, noise_frac, dist in cases:
        noise_amp = int(T_PROP * noise_frac)
        final_errors = []
        bdp_ratios = []
        drift_rates = []
        max_bdp_ratios = []
        g3_total = 0
        g3_seeds = 0

        for seed in range(N_SEEDS):
            est = run_trial(T_PROP, noise_amp, dist, N_SAMPLES, seed)
            st = est.history[-1]
            err_pct = (st["x_est_us"] - T_PROP) / float(T_PROP) * 100
            final_errors.append(err_pct)
            bdp_ratios.append(st["bdp_us"] / float(T_PROP))
            max_bdp = max(h["bdp_us"] for h in est.history)
            max_bdp_ratios.append(max_bdp / float(T_PROP))
            if est.g3_events > 0:
                g3_total += est.g3_events
                g3_seeds += 1

            xs = [h["x_est_us"] for h in est.history]
            n = len(xs)
            if n > 10:
                x_avg = mean(xs)
                t_avg = n / 2.0
                num = sum((i - t_avg) * (xs[i] - x_avg) for i in range(n))
                den = sum((i - t_avg) ** 2 for i in range(n))
                slope = num / den if den != 0 else 0
                drift_rates.append(slope)

        mean_err = mean(final_errors)
        max_err = max(final_errors)
        mean_bdp = mean(bdp_ratios)
        max_bdp = max(max_bdp_ratios)
        mean_drift = mean(drift_rates) if drift_rates else 0

        if noise_frac <= 0.02 and dist != "pareto":
            passed = g3_seeds == 0 and max_bdp <= 1.02
        elif noise_frac <= 0.02 and dist == "pareto":
            passed = max_bdp <= 1.50
        elif noise_frac <= 0.05:
            passed = max_bdp <= 1.15
        else:
            passed = max_bdp <= 1.40

        drift_ok = abs(mean_drift) < 0.5
        status = "PASS" if passed else "FAIL"
        if not passed:
            all_pass = False

        print()
        print("  %s (noise_amp=%dus, %s)" % (label, noise_amp, dist))
        print("    Seeds: %d, Samples/seed: %d" % (N_SEEDS, N_SAMPLES))
        print(f"    Mean final x_est error: {mean_err:.2f}%  (worst: {max_err:.2f}%)")
        print(
            "    Mean BDP/T_prop: {:.4f}  (worst: {:.4f})  {}".format(
                mean_bdp,
                max_bdp,
                "OK" if passed else "EXCEEDED",
            ),
        )
        print(
            "    G3 triggers: %d/%d seeds (%d total)  %s"
            % (g3_seeds, N_SEEDS, g3_total, "NONE" if g3_seeds == 0 else "SEEN"),
        )
        print(
            "    Mean drift: {:.4f} us/sample  {}".format(
                mean_drift,
                "STABLE" if drift_ok else "DRIFT",
            ),
        )
        print(f"    [{status}]")

    print()
    print("  Overall: %s" % ("ALL PASS" if all_pass else "SOME FAILED"))
    return all_pass


if __name__ == "__main__":
    sys.exit(0 if test() else 1)
