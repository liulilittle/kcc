#!/usr/bin/env python3
"""
B6: RTT Asymmetry (T_fwd != T_rev)
Guarantee: T_prop = T_fwd + T_rev. Geodesic operates on end-to-end RTT,
symmetric in (T_fwd, T_rev). BDP = total minimum. No direction bias.
"""

import os
import random
import sys

sys.path.insert(0, os.path.dirname(__file__))
from common import GeodesicEstimator


def mean(vals):
    return sum(vals) / float(len(vals))


def run_trial(t_prop_us, t_fwd_us, t_rev_us, noise_sigma_us, n_samples, seed):
    random.seed(seed)
    est = GeodesicEstimator(t_prop_us)
    for _ in range(n_samples):
        noise_fwd = random.gauss(0, noise_sigma_us * t_fwd_us / t_prop_us)
        noise_rev = random.gauss(0, noise_sigma_us * t_rev_us / t_prop_us)
        z = t_fwd_us + t_rev_us + noise_fwd + noise_rev
        est.update(max(1, int(z)))
    final = est.history[-1]
    x_est = final["x_est_us"]
    bdp = final["bdp_us"]
    max_bdp = max(h["bdp_us"] for h in est.history)
    return x_est, bdp, max_bdp, est.min_rtt_us


def test():
    print("=" * 72)
    print("B6: RTT Asymmetry (T_fwd != T_rev)")
    print("=" * 72)

    cases = [
        ("T_fwd=1ms, T_rev=9ms", 10000, 1000, 9000),
        ("T_fwd=5ms, T_rev=5ms", 10000, 5000, 5000),
        ("T_fwd=9ms, T_rev=1ms", 10000, 9000, 1000),
        ("T_fwd=2ms, T_rev=18ms", 20000, 2000, 18000),
        ("T_fwd=15ms,T_rev=5ms", 20000, 15000, 5000),
    ]

    N_SEEDS = 30
    N_SAMPLES = 200
    NOISE_SIGMA = 500

    all_pass = True
    for label, t_prop, t_fwd, t_rev in cases:
        errors = []
        bdp_ratios = []
        max_bdp_ratios = []
        for seed in range(N_SEEDS):
            x_est, bdp, max_bdp, _min_rtt = run_trial(
                t_prop,
                t_fwd,
                t_rev,
                NOISE_SIGMA,
                N_SAMPLES,
                seed,
            )
            err_pct = abs(x_est - t_prop) / float(t_prop) * 100
            errors.append(err_pct)
            bdp_ratios.append(bdp / float(t_prop))
            max_bdp_ratios.append(max_bdp / float(t_prop))

        mean_err = mean(errors)
        max_err = max(errors)
        mean_bdp_ratio = mean(bdp_ratios)
        max_bdp_ratio = max(max_bdp_ratios)
        pass_conv = mean_err < 5.0
        pass_bdp = max_bdp_ratio <= 1.15
        passed = pass_conv and pass_bdp
        if not passed:
            all_pass = False

        status = "PASS" if passed else "FAIL"
        print()
        print(f"  {label}  T_prop={t_prop / 1000.0:.1f}ms")
        print("    Seeds: %d, Samples/seed: %d" % (N_SEEDS, N_SAMPLES))
        print(
            "    Mean conv error: {:.2f}%  (max: {:.2f}%)  {}".format(
                mean_err,
                max_err,
                "OK" if pass_conv else "FAIL",
            ),
        )
        print(
            "    Mean BDP/T_prop: {:.4f}  (max: {:.4f})  {}".format(
                mean_bdp_ratio,
                max_bdp_ratio,
                "OK" if pass_bdp else "FAIL",
            ),
        )
        print(f"    [{status}]")

    print()
    print("  Overall: %s" % ("ALL PASS" if all_pass else "SOME FAILED"))
    return all_pass


if __name__ == "__main__":
    sys.exit(0 if test() else 1)
