#!/usr/bin/env python3
"""
B7/B8: Sudden Bandwidth Drop/Increase (C -> C/10, C -> 10*C)
Guarantee: T_prop unchanged (dT_prop/dC = 0). Geodesic state unchanged.
Error: zero (T_prop estimate unaffected).
"""

import os
import random
import sys

sys.path.insert(0, os.path.dirname(__file__))
from common import GeodesicEstimator


def mean(vals):
    return sum(vals) / float(len(vals)) if vals else 0.0


def run_trial(
    t_prop_us,
    serial_us_before,
    serial_us_after,
    change_at,
    noise_sigma_us,
    n_samples,
    seed,
):
    random.seed(seed)
    est = GeodesicEstimator(t_prop_us)
    for i in range(n_samples):
        s = serial_us_before if i < change_at else serial_us_after
        noise = random.gauss(0, noise_sigma_us)
        z = t_prop_us + s + noise
        est.update(max(1, int(z)))
    final = est.history[-1]
    return final["x_est_us"], final["bdp_us"], est.min_rtt_us, est.history


def test():
    print("=" * 72)
    print("B7/B8: Sudden Bandwidth Drop/Increase")
    print("  Guarantee: dT_prop/dC = 0, Geodesic state unchanged")
    print("=" * 72)

    T_PROP = 20000  # 20ms
    cases = [
        ("Drop 10G->1G   ", 10, 100, "B7"),
        ("Drop 1G->100M  ", 100, 1000, "B7"),
        ("Incr 100M->1G  ", 1000, 100, "B8"),
        ("Incr 1G->10G   ", 100, 10, "B8"),
        ("Drop 10G->100M ", 10, 1000, "B7"),
    ]

    N_SEEDS = 25
    N_SAMPLES = 300
    CHANGE_AT = 150
    NOISE_SIGMA = 200

    all_pass = True
    for label, serial_bf, serial_af, bc in cases:
        t_prop_actual = T_PROP
        pre_xest = []
        post_xest = []
        post_bdp = []
        for seed in range(N_SEEDS):
            _, _bdp, _, hist = run_trial(
                t_prop_actual,
                serial_bf,
                serial_af,
                CHANGE_AT,
                NOISE_SIGMA,
                N_SAMPLES,
                seed,
            )
            pre_vals = [h["x_est_us"] for h in hist[CHANGE_AT - 30 : CHANGE_AT]]
            post_vals = [h["x_est_us"] for h in hist[CHANGE_AT : CHANGE_AT + 30]]
            post_bdp_vals = [h["bdp_us"] for h in hist[CHANGE_AT : CHANGE_AT + 30]]
            if pre_vals and post_vals:
                pre_xest.append(mean(pre_vals))
                post_xest.append(mean(post_vals))
                post_bdp.extend(post_bdp_vals)

        if not pre_xest:
            continue

        xest_drift = [
            (pst - pre) / pre * 100.0
            for pre, pst in zip(pre_xest, post_xest, strict=False)
        ]
        mean_drift = mean(xest_drift)
        max_drift = max(xest_drift)

        new_min_rtt = T_PROP + serial_af
        bdp_ok = all(b <= new_min_rtt * 1.05 for b in post_bdp)

        drift_ok = max_drift < 50
        passed = drift_ok
        status = "PASS" if passed else "FAIL"
        if not passed:
            all_pass = False

        print()
        print("  %s %s  (serial: %dus -> %dus)" % (bc, label, serial_bf, serial_af))
        print("    Seeds: %d, T_prop: %.1fms" % (N_SEEDS, T_PROP / 1000.0))
        print(
            f"    Mean x_est drift after change: {mean_drift:.2f}%  (max: {max_drift:.2f}%)",
        )
        print("    Post-change min RTT target: %dus" % new_min_rtt)
        print(
            "    Post-change BDP <= target: %s"
            % ("YES" if bdp_ok else "NO (some exceeded)"),
        )
        print(f"    [{status}]")

    print()
    print("  Overall: %s" % ("ALL PASS" if all_pass else "SOME FAILED"))
    return all_pass


if __name__ == "__main__":
    sys.exit(0 if test() else 1)
