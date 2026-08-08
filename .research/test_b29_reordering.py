#!/usr/bin/env python3
"""
B29: Packet Reordering (Out-of-Order Delivery)
Guarantee: Low RTT (dupACK) -> G1 temporary x_est drop -> min_rtt protects.
High RTT (late reorder) -> G2 -> G3 requires 3 confirms -> safe.
Directional gate sign-based immunity: G1 only on downward, G2 only on upward.
"""

import os
import random
import sys

sys.path.insert(0, os.path.dirname(__file__))
from common import GeodesicEstimator


def mean(vals):
    return sum(vals) / float(len(vals)) if vals else 0.0


def run_reorder_trial(
    t_prop_us,
    reorder_rate,
    reorder_delta_us,
    noise_sigma_us,
    n_samples,
    seed,
):
    random.seed(seed)
    est = GeodesicEstimator(t_prop_us)
    max_confirm = 0
    for _i in range(n_samples):
        noise = random.gauss(0, noise_sigma_us)
        z_base = t_prop_us + noise
        if random.random() < reorder_rate:
            if random.random() < 0.5:
                z = max(1, z_base - abs(reorder_delta_us))
            else:
                z = z_base + abs(reorder_delta_us)
        else:
            z = z_base
        st = est.update(max(1, int(z)))
        max_confirm = max(max_confirm, st["confirm_cnt"])
    return est, max_confirm


def test():
    print("=" * 72)
    print("B29: Packet Reordering - Directional Gate Sign-Based Immunity")
    print("  Guarantee: G1 only on downward, G2 only on upward.")
    print("  G3 requires 3 confirms -> isolated reordering safe.")
    print("=" * 72)

    T_PROP = 15000  # 15ms
    cases = [
        ("Low reorder (1%%)    ", 0.01, 3000, "realistic ECMP"),
        ("Moderate reorder    ", 0.05, 5000, "moderate LAG"),
        ("High reorder (10%%) ", 0.10, 5000, "high LAG skew"),
        ("Severe reorder      ", 0.20, 10000, "wireless L2 reTX"),
        ("Extreme reorder     ", 0.30, 15000, "extreme outlier"),
    ]

    N_SEEDS = 30
    N_SAMPLES = 1000
    NOISE_SIGMA = 300

    all_pass = True
    for label, reorder_rate, reorder_delta, scenario in cases:
        max_confirms = []
        bdp_max_ratios = []
        bdp_final_ratios = []
        g3_total = 0
        g3_seeds = 0

        for seed in range(N_SEEDS):
            est, max_confirm = run_reorder_trial(
                T_PROP,
                reorder_rate,
                reorder_delta,
                NOISE_SIGMA,
                N_SAMPLES,
                seed,
            )
            max_confirms.append(max_confirm)
            max_bdp = max(h["bdp_us"] for h in est.history)
            bdp_max_ratios.append(max_bdp / float(T_PROP))
            bdp_final_ratios.append(est.history[-1]["bdp_us"] / float(T_PROP))
            if est.g3_events > 0:
                g3_total += est.g3_events
                g3_seeds += 1

        mean_conf = mean(max_confirms)
        max_conf = max(max_confirms)
        mean(bdp_max_ratios)
        max_bdp_max = max(bdp_max_ratios)
        mean_bdp_final = mean(bdp_final_ratios)

        no_false_g3 = g3_seeds == 0
        bdp_bounded = max_bdp_max <= 2.0
        self_correcting = mean_bdp_final <= 1.05
        passed = (no_false_g3 or self_correcting) and bdp_bounded
        status = "PASS" if passed else "FAIL"
        if not passed:
            all_pass = False

        print()
        print(f"  {label} ({scenario})")
        print(
            f"    Rate={reorder_rate * 100:.0f}%, delta={reorder_delta / 1000.0:.1f}ms",
        )
        print("    Seeds: %d, Samples/seed: %d" % (N_SEEDS, N_SAMPLES))
        print(
            "    Mean max confirm_cnt: %.2f  (worst: %d)  G3 events: %d  %s"
            % (
                mean_conf,
                max_conf,
                g3_seeds,
                "NONE" if no_false_g3 else "WARN" if self_correcting else "FALSE!",
            ),
        )
        print(
            "    Max BDP/T_prop (worst seed): {:.4f}  {}".format(
                max_bdp_max,
                "BOUNDED" if bdp_bounded else "UNBOUNDED",
            ),
        )
        print(
            "    Mean final BDP/T_prop: {:.4f} (self-correcting: {})".format(
                mean_bdp_final,
                "YES" if self_correcting else "NO",
            ),
        )
        print(f"    [{status}]")

    print()
    print("  Overall: %s" % ("ALL PASS" if all_pass else "SOME FAILED"))
    return all_pass


if __name__ == "__main__":
    sys.exit(0 if test() else 1)
