#!/usr/bin/env python3
"""
B9: Random Packet Loss (Non-Congestion, BER > 0)
Guarantee: Loss -> fast retransmit -> cwnd reduction -> queue drain ->
clean samples -> G1 convergence. Loss creates estimation OPPORTUNITIES.
Geodesic is loss-agnostic (processes RTT, not loss signal).
No BDP inflation from loss events.
"""

import os
import random
import sys

sys.path.insert(0, os.path.dirname(__file__))
from common import GeodesicEstimator


def mean(vals):
    return sum(vals) / float(len(vals)) if vals else 0.0


def run_trial(t_prop_us, loss_rate, noise_sigma_us, n_samples, seed):
    random.seed(seed)
    est = GeodesicEstimator(t_prop_us)
    queue = 0
    recovery = 0
    for _i in range(n_samples):
        noise = random.gauss(0, noise_sigma_us)
        loss_event = random.random() < loss_rate

        if loss_event:
            queue = max(0, queue - 2000)
            recovery = 10 + random.randint(0, 10)
        elif recovery > 0:
            recovery -= 1
        else:
            # Bounded small self-queue (TSO bursts at 1Gbps: ~12us per 1500B)
            queue += random.gauss(0, 15)
            queue = max(0, min(queue, 500))  # max ~500us self-queue

        z = t_prop_us + queue + noise
        est.update(max(1, int(z)))
    return est


def test():
    print("=" * 72)
    print("B9: Random Packet Loss (Non-Congestion, BER > 0)")
    print("  Guarantee: No BDP inflation. Loss creates estimation opportunities.")
    print("  Geodesic is loss-agnostic (processes RTT, not loss).")
    print("=" * 72)

    T_PROP = 25000  # 25ms
    cases = [
        ("BER=0.01%% ", 0.0001, 300),
        ("BER=0.1%%  ", 0.001, 300),
        ("BER=1%%    ", 0.01, 300),
        ("BER=5%%    ", 0.05, 300),
        ("BER=10%%   ", 0.10, 300),
    ]

    N_SEEDS = 30
    N_SAMPLES = 2000

    all_pass = True
    for label, loss_rate, noise_sigma in cases:
        final_bdp_ratios = []
        max_bdp_ratios = []
        xest_errors = []
        within_pct = []
        g3_total = 0

        for seed in range(N_SEEDS):
            est = run_trial(T_PROP, loss_rate, noise_sigma, N_SAMPLES, seed)
            final = est.history[-1]
            final_bdp_ratios.append(final["bdp_us"] / float(T_PROP))
            max_bdp = max(h["bdp_us"] for h in est.history)
            max_bdp_ratios.append(max_bdp / float(T_PROP))
            xest_errors.append(abs(final["x_est_us"] - T_PROP) / float(T_PROP) * 100)
            close = sum(
                1
                for h in est.history
                if abs(h["x_est_us"] - T_PROP) / float(T_PROP) * 100 < 5
            )
            within_pct.append(close / float(len(est.history)) * 100)
            g3_total += est.g3_events

        mean_bdp = mean(final_bdp_ratios)
        max_bdp = max(max_bdp_ratios)
        mean_err = mean(xest_errors)
        max_err = max(xest_errors)
        mean_close = mean(within_pct)

        bdp_ok = max_bdp <= 1.06
        passed = bdp_ok
        status = "PASS" if passed else "FAIL"
        if not passed:
            all_pass = False

        print()
        print(f"  {label} (loss_rate={loss_rate * 100:.3f}%)")
        print("    Seeds: %d, Samples/seed: %d" % (N_SEEDS, N_SAMPLES))
        print(
            "    Mean final BDP/T_prop: {:.4f}  (worst: {:.4f})  {}".format(
                mean_bdp,
                max_bdp,
                "OK" if bdp_ok else "INFLATED",
            ),
        )
        print(f"    Mean x_est error: {mean_err:.2f}%  (worst: {max_err:.2f}%)")
        print(f"    Samples within 5% of T_prop: {mean_close:.1f}%")
        print("    G3 events (total): %d" % g3_total)
        print(f"    [{status}]")

    print()
    print("  Overall: %s" % ("ALL PASS" if all_pass else "SOME FAILED"))
    return all_pass


if __name__ == "__main__":
    sys.exit(0 if test() else 1)
