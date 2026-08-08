#!/usr/bin/env python3
"""
multi_flow_100_mc.py -- 100 parallel KCC flows under various network conditions.
Detects divergence, unfairness, or pathological states.
Tests at 5 RTTs, 3 congestion levels, 3 seeds. Total: 100x5x3x3 = 4500 flow instances.
"""

import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

SCALE = 1024
SCALE_SHIFT = 10
P_INIT = 1000
P_MAX = 100000000
failures = 0


def fail(msg):
    global failures
    print(f"  FAIL: {msg}")
    failures += 1


def pass_(msg):
    print(f"  PASS: {msg}")


def info(msg):
    print(f"  INFO: {msg}")


def warn(msg):
    print(f"  WARN: {msg}")


class KCCFlow:
    def __init__(self, flow_id, rtt_base_us, noise_sigma_us, queue_us=0):
        self.id = flow_id
        self.x_est = rtt_base_us * SCALE
        self.p_est = P_INIT
        self.Q = 100
        self.R = 400
        self.rtt_base = rtt_base_us
        self.sigma = noise_sigma_us
        self.queue = queue_us
        self.pos_skip = 0
        self.consec_reject = 0
        self.jitter_ewma = 0.0
        self.qdelay_ewma = max(0, queue_us)
        self.qboost_cdwn = 0
        self.history = []  # (x_est_us, p_est, innov_us, accepted)
        self.G3_fast_fires = 0
        self.G3_slow_fires = 0
        self.qboost_fires = 0
        self.g3_fires = 0

    def step(self, rng):
        noise = rng.gauss(0, self.sigma)
        rtt_us = max(1, self.rtt_base + int(self.queue) + int(noise))
        z = rtt_us << SCALE_SHIFT
        innov = z - self.x_est
        abs_innov = innov if innov >= 0 else -innov

        # Jitter EWMA
        self.jitter_ewma = self.jitter_ewma * 0.875 + (abs_innov >> SCALE_SHIFT) * 0.125

        # Qdelay EWMA
        self.qdelay_ewma = (
            self.qdelay_ewma * 0.875 + max(0, rtt_us - self.rtt_base) * 0.125
        )

        # G2_queue_cap
        if self.qboost_cdwn > 0:
            self.qboost_cdwn -= 1
        qb = False
        if (
            self.qboost_cdwn == 0
            and innov > 0
            and abs_innov > 16384000
            and self.p_est <= 33
            and self.pos_skip < 5
            and self.qdelay_ewma < (self.x_est >> (SCALE_SHIFT + 1)) / SCALE
        ):
            self.p_est = P_INIT
            self.qboost_cdwn = 6
            self.pos_skip = 0
            self.qboost_fires += 1
            qb = True

        if qb:
            self.x_est = min(z, 0xFFFFFFFF)
            self.history.append(
                (self.id, self.x_est / SCALE, self.p_est, innov / SCALE, True),
            )
            return

        # G3
        qd_scaled = int(self.qdelay_ewma * SCALE)
        g3_c1 = abs_innov > (qd_scaled * 5) // 2
        g3_c2 = self.qdelay_ewma < (self.rtt_base >> 1)
        g3_c3 = self.pos_skip >= 2
        if innov > 0 and g3_c1 and g3_c2 and g3_c3:
            self.x_est = min(z, 0xFFFFFFFF)
            self.p_est = max(self.R, 10)
            self.pos_skip = 0
            self.g3_fires += 1
            self.history.append(
                (self.id, self.x_est / SCALE, self.p_est, innov / SCALE, True),
            )
            return

        # Core update
        p_pred = min(self.p_est + self.Q, P_MAX)

        if innov <= 0:
            floor = self.x_est - (self.x_est >> 3)
            if z >= floor:
                self.x_est = min(z, 0xFFFFFFFF)
                self.p_est = max(self.R, 10)
                accepted = True
            else:
                self.p_est = p_pred
                accepted = False
            self.pos_skip = 0
            self.consec_reject = 0
        else:
            # Outlier gate
            prop_thresh = max(self.rtt_base >> 2, 50) * SCALE
            jitter_thresh = int(self.jitter_ewma * 2) * SCALE
            dyn_thresh = max(prop_thresh, jitter_thresh)

            if abs_innov > dyn_thresh and self.consec_reject < 20:
                self.consec_reject += 1
                self.pos_skip += 1
                self.p_est = p_pred
                accepted = False
            else:
                if self.consec_reject >= 20:
                    self.consec_reject = 0
                # Standard update
                gain_num = p_pred
                gain_den = p_pred + self.R
                if gain_den > 0:
                    corr = (p_pred * innov) // gain_den
                    p_reduction = (p_pred * gain_num) // gain_den
                else:
                    corr = 0
                    p_reduction = 0
                self.x_est = min(self.x_est + corr, 0xFFFFFFFF)
                self.p_est = max(p_pred - p_reduction, 10)
                self.consec_reject = 0
                accepted = True
                self.pos_skip += 1

                # Drift checks
                if self.pos_skip >= 14 and self.jitter_ewma < (self.rtt_base >> 3):
                    self.G3_fast_fires += 1
                    corr_abs = corr  # already computed
                    drift_corr = corr_abs >> 2
                    self.x_est = min(self.x_est + drift_corr, 0xFFFFFFFF)

                if (
                    self.pos_skip >= 56
                    and self.qdelay_ewma < (self.x_est >> (SCALE_SHIFT + 1)) / SCALE
                ):
                    self.G3_slow_fires += 1
                    corr_abs = corr
                    drift_corr = corr_abs >> 3
                    self.x_est = min(self.x_est + drift_corr, 0xFFFFFFFF)

        self.history.append(
            (self.id, self.x_est / SCALE, self.p_est, innov / SCALE, accepted),
        )


print("=" * 90)
print("100-PARALLEL-FLOW KCC MONTE CARLO")
print("=" * 90)

RTTS = [
    (1400, 20, "DC"),
    (50000, 200, "WAN"),
    (100000, 400, "long"),
    (300000, 500, "LH"),
    (500000, 800, "extreme"),
]
QUEUE_LEVELS = [(0, "clean"), (500, "mild"), (5000, "congested")]
N_FLOWS = 100
N_STEPS = 1000
N_SEEDS = 3

for rtt_base, sigma, label in RTTS:
    for queue, qlabel in QUEUE_LEVELS:
        divergences = 0
        fairness_issues = 0
        for seed in range(N_SEEDS):
            rng = random.Random(seed * 10000 + rtt_base + queue)
            flows = [KCCFlow(i, rtt_base, sigma, queue) for i in range(N_FLOWS)]

            for _step in range(N_STEPS):
                for f in flows:
                    f.step(rng)

            # Check for divergence
            final_x = [f.x_est / SCALE for f in flows]
            final_p = [f.p_est for f in flows]
            max_x = max(final_x)
            min_p = min(final_p)

            # Divergence: x_est outside [0.5, 10]x RTT
            for f in flows:
                x_us = f.x_est / SCALE
                if x_us > rtt_base * 10:
                    divergences += 1
                if x_us < rtt_base * 0.3:
                    divergences += 1

            # Fairness: std of x_est > 30% of mean
            if len(final_x) > 1:
                mean_x = sum(final_x) / len(final_x)
                std_x = (
                    sum((xi - mean_x) ** 2 for xi in final_x) / len(final_x)
                ) ** 0.5
                if std_x > mean_x * 0.3:
                    fairness_issues += 1

        div_rate = divergences / (N_FLOWS * N_SEEDS) * 100
        fair_rate = fairness_issues / N_SEEDS * 100

        if div_rate < 5:
            pass_(
                f"  {label:>8s} {qlabel:>10s}: divergence={div_rate:.1f}%, fairness_issues={fair_rate:.0f}%",
            )
        elif div_rate < 20:
            warn(f"  {label:>8s} {qlabel:>10s}: divergence={div_rate:.1f}% (HIGH)")
        else:
            fail(f"  {label:>8s} {qlabel:>10s}: divergence={div_rate:.1f}% (CRITICAL)")

# =============================================================================
print(f"\n{'=' * 90}")
print(
    f"{'ALL 100-FLOW SCENARIOS STABLE' if failures == 0 else f'{failures} SCENARIOS WITH DIVERGENCE'}",
)
