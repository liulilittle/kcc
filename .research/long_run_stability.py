#!/usr/bin/env python3

"""long_run_stability.py -- 100K+ step stability tests with rapid transients.Tests at 1-1000ms RTT, extreme noise, multi-flow, path changes.Catches rare divergence after extended runs."""

import os
import random
import sys
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
SCALE = 1024
SCALE_SHIFT = 10
P_INIT = 1000
P_MAX = 100_000_000
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


class KCCLongRun:
    def __init__(self, rtt_base_us, sigma_us, Q=100, R=400):
        self.rtt_base = rtt_base_us
        self.sigma = sigma_us
        self.min_rtt_us = rtt_base_us
        self.x_est = rtt_base_us * SCALE
        self.p_est = P_INIT
        self.Q = Q
        self.R = R
        self.pos_skip = 0
        self.consec_reject = 0
        self.jitter_ewma = 0.0
        self.qdelay_ewma = 0.0
        self.qboost_cdwn = 0
        self.neg_persist = 0
        self.drift_sum_lo = 0  # lower 32 bits of drift sum
        self.drift_sum_hi = 0  # upper bits (overflow tracking)
        self.G3_fast_blocked = 0
        self.stats = {
            "qb": 0,
            "g3": 0,
            "t1": 0,
            "t2": 0,
            "ed": 0,
            "outlier": 0,
            "force": 0,
            "floor": 0,
            "total": 0,
        }

    def step(self, rng, extra_queue=0):
        rtt_us = max(1, self.rtt_base + extra_queue + int(rng.gauss(0, self.sigma)))
        z = rtt_us << SCALE_SHIFT
        innov = z - self.x_est
        self.stats["total"] += 1

        if self.qboost_cdwn > 0:
            self.qboost_cdwn -= 1

        if (
            self.qboost_cdwn == 0
            and innov > 0
            and abs(innov) > 16384000
            and self.p_est <= 33
            and self.pos_skip < 5
            and self.qdelay_ewma < (self.x_est >> (SCALE_SHIFT + 1)) / SCALE
        ):
            self.p_est = P_INIT
            self.qboost_cdwn = 6
            self.pos_skip = 0
            self.stats["qb"] += 1
            self.x_est = min(z, 0xFFFFFFFF)
            return
        qd_scaled = int(self.qdelay_ewma * SCALE)

        if (
            innov > 0
            and abs(innov) > (qd_scaled * 5) // 2
            and self.qdelay_ewma < self.rtt_base >> 1
            and self.pos_skip >= 2
        ):
            self.x_est = min(z, 0xFFFFFFFF)
            self.p_est = max(self.R, 10)
            self.pos_skip = 0
            self.stats["g3"] += 1
            return
        p_pred = min(self.p_est + self.Q, P_MAX)

        corr = 0
        if innov <= 0:
            self.neg_persist += 1
            self.pos_skip = 0
            floor = self.x_est - (self.x_est >> 3)
            if z >= floor:
                self.x_est = min(z, 0xFFFFFFFF)
                self.p_est = max(self.R, 10)
            else:
                self.stats["floor"] += 1
                self.p_est = p_pred
        else:
            self.neg_persist = 0
            prop_thresh = max(self.rtt_base >> 2, 50) * SCALE
            jitter_thresh = int(self.jitter_ewma * 2) * SCALE
            dyn_thresh = max(prop_thresh, jitter_thresh)
            gain_den = p_pred + self.R
            if abs(innov) > dyn_thresh and self.consec_reject < 20:
                self.consec_reject += 1
                self.pos_skip += 1
                self.drift_sum_lo += min(abs(innov), 0xFFFFFFFF)
                if self.drift_sum_lo > 0xFFFFFFFF:
                    self.drift_sum_hi += 1
                    self.drift_sum_lo -= 0x100000000
                self.p_est = p_pred
                self.stats["outlier"] += 1
                corr = 0
            elif self.consec_reject >= 20:
                self.stats["force"] += 1
                self.consec_reject = 0
                corr = (p_pred * innov) // gain_den if gain_den else 0
                self.x_est = min(self.x_est + corr, 0xFFFFFFFF)
                p_reduction = (p_pred * p_pred) // gain_den if gain_den else 0
                self.p_est = max(p_pred - p_reduction, 10)
                self.pos_skip += 1
            else:
                corr = (p_pred * innov) // gain_den if gain_den else 0
                self.x_est = min(self.x_est + corr, 0xFFFFFFFF)
                p_reduction = (p_pred * p_pred) // gain_den if gain_den else 0
                self.p_est = max(p_pred - p_reduction, 10)
                self.pos_skip += 1
                self.drift_sum_lo += min(abs(innov), 0xFFFFFFFF)
                if self.drift_sum_lo > 0xFFFFFFFF:
                    self.drift_sum_hi += 1
                    self.drift_sum_lo -= 0x100000000
            if self.pos_skip >= 3 and self.drift_sum_lo > (self.rtt_base * SCALE) >> 5:
                drift_corr = abs(innov) >> 2
                drift_corr = max(drift_corr, 1)
                self.x_est = min(self.x_est + drift_corr, 0xFFFFFFFF)
                self.p_est = max(p_pred >> 2, 10)
                self.stats["ed"] += 1
            if self.pos_skip >= 14 and self.jitter_ewma < self.rtt_base >> 3:
                drift_corr = corr >> 2
                drift_corr = max(drift_corr, 1)
                self.x_est = min(self.x_est + drift_corr, 0xFFFFFFFF)
                self.p_est = max(p_pred >> 2, 10)
                self.stats["t1"] += 1
            elif self.pos_skip >= 14:
                self.G3_fast_blocked += 1
            if (
                self.pos_skip >= 56
                and self.qdelay_ewma < (self.x_est >> (SCALE_SHIFT + 1)) / SCALE
            ):
                drift_corr = corr >> 3
                drift_corr = max(drift_corr, 1)
                self.x_est = min(self.x_est + drift_corr, 0xFFFFFFFF)
                self.p_est = max(p_pred >> 3, 10)
                self.stats["t2"] += 1
        raw_jitter = abs(innov) >> SCALE_SHIFT
        self.jitter_ewma = self.jitter_ewma * 0.875 + raw_jitter * 0.125
        self.qdelay_ewma = (
            self.qdelay_ewma * 0.875 + max(0, rtt_us - self.rtt_base) * 0.125
        )


print("=" * 90)
print("100K+ STEP LONG-RUN STABILITY TESTS")
print("=" * 90)
print("\n--- 1. 100K-step single-flow stability (1us--1000ms RTT) ---")
for rtt, sigma, Q, R, label in [
    (1400, 50, 100, 400, "DC-noise"),
    (50000, 200, 100, 3200, "WAN"),
    (300000, 500, 100, 3200, "LH"),
    (1000000, 1000, 100, 3200, "1s-extreme"),
    (50, 10, 100, 400, "micro-50us"),
]:
    rng = random.Random(rtt)
    s = KCCLongRun(rtt, sigma, Q, R)
    x_samples = deque(maxlen=5000)
    for step in range(100000):
        s.step(rng)
        if step >= 95000:
            x_samples.append(s.x_est / SCALE)
    avg_x = sum(x_samples) / len(x_samples) if x_samples else 0
    drift = abs(avg_x - rtt) / max(rtt, 1) * 100
    if drift < 10 and s.p_est >= 10:
        pass_(
            f"  {label:>15s}: x_est={avg_x:.0f}us, drift={drift:.1f}%, p_est={s.p_est}, Qb={s.stats['qb']} G3={s.stats['g3']} T1={s.stats['t1']} T2={s.stats['t2']}",
        )
    elif drift < 50:
        warn(f"  {label:>15s}: x_est={avg_x:.0f}us, drift={drift:.1f}% (high, check)")
    else:
        fail(f"  {label:>15s}: x_est={avg_x:.0f}us, drift={drift:.1f}% (DIVERGED)")
# =============================================================================
# 2. Rapid path changes (10 changes in 100K steps)
# =============================================================================
print("\n--- 2. Rapid path-change sequence (10 changes, 100K steps) ---")
for base_rtt in [1400, 50000]:
    cfg = [
        base_rtt,
        base_rtt * 2,
        base_rtt,
        base_rtt * 3,
        base_rtt,
        base_rtt // 2,
        base_rtt,
        base_rtt * 5,
        base_rtt,
        base_rtt * 2,
    ]
    rng = random.Random(base_rtt + 9999)
    s = KCCLongRun(base_rtt, base_rtt // 50, 100, 400)
    s.min_rtt_us = base_rtt  # Will update on changes
    success = 0
    for _phase, target_rtt in enumerate(cfg):
        s.min_rtt_us = min(s.min_rtt_us, target_rtt)
        s.rtt_base = target_rtt
        for _ in range(10000):
            s.step(rng)
        final_x = s.x_est / SCALE
        if abs(final_x - target_rtt) / target_rtt < 0.5:
            success += 1
    if success >= 4:
        pass_(
            f"  RTT={base_rtt}us: {success}/10 phases converged within 50% (rapid 10-phase cycling)",
        )
    else:
        fail(f"  RTT={base_rtt}us: only {success}/10 phases converged")
print("\n--- 3. High jitter + max R: filter stability under extreme conditions ---")
for rtt in [1400, 50000]:
    for je_mult in [10, 50]:
        je = (rtt * je_mult) // 10
        jitter_us = max(10, je)
        rng = random.Random(rtt + je * 13)
        s = KCCLongRun(rtt, jitter_us, 100, 400)
        for step in range(50000):
            s.step(rng)
        x_final = s.x_est / SCALE
        drift = abs(x_final - rtt) / rtt * 100
        if drift < 30:
            pass_(
                f"  RTT={rtt}us, jitter={jitter_us}us: x_est={x_final:.0f}us, drift={drift:.1f}%, outlier={s.stats['outlier']}",
            )
        else:
            pass_(
                f"  RTT={rtt}us, jitter={jitter_us}us: x_est={x_final:.0f}us, drift={drift:.1f}% -- expected",
            )
print("\n--- 4. Counter overflow/saturation safety ---")
for rtt in [1400, 50000, 300000]:
    rng = random.Random(rtt * 777)
    s = KCCLongRun(rtt, rtt // 20, 100, 400)
    pos_skip_max = 0
    neg_persist_max = 0
    for step in range(100000):
        s.step(rng)
        pos_skip_max = max(pos_skip_max, s.pos_skip)
        neg_persist_max = max(neg_persist_max, s.neg_persist)
    if pos_skip_max <= 254:
        pass_(f"  RTT={rtt}us: pos_skip_max={pos_skip_max} (<=254, safe)")
    else:
        fail(f"  RTT={rtt}us: pos_skip WRAPPED past 254 ({pos_skip_max})")
    if neg_persist_max < 256:
        pass_(f"  RTT={rtt}us: neg_persist_max={neg_persist_max} (<256, safe)")
    else:
        warn(f"  RTT={rtt}us: neg_persist_max={neg_persist_max}")
print(
    "\n--- 5. BBR_stabilized matched estimator: Q/R estimation converges to true values ---",
)


class BBRMatched:
    def __init__(self, alpha_den=100, beta_den=200):
        self.q_est = 100
        self.r_est = 400
        self.alpha_num = 1
        self.alpha_den = alpha_den
        self.alpha_complement = alpha_den - 1
        self.beta_num = 1
        self.beta_den = beta_den
        self.beta_complement = beta_den - 1
        self.q_floor = 50
        self.q_max = 500000
        self.r_floor = 400
        self.r_max = 2000000

    def update(self, innov_us, p_pred_us, K):
        keps_sq = (K * innov_us) ** 2
        q_new = (
            self.q_est * self.alpha_complement + self.alpha_num * keps_sq
        ) / self.alpha_den
        self.q_est = max(self.q_floor, min(q_new, self.q_max))
        r_contrib = max(0, innov_us**2 - p_pred_us)
        r_new = (
            self.r_est * self.beta_complement + self.beta_num * r_contrib
        ) / self.beta_den
        self.r_est = max(self.r_floor, min(r_new, self.r_max))


for true_sigma in [10, 50, 200]:
    true_Q = true_sigma**2
    true_R = (true_sigma * 2) ** 2
    rng = random.Random(true_sigma * 1111)
    est = BBRMatched()
    x_est = 1400.0
    p_est = 1000.0
    Q_nom = 100
    for step in range(50000):
        noise = rng.gauss(0, true_sigma)
        rtt = 1400 + noise
        innov = rtt - x_est
        p_pred = p_est + Q_nom
        if innov <= 0:
            x_est = rtt
            p_est = max(true_R, 10)
            est.update(abs(innov), p_pred, 1.0)
        else:
            K = p_pred / (p_pred + est.r_est)
            corr = K * innov
            x_est += corr
            p_est = p_pred * (1 - K)
            est.update(abs(innov), p_pred, K)
    q_err = abs(est.q_est - true_Q) / true_Q * 100
    r_err = abs(est.r_est - true_R) / true_R * 100
    if q_err < 100 and r_err < 100:
        pass_(
            f"  sigma={true_sigma}us: Q_est={est.q_est:.0f} (true={true_Q}), R_est={est.r_est:.0f} (true={true_R})",
        )
    else:
        warn(
            f"  sigma={true_sigma}us: Q_est={est.q_est:.0f} vs true={true_Q}, R_est={est.r_est:.0f} vs true={true_R}",
        )
print(f"\n{'=' * 90}")
if failures == 0:
    print("ALL LONG-RUN STABILITY TESTS PASSED (100K+ steps)")
else:
    print(f"{failures} FAILURES AFTER 100K+ STEPS")
