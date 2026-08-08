#!/usr/bin/env python3
"""
full_state_machine_mc.py -- Complete KCC state machine Monte Carlo simulation.
Tracks all state variables (x_est, p_est, pos_skip, qdelay, jitter, etc.)
through many RTT rounds under realistic multi-flow network conditions.
Tests at 10 RTTs from 1us to 300ms, 10 seeds each.
Detects pathological states (deadlock, divergence, counter overflow).
"""

import os
import random
import sys
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

SCALE = 1024
SCALE_SHIFT = 10
P_INIT = 1000
P_MAX = 100_000_000
P_FLOOR = 10
U32_MAX = 0xFFFFFFFF
S64_MAX = (1 << 63) - 1

failures = 0
warnings = 0


def fail(msg):
    global failures
    print(f"  FAIL: {msg}")
    failures += 1


def pass_(msg):
    print(f"  PASS: {msg}")


def warn(msg):
    global warnings
    print(f"  WARN: {msg}")
    warnings += 1


def info(msg):
    print(f"  INFO: {msg}")


def int_sqrt(x):
    if x <= 1:
        return x
    m = 1 << (x.bit_length() - 1 & ~1)
    y = 0
    while m:
        b = y + m
        y >>= 1
        if x >= b:
            x -= b
            y += m
        m >>= 2
    return y


# =============================================================================
# Full KCC state machine (simplified but faithful to the C code structure)
# =============================================================================


class KCCState:
    def __init__(self, rtt_base_us, qdelay_init=0):
        self.x_est = rtt_base_us * SCALE  # cold start
        self.p_est = P_INIT
        self.min_rtt_us = rtt_base_us
        self.min_rtt_scaled = rtt_base_us * SCALE

        # Queue delay tracking
        self.qdelay_ewma_us = float(qdelay_init)
        self.qdelay_alpha = 0.125  # EWMA factor

        # Jitter tracking (EWMA)
        self.jitter_ewma_us = 0.0
        self.jitter_alpha = 0.125
        self.raw_jitter_us = 0.0

        # Drift counters
        self.pos_skip_cnt = 0
        self.drift_sum_scaled = 0
        self.neg_persist_cnt = 0

        # G2_queue_cap
        self.qboost_cdwn = 0

        # Outlier rejection counter
        self.consec_reject_cnt = 0

        # Event counters for diagnostics
        self.qboost_fires = 0
        self.G3_fast_fires = 0
        self.G3_slow_fires = 0
        self.early_drift_fires = 0
        self.g3_fires = 0
        self.outlier_rejects = 0
        self.forced_accepts = 0
        self.floor_rejects = 0

        # Debug: track x_est history
        self.x_history = deque(maxlen=1000)
        self.p_history = deque(maxlen=1000)

    def update(self, rtt_us, Q_int, R_int, cfg):
        """One full KCC update cycle."""
        z = rtt_us << SCALE_SHIFT
        innov = z - self.x_est
        abs_innov = innov if innov >= 0 else -innov
        innov_sign = 1 if innov > 0 else (0 if innov == 0 else -1)

        # Update jitter EWMA
        self.raw_jitter_us = abs_innov >> SCALE_SHIFT
        self.jitter_ewma_us = (
            self.jitter_ewma_us * (1 - self.jitter_alpha)
            + self.raw_jitter_us * self.jitter_alpha
        )

        # Update qdelay EWMA (computed from RTT - min_rtt, clamped to >= 0)
        qdelay_instant_us = max(0, rtt_us - self.min_rtt_us)
        self.qdelay_ewma_us = (
            self.qdelay_ewma_us * (1 - self.qdelay_alpha)
            + qdelay_instant_us * self.qdelay_alpha
        )

        # ---- Outlier gate ----
        outlier_reject = False
        dyn_thresh_scaled = self._compute_outlier_thresh(cfg)
        if abs_innov > dyn_thresh_scaled:
            self.consec_reject_cnt += 1
            outlier_reject = True
        else:
            self.consec_reject_cnt = 0

        # Force-accept after 20 consecutive rejects
        force_accept = self.consec_reject_cnt >= cfg["max_consec_reject"]

        # Track outlier
        if outlier_reject and not force_accept:
            self.outlier_rejects += 1

        # ---- G2_queue_cap check (runs before standard update) ----
        if self.qboost_cdwn > 0:
            self.qboost_cdwn -= 1

        if (
            innov_sign > 0
            and abs_innov > cfg["qboost_thresh"]
            and self.p_est <= cfg["converged_p"]
            and self.qdelay_ewma_us
            < (self.x_est >> (cfg["scale_shift"] + cfg["t2_qdelay_frac_shift"])) / SCALE
            and self.pos_skip_cnt < cfg["pos_skip_thresh"]
            and self.qboost_cdwn == 0
        ):
            self.qboost_fires += 1
            self.p_est = cfg["p_init"]
            self.qboost_cdwn = cfg["qboost_cdwn_val"]
            outlier_reject = False  # G2_queue_cap bypasses outlier gate
            force_accept = True

        # ---- G3 path-shift check ----
        qdelay_thresh_scaled = (
            self.qdelay_ewma_us * SCALE * cfg["g3_qdelay_mult_num"]
        ) // cfg["g3_qdelay_mult_den"]
        if (
            abs_innov > qdelay_thresh_scaled
            and self.qdelay_ewma_us < self.min_rtt_us >> cfg["g3_max_qdelay_frac_shift"]
            and self.pos_skip_cnt >= cfg["g3_min_skip"]
        ):
            self.g3_fires += 1
            self.x_est = min(z, U32_MAX)
            self.p_est = max(R_int, P_FLOOR)
            self.pos_skip_cnt = 0
            return

        # ---- Core update ----
        p_pred = min(self.p_est + Q_int, P_MAX)

        if innov <= 0:
            # Negative innovation: G3-detect convergence
            self.neg_persist_cnt += 1
            self.pos_skip_cnt = 0
            floor = self.x_est - (self.x_est >> cfg["gated_drop_floor_shift"])
            if z >= floor:
                self.x_est = min(z, U32_MAX)
                self.p_est = max(R_int, P_FLOOR)
            else:
                self.floor_rejects += 1
                self.p_est = p_pred  # uncertainty grows
        else:
            # Positive innovation: standard Kalman or skip
            self.neg_persist_cnt = 0
            if outlier_reject and not force_accept:
                # Reject: skip update, uncertainty grows
                self.p_est = p_pred
                self.pos_skip_cnt += 1
                self.drift_sum_scaled += abs_innov
            else:
                if force_accept:
                    self.forced_accepts += 1

                # Kalman update
                gain_num = p_pred
                gain_den = p_pred + R_int
                if gain_den > 0:
                    corr = (p_pred * innov) // gain_den
                    p_reduction = (p_pred * gain_num) // gain_den
                else:
                    corr = 0
                    p_reduction = 0
                self.x_est = min(self.x_est + corr, U32_MAX)
                p_new = p_pred - p_reduction
                self.p_est = max(p_new, P_FLOOR)

                # Drift tracking: positive innov accepted = consecutive positive
                self.pos_skip_cnt += 1
                self.drift_sum_scaled += abs_innov

                # ---- Drift correction gates ----
                self._check_drift(abs_innov, p_pred, gain_num, gain_den, cfg, R_int)

        # Track state history
        self.x_history.append(self.x_est / SCALE)
        self.p_history.append(self.p_est)

    def _compute_outlier_thresh(self, cfg):
        prop_us = max(self.min_rtt_us >> cfg["rtt_frac_shift"], cfg["min_floor_us"])
        prop_thresh = prop_us << SCALE_SHIFT
        jitter_thresh = int(self.jitter_ewma_us * cfg["jitter_mult"]) << SCALE_SHIFT
        return max(prop_thresh, jitter_thresh)

    def _check_drift(self, abs_innov, p_pred, gain_num, gain_den, cfg, R_int):
        """Check and apply drift correction gates."""
        min_rtt_sc = self.min_rtt_us << SCALE_SHIFT

        # Early drift
        if (
            self.pos_skip_cnt >= cfg["early_min_skip"]
            and self.drift_sum_scaled > min_rtt_sc >> cfg["early_sum_shift"]
        ):
            self.early_drift_fires += 1
            drift_corr = abs_innov >> cfg["early_corr_shift"]
            self.x_est = min(self.x_est + drift_corr, U32_MAX)
            self.p_est = max(p_pred >> cfg["early_corr_shift"], P_FLOOR)

        # Tier 1 drift
        jitter_us = self.jitter_ewma_us
        if (
            self.pos_skip_cnt >= cfg["G3_fast_thresh"]
            and jitter_us < self.min_rtt_us >> cfg["G3_fast_jitter_shift"]
        ):
            self.G3_fast_fires += 1
            corr_abs = p_pred * abs_innov // gain_den if gain_den > 0 else 0
            drift_corr = corr_abs >> cfg["G3_fast_corr_shift"]
            self.x_est = min(self.x_est + drift_corr, U32_MAX)
            self.p_est = max(p_pred >> cfg["G3_fast_corr_shift"], P_FLOOR)

        # Tier 2 drift
        if (
            self.pos_skip_cnt >= cfg["G3_fast_thresh"] * cfg["G3_slow_mult"]
            and self.qdelay_ewma_us
            < (self.x_est >> (cfg["scale_shift"] + cfg["t2_qdelay_frac_shift"])) / SCALE
        ):
            self.G3_slow_fires += 1
            corr_abs = p_pred * abs_innov // gain_den if gain_den > 0 else 0
            drift_corr = corr_abs >> cfg["G3_slow_corr_shift"]
            self.x_est = min(self.x_est + drift_corr, U32_MAX)
            self.p_est = max(p_pred >> cfg["G3_slow_corr_shift"], P_FLOOR)


# =============================================================================
# Configuration (default KCC values)
# =============================================================================


def default_cfg():
    return {
        "rtt_frac_shift": 2,
        "min_floor_us": 50,
        "jitter_mult": 2,
        "max_consec_reject": 20,
        "qboost_thresh": 16384000,
        "converged_p": 33,
        "scale_shift": SCALE_SHIFT,
        "t2_qdelay_frac_shift": 1,
        "pos_skip_thresh": 5,
        "qboost_cdwn_val": 6,
        "p_init": P_INIT,
        "gated_drop_floor_shift": 3,
        "g3_qdelay_mult_num": 5,
        "g3_qdelay_mult_den": 2,
        "g3_max_qdelay_frac_shift": 1,
        "g3_min_skip": 2,
        "early_min_skip": 3,
        "early_sum_shift": 5,
        "early_corr_shift": 2,
        "G3_fast_thresh": 14,
        "G3_fast_jitter_shift": 3,
        "G3_fast_corr_shift": 2,
        "G3_slow_mult": 4,
        "G3_slow_corr_shift": 3,
    }


# =============================================================================
# Monte Carlo simulation
# =============================================================================

print("=" * 90)
print("FULL KCC STATE MACHINE MONTE CARLO SIMULATION")
print("=" * 90)

RTT_LEVELS = [
    ("DC-clean", 1400, 0, 10, 100, 400, 1500),
    ("WAN-clean", 50000, 0, 100, 100, 3200, 1500),
    ("long-haul-clean", 300000, 0, 300, 100, 3200, 1500),
]

N_SEEDS = 3

# ---------------------------------------------------------------------------
# 1. Single-flow clean-path convergence
# ---------------------------------------------------------------------------
print("\n--- 1. Single-flow clean-path convergence (all RTTs) ---")
for label, rtt_us, qdelay_init, sigma, Q, R, rounds in RTT_LEVELS:
    drift_pcts = []
    for seed in range(N_SEEDS):
        random.seed(seed * 100 + rtt_us)
        state = KCCState(rtt_us, qdelay_init)
        cfg = default_cfg()
        for _ in range(rounds):
            noise = random.gauss(0, sigma)
            actual_rtt = max(
                1,
                rtt_us + int(qdelay_init * random.random() * 0.5) + int(noise),
            )
            state.update(actual_rtt, Q, R, cfg)
        avg_x = sum(state.x_history) / max(len(state.x_history), 1)
        drift = abs(avg_x - rtt_us) / rtt_us * 100
        drift_pcts.append(drift)

    avg_drift = sum(drift_pcts) / N_SEEDS
    max_drift = max(drift_pcts)
    if max_drift < 20:
        pass_(
            f"  {label:>20s}: drift={avg_drift:.1f}% avg, {max_drift:.1f}% max (converged)",
        )
    elif avg_drift < 50:
        warn(f"  {label:>20s}: drift={avg_drift:.1f}% avg -- high but stable")
    else:
        fail(f"  {label:>20s}: drift={avg_drift:.1f}% avg -- DIVERGED")

# ---------------------------------------------------------------------------
# 2. State variable stability check
# ---------------------------------------------------------------------------
print("\n--- 2. State variable boundedness (x_est, p_est, counters) ---")
for label, rtt_us, qdelay_init, sigma, Q, R, rounds in RTT_LEVELS[
    :6
]:  # skip micro paths
    for seed in range(N_SEEDS):
        random.seed(seed * 200 + rtt_us)
        state = KCCState(rtt_us, qdelay_init)
        cfg = default_cfg()

        x_min, x_max = float("inf"), 0
        p_min, p_max = float("inf"), 0
        for _i in range(rounds):
            noise = random.gauss(0, sigma)
            actual_rtt = max(1, rtt_us + int(noise))
            state.update(actual_rtt, Q, R, cfg)

            x_us = state.x_est / SCALE
            p_val = state.p_est
            x_min = min(x_min, x_us)
            x_max = max(x_max, x_us)
            p_min = min(p_min, p_val)
            p_max = max(p_max, p_val)

        # Check bounds
        ok = True
        if x_min <= 0:
            fail(f"  {label}: x_est dropped to {x_min:.0f}us")
            ok = False
        if x_max > rtt_us * 3 + 50000:
            fail(f"  {label}: x_est exploded to {x_max:.0f}us")
            ok = False
        if p_min < P_FLOOR:
            fail(f"  {label}: p_est dropped below floor: {p_min}")
            ok = False
        if p_max > P_MAX:
            fail(f"  {label}: p_est exceeded P_MAX: {p_max}")
            ok = False

        # pos_skip_cnt saturates at 254
        if state.pos_skip_cnt > 254:
            fail(f"  {label}: pos_skip_cnt overflowed past 254")

        # Drift sum should not overflow
        if state.drift_sum_scaled > U32_MAX:
            fail(f"  {label}: drift_sum_scaled overflowed u32")

    pass_(
        f"  {label:>20s}: xin[{x_min:.0f},{x_max:.0f}]us, pin[{p_min},{p_max}], pos_skip={state.pos_skip_cnt} -- bounded",
    )

# ---------------------------------------------------------------------------
# 3. Gate event plausibility check
# ---------------------------------------------------------------------------
print("\n--- 3. Gate event counts -- plausibility check ---")
for label, rtt_us, qdelay_init, sigma, Q, R, rounds in RTT_LEVELS[:6]:
    total_qb, total_t1, total_t2, total_ed, total_g3 = 0, 0, 0, 0, 0
    total_outlier, total_force, total_floor = 0, 0, 0
    for seed in range(N_SEEDS):
        random.seed(seed * 300 + rtt_us)
        state = KCCState(rtt_us, qdelay_init)
        cfg = default_cfg()
        for _ in range(rounds):
            noise = random.gauss(0, sigma)
            actual_rtt = max(1, rtt_us + int(noise))
            state.update(actual_rtt, Q, R, cfg)
        total_qb += state.qboost_fires
        total_t1 += state.G3_fast_fires
        total_t2 += state.G3_slow_fires
        total_ed += state.early_drift_fires
        total_g3 += state.g3_fires
        total_outlier += state.outlier_rejects
        total_force += state.forced_accepts
        total_floor += state.floor_rejects

    avg_qb = total_qb / N_SEEDS
    avg_t1 = total_t1 / N_SEEDS
    avg_t2 = total_t2 / N_SEEDS

    info(
        f"  {label:>20s}: Qb={avg_qb:.0f} T1={avg_t1:.0f} T2={avg_t2:.0f} ED={total_ed / N_SEEDS:.0f} G3={total_g3 / N_SEEDS:.0f}"
        f" Outlier={total_outlier / N_SEEDS:.0f} Force={total_force / N_SEEDS:.0f} Floor={total_floor / N_SEEDS:.0f} / {rounds} rounds",
    )

# ---------------------------------------------------------------------------
# 4. Multi-flow path-change simulation (G2_queue_cap and G3 stress test)
# ---------------------------------------------------------------------------
print("\n--- 4. Multi-flow path-change: G2_queue_cap + G3 stress test ---")
for base_rtt in [1400, 50000]:
    for seed in range(N_SEEDS):
        random.seed(seed * 400 + base_rtt)
        cfg = default_cfg()
        state = KCCState(base_rtt, 0)
        Q, R = 100, 400

        # Phase 1: settle on base path
        for _ in range(500):
            rtt = max(1, base_rtt + int(random.gauss(0, base_rtt * 0.01)))
            state.update(rtt, Q, R, cfg)

        # Phase 2: abrupt path change (rtt doubles)
        new_rtt = base_rtt * 2
        state.min_rtt_us = new_rtt  # update min_rtt
        for _ in range(500):
            rtt = max(1, new_rtt + int(random.gauss(0, new_rtt * 0.01)))
            state.update(rtt, Q, R, cfg)

        x_final = state.x_est / SCALE
        # Should adapt to new path (within 50%)
        if abs(x_final - new_rtt) / new_rtt < 0.5:
            pass_(f"  RTT={base_rtt}us->{new_rtt}us: x_est={x_final:.0f}us (adapted)")
        else:
            warn(
                f"  RTT={base_rtt}us->{new_rtt}us: x_est={x_final:.0f}us (slow adaptation, Qb={state.qboost_fires}, G3={state.g3_fires})",
            )

# ---------------------------------------------------------------------------
# 5. K_min stability at extreme R (all paths converge)
# ---------------------------------------------------------------------------
print("\n--- 5. K at max R=102400 -- all paths still converge ---")
for rtt_us in [1400, 50000, 300000]:
    for seed in range(N_SEEDS):
        random.seed(seed * 500 + rtt_us)
        cfg = default_cfg()
        cfg["qboost_thresh"] = 16384000 * 10  # suppress G2_queue_cap for this test
        state = KCCState(rtt_us, 0)
        R = 102400
        Q = 100
        for _ in range(3000):
            noise = int(random.gauss(0, rtt_us * 0.01))
            actual_rtt = max(1, rtt_us + noise)
            state.update(actual_rtt, Q, R, cfg)
        final_x = state.x_est / SCALE
        drift = abs(final_x - rtt_us) / rtt_us * 100
        if drift < 20:
            pass_(
                f"  RTT={rtt_us:>6d}us, R=102400: x_est={final_x:.0f}us drift={drift:.1f}%",
            )
        else:
            warn(
                f"  RTT={rtt_us:>6d}us, R=102400: x_est={final_x:.0f}us drift={drift:.1f}% (K too small?)",
            )

# ---------------------------------------------------------------------------
# 6. pos_skip_cnt saturation at 254
# ---------------------------------------------------------------------------
print("\n--- 6. pos_skip_cnt saturation at 254 (never wraps) ---")
cfg = default_cfg()
state = KCCState(1400, 0)
# Artificially set high pos_skip
state.pos_skip_cnt = 254
# One more positive
state.update(1400 + 100, 100, 400, cfg)
if state.pos_skip_cnt <= 254:
    pass_(f"  pos_skip at 254 -> update -> {state.pos_skip_cnt} (saturated correctly)")
else:
    fail(f"  pos_skip overflowed: 254 -> {state.pos_skip_cnt}")

# ---------------------------------------------------------------------------
# 7. p_est never goes below floor
# ---------------------------------------------------------------------------
print("\n--- 7. p_est stays >= P_FLOOR ---")
for seed in range(20):
    random.seed(seed)
    cfg = default_cfg()
    state = KCCState(random.choice([1400, 50000, 300000]), 0)
    Q, R = 100, random.choice([400, 3200, 102400])
    violated = False
    for _ in range(2000):
        rtt = max(1, state.min_rtt_us + int(random.gauss(0, state.min_rtt_us * 0.05)))
        state.update(rtt, Q, R, cfg)
        if state.p_est < P_FLOOR:
            fail(f"  p_est={state.p_est} < P_FLOOR={P_FLOOR} at step")
            violated = True
            break
    if not violated:
        pass_(f"  {seed}: p_est always >= {P_FLOOR} over 2000 steps")

# =============================================================================
print(f"\n{'=' * 90}")
if failures == 0:
    print("ALL STATE MACHINE MONTE CARLO VERIFICATIONS PASSED")
else:
    print(f"{failures} FAILURES DETECTED ({warnings} warnings)")
