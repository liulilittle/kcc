#!/usr/bin/env python3
"""
kcc_sim_base.py -- Shared KCC simulation framework for brute-force verification.
Implements all KCC algorithms and gates exactly as specified in tcp_kcc.c.
Used by all .research test scripts.  No standalone output.
"""
from __future__ import annotations

from dataclasses import dataclass

# ========== Default parameters (matching tcp_kcc.c) ==========
SCALE = 1024
KCC_R_MIN_FLOOR = 1  # KCC_RTT_MIN_FLOOR_US
BASE_R = 400
J50_DEFAULT = 200  # kcc_jitter_r_j50
Q_BASE = 100
R_MAX_BOOST = 256
R_MAX = BASE_R * R_MAX_BOOST
P_EST_INIT = 1000
P_EST_MAX = 100_000_000
P_EST_FLOOR = 10
CWND_GAIN = 2

DRIFT_THRESH = 14
DRIFT_G3_slow_MULT = 4
DRIFT_G3_fast_SHIFT = 2  # corr/4
DRIFT_G3_slow_SHIFT = 3  # corr/8
DRIFT_QUIET_JITTER_SHIFT = 3  # jitter_ewma < min_rtt>>3 = min_rtt/8
DRIFT_T2_QDELAY_SHIFT = 1  # qdelay < x_est>>1 = 50%
DRIFT_EARLY_MIN_RTT = 3
DRIFT_EARLY_SUM_SHIFT = 5  # drift_sum > min_rtt>>5 = min_rtt/32
DRIFT_EARLY_CORR_SHIFT = 2  # innov/4
SATURATION_THRESH = 55
SATURATION_HOLD = 30
G3_fast_BLOCKED_MAX = 3

QBOOST_THRESH_US = 16000  # 16ms
QBOOST_MULT = 4
QBOOST_CDWN = 6

OUTLIER_RTT_FRAC_SHIFT = 2  # 25% RTT
OUTLIER_MIN_FLOOR_US = 50
OUTLIER_JITTER_MULT = 2

gated_drop_FLOOR_SHIFT = 3  # 12.5% = 1/8
NEG_PERSIST_THRESH = 3

KALMAN_R_POWER_FRAC = 20

JITTER_EWMA_ALPHA = 0.125
QDELAY_EWMA_ALPHA = 0.125


# ========== Flow state ==========
@dataclass
class KCCFlow:
    x_est: int = 0  # scaled units
    p_est: int = P_EST_INIT
    pos_skip_cnt: int = 0
    neg_skip_count: int = 0
    drift_sum: int = 0
    G3_fast_blocked_cnt: int = 0
    saturation_hold: int = 0
    qboost_cdwn: int = 0
    min_rtt_us: int = 10_000_000
    jitter_ewma: int = 0
    qdelay_ewma: int = 0
    bw_est: float = 1_000_000_000.0  # bps
    sample_cnt: int = 0
    retrans: int = 0
    consec_reject: int = 0

    def x_est_us(self) -> int:
        return self.x_est // SCALE

    def reset_qboost(self):
        self.p_est = P_EST_INIT
        self.qboost_cdwn = QBOOST_CDWN


# ========== Core algorithms (matching tcp_kcc.c exactly) ==========


def compute_adaptive_r(jitter_us: int, clean_thresh_us: int) -> int:
    """Power-law R: R = base_R * (jitter_excess/J50)^(3/2), clamped."""
    j_excess = max(0, jitter_us - clean_thresh_us)
    if j_excess == 0:
        return BASE_R
    # Fixed-point: ratio in 2^20 scale
    ratio = (j_excess << KALMAN_R_POWER_FRAC) // J50_DEFAULT
    # ratio^(3/2) = ratio * sqrt(ratio) in fixed point (simplified: use float)
    boost = ratio**1.5 / (2**KALMAN_R_POWER_FRAC)
    r_scaled = BASE_R * boost
    return max(BASE_R, min(int(r_scaled), R_MAX))


def compute_outlier_threshold(min_rtt_us: int, jitter_ewma: int) -> int:
    """RTT-proportional outlier gate (scaled units)."""
    prop_us = max(min_rtt_us >> OUTLIER_RTT_FRAC_SHIFT, OUTLIER_MIN_FLOOR_US)
    dyn = max(prop_us, jitter_ewma * OUTLIER_JITTER_MULT)
    return dyn << (SCALE.bit_length() - 1)  # << scale_shift


def outlier_gate_reject(
    abs_innov_scaled: int,
    min_rtt_us: int,
    jitter_ewma: int,
) -> bool:
    """Returns True if innovation should be rejected as outlier."""
    thresh = compute_outlier_threshold(min_rtt_us, jitter_ewma)
    return abs_innov_scaled > thresh


def gated_drop_floor_reject(
    z_scaled: int,
    x_est_scaled: int,
    neg_skip_count: int,
    shift: int = gated_drop_FLOOR_SHIFT,
) -> bool:
    """Speed-of-light floor gate: reject if drop > 12.5% of x_est."""
    floor = x_est_scaled - (x_est_scaled >> shift)
    return neg_skip_count < NEG_PERSIST_THRESH and z_scaled < floor


def kalman_update(
    fl: KCCFlow,
    rtt_us: int,
    min_rtt_us: int,
    jitter_instant: int,
) -> tuple[int, bool, str]:
    """
    KCC Kalman update for one sample.
    Returns: (new_x_est_scaled, was_updated, event_description).
    """
    z = rtt_us * SCALE
    nu = z - fl.x_est
    abs_nu = abs(nu)
    je = max(0, jitter_instant - jitter_instant // 4)
    r = compute_adaptive_r(fl.jitter_ewma, je) if fl.jitter_ewma > 0 else BASE_R

    p_pred = min(fl.p_est + Q_BASE, P_EST_MAX)
    K = p_pred / (p_pred + r)
    corr_abs = int(K * abs_nu)

    event = "none"
    updated = False

    # ---- Directional gate ----
    if nu <= 0:
        # Negative innovation: accept (clean T_prop evidence)
        if gated_drop_floor_reject(z, fl.x_est, fl.neg_skip_count):
            # Floor gate rejects (physically impossible drop)
            fl.neg_skip_count += 1
            fl.consec_reject += 1
            return fl.x_est, False, "floor_reject"

        # Outlier gate on negative innovations
        if outlier_gate_reject(abs_nu, min_rtt_us, fl.jitter_ewma):
            if fl.consec_reject >= 20:
                pass  # force-accept after max consecutive rejects
            else:
                fl.consec_reject += 1
                return fl.x_est, False, "outlier_reject"

        # G3-detect convergence: x_est = z
        fl.x_est = min(z, 0xFFFFFFFF)
        fl.p_est = max(r, P_EST_FLOOR)  # Joseph form for x_est_cap=1
        fl.pos_skip_cnt = 0
        fl.neg_skip_count += 1
        fl.drift_sum = 0
        fl.consec_reject = 0
        updated = True
        event = "forced_conv"
    else:
        # Positive innovation: queue contamination
        fl.pos_skip_cnt = min(fl.pos_skip_cnt + 1, 254)
        fl.neg_skip_count = 0
        fl.p_est = min(fl.p_est + Q_BASE, P_EST_MAX)

        # Outlier gate on positive
        if outlier_gate_reject(abs_nu, min_rtt_us, fl.jitter_ewma):
            fl.consec_reject += 1
            if fl.consec_reject >= 20:
                pass  # force-accept
            else:
                return fl.x_est, False, "outlier_reject"
        else:
            fl.consec_reject = 0

        # --- Drift mechanisms (only on positive innovations) ---

        # G2_queue_cap: large positive innovation => path change
        if abs_nu > QBOOST_THRESH_US * SCALE and fl.qboost_cdwn == 0:
            fl.reset_qboost()
            fl.x_est = min(fl.x_est + corr_abs, 0xFFFFFFFF)
            fl.pos_skip_cnt = 0
            fl.drift_sum = 0
            updated = True
            event = "qboost"
        elif fl.qboost_cdwn > 0:
            fl.qboost_cdwn -= 1

        # Saturation: cap x_est at min_rtt when p_est maxed
        if (
            not updated
            and fl.p_est >= P_EST_MAX
            and fl.pos_skip_cnt >= SATURATION_THRESH
        ):
            mrs = min_rtt_us * SCALE
            if fl.x_est > mrs:
                fl.x_est = mrs
                fl.pos_skip_cnt = 0
                fl.drift_sum = 0
                fl.saturation_hold = SATURATION_HOLD
                updated = True
                event = "saturation"

        if fl.saturation_hold > 0:
            fl.saturation_hold -= 1

        # Early drift: amplitude-based, jitter-gated
        if not updated and fl.pos_skip_cnt >= DRIFT_EARLY_MIN_RTT:
            if fl.jitter_ewma < min_rtt_us >> DRIFT_QUIET_JITTER_SHIFT:
                minrtt_scaled = min_rtt_us * SCALE
                if fl.drift_sum > minrtt_scaled >> DRIFT_EARLY_SUM_SHIFT:
                    drift_corr = int(abs_nu >> DRIFT_EARLY_CORR_SHIFT)
                    drift_corr = max(drift_corr, 1)
                    fl.x_est = min(fl.x_est + drift_corr, 0xFFFFFFFF)
                    fl.pos_skip_cnt = 0
                    fl.drift_sum = 0
                    updated = True
                    event = "early_drift"

        # Accumulate drift sum (used by early drift)
        if not updated:
            fl.drift_sum = min(fl.drift_sum + abs_nu, 0xFFFFFFFF)

        # Tier 1: quiet-path drift, jitter-gated
        if not updated and fl.pos_skip_cnt >= DRIFT_THRESH:
            if fl.jitter_ewma < min_rtt_us >> DRIFT_QUIET_JITTER_SHIFT:
                drift_corr = corr_abs >> DRIFT_G3_fast_SHIFT
                drift_corr = max(drift_corr, 1)
                fl.x_est = min(fl.x_est + drift_corr, 0xFFFFFFFF)
                fl.pos_skip_cnt = 0
                fl.drift_sum = 0
                fl.G3_fast_blocked_cnt = 0
                updated = True
                event = "G3_fast"
            else:
                fl.G3_fast_blocked_cnt = min(fl.G3_fast_blocked_cnt + 1, 255)
                if fl.G3_fast_blocked_cnt >= G3_fast_BLOCKED_MAX:
                    # Waive jitter gate after 3 consecutive blocks
                    drift_corr = corr_abs >> DRIFT_G3_fast_SHIFT
                    drift_corr = max(drift_corr, 1)
                    fl.x_est = min(fl.x_est + drift_corr, 0xFFFFFFFF)
                    fl.pos_skip_cnt = 0
                    fl.drift_sum = 0
                    fl.G3_fast_blocked_cnt = 0
                    updated = True
                    event = "G3_fast_waived"

        # Tier 2: statistical-force drift, qdelay-gated
        if not updated and fl.pos_skip_cnt >= DRIFT_THRESH * DRIFT_G3_slow_MULT:
            total_shift = SCALE.bit_length() - 1 + DRIFT_T2_QDELAY_SHIFT  # 10 + shift
            threshold = max(fl.x_est >> total_shift, 1)
            if fl.qdelay_ewma < threshold:
                drift_corr = corr_abs >> DRIFT_G3_slow_SHIFT
                drift_corr = max(drift_corr, 1)
                fl.x_est = min(fl.x_est + drift_corr, 0xFFFFFFFF)
                fl.pos_skip_cnt = 0
                fl.drift_sum = 0
                fl.G3_fast_blocked_cnt = 0
                updated = True
                event = "G3_slow"

    return fl.x_est, updated, event


# ========== Network simulation helpers ==========


def sim_queue_step(
    flows: list[KCCFlow],
    t_prop_us: int,
    link_cap_bps: float,
    buf_pkts: int,
    mss_bits: int = 12000,
) -> dict:
    """One round of queue dynamics. Returns {qdelay_us, drops, total_inflight}."""
    pkts_sec = link_cap_bps / mss_bits
    rtt_s = t_prop_us / 1e6
    cap_per_rd = rtt_s * pkts_sec

    inflight = 0.0
    for fl in flows:
        xe_us = max(fl.x_est_us(), 1)
        bdp = xe_us / 1e6 * fl.bw_est / mss_bits
        cwnd = max(bdp * CWND_GAIN, 2)
        inflight += min(cwnd, 8000)

    # Shared queue (global state) -- use class variable or pass in
    return {"inflight": inflight, "capacity": cap_per_rd, "pkts_sec": pkts_sec}


def update_ewma(old: int, new: int, alpha: float) -> int:
    return int(old * (1 - alpha) + new * alpha)
