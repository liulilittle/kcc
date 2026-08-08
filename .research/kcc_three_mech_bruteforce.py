#!/usr/bin/env python3
"""KCC Three-Mechanism Exhaustive Brute-Force Analysis
====================================================
Part 1: G2_queue_cap parameter sweep
Part 2: Drift detection parameter sweep
Part 3: Outlier gate parameter sweep
Part 4: Cross-optimization of all three mechanisms

This analysis is calibrated against the KCC codebase (tcp_kcc.c)
and builds on the G3 C3 analysis from g3_c3_bruteforce.py.
"""

import math

# ============================================================
# PHYSICAL MODELS
# ============================================================
# Innovation noise model under H0 (no Tprop change)
# Innovation ~ N(0, sigma_innov^2) with slight positive bias
# sigma_innov ~ jitter * 1.2 (Kalman residual scaling)
# P(nu > 0 | H0) = p_pos_H0 ~ 0.52 (directional gate bias)
# Under H1 (Tprop jump of Delta):
# First sample: nu ~ Delta + N(0, sigma^2), Delta >> sigma => nu >> 0
# pos_skip_cnt accumulates from its pre-step carryover value
# G2_queue_cap: |nu| > qboost_thresh => typically fires on first post-step sample
# G3: pos_skip >= 2 AND nu > 2.5*qdelay AND qdelay < RTT/2
# Drift Tier-1: pos_skip >= drift_thresh AND jitter < min_rtt/8
# Drift Tier-2: pos_skip >= drift_thresh * 8 (unconditional)

# ============================================================
# SCENARIO DEFINITIONS
# ============================================================
SCENARIOS = {
    "WAN": {
        "min_rtt_us": 50000,
        "qdelay_us": 5000,
        "jitter_us": 3000,
        "Tprop_delta_us": 200000,
        "desc": "WAN 50ms RTT, 5ms Q, 3ms jitter, Tprop+200ms",
        "p_pos_H0": 0.52,
        "fpr_innov_positive": 0.15,
        "fpr_qdelay_small": 0.65,
    },
    "DC": {
        "min_rtt_us": 1000,
        "qdelay_us": 100,
        "jitter_us": 50,
        "Tprop_delta_us": 1000,
        "desc": "DC 1ms RTT, 0.1ms Q, 0.05ms jitter, Tprop+1ms",
        "p_pos_H0": 0.52,
        "fpr_innov_positive": 0.03,
        "fpr_qdelay_small": 0.95,
    },
    "Mobile": {
        "min_rtt_us": 20000,
        "qdelay_us": 3000,
        "jitter_us": 8000,
        "Tprop_delta_us": 40000,
        "desc": "Mobile 20ms RTT, 3ms Q, 8ms jitter, Tprop+40ms",
        "p_pos_H0": 0.56,
        "fpr_innov_positive": 0.30,
        "fpr_qdelay_small": 0.45,
    },
    "Satellite": {
        "min_rtt_us": 500000,
        "qdelay_us": 5000,
        "jitter_us": 1000,
        "Tprop_delta_us": 20000,
        "desc": "Satellite 500ms RTT, 5ms Q, 1ms jitter, Tprop+20ms",
        "p_pos_H0": 0.52,
        "fpr_innov_positive": 0.10,
        "fpr_qdelay_small": 0.80,
    },
}

# ============================================================
# ANALYSIS FUNCTIONS
# ============================================================


def analyze_qboost(sc, q_boost_thresh_ms, qboost_cdwn, pos_skip_thresh):
    """Analyze G2_queue_cap parameters for given scenario."""
    min_rtt = sc["min_rtt_us"]
    jitter = sc["jitter_us"]
    Delta = sc["Tprop_delta_us"]
    rtt_us = min_rtt + sc.get("qdelay_us", 0)
    p_pos = sc["p_pos_H0"]

    q_boost_thresh_us = q_boost_thresh_ms * 1000

    # Expected carryover of pos_skip from pre-step period
    carryover = p_pos / (1.0 - p_pos)

    # G2_queue_cap fires when:
    # 1. qboost_cdwn == 0 (not in cooldown)
    # 2. innov > 0 AND abs(innov) > qboost_thresh
    # 3. p_est <= converged_val (~33)
    # 4. pos_skip < pos_skip_thresh
    # 5. qdelay_ewma < x_est / 2

    # G2_queue_cap detection delay (RTTs from step to qboost fire)
    expected_delay_rtt = max(1.0, pos_skip_thresh - carryover)

    # False positive rate per RTT under H0
    # P(qboost fires | H0) = P(|innov| > qboost_thresh | H0)
    # Under H0: innov ~ N(0, sigma^2) where sigma ~ jitter * 1.2
    sigma = jitter * 1.2
    z_qb = q_boost_thresh_us / max(sigma, 1.0)
    fp_per_rtt = 2.0 * (1.0 - 0.5 * (1.0 + math.erf(z_qb / math.sqrt(2.0))))

    # Probability that p_est <= 33 when step occurs
    # Under steady-state H0 with Kalman filter: p_est converges to p_ss
    # p_ss = (-Q + sqrt(Q^2 + 4*Q*R)) / 2
    # With default Q=100, R=400: p_ss = (-100 + sqrt(10000+160000))/2 = (-100+412)/2 = 156
    # p_est converges but may be > 33 (only at low R). For high R, p_est stays high.
    # So G2_queue_cap may be blocked on noisy paths where R is large.
    p_converged = 1.0 / (1.0 + (jitter / 200.0) ** 1.5)

    # G2_queue_cap blocked when p_est > 33 (not converged)

    # Probability qboost fires on first post-step sample
    # Under H1: innov = Delta + noise, Delta >> threshold
    # P(qboost fires | H1) ~= 1.0 for large Delta
    p_fire_H1 = (
        1.0
        if Delta > q_boost_thresh_us * 2
        else max(0.5, Delta / q_boost_thresh_us - 0.5)
    )

    # Cooldown blocks subsequent fires
    # After fire, kcc_step sets qboost_cdwn = 6, inhibits refire
    # If cooldown too short, may double-fire -> unnecessary aggressiveness
    # If too long, may miss second step

    return {
        "q_boost_thresh_ms": q_boost_thresh_ms,
        "qboost_cdwn": qboost_cdwn,
        "pos_skip_thresh": pos_skip_thresh,
        "detection_delay_rtt": round(expected_delay_rtt, 2),
        "detection_delay_ms": round(expected_delay_rtt * rtt_us / 1000.0, 1),
        "fp_per_rtt": round(fp_per_rtt, 6),
        "p_converged": round(p_converged, 3),
        "p_fire_H1": round(p_fire_H1, 3),
    }


def analyze_drift(sc, drift_thresh, G3_fast_div, G3_slow_mult):
    """
    Analyze drift detection for given parameters.
    drift_thresh: Number of consecutive positive innovations for Tier-1
    G3_fast_div: Divisor for Tier-1 correction (corr / G3_fast_div)
    G3_slow_mult: Multiplier for Tier-2 threshold (drift_thresh * G3_slow_mult)
    Returns detection delay and false positive stats.
    """
    min_rtt = sc["min_rtt_us"]
    jitter = sc["jitter_us"]
    sc["Tprop_delta_us"]
    qdelay = sc.get("qdelay_us", 0)
    rtt_us = min_rtt + qdelay
    p_pos = sc["p_pos_H0"]

    quiet_jitter_thresh = min_rtt >> 3

    carryover = p_pos / (1.0 - p_pos)

    # Tier-1 detection delay
    delay_G3_fast_rtt = max(drift_thresh - carryover, 1.0)
    G3_fast_jitter_window = 5.0
    G3_fast_viable = delay_G3_fast_rtt <= G3_fast_jitter_window

    if G3_fast_viable:
        detection_delay_rtt = delay_G3_fast_rtt
        tier_used = "Tier-1"
    else:
        delay_G3_slow_rtt = max(drift_thresh * G3_slow_mult - carryover, 1.0)
        detection_delay_rtt = delay_G3_slow_rtt
        tier_used = "Tier-2"

    K_min = 0.05
    convergence_steps_G3_fast = max(1.0, 0.95 * G3_fast_div / K_min)
    convergence_steps_G3_slow = max(1.0, 0.95 * G3_slow_mult / K_min)

    # False positive probability
    p_pos_ge_drift = p_pos ** (drift_thresh - 1)
    jitter / max(min_rtt, 1)
    p_jitter_low_H0 = (
        1.0
        if jitter * 2 < quiet_jitter_thresh
        else (jitter / quiet_jitter_thresh if jitter < quiet_jitter_thresh else 0.1)
    )

    fp_G3_fast_per_event = p_pos_ge_drift * p_jitter_low_H0
    p_pos_ge_G3_slow = p_pos ** (drift_thresh * G3_slow_mult - 1)
    fp_G3_slow_per_event = p_pos_ge_G3_slow

    fp_G3_fast_per_rtt = fp_G3_fast_per_event
    fp_G3_slow_per_rtt = fp_G3_slow_per_event

    # G2_queue_cap / G3 overlap
    overlap_size = max(0, min(3, drift_thresh - 2) + 1)
    qboost_range = 3
    g3_range = 254
    q_exclusive = 2
    g3_exclusive = g3_range - overlap_size - 2
    overlap_fraction = overlap_size / (qboost_range + g3_range - overlap_size)

    return {
        "drift_thresh": drift_thresh,
        "G3_fast_div": G3_fast_div,
        "G3_slow_mult": G3_slow_mult,
        "detection_delay_rtt": round(detection_delay_rtt, 2),
        "detection_delay_ms": round(detection_delay_rtt * rtt_us / 1000.0, 1),
        "tier_used": tier_used,
        "fp_G3_fast_per_rtt": round(fp_G3_fast_per_rtt, 6),
        "fp_G3_slow_per_rtt": round(fp_G3_slow_per_rtt, 6),
        "convergence_steps_G3_fast": round(convergence_steps_G3_fast, 1),
        "convergence_steps_G3_slow": round(convergence_steps_G3_slow, 1),
        "g3_exclusive": g3_exclusive,
        "overlap_fraction": round(overlap_fraction, 3),
        "q_exclusive": q_exclusive,
    }


def sweep_qboost():
    """Exhaustive G2_queue_cap parameter sweep."""
    results = []
    for sc_name, sc in SCENARIOS.items():
        for q_boost_thresh_ms in [1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 15, 20]:
            for qboost_cdwn in [2, 4, 6, 8, 10, 12, 16, 24, 32]:
                for pos_skip_thresh in [3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16]:
                    r = analyze_qboost(
                        sc,
                        q_boost_thresh_ms,
                        qboost_cdwn,
                        pos_skip_thresh,
                    )
                    r["scenario"] = sc_name
                    results.append(r)
    return results


def sweep_drift():
    """Exhaustive drift detection parameter sweep."""
    results = []
    for sc_name, sc in SCENARIOS.items():
        for drift_thresh in [10, 12, 14, 16, 18, 20]:
            for G3_fast_div in [2, 3, 4]:
                for G3_slow_mult in [4, 6, 8]:
                    r = analyze_drift(sc, drift_thresh, G3_fast_div, G3_slow_mult)
                    r["scenario"] = sc_name
                    results.append(r)
    return results


if __name__ == "__main__":
    qboost_results = sweep_qboost()
    drift_results = sweep_drift()
    print(f"G2_queue_cap sweep: {len(qboost_results)} combinations")
    print(f"Drift sweep: {len(drift_results)} combinations")
