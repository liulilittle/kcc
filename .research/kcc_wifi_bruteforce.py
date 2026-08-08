#!/usr/bin/env python3

"""KCC WiFi WLAN Exhaustive Parameter Brute-Force Analysis
=======================================================
WiFi Scenario Characteristics:
  - RTT: 2-20ms (typical 5ms @ 5GHz, 10ms @ 2.4GHz)
  - Jitter: 2-10ms (interference / CSMA backoff / AP queuing)
  - Bandwidth: 50-600Mbps (802.11ac/ax)
  - T_prop: <1ms (LAN), but AP switching may cause jumps
  - Buffer: AP-side medium buffering
  - Loss: occasional interference drops
  - Mobility: AP roaming (T_prop may shift slightly)

Key WiFi-specific challenges:
  1. Jitter/RTT ratio is very high (up to 100%)
  2. TSO aggregation causes burst self-queue at AP
  3. CSMA backoff produces non-Gaussian delay spikes
  4. AP roaming = sudden T_prop change
  5. Medium buffers at AP = potential bufferbloat

Parameter groups:
  1. Kalman noise model: Q, R, p_est_init
  2. Persistence & detection: neg_persist_thresh, drift_thresh, pos_skip_thresh
  3. Outlier gating: outlier_ms, outlier_jitter_mult
  4. Buffer: probe_rtt_base_sec, qdelay thresholds
  5. TSO: tso_max_segs
  6. min_rtt sticky: kcc_minrtt_sticky ratio

References: tcp_kcc.c (KCC v2.0)
All values derived directly from KCC source code parameter definitions."""

import math

# ============================================================
# WiFi SCENARIO MODELS
# ============================================================
WiFi_SCENARIOS = {
    "WiFi_5GHz": {
        "min_rtt_ms": 3.0,
        "avg_rtt_ms": 5.0,
        "bw_mbps": 500,
        "jitter_ms": 3.0,
        "mss": 1448,
        "desc": "5GHz 500Mbps, 3ms RTT, 3ms jitter",
        "p_clean": 0.70,
        "target_util": 0.95,
    },
    "WiFi_2GHz": {
        "min_rtt_ms": 6.0,
        "avg_rtt_ms": 10.0,
        "bw_mbps": 200,
        "jitter_ms": 7.0,
        "mss": 1448,
        "desc": "2.4GHz 200Mbps, 6ms RTT, 7ms jitter",
        "p_clean": 0.55,
        "target_util": 0.90,
    },
    "WiFi_congested": {
        "min_rtt_ms": 8.0,
        "avg_rtt_ms": 15.0,
        "bw_mbps": 80,
        "jitter_ms": 8.0,
        "mss": 1448,
        "desc": "Congested 2.4GHz 80Mbps, 8ms RTT, 8ms jitter",
        "p_clean": 0.40,
        "target_util": 0.85,
    },
    "WiFi_6GHz": {
        "min_rtt_ms": 2.0,
        "avg_rtt_ms": 3.0,
        "bw_mbps": 800,
        "jitter_ms": 2.0,
        "mss": 1448,
        "desc": "6GHz WiFi6E 800Mbps, 2ms RTT, 2ms jitter",
        "p_clean": 0.80,
        "target_util": 0.97,
    },
    "WiFi_mobile_roam": {
        "min_rtt_ms": 10.0,
        "avg_rtt_ms": 18.0,
        "bw_mbps": 300,
        "jitter_ms": 6.0,
        "mss": 1448,
        "desc": "Mobile WiFi roaming, 10ms RTT, 6ms jitter",
        "p_clean": 0.50,
        "target_util": 0.88,
    },
}

# ============================================================
# KALMAN SCALE
# ============================================================
KALMAN_SCALE = 1024
USEC_PER_MSEC = 1000

# ============================================================
# UTILITY FUNCTIONS
# ============================================================


def bdp_packets(rtt_ms, bw_mbps, mss=1448):
    """BDP in packets."""
    return (bw_mbps * 1e6 * rtt_ms / 1000.0) / (mss * 8)


def bdp_bytes(rtt_ms, bw_mbps):
    return (bw_mbps * 1e6 * rtt_ms / 1000.0) / 8.0


# ============================================================
# PART 1: KALMAN NOISE MODEL
# ============================================================
# kcc_kalman_q in {100, 150, 200}
# kcc_kalman_r in {400, 600, 800}
# kcc_kalman_p_est_init in {1000, 1500}


def compute_kalman_ss(q_base, r_base, min_rtt_ms, jitter_ms, bw_mbps, mss=1448):
    """
    Compute steady-state Kalman parameters for given Q/R.
    Adaptive Q on WiFi: Q_adapted = Q_base * max(q_min_factor, min_rtt_us/1000)
    Adaptive R on WiFi: R_adapted = R_base + (jitter - jr_thresh) * R_base / jr_scale
    """
    min_rtt_us = min_rtt_ms * 1000
    # Q adaptation
    rtt_factor = max(10.0, min_rtt_us / 1000.0)
    q_adapted = min(q_base * rtt_factor, 2000.0)
    # R adaptation for WiFi jitter
    jr_thresh = 1000  # 1ms
    jr_scale = 5000  # 5ms
    jitter_us = jitter_ms * 1000
    if jitter_us > jr_thresh:
        r_jitter_boost = (jitter_us - jr_thresh) * q_base / jr_scale
    else:
        r_jitter_boost = 0
    r_adapted = min(r_base + r_jitter_boost, r_base * 8)

    # Steady-state covariance (solve covariance_update: p_ss = (p_ss + q) * r / (p_ss + q + r))
    # Quadratic: p_ss^2 + q*p_ss - q*r = 0
    # p_ss = (-q + sqrt(q^2 + 4*q*r)) / 2
    p_ss = (-q_adapted + math.sqrt(q_adapted**2 + 4.0 * q_adapted * r_adapted)) / 2.0
    K_obs_drain = p_ss / (p_ss + r_adapted)

    # BDP-based analysis
    bdp_packets(min_rtt_ms, bw_mbps, mss)

    # Convergence time to 95% of step
    convergence_95 = math.ceil(3.0 / K_obs_drain) if K_obs_drain > 0 else float("inf")
    convergence_95_ms = convergence_95 * min_rtt_ms

    # Noise amplification: variance of estimate from measurement noise
    noise_amp = K_obs_drain**2 * r_adapted / (1.0 - (1.0 - K_obs_drain) ** 2)

    # Tracking bandwidth (cutoff frequency ~ K_obs_drain / (2*pi) in samples)
    tracking_bw = K_obs_drain / (2.0 * math.pi)

    # Q/R ratio determines filter aggressiveness
    qr_ratio = q_adapted / r_adapted

    return {
        "q_base": q_base,
        "r_base": r_base,
        "q_adapted": q_adapted,
        "r_adapted": r_adapted,
        "p_ss": p_ss,
        "K_obs_drain": K_obs_drain,
        "qr_ratio": qr_ratio,
        "convergence_95_rtt": convergence_95,
        "convergence_95_ms": convergence_95_ms,
        "noise_amp": noise_amp,
        "tracking_bw": tracking_bw,
        "rtt_factor": rtt_factor,
    }


def analyze_kalman_wifi():
    """Part 1: Kalman noise model exhaustive analysis for WiFi."""
    print("=" * 80)
    print("PART 1: KALMAN NOISE MODEL -- WiFi Exhaustive Analysis")
    print("=" * 80)
    all_results = []
    for sc_name, sc in WiFi_SCENARIOS.items():
        print("\n--- {}: {} ---".format(sc_name, sc["desc"]))
        print(
            "    BDP = {:.0f} pkts, {:.0f} KB".format(
                bdp_packets(sc["min_rtt_ms"], sc["bw_mbps"]),
                bdp_bytes(sc["min_rtt_ms"], sc["bw_mbps"]) / 1024,
            ),
        )
        print()
        header = (
            "{:>4s} {:>4s} {:>5s} {:>6s} {:>7s} {:>7s} "
            "{:>7s} {:>7s} {:>7s} {:>6s}".format(
                "Q",
                "R",
                "P0",
                "K_obs_drain",
                "Q_ada",
                "R_ada",
                "Conv95",
                "N_amp",
                "TrkBW",
                "QR",
            )
        )
        print(header)
        print("-" * len(header))
        for q in [100, 150, 200]:
            for r in [400, 600, 800]:
                for p0 in [1000, 1500]:
                    ks = compute_kalman_ss(
                        q,
                        r,
                        sc["min_rtt_ms"],
                        sc["jitter_ms"],
                        sc["bw_mbps"],
                    )
                    ks["p_est_init"] = p0
                    ks["scenario"] = sc_name
                    print(
                        "{:>4d} {:>4d} {:>5d} {:>6.3f} {:>7.0f} {:>7.0f} "
                        "{:>7.0f} {:>7.1f} {:>7.4f} {:>6.2f}".format(
                            q,
                            r,
                            p0,
                            ks["K_obs_drain"],
                            ks["q_adapted"],
                            ks["r_adapted"],
                            ks["convergence_95_rtt"],
                            ks["noise_amp"],
                            ks["tracking_bw"],
                            ks["qr_ratio"],
                        ),
                    )
                    all_results.append(ks)
        print()
        print("  WiFi-specific Kalman considerations:")
        print("    - RTT={}ms, jitter={}ms".format(sc["min_rtt_ms"], sc["jitter_ms"]))
        print(
            "    - Jitter/RTT ratio = {:.0f}%".format(
                sc["jitter_ms"] / sc["min_rtt_ms"] * 100,
            ),
        )
        print("    - Q adaptation factor = {}".format(max(10, sc["min_rtt_ms"])))
        print("    - On WiFi, high jitter boosts R_adapted significantly")
        print(
            "    - TSO bursts (64 segs * 1448B) = negligible at {}Mbps".format(
                sc["bw_mbps"],
            ),
        )
        print()
        print(f"  >>> For {sc_name}:")
        print("      - K_obs_drain should target 0.15-0.35 for WiFi jitter")
        print(
            "      - Lower Q reduces noise tracking; higher Q tracks AP switches faster",
        )
        print("      - Higher R smooths jitter but slows convergence")
    return all_results


# ============================================================
# PART 2: PERSISTENCE & DETECTION PARAMETERS
# ============================================================
# kcc_negative_innov_count_thresh in {2, 3, 4}
# kcc_kalman_drift_thresh in {10, 12, 14}
# kcc_kalman_pos_skip_thresh in {4, 5, 6}


def compute_neg_persist_stats(sc, neg_persist):
    """
    Compute neg_persist_thresh statistics for WiFi.
    Under H0 (stable T_prop): P(neg_skip >= N) ~= (0.5)^N
    Under H1 (T_prop decreased by AP switch): detection latency

    WiFi-specific: jitter can cause many positive innovations (CSMA backoff)
    so neg_skip counter resets frequently. The neg_persist mechanism
    must balance guarding against noise-driven floor bypass vs timely
    convergence when T_prop genuinely drops (AP roaming).
    """
    # Placeholder for full implementation


def compute_drift_detect_stats(sc, drift_thresh, pos_skip_thresh, p_pos_H0=None):
    """
    Compute drift detection statistics for given parameters.
    drift_thresh: Number of consecutive positive innovations for Tier-1
    pos_skip_thresh: Number of positive innovations for G2_queue_cap overlap
    Returns detection delay and false positive stats.
    """
    min_rtt = int(sc["min_rtt_ms"] * 1000)
    jitter = int(sc["jitter_ms"] * 1000)
    sc.get("Tprop_delta_us", 0)
    qdelay = sc.get("qdelay_us", sc.get("avg_rtt_ms", 0) * 1000 - min_rtt)
    rtt_us = min_rtt + qdelay
    if p_pos_H0 is None:
        p_pos_H0 = sc.get("p_pos_H0", 0.52)

    quiet_jitter_thresh = min_rtt >> 3  # min_rtt / 8

    carryover = p_pos_H0 / (1.0 - p_pos_H0)

    delay_G3_fast_rtt = max(drift_thresh - carryover, 1.0)
    G3_fast_jitter_window = 5.0
    G3_fast_viable = delay_G3_fast_rtt <= G3_fast_jitter_window

    if G3_fast_viable:
        detection_delay_rtt = delay_G3_fast_rtt
    else:
        delay_G3_slow_rtt = max(drift_thresh * 8 - carryover, 1.0)
        detection_delay_rtt = delay_G3_slow_rtt

    p_pos_ge_drift = p_pos_H0 ** (drift_thresh - 1)
    jitter_ratio = jitter / max(min_rtt, 1)
    p_jitter_low_H0 = (
        1.0
        if jitter * 2 < quiet_jitter_thresh
        else (jitter / quiet_jitter_thresh if jitter < quiet_jitter_thresh else 0.1)
    )
    fp_G3_fast_per_event = p_pos_ge_drift * p_jitter_low_H0

    p_pos_ge_G3_slow = p_pos_H0 ** (drift_thresh * 8 - 1)
    fp_G3_slow_per_event = p_pos_ge_G3_slow

    detect_ms_G3_fast = delay_G3_fast_rtt * rtt_us / 1000.0

    # G3/G2_queue_cap overlap analysis
    g3_overlap_size = max(0, min(pos_skip_thresh - 1, 254) - 2 + 1)
    q_exclusive = min(pos_skip_thresh, 3)

    return {
        "drift_thresh": drift_thresh,
        "pos_skip_thresh": pos_skip_thresh,
        "p_pos_H0": p_pos_H0,
        "fp_G3_fast": fp_G3_fast_per_event,
        "fp_G3_slow": fp_G3_slow_per_event,
        "detect_delay_G3_fast_rtt": detection_delay_rtt,
        "detect_ms_G3_fast": detect_ms_G3_fast,
        "g3_overlap_size": g3_overlap_size,
        "q_exclusive_range": q_exclusive,
        "jitter_ratio": jitter_ratio,
    }


def analyze_persistence_wifi():
    """Part 2: Persistence & detection parameters exhaustive analysis for WiFi."""
    print("\n" + "=" * 80)
    print("PART 2: PERSISTENCE & DETECTION PARAMETERS -- WiFi Exhaustive Analysis")
    print("=" * 80)
    for sc_name, sc in WiFi_SCENARIOS.items():
        print("\n--- {}: {} ---".format(sc_name, sc["desc"]))
        print(
            "    WiFi jitter/RTT ratio = {:.0f}%".format(
                sc["jitter_ms"] / sc["min_rtt_ms"] * 100,
            ),
        )
        print("\n  drift_thresh x pos_skip_thresh:")
        print(
            "  {:>4s} {:>4s} {:>10s} {:>15s} {:>7s} {:>7s} {:>5s} {:>6s}".format(
                "dTh",
                "pSk",
                "FP_T1",
                "FP_T2",
                "DetT1",
                "DetMs",
                "G3ov",
                "Qexcl",
            ),
        )
        print("  " + "-" * 55)
        for dt in [10, 12, 14]:
            for pst in [4, 5, 6]:
                r = compute_drift_detect_stats(sc, dt, pst)
                print(
                    "  {:>4d} {:>4d} {:>10.2e} {:>15.2e} {:>7.1f} {:>7.0f} {:>5d} {:>6d}".format(
                        dt,
                        pst,
                        r["fp_G3_fast"],
                        r["fp_G3_slow"],
                        r["detect_delay_G3_fast_rtt"],
                        r["detect_ms_G3_fast"],
                        r["g3_overlap_size"],
                        r["q_exclusive_range"],
                    ),
                )

        print("\n  WiFi-specific recommendations:")
        print(
            "    - With {:.0f}ms jitter on {:.0f}ms RTT:".format(
                sc["jitter_ms"],
                sc["min_rtt_ms"],
            ),
        )
        print("    - pos_skip_thresh: lower G3/G2_queue_cap overlap preferred")
        print("    - drift_thresh: shorter preferred for fast AP roam response")


# ============================================================
# PART 3: OUTLIER GATING
# ============================================================
def phi(x):
    """Standard normal CDF approximation."""
    return 0.5 * (1.0 + math.erf(x / math.sqrt(2.0)))


def compute_outlier_wifi(sc, outlier_ms, jitter_mult):
    """
    Compute outlier gate stats for WiFi.
    Outlier gate: |innov| > max(outlier_ms*1000, jitter_ewma * jitter_mult)
    On WiFi, jitter is high (2-10ms), so the adaptive threshold is often
    the binding constraint. This means outlier_ms mainly acts as a floor.
    """
    jitter_ms = sc["jitter_ms"]
    jitter_us = jitter_ms * 1000
    sc["min_rtt_ms"]
    base_thresh_us = outlier_ms * USEC_PER_MSEC
    adaptive_thresh_us = jitter_us * jitter_mult
    effective_thresh_ms = max(base_thresh_us, adaptive_thresh_us) / 1000.0

    # Under H0: |innov| ~ folded-normal(0, sigma) where sigma ~ jitter * 1.2
    sigma = jitter_ms * 1.2
    z_clean = effective_thresh_ms / max(sigma, 0.01)
    p_reject_clean = 2.0 * (1.0 - phi(z_clean))

    # Under noise spike: magnitude = outlier_ms * 3
    noise_spike_ms = outlier_ms * 3.0
    z_noise = (noise_spike_ms - effective_thresh_ms) / max(sigma, 0.01)
    p_admit_noise = 1.0 - phi(z_noise) if z_noise > 0 else 0.85

    feedback_risk = adaptive_thresh_us > base_thresh_us
    feedback_severity = (
        effective_thresh_ms / base_thresh_us if base_thresh_us > 0 else 0
    )

    return {
        "outlier_ms": outlier_ms,
        "jitter_mult": jitter_mult,
        "base_thresh_us": base_thresh_us,
        "adaptive_thresh_us": adaptive_thresh_us,
        "effective_thresh_ms": effective_thresh_ms,
        "p_reject_clean": p_reject_clean,
        "p_admit_noise": p_admit_noise,
        "feedback_risk": feedback_risk,
        "feedback_severity": feedback_severity,
        "quality": 1.0 - p_reject_clean if p_admit_noise < 0.5 else 0.0,
    }


if __name__ == "__main__":
    analyze_kalman_wifi()
    analyze_persistence_wifi()
