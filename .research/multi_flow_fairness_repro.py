#!/usr/bin/env python3
"""
multi_flow_fairness_repro.py -- Reproduce the 10-flow fairness problem.
Simulates N flows sharing a bottleneck link with KCC-like behavior.
Measures fairness (Jain index), x_est drift, and throughput distribution.
Compares: current code (x_est) vs MIN_RTT fix.
"""

import os
import random
import sys

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


class KCCFlow:
    def __init__(self, flow_id, rtt_base_us, bottleneck_bw_mbps, noise_sigma_us):
        self.id = flow_id
        self.T_prop = rtt_base_us  # true propagation delay
        self.min_rtt_us = rtt_base_us  # min RTT seen
        self.x_est = rtt_base_us * SCALE  # Kalman estimate of T_prop
        self.p_est = P_INIT
        self.Q = 100
        self.R = 400
        self.sigma = noise_sigma_us
        self.cwnd_bytes = bottleneck_bw_mbps * 125000 * rtt_base_us / 1e6  # initial BDP
        self.cwnd_bytes = max(self.cwnd_bytes, 4 * 1500)
        self.pacing_rate_bps = bottleneck_bw_mbps * 125000
        self.pos_skip = 0
        self.jitter_ewma = 0.0
        self.qdelay_ewma = 0.0
        self.total_bytes = 0
        self.total_retrans = 0
        self.qboost_cdwn = 0
        self.consec_reject = 0

    def get_cwnd_packets(self, MSS=1500):
        return max(4, self.cwnd_bytes // MSS)

    def get_pacing_rate(self):
        return self.pacing_rate_bps

    def update_on_ack(self, rtt_us, use_min_rtt=False):
        """KCC update on ACK receipt. If use_min_rtt=True, x_est = min_rtt always."""
        self.min_rtt_us = min(self.min_rtt_us, rtt_us)

        if use_min_rtt:
            # MIN_RTT fix: force x_est = min_rtt
            self.x_est = self.min_rtt_us * SCALE
            self.p_est = max(self.R, 10)
            return

        z = rtt_us << SCALE_SHIFT
        innov = z - self.x_est

        # G2_queue_cap
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
            self.x_est = min(z, 0xFFFFFFFF)
            return

        # G3
        qd_scaled = int(self.qdelay_ewma * SCALE)
        if (
            innov > 0
            and abs(innov) > (qd_scaled * 5) // 2
            and self.qdelay_ewma < self.min_rtt_us >> 1
            and self.pos_skip >= 2
        ):
            self.x_est = min(z, 0xFFFFFFFF)
            self.p_est = max(self.R, 10)
            self.pos_skip = 0
            return

        p_pred = min(self.p_est + self.Q, P_MAX)

        if innov <= 0:
            # NEGATIVE: G3-detect convergence with speed-of-light floor
            self.pos_skip = 0
            floor = self.x_est - (self.x_est >> 3)  # 12.5% floor -- THE CRITICAL LINE
            if z >= floor:
                self.x_est = min(z, 0xFFFFFFFF)
                self.p_est = max(self.R, 10)
            else:
                self.p_est = p_pred  # REJECTED: x_est stays high
        else:
            # POSITIVE: standard Kalman (with outlier gate)
            prop_thresh = max(self.min_rtt_us >> 2, 50) * SCALE
            jitter_thresh = int(self.jitter_ewma * 2) * SCALE
            dyn_thresh = max(prop_thresh, jitter_thresh)

            if abs(innov) > dyn_thresh and self.consec_reject < 20:
                self.consec_reject += 1
                self.pos_skip += 1
                self.p_est = p_pred
            else:
                if self.consec_reject >= 20:
                    self.consec_reject = 0
                self.consec_reject = 0
                gain_den = p_pred + self.R
                corr = (p_pred * innov) // gain_den if gain_den else 0
                self.x_est = min(self.x_est + corr, 0xFFFFFFFF)
                p_reduction = (p_pred * p_pred) // gain_den if gain_den else 0
                self.p_est = max(p_pred - p_reduction, 10)
                self.pos_skip += 1

        # Update EWMA
        raw_jitter = abs(innov) >> SCALE_SHIFT
        self.jitter_ewma = self.jitter_ewma * 0.875 + raw_jitter * 0.125
        self.qdelay_ewma = (
            self.qdelay_ewma * 0.875 + max(0, rtt_us - self.T_prop) * 0.125
        )


def simulate_bottleneck(
    N_flows,
    rtt_base_us,
    bottleneck_bps,
    duration_s=10,
    use_min_rtt=False,
):
    """Simulate N flows sharing a bottleneck link."""
    bw_per_flow_mbps = bottleneck_bps / 1e6 / N_flows
    flows = [
        KCCFlow(i, rtt_base_us, bw_per_flow_mbps, rtt_base_us * 0.01)
        for i in range(N_flows)
    ]

    MSS = 1500  # bytes
    int(bottleneck_bps * rtt_base_us / 1e6 / 8)
    queue_bytes = 0  # shared bottleneck queue

    step_us = 100  # 100us time step (10K steps/sec)
    total_steps = duration_s * 1_000_000 // step_us
    CWND_UPDATE_INTERVAL_US = 10_000  # update cwnd every 10ms

    rng = random.Random(42)

    for step in range(total_steps):
        elapsed_us = step * step_us

        # Each flow sends at its pacing rate
        total_sent_bytes = 0
        for f in flows:
            # Bytes this flow wants to send in this 100us step
            pacing_bytes = f.get_pacing_rate() * step_us / 1e6
            cwnd_bytes = f.cwnd_bytes
            min(f.total_bytes, total_sent_bytes)  # simplified
            can_send = max(0, min(pacing_bytes, cwnd_bytes))
            total_sent_bytes += can_send
            f.total_bytes += can_send

        # Shared bottleneck: service rate = bottleneck_bps
        service_bytes = bottleneck_bps * step_us / 1e6
        delivered = min(total_sent_bytes, service_bytes + queue_bytes)
        queue_bytes = max(0, queue_bytes + total_sent_bytes - delivered)

        # Distribute delivered bytes among flows (FIFO approximation)
        if total_sent_bytes > 0:
            for f in flows:
                pacing_bytes = f.get_pacing_rate() * step_us / 1e6
                delivered * pacing_bytes / total_sent_bytes if total_sent_bytes > 0 else 0

                # Queue delay for this flow
                queue_delay_us = queue_bytes / (bottleneck_bps / 8) * 1e6  # bytes -> us
                queue_noise = rng.gauss(0, f.sigma) * 0.1

                # RTT experienced by this flow
                rtt_experienced = max(
                    1,
                    f.T_prop + int(queue_delay_us) + int(queue_noise),
                )

                # KCC update on each ACK
                f.update_on_ack(rtt_experienced, use_min_rtt=use_min_rtt)

        # Update cwnd periodically (BBR-like: cwnd = BDP with gain)
        if elapsed_us % CWND_UPDATE_INTERVAL_US < step_us:
            for f in flows:
                # BDP = bandwidth * min_rtt (or x_est)
                t_prop_est = f.x_est // SCALE  # Kalman estimate
                bdp_est = int(bottleneck_bps / 8 * t_prop_est / 1e6)
                # Pacing rate = bandwidth * pacing_gain
                f.pacing_rate_bps = int(bottleneck_bps * 0.9)  # 90% for stability

                # cwnd = BDP estimate (with headroom)
                f.cwnd_bytes = max(4 * MSS, int(bdp_est * 1.25))

    # Collect results
    throughputs = [f.total_bytes * 8 / duration_s / 1e6 for f in flows]  # Mbps
    x_est_final = [f.x_est / SCALE for f in flows]
    min_rtt_final = [f.min_rtt_us for f in flows]

    return throughputs, x_est_final, min_rtt_final


def jain_index(values):
    n = len(values)
    if n <= 1:
        return 1.0
    sum_sq = sum(v * v for v in values)
    sum_sq_total = sum(values) ** 2
    if sum_sq == 0:
        return 1.0
    return sum_sq_total / (n * sum_sq)


print("=" * 90)
print("MULTI-FLOW FAIRNESS REPRODUCTION: 10 flows, 1Gbps bottleneck")
print("=" * 90)

N = 10
RTT_BASE = 1400  # us (DC)
BOTTLENECK_BPS = 1_000_000_000  # 1 Gbps

# ---- Test 1: Current KCC code (x_est based) ----
print("\n--- Test 1: Current code (Kalman x_est) ---")
tputs_kf, x_est_kf, mrtt_kf = simulate_bottleneck(
    N,
    RTT_BASE,
    BOTTLENECK_BPS,
    duration_s=10,
)
ji_kf = jain_index(tputs_kf)
min_tput = min(tputs_kf)
max_tput = max(tputs_kf)
ratio = max_tput / max(min_tput, 0.001)
x_drift = [(x - RTT_BASE) / RTT_BASE * 100 for x in x_est_kf]
info(f"  Throughputs (Mbps): {[f'{t:.1f}' for t in tputs_kf]}")
info(f"  Jain fairness: {ji_kf:.4f}")
info(f"  Max/min ratio: {ratio:.2f}x")
info(f"  x_est drift from T_prop: {[f'{d:+.1f}%' for d in x_drift]}")
info(f"  x_est values (us): {[f'{x:.0f}' for x in x_est_kf]}")

# ---- Test 2: MIN_RTT fix ----
print("\n--- Test 2: MIN_RTT fix (x_est = min_rtt) ---")
tputs_mr, x_est_mr, mrtt_mr = simulate_bottleneck(
    N,
    RTT_BASE,
    BOTTLENECK_BPS,
    duration_s=10,
    use_min_rtt=True,
)
ji_mr = jain_index(tputs_mr)
min_tput_mr = min(tputs_mr)
max_tput_mr = max(tputs_mr)
ratio_mr = max_tput_mr / max(min_tput_mr, 0.001)
info(f"  Throughputs (Mbps): {[f'{t:.1f}' for t in tputs_mr]}")
info(f"  Jain fairness: {ji_mr:.4f}")
info(f"  Max/min ratio: {ratio_mr:.2f}x")

# ---- Analysis ----
print("\n--- Analysis ---")
if ji_mr > ji_kf:
    improvement = (ji_mr - ji_kf) * 100
    info(f"  MIN_RTT improves Jain fairness by +{improvement:.1f} percentage points")
    info(f"  Unfairness ratio improves from {ratio:.2f}x -> {ratio_mr:.2f}x")
    pass_("MIN_RTT fix confirmed effective for fairness")
else:
    info(f"  Both approaches similar (JI: {ji_kf:.4f} vs {ji_mr:.4f})")

# Check x_est drift above min_rtt
for flow_idx in range(N):
    drift = x_est_kf[flow_idx] - mrtt_kf[flow_idx]
    if drift > RTT_BASE * 0.1:
        info(
            f"  Flow {flow_idx}: x_est={x_est_kf[flow_idx]:.0f}us, min_rtt={mrtt_kf[flow_idx]:.0f}us, drift=+{drift:.0f}us (+{drift / RTT_BASE * 100:.1f}%)",
        )

# =============================================================================
# Root cause: floor gate prevents downward convergence
# =============================================================================
print("\n--- Root cause: speed-of-light floor prevents rapid downward tracking ---")
# Simulate a single flow where queue suddenly clears
RTT_BASE = 1400
for floor_shift in [3, 4, 5, 6]:
    x_est = 2000 * SCALE  # started at 2ms (T_prop=1400 + queue=600)
    p_est = 1000
    floor_rejects = 0
    rng = random.Random(42)
    for _ in range(1000):
        # Queue cleared: RTT = T_prop + noise
        rtt = max(1, RTT_BASE + int(rng.gauss(0, 5)))
        z = rtt * SCALE
        innov = z - x_est
        if innov <= 0:
            floor = x_est - (x_est >> floor_shift)
            if z >= floor:
                x_est = z
                p_est = max(400, 10)
            else:
                floor_rejects += 1
                p_est = min(p_est + 100, P_MAX)
        else:
            # positive innov
            p_pred = min(p_est + 100, P_MAX)
            corr = (p_pred * innov) // (p_pred + 400)
            x_est = min(x_est + corr, 0xFFFFFFFF)
            p_reduction = (p_pred * p_pred) // (p_pred + 400)
            p_est = max(p_pred - p_reduction, 10)

    final_x = x_est / SCALE
    info(
        f"  floor_shift={floor_shift} ({100 >> floor_shift}%): final_x={final_x:.0f}us, floor_rejects={floor_rejects}"
        f" -> {'CONVERGED' if abs(final_x - RTT_BASE) < 50 else 'DIVERGED'}",
    )

# =============================================================================
print(f"\n{'=' * 90}")
print("FAIRNESS ANALYSIS COMPLETE -- see results above for recommended fix")
