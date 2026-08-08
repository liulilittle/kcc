#!/usr/bin/env python3
"""
floor_deadlock_analysis.py -- Root cause: x_est deadlock above min_rtt.
Once x_est exceeds min_rtt by >12.5%, the speed-of-light floor
permanently blocks downward convergence. Every sample gets rejected.
Fix: hard-cap x_est at min_rtt after every update.
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

SCALE = 1024
SCALE_SHIFT = 10
P_INIT = 1000
P_MAX = 100_000_000

print("=" * 90)
print("SPEED-OF-LIGHT FLOOR DEADLOCK ANALYSIS")
print("=" * 90)

# =============================================================================
# 1. Demonstrate the deadlock
# =============================================================================
print("\n--- 1. Deadlock: x_est stuck above min_rtt forever ---")


def run_floor_test(
    x_start_us,
    true_rtt_us,
    floor_shift,
    sigma_us,
    steps,
    cap_at_min_rtt=False,
):
    x_est = x_start_us * SCALE
    p_est = P_INIT
    rejects = 0
    accepts = 0
    rng = random.Random(42)
    min_rtt_us = true_rtt_us  # min_rtt known

    for _ in range(steps):
        rtt_us = max(1, true_rtt_us + int(rng.gauss(0, sigma_us)))
        z = rtt_us * SCALE
        innov = z - x_est

        if cap_at_min_rtt:
            # FIX: hard cap
            min_rtt_scaled = min_rtt_us * SCALE
            if x_est > min_rtt_scaled:
                x_est = min_rtt_scaled
                p_est = max(400, 10)
                # G3_trigger_convergencence: x_est = min_rtt
                continue

        p_pred = min(p_est + 100, P_MAX)

        if innov <= 0:
            floor = x_est - (x_est >> floor_shift)
            if z >= floor:
                x_est = min(z, 0xFFFFFFFF)
                p_est = max(400, 10)
                accepts += 1
            else:
                p_est = p_pred
                rejects += 1
        else:
            gain_den = p_pred + 400
            corr = (p_pred * innov) // gain_den if gain_den else 0
            x_est = min(x_est + corr, 0xFFFFFFFF)
            p_reduction = (p_pred * p_pred) // gain_den if gain_den else 0
            p_est = max(p_pred - p_reduction, 10)
            accepts += 1

    return x_est / SCALE, rejects, accepts


for initial_offset_pct in [15, 30, 50, 100, 450]:
    x_start = 1400 + int(1400 * initial_offset_pct / 100)
    final_no_fix, rej, acc = run_floor_test(x_start, 1400, 3, 5, 2000)
    final_fix, rej_f, acc_f = run_floor_test(
        x_start,
        1400,
        3,
        5,
        2000,
        cap_at_min_rtt=True,
    )
    status_nofix = "STUCK" if abs(final_no_fix - 1400) > 50 else "CONVERGED"
    status_fix = "STUCK" if abs(final_fix - 1400) > 50 else "CONVERGED"
    print(
        f"  Offset={initial_offset_pct:>3d}% (x0={x_start}us): No_fix={final_no_fix:.0f}us [{status_nofix}], Fix={final_fix:.0f}us [{status_fix}]",
    )

# =============================================================================
# 2. Critical threshold analysis
# =============================================================================
print("\n--- 2. Critical threshold: when does deadlock begin? ---")

for floor_shift in [3, 4, 5, 6, 7, 8]:
    max_drop_allowed = 100 / (2**floor_shift)
    # If x_est - true_rtt > max_drop_allowed% of x_est, deadlock ensues
    # x_est > true_rtt * (1 + max_drop_allowed/100)
    for true_rtt in [1400, 50000]:
        min_x_for_deadlock = true_rtt * (1 + max_drop_allowed / 100)
        print(
            f"  shift={floor_shift} ({max_drop_allowed:.1f}% max drop): deadlock at x_est >= {min_x_for_deadlock:.0f}us"
            f" for true_rtt={true_rtt}us",
        )

# =============================================================================
# 3. How does x_est normally exceed min_rtt? Upward drift simulation
# =============================================================================
print("\n--- 3. How x_est drifts above min_rtt (multi-flow queue scenario) ---")


def run_queue_fill_drain(rtt_base, queue_max_us, floor_shift, steps):
    """Simulate queue building up then clearing, observe x_est"""
    x_est = rtt_base * SCALE
    p_est = P_INIT
    min_rtt = rtt_base
    history = []
    rng = random.Random(12345)

    # Phase 1: Queue builds up (congestion)
    for i in range(steps // 2):
        queue = int(
            queue_max_us * (0.5 + 0.5 * math.sin(i * 2 * math.pi / (steps / 4))),
        )
        queue = max(0, queue)
        rtt_us = max(1, rtt_base + queue + int(rng.gauss(0, rtt_base * 0.01)))
        z = rtt_us * SCALE
        innov = z - x_est
        p_pred = min(p_est + 100, P_MAX)
        min_rtt = min(min_rtt, rtt_us)

        if innov <= 0:
            floor = x_est - (x_est >> floor_shift)
            if z >= floor:
                x_est = min(z, 0xFFFFFFFF)
                p_est = max(400, 10)
            else:
                p_est = p_pred
        else:
            gain_den = p_pred + 400
            corr = (p_pred * innov) // gain_den if gain_den else 0
            x_est = min(x_est + corr, 0xFFFFFFFF)
            p_reduction = (p_pred * p_pred) // gain_den if gain_den else 0
            p_est = max(p_pred - p_reduction, 10)
        history.append((x_est / SCALE, rtt_us, min_rtt, queue))

    return history, min_rtt


hist, mrtt = run_queue_fill_drain(1400, 1000, 3, 2000)
final_x = hist[-1][0]
drift_pct = (final_x - mrtt) / mrtt * 100
print(
    f"  After queue fill/drain: x_est={final_x:.0f}us, min_rtt={mrtt}us, drift={drift_pct:+.1f}%",
)

# Show time series
for t in [0, 200, 400, 600, 800, 999]:
    x, rtt, mr, q = hist[t]
    print(
        f"    t={t:>4d}: x_est={x:.0f}us, RTT={rtt:.0f}us, min_rtt={mr}us, queue={q}us",
    )

# =============================================================================
# 4. Verify fix: cap x_est at min_rtt after every update
# =============================================================================
print("\n--- 4. FIX VERIFICATION: cap x_est at min_rtt after every positive update ---")


def run_with_fix(rtt_base, queue_max_us, floor_shift, steps, do_cap=True):
    x_est = rtt_base * SCALE
    p_est = P_INIT
    min_rtt = rtt_base
    rng = random.Random(67890)

    for i in range(steps):
        queue = int(
            queue_max_us * (0.5 + 0.5 * math.sin(i * 2 * math.pi / (steps / 4))),
        )
        queue = max(0, queue)
        rtt_us = max(1, rtt_base + queue + int(rng.gauss(0, rtt_base * 0.01)))
        z = rtt_us * SCALE
        innov = z - x_est
        p_pred = min(p_est + 100, P_MAX)
        min_rtt = min(min_rtt, rtt_us)

        if innov <= 0:
            floor = x_est - (x_est >> floor_shift)
            if z >= floor:
                x_est = min(z, 0xFFFFFFFF)
                p_est = max(400, 10)
            else:
                p_est = p_pred
        else:
            gain_den = p_pred + 400
            corr = (p_pred * innov) // gain_den if gain_den else 0
            x_est = min(x_est + corr, 0xFFFFFFFF)
            p_reduction = (p_pred * p_pred) // gain_den if gain_den else 0
            p_est = max(p_pred - p_reduction, 10)

            if do_cap:
                min_rtt_scaled = min_rtt * SCALE
                x_est = min(x_est, min_rtt_scaled)

    return x_est / SCALE, min_rtt


final_nofix, mrtt_nf = run_with_fix(1400, 1000, 3, 2000, do_cap=False)
final_fix, mrtt_f = run_with_fix(1400, 1000, 3, 2000, do_cap=True)

print(
    f"  Without fix: x_est={final_nofix:.0f}us, min_rtt={mrtt_nf}us, drift={(final_nofix - mrtt_nf) / mrtt_nf * 100:+.1f}%",
)
print(
    f"  With fix:    x_est={final_fix:.0f}us, min_rtt={mrtt_f}us, drift={(final_fix - mrtt_f) / mrtt_f * 100:+.1f}%",
)

# =============================================================================
# 5. Impact on multi-flow fairness
# =============================================================================
print("\n--- 5. Multi-flow fairness with min_rtt cap ---")
# Simplified: 8 flows, shared bottleneck, some flows get queue advantage


def fairness_test(N, rtt_base, bottle_bps, use_cap=False):
    SCALE = 1024
    flows = []
    for i in range(N):
        f = {
            "x_est": rtt_base * SCALE,
            "p_est": 1000,
            "min_rtt": rtt_base,
            "total_bytes": 0,
            "cwnd_bytes": 10000,  # start small
            "pacing": bottle_bps / N,
            "id": i,
        }
        flows.append(f)

    rng = random.Random(42)
    queue_bytes = 0
    MSS = 1500
    total_time = 5.0  # seconds
    step_us = 1000  # 1ms steps
    steps = int(total_time * 1e6 / step_us)

    for step in range(steps):
        # Each flow sends
        total_sent = 0
        for f in flows:
            cwnd = f["cwnd_bytes"]
            pacing_send = f["pacing"] * step_us / 1e6  # bytes in this step
            f["total_bytes"]  # simplified
            can_send = max(0, min(pacing_send, cwnd))
            total_sent += can_send
            f["total_bytes"] += can_send

        # Bottleneck service
        service = bottle_bps * step_us / 1e6
        delivered = min(total_sent, service + queue_bytes)
        queue_bytes = max(0, queue_bytes + total_sent - delivered)

        # ACK processing
        for f in flows:
            pacing_send = f["pacing"] * step_us / 1e6

            queue_us = queue_bytes / (bottle_bps / 8) * 1e6
            rtt_us = max(1, rtt_base + int(queue_us) + rng.randint(-5, 5))
            f["min_rtt"] = min(f["min_rtt"], rtt_us)

            # KCC update (simplified)
            z = rtt_us * SCALE
            innov = z - f["x_est"]
            p_pred = min(f["p_est"] + 100, P_MAX)

            if innov <= 0:
                floor = f["x_est"] - (f["x_est"] >> 3)
                if z >= floor:
                    f["x_est"] = min(z, 0xFFFFFFFF)
                    f["p_est"] = max(400, 10)
                else:
                    f["p_est"] = p_pred
            else:
                gain_den = p_pred + 400
                corr = (p_pred * innov) // gain_den if gain_den else 0
                f["x_est"] = min(f["x_est"] + corr, 0xFFFFFFFF)
                p_reduction = (p_pred * p_pred) // gain_den if gain_den else 0
                f["p_est"] = max(p_pred - p_reduction, 10)

                if use_cap:
                    cap = f["min_rtt"] * SCALE
                    f["x_est"] = min(f["x_est"], cap)

        # Update cwnd every 10ms
        if step % 10 == 0:
            for f in flows:
                t_prop_est = f["x_est"] // SCALE
                bdp = int(bottle_bps / 8 * t_prop_est / 1e6)
                f["cwnd_bytes"] = max(4 * MSS, int(bdp * 1.25))
                f["pacing"] = bottle_bps * 0.9

    # Compute Jain fairness
    tputs = [f["total_bytes"] * 8 / total_time / 1e6 for f in flows]
    n = len(tputs)
    sum_t = sum(tputs)
    sum_sq = sum(t * t for t in tputs)
    ji = (sum_t * sum_t) / (n * sum_sq) if sum_sq > 0 else 1
    return ji, tputs, [(f["x_est"] // SCALE, f["min_rtt"]) for f in flows]


for N_test in [8, 16]:
    ji_nocap, tp_nocap, xs_nocap = fairness_test(N_test, 1400, 1e9, use_cap=False)
    ji_cap, tp_cap, xs_cap = fairness_test(N_test, 1400, 1e9, use_cap=True)
    x_nocap = [x for x, _ in xs_nocap]
    min_nocap = [m for _, m in xs_nocap]
    x_cap = [x for x, _ in xs_cap]

    print(
        f"  N={N_test}: WITHOUT cap -- JI={ji_nocap:.4f}, x_est range=[{min(x_nocap):.0f},{max(x_nocap):.0f}]us",
    )
    print(
        f"           WITH cap    -- JI={ji_cap:.4f}, x_est range=[{min(x_cap):.0f},{max(x_cap):.0f}]us",
    )
    if ji_cap >= ji_nocap:
        print(
            f"           -> Cap {'maintains' if abs(ji_cap - ji_nocap) < 0.01 else 'IMPROVES'} fairness",
        )

# =============================================================================
print(f"\n{'=' * 90}")
print("FIX RECOMMENDATION: Add x_est <= min_rtt_us cap after every positive update")
print("Location: tcp_kcc.c after line ~12908 (after x_est update)")
print("Change:  if (ext->x_est > (u64)kcc->min_rtt_us << kalman_scale_shift)")
print("             ext->x_est = (u32)((u64)kcc->min_rtt_us << kalman_scale_shift);")
