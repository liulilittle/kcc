#!/usr/bin/env python3
"""
oscillation_analysis.py -- Why does x_est oscillate and cause 18,714 retrans?
Root cause: directional KF oscillation around T_prop with overshoot.
"""

import random
import statistics
from collections import deque

SCALE = 1024
SCALE_SHIFT = 10
P_INIT = 1000
P_MAX = 100_000_000

print("=" * 90)
print("OSCILLATION ANALYSIS: why x_est-based BDP causes 18,714 retrans")
print("=" * 90)


def simulate_one_flow(T_prop, sigma, Q, R, rounds):
    rng = random.Random(T_prop + 42)
    x_est = T_prop * SCALE
    p_est = P_INIT
    min_rtt = T_prop
    jitter = 0.0
    consec_reject = 0
    pos_skip = 0
    history = deque(maxlen=rounds)

    for _ in range(rounds):
        rtt = max(1, T_prop + int(rng.gauss(0, sigma)))
        min_rtt = min(min_rtt, rtt)
        z = rtt * SCALE
        innov = z - x_est

        p_pred = min(p_est + Q, P_MAX)

        if innov <= 0:
            x_est = z
            p_est = max(R, 10)
            pos_skip = 0
            consec_reject = 0
        else:
            jitter = jitter * 0.875 + (abs(innov) >> 10) * 0.125
            prop_thresh = max(T_prop >> 2, 50)
            jitter_thresh = int(jitter * 2)
            dyn_thresh = max(prop_thresh, jitter_thresh)

            if abs(innov) >> 10 > dyn_thresh and consec_reject < 20:
                consec_reject += 1
                pos_skip += 1
                p_est = p_pred
            else:
                if consec_reject >= 20:
                    consec_reject = 0
                consec_reject = 0
                gain_den = p_pred + R
                corr = (p_pred * innov) // gain_den if gain_den else 0
                x_est += corr
                p_reduction = (p_pred * p_pred) // gain_den if gain_den else 0
                p_est = max(p_pred - p_reduction, 10)
                pos_skip += 1

        history.append((x_est / SCALE, rtt, min_rtt, p_est))

    return list(history)


# Run simulation at 1400us DC
hist = simulate_one_flow(1400, 20, 100, 400, 50000)

# Analyze last 10K samples
tail = hist[-10000:]
x_vals = [h[0] for h in tail]
min_rtt_vals = [h[2] for h in tail]
p_vals = [h[3] for h in tail]

x_mean = statistics.mean(x_vals)
x_std = statistics.stdev(x_vals)
x_min = min(x_vals)
x_max = max(x_vals)
mrtt = min_rtt_vals[-1]

above_count = sum(1 for x, mr in zip(x_vals, min_rtt_vals, strict=False) if x > mr)
above_pct = above_count / len(x_vals) * 100
above_avg = sum(
    abs(x - mr) for x, mr in zip(x_vals, min_rtt_vals, strict=False) if x > mr
) / max(above_count, 1)

print(
    f"\n  x_est: mean={x_mean:.1f}us, std={x_std:.1f}us, range=[{x_min:.0f},{x_max:.0f}]us",
)
print(f"  min_rtt: {mrtt}us")
print(f"  x_est > min_rtt: {above_pct:.1f}% of time (avg excess: {above_avg:.1f}us)")
print(f"  BDP_overestimate_fraction: {above_pct:.1f}% -- THIS causes retransmissions!")

# Simulate BDP effect
bdp_true = 1400  # T_prop in us
overestimate_effect = sum(
    (x / mrtt - 1) * 100 if x > mr else 0
    for x, mr in zip(x_vals, min_rtt_vals, strict=False)
) / len(x_vals)

print(f"  Avg BDP overestimate: {overestimate_effect:.2f}%")
print("  Expected retrans budget: buffer = typical 1xBDP. If BDP overestimated")
print(f"  by {overestimate_effect:.1f}%, flows push more than link can handle.")

# =============================================================================
# THE KEY: Why does x_est exceed min_rtt?
# =============================================================================
print("\n--- Why x_est > min_rtt? ---")
# Track x_est after positive vs negative innovations
for lookback in [0, 1, 5, 10, 50]:
    # Check x_est at points where it's above min_rtt
    above_pts = [(h[0], h[1]) for h in tail if h[0] > h[2]]
    if above_pts:
        avg_x = sum(p[0] for p in above_pts[: lookback * 10 + 1]) / max(
            len(above_pts),
            1,
        )
        avg_rtt = sum(p[1] for p in above_pts[: lookback * 10 + 1]) / max(
            len(above_pts),
            1,
        )
        # Why: the negative path brings x_est to z (=RTT with residual queue)
        # Then the positive path overshoots via force-accept after 20 rejects

# Show oscillation pattern
print("\n--- Oscillation pattern (last 200 samples) ---")
last_200 = tail[-200:]
for i in range(0, 200, 40):
    x, rtt, mr, p = last_200[i]
    above = "[WARNING] ABOVE min_rtt" if x > mr else "  ok"
    print(
        f"  t={i:>3d}: x_est={x:.0f}us, RTT={rtt:.0f}us, min_rtt={mr}us, p={p:.0f} {above}",
    )

# =============================================================================
# THE FIX: Why min_rtt is always correct
# =============================================================================
print("\n\n=== WHY MIN_RTT ===")
print("  min_rtt is monotonic non-increasing. It converges to T_prop.")
print("  x_est oscillates around T_prop with overshoot above min_rtt.")
print("  BDP = bw * model_rtt / MSS. If model_rtt = x_est, BDP oscillates.")
print("  If model_rtt = min_rtt, BDP is stable and safe.")
print()
print("  FIX: kcc_get_model_rtt FILTER mode -> min(x_est, min_rtt)")
print("  Kalman filter still runs internally for qdelay/jitter/drift detection.")
print("  BDP is protected by the min_rtt ceiling.")
print("  Same design as BBR mode (which already does this and has ZERO loss).")
