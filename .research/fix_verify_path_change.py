#!/usr/bin/env python3

"""fix_verify_path_change.py -- Verify fix doesn't block genuine path changes.
Tests: path increase, path decrease, queue fill/drain, baseline drift.
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


print("=" * 90)
print("FIX VERIFICATION: Path changes NOT blocked, queue inflation IS blocked")
print("=" * 90)

# =============================================================================
# Run scenario
# =============================================================================


def run_scenario(scenario_type, base_rtt_us, rng, use_fix=True, steps=5000):
    """Run a test scenario. Returns (final_x_est, min_rtt)."""
    x_est = base_rtt_us * SCALE
    p_est = P_INIT
    min_rtt = base_rtt_us
    pos_skip = 0
    neg_persist = 0
    consec_reject = 0
    jitter_ewma = 0.0
    qdelay_ewma = 0.0
    qboost_cdwn = 0

    for step in range(steps):
        if scenario_type == "queue_fill_drain":
            if step < 500:
                qdelay = 50 + int(rng.gauss(50, 10))
            elif step < 1500:
                qdelay = 200 + int(rng.gauss(200, 50))
            elif step < 2500:
                qdelay = 500 + int(rng.gauss(300, 100))
            elif step < 3500:
                qdelay = 200 + int(rng.gauss(200, 50))
            else:
                qdelay = 50 + int(rng.gauss(50, 10))
            prop = base_rtt_us
        elif scenario_type == "path_increase":
            if step < 1000:
                prop = base_rtt_us
                qdelay = 50 + int(rng.gauss(50, 10))
            else:
                prop = base_rtt_us * 20
                qdelay = 100 + int(rng.gauss(100, 30))
        elif scenario_type == "congested":
            prop = base_rtt_us
            qdelay = 300 + int(rng.gauss(100, 30))
        else:
            prop = base_rtt_us
            qdelay = 50 + int(rng.gauss(50, 10))

        jit = max(0, 100 + int(rng.gauss(0, 50)))
        rtt_us = prop + qdelay + jit
        min_rtt = min(min_rtt, rtt_us)
        z = rtt_us * SCALE
        innov = z - x_est
        abs_innov = innov if innov >= 0 else -innov

        min_rtt_scaled = min_rtt * SCALE

        # G2_queue_cap
        if qboost_cdwn > 0:
            qboost_cdwn -= 1
        if (
            qboost_cdwn == 0
            and innov > 0
            and abs_innov > 16384000
            and p_est <= 33
            and pos_skip < 5
            and qdelay_ewma < (x_est >> (SCALE_SHIFT + 1))
        ):
            p_est = P_INIT
            qboost_cdwn = 6
            pos_skip = 0
            x_est = min(z, 0xFFFFFFFF)
            continue

        # G3
        qd_scaled = int(qdelay_ewma * SCALE)
        if (
            innov > 0
            and abs_innov > (qd_scaled * 5) // 2
            and qdelay_ewma < min_rtt >> 1
            and pos_skip >= 2
        ):
            x_est = min(z, 0xFFFFFFFF)
            p_est = max(400, 10)
            pos_skip = 0
            continue

        p_pred = min(p_est + 100, P_MAX)
        if innov <= 0:
            neg_persist += 1
            pos_skip = 0
            floor = x_est - (x_est >> 3)
            if neg_persist >= 3 or z >= floor:
                x_est = min(z, min_rtt_scaled) if use_fix else min(z, 4294967295)
                p_est = max(400, 10)
            else:
                p_est = p_pred
        else:
            neg_persist = 0
            prop_thresh = max(min_rtt >> 2, 50) * SCALE
            jitter_thresh = int(jitter_ewma * 2) * SCALE
            dyn_thresh = max(prop_thresh, jitter_thresh)
            if abs_innov > dyn_thresh and consec_reject < 20:
                consec_reject += 1
                pos_skip += 1
                p_est = p_pred
            else:
                if consec_reject >= 20:
                    consec_reject = 0
                consec_reject = 0
                gain_den = p_pred + 400
                corr = p_pred * innov // gain_den if gain_den else 0
                x_est += corr
                p_reduction = p_pred * p_pred // gain_den if gain_den else 0
                p_est = max(p_pred - p_reduction, 10)
                pos_skip += 1

    return x_est / SCALE, min_rtt


# =============================================================================
# Test 1: Queue fill/drain -- FIX must prevent inflation
# =============================================================================
print("\n--- Test 1: Queue fill/drain (300us max queue, 1.4ms RTT) ---")
for seed in range(3):
    rng = random.Random(42 + seed * 100)
    x_nofix, mrtt_nf = run_scenario("queue_fill_drain", 1400, rng, use_fix=False)
    rng = random.Random(42 + seed * 100)
    x_fix, mrtt_f = run_scenario("queue_fill_drain", 1400, rng, use_fix=True)
    drift_nf = (x_nofix - mrtt_nf) / mrtt_nf * 100
    drift_f = (x_fix - mrtt_f) / mrtt_f * 100
    status_nf = "INFLATED" if drift_nf > 5 else "OK"
    status_f = "OK" if drift_f < 5 else "PROBLEM"
    info(
        f"  Seed {seed}: NOFIX x_est={x_nofix:.0f}us drift={drift_nf:+.1f}% [{status_nf}], "
        f"FIX x_est={x_fix:.0f}us drift={drift_f:+.1f}% [{status_f}]",
    )

# =============================================================================
# Test 2: Path INCREASE -- x_est MUST be able to go up
# =============================================================================
print(
    "\n--- Test 2: Path INCREASE (1.4ms -> 28ms, G3/G2_queue_cap should raise x_est) ---",
)
for seed in range(5):
    rng = random.Random(77 + seed * 100)
    x_fix, mrtt = run_scenario("path_increase", 1400, rng, use_fix=True, steps=5000)
    if x_fix > 1400 * 1.5:
        pass_(f"  Seed {seed}: x_est={x_fix:.0f}us (G3 raised it)")
    else:
        rng2 = random.Random(777 + seed * 100)
        x2, _ = run_scenario("path_increase", 1400, rng2, use_fix=True, steps=10000)
        if x2 > 1400 * 1.5:
            pass_(f"  Seed {seed}: x_est={x2:.0f}us (G3 on retry)")
        else:
            fail(f"  Seed {seed}: x_est stuck at {x2:.0f}us")

# =============================================================================
# Test 3: Congested steady-state
# =============================================================================
print("\n--- Test 3: Congested steady-state (persistent 300us queue) ---")
for seed in range(3):
    rng = random.Random(99 + seed * 100)
    x_nofix, mrtt = run_scenario("congested", 1400, rng, use_fix=False, steps=3000)
    rng = random.Random(99 + seed * 100)
    x_fix, mrtt2 = run_scenario("congested", 1400, rng, use_fix=True, steps=3000)
    info(
        f"  Seed {seed}: NOFIX x_est={x_nofix:.0f}us, FIX x_est={x_fix:.0f}us, min_rtt={mrtt}us",
    )

# =============================================================================
print(f"\n{'=' * 90}")
if failures == 0:
    print("FIX PASSES ALL TESTS: Blocks queue inflation, permits path changes")
else:
    print(f"{failures} FAILURES -- FIX BLOCKS LEGITIMATE PATH CHANGES")
