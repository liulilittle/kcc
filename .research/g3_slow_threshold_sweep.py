#!/usr/bin/env python3

"""
G3 Slow Counter Threshold Sweep

Simulates the G3 slow-path cumulative counter to find the minimum safe
threshold that achieves zero false positives under H0 (pure Gaussian noise)
while minimising detection delay for +5% (required) and +3% (stretch) path changes.

Slow counter algorithm:
  - Counter  += 1  when  RTT >= baseline * 1.05   (≥5% exceedance)
  - Counter  = 0   when  RTT <= baseline * 1.01   (return to baseline)
  - No change when  baseline < RTT < baseline * 1.05
  - Alarm triggers when counter reaches threshold N
"""

import numpy as np
import time
import json
import os

# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
BASELINE = 100.0          # ms (T_prop)
SIGMA = BASELINE / 100.0  # 1 ms jitter
EXCEED_FACTOR = 1.05
RETURN_FACTOR = 1.01

EXCEED_VAL = BASELINE * EXCEED_FACTOR   # 105.0
RETURN_VAL = BASELINE * RETURN_FACTOR   # 101.0

THRESHOLDS = [3, 5, 7, 10, 15, 20, 25, 30, 40, 50]

H0_SEEDS = 10000
H0_SAMPLES = 50000

H1_SEEDS = 50
H1_SAMPLES = 20000  # generous cap; will stop early on detection

# ---------------------------------------------------------------------------
# Core slow-counter simulation functions
# ---------------------------------------------------------------------------
def slow_counter_check(rtts, threshold):
    """Return True if the counter ever reaches threshold, else False."""
    counter = 0
    for r in rtts:
        if r >= EXCEED_VAL:
            counter += 1
        elif r <= RETURN_VAL:
            counter = 0
        if counter >= threshold:
            return True
    return False


def detection_delay(rtts, threshold):
    """Return sample index (1-based) when counter first reaches threshold.

    Returns len(rtts) if never detected.
    """
    counter = 0
    for i, r in enumerate(rtts):
        if r >= EXCEED_VAL:
            counter += 1
        elif r <= RETURN_VAL:
            counter = 0
        if counter >= threshold:
            return i + 1
    return len(rtts)


# ---------------------------------------------------------------------------
# H0 false-positive test
# ---------------------------------------------------------------------------
def run_h0_test(threshold, rng):
    """Return number of seeds (out of H0_SEEDS) that produce a false alarm."""
    exceedances_needed = threshold  # minimum to possibly trigger

    fp_count = 0
    for seed_idx in range(H0_SEEDS):
        rtts = rng.normal(BASELINE, SIGMA, H0_SAMPLES)

        # Fast path: count exceedances; if fewer than threshold, impossible to trigger.
        # Under H0 (sigma=1, exceed at >=105) this filters out >98% of seeds.
        exceed_mask = rtts >= EXCEED_VAL
        n_exceed = np.sum(exceed_mask)
        if n_exceed < exceedances_needed:
            continue

        # Full sequential check for seeds with enough exceedances.
        if slow_counter_check(rtts, threshold):
            fp_count += 1
            # Early exit: any false positive means this threshold fails.
            break

        # Progress every 1000 seeds
        if (seed_idx + 1) % 1000 == 0:
            eta = (time.time() - _t0) / (seed_idx + 1) * (H0_SEEDS - seed_idx - 1)
            print(f"      [{seed_idx+1}/{H0_SEEDS}]  FP={fp_count}  ETA={eta:.0f}s")

    return fp_count


# ---------------------------------------------------------------------------
# H1 detection tests
# ---------------------------------------------------------------------------
def run_h1_test(threshold, shift_pct, rng):
    """Return list of detection delays for 50 seeds under a shift_pct path change."""
    mean = BASELINE * (1.0 + shift_pct / 100.0)
    delays = []
    for seed_idx in range(H1_SEEDS):
        rtts = rng.normal(mean, SIGMA, H1_SAMPLES)
        d = detection_delay(rtts, threshold)
        delays.append(d)
    return delays


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    print("=" * 72)
    print("  G3 Slow Counter Threshold Sweep")
    print("=" * 72)
    print()
    print("  Slow-counter algorithm:")
    print(f"    Counter += 1  when  RTT >= {EXCEED_VAL}  (≥{100*(EXCEED_FACTOR-1):.0f}% exceedance)")
    print(f"    Counter  = 0  when  RTT <= {RETURN_VAL}  (return to baseline)")
    print(f"    No change    when  BASELINE < RTT < {EXCEED_VAL}")
    print("    Alarm when counter reaches threshold N")
    print()
    print(f"  H0: {H0_SEEDS} seeds x {H0_SAMPLES} RTTs  (pure noise, sigma={SIGMA})")
    print(f"  H1: {H1_SEEDS} seeds x up to {H1_SAMPLES} RTTs  (+5%  and  +3% shift)")
    print(f"  Thresholds: {THRESHOLDS}")
    print()

    results = {}

    for thresh in THRESHOLDS:
        print(f"  {'─' * 64}")
        print(f"  Threshold N = {thresh}")
        print(f"  {'─' * 64}")

        # ---- H0 ----
        t0 = time.time()
        global _t0
        _t0 = t0
        rng = np.random.default_rng(42)
        fp = run_h0_test(thresh, rng)
        t_h0 = time.time() - t0

        passed_h0 = fp == 0
        status = "PASS" if passed_h0 else "FAIL"
        print(f"    H0:  {fp:>5}/{H0_SEEDS} false positives  [{status}]  ({t_h0:.1f}s)")

        # ---- H1 (+5%) ----
        t0 = time.time()
        rng = np.random.default_rng(123)
        delays_5 = run_h1_test(thresh, 5, rng)
        t_h1_5 = time.time() - t0
        detected_5 = sum(1 for d in delays_5 if d < H1_SAMPLES)
        median_5 = int(np.median(delays_5))
        print(f"    +5%:  {detected_5:>3}/{H1_SEEDS} detected  median delay: {median_5:>5} samples  ({t_h1_5:.1f}s)")

        # ---- H1 (+3%) ----
        t0 = time.time()
        rng = np.random.default_rng(456)
        delays_3 = run_h1_test(thresh, 3, rng)
        t_h1_3 = time.time() - t0
        detected_3 = sum(1 for d in delays_3 if d < H1_SAMPLES)
        median_3 = int(np.median(delays_3))
        print(f"    +3%:  {detected_3:>3}/{H1_SEEDS} detected  median delay: {median_3:>5} samples  ({t_h1_3:.1f}s)")

        results[thresh] = {
            "fp": int(fp),
            "fp_rate": float(fp) / H0_SEEDS,
            "passed_h0": passed_h0,
            "detected_5pct": detected_5,
            "median_delay_5pct": median_5,
            "detected_3pct": detected_3,
            "median_delay_3pct": median_3,
        }

    # -----------------------------------------------------------------------
    # Summary table
    # -----------------------------------------------------------------------
    print()
    print("  " + "=" * 72)
    print("  SUMMARY")
    print("  " + "=" * 72)
    hdr = f"  {'Thresh':>6} | {'H0 FP':>8} | {'+5% det':>8} | {'+5% med':>8} | {'+3% det':>8} | {'+3% med':>9}"
    sep = f"  {'─'*6}-+-{'─'*8}-+-{'─'*8}-+-{'─'*8}-+-{'─'*8}-+-{'─'*9}"
    print(hdr)
    print(sep)

    passed = []
    for thresh in THRESHOLDS:
        r = results[thresh]
        if r["passed_h0"]:
            passed.append(thresh)
            line = (
                f"  {thresh:>6} | {'0':>8} | "
                f"{r['detected_5pct']:>3}/{H1_SEEDS:<3} | "
                f"{r['median_delay_5pct']:>8} | "
                f"{r['detected_3pct']:>3}/{H1_SEEDS:<3} | "
                f"{r['median_delay_3pct']:>9}"
            )
        else:
            line = (
                f"  {thresh:>6} | {r['fp']:>8} | "
                f"{'--':>8} | {'--':>8} | "
                f"{'--':>8} | {'--':>9}"
            )
        print(line)

    print()
    if not passed:
        print("  WARNING: No threshold passed the H0 test!")
        print("  Consider widening the noise model or increasing the baseline.")
    else:
        # Optimal = lowest threshold that passes H0 (yields fastest detection)
        optimal = passed[0]
        r = results[optimal]
        print(f"  {'★' * 68}")
        print(f"  Optimal threshold:  {optimal}")
        print(f"  {'★' * 68}")
        print(f"    H0 false positives:  0 / {H0_SEEDS}")
        print(f"    +5% detection:        {r['detected_5pct']}/{H1_SEEDS} seeds  (median delay {r['median_delay_5pct']} samples)")
        print(f"    +3% detection:        {r['detected_3pct']}/{H1_SEEDS} seeds  (median delay {r['median_delay_3pct']} samples)")
        print()
        print(f"  Justification: Lowest threshold ({optimal}) that yields zero false")
        print("  positives under H0 noise, giving the fastest possible detection")
        print("  for both +5% and +3% path changes. No higher threshold improves")
        print("  H0 safety (already zero), and every higher threshold strictly")
        print("  increases median detection delay.")

    # -----------------------------------------------------------------------
    # Save results to JSON
    # -----------------------------------------------------------------------
    out = {
        "parameters": {
            "baseline_ms": BASELINE,
            "sigma_ms": SIGMA,
            "exceed_factor": EXCEED_FACTOR,
            "return_factor": RETURN_FACTOR,
            "h0_seeds": H0_SEEDS,
            "h0_samples": H0_SAMPLES,
            "h1_seeds": H1_SEEDS,
            "h1_samples": H1_SAMPLES,
        },
        "results": {str(k): v for k, v in results.items()},
        "optimal_threshold": optimal if passed else None,
    }

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(script_dir, "g3_slow_results.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2, default=str)
    print(f"\n  Results saved to: {out_path}")
    print()


if __name__ == "__main__":
    main()
