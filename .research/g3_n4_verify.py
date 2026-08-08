#!/usr/bin/env python3
"""
G3 N=4 Slow Counter Verification
=================================
Test false-positive rate and detection delay with confirm_slow_cnt >= 4.

Previous N=3 reference:
  - FP rate (H0, 2000 seeds x 50000 RTTs): ~0%
  - Detection delay (+5% path increase): ~3 RTTs
"""
import random, math, sys
import time as tm

SCALE = 256
SHIFT = 8
T_PROP = 10000  # us
JITTER_SIGMA = T_PROP // 100  # 1% = 100us

SLOW_N = 4      # new threshold
FAST_TH_NUM = 11
FAST_TH_DEN = 10
SLOW_TH_NUM = 21
SLOW_TH_DEN = 20


class Geodesic:
    def __init__(self, tprop=T_PROP):
        self.x = tprop * SCALE       # scaled estimate
        self.mr_us = tprop           # min_rtt_us (unscaled, like kcc->min_rtt_us)
        self.cnf = 0
        self.csl = 0

    def update(self, z):
        """Process one RTT sample through G1/G2/G3."""
        zs = int(z * SCALE)  # scale up

        # G1/G2
        if zs <= self.x:
            self.x = zs  # G1 instant min
        else:
            raw = self.x + (self.x * 122 // 1000)
            self.x = raw if raw < zs else zs  # G2 capped

        # G3 — compare x_est against mr_us * SCALE
        mr_s = self.mr_us * SCALE
        if self.x >= mr_s * FAST_TH_NUM // FAST_TH_DEN:
            self.cnf += 1
            self.csl += 1
        elif self.x >= mr_s * SLOW_TH_NUM // SLOW_TH_DEN:
            self.cnf = 0
            self.csl += 1
        elif self.x <= mr_s:
            self.cnf = 0
            self.csl = 0
        else:
            self.cnf = 0

        if self.cnf >= 4:
            self.mr_us = self.x >> SHIFT
            self.cnf = 0
            self.csl = 0
        elif self.csl >= SLOW_N:
            self.mr_us = self.x >> SHIFT
            self.cnf = 0
            self.csl = 0

        return self.csl


def h0_trial(seed, n_rtt=50000):
    """No path change - measure false positive rate."""
    rng = random.Random(seed)
    g = Geodesic()
    for _ in range(n_rtt):
        noise = rng.gauss(0, JITTER_SIGMA)
        z = T_PROP + noise
        if z < 1:
            z = 1
        g.update(z)
    return g.mr_us != T_PROP  # False positive if mr_us changed


def detection_trial(seed, path_increase_pct=0.05, n_rtt=50000):
    """Measure detection delay for a path increase."""
    rng = random.Random(seed)
    g = Geodesic()

    # Phase 1: settle at baseline
    for _ in range(1000):
        noise = rng.gauss(0, JITTER_SIGMA)
        z = T_PROP + noise
        if z < 1:
            z = 1
        g.update(z)

    # Phase 2: path increase
    new_tprop = int(T_PROP * (1 + path_increase_pct))
    mr_before = g.mr_us
    detect_rtt = None
    for i in range(n_rtt):
        noise = rng.gauss(0, JITTER_SIGMA)
        z = new_tprop + noise
        if z < 1:
            z = 1
        g.update(z)
        if g.mr_us > mr_before and detect_rtt is None:
            detect_rtt = i + 1
            break

    return detect_rtt


def main():
    print("=" * 60)
    print("G3 N=4 Slow Counter Verification")
    print("=" * 60)

    total_t0 = tm.time()

    # Test 1: False-positive rate (H0, no path change)
    print("\n--- Test 1: FP Rate (H0, no path change) ---")
    n_seeds_h0 = 2000
    n_rtt_h0 = 50000
    fp_count = 0
    t0 = tm.time()
    for s in range(n_seeds_h0):
        if h0_trial(s, n_rtt_h0):
            fp_count += 1
        if (s + 1) % 200 == 0:
            elapsed = tm.time() - t0
            done_pct = (s + 1.0) / n_seeds_h0 * 100
            print("  Progress: {}/{} ({:.0f}%), FP so far: {}, elapsed: {:.1f}s".format(
                s + 1, n_seeds_h0, done_pct, fp_count, elapsed))

    fp_rate = float(fp_count) / n_seeds_h0
    print("  H0 FP rate (N=4): {}/{} = {:.6f}".format(fp_count, n_seeds_h0, fp_rate))

    # Test 2: Detection delay (+5% path increase)
    print("\n--- Test 2: Detection Delay (+5% path increase) ---")
    n_seeds_detect = 50
    delays = []
    for s in range(n_seeds_detect):
        d = detection_trial(s, path_increase_pct=0.05)
        if d is not None:
            delays.append(d)

    if delays:
        avg_delay = sum(delays) / len(delays)
        min_delay = min(delays)
        max_delay = max(delays)
        print("  Detection delay (N=4): avg={:.2f}, min={}, max={} RTTs".format(avg_delay, min_delay, max_delay))
        print("  Detected in {}/{} trials".format(len(delays), n_seeds_detect))
    else:
        print("  No detections!")

    # Report comparison
    print("\n--- Comparison vs N=3 ---")
    print("  Metric              | N=3 (reference) | N=4 (new)")
    print("  --------------------+-----------------+----------")
    if delays:
        print("  H0 FP rate          | ~0.000000       | {:.6f}".format(fp_rate))
        print("  Detection delay avg | ~3.0 RTTs       | {:.2f} RTTs".format(avg_delay))
    else:
        print("  H0 FP rate          | ~0.000000       | {:.6f}".format(fp_rate))
        print("  Detection delay avg | ~3.0 RTTs       | N/A")

    print()
    print("  N=4 requires 4 cumulative slow exceedances vs 3 previously.")
    print("  FP rate expected to remain ~0 while detection delay may increase ~1 RTT.")

    total_time = tm.time() - total_t0
    print("\nTotal time: {:.1f}s".format(total_time))

    return 0 if fp_rate == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
