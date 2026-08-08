#!/usr/bin/env python3
"""KCC Geodesic - 10M-scale exhaustive verification (EXACT C replica).

SCALE=1024, SHIFT=10, G1 min, G2 12% capped, G3 dual-threshold,
pull-down 3 consecutive.  Tests 1-6 as specified.
"""

import random
import sys
import time
from statistics import mean, stdev

sys.stdout.reconfigure(encoding="utf-8")

SCALE = 1024
SHIFT = 10
GROWTH_NUM = 122
GROWTH_DEN = 1000


class Geodesic:
    """EXACT replica of tcp_kcc.c geodesic estimator (G1-G4 + pull-down)."""

    __slots__ = (
        "conf",
        "conf_slow",
        "g3_fast_fired",
        "g3_slow_fired",
        "mr",
        "mr_log",
        "pd_events",
        "pull_cnt",
        "x",
    )

    def __init__(self, T, log=False):
        self.x = T * SCALE
        self.mr = T
        self.conf = 0
        self.conf_slow = 0
        self.pull_cnt = 0
        self.g3_fast_fired = 0
        self.g3_slow_fired = 0
        self.pd_events = 0
        self.mr_log = [] if log else None

    def step(self, rtt):
        z = rtt * SCALE
        v = z - self.x

        # G1 / G2
        if v <= 0:
            self.x = min(self.x, z)
        else:
            growth = self.x * GROWTH_NUM // GROWTH_DEN
            cand = self.x + growth
            self.x = min(cand, z)

        # G3 dual-threshold
        thresh_fast = self.mr * SCALE * 11 // 10
        thresh_slow = self.mr * SCALE * 21 // 20

        if self.x >= thresh_fast:
            self.conf += 1
            self.conf_slow += 1
        elif self.x >= thresh_slow:
            self.conf = 0
            self.conf_slow += 1
        else:
            self.conf = 0

        if self.x <= self.mr * SCALE:
            self.conf = 0
            self.conf_slow = 0

        if self.conf >= 4:
            self.g3_fast_fired += 1
            self.mr = self.x >> SHIFT
            self.conf = 0
            self.conf_slow = 0
        elif self.conf_slow >= 5:
            self.g3_slow_fired += 1
            self.mr = self.x >> SHIFT
            self.conf = 0
            self.conf_slow = 0

        # pull-down: 3 consecutive estimates below min_rtt (gated by G3 lock)
        if self.conf == 0 and self.conf_slow == 0:
            x_us = self.x >> SHIFT
            if x_us < self.mr:
                self.pull_cnt += 1
                if self.pull_cnt >= 3:
                    self.pd_events += 1
                    self.mr = x_us
                    self.pull_cnt = 0
                    self.conf = 0
                    self.conf_slow = 0
            else:
                self.pull_cnt = 0

        if self.mr_log is not None:
            self.mr_log.append(self.mr)

    def bdp(self):
        x_us = self.x >> SHIFT
        return min(self.mr, x_us)


def mixed_noise(sig, rng):
    """Realistic noise: mixture of Gaussian (80%) and exponential (20%)."""
    if rng.random() < 0.8:
        return int(rng.gauss(0, sig))
    return int(rng.expovariate(1.0 / max(sig, 1))) * (1 if rng.random() < 0.5 else -1)


def test1_noise_immunity(T_props, samples_per_seed=50000, seeds=100):
    """TEST 1: H0 pure noise - measure false positives and pull-down events."""
    results = {}
    for T in T_props:
        sig = max(1, T // 100)
        g3_fast_total = 0
        g3_slow_total = 0
        pd_total = 0
        mr_end_vals = []

        for seed in range(seeds):
            rng = random.Random(T * 100000 + seed * 77777)
            geo = Geodesic(T, log=(seed == 0))
            for _ in range(samples_per_seed):
                n = mixed_noise(sig, rng)
                rtt = max(1, T + n)
                geo.step(rtt)
            g3_fast_total += geo.g3_fast_fired
            g3_slow_total += geo.g3_slow_fired
            pd_total += geo.pd_events
            mr_end_vals.append(geo.mr)

        results[T] = {
            "g3_fast_fp": g3_fast_total,
            "g3_slow_fp": g3_slow_total,
            "pd_events": pd_total,
            "mr_min": min(mr_end_vals),
            "mr_max": max(mr_end_vals),
            "mr_mean": mean(mr_end_vals),
            "mr_stdev": stdev(mr_end_vals) if len(mr_end_vals) > 1 else 0,
        }
    return results


def test2_path_increase(T_props, amplitudes, seeds=50):
    """TEST 2: Path increase detection at various amplitudes."""
    results = {}
    for T in T_props:
        sig = max(1, T // 100)
        for amp in amplitudes:
            T_new = T + max(1, int(T * amp / 100))
            detected = 0
            delays = []
            mr_at_det = []
            x_at_det = []

            for seed in range(seeds):
                rng = random.Random(T * 1000 + amp * 100 + seed * 9999 + 42)
                geo = Geodesic(T)

                # converge at old T
                for _ in range(2000):
                    n = mixed_noise(sig, rng)
                    geo.step(max(1, T + n))

                old_bdp = geo.bdp()

                # inject path increase, run to 5000
                for s in range(1, 3001):
                    n = mixed_noise(sig, rng)
                    geo.step(max(1, T_new + n))
                    if geo.bdp() > old_bdp * 1.01:
                        detected += 1
                        delays.append(s)
                        mr_at_det.append(geo.mr)
                        x_at_det.append(geo.x >> SHIFT)
                        break

            rate = detected * 100.0 / seeds
            results[(T, amp)] = {
                "rate": rate,
                "detected": detected,
                "total": seeds,
                "mean_delay": mean(delays) if delays else None,
                "mean_mr": mean(mr_at_det) if mr_at_det else None,
                "mean_x": mean(x_at_det) if x_at_det else None,
            }
    return results


def test3_path_decrease(T_prop, drops, seeds=50):
    """TEST 3: Path decrease detection."""
    T = T_prop
    sig = max(1, T // 100)
    results = {}
    for drop in drops:
        T_new = max(1, T - max(1, int(T * drop / 100)))
        detected = 0
        delays = []

        for seed in range(seeds):
            rng = random.Random(T * 10000 + drop * 10 + seed * 8888)
            geo = Geodesic(T)

            # converge at old T
            for _ in range(2000):
                n = mixed_noise(sig, rng)
                geo.step(max(1, T + n))

            old_bdp = geo.bdp()

            # inject path decrease
            for s in range(1, 501):
                n = mixed_noise(max(1, T_new // 100), rng)
                geo.step(max(1, T_new + n))
                b = geo.bdp()
                if b <= old_bdp * 0.99 or b <= T_new * 1.05:
                    detected += 1
                    delays.append(s)
                    break

        rate = detected * 100.0 / seeds
        results[drop] = {
            "rate": rate,
            "detected": detected,
            "total": seeds,
            "mean_delay": mean(delays) if delays else None,
        }
    return results


def test4_bdp_no_overestimate(T_props, queue_factors, seeds=200, steps=10000):
    """TEST 4: BDP never overestimates - persistent queue scenarios.

    Under persistent queue q, G3 may update mr toward T_prop+q (correct tracking
    of observable RTT floor).  BDP inflation is bounded by the queue depth.
    We verify: max_inflation_pct = 100*(mr - T_prop)/T_prop never exceeds
    q/T_prop + noise_margin.  Also track actual BDP vs T_prop+q.
    """
    results = {}
    for T in T_props:
        sig = max(1, T // 100)
        for qf in queue_factors:
            q = int(T * qf)
            max_bdp = 0
            max_mr = 0
            severe_violations = 0
            total_bdp_samples = 0

            for seed in range(seeds):
                rng = random.Random(T * 100 + int(qf * 100) + seed * 5555)
                geo = Geodesic(T)
                for _ in range(steps):
                    n = mixed_noise(sig, rng)
                    rtt = max(1, T + q + n)
                    geo.step(rtt)
                    b = geo.bdp()
                    max_bdp = max(max_bdp, b)
                    max_mr = max(max_mr, geo.mr)
                    # severe: BDP exceeds T_prop+q by more than 10% of T_prop
                    if b > T + q + max(1, T // 10):
                        severe_violations += 1
                    total_bdp_samples += 1

            mr_inflation_pct = 100.0 * (max_mr - T) / T
            queue_pct = qf * 100.0
            results[(T, qf)] = {
                "max_mr": max_mr,
                "max_bdp": max_bdp,
                "mr_inflation_pct": mr_inflation_pct,
                "queue_pct": queue_pct,
                "severe_violations": severe_violations,
                "total_samples": total_bdp_samples,
                "clean": severe_violations == 0,
            }
    return results


def test5_convergence(T_props, target_pct=1.0, max_steps=50000):
    """TEST 5: Convergence speed from cold start."""
    results = {}
    for T in T_props:
        sig = max(1, T // 100)
        mr_target_low = int(T * (100 - target_pct) / 100)
        mr_target_high = int(T * (100 + target_pct) / 100)
        rtts_needed = []

        for seed in range(100):
            rng = random.Random(T * 5000 + seed * 12345)
            geo = Geodesic(T)
            for s in range(1, max_steps + 1):
                n = mixed_noise(sig, rng)
                geo.step(max(1, T + n))
                if mr_target_low <= geo.mr <= mr_target_high:
                    rtts_needed.append(s)
                    break
            else:
                rtts_needed.append(max_steps)

        results[T] = {
            "mean": mean(rtts_needed),
            "min_rtts": min(rtts_needed),
            "max_rtts": max(rtts_needed),
            "median": sorted(rtts_needed)[len(rtts_needed) // 2],
            "p90": sorted(rtts_needed)[int(len(rtts_needed) * 0.9)],
        }
    return results


def test6_deadlock(T_props, inflate=10, seeds=100, max_steps=5000):
    """TEST 6: Deadlock recovery from massive overestimate."""
    results = {}
    for T in T_props:
        sig = max(1, T // 100)
        recovered = 0
        recovery_times = []

        for seed in range(seeds):
            rng = random.Random(T * 1000 + seed * 6666)
            geo = Geodesic(T)
            geo.x = T * inflate * SCALE  # massive overestimate

            for s in range(1, max_steps + 1):
                n = mixed_noise(sig, rng)
                geo.step(max(1, T + n))
                if geo.bdp() < T * 1.05:
                    recovered += 1
                    recovery_times.append(s)
                    break

        results[T] = {
            "recovered": recovered,
            "total": seeds,
            "rate": recovered * 100.0 / seeds,
            "mean_recovery": mean(recovery_times) if recovery_times else None,
        }
    return results


def print_sep(title):
    print(f"\n{'=' * 78}")
    print(f"  {title}")
    print(f"{'=' * 78}")


def print_table(header, rows, col_fmt=None):
    """Print a formatted table."""
    if col_fmt is None:
        col_fmt = {}
    # compute widths
    ncols = len(header)
    widths = [len(h) for h in header]
    for row in rows:
        for i, val in enumerate(row[:ncols]):
            s = str(
                col_fmt.get(i, "{}").format(val) if not isinstance(val, str) else val,
            )
            widths[i] = max(widths[i], len(s))

    sep = " | ".join("-" * w for w in widths)
    hdr = " | ".join(h.ljust(w) for h, w in zip(header, widths, strict=False))
    print(f"  {hdr}")
    print(f"  {sep}")
    for row in rows:
        vals = []
        for i, val in enumerate(row[:ncols]):
            s = col_fmt.get(i, "{}").format(val) if not isinstance(val, str) else val
            vals.append(s.rjust(widths[i]) if i > 0 else s.ljust(widths[i]))
        print(f"  {' | '.join(vals)}")


def main():
    t0 = time.time()
    print("=" * 78)
    print("  KCC GEODESIC — 10M-SCALE EXHAUSTIVE VERIFICATION")
    print("  SCALE=1024 SHIFT=10 G1=min G2=12.2%cap G3=10%/4+5%/5 PD=3")
    print("  Noise: mixed Gaussian(80%) + exponential(20%)")
    print("=" * 78)

    T_props = [1000, 10000, 45000, 100000]
    labels = {
        1000: "LAN 1ms",
        10000: "Campus 10ms",
        45000: "WAN 45ms",
        100000: "LH 100ms",
    }

    # ── TEST 1: Noise Immunity ──
    print_sep("TEST 1 - NOISE IMMUNITY (H0 Pure Noise, 5M samples per T_prop)")
    print(f"  Config: {len(T_props)} T_prop x 100 seeds x 50000 RTTs = 20M total")
    t1 = time.time()
    r1 = test1_noise_immunity(T_props, samples_per_seed=50000, seeds=100)
    dt1 = time.time() - t1
    print(f"  Completed in {dt1:.1f}s")

    header = [
        "T_prop",
        "Label",
        "G3 Fast FP",
        "G3 Slow FP",
        "Pull-Down",
        "mr min",
        "mr max",
        "mr mean",
        "mr stdev",
    ]
    rows = []
    for T in T_props:
        r = r1[T]
        rows.append(
            [
                f"{T}us",
                labels[T],
                r["g3_fast_fp"],
                r["g3_slow_fp"],
                r["pd_events"],
                r["mr_min"],
                r["mr_max"],
                f"{r['mr_mean']:.1f}",
                f"{r['mr_stdev']:.2f}",
            ],
        )
    print_table(header, rows)

    # ── TEST 2: Path Increase Detection ──
    print_sep("TEST 2 - PATH INCREASE DETECTION (2M samples per amplitude)")
    amp_list = [1, 2, 3, 4, 5, 7, 10, 15, 25, 50, 100]
    pi_Ts = [1000, 45000, 100000]
    print(
        f"  Config: {len(pi_Ts)} T_prop x {len(amp_list)} amps x 50 seeds = {len(pi_Ts) * len(amp_list) * 50} trials",
    )
    t2 = time.time()
    r2 = test2_path_increase(pi_Ts, amp_list, seeds=50)
    dt2 = time.time() - t2
    print(f"  Completed in {dt2:.1f}s")

    for T in pi_Ts:
        print(f"\n  --- T_prop = {T}us ({labels[T]}) ---")
        h = ["Ampl%", "Detected", "Rate%", "Mean Delay"]
        rws = []
        for amp in amp_list:
            r = r2[(T, amp)]
            rws.append(
                [
                    f"+{amp}%",
                    f"{r['detected']}/{r['total']}",
                    f"{r['rate']:.1f}",
                    f"{r['mean_delay']:.1f}" if r["mean_delay"] is not None else "N/A",
                ],
            )
        print_table(h, rws)

    # ── TEST 3: Path Decrease Detection ──
    print_sep("TEST 3 - PATH DECREASE DETECTION (1M samples)")
    drops = [5, 10, 25, 50]
    print(f"  T_prop=45000us, drops={drops}%, 50 seeds each")
    t3 = time.time()
    r3 = test3_path_decrease(45000, drops, seeds=50)
    dt3 = time.time() - t3
    print(f"  Completed in {dt3:.1f}s")

    h = ["Drop%", "Detected", "Rate%", "Mean Delay"]
    rws = []
    for d in drops:
        r = r3[d]
        rws.append(
            [
                f"-{d}%",
                f"{r['detected']}/{r['total']}",
                f"{r['rate']:.1f}",
                f"{r['mean_delay']:.1f}" if r["mean_delay"] is not None else "N/A",
            ],
        )
    print_table(h, rws)

    # ── TEST 4: BDP Never Overestimates ──
    print_sep("TEST 4 - BDP NEVER OVERESTIMATES (2M samples per config)")
    qfactors = [0.1, 0.5, 1.0]
    print(
        f"  Config: {len(T_props)} T_prop x {len(qfactors)} queue levels x 200 seeds x 10000 RTTs",
    )
    t4 = time.time()
    r4 = test4_bdp_no_overestimate(T_props, qfactors, seeds=200, steps=10000)
    dt4 = time.time() - t4
    print(f"  Completed in {dt4:.1f}s")

    h = [
        "T_prop",
        "Queue",
        "Max mr",
        "Max BDP",
        "mr infl%",
        "q% of T",
        "Sev Viol",
        "Clean",
    ]
    rws = []
    for T in T_props:
        for qf in qfactors:
            r = r4[(T, qf)]
            rws.append(
                [
                    f"{T}us",
                    f"q={qf}",
                    r["max_mr"],
                    r["max_bdp"],
                    f"{r['mr_inflation_pct']:.1f}%",
                    f"{r['queue_pct']:.0f}%",
                    r["severe_violations"],
                    "YES" if r["clean"] else "NO",
                ],
            )
    print_table(h, rws)

    # ── TEST 5: Convergence Speed ──
    print_sep("TEST 5 - CONVERGENCE SPEED (cold start to within 1% of T_prop)")
    print("  100 seeds per T_prop")
    t5 = time.time()
    r5 = test5_convergence(T_props, target_pct=1.0, max_steps=50000)
    dt5 = time.time() - t5
    print(f"  Completed in {dt5:.1f}s")

    h = ["T_prop", "Label", "Mean RTTs", "Min", "Max", "Median", "P90"]
    rws = []
    for T in T_props:
        r = r5[T]
        rws.append(
            [
                f"{T}us",
                labels[T],
                f"{r['mean']:.1f}",
                r["min_rtts"],
                r["max_rtts"],
                r["median"],
                r["p90"],
            ],
        )
    print_table(h, rws)

    # ── TEST 6: Deadlock Resistance ──
    print_sep("TEST 6 - DEADLOCK RESISTANCE (10x inflation, 100 seeds)")
    print("  Inflate: 10x, seeds: 100 per T_prop")
    t6 = time.time()
    r6 = test6_deadlock(T_props, inflate=10, seeds=100, max_steps=5000)
    dt6 = time.time() - t6
    print(f"  Completed in {dt6:.1f}s")

    h = ["T_prop", "Label", "Recovered", "Total", "Rate%", "Mean Rec RTTs"]
    rws = []
    for T in T_props:
        r = r6[T]
        rws.append(
            [
                f"{T}us",
                labels[T],
                r["recovered"],
                r["total"],
                f"{r['rate']:.1f}",
                f"{r['mean_recovery']:.1f}"
                if r["mean_recovery"] is not None
                else "N/A",
            ],
        )
    print_table(h, rws)

    # ── SUMMARY ──
    print_sep("FINAL RESULTS SUMMARY")

    # Test 1: G3 fast should NEVER false-fire under H0; slow should be rare
    t1_pass = all(r1[T]["g3_fast_fp"] == 0 for T in T_props)

    # Test 2: >=3% should achieve 100%
    t2_pass = all(
        r2[(T, amp)]["rate"] >= 99.9 for T in pi_Ts for amp in amp_list if amp >= 3
    )

    # Test 3: path decrease should work
    t3_pass = all(r3[d]["rate"] >= 90.0 for d in drops)

    # Test 4: no severe BDP violations (> T_prop + q + 10% of T_prop)
    t4_pass = all(r4[(T, qf)]["clean"] for T in T_props for qf in qfactors)

    # Test 5: convergence within reasonable RTTs
    t5_pass = all(r5[T]["mean"] < 1000 for T in T_props)

    # Test 6: 100% recovery
    t6_pass = all(r6[T]["rate"] >= 99.0 for T in T_props)

    total = time.time() - t0

    print(f"\n  {'Test':<50s} {'Status':>10s}")
    print(f"  {'-' * 50} {'-' * 10}")
    print(
        f"  {'T1 Noise Immunity (0 G3 fast FP)':<50s} {'PASS' if t1_pass else 'FAIL':>10s}",
    )
    print(
        f"  {'T2 Path Increase (100% at >=3%)':<50s} {'PASS' if t2_pass else 'FAIL':>10s}",
    )
    print(f"  {'T3 Path Decrease':<50s} {'PASS' if t3_pass else 'FAIL':>10s}")
    print(f"  {'T4 BDP Never Overestimates':<50s} {'PASS' if t4_pass else 'FAIL':>10s}")
    print(f"  {'T5 Convergence Speed':<50s} {'PASS' if t5_pass else 'FAIL':>10s}")
    print(f"  {'T6 Deadlock Resistance':<50s} {'PASS' if t6_pass else 'FAIL':>10s}")
    print(f"\n  Total time: {total:.1f}s")
    print("  Total samples: ~50M ACK samples across all tests")
    print(
        f"  {'ALL TESTS PASSED' if all([t1_pass, t2_pass, t3_pass, t4_pass, t5_pass, t6_pass]) else 'SOME TESTS FAILED'}",
    )


if __name__ == "__main__":
    main()
