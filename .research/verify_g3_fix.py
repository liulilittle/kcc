#!/usr/bin/env python3
"""Comprehensive verification of G3 fix for GeodesicEstimator."""

import math
import random

SCALE = 1024
GROWTH_NUM = 122
GROWTH_DEN = 1000
OUTPUT = r"D:\dd\ucp\.research\g3_fix_verification.txt"


class GeodesicEstimator:
    """FIXED: mr only updated on G3 fire, never by running min."""

    def __init__(self, T_prop, sigma):
        self.x = T_prop * SCALE
        self.mr = T_prop
        self.conf = 0
        self.conf_slow = 0
        self.T = T_prop
        self.sigma = sigma

    def step(self, rtt):
        z = rtt * SCALE
        v = z - self.x
        g3_fired = False
        if v <= 0:
            self.x = min(self.x, z)
        else:
            growth = (self.x * GROWTH_NUM) // GROWTH_DEN
            self.x = min(self.x + growth, z)
        thresh_fast = (self.mr * SCALE * 11) // 10
        thresh_slow = (self.mr * SCALE * 21) // 20
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
        if self.conf >= 4 or self.conf_slow >= 5:
            self.mr = self.x // SCALE
            self.conf = 0
            self.conf_slow = 0
            g3_fired = True
        x_us = self.x // SCALE
        return g3_fired, x_us, min(x_us, self.mr)


class GeodesicEstimatorOld:
    """OLD: mr polluted by running min on every RTT (the bug)."""

    def __init__(self, T_prop, sigma):
        self.x = T_prop * SCALE
        self.mr = T_prop
        self.conf = 0
        self.T = T_prop
        self.sigma = sigma

    def step(self, rtt):
        self.mr = min(self.mr, rtt)
        z = rtt * SCALE
        v = z - self.x
        g3_fired = False
        if v <= 0:
            self.x = min(self.x, z)
        else:
            growth = (self.x * GROWTH_NUM) // GROWTH_DEN
            self.x = min(self.x + growth, z)
        thresh_fast = (self.mr * SCALE * 11) // 10
        if self.x > thresh_fast:
            self.conf += 1
        elif self.x <= self.mr * SCALE:
            self.conf = 0
        if self.conf >= 4:
            self.mr = self.x // SCALE
            self.conf = 0
            g3_fired = True
        x_us = self.x // SCALE
        return g3_fired, x_us, min(x_us, self.mr)


def fmt_header(title):
    w = 90
    return "\n" + "=" * w + "\n" + title.center(w) + "\n" + "=" * w


def fmt_table(rows, headers):
    ncol = len(headers)
    cw = [max(len(str(h)), 8) for h in headers]
    for row in rows:
        for i, v in enumerate(row):
            cw[i] = max(cw[i], len(str(v)))
    line = " | ".join(h.ljust(cw[i]) for i, h in enumerate(headers))
    sep = "-+-".join("-" * cw[i] for i in range(ncol))
    body = []
    body.append(line)
    body.append(sep)
    for row in rows:
        body.append(" | ".join(str(v).ljust(cw[i]) for i, v in enumerate(row)))
    return "\n".join(body)


# ==============================================================================
# TEST 1: Noise immunity (H0 - no path change)
# ==============================================================================
def test1():
    lines = [fmt_header("TEST 1: Noise Immunity (H0 - no path change)")]
    RTTs = [25, 50, 100, 200, 500, 1000, 5000, 10000, 50000, 100000, 500000, 1000000]
    SEEDS = 200
    STEPS = 5000

    rows = []
    for T in RTTs:
        sigma = max(1, T // 100)
        g3_total = 0
        max_conf_seen = 0
        dev_sum = 0.0
        dev_max = 0.0
        for seed in range(SEEDS):
            rng = random.Random(T * 1000 + seed)
            est = GeodesicEstimator(T, sigma)
            for _ in range(STEPS):
                rtt = max(1, T + int(rng.gauss(0, sigma)))
                g3, x_us, _ = est.step(rtt)
                if g3:
                    g3_total += 1
                dev = abs(x_us - T)
                dev_sum += dev
                dev_max = max(dev_max, dev)
                max_conf_seen = max(max_conf_seen, est.conf)
        fp_rate = g3_total / (SEEDS * STEPS) * 100
        mean_dev = dev_sum / (SEEDS * STEPS)
        rows.append(
            [
                T,
                sigma,
                f"{fp_rate:.4f}%",
                f"{max_conf_seen}",
                f"{mean_dev:.2f}",
                f"{dev_max:.0f}",
            ],
        )
    headers = ["RTT(us)", "sigma", "FP_rate", "max_conf", "mean_dev(us)", "max_dev(us)"]
    lines.append(fmt_table(rows, headers))
    return "\n".join(lines)


# ==============================================================================
# TEST 2: Path increase detection (H1 - real path change)
# ==============================================================================
def test2():
    lines = [fmt_header("TEST 2: Path Increase Detection (H1 - real path change)")]
    RTTs = [1000, 10000, 100000, 1000000]
    AMPS = [1, 2, 5, 10, 25, 50, 100, 200]
    SEEDS = 20
    WARMUP = 500
    MAX_STEPS = 10000

    for T in RTTs:
        sigma = max(1, T // 100)
        lines.append(f"\n--- Base RTT = {T} us (sigma = {sigma}) ---")
        sub = []
        for amp in AMPS:
            T_new = T * (100 + amp) // 100
            if T_new == T:
                continue
            delays = []
            dnfs = 0
            expected_steps = "N/A"
            if amp >= 5:
                ratio = 1.1 / (1 + amp / 100.0)
                if amp >= 10:
                    expected_steps = "0 (immediate)"
                else:
                    n = math.ceil(math.log(ratio) / math.log(1.12))
                    expected_steps = f"{n} growth steps"
            for seed in range(SEEDS):
                rng = random.Random(T * 10000 + amp * 1000 + seed)
                est = GeodesicEstimator(T, sigma)
                for _ in range(WARMUP):
                    rtt = max(1, T + int(rng.gauss(0, sigma)))
                    est.step(rtt)
                for step in range(1, MAX_STEPS + 1):
                    rtt = max(1, T_new + int(rng.gauss(0, sigma)))
                    g3, _, _ = est.step(rtt)
                    if g3:
                        delays.append(step)
                        break
                else:
                    dnfs += 1
            if len(delays) == 0:
                det_str = "NONE"
                mean_d = "---"
                median_d = "---"
                min_d = "---"
                max_d = "---"
            else:
                mean_d = f"{sum(delays) / len(delays):.1f}"
                delays_sorted = sorted(delays)
                median_d = f"{delays_sorted[len(delays_sorted) // 2]}"
                min_d = f"{min(delays)}"
                max_d = f"{max(delays)}"
                det_str = f"{len(delays)}/{SEEDS}"
            sub.append(
                [
                    f"+{amp}%",
                    T_new,
                    expected_steps,
                    det_str,
                    mean_d,
                    median_d,
                    min_d,
                    max_d,
                    f"{dnfs}",
                ],
            )
        headers = [
            "amp",
            "T_new(us)",
            "exp_steps",
            "detected",
            "mean_dly",
            "med_dly",
            "min",
            "max",
            "DNF",
        ]
        lines.append(fmt_table(sub, headers))
    return "\n".join(lines)


# ==============================================================================
# TEST 3: BDP overestimation cost demonstration
# ==============================================================================
def test3():
    lines = [fmt_header("TEST 3: BDP Overestimation - Fix vs Old (mr pollution)")]
    T = 100000
    sigma = T // 100
    SEEDS = 5
    STEPS = 5000

    lines.append(
        f"\nRTT = {T} us, sigma = {sigma} us (1%), {STEPS} steps, {SEEDS} seeds\n",
    )

    # Collect data for one detailed trace per version
    trace_seed = 42

    # --- Detailed trace for one seed ---
    rng = random.Random(trace_seed)
    est_fix = GeodesicEstimator(T, sigma)
    trace_fix = []
    for _ in range(STEPS):
        rtt = max(1, T + int(rng.gauss(0, sigma)))
        _, x_us, bdp = est_fix.step(rtt)
        trace_fix.append((rtt, x_us, bdp, est_fix.mr))

    rng = random.Random(trace_seed)
    est_old = GeodesicEstimatorOld(T, sigma)
    trace_old = []
    for _ in range(STEPS):
        rtt = max(1, T + int(rng.gauss(0, sigma)))
        _, x_us, bdp = est_old.step(rtt)
        trace_old.append((rtt, x_us, bdp, est_old.mr))

    # Summary: sample every 500 steps
    lines.append("Detailed trace (every 500 steps):")
    rows = []
    for i in range(0, STEPS, 500):
        r_fix = trace_fix[i]
        r_old = trace_old[i]
        rows.append(
            [
                str(i),
                str(r_fix[0]),
                str(r_fix[1]),
                str(r_fix[3]),
                str(r_fix[2]),
                str(r_old[0]),
                str(r_old[1]),
                str(r_old[3]),
                str(r_old[2]),
            ],
        )
    headers = [
        "step",
        "rtt_fix",
        "x_fix",
        "mr_fix",
        "bdp_fix",
        "rtt_old",
        "x_old",
        "mr_old",
        "bdp_old",
    ]
    lines.append(fmt_table(rows, headers))

    # Per-seed summary
    lines.append("\nPer-seed end-state summary:")
    rows = []
    for seed in range(SEEDS):
        rng = random.Random(seed * 1000 + 999)
        ef = GeodesicEstimator(T, sigma)
        eo = GeodesicEstimatorOld(T, sigma)
        for _ in range(STEPS):
            rtt = max(1, T + int(rng.gauss(0, sigma)))
            ef.step(rtt)
            eo.step(rtt)
        rows.append(
            [
                str(seed),
                str(ef.mr),
                str(ef.x // SCALE),
                str(min(ef.x // SCALE, ef.mr)),
                str(eo.mr),
                str(eo.x // SCALE),
                str(min(eo.x // SCALE, eo.mr)),
            ],
        )
    headers = ["seed", "mr_fix", "x_fix", "bdp_fix", "mr_old", "x_old", "bdp_old"]
    lines.append(fmt_table(rows, headers))

    # BDP inflation stats
    lines.append("\nBDP inflation (over T_prop):")
    rows = []
    for seed in range(SEEDS):
        rng = random.Random(seed * 1000 + 999)
        ef = GeodesicEstimator(T, sigma)
        eo = GeodesicEstimatorOld(T, sigma)
        for _ in range(STEPS):
            rtt = max(1, T + int(rng.gauss(0, sigma)))
            ef.step(rtt)
            eo.step(rtt)
        bdp_f = min(ef.x // SCALE, ef.mr)
        bdp_o = min(eo.x // SCALE, eo.mr)
        infl_f = (bdp_f - T) / T * 100
        infl_o = (bdp_o - T) / T * 100
        rows.append(
            [str(seed), str(bdp_f), f"{infl_f:+.2f}%", str(bdp_o), f"{infl_o:+.2f}%"],
        )
    headers = ["seed", "bdp_fix(us)", "inflation_fix", "bdp_old(us)", "inflation_old"]
    lines.append(fmt_table(rows, headers))

    return "\n".join(lines)


# ==============================================================================
# MAIN
# ==============================================================================
def main():
    result = []
    result.append("G3 FIX VERIFICATION REPORT")
    result.append(f"SCALE = {SCALE}, growth = 12.2%/step, G3 threshold = 1.1x mr")

    result.append(test1())
    result.append(test2())
    result.append(test3())

    out = "\n".join(result)
    print(out)
    with open(OUTPUT, "w", encoding="utf-8") as f:
        f.write(out)
    print(f"\nResults saved to {OUTPUT}")


if __name__ == "__main__":
    main()
