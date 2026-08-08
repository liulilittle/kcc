#!/usr/bin/env python3
"""PROBE_RTT deletion — systematic verification. Geodesic accuracy."""

import random
import sys

sys.stdout.reconfigure(encoding="utf-8")

SCALE, SCALE_SHIFT = 1024, 10
G2_GROWTH = 12
G3_FAST_N, G3_FAST_TH = 3, 11
G3_SLOW_N, G3_SLOW_TH = 50, 21
JITTER_DIV = 100.0
PD_FAST_FALL = 3  # geodesic pull-down needs 3 consecutive x_est < mr


class Geo:
    def __init__(self, tp, seed=0):
        self.tp = max(1, tp)
        self.mr = self.tp  # min_rtt_us
        self.x = self.tp * SCALE  # x_est(scaled)
        self.cnf, self.csl = 0, 0
        self.pd_cnt = 0  # pull-down counter
        self.jtr = max(1.0, self.tp / JITTER_DIV)
        self.rng = random.Random(seed)
        self.cnt = 0
        self.g3, self.gs, self.pd = 0, 0, 0

    def update(self, rtt):
        z = max(1, int(rtt)) * SCALE
        # G1/G2
        if z <= self.x:
            self.x = z
        else:
            g = self.x * G2_GROWTH // 100
            self.x = min(self.x + g, z)

        # G3 thresholds
        ft = self.mr * SCALE * G3_FAST_TH // 10
        st = self.mr * SCALE * G3_SLOW_TH // 20
        bl = self.mr * SCALE

        if self.x >= ft:
            self.cnf += 1
            self.csl += 1
        elif self.x >= st:
            self.cnf = 0
            self.csl += 1
        else:
            self.cnf = 0

        if self.x <= bl:
            self.cnf = 0
            self.csl = 0

        # G3 triggers
        fired = False
        if self.cnf >= G3_FAST_N:
            self.mr = self.x >> SCALE_SHIFT
            self.cnf = self.csl = self.pd_cnt = 0
            self.g3 += 1
            fired = True
        elif self.csl >= G3_SLOW_N:
            self.mr = self.x >> SCALE_SHIFT
            self.cnf = self.csl = self.pd_cnt = 0
            self.gs += 1
            fired = True

        # G3 lock: skip pull-down & window while counters>0
        if self.cnf > 0 or self.csl > 0:
            self.cnt += 1
            return self.mr, self.x >> SCALE_SHIFT, fired

        # Geodesic pull-down: x_est < mr for N consecutive RTTs
        xus = self.x >> SCALE_SHIFT
        if xus < self.mr:
            self.pd_cnt += 1
            if self.pd_cnt >= PD_FAST_FALL:
                self.mr = xus
                self.pd_cnt = 0
                self.pd += 1
        else:
            self.pd_cnt = 0

        self.cnt += 1
        return self.mr, xus, fired


def stat(nums):
    return (
        sum(nums) / len(nums) if nums else 0,
        min(nums) if nums else 0,
        max(nums) if nums else 0,
    )


def pct(a, ref):
    return (a - ref) / ref * 100 if ref else 0


def hdr(s):
    print(f"\n{'=' * 60}\n  {s}\n{'=' * 60}")


# ---- A: cold-start convergence ----
hdr("TEST A  cold-start: mr converges to true T_prop?")
for tp in [1000, 10000, 45000, 100000, 1000000]:
    final = []
    for s in range(100):
        g = Geo(tp, s)
        for _ in range(5000):
            g.update(tp + g.rng.gauss(0, g.jtr))
        final.append(g.mr)
    mu, lo, hi = stat(final)
    print(
        f"  T={tp:>7}u  mr={mu:>8.1f}({pct(mu, tp):+.2f}%)  [{lo:.0f},{hi:.0f}]  G3={sum(Geo(tp, s).g3 > 0 for s in range(20))}/{20}",
    )

# ---- B: self-queue + natural drain ----
hdr("TEST B  self-queue injection, natural drain (no PROBE_RTT)")
for tp, qpct in [(1000, 30), (5000, 25), (45000, 25), (100000, 20)]:
    rec = []
    for s in range(50):
        g = Geo(tp, s)
        for _ in range(500):
            g.update(tp + g.rng.gauss(0, g.jtr))  # warmup
        qus = int(tp * qpct / 100)
        for _ in range(500):
            g.update(tp + qus + g.rng.gauss(0, g.jtr))  # inject queue
        r = None
        for i in range(5000):
            g.update(tp + g.rng.gauss(0, g.jtr))  # drain
            if g.mr <= tp * 1.02 and r is None:
                r = i
        rec.append(r or 5000)
    rmed = sorted(rec)[len(rec) // 2]
    rgot = sum(1 for x in rec if x < 5000)
    print(f"  T={tp:>7}u  q={qpct}%  recovered={rgot}/50  median={rmed}RTT")

# ---- C: path increase detection ----
hdr("TEST C  path INCREASE (>=5% must be 100%, fast+slow combined)")
for tp in [1000, 10000, 45000, 100000]:
    row = []
    for a in [3, 5, 7, 10, 15, 25, 50, 100]:
        ok = 0
        delays = []
        for s in range(50):
            g = Geo(tp, s)
            for _ in range(1000):
                g.update(tp + g.rng.gauss(0, g.jtr))
            new = int(tp * (1 + a / 100))
            for i in range(1, 5000):
                _, _, f = g.update(new + g.rng.gauss(0, g.jtr))
                if f:
                    ok += 1
                    delays.append(i)
                    break
        d = int(sorted(delays)[len(delays) // 2]) if delays else 0
        row.append(f"{a:>3}%:{ok:>3}/{50}:{d:>4}")
    print(f"  T={tp:>7}u  " + "  ".join(row))

# ---- D: path decrease ----
hdr("TEST D  path DECREASE (G1 instant + pull-down)")
for tp in [1000, 10000, 45000, 100000]:
    for drop in [5, 10, 25, 50]:
        ok, ds = 0, []
        for s in range(50):
            g = Geo(tp, s)
            for _ in range(500):
                g.update(tp + g.rng.gauss(0, g.jtr))
            new = int(tp * (1 - drop / 100))
            for i in range(1, 500):
                g.update(new + g.rng.gauss(0, g.jtr))
                if g.mr <= new * 1.02:
                    ok += 1
                    ds.append(i)
                    break
        d = int(sorted(ds)[len(ds) // 2]) if ds else 0
        print(f"  T={tp:>7}u  -{drop:>2}%  ok={ok}/50  median={d}RTT")

# ---- E: BDP safety ----
hdr("TEST E  BDP safety: mr <= T_prop under persistent queue?")
for tp, q in [(1000, 200), (10000, 2000), (45000, 5000), (100000, 10000)]:
    s = 0
    for seed in range(100):
        g = Geo(tp, seed)
        for _ in range(5000):
            g.update(tp + q + g.rng.gauss(0, g.jtr))
        if g.mr <= tp:
            s += 1
    print(f"  T={tp:>7}u  q={q}u  safe={s}/100")

# ---- F: deadlock ----
hdr("TEST F  10x inflation deadlock recovery")
for tp in [1000, 45000, 100000]:
    ok, ds = 0, []
    for s in range(50):
        g = Geo(tp, s)
        g.mr = tp * 10
        g.x = tp * 10 * SCALE
        for i in range(1, 10000):
            g.update(tp + g.rng.gauss(0, g.jtr))
            if g.mr <= tp * 1.01:
                ok += 1
                ds.append(i)
                break
    d = int(sorted(ds)[len(ds) // 2]) if ds else 0
    print(f"  T={tp:>7}u  recovered={ok}/50  median={d}RTT")

# ---- G: long-term stability ----
hdr("TEST G  long-term stability (50k RTT, no PROBE_RTT)")
for tp in [1000, 45000, 100000, 1000000]:
    final = []
    for s in range(30):
        g = Geo(tp, s)
        for _ in range(50000):
            g.update(tp + g.rng.gauss(0, g.jtr))
        final.append(g.mr)
    mu, lo, hi = stat(final)
    print(f"  T={tp:>7}u  mr={mu:>8.0f}({pct(mu, tp):+.2f}%)  [{lo:.0f},{hi:.0f}]")

# ---- H: oscillation ----
hdr("TEST H  path oscillation +-10%/200RTT (time-averaged bias)")
for tp in [1000, 45000, 100000]:
    final = []
    for s in range(30):
        g = Geo(tp, s)
        hi, lo = int(tp * 1.10), int(tp * 0.90)
        mr_sum, n = 0.0, 0
        for cyc in range(25):
            tgt = hi if cyc % 2 == 0 else lo
            for _ in range(200):
                m, _, _ = g.update(tgt + g.rng.gauss(0, g.jtr))
                mr_sum += m
                n += 1
        final.append(mr_sum / n)  # time-averaged mr
    mu = stat(final)[0]
    print(f"  T={tp:>7}u  time_avg_mr_bias={pct(mu, tp):+.2f}%  (expect ~0%)")

# ---- I: Minimum detectable with slow path ----
hdr("TEST I  slow-path detection threshold sweep")
for tp in [1000, 45000, 100000]:
    row = []
    for a in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]:
        ok = 0
        ds = []
        for s in range(30):
            g = Geo(tp, s)
            for _ in range(1000):
                g.update(tp + g.rng.gauss(0, g.jtr))
            new = int(tp * (1 + a / 100))
            for i in range(1, 5000):
                _, _, f = g.update(new + g.rng.gauss(0, g.jtr))
                if f:
                    ok += 1
                    ds.append(i)
                    break
        d = int(sorted(ds)[len(ds) // 2]) if ds else 0
        row.append(f"{a}%:{ok}/{30}")
    print(f"  T={tp:>7}u  " + "  ".join(row))

print("\n" + "=" * 60)
print("  COMPLETE")
print("=" * 60)
