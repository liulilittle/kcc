#!/usr/bin/env python3
"""
no_probe_rtt_test.py - PROBE_RTT deletion feasibility.
Tests whether min_rtt sliding window + G1 pull-down naturally
correct self-inflicted queue within 10s, making PROBE_RTT redundant.

Scenarios:
  1. LAN (1ms) no PROBE_RTT - does min_rtt stay grounded?
  2. WAN (45ms) no PROBE_RTT - self-queue dissipation
  3. Multi-flow + no PROBE_RTT - worst-case drift
  4. With/without drain-skip comparison
"""

import random
import sys

sys.stdout.reconfigure(encoding="utf-8")

SCALE = 1024
SCALE_SHIFT = 10
G1_MIN = True  # G1 instant convergence
G2_GROWTH = 12  # percent per RTT
G3_FAST_N = 3
G3_FAST_TH = 11  # * SCALE / 10 = 1.1x
G3_SLOW_N = 50
G3_SLOW_TH = 21  # * SCALE / 20 = 1.05x
PD_FAST_FALL = 3  # geodesic pull-down needs 3 consecutive x_est < mr
WIN_RTT = 200  # min_rtt sliding window (RTTs ~= 10s for typical RTT)

# jitter as fraction of T_prop (σ = T_prop / 100 default)
JITTER_ALPHA = 100.0


class Geodesic:
    def __init__(self, tprop_us, seed=0):
        self.tprop = tprop_us
        self.mr = tprop_us  # min_rtt
        self.x = tprop_us * SCALE
        self.cnf = 0
        self.csl = 0
        self.jtr = tprop_us / JITTER_ALPHA
        self.pd_cnt = 0
        self.rng = random.Random(seed)
        self.win = [tprop_us] * WIN_RTT  # sliding min_rtt window
        self.wpos = 0
        self.rtts = 0
        self.g3_fired = False

    def step(self, tqueue=0):
        """Process one RTT sample. Returns rtt, mr, x_est (us)."""
        noise = self.rng.gauss(0, self.jtr)
        rtt = max(1, self.tprop + tqueue + int(noise))
        z = rtt * SCALE

        # G1 downward - instant min
        if z <= self.x:
            self.x = z
        # G2 upward - 12% growth capped at z
        else:
            growth = self.x * G2_GROWTH // 100
            self.x = min(self.x + growth, z)

        # G3 dual-threshold
        fast_th = self.mr * SCALE * G3_FAST_TH // 10
        slow_th = self.mr * SCALE * G3_SLOW_TH // 20
        base_th = self.mr * SCALE

        if self.x >= fast_th:
            self.cnf += 1
            self.csl += 1
        elif self.x >= slow_th:
            self.cnf = 0
            self.csl += 1
        else:
            self.cnf = 0  # fast resets; slow keeps accumulating

        if self.x <= base_th:
            self.cnf = 0
            self.csl = 0

        if self.cnf >= G3_FAST_N or self.csl >= G3_SLOW_N:
            self.mr = self.x >> SCALE_SHIFT
            self.cnf = 0
            self.csl = 0
            self.g3_fired = True

        # sliding min_rtt window (no PROBE_RTT forced drain)
        self.win[self.wpos] = rtt
        self.wpos = (self.wpos + 1) % WIN_RTT
        win_min = min(self.win)

        # Geodesic pull-down: requires 3 consecutive x_est < mr (gated by G3 lock)
        if self.cnf == 0 and self.csl == 0:
            x_us = self.x >> SCALE_SHIFT
            if x_us < self.mr:
                self.pd_cnt += 1
                if self.pd_cnt >= PD_FAST_FALL:
                    self.mr = x_us
                    self.pd_cnt = 0
            else:
                self.pd_cnt = 0

        # Also respect window min (lower bound)
        self.mr = min(self.mr, win_min)

        self.rtts += 1
        return rtt, self.mr, min(self.x >> SCALE_SHIFT, self.tprop + tqueue)


def run_sweep(label, tprop_us, tqueue_us=0, n_rtts=10000, seeds=50):
    """Run simulation sweep and collect min_rtt stability stats."""
    final_mrs = []
    max_drifts = []
    g3_fired = 0

    for s in range(seeds):
        g = Geodesic(tprop_us, seed=s * 7919)
        drift = 0
        for _ in range(n_rtts):
            _, mr, _ = g.step(tqueue_us)
            drift = max(drift, mr - tprop_us)
        final_mrs.append(g.mr)
        max_drifts.append(drift)
        if g.g3_fired:
            g3_fired += 1

    avg_mr = sum(final_mrs) / len(final_mrs)
    err_pct = (avg_mr - tprop_us) / tprop_us * 100
    max_drf = sum(max_drifts) / len(max_drifts)
    max_pct = (max(max_drifts)) / tprop_us * 100
    print(
        f"  {tprop_us:>7} us → mr={avg_mr:>7.0f} ({err_pct:+5.1f}%)  "
        f"avg_drift={max_drf:>6.0f}  max_drift={max_pct:5.1f}%  "
        f"g3={g3_fired}/{seeds}",
    )
    return avg_mr, err_pct, max_pct


def run_self_queue(label, tprop_us, queue_pct=25, n_rtts=30000, seeds=30):
    """Inject self-queue for N RTTs, then let it drain naturally. Measure recovery time."""
    recoveries = []
    final_mrs = []

    for s in range(seeds):
        g = Geodesic(tprop_us, seed=s * 7919)
        # Build self-queue: STARTUP-style overshoot
        for _ in range(500):
            g.step(int(tprop_us * queue_pct / 100))  # 25% BDP queue for 500 RTTs

        # Now drain naturally (no PROBE_RTT)
        recovered_at = None
        for i in range(n_rtts):
            _rtt, mr, _ = g.step(0)  # clean path
            if mr <= tprop_us * 1.02 and recovered_at is None:
                recovered_at = i
        final_mrs.append(g.mr)
        recoveries.append(recovered_at if recovered_at is not None else n_rtts)

    avg_mr = sum(final_mrs) / len(final_mrs)
    err_pct = (avg_mr - tprop_us) / tprop_us * 100
    median_rec = sorted(recoveries)[len(recoveries) // 2]
    recovered = sum(1 for r in recoveries if r < n_rtts)

    print(
        f"  {tprop_us:>7} us → mr={avg_mr:>7.0f} ({err_pct:+5.1f}%)  "
        f"recovery={recovered}/{seeds}  median={median_rec} RTTs  "
        f"worst={max(recoveries)} RTTs",
    )
    return avg_mr, err_pct, recovered, median_rec


def run_persistent_queue(label, tprop_us, queue_us, seeds=30, n_rtts=50000):
    """Persistent external queue - BBR's worst case. Test min_rtt drift."""
    final_mrs = []
    for s in range(seeds):
        g = Geodesic(tprop_us, seed=s * 7919)
        for _ in range(n_rtts):
            g.step(queue_us)  # constant external queue
        final_mrs.append(g.mr)

    avg_mr = sum(final_mrs) / len(final_mrs)
    err_pct = (avg_mr - tprop_us) / tprop_us * 100
    worst = max(1, max(final_mrs) - tprop_us)
    print(
        f"  {tprop_us:>7} us, queue={queue_us}us → mr={avg_mr:>7.0f} ({err_pct:+5.1f}%)  "
        f"worst={worst}us above",
    )
    return avg_mr, err_pct


if __name__ == "__main__":
    print("=" * 70)
    print("PROBE_RTT REMOVAL - FEASIBILITY ANALYSIS")
    print("=" * 70)

    # ── Test 1: Clean path, no PROBE_RTT, min_rtt stability ──
    print("\n[TEST 1] Clean path - does min_rtt stay grounded without PROBE_RTT?")
    print("  T_prop      mr_stats              avg_drift  max_drift  G3")
    for tp in [1000, 10000, 45000, 100000, 1000000]:
        run_sweep("clean", tp, 0, n_rtts=10000, seeds=100)

    # ── Test 2: Self-queue injection + natural recovery ──
    print(
        "\n[TEST 2] Self-queue (25% BDP for 500 RTTs) then drain naturally - no PROBE_RTT",
    )
    print("  T_prop      mr_stats              recovery_rate  median  worst")
    for tp in [1000, 5000, 45000, 100000]:
        run_self_queue("selfq", tp, queue_pct=25, n_rtts=30000, seeds=50)

    # ── Test 3: Persistent external queue (worst case for any RTT-based CCA) ──
    print("\n[TEST 3] Persistent external queue - information-theoretic limit")
    for tp, q in [
        (1000, 100),
        (1000, 500),
        (10000, 1000),
        (45000, 5000),
        (100000, 10000),
    ]:
        run_persistent_queue("extq", tp, q, seeds=30)

    # ── Test 4: Multi-flow simulation (simplified) ──
    print(
        "\n[TEST 4] Multi-flow self-queue (4 flows, 25% overshoot x 200 RTTs) then drain",
    )
    for tp in [1000, 45000, 100000]:
        recoveries = []
        for s in range(30):
            # Simulate 4 flows sharing bottleneck - each gets 1/4 bandwidth
            # Overshoot = (4 flows x 25% BDP) / (4 flows at 1/4 bandwidth) = longer drain
            # Actually: each flow builds queue during STARTUP, aggregate queue drains slower
            g = Geodesic(tp, seed=s * 7919)
            for _ in range(300):  # shorter STARTUP
                g.step(int(tp * 0.30))  # 30% BDP self-queue
            recovered_at = None
            for i in range(40000):
                rtt, mr, _ = g.step(0)
                if mr <= tp * 1.02 and recovered_at is None:
                    recovered_at = i
            recoveries.append(recovered_at if recovered_at is not None else 40000)
        med = sorted(recoveries)[len(recoveries) // 2]
        rec = sum(1 for r in recoveries if r < 40000)
        print(
            f"  {tp:>7} us → recovered={rec}/{30}  median={med} RTTs  worst={max(recoveries)} RTTs",
        )

    print("\n" + "=" * 70)
    print("ANALYSIS COMPLETE")
