#!/usr/bin/env python3
"""
KCC Geodesic Estimator — Maximum Convergence Time Simulation
=============================================================
Exhaustive sweep across physical propagation delay scales (500 us – 1000 ms),
noise levels, initial conditions, and path-change magnitudes.

Reports per-scenario convergence ACKs, RTTs, G3 detection latency,
and identifies the GLOBAL WORST CASE for each scale.

Algorithm (from tcp_kcc.c):
  G1: innovation <= 0  → x_est = min(x_est, z)
  G2: innovation  > 0  → x_est = min(x_est + x_est*122/1000, z)
  G3: fast (1.10x * 4 consecutive) / slow (1.05x * 5 cumulative)
  Baseline return: x_est <= min_rtt → reset both confirm counters
  Running min update + geodesic pull-down

Usage:
  python kcc_geodesic_convergence.py

Output:
  console summary tables + CSV files (.research/sim/)

Author: KCC v2.0 validation suite
"""

import math, csv, os, sys, random
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

# ── KCC constants ────────────────────────────────────────────────────
KCC_SCALE             = 1024
KCC_G2_GROWTH_NUM     = 122
KCC_G2_GROWTH_DEN     = 1000
KCC_G3_FAST_TH_NUM    = 11
KCC_G3_FAST_TH_DEN    = 10
KCC_G3_SLOW_TH_NUM    = 21
KCC_G3_SLOW_TH_DEN    = 20
KCC_G3_FAST_CNT       = 4
KCC_G3_SLOW_CNT       = 5
KCC_MINRTT_FAST_FALL_CNT = 5
KCC_MINRTT_FAST_FALL_DIV = 4
KCC_MINRTT_STICKY_NUM = 75
KCC_MINRTT_STICKY_DEN = 100
KCC_PD_NOISE_GATE_NUM = 95
KCC_PD_NOISE_GATE_DEN = 100
KCC_RTT_MIN_FLOOR_US  = 1
KCC_MIN_SAMPLES       = 5
KCC_STALENESS_RNDS    = 128
KCC_BITFIELD_3BIT_MAX = 7
KCC_JITTER_SEED_DIV   = 4

# ── Sweep dimensions ─────────────────────────────────────────────────
T_PROP_US = [
    500,          # 0.5 ms   DC / campus
    1_000,        # 1   ms   metro
    5_000,        # 5   ms   regional
    10_000,       # 10  ms   continental
    50_000,       # 50  ms   trans-oceanic
    100_000,      # 100 ms   long-haul
    500_000,      # 500 ms   GEO satellite
    1_000_000,    # 1 000 ms extreme
]

NOISE_PCT = [0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0]

# Over-estimate: G1 pulls down instantly (1 ACK)
OVERSHOOT = [2.0, 5.0, 10.0, 50.0, 100.0]

# Under-estimate: G2 grows geometrically (costs N RTTs)
UNDERSHOOT = [0.001, 0.01, 0.1, 0.25, 0.5, 0.75]

# Path increase factors for G3 latency test
PATH_FACTORS = [1.05, 1.10, 1.25, 1.50, 2.0, 5.0, 10.0, 20.0]

MAX_ACKS = 10000
CONVERGE_RATIO = 0.02  # 2% relative error

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)))
os.makedirs(OUT_DIR, exist_ok=True)
random.seed(42)


@dataclass
class GeodesicState:
    """Mirrors struct kcc + kcc_ext estimator fields."""
    x_est: int = 0
    min_rtt_us: int = 10000
    sample_cnt: int = 0
    confirm_cnt: int = 0
    confirm_slow_cnt: int = 0
    mr_update_rtt_cnt: int = 0
    rtt_cnt: int = 0
    round_start: bool = False
    min_rtt_fast_fall_cnt: int = 0
    true_min_rtt_us: int = 10000
    # Tracking
    converged_ack: Optional[int] = None
    minrtt_raised_ack: Optional[int] = None


def g1_g2(state: GeodesicState, rtt_us: int) -> bool:
    """G1/G2 geodesic update. Returns True if x_est changed significantly."""
    rtt_us = max(rtt_us, KCC_RTT_MIN_FLOOR_US)
    z = rtt_us * KCC_SCALE

    if state.sample_cnt == 0:
        state.x_est = z
        state.sample_cnt = 1
        return True

    if state.sample_cnt == 1 and state.min_rtt_us:
        ceiling = state.min_rtt_us * KCC_SCALE
        if state.x_est > ceiling:
            state.x_est = ceiling

    innovation = z - state.x_est
    if innovation <= 0:
        state.x_est = min(state.x_est, z)              # G1
    else:
        growth = state.x_est * KCC_G2_GROWTH_NUM // KCC_G2_GROWTH_DEN
        state.x_est = min(state.x_est + growth, z)     # G2

    # Staleness guard
    if (state.rtt_cnt - state.mr_update_rtt_cnt >= KCC_STALENESS_RNDS and
            state.x_est <= state.min_rtt_us * KCC_SCALE * KCC_G3_FAST_TH_NUM // KCC_G3_FAST_TH_DEN):
        state.x_est = (state.min_rtt_us * KCC_SCALE *
                       KCC_PD_NOISE_GATE_NUM // KCC_PD_NOISE_GATE_DEN)
        state.mr_update_rtt_cnt = state.rtt_cnt

    state.sample_cnt += 1
    return True


def g3_update(state: GeodesicState, rtt_us: int) -> bool:
    """G3 confirm + min_rtt running update. Returns True if min_rtt changed."""
    mr = state.min_rtt_us
    x = state.x_est
    thr_fast = mr * KCC_SCALE * KCC_G3_FAST_TH_NUM // KCC_G3_FAST_TH_DEN
    thr_slow = mr * KCC_SCALE * KCC_G3_SLOW_TH_NUM // KCC_G3_SLOW_TH_DEN
    bline     = mr * KCC_SCALE

    # G3: accumulate / reset
    if x >= thr_fast:
        if state.confirm_cnt < 255:   state.confirm_cnt += 1
        if state.confirm_slow_cnt < 255: state.confirm_slow_cnt += 1
    elif x >= thr_slow:
        state.confirm_cnt = 0
        if state.confirm_slow_cnt < 255: state.confirm_slow_cnt += 1
    else:
        state.confirm_cnt = 0

    if x <= bline:
        state.confirm_cnt = 0
        state.confirm_slow_cnt = 0

    # G3 commit
    if state.confirm_cnt >= KCC_G3_FAST_CNT:
        state.min_rtt_us = max(x // KCC_SCALE, KCC_RTT_MIN_FLOOR_US)
        state.confirm_cnt = 0
        state.confirm_slow_cnt = 0
        state.mr_update_rtt_cnt = state.rtt_cnt
        return True
    if state.confirm_slow_cnt >= KCC_G3_SLOW_CNT:
        state.min_rtt_us = max(x // KCC_SCALE, KCC_RTT_MIN_FLOOR_US)
        state.confirm_cnt = 0
        state.confirm_slow_cnt = 0
        state.mr_update_rtt_cnt = state.rtt_cnt
        return True

    # Lock during accumulation
    if state.confirm_cnt > 0 or state.confirm_slow_cnt > 0:
        return False

    # Running minimum (sticky / fast-fall)
    if rtt_us <= state.min_rtt_us:
        rtt_c = max(rtt_us, KCC_RTT_MIN_FLOOR_US)
        if rtt_c < state.min_rtt_us * KCC_MINRTT_STICKY_NUM // KCC_MINRTT_STICKY_DEN:
            if rtt_c < state.min_rtt_us // KCC_MINRTT_FAST_FALL_DIV:
                state.min_rtt_us = rtt_c
                state.min_rtt_fast_fall_cnt = 0
            else:
                state.min_rtt_fast_fall_cnt = min(
                    state.min_rtt_fast_fall_cnt + 1, KCC_BITFIELD_3BIT_MAX)
                if state.min_rtt_fast_fall_cnt >= KCC_MINRTT_FAST_FALL_CNT:
                    state.min_rtt_us = rtt_c
                    state.min_rtt_fast_fall_cnt = 0
                elif state.round_start:
                    state.min_rtt_us = max(KCC_RTT_MIN_FLOOR_US,
                        state.min_rtt_us * KCC_MINRTT_STICKY_NUM // KCC_MINRTT_STICKY_DEN)
        else:
            state.min_rtt_us = rtt_c
            state.min_rtt_fast_fall_cnt = 0

    # Geodesic pull-down
    if state.sample_cnt >= KCC_MIN_SAMPLES and state.x_est:
        krtt = state.x_est // KCC_SCALE
        if (krtt < state.min_rtt_us and
                krtt < state.min_rtt_us * KCC_PD_NOISE_GATE_NUM // KCC_PD_NOISE_GATE_DEN):
            state.min_rtt_us = krtt
            state.mr_update_rtt_cnt = state.rtt_cnt

    return state.min_rtt_us != mr


def run_scenario(t_prop_us: int, noise_pct: float,
                 init_factor: float, warm_start: bool,
                 path_change_ack: Optional[int] = None,
                 path_factor: float = 1.0,
                 queue_rtt_us: float = 0.0,
                 max_acks: int = MAX_ACKS) -> dict:
    """
    Single scenario, returns metrics dict.
    warm_start=True → skip cold-start (init_factor takes effect immediately).
    """
    st = GeodesicState()
    st.true_min_rtt_us = t_prop_us
    st.min_rtt_us = t_prop_us
    # Set initial x_est
    if init_factor is not None:
        st.x_est = max(int(t_prop_us * KCC_SCALE * init_factor), 1)
    else:
        st.x_est = t_prop_us * KCC_SCALE
    if warm_start:
        st.sample_cnt = KCC_MIN_SAMPLES  # bypass cold-start init

    rtt_count = 0
    path_changed = False
    new_t_prop = t_prop_us
    sigma = t_prop_us * noise_pct / 100.0

    # Per-step log
    steps = []

    for ack in range(max_acks):
        # Path change
        if path_change_ack is not None and ack >= path_change_ack and not path_changed:
            new_t_prop = int(t_prop_us * path_factor)
            st.true_min_rtt_us = new_t_prop
            sigma = new_t_prop * noise_pct / 100.0
            path_changed = True

        # Round boundary: every 20 packets (ACK-clocked approximation)
        round_start = (ack > 0 and ack % 20 == 0)
        if round_start:
            rtt_count += 1
            st.rtt_cnt += 1
        st.round_start = round_start

        # Generate RTT sample = T_prop + T_queue + T_noise
        noise = random.gauss(0, sigma)
        qdelay = queue_rtt_us if (path_changed and ack >= (path_change_ack or 999999)) else 0.0
        rtt_sample = int(max(new_t_prop + qdelay + noise, KCC_RTT_MIN_FLOOR_US))

        # G1/G2
        g1_g2(st, rtt_sample)

        # G3 + min_rtt
        g3_update(st, rtt_sample)

        # Convergence check: |x_est - true_min| / true_min <= CONVERGE_RATIO
        if st.converged_ack is None and st.sample_cnt >= KCC_MIN_SAMPLES:
            x_us = st.x_est // KCC_SCALE
            if x_us > 0:
                err = abs(x_us - new_t_prop) / new_t_prop
                if err <= CONVERGE_RATIO:
                    st.converged_ack = ack

        # G3 detection: min_rtt raised to >= 95% of new T_prop
        if path_changed and st.minrtt_raised_ack is None:
            if st.min_rtt_us >= new_t_prop * 95 // 100 and st.confirm_cnt == 0 and st.confirm_slow_cnt == 0:
                st.minrtt_raised_ack = ack

        # Log every 10% of the simulation for diagnostics
        if ack < 500 or ack % 500 == 0 or (path_changed and ack < path_change_ack + 100):
            if ack < 2000:  # cap log size
                steps.append({
                    'ack': ack, 'rtt_us': rtt_sample,
                    'x_est_us': st.x_est // KCC_SCALE,
                    'min_rtt_us': st.min_rtt_us,
                    'true_tprop': new_t_prop,
                    'conf': st.confirm_cnt,
                    'conf_slow': st.confirm_slow_cnt,
                    'err_pct': abs((st.x_est // KCC_SCALE) - new_t_prop) / new_t_prop * 100,
                })

    x_us = st.x_est // KCC_SCALE
    return {
        't_prop_us': t_prop_us,
        'noise_pct': noise_pct,
        'init_factor': init_factor,
        'warm_start': warm_start,
        'path_change_ack': path_change_ack,
        'path_factor': path_factor,
        'converged_ack': st.converged_ack if st.converged_ack is not None else max_acks,
        'converged_rtt': rtt_count if st.converged_ack is not None else -1,
        'detection_ack': st.minrtt_raised_ack if st.minrtt_raised_ack is not None else (-1 if path_change_ack is None else max_acks),
        'detection_delta_ack': (st.minrtt_raised_ack - path_change_ack) if (path_change_ack is not None and st.minrtt_raised_ack is not None) else -1,
        'final_x_est_us': x_us,
        'final_min_rtt_us': st.min_rtt_us,
        'true_min_rtt_us': new_t_prop,
        'final_err_pct': abs(x_us - new_t_prop) / new_t_prop * 100,
        'acks_total': ack + 1,
        'final_confirm_cnt': st.confirm_cnt,
        'final_confirm_slow_cnt': st.confirm_slow_cnt,
        'steps': steps,
    }


# ── Sweep functions ──────────────────────────────────────────────────

def sweep_g1_overshoot() -> List[dict]:
    """G1: over-estimate convergence (should be ~1 ACK)."""
    results = []
    for tp in T_PROP_US:
        for n in NOISE_PCT:
            for f in OVERSHOOT:
                r = run_scenario(tp, n, f, warm_start=False)
                results.append(r)
    return results


def sweep_g2_undershoot() -> List[dict]:
    """G2: under-estimate convergence (geometric growth, most costly)."""
    results = []
    for tp in T_PROP_US:
        for n in NOISE_PCT:
            for f in UNDERSHOOT:
                r = run_scenario(tp, n, f, warm_start=True)
                results.append(r)
    return results


def sweep_g3_path_change() -> List[dict]:
    """G3: path-increase detection latency."""
    results = []
    for tp in T_PROP_US:
        for n in [1.0, 5.0]:
            for f in PATH_FACTORS:
                r = run_scenario(tp, n, 1.0, warm_start=True,
                                 path_change_ack=500, path_factor=f)
                results.append(r)
    return results


def sweep_false_positive() -> List[dict]:
    """
    G3 false-positive audit: run pure-noise scenarios (NO path change)
    and check whether G3 ever raises min_rtt.
    """
    results = []
    for tp in T_PROP_US:
        for n in NOISE_PCT:
            for init_f in [1.0, 5.0, 10.0]:
                for trial in range(5):  # 5 trials each for statistical significance
                    r = run_scenario(tp, n, init_f, warm_start=(init_f > 1.0),
                                     path_change_ack=None, path_factor=1.0,
                                     max_acks=5000)
                    # Check for false positive: did min_rtt increase via G3
                    # (confirmed != baseline at end means G3 was accumulating)
                    r['trial'] = trial
                    results.append(r)
    return results


def sweep_queue_assisted() -> List[dict]:
    """G2 convergence under queue pressure (T_queue > 0)."""
    results = []
    for tp in T_PROP_US:
        for n in [1.0, 5.0]:
            for q in [0.05, 0.10, 0.25]:  # queue as fraction of T_prop
                for f in [0.1, 0.5]:
                    r = run_scenario(tp, n, f, warm_start=True, queue_rtt_us=tp * q)
                    results.append(r)
    return results


def theoretical_g2_acks(tprop: int, init_factor: float, noise_pct: float) -> int:
    """Theoretical minimum ACKs for G2 to grow from init_factor to 1.0."""
    if init_factor >= 1.0:
        return KCC_MIN_SAMPLES
    ratio = 1.0 / max(init_factor, 0.001)
    # Growth per ACK: 12.2%. Need N such that (1.122)^N >= ratio
    import math
    steps = math.ceil(math.log(ratio) / math.log(1.0 + KCC_G2_GROWTH_NUM / KCC_G2_GROWTH_DEN))
    return max(steps + 2, KCC_MIN_SAMPLES)  # +2 for cold-start guard


def theoretical_g3_acks(path_factor: float) -> int:
    """Theoretical minimum ACKs for G3 to detect path increase."""
    if path_factor <= 1.0:
        return 0
    # G2 must grow x_est from current (≈ old T_prop) to new T_prop * threshold
    g3_threshold = 1.10  # fast threshold
    ratio = max(g3_threshold, path_factor) / 1.0
    g2_steps = math.ceil(math.log(ratio) / math.log(1.0 + KCC_G2_GROWTH_NUM / KCC_G2_GROWTH_DEN))
    # Then 3 consecutive confirms
    total = g2_steps + KCC_G3_FAST_CNT + 1
    return total


# ── Reporting ─────────────────────────────────────────────────────────

def format_table(rows: List[dict], keys: List[str], headers: List[str],
                 title: str, sort_key: str = 'converged_ack'):
    """Print aligned summary table."""
    if not rows:
        return
    # Find worst-case per T_prop
    by_tp = {}
    for r in rows:
        tp = r['t_prop_us']
        by_tp.setdefault(tp, []).append(r)

    print(f"\n{'=' * 110}")
    print(f"  {title}")
    print(f"{'=' * 110}")
    # Header
    hdr = '  '.join(f'{h:>12}' for h in headers)
    print(f"  {hdr}")
    print(f"  {'-' * (len(hdr) + 4)}")

    for tp in sorted(by_tp):
        group = sorted(by_tp[tp], key=lambda x: x[sort_key])
        # Show min, max, avg for convergence
        acks = [r['converged_ack'] for r in group if r['converged_ack'] < MAX_ACKS]
        dets = [r.get('detection_delta_ack', -1) for r in group
                if r.get('detection_delta_ack', -1) >= 0]
        max_ack = max(acks) if acks else MAX_ACKS
        avg_ack = sum(acks) / len(acks) if acks else MAX_ACKS
        min_ack = min(acks) if acks else MAX_ACKS

        max_det = max(dets) if dets else -1
        avg_det = sum(dets) / len(dets) if dets else -1

        print(f"  {tp:>10}us  |  conv(min/avg/max): {min_ack:>4}/{avg_ack:>5.0f}/{max_ack:>4} ACKs  "
              f"|  det(max): {max_det:>4} ACKs  |  n={len(group)}")

    # Global worst-case
    all_acks = [r['converged_ack'] for r in rows if r['converged_ack'] < MAX_ACKS]
    if all_acks:
        wr = max(rows, key=lambda r: r['converged_ack'] if r['converged_ack'] < MAX_ACKS else -1)
        print(f"\n  >>> GLOBAL WORST: T_prop={wr['t_prop_us']}us  "
              f"noise={wr['noise_pct']}%  init={wr['init_factor']}x  "
              f"converged={wr['converged_ack']} ACKs")
    if dets:
        wr_d = max(rows, key=lambda r: r.get('detection_delta_ack', -1)
                   if r.get('detection_delta_ack', -1) >= 0 else -1)
        print(f"  >>> GLOBAL WORST DETECTION: T_prop={wr_d['t_prop_us']}us  "
              f"path_factor={wr_d['path_factor']}x  "
              f"delta={wr_d['detection_delta_ack']} ACKs")


def save_csv(results: List[dict], name: str):
    """Save summary (without step logs) to CSV."""
    if not results:
        return
    keys = [k for k in results[0].keys() if k != 'steps']
    path = os.path.join(OUT_DIR, name)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
        w.writeheader()
        for r in results:
            w.writerow({k: r[k] for k in keys})
    print(f"  [CSV: {path}]")


def print_theoretical_bounds():
    """Print theoretical bounds for G2 convergence and G3 detection."""
    print(f"\n{'=' * 110}")
    print(f"  THEORETICAL BOUNDS")
    print(f"{'=' * 110}")

    print(f"\n  G2 convergence (geometric growth, {KCC_G2_GROWTH_NUM}/{KCC_G2_GROWTH_DEN} = "
          f"{KCC_G2_GROWTH_NUM/KCC_G2_GROWTH_DEN:.1%} per ACK):")
    print(f"  {'init_factor':>12} {'steps':>8}")
    for f in [0.001, 0.01, 0.1, 0.25, 0.5, 0.75]:
        steps = theoretical_g2_acks(10000, f, 0)
        print(f"  {f:>12.3f} {steps:>8}")

    print(f"\n  G3 detection (fast path: 1.10x * 4 consecutive):")
    print(f"  {'path_factor':>12} {'theor(steps)':>14}")
    for pf in [1.05, 1.10, 1.25, 1.50, 2.0, 5.0, 10.0, 20.0]:
        steps = theoretical_g3_acks(pf)
        print(f"  {pf:>12.2f} {steps:>14}")


# ── Main ─────────────────────────────────────────────────────────────

def main():
    print("=" * 110)
    print("  KCC Geodesic Estimator — Max Convergence Time Simulation")
    print("  G1: instant downward  |  G2: +12.2%/ACK  |  G3: 1.10x*3 / 1.05x*4")
    print("  8 T_prop scales × 7 noise levels × diverse init conditions")
    print("=" * 110)

    print_theoretical_bounds()

    # ── Phase 1: G1 over-estimate ──
    print(f"\n{'#' * 110}")
    print(f"  PHASE 1: G1 OVER-ESTIMATE CONVERGENCE")
    r1 = sweep_g1_overshoot()
    format_table(r1, [], [], "G1 over-estimate convergence")
    save_csv(r1, 'phase01_g1_overshoot.csv')

    # ── Phase 2: G2 under-estimate ──
    print(f"\n{'#' * 110}")
    print(f"  PHASE 2: G2 UNDER-ESTIMATE CONVERGENCE (WORST CASE)")
    r2 = sweep_g2_undershoot()
    format_table(r2, [], [], "G2 under-estimate convergence")
    save_csv(r2, 'phase02_g2_undershoot.csv')

    # ── Phase 3: G3 path-change ──
    print(f"\n{'#' * 110}")
    print(f"  PHASE 3: G3 PATH-INCREASE DETECTION LATENCY")
    r3 = sweep_g3_path_change()
    format_table(r3, [], [], "G3 path-change detection",
                 sort_key='detection_delta_ack')

    # Per-T_prop detection detail
    by_tp = {}
    for r in r3:
        by_tp.setdefault(r['t_prop_us'], []).append(r)
    print(f"\n  Per-scale detection delta (ACKs from path change at ACK 500):")
    print(f"  {'T_prop':>10} ", end='')
    for pf in PATH_FACTORS:
        print(f'{pf:>7.2f}x', end='')
    print()
    for tp in sorted(by_tp):
        grp = {r['path_factor']: r['detection_delta_ack'] for r in by_tp[tp]}
        print(f"  {tp:>7}us  ", end='')
        for pf in PATH_FACTORS:
            d = grp.get(pf, -1)
            print(f'{d:>7}' if d >= 0 else f'{"—":>7}', end='')
        print()

    save_csv(r3, 'phase03_g3_path_detection.csv')

    # ── Phase 4: Queue-assisted ──
    print(f"\n{'#' * 110}")
    print(f"  PHASE 5: G3 FALSE-POSITIVE AUDIT (PURE NOISE, 5000 ACKs each)")
    print(f"  The geodesic G3 must NEVER raise min_rtt under pure noise.")
    r5 = sweep_false_positive()
    total = len(r5)
    fp = [r for r in r5 if r['final_confirm_cnt'] > 0 or r['final_confirm_slow_cnt'] > 0]
    g3_raises = [r for r in r5
        if r['final_min_rtt_us'] > r['true_min_rtt_us'] * 1.02
        and r['final_confirm_cnt'] == 0 and r['final_confirm_slow_cnt'] == 0
        and r['converged_ack'] < r['acks_total']]
    print(f"\n  Total scenarios: {total}")
    print(f"  G3 accumulators non-zero at end: {len(fp)}")
    if fp:
        # Stratify by noise
        for n in [0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0]:
            sub = [r for r in fp if abs(r['noise_pct'] - n) < 0.01]
            if sub:
                print(f"    noise={n:>4}%: {len(sub):>3} scenarios  "
                      f"(slow_cnt max={max(r['final_confirm_slow_cnt'] for r in sub)})")
        # Show worst few
        wr_fp = sorted(fp, key=lambda r: r['final_confirm_slow_cnt'], reverse=True)[:3]
        print(f"    Worst: {wr_fp[0]['t_prop_us']}us n={wr_fp[0]['noise_pct']}% "
              f"slow={wr_fp[0]['final_confirm_slow_cnt']}")
    else:
        print(f"  ZERO G3 accumulators active at end.")
    print(f"  G3 commits (final min_rtt > 1.02x true, counters clear): {len(g3_raises)}")
    if g3_raises:
        for n in sorted(set(r['noise_pct'] for r in g3_raises)):
            sub = [r for r in g3_raises if abs(r['noise_pct'] - n) < 0.01]
            tprops = sorted(set(r['t_prop_us'] for r in sub))
            print(f"    noise={n:>4}%: {len(sub):>3} raises across {tprops}")

    # Deep-dive: track min_rtt over time for worst-case false-positive params
    print(f"\n  Deep-dive: tracking min_rtt under noise (T_prop=500us, 5%, 50000 ACKs)")
    st_dd = GeodesicState()
    st_dd.true_min_rtt_us = 500
    st_dd.min_rtt_us = 500
    st_dd.x_est = 500 * KCC_SCALE
    st_dd.sample_cnt = KCC_MIN_SAMPLES
    sigma_dd = 500 * 0.05
    minrtt_samples = []
    for ack_dd in range(5000):
        ns = random.gauss(0, sigma_dd)
        rs = int(max(500 + ns, KCC_RTT_MIN_FLOOR_US))
        rd = (ack_dd > 0 and ack_dd % 20 == 0)
        if rd: st_dd.rtt_cnt += 1
        st_dd.round_start = rd
        g1_g2(st_dd, rs)
        g3_update(st_dd, rs)
        if ack_dd % 50 == 0:
            minrtt_samples.append(st_dd.min_rtt_us)
    overs = [m for m in minrtt_samples if m > 525]
    print(f"    Final min_rtt: {st_dd.min_rtt_us}us  |  "
          f"Samples > 525us (5% above true): {len(overs)}/{len(minrtt_samples)}"
          f"  |  Max excursion: {max(minrtt_samples)}us")
    save_csv(r5, 'phase05_false_positive.csv')
    r4 = sweep_queue_assisted()
    format_table(r4, [], [], "G2 + queue convergence")
    save_csv(r4, 'phase04_g2_queue.csv')

    # ── Combined worst-case analysis ──
    print(f"\n{'#' * 110}")
    print(f"  SUMMARY: MAXIMUM CONVERGENCE TIME BY SCALE")
    print(f"{'=' * 110}")
    print(f"  {'Scale':>10} {'Worst init':>12} {'Worst conv(acks)':>18} "
          f"{'Worst conv(rtt)':>16} {'G3 worst(delta)':>16}")
    all_scenarios = r1 + r2 + r4
    for tp in T_PROP_US:
        pool = [r for r in all_scenarios if r['t_prop_us'] == tp]
        wr = max(pool, key=lambda r: r['converged_ack'])
        # G3 worst for this scale
        g3pool = [r for r in r3 if r['t_prop_us'] == tp]
        wr3 = max(g3pool, key=lambda r: r['detection_delta_ack']) if g3pool else None
        d3 = wr3['detection_delta_ack'] if wr3 and wr3['detection_delta_ack'] >= 0 else '—'
        print(f"  {tp:>7}us   {wr['init_factor']:>7.3f}x   {wr['converged_ack']:>8} ACKs  "
              f"{'—':>12}  {str(d3):>8} ACKs")

    print(f"\n{'=' * 110}")
    print(f"  Done.  CSV files in {OUT_DIR}")
    print(f"{'=' * 110}")


if __name__ == '__main__':
    main()
