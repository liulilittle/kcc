# KCC 2.0 P-controller: PROBE_BW cruise phase simulation
# Focused analysis - no STARTUP/DRAIN, just steady-state with cross-traffic changes
# py -3 this.py
import random, math, sys, json
from collections import deque

MSS = 1448
BW_Mbps = 1260.0
BW_bps = BW_Mbps * 1e6
BD_BYTE_PER_S = BW_bps / 8
T_PROP_US = 35000
T_PROP_S = T_PROP_US * 1e-6
BDP_bytes = BW_bps * T_PROP_S / 8
BDP_pkts = BDP_bytes / MSS

BBR_UNIT = 256
CWND_GAIN = BBR_UNIT * 2
CYCLE = [int(BBR_UNIT * 1.25), int(BBR_UNIT * 0.75)] + [BBR_UNIT] * 6
GAIN_MIN = int(BBR_UNIT * 0.75)
GAIN_MAX = int(BBR_UNIT * 1.25)

P_TARGET_SCALED = 0
P_SLOPE_NUM = 1
P_SLOPE_DEN = 4
P_ADJUST_MAX = BBR_UNIT // 20

IRQ_MAX_US = 50.0
TSO_BURST_PKTS = 45
ACK_RATIO = 2

N_ROUNDS = 1000  # per seed
N_SEEDS = 50
TOTAL_SCENARIOS = 3

def p_control(tprop_us, round_qdelay_us):
    """P-controller: excess queue -> gain adjustment."""
    if tprop_us == 0:
        return BBR_UNIT
    excess_us = max(0, int(round_qdelay_us) - int(tprop_us))
    pressure_scaled = (excess_us << 10) // tprop_us
    error = P_TARGET_SCALED - pressure_scaled
    adjust = (error * P_SLOPE_NUM) // P_SLOPE_DEN
    adjust = max(-P_ADJUST_MAX, min(adjust, P_ADJUST_MAX))
    return max(GAIN_MIN, min(BBR_UNIT + adjust, GAIN_MAX))

def simulate_one(params):
    seed, n_flows, scenario = params
    rng = random.Random(seed)

    # Flows with slightly different base RTTs
    base_rtts = [T_PROP_US + rng.randint(-5000, 5000) for _ in range(n_flows)]
    base_rtts = [max(5000, br) for br in base_rtts]

    # Per-flow: pacing_gain starts at BBR_UNIT, cycle_idx starts random
    pgains = [BBR_UNIT for _ in range(n_flows)]
    cycle_idx = [rng.randint(0, 7) for _ in range(n_flows)]
    cwnd_pkts = [BDP_pkts * 2.0 for _ in range(n_flows)]  # start at target
    min_rtts = [T_PROP_US for _ in range(n_flows)]  # geodesic output

    # Stats
    qdelay_history = []
    gain_history = [[] for _ in range(n_flows)]
    rate_history = [[] for _ in range(n_flows)]

    for round_n in range(N_ROUNDS):
        # Cross-traffic: determines extra queue pressure on the bottleneck
        if scenario == 'alone':
            cross_mult = 1.0
        elif scenario == 'step':
            # Cross traffic enters at round 300, leaves at round 700
            cross_mult = 2.0 if 300 <= round_n < 700 else 1.0
        elif scenario == 'ramp':
            # Cross traffic ramps up and down
            if round_n < 200: cross_mult = 1.0
            elif round_n < 400: cross_mult = 1.0 + (round_n - 200) / 200.0
            elif round_n < 600: cross_mult = 2.0
            elif round_n < 800: cross_mult = 2.0 - (round_n - 600) / 200.0
            else: cross_mult = 1.0

        # Each flow: compute actual queue = BDP * pacing_gain_ratio * cross_mult - BDP
        total_inflight_pkts = 0
        for i in range(n_flows):
            inflight = BDP_pkts * (CWND_GAIN / BBR_UNIT) * (pgains[i] / BBR_UNIT)
            total_inflight_pkts += inflight
            cwnd_pkts[i] = inflight  # instant convergence

        total_inflight_bytes = total_inflight_pkts * MSS
        cross_bytes = BDP_bytes * (cross_mult - 1.0)
        total_bytes = total_inflight_bytes + cross_bytes
        queue_bytes = max(0.0, total_bytes - BDP_bytes)
        queue_s = queue_bytes / BD_BYTE_PER_S
        queue_us = queue_s * 1e6

        # Per-flow: construct per-round RTT min (with noise)
        for i in range(n_flows):
            n_samples = max(5, int(cwnd_pkts[i] / ACK_RATIO / 4))
            round_samples = []
            for _ in range(n_samples):
                tso_delay = rng.randint(1, TSO_BURST_PKTS) * MSS / BD_BYTE_PER_S
                noise = rng.uniform(0, IRQ_MAX_US * 1e-6)
                sample = base_rtts[i] * 1e-6 + queue_s + tso_delay + noise
                round_samples.append(sample)

            round_min_s = min(round_samples)
            round_qdelay_s = max(0.0, round_min_s - base_rtts[i] * 1e-6)
            round_qdelay_us = round_qdelay_s * 1e6

            # Update geodesic (simplified: directly update min_rtt from clean samples)
            clean_sample = base_rtts[i] * 1e-6 + queue_s  # ground truth RTT
            if clean_sample < min_rtts[i] * 1e-6:
                min_rtts[i] = int(clean_sample * 1e6)

            # Cycle advance
            cycle_idx[i] = (cycle_idx[i] + 1) & 7
            phase_gain = CYCLE[cycle_idx[i] & 7]

            # P-controller: only during cruise (phase_gain == BBR_UNIT)
            if phase_gain == BBR_UNIT:
                pgains[i] = p_control(min_rtts[i], int(round_qdelay_us))
            else:
                pgains[i] = phase_gain

            gain_history[i].append(pgains[i] / BBR_UNIT)

        qdelay_history.append(queue_us)

        # Rate estimate per flow
        for i in range(n_flows):
            rate = cwnd_pkts[i] * MSS * 8 / (base_rtts[i] * 2e-6 + queue_us) / 1e6  # Mbps
            rate_history[i].append(rate)

    # Metrics
    steady_start = 200  # skip first 200 rounds for steady-state analysis

    qdelays = qdelay_history[steady_start:]
    qdelays.sort()
    n_q = len(qdelays)
    q_mean = sum(qdelays) / n_q if n_q else 0
    q_p50 = qdelays[n_q // 2] if n_q else 0
    q_p95 = qdelays[min(n_q - 1, int(n_q * 0.95))] if n_q else 0
    q_p99 = qdelays[min(n_q - 1, int(n_q * 0.99))] if n_q else 0

    # Gain stats
    all_gains = []
    for g in gain_history:
        all_gains.extend(g[steady_start:])
    g_mean = sum(all_gains) / len(all_gains) if all_gains else 1.0
    g_std = math.sqrt(sum((x - g_mean)**2 for x in all_gains) / len(all_gains)) if all_gains else 0

    # Rate stats
    flow_rates = []
    for rh in rate_history:
        flow_rates.append(sum(rh[steady_start:]) / len(rh[steady_start:]) if rh[steady_start:] else 0)

    total_rate = sum(flow_rates)
    n_f = len(flow_rates)
    if n_f > 1 and sum(r*r for r in flow_rates) > 0:
        jain = (total_rate ** 2) / (n_f * sum(r*r for r in flow_rates))
    else:
        jain = 1.0

    # Phase breakdown
    phases = {}
    for label, start, end in [
        ('alone_start', 200, 300),
        ('cross_mid', 400, 600),
        ('alone_end', 800, 1000),
    ]:
        if scenario == 'alone':
            ph_q = qdelay_history[start:end]
        elif scenario == 'step':
            ph_q = qdelay_history[start:end]
        else:
            ph_q = qdelay_history[start:end]

        if ph_q:
            phases[label + '_q_mean'] = sum(ph_q) / len(ph_q)
            ph_gs = []
            for g in gain_history:
                ph_gs.extend(g[start:end])
            if ph_gs:
                phases[label + '_g_mean'] = sum(ph_gs) / len(ph_gs)

    return {
        'seed': seed, 'scenario': scenario,
        'q_mean': q_mean, 'q_p50': q_p50, 'q_p95': q_p95, 'q_p99': q_p99,
        'g_mean': g_mean, 'g_std': g_std,
        'total_rate': total_rate, 'jain': jain,
        'flow_rates': flow_rates,
        'phases': phases,
    }

def run_all():
    print("=" * 80)
    print("KCC 2.0 P-Controller: PROBE_BW Cruise Phase Simulation")
    print(f"  T_prop={T_PROP_US}us  BW={BW_Mbps}Mbps  BDP={BDP_pkts:.0f}pkts  CWND_GAIN=2x")
    print(f"  P: target={P_TARGET_SCALED}/1024  slope={P_SLOPE_NUM}/{P_SLOPE_DEN}  ±{P_ADJUST_MAX/BBR_UNIT*100:.0f}%")
    print(f"  {N_FLOWS} flows  {N_ROUNDS} rounds  {N_SEEDS} seeds")
    print("=" * 80)

    for nf in [1, 3]:
        for scenario in ['alone', 'step', 'ramp']:
            print(f"\n--- {nf} flow(s), {scenario} ---")

            params = [(42 + s, nf, scenario) for s in range(N_SEEDS)]
            results = []
            for p in params:
                results.append(simulate_one(p))

            n_ok = len(results)

            def st(arr, label=''):
                s = sorted(arr)
                n = len(s)
                return f"{sum(s)/n:>8.1f} {s[n//2]:>8.1f} {s[max(0,n//20)]:>8.1f} {s[min(n-1,n*19//20)]:>8.1f}"

            print(f"  {'Metric':<18} {'Mean':>8} {'P50':>8} {'P5':>8} {'P95':>8}")
            print(f"  {'-'*18} {'-'*8} {'-'*8} {'-'*8} {'-'*8}")

            for name, key in [
                ('Queue_us', 'q_mean'), ('Q_P50', 'q_p50'), ('Q_P95', 'q_p95'), ('Q_P99', 'q_p99'),
                ('Gain_mean', 'g_mean'), ('Gain_std', 'g_std'),
                ('Rate_Mbps', 'total_rate'), ('Jain', 'jain'),
            ]:
                vals = [r[key] for r in results]
                s = sorted(vals); n = len(s)
                print(f"  {name:<18} {sum(s)/n:>8.1f} {s[n//2]:>8.1f} {s[max(0,n//20)]:>8.1f} {s[min(n-1,n*19//20)]:>8.1f}")

            # Per-flow rates
            if nf > 1 and results:
                for fi in range(nf):
                    frates = [r['flow_rates'][fi] for r in results if fi < len(r['flow_rates'])]
                    s = sorted(frates); n = len(s)
                    print(f"  Flow{fi}_Mbps        {sum(s)/n:>8.1f} {s[n//2]:>8.1f} {s[max(0,n//20)]:>8.1f} {s[min(n-1,n*19//20)]:>8.1f}")

            # Phase breakdown (last seed only)
            if results:
                r = results[-1]
                for label, v in sorted(r['phases'].items()):
                    print(f"  {label:<18} {v:>8.1f}")

    # Reference
    print(f"\n{'='*80}")
    print(f"Reference BDP queue (cwnd=1x): {T_PROP_US}us")
    print(f"Natural BDP queue (cwnd=2x): {T_PROP_US}us")
    print(f"P-controller: excess relative to T_prop, target=0, slope=1/4")
    print(f"Expected: queue ≈ T_prop, gain ≈ 1.0, rate ≈ BW / N_flows")

if __name__ == '__main__':
    N_FLOWS = 3  # default, overridden in loop
    run_all()
