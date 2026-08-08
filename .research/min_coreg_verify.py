# KCC 2.0: per-round min filter signal extraction test
# Question: given known T_prop, can round_rtt_min accurately recover true queue?
# Independent of geodesic estimation quality — isolates the min filter.
import random, sys, math
from collections import deque

MSS = 1448
BW_Mbps = 1260.0
BW_bps = BW_Mbps * 1e6
T_PROP_s = 0.035        # 35ms propagation delay (known truth)
RTT_s = T_PROP_s
BDP_bytes = BW_bps * T_PROP_s / 8
BDP_pkts = BDP_bytes / MSS

CWND_GAIN = 2.0         # creates queue ≈ BDP (35ms)

# TSO per-packet position-dependent delay
TSO_BURST_BYTES = 65536
TSO_PKTS = max(1, int(TSO_BURST_BYTES / MSS))  # ~45
# Per-packet TSO serialization delay (k within burst, 1-indexed)
TSO_SERIAL_DELAY_us = MSS * 8 / BW_Mbps         # ~9.2us per packet

# Noise: interrupt coalescing delay (uniform per ACK)
IRQ_MAX_US = 50.0

# Simulation: discrete RTT rounds
N_ROUNDS = 2000

def run_single(test_name, queue_series_s, options):
    """Run one test with given ground-truth queue series."""
    rng = random.Random(options.get('seed', 0))
    tso_on = options.get('tso', True)
    noise_on = options.get('noise', True)
    samples_per_round = options.get('samples_per_round', 100)
    aux_noise_sigma_us = options.get('aux_noise', 0.0)
    tso_burst_pkts = options.get('tso_burst_pkts', TSO_PKTS)

    measured_q_us = []
    var_proxy_us_list = []
    true_q_us = []

    # Start queue tracking for EWMA baseline (previous approach)
    qdelay_avg = 0.0

    for rd, true_q_s in enumerate(queue_series_s):
        tq_us = true_q_s * 1e6
        true_q_us.append(tq_us)

        # Generate RTT samples within this round
        round_samples = []
        tso_idx = 0  # packet index within TSO burst

        for i in range(samples_per_round):
            # TSO position within burst
            k = (tso_idx % tso_burst_pkts) + 1  # 1-indexed position
            tso_delay_s = (k * TSO_SERIAL_DELAY_us * 1e-6) if tso_on else 0.0
            tso_idx += 1

            # Random noise
            noise_s = rng.uniform(0, IRQ_MAX_US * 1e-6) if noise_on else 0.0

            # Aux Gaussian noise
            aux_s = max(0.0, rng.gauss(0, aux_noise_sigma_us * 1e-6)) if aux_noise_sigma_us > 0 else 0.0

            rtt_s = T_PROP_s + true_q_s + tso_delay_s + noise_s + aux_s
            round_samples.append(rtt_s)

        # Per-round min filter
        round_min = min(round_samples)
        round_max = max(round_samples)
        qdelay_measured_s = max(0.0, round_min - T_PROP_s)
        var_proxy_s = round_max - round_min

        measured_q_us.append(qdelay_measured_s * 1e6)
        var_proxy_us_list.append(var_proxy_s * 1e6)

        # EWMA baseline for comparison
        qi = max(0.0, round_min - T_PROP_s)
        qdelay_avg = qdelay_avg * 0.875 + qi * 0.125

    # Compute error metrics
    errors = [abs(m - t) for m, t in zip(measured_q_us, true_q_us)]
    avg_err = sum(errors) / len(errors)
    max_err = max(errors)
    rmse = math.sqrt(sum(e*e for e in errors) / len(errors))

    # Variance stats
    avg_var = sum(var_proxy_us_list) / len(var_proxy_us_list)
    max_var = max(var_proxy_us_list)

    return {
        'name': test_name,
        'avg_error_us': avg_err,
        'max_error_us': max_err,
        'rmse_us': rmse,
        'avg_var_us': avg_var,
        'max_var_us': max_var,
        'true_q': true_q_us,
        'measured_q': measured_q_us,
        'var_history': var_proxy_us_list,
    }

def make_queue_pattern(pattern, n_rounds=N_ROUNDS):
    """Generate synthetic ground-truth queue delay (seconds)."""
    if pattern == 'steady_bdp':
        # Persistent queue = BDP (cwnd_gain=2.0 creates this)
        return [T_PROP_s] * n_rounds

    elif pattern == 'square_wave':
        # Alternating: queue present / queue absent
        result = []
        for i in range(n_rounds):
            phase = (i % 40) // 20
            result.append(T_PROP_s if phase == 0 else 0.0)
        return result

    elif pattern == 'sine_wave':
        result = []
        for i in range(n_rounds):
            phase = 2 * math.pi * i / 80.0
            q = T_PROP_s * (0.5 + 0.5 * math.sin(phase))
            result.append(max(0.0, q))
        return result

    elif pattern == 'drain_cycle':
        # BBR-style: 6 rounds cruise + 1 round drain (0.75x gain)
        result = []
        for i in range(n_rounds):
            phase = (i % 8)
            if phase == 1:   # drain phase: queue near 0
                result.append(T_PROP_s * 0.02)
            elif phase == 0: # probe up: queue larger
                result.append(T_PROP_s * 1.4)
            else:            # cruise: BDP queue
                result.append(T_PROP_s)
        return result

    elif pattern == 'slow_ramp':
        # Gradual queue buildup (cross-traffic entering)
        result = []
        for i in range(n_rounds):
            if i < 100:
                result.append(0.0)
            elif i < 800:
                frac = (i - 100) / 700.0
                result.append(T_PROP_s * frac)
            elif i < 1500:
                result.append(T_PROP_s)
            else:
                frac = 1.0 - (i - 1500) / 500.0
                result.append(T_PROP_s * max(0.0, frac))
        return result

    elif pattern == 'random_walk':
        # Random walk with reflecting boundaries
        rng = random.Random(42)
        result = []
        q = 0.0
        for i in range(n_rounds):
            step = rng.gauss(0, T_PROP_s * 0.05)
            q = max(0.0, min(T_PROP_s * 2.0, q + step))
            result.append(q)
        return result

    elif pattern == 'three_phase':
        # Startup (variance high) -> steady (variance low) -> drain (variance spike)
        result = []
        rng = random.Random(42)
        for i in range(n_rounds):
            if i < 100:        # startup: volatile
                q = T_PROP_s * abs(rng.gauss(1.0, 0.5))
            elif i < 1500:     # steady: stable
                q = T_PROP_s * (1.0 + rng.gauss(0, 0.02))
            else:              # drain: drop
                frac = max(0.0, 1.0 - (i - 1500) / 100.0)
                q = T_PROP_s * frac * abs(rng.gauss(1.0, 0.3))
            result.append(max(0.0, q))
        return result

    return []

def print_table(results):
    print(f"{'Test':<28} {'AvgErr':>8} {'MaxErr':>8} {'RMSE':>8} {'AvgVar':>8} {'MaxVar':>8}")
    print("-" * 70)
    for r in results:
        print(f"{r['name']:<28} {r['avg_error_us']:>7.1f} {r['max_error_us']:>7.1f} "
              f"{r['rmse_us']:>7.1f} {r['avg_var_us']:>7.1f} {r['max_var_us']:>7.1f}")

def run_all():
    patterns = [
        'steady_bdp',
        'square_wave',
        'sine_wave',
        'drain_cycle',
        'slow_ramp',
        'random_walk',
        'three_phase',
    ]
    options_list = [
        ('clean',         dict(tso=False, noise=False, aux_noise=0)),
        ('tso_only',      dict(tso=True,  noise=False, aux_noise=0)),
        ('noise_25us',    dict(tso=False, noise=True,  aux_noise=0)),
        ('tso+noise',     dict(tso=True,  noise=True,  aux_noise=0)),
        ('severe_noise',  dict(tso=True,  noise=True,  aux_noise=100)),
        ('few_samples',   dict(tso=True,  noise=True,  aux_noise=0, samples_per_round=10)),
    ]

    print("KCC Per-Round Min Filter: Signal Extraction Accuracy")
    print("=" * 70)
    print(f"T_prop = {T_PROP_s*1000:.0f}ms, BW = {BW_Mbps}Mbps, BDP = {BDP_pkts:.0f} pkts")
    print(f"TSO burst = {TSO_PKTS} pkts, IRQ jitter = {IRQ_MAX_US:.0f}us")
    print()

    for pattern in patterns:
        q_series = make_queue_pattern(pattern)
        print(f"\n--- Queue Pattern: {pattern} ---")
        print(f"    Q range: [{min(q_series)*1e6:.0f}, {max(q_series)*1e6:.0f}] us")

        results = []
        for oname, opts in options_list:
            opts['seed'] = hash(pattern + oname) & 0xFFFFFFFF
            r = run_single(f"{pattern}/{oname}", q_series, opts)
            results.append(r)

        print_table(results)

        # Best case analysis
        best = min(results, key=lambda x: x['avg_error_us'])
        print(f"  Best: {best['name']} -> avg_err={best['avg_error_us']:.1f}us, rmse={best['rmse_us']:.1f}us")

    # Final: verify model_rtt (min of x_est and min_rtt) from geodesic
    print("\n" + "=" * 70)
    print("Geodesic Estimator Accuracy (with known T_prop)")
    print("=" * 70)
    from common import GeodesicEstimator
    for pattern_name in ['steady_bdp', 'drain_cycle', 'random_walk']:
        q_series = make_queue_pattern(pattern_name)
        geo = GeodesicEstimator(T_PROP_s * 1e6)  # initial RTT in us
        model_rtt_errors = []
        t_prop_errors = []

        for rd, true_q_s in enumerate(q_series):
            # Simulated RTT measurement (with queue but NO noise/TSO)
            rtt_us = (T_PROP_s + true_q_s) * 1e6
            geo.update(rtt_us)

            # model_rtt = min(x_est, min_rtt_us)
            model_rtt_us = min(geo.x_est / geo.SCALE, geo.min_rtt_us)
            t_prop_est_us = geo.min_rtt_us
            t_prop_true_us = T_PROP_s * 1e6

            model_rtt_errors.append(abs(model_rtt_us - t_prop_true_us))
            t_prop_errors.append(abs(t_prop_est_us - t_prop_true_us))

        avg_model_err = sum(model_rtt_errors) / len(model_rtt_errors)
        avg_tprop_err = sum(t_prop_errors) / len(t_prop_errors)

        # After convergence (last 500 rounds)
        conv_model = sum(model_rtt_errors[-500:]) / 500
        conv_tprop = sum(t_prop_errors[-500:]) / 500

        print(f"  {pattern_name:<20} model_rtt: avg={avg_model_err:.0f}us conv={conv_model:.0f}us  "
              f"min_rtt: avg={avg_tprop_err:.0f}us conv={conv_tprop:.0f}us")

if __name__ == '__main__':
    run_all()
