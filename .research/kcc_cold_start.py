# KCC v2.0 Cold-Start Measurement — CWND_PULSE initial mode
# py -3 this.py
import random, math, json

BBR_UNIT = 256; PG_MIN = 32; PG_MAX = int(BBR_UNIT * 1.05)
AI_N, AI_D = 1, 100; MD_N, MD_D = 1, 8
DRAIN_N, DRAIN_D = 92, 100
FP_JUMP = BBR_UNIT // 10; FP_DURATION = 2; FP_COOLDOWN = 8; FP_TRIGGER = 4
PULSE_INIT = int(BBR_UNIT * 1.25); PULSE_G = 125; PULSE_D = 100
PULSE_MAX = BBR_UNIT * 2; PULSE_MAX_RNDS = 4
TARGET_DIV = 128; DRAIN_DIV = 32; DRAIN_EXIT_RNDS = 4

S_ST=0; S_FP=1; S_PL=2; S_DR=3
S_NAMES = {0:'STEADY', 1:'FAST_PROBE', 2:'CWND_PULSE', 3:'DRAINING'}

def step(pg, cwnd_g, excess, tprop, ez, fpr, fpc, plr, bws, mbw, bw, st):
    if bw > mbw: mbw = bw; bws = 0
    else: bws += 1
    if fpc > 0: fpc -= 1
    target = max(1, tprop)//TARGET_DIV; drain_trig = max(1, tprop)//DRAIN_DIV
    dok = 0

    if st == S_ST:
        cwnd_g = pg
        if excess <= target: ez += 1; pg = min(pg + BBR_UNIT*AI_N//AI_D, PG_MAX)
        else: ez = 0; pg = max(pg - (pg*excess*MD_N)//(max(1,tprop)*MD_D), PG_MIN)
        if excess >= drain_trig and fpc == 0: st = S_DR; dok = 0
        elif ez >= FP_TRIGGER and fpc == 0 and pg < PG_MAX - BBR_UNIT//100: st = S_FP; fpr = 0

    elif st == S_FP:
        fpr += 1
        if excess > target or fpr >= FP_DURATION:
            fpc = FP_COOLDOWN
            if excess > target: st = S_ST; ez = 0; pg = max(PG_MIN, pg - BBR_UNIT//20)
            elif pg >= PG_MAX - BBR_UNIT//100: st = S_ST
            else: st = S_PL; plr = 0
        else: pg = min(pg + FP_JUMP, PG_MAX)

    elif st == S_PL:
        plr += 1; cg = PULSE_INIT
        for _ in range(plr - 1): cg = (cg * PULSE_G)//PULSE_D
        cwnd_g = min(cg, PULSE_MAX); pg = min(cwnd_g, PG_MAX)
        if excess > target: st = S_ST; fpc = FP_COOLDOWN; ez = 0; pg = max(PG_MIN, min(pg, PG_MAX))
        elif plr >= PULSE_MAX_RNDS or (bws >= 3 and plr >= 2): st = S_ST; fpc = FP_COOLDOWN; ez = 0
        elif excess >= drain_trig: st = S_DR; dok = 0; fpc = FP_COOLDOWN

    elif st == S_DR:
        pg = max(pg * DRAIN_N//DRAIN_D, PG_MIN); cwnd_g = pg
        if excess <= target: dok += 1
        else: dok = 0
        if dok >= DRAIN_EXIT_RNDS: st = S_ST; ez = 0

    return (pg, cwnd_g, ez, fpr, fpc, plr, bws, mbw, st)

MSS = 1448; BW_Mbps = 1260.0; BW_bps = BW_Mbps * 1e6; BD_BPS = BW_bps / 8
T_PROP_US = 35000; BDP_bytes = BD_BPS * T_PROP_US * 1e-6; BDP_pkts = BDP_bytes / MSS

def simulate_flow(nf, cross_fn, n_rnds=200, seed=42):
    """Start in CWND_PULSE (mode=2)."""
    rng = random.Random(seed)
    base_rtts = [max(3000, T_PROP_US + rng.randint(-3000, 3000)) for _ in range(nf)]

    # COLD START: mode=2 (CWND_PULSE), probe_round=0
    pg = [BBR_UNIT] * nf; cwnd_g = [BBR_UNIT] * nf
    st = [S_PL] * nf  # <— CWND_PULSE from start
    ez = [0]*nf; fpr = [0]*nf; fpc = [0]*nf; plr = [0]*nf
    bws = [0]*nf; mbw = [0.0]*nf

    cross_bytes = 0.0; stats = []

    for rd in range(n_rnds):
        target_cross = cross_fn(rd) * BDP_bytes
        cross_bytes += (target_cross - cross_bytes) * 0.3

        total_wt = sum(pg[i] for i in range(nf))
        avg_tprop = sum(base_rtts[i] for i in range(nf)) // nf
        rtt_s = avg_tprop * 1e-6
        for _ in range(8):
            tr = 0.0; kif = 0.0
            for i in range(nf):
                pacing = BW_bps * pg[i] / BBR_UNIT
                cwnd_r = (BDP_pkts * cwnd_g[i] / BBR_UNIT) * MSS * 8 / rtt_s
                r = min(pacing, cwnd_r); tr += r; kif += r * rtt_s / 8 / MSS
            tr = min(tr, BW_bps)
            qb = max(0.0, kif * MSS + cross_bytes - BDP_bytes)
            rtt_s = avg_tprop * 1e-6 + qb / BD_BPS

        queue_us = qb / BD_BPS * 1e6
        excess = max(0.0, queue_us - avg_tprop)
        frates = [BW_bps * pg[i] / total_wt / 1e6 if total_wt>0 else 0.0 for i in range(nf)]

        for i in range(nf):
            pg[i], cwnd_g[i], ez[i], fpr[i], fpc[i], plr[i], bws[i], mbw[i], st[i] = \
                step(pg[i], cwnd_g[i], excess, avg_tprop, ez[i], fpr[i], fpc[i],
                     plr[i], bws[i], mbw[i], frates[i], st[i])

        stats.append({'rd': rd, 'queue_us': queue_us, 'excess_us': excess,
            'pg': [p/BBR_UNIT for p in pg], 'cg': [c/BBR_UNIT for c in cwnd_g],
            'rate': sum(frates), 'st': list(st), 'st_name': [S_NAMES[s] for s in st]})

    return stats

# ============================================================
# Test: 1 flow cold start, 3 flow cold start
# ============================================================
print("=" * 85)
print("KCC v2.0 COLD-START — CWND_PULSE Initial Mode")
print(f"  T_prop={T_PROP_US}us={T_PROP_US/1000:.0f}ms  BW={BW_Mbps}Mbps  BDP={BDP_pkts:.0f}pkts")
print("=" * 85)

for nf, label in [(1, 'Single flow'), (3, '3 flows')]:
    print(f"\n{'='*85}")
    print(f"  {label} cold start")
    print(f"{'='*85}")
    print(f"  {'RTT':>4} {'CWND':>7} {'PG':>7} {'State':>12} {'Inflight(BDP)':>13} {'Q(us)':>8} {'Rate(Mbps)':>10}")
    print(f"  {'-'*4} {'-'*7} {'-'*7} {'-'*12} {'-'*13} {'-'*8} {'-'*10}")

    stats = simulate_flow(nf, lambda rd: 0.0, n_rnds=100, seed=42)

    for s in stats:
        rd = s['rd']
        if rd < 30:
            cg = s['cg'][0]
            pg = s['pg'][0]
            stn = s['st_name'][0]
            # Inflight in BDP units: cwnd_gain * pacing_gain (for single flow)
            inflight_bdp = cg * pg * nf
            q = s['queue_us']
            rate = s['rate']
            print(f"  {rd:>4} {cg:>7.3f} {pg:>7.3f} {stn:>12} {inflight_bdp:>13.2f} {q:>8.0f} {rate:>10.0f}")

    # Summary
    print(f"\n  Final steady state (rounds 80-100):")
    last = stats[-20:]
    avg_q = sum(s['queue_us'] for s in last) / len(last)
    avg_r = sum(s['rate'] for s in last) / len(last)
    avg_pg = sum(s['pg'][0] for s in last) / len(last)
    states = {}
    for s in last:
        for sn in s['st_name']: states[sn] = states.get(sn, 0) + 1
    total = sum(states.values())
    st_str = ' '.join(f'{k}={v/total*100:.0f}%' for k, v in states.items())
    print(f"    Queue={avg_q:.0f}us  Rate={avg_r:.0f}Mbps  PG={avg_pg:.3f}  States: {st_str}")

# ============================================================
# Multi-seed statistics
# ============================================================
print(f"\n{'='*85}")
print("MULTI-SEED STATISTICS (30 seeds)")
print(f"{'='*85}")

for nf in [1, 3, 5, 8]:
    results = []
    for seed in range(30):
        stats = simulate_flow(nf, lambda rd: 0.0, n_rnds=120, seed=seed + 42)

        # Pipe-full detection: rounds until rate > 90% of BW or pg > 0.9
        pipe_full_rnd = None
        for s in stats:
            pg0 = s['pg'][0]
            rate = s['rate']
            if rate > BW_Mbps * 0.9 or pg0 > 0.9:
                pipe_full_rnd = s['rd']; break

        # Convergence: rounds until state count stable (no mode changes for 20 rounds)
        conv_rnd = None
        last_st = None; stable_cnt = 0
        for s in stats:
            current_st = s['st'][0]
            if current_st == last_st:
                stable_cnt += 1
                if stable_cnt >= 8 and conv_rnd is None:
                    conv_rnd = s['rd']
            else:
                stable_cnt = 0
            last_st = current_st

        # Peak queue during startup
        peak_q = max(s['queue_us'] for s in stats[:40])

        # Final steady state
        last = stats[-20:]
        final_q = sum(s['queue_us'] for s in last) / len(last)
        final_rate = sum(s['rate'] for s in last) / len(last)
        final_pg = sum(s['pg'][0] for s in last) / len(last)

        results.append({
            'seed': seed, 'nf': nf,
            'pipe_full_rnd': pipe_full_rnd or 999,
            'conv_rnd': conv_rnd or 999,
            'peak_q': peak_q,
            'final_q': final_q,
            'final_rate': final_rate,
            'final_pg': final_pg,
        })

    def st(arr):
        s = sorted(arr); n = len(s)
        return f"{sum(s)/n:>7.1f} {s[n//2]:>7.1f} {s[max(0,n//20)]:>7.1f} {s[min(n-1,n*19//20)]:>7.1f}" if n else 'N/A'

    pfr = [r['pipe_full_rnd'] for r in results]
    cr = [r['conv_rnd'] for r in results]
    pq = [r['peak_q'] for r in results]
    fq = [r['final_q'] for r in results]
    fr = [r['final_rate'] for r in results]
    fpg = [r['final_pg'] for r in results]

    print(f"\n  N={nf}:")
    print(f"    {'Metric':<22} {'Mean':>8} {'P50':>8} {'P5':>8} {'P95':>8}")
    print(f"    {'-'*22} {'-'*8} {'-'*8} {'-'*8} {'-'*8}")
    for name, vals in [
        ('Pipe_full(RTTs)', pfr), ('Converged(RTTs)', cr),
        ('Peak_Q(us)', pq), ('Final_Q(us)', fq),
        ('Final_Rate(Mbps)', fr), ('Final_PG', fpg),
    ]:
        print(f"    {name:<22} {st(vals)}")

print()
