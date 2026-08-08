# KCC v2.0: FAST_PROBE + CWND_PULSE recovery speed test
# Cross-traffic modeled as persistent inflight (queue has memory)
# py -3 this.py
import random, math

BBR_UNIT = 256; PG_MIN = 32; PG_MAX = int(BBR_UNIT * 1.05)
AI_N, AI_D = 1, 100; MD_N, MD_D = 1, 8
DRAIN_N, DRAIN_D = 92, 100
TARGET_DIV = 128; DRAIN_DIV = 32; DRAIN_EXIT_RNDS = 4
FP_JUMP = BBR_UNIT // 10; FP_DURATION = 2; FP_COOLDOWN = 8; FP_TRIGGER = 4
PULSE_START = int(BBR_UNIT * 1.25); PULSE_G = 125; PULSE_D = 100; PULSE_MAX = BBR_UNIT * 2

S_ST = 0; S_FP = 1; S_PL = 2; S_DR = 3

def step_fast(pg, cwnd_g, excess, tprop, ez, fpr, fpc, plr, bws, mbw, bw, st):
    if bw > mbw: mbw = bw; bws = 0
    else: bws += 1
    if fpc > 0: fpc -= 1
    target = max(1, tprop) // TARGET_DIV; drain_trig = max(1, tprop) // DRAIN_DIV
    dok = 0

    if st == S_ST:
        if excess <= target: ez += 1; pg = min(pg + BBR_UNIT*AI_N//AI_D, PG_MAX)
        else: ez = 0; pg = max(pg - (pg*excess*MD_N)//(max(1,tprop)*MD_D), PG_MIN)
        if excess >= drain_trig: st = S_DR; dok = 0

    elif st == S_FP:
        fpr += 1
        if excess > target or fpr >= FP_DURATION:
            if excess > target: st = S_ST; fpc = FP_COOLDOWN; ez = 0; pg = max(PG_MIN, pg - BBR_UNIT//20)
            elif pg >= PG_MAX - BBR_UNIT//100: st = S_ST; fpc = FP_COOLDOWN
            else: st = S_PL; plr = 0
        else: pg = min(pg + FP_JUMP, PG_MAX)

    elif st == S_PL:
        plr += 1; cg = PULSE_START
        for _ in range(plr - 1): cg = (cg * PULSE_G) // PULSE_D
        cwnd_g = min(cg, PULSE_MAX); pg = min(cwnd_g, PG_MAX)
        if excess > target: st = S_ST; fpc = FP_COOLDOWN; ez = 0; pg = max(PG_MIN, min(pg, PG_MAX))
        elif plr >= 4 or (bws >= 3 and plr >= 2): st = S_ST; fpc = FP_COOLDOWN; ez = 0
        elif excess >= drain_trig: st = S_DR; dok = 0; fpc = FP_COOLDOWN

    elif st == S_DR:
        pg = max(pg * DRAIN_N // DRAIN_D, PG_MIN)
        if excess <= target: dok += 1
        else: dok = 0
        if dok >= DRAIN_EXIT_RNDS: st = S_ST; ez = 0

    ret = (pg, cwnd_g, ez, fpr, fpc, plr, bws, mbw, st)

def step_ai(pg, cwnd_g, excess, tprop, ez, bws, mbw, bw, st):
    if bw > mbw: mbw = bw; bws = 0
    else: bws += 1
    target = max(1, tprop) // TARGET_DIV; drain_trig = max(1, tprop) // DRAIN_DIV
    dok = 0
    if st == S_ST:
        if excess <= target: ez += 1; pg = min(pg + BBR_UNIT*AI_N//AI_D, PG_MAX)
        else: ez = 0; pg = max(pg - (pg*excess*MD_N)//(max(1,tprop)*MD_D), PG_MIN)
        if excess >= drain_trig: st = S_DR
    elif st == S_DR:
        pg = max(pg * DRAIN_N // DRAIN_D, PG_MIN)
        if excess <= target: dok += 1
        else: dok = 0
        if dok >= DRAIN_EXIT_RNDS: st = S_ST; ez = 0
    return (pg, cwnd_g, ez, bws, mbw, st)

MSS = 1448; BW_Mbps = 1260.0; BW_bps = BW_Mbps * 1e6; BD_BPS = BW_bps / 8
T_PROP_US = 35000; BDP_bytes = BD_BPS * T_PROP_US * 1e-6; BDP_pkts = BDP_bytes / MSS

def simulate_flow(nf, cross_inflight_fn, use_fast, n_rnds=600, seed=42):
    rng = random.Random(seed)
    base_rtts = [max(3000, T_PROP_US + rng.randint(-3000, 3000)) for _ in range(nf)]
    pg = [BBR_UNIT]*nf; cg = [BBR_UNIT]*nf; st = [S_ST]*nf
    ez = [0]*nf; fpr = [0]*nf; fpc = [0]*nf; plr = [0]*nf; dok = [0]*nf
    bws = [0]*nf; mbw = [0.0]*nf
    cross_bytes = 0.0; stats = []

    for rd in range(n_rnds):
        target_cross = cross_inflight_fn(rd) * BDP_bytes
        cross_bytes += (target_cross - cross_bytes) * 0.3

        total_wt = sum(pg[i] for i in range(nf))
        avg_tprop = sum(base_rtts[i] for i in range(nf)) // nf
        rtt_s = avg_tprop * 1e-6
        for _ in range(8):
            total_rate = 0.0; kcc_inflight = 0.0
            for i in range(nf):
                pacing = BW_bps * pg[i] / BBR_UNIT
                cwnd_rate = (BDP_pkts * cg[i] / BBR_UNIT) * MSS * 8 / rtt_s
                total_rate += min(pacing, cwnd_rate)
                kcc_inflight += min(pacing, cwnd_rate) * rtt_s / 8 / MSS
            total_rate = min(total_rate, BW_bps)
            inflight_bytes = kcc_inflight * MSS + cross_bytes
            queue_bytes = max(0.0, inflight_bytes - BDP_bytes)
            rtt_s = avg_tprop * 1e-6 + queue_bytes / BD_BPS

        queue_us = queue_bytes / BD_BPS * 1e6
        excess = max(0.0, queue_us - avg_tprop)
        frates = [BW_bps * pg[i] / total_wt / 1e6 if total_wt > 0 else 0.0 for i in range(nf)]

        for i in range(nf):
            if use_fast:
                pg[i], cg[i], ez[i], fpr[i], fpc[i], plr[i], bws[i], mbw[i], st[i] = \
                    step_fast(pg[i], cg[i], excess, avg_tprop, ez[i], fpr[i], fpc[i],
                              plr[i], bws[i], mbw[i], frates[i], st[i])
            else:
                pg[i], cg[i], ez[i], bws[i], mbw[i], st[i] = \
                    step_ai(pg[i], cg[i], excess, avg_tprop, ez[i], bws[i], mbw[i], frates[i], st[i])

        if rd >= 20:
            stats.append({'rd': rd, 'queue_us': queue_us,
                'pg': [p/BBR_UNIT for p in pg], 'rate': sum(frates), 'st': list(st)})
    return stats

# ============================================================
# Tests
# ============================================================
CROSS_LEAVE_RND = 200
SIM_RND = 400

print("=" * 85)
print("FAST_PROBE RECOVERY SPEED — Queue-Memory Model")
print(f"  T_prop=35ms  cross leaves at round {CROSS_LEAVE_RND}")
print(f"  AI=1%/RTT  FAST_PROBE=+10%/RTT×{FP_DURATION} + CWND_PULSE×1.25")
print("=" * 85)

# Test 1: 1 flow, 1.5 BDP cross inflight leaves
print(f"\n  {'Scenario':<35} {'Variant':<15} {'Recovery':>8} {'FinalPG':>8}")
print(f"  {'-'*35} {'-'*15} {'-'*8} {'-'*8}")

for label, nf, cr_factor in [
    ('1 flow + 1×BDP cross leaves', 1, 1.0),
    ('1 flow + 2×BDP cross leaves', 1, 2.0),
    ('1 flow + 4×BDP cross leaves', 1, 4.0),
    ('3 flows + 1×BDP cross leaves', 3, 1.0),
    ('3 flows + 3×BDP cross leaves', 3, 3.0),
]:
    for use_fp in [False, True]:
        var = "FAST_PROBE" if use_fp else "AI only"
        cross_fn = lambda rd, f=cr_factor: f if rd < CROSS_LEAVE_RND else 0.0
        stats = simulate_flow(nf, cross_fn, use_fp, n_rnds=SIM_RND, seed=42)

        # Recovery time
        pg_vals = [(s['rd'], s['pg'][0]) for s in stats if s['rd'] >= CROSS_LEAVE_RND]
        recovery = 'N/A'
        for rd, pg in pg_vals:
            if pg > 0.95:
                recovery = f"{rd - CROSS_LEAVE_RND:>4} RTT"; break
        if recovery == 'N/A': recovery = '>160'

        final_pg = stats[-1]['pg'][0] if stats else 0
        print(f"  {label:<35} {var:<15} {recovery:>8} {final_pg:>8.3f}")

    print()

# Test 2: Show pg trajectory for AI vs FAST_PROBE
print("=" * 85)
print("PG TRAJECTORY: 1 flow + 4×BDP cross leaves at round 200")
print("=" * 85)

cross_fn = lambda rd: 4.0 if rd < 200 else 0.0

for use_fp in [False, True]:
    var = "FAST_PROBE" if use_fp else "AI only"
    print(f"\n  {var}:")
    print(f"  {'RTT':>5} {'pg':>8} {'rate':>8} {'queue':>8}")
    print(f"  {'-'*5} {'-'*8} {'-'*8} {'-'*8}")

    stats = simulate_flow(1, cross_fn, use_fp, n_rnds=SIM_RND, seed=42)
    for s in stats:
        rd = s['rd']
        if rd in [198, 199, 200, 201, 202, 203, 204, 205, 206, 208, 210, 215, 220, 230, 250, 300, 350]:
            pg = s['pg'][0]
            rate = s['rate']
            q = s['queue_us']
            bar = '#' * int(pg * 40)
            print(f"  {rd:>5} {pg:>8.3f} {rate:>8.0f} {q:>8.0f}  {bar}")

# Test 3: Speedup factor
print("\n" + "=" * 85)
print("SPEEDUP SUMMARY")
print("=" * 85)
print(f"  {'Scenario':<35} {'AI(recover)':>12} {'FP(recover)':>12} {'Speedup':>8}")
print(f"  {'-'*35} {'-'*12} {'-'*12} {'-'*8}")

for label, nf, cr_factor in [
    ('1f + 1×BDP cross', 1, 1.0),
    ('1f + 2×BDP cross', 1, 2.0),
    ('1f + 4×BDP cross', 1, 4.0),
    ('3f + 1×BDP cross', 3, 1.0),
    ('3f + 3×BDP cross', 3, 3.0),
]:
    cross_fn = lambda rd, f=cr_factor: f if rd < CROSS_LEAVE_RND else 0.0
    ai_stats = simulate_flow(nf, cross_fn, False, n_rnds=SIM_RND, seed=42)
    fp_stats = simulate_flow(nf, cross_fn, True, n_rnds=SIM_RND, seed=42)

    ai_rec = 999; fp_rec = 999
    for s in ai_stats:
        if s['rd'] >= CROSS_LEAVE_RND and s['pg'][0] > 0.95:
            ai_rec = s['rd'] - CROSS_LEAVE_RND; break
    for s in fp_stats:
        if s['rd'] >= CROSS_LEAVE_RND and s['pg'][0] > 0.95:
            fp_rec = s['rd'] - CROSS_LEAVE_RND; break

    speedup = f"{ai_rec/fp_rec:.1f}×" if ai_rec < 999 and fp_rec < 999 else 'N/A'
    print(f"  {label:<35} {ai_rec:>9} RTT {fp_rec:>9} RTT {speedup:>8}")
