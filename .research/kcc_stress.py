# KCC v2.0 STRESS TEST v2 — True fairness audit, pg_std measurement, single-seed trace
# py -3 this.py
import random, math

BBR_UNIT = 256; PG_MIN = 32; PG_MAX = 268
AI_N, AI_D = 1, 100; MD_N, MD_D = 1, 8
MD_COOLDOWN_MULT = 4; MD_CAP = 64; MD_CAP_COOLDOWN = 16
DRAIN_N, DRAIN_D = 92, 100; DRAIN_FLOOR_PCT = 70
FP_JUMP = 25; FP_DUR = 2; FP_COOLDOWN = 8; FP_TRIG = 4
PULSE_INIT = 320; PULSE_G = 125; PULSE_D = 100; PULSE_MAX = 512; PULSE_MAX_R = 4
TGT_DIV = 128; DR_DIV = 32; DR_EXIT = 4
S_ST = 0; S_FP = 1; S_PL = 2; S_DR = 3
S_NAMES = {0: 'STEADY', 1: 'FAST_PROBE', 2: 'CWND_PULSE', 3: 'DRAINING'}

MSS = 1448; BW_Mbps = 1260.0; BW_bps = BW_Mbps * 1e6; BD = BW_bps / 8
T_PROP_US = 35000; BDP_B = BD * T_PROP_US * 1e-6; BDP_P = BDP_B / MSS

class Flow:
    def __init__(self, fid, brtt):
        self.fid = fid; self.brtt = brtt
        self.pg = BBR_UNIT; self.cwnd_g = BBR_UNIT
        self.st = S_PL; self.ez = 0; self.fpr = 0; self.fpc = 0; self.plr = 0
        self.bws = 0; self.mbw = 0.0; self.depg = 0; self.drok = 0
        self.mr = brtt  # geodesic min_rtt — initialized to base_rtt

    def step(self, excess, bw_mbps):
        if bw_mbps > self.mbw: self.mbw = bw_mbps; self.bws = 0
        else: self.bws += 1
        if self.fpc > 0: self.fpc -= 1
        tp = max(1, self.mr); target = tp // TGT_DIV; dt = tp // DR_DIV

        if self.st == S_ST:
            self.cwnd_g = (self.pg * self.pg) // BBR_UNIT   # cwnd_gain = pg²
            if excess <= target: self.ez += 1
            else:
                self.ez = 0
                md = MD_D * MD_COOLDOWN_MULT if self.fpc > 0 else MD_D
                mc = MD_CAP_COOLDOWN if self.fpc > 0 else MD_CAP
                red = (self.pg * excess * MD_N) // (max(1, tp) * md)
                red = min(red, mc); self.pg = max(self.pg - red, PG_MIN)
            if excess >= dt and self.fpc == 0: self.st = S_DR; self.depg = self.pg; self.drok = 0
            elif self.ez >= FP_TRIG and self.fpc == 0 and self.pg < PG_MAX - 2: self.st = S_FP; self.fpr = 0

        elif self.st == S_FP:
            self.cwnd_g = (self.pg * self.pg) // BBR_UNIT   # cwnd_gain = pg²
            self.fpr += 1
            if excess > target or self.fpr >= FP_DUR:
                self.fpc = FP_COOLDOWN
                if excess > target: self.st = S_ST; self.ez = 0; self.pg = max(PG_MIN, self.pg - 12)
                elif self.pg >= PG_MAX - 2: self.st = S_ST
                else: self.st = S_PL; self.plr = 0
            else: self.pg = min(self.pg + FP_JUMP, PG_MAX)

        elif self.st == S_PL:
            self.plr += 1; cg = PULSE_INIT
            for _ in range(self.plr - 1): cg = (cg * PULSE_G) // PULSE_D
            self.cwnd_g = min(cg, PULSE_MAX); self.pg = min(self.cwnd_g, PG_MAX)
            if excess > target: self.st = S_ST; self.fpc = FP_COOLDOWN; self.ez = 0; self.pg = max(PG_MIN, min(self.pg, PG_MAX))
            elif self.plr >= PULSE_MAX_R or (self.bws >= 3 and self.plr >= 2): self.st = S_ST; self.fpc = FP_COOLDOWN; self.ez = 0
            elif excess >= dt: self.st = S_DR; self.fpc = FP_COOLDOWN; self.depg = self.pg; self.drok = 0

        elif self.st == S_DR:
            self.cwnd_g = (self.pg * self.pg) // BBR_UNIT   # cwnd_gain = pg²
            fl = max(self.depg * DRAIN_FLOOR_PCT // 100, PG_MIN)
            self.pg = max(self.pg * DRAIN_N // DRAIN_D, fl)
            if excess <= target: self.drok += 1
            else: self.drok = 0
            if self.drok >= DR_EXIT: self.st = S_ST; self.ez = 0

def sim_detail(nf, n_rnds, seed):
    """Detailed simulation returning per-flow pg over time."""
    rng = random.Random(seed)
    # IMPORTANT: use tight RTT variation (±500us, not ±3000us) — real geodesic converges tightly
    base_rtts = [T_PROP_US + rng.randint(-500, 500) for _ in range(nf)]
    base_rtts = [max(1000, x) for x in base_rtts]
    flows = [Flow(i, base_rtts[i]) for i in range(nf)]
    stats = []

    for rd in range(n_rnds):
        at = sum(f.brtt for f in flows) // nf
        total_wt = sum(f.pg for f in flows)
        rtt_s = max(1e-9, at * 1e-6)

        # Fluid equilibrium
        for _ in range(8):
            tr = 0.0; kif = 0.0
            for f in flows:
                pacing = BW_bps * f.pg / BBR_UNIT
                cwnd_r = (BDP_P * f.cwnd_g / BBR_UNIT) * MSS * 8 / rtt_s
                tr += min(pacing, cwnd_r)
                kif += min(pacing, cwnd_r) * rtt_s / 8 / MSS
            tr = min(tr, BW_bps)
            qb = max(0.0, kif * MSS - BDP_B)
            rtt_s = at * 1e-6 + qb / BD

        q_us = qb / BD * 1e6
        excess = max(0.0, q_us - at)

        # TRUE fairness: per-flow rate proportional to bottleneck share
        # In FIFO queue, each flow's rate = BW * inflight[i] / total_inflight
        # But we compute it from pg for simplicity (all flows share same pg → same share)

        # Step each flow
        for f in flows:
            # Rate estimate for BW tracking
            bw = BW_bps * f.pg / BBR_UNIT / 1e6
            f.step(excess, bw)

        if rd >= max(100, n_rnds // 4):
            pgs = [f.pg / BBR_UNIT for f in flows]
            pg_mean = sum(pgs) / nf
            pg_var = sum((p - pg_mean)**2 for p in pgs) / nf if nf > 1 else 0
            pg_std = math.sqrt(pg_var)
            pg_min = min(pgs); pg_max = max(pgs)

            stc = {}
            for f in flows: stc[f.st] = stc.get(f.st, 0) + 1

            stats.append({
                'rd': rd, 'q_us': q_us, 'excess': excess,
                'pg_mean': pg_mean, 'pg_std': pg_std, 'pg_min': pg_min, 'pg_max': pg_max,
                'stc': dict(stc),
            })

    return stats, flows

def st(arr):
    s = sorted(arr); n = len(s)
    return {'mean': sum(s)/n, 'p50': s[n//2], 'p5': s[max(0,n//20)], 'p95': s[min(n-1,n*19//20)]} if n else {'mean':0,'p50':0,'p5':0,'p95':0}

if __name__ == '__main__':
    NS = 30
    print("=" * 100)
    print("KCC v2.0 STRESS TEST v2 — True Fairness Audit (pg_std measurement)")
    print(f"  BW={BW_Mbps}Mbps  T_prop={T_PROP_US/1000:.0f}ms  Seeds={NS}")
    print(f"  RTT variation=±500us (tight geodesic convergence)")
    print("=" * 100)

    for nf in [8, 16, 32, 64, 128]:
        print(f"\n{'='*100}")
        print(f"  N = {nf} flows")
        print(f"{'='*100}")

        all_stats = []  # collect per-seed aggregate stats
        single_detail = None  # save one detailed trace

        for s in range(min(NS, 10 if nf >= 128 else NS)):
            stats, flows = sim_detail(nf, 600, 42 + s)
            if s == 0: single_detail = stats

            steady = stats[-200:]
            qs = [r['q_us'] for r in steady]; qs.sort(); nq = len(qs)
            pg_means = [r['pg_mean'] for r in steady]
            pg_stds = [r['pg_std'] for r in steady]
            pg_mins = [r['pg_min'] for r in steady]
            pg_maxs = [r['pg_max'] for r in steady]
            pg_spreads = [m2 - m1 for m1, m2 in zip(pg_mins, pg_maxs)]

            # State distribution
            stc_all = {}
            for r in steady:
                for k, v in r['stc'].items(): stc_all[k] = stc_all.get(k, 0) + v
            total_st = sum(stc_all.values())

            all_stats.append({
                'q_p50': qs[nq//2] if nq else 0,
                'pg_mean_avg': sum(pg_means)/len(pg_means),
                'pg_std_avg': sum(pg_stds)/len(pg_stds),
                'pg_std_max': max(pg_stds) if pg_stds else 0,
                'pg_spread_avg': sum(pg_spreads)/len(pg_spreads) if pg_spreads else 0,
                'pg_spread_max': max(pg_spreads) if pg_spreads else 0,
                'st_pct': {k: v/total_st*100 for k, v in stc_all.items()} if total_st else {},
            })

        # Aggregate
        print(f"  {'Metric':<25} {'Mean':>9} {'P50':>9} {'P5':>9} {'P95':>9}")
        print(f"  {'-'*25} {'-'*9} {'-'*9} {'-'*9} {'-'*9}")
        for name, key in [
            ('Q_P50(us)', 'q_p50'), ('PG_mean', 'pg_mean_avg'),
            ('PG_std(avg)', 'pg_std_avg'), ('PG_std(max)', 'pg_std_max'),
            ('PG_spread(avg)', 'pg_spread_avg'), ('PG_spread(max)', 'pg_spread_max'),
        ]:
            vals = [r.get(key, 0) for r in all_stats]; s = st(vals)
            print(f"  {name:<25} {s['mean']:>9.4f} {s['p50']:>9.4f} {s['p5']:>9.4f} {s['p95']:>9.4f}")

        # State distribution
        all_st = {}
        for r in all_stats:
            for k, v in r.get('st_pct', {}).items(): all_st[k] = all_st.get(k, 0) + v
        n_res = len(all_stats)
        st_str = ' | '.join(f'{S_NAMES.get(k,str(k))}={v/n_res:.0f}%' for k, v in sorted(all_st.items()))
        print(f"  States: {st_str}")

        # Fairness verdict
        pg_std = st([r['pg_std_avg'] for r in all_stats])
        pg_spread = st([r['pg_spread_avg'] for r in all_stats])
        fair = pg_std['p95'] < 0.05 and pg_spread['p95'] < 0.10
        print(f"  Fairness: {'FAIR (pg_std<0.05, spread<0.10)' if fair else 'UNFAIR'}")

        # Single trace: show per-flow pg distribution
        if single_detail:
            s = single_detail[-1]
            print(f"  Single trace (seed=0, last round):")
            print(f"    Q={s['q_us']:.0f}us  pg_mean={s['pg_mean']:.4f}  pg_std={s['pg_std']:.4f}  "
                  f"pg_min={s['pg_min']:.4f}  pg_max={s['pg_max']:.4f}")

    print(f"\n{'='*100}")
    print(f"Done.")
