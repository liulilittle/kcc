# KCC v2.0 FINAL: 1-1024 flows, full detail
# py -3 this.py
import random, math

BBR_UNIT = 256; PG_MIN = BBR_UNIT // 64; PG_MAX = 268
AI_N, AI_D = 1, 100
MD_N, MD_D = 1, 8; MD_COOLDOWN_MULT = 4; MD_CAP = 64; MD_CAP_COOLDOWN = 16
DRAIN_N, DRAIN_D = 92, 100; DRAIN_FLOOR_PCT = 0  # disabled — dynamic exit replaces it
FP_JUMP = 25; FP_DUR = 2; FP_COOLDOWN = 8; FP_TRIG = 4
P_INIT = 320; P_G = 125; P_D = 100; P_MAX = 512; P_MAX_R = 4
TGT_DIV = 128; DR_DIV = 32; DR_EXIT = 4
S_ST = 0; S_FP = 1; S_PL = 2; S_DR = 3
S_N = {0:'STEADY', 1:'FAST', 2:'PULSE', 3:'DRAIN'}

MSS = 1448; BW_M = 1260.0; BW_BPS = BW_M * 1e6; BD = BW_BPS / 8
T_PROP = 35000; BDP_B = BD * T_PROP * 1e-6; BDP_P = BDP_B / MSS

class Flow:
    def __init__(self, fid, base_rtt):
        self.fid = fid; self.brtt = base_rtt
        self.pg = BBR_UNIT; self.cwnd_g = BBR_UNIT; self.st = S_PL
        self.ez = 0; self.fpr = 0; self.fpc = 0; self.plr = 0
        self.bws = 0; self.mbw = 0.0; self.depg = 0; self.drok = 0
        self.mr = base_rtt

    def step(self, excess, bw_mbps):
        if bw_mbps > self.mbw: self.mbw = bw_mbps; self.bws = 0
        else: self.bws += 1
        if self.fpc > 0: self.fpc -= 1
        tp = max(1, self.mr); T = tp // TGT_DIV; D = tp // DR_DIV

        def cwnd_sq(p): return (p * p) // BBR_UNIT

        if self.st == S_ST:
            if excess <= T:
                self.ez += 1
            else:
                self.ez = 0
                md_den = MD_D * MD_COOLDOWN_MULT if self.fpc > 0 else MD_D
                md_cap = MD_CAP_COOLDOWN if self.fpc > 0 else MD_CAP
                red = (self.pg * excess * MD_N) // (max(1, tp) * md_den)
                red = min(red, md_cap)
                self.pg = max(self.pg - red, PG_MIN)
            if excess >= D and self.fpc == 0:
                self.st = S_DR; self.depg = 999999999; self.drok = 0
            elif self.ez >= FP_TRIG and self.fpc == 0 and self.pg < PG_MAX - 2:
                self.st = S_FP; self.fpr = 0

        elif self.st == S_FP:
            self.fpr += 1
            if excess > T or self.fpr >= FP_DUR:
                self.fpc = FP_COOLDOWN
                if excess > T:
                    self.st = S_ST; self.ez = 0
                    self.pg = max(PG_MIN, self.pg - 12)
                elif self.pg >= PG_MAX - 2:
                    self.st = S_ST
                else:
                    self.st = S_PL; self.plr = 0
            else:
                self.pg = min(self.pg + FP_JUMP, PG_MAX)

        elif self.st == S_PL:
            self.plr += 1
            cg = P_INIT
            for _ in range(self.plr - 1):
                cg = (cg * P_G) // P_D
            self.cwnd_g = min(cg, P_MAX)
            self.pg = min(self.cwnd_g, PG_MAX)
            if excess > T:
                self.st = S_ST; self.fpc = FP_COOLDOWN; self.ez = 0
                self.pg = max(PG_MIN, min(self.pg, PG_MAX))
            elif self.plr >= P_MAX_R or (self.bws >= 3 and self.plr >= 2):
                self.st = S_ST; self.fpc = FP_COOLDOWN; self.ez = 0
            elif excess >= D:
                self.st = S_DR; self.fpc = FP_COOLDOWN; self.depg = 999999999; self.drok = 0

        elif self.st == S_DR:
            self.pg = max(self.pg * DRAIN_N // DRAIN_D, PG_MIN)
            # Dynamic exit: decreasing excess → keep draining; plateau → stop
            if excess <= T:
                self.drok += 1
            elif excess < self.depg:
                self.drok += 1
            else:
                self.drok = 0
            self.depg = excess
            if self.drok >= DR_EXIT: self.st = S_ST; self.ez = 0

        if self.st != S_PL:
            self.cwnd_g = cwnd_sq(self.pg)

def simulate(nf, seed):
    rng = random.Random(seed)
    brtts = [max(3000, T_PROP + rng.randint(-1000, 1000)) for _ in range(nf)]
    flows = [Flow(i, brtts[i]) for i in range(nf)]
    stats = []
    for rd in range(600):
        total_pg = sum(f.pg for f in flows)
        avg_tp = sum(f.brtt for f in flows) // nf
        rtt_s = max(1e-9, avg_tp * 1e-6)
        for _ in range(8):
            total_rate = 0.0; total_inflight = 0.0
            for f in flows:
                pacing = BW_BPS * f.pg / BBR_UNIT
                cwnd_r = (BDP_P * f.cwnd_g / BBR_UNIT) * MSS * 8 / rtt_s
                rate = min(pacing, cwnd_r)
                total_rate += rate
                total_inflight += rate * rtt_s / 8 / MSS
            total_rate = min(total_rate, BW_BPS)
            queue_bytes = max(0.0, total_inflight * MSS - BDP_B)
            rtt_s = avg_tp * 1e-6 + queue_bytes / BD
        queue_us = queue_bytes / BD * 1e6
        excess = max(0.0, queue_us - avg_tp)
        for f in flows:
            f.step(excess, BW_BPS * f.pg / BBR_UNIT / 1e6)
        if rd >= 100:
            pgs = [f.pg / BBR_UNIT for f in flows]
            pm = sum(pgs) / nf
            ps = math.sqrt(sum((p - pm)**2 for p in pgs) / nf) if nf > 1 else 0
            sc = {}
            for f in flows: sc[f.st] = sc.get(f.st, 0) + 1
            stats.append({'rd': rd, 'q_us': queue_us, 'excess': excess,
                'pg_mean': pm, 'pg_std': ps, 'pg_min': min(pgs), 'pg_max': max(pgs),
                'states': dict(sc)})
    return stats

if __name__ == '__main__':
    print("=" * 100)
    print(f"KCC v2.0 FINAL — 1-1024 flows, PG_MIN={PG_MIN/BBR_UNIT:.4f}x, floor={DRAIN_FLOOR_PCT}%")
    print(f"  BW={BW_M}Mbps  T_prop={T_PROP/1000:.0f}ms  cwnd_gain=pg^2")
    print("=" * 100)

    for nf in [1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024]:
        stats = simulate(nf, 42)
        ss = stats[-200:]
        qs = sorted([s['q_us'] for s in ss]); nq = len(qs)
        q_avg = sum(qs) / nq; q50 = qs[nq // 2]
        q95 = qs[min(nq - 1, int(nq * 0.95))]
        q99 = qs[min(nq - 1, int(nq * 0.99))]
        q_max = qs[-1]
        pm_avg = sum(s['pg_mean'] for s in ss) / nq
        ps_avg = sum(s['pg_std'] for s in ss) / nq
        pmin_avg = min(s['pg_min'] for s in ss)
        pmax_avg = max(s['pg_max'] for s in ss)
        sc = {}
        for s in ss:
            for k, v in s['states'].items(): sc[k] = sc.get(k, 0) + v
        total_st = sum(sc.values())
        states = ' '.join(f'{S_N[k]}={v/total_st*100:.0f}%' for k, v in sorted(sc.items()))
        bbr_q = (2 * nf - 1) * T_PROP if nf > 1 else T_PROP
        improv = (1 - q50 / bbr_q) * 100 if bbr_q > 0 else 0

        print(f"\n{'='*100}")
        print(f"  N={nf:>4} | Q_avg={q_avg:>8.0f}us  Q_P50={q50:>8.0f}us  Q_P95={q95:>8.0f}us  Q_P99={q99:>8.0f}us  Q_max={q_max:>8.0f}us")
        print(f"         vsBBR={bbr_q/1000:.0f}ms -> {improv:.0f}% improvement")
        print(f"         PG_mean={pm_avg:.4f}  PG_std={ps_avg:.4f}  PG_range=[{pmin_avg:.4f}, {pmax_avg:.4f}]")
        print(f"         States: {states}")

        if nf <= 8:
            print(f"         Time trace (every 30 rounds, last 200):")
            print(f"         {'Rnd':>5} {'Q(us)':>9} {'PG':>7} {'std':>7}  States")
            for s in ss[::30]:
                st_s = ' '.join(f'{S_N[k]}={v}' for k, v in sorted(s['states'].items()))
                print(f"         {s['rd']:>5} {s['q_us']:>9.0f} {s['pg_mean']:>7.4f} {s['pg_std']:>7.4f}  {st_s}")

        if nf >= 64:
            print(f"         PG convergence (every 40 rounds):")
            pts = [(s['rd'], s['pg_mean'], s['pg_std']) for s in ss[::40]]
            print(f"         Rnd  PG_mean  std")
            for rd, pmv, psv in pts:
                print(f"         {rd:>4} {pmv:>8.4f} {psv:>6.4f}")

    # Summary
    print(f"\n{'='*100}")
    print("SUMMARY TABLE")
    print(f"{'='*100}")
    print(f"  {'N':>5} {'Q_P50':>8} {'Q_P95':>8} {'vsBBR':>6} {'PG':>6} {'std':>6} {'Spread':>7} {'States'}")
    print(f"  {'-'*5} {'-'*8} {'-'*8} {'-'*6} {'-'*6} {'-'*6} {'-'*7} {'-'*50}")
    for nf in [1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024]:
        stats = simulate(nf, 42)
        ss = stats[-200:]; nq = len(ss)
        qs = sorted([s['q_us'] for s in ss]); q50 = qs[nq // 2]; q95 = qs[min(nq - 1, int(nq * 0.95))]
        pm = sum(s['pg_mean'] for s in ss) / nq
        ps = sum(s['pg_std'] for s in ss) / nq
        spread = max(s['pg_max'] - s['pg_min'] for s in ss)
        sc = {}
        for s in ss:
            for k, v in s['states'].items(): sc[k] = sc.get(k, 0) + v
        total_st = sum(sc.values())
        states = ' '.join(f'{S_N[k]}={v/total_st*100:.0f}%' for k, v in sorted(sc.items()))
        bbr_q = (2 * nf - 1) * T_PROP if nf > 1 else T_PROP
        improv = (1 - q50 / bbr_q) * 100 if bbr_q > 0 else 0
        print(f"  {nf:>5} {q50:>8.0f} {q95:>8.0f} {improv:>5.0f}% {pm:>5.4f} {ps:>5.4f} {spread:>7.5f}  {states}")
