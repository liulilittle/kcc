# KCC v2.0 HONEST FULL REPORT — 1-1024 flows, per-flow noise, detailed data
# py -3 this.py
import random, math

BBR_UNIT = 256; PG_MIN = BBR_UNIT // 64; PG_MAX = 268
AI_N, AI_D = 1, 100
MD_N, MD_D = 1, 8; MD_CM = 4; MD_CAP = 64; MD_CAP_C = 16
DN, DD = 92, 100
FJ = 25; FD = 2; FC = 8; FT = 8  # was 4 — slower probe trigger
PI = 320; PG = 125; PD = 100; P_MAX = 512; P_MAX_R = 4
TGT = 128; DR_DIV = 32; DR_EXIT = 4
S_ST=0; S_FP=1; S_PL=2; S_DR=3
S_NAMES = {0:'STEADY',1:'FAST',2:'PULSE',3:'DRAIN'}

MSS = 1448; BW = 1260.0; BWbps = BW * 1e6; BD = BWbps / 8
TP = 35000; BDB = BD * TP * 1e-6; BDPp = BDB / MSS

IRQ_MAX = 50.0; TSO_PKTS = 45; RTT_JIT = 0.015; LOSS_RATE = 0.005

class Flow:
    def __init__(self, fid, brtt, rng):
        self.fid = fid; self.brtt = brtt; self.rng = rng
        self.pg = BBR_UNIT; self.cwnd_g = BBR_UNIT; self.st = S_PL
        self.ez = 0; self.fpr = 0; self.fpc = 0; self.plr = 0
        self.bws = 0; self.mbw = 0.0; self.depg = 999999999; self.drok = 0
        self.mr = brtt

    def noisy_rtt_min(self, queue_us):
        best = 1e9
        for _ in range(30):
            tso = (self.rng.randint(1, TSO_PKTS)) * MSS * 8 / (BW * 1e6) * 1e6
            irq = self.rng.uniform(0, IRQ_MAX)
            jit = self.rng.gauss(0, self.brtt * RTT_JIT)
            rtt = max(1, self.brtt + queue_us + tso + irq + jit)
            if rtt < best: best = rtt
        if self.rng.random() < LOSS_RATE:
            best = self.brtt + queue_us + self.rng.uniform(100, 500)
        return best

    def step(self, queue_us, bw):
        rtt_min = self.noisy_rtt_min(queue_us)
        if rtt_min < self.mr: self.mr = int(rtt_min)
        qdelay = max(0.0, rtt_min - self.brtt)
        excess = max(0, int(qdelay) - self.mr)
        tp = max(1, self.mr); T = tp // TGT; D = tp // DR_DIV
        sq = lambda p: (p * p) // BBR_UNIT

        if bw > self.mbw: self.mbw = bw; self.bws = 0
        else: self.bws += 1
        if self.fpc > 0: self.fpc -= 1

        if self.st == S_ST:
            if excess <= T: self.ez += 1
            else:
                self.ez = 0
                md_den = MD_D * MD_CM if self.fpc > 0 else MD_D
                md_cap = MD_CAP_C if self.fpc > 0 else MD_CAP
                red = (self.pg * excess * MD_N) // (max(1, tp) * md_den)
                red = min(red, md_cap); self.pg = max(self.pg - red, PG_MIN)
            if excess >= D and self.fpc == 0: self.st = S_DR; self.depg = 999999999; self.drok = 0
            elif self.ez >= FT and self.fpc == 0 and self.pg < PG_MAX - 2:
                self.st = S_FP; self.fpr = 0

        elif self.st == S_FP:
            self.fpr += 1
            if excess > T or self.fpr >= FD:
                self.fpc = FC
                if excess > T: self.st = S_ST; self.ez = 0; self.pg = max(PG_MIN, self.pg - 12)
                elif self.pg >= PG_MAX - 2: self.st = S_ST
                else: self.st = S_PL; self.plr = 0
            else: self.pg = min(self.pg + FJ, PG_MAX)

        elif self.st == S_PL:
            self.plr += 1
            cg = self.pg  # start from current pg, not fixed 1.25
            for _ in range(self.plr):
                cg = (cg * PG) // PD
            self.cwnd_g = min(cg, P_MAX); self.pg = min(self.cwnd_g, PG_MAX)
            if excess > T: self.st = S_ST; self.fpc = FC; self.ez = 0; self.pg = max(PG_MIN, min(self.pg, PG_MAX))
            elif self.plr >= P_MAX_R or (self.bws >= 3 and self.plr >= 2):
                self.st = S_ST; self.fpc = FC; self.ez = 0
            elif excess >= D: self.st = S_DR; self.fpc = FC; self.depg = 999999999; self.drok = 0

        elif self.st == S_DR:
            self.pg = max(self.pg * DN // DD, PG_MIN)
            if excess <= T: self.drok += 1
            elif excess < self.depg: self.drok += 1
            else: self.drok = 0
            self.depg = excess
            if self.drok >= DR_EXIT: self.st = S_ST; self.ez = 0; self.fpc = FC

        if self.st != S_PL: self.cwnd_g = sq(self.pg)

def simulate(nf, seed, n_rnds=800):
    base_rng = random.Random(seed)
    flows = [Flow(i, max(3000, TP + base_rng.randint(-1000, 1000)),
                   random.Random(base_rng.randint(0, 1<<30))) for i in range(nf)]
    stats = []
    for rd in range(n_rnds):
        nf2 = len(flows)
        total_pg = sum(f.pg for f in flows)
        avg_tp = sum(f.brtt for f in flows) // nf2
        rs = max(1e-9, avg_tp * 1e-6)
        for _ in range(8):
            tr = 0.0; ki = 0.0
            for f in flows:
                pacing = BWbps * f.pg / BBR_UNIT
                cwnd_r = (BDPp * f.cwnd_g / BBR_UNIT) * MSS * 8 / rs
                tr += min(pacing, cwnd_r)
                ki += min(pacing, cwnd_r) * rs / 8 / MSS
            tr = min(tr, BWbps)
            qb = max(0.0, ki * MSS - BDB)
            rs = avg_tp * 1e-6 + qb / BD
        q_us = qb / BD * 1e6
        for f in flows: f.step(q_us, BWbps * f.pg / BBR_UNIT / 1e6)
        if rd >= max(200, n_rnds // 4):
            pgs = [f.pg / BBR_UNIT for f in flows]
            pm = sum(pgs) / nf2
            ps = math.sqrt(sum((p - pm)**2 for p in pgs) / nf2) if nf2 > 1 else 0
            sc = {}
            for f in flows: sc[f.st] = sc.get(f.st, 0) + 1
            jain = 1.0 if nf2==1 else (sum(pgs)**2/(nf2*sum(p*p for p in pgs))) if sum(p*p for p in pgs)>0 else 0
            stats.append({'rd':rd,'q':q_us,'pm':pm,'ps':ps,'pmin':min(pgs),'pmax':max(pgs),
                          'sts':dict(sc),'jain':jain,'pgs':[round(p,4) for p in pgs]})
    return stats

if __name__ == '__main__':
    print("=" * 110)
    print(f"KCC v2.0 FULL REPORT — 1-1024 flows, per-flow noise, IRQ={IRQ_MAX}us, loss={LOSS_RATE*100:.1f}%")
    print(f"  BW={BW}Mbps  T_prop={TP/1000:.0f}ms  PG_MIN={PG_MIN/BBR_UNIT:.4f}  cwnd_gain=pg^2")
    print("=" * 110)

    for nf in [1,2,4,8,16,32,64,128,256,512,1024]:
        stats = simulate(nf, 42)
        ss = stats[-200:]
        qs = sorted([s['q'] for s in ss]); nq = len(qs)
        q50 = qs[nq//2]; q95 = qs[min(nq-1,int(nq*0.95))]; q99 = qs[min(nq-1,int(nq*0.99))]
        pm_avg = sum(s['pm'] for s in ss)/nq
        ps_avg = sum(s['ps'] for s in ss)/nq
        ps_max = max(s['ps'] for s in ss)
        spread_avg = sum(s['pmax']-s['pmin'] for s in ss)/nq
        j_avg = sum(s['jain'] for s in ss)/nq
        # State dist
        sc_all = {}
        for s in ss:
            for k,v in s['sts'].items(): sc_all[k]=sc_all.get(k,0)+v
        tot = sum(sc_all.values())
        st_str = ' '.join(f'{S_NAMES[k]}={v/tot*100:.0f}%' for k,v in sorted(sc_all.items()))
        bbr_q = (2*nf-1)*TP if nf>1 else TP
        imp = (1-q50/bbr_q)*100 if bbr_q else 0

        print(f"\n{'='*110}")
        print(f"  N={nf:>4}  Q_P50={q50:>8.0f}us  Q_P95={q95:>8.0f}us  Q_P99={q99:>8.0f}us")
        print(f"         PG_mean={pm_avg:.4f}  PG_std(avg)={ps_avg:.4f}  PG_std(max)={ps_max:.4f}  spread={spread_avg:.4f}")
        print(f"         Jain(avg)={j_avg:.4f}  vsBBR: {bbr_q/1000:.0f}ms -> {imp:.0f}% improvement")
        print(f"         States: {st_str}")

        # Per-flow pg distribution for small N
        if nf <= 16:
            print(f"         All pg values (last round): {' '.join(f'{p:.4f}' for p in ss[-1]['pgs'])}")
        elif nf <= 64:
            last_pgs = ss[-1]['pgs']
            last_pgs.sort()
            print(f"         PG distribution (last round): min={min(last_pgs):.4f} p25={last_pgs[nf//4]:.4f} "
                  f"p50={last_pgs[nf//2]:.4f} p75={last_pgs[3*nf//4]:.4f} max={max(last_pgs):.4f}")
        else:
            s_last = ss[-1]
            print(f"         PG range (last round): [{s_last['pmin']:.4f}, {s_last['pmax']:.4f}]  "
                  f"mean={s_last['pm']:.4f}  std={s_last['ps']:.4f}")

        # Time evolution snapshot
        print(f"         Time evolution (every 50 rounds, last 200):")
        for s in ss[::50]:
            st_s = ' '.join(f'{S_NAMES[k]}={v}' for k,v in sorted(s['sts'].items()))
            print(f"           rd={s['rd']:>4}  Q={s['q']:>8.0f}us  PG={s['pm']:.4f}  std={s['ps']:.4f}  J={s['jain']:.4f}  {st_s}")
