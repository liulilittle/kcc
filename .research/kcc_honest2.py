# KCC v2.0 HONEST SIM v3 — Fluid equilibrium queue + per-flow noisy RTT samples
# Queue = correct fluid model. Per-flow decisions = independent noisy RTT min.
# py -3 this.py
import random, math

BBR_UNIT = 256; PG_MIN = BBR_UNIT // 64; PG_MAX = 268
AI_N, AI_D = 1, 100
MD_N, MD_D = 1, 8; MD_CM = 4; MD_CAP = 64; MD_CAP_C = 16
DN, DD = 92, 100
FJ = 25; FD = 2; FC = 8; FT = 4
PI = 320; PG = 125; PD = 100; P_MAX = 512; P_MAX_R = 4
TGT = 128; DR_DIV = 32; DR_EXIT = 4
S_ST=0; S_FP=1; S_PL=2; S_DR=3
SN={0:'S',1:'F',2:'P',3:'D'}

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
        # Per-flow noisy RTT min — realistic noise model
        best = 1e9
        n_samp = 30
        for _ in range(n_samp):
            tso = (self.rng.randint(1, TSO_PKTS)) * MSS * 8 / (BW * 1e6) * 1e6
            irq = self.rng.uniform(0, IRQ_MAX)
            jit = self.rng.gauss(0, self.brtt * RTT_JIT)
            rtt = max(1, self.brtt + queue_us + tso + irq + jit)
            if rtt < best: best = rtt
        if self.rng.random() < LOSS_RATE:
            best = self.brtt + queue_us + self.rng.uniform(100, 500)
        return best

    def step(self, queue_us, bw):
        # Noisy per-flow RTT min — THE key realism addition
        rtt_min = self.noisy_rtt_min(queue_us)
        if rtt_min < self.mr:
            self.mr = int(rtt_min)
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
            self.plr += 1; cg = PI
            for _ in range(self.plr - 1): cg = (cg * PG) // PD
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
            if self.drok >= DR_EXIT: self.st = S_ST; self.ez = 0

        if self.st != S_PL: self.cwnd_g = sq(self.pg)

rng = random.Random(42)
base_rng = random.Random(12345)
flows = []
stats = []

for rd in range(1000):
    if rd == 0:
        for _ in range(1):
            flows.append(Flow(len(flows), max(3000, TP + base_rng.randint(-1000, 1000)),
                             random.Random(base_rng.randint(0, 1<<30))))
    if rd == 250:
        for _ in range(3):
            flows.append(Flow(len(flows), max(3000, TP + base_rng.randint(-1000, 1000)),
                             random.Random(base_rng.randint(0, 1<<30))))
    if rd == 500:
        for _ in range(4):
            flows.append(Flow(len(flows), max(3000, TP + base_rng.randint(-1000, 1000)),
                             random.Random(base_rng.randint(0, 1<<30))))
    if rd == 750:
        flows.sort(key=lambda f: f.fid)
        flows = flows[4:]

    nf = len(flows)
    if nf == 0: continue

    # FLUID EQUILIBRIUM — correct queue computation
    total_pg = sum(f.pg for f in flows)
    avg_tp = sum(f.brtt for f in flows) // nf
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

    # PER-FLOW NOISY DECISIONS
    for f in flows:
        f.step(q_us, BWbps * f.pg / BBR_UNIT / 1e6)

    if rd >= 200:
        pgs = [f.pg / BBR_UNIT for f in flows]
        pm = sum(pgs) / nf
        ps = math.sqrt(sum((p - pm)**2 for p in pgs) / nf) if nf > 1 else 0
        stc = {}
        for f in flows: stc[SN[f.st]] = stc.get(SN[f.st], 0) + 1
        sts = ','.join(f'{k}{v}' for k, v in sorted(stc.items()))
        jain = 1.0 if nf == 1 else (sum(pgs)**2 / (nf * sum(p*p for p in pgs))) if sum(p*p for p in pgs) > 0 else 0
        stats.append({'rd': rd, 'nf': nf, 'q': q_us, 'pm': pm, 'ps': ps,
                       'pmin': min(pgs), 'pmax': max(pgs), 'sts': sts, 'jain': jain,
                       'pgs': [round(p, 4) for p in pgs]})

print("=" * 110)
print(f"KCC v2.0 HONEST v3 — Fluid queue + per-flow noisy decisions, IRQ={IRQ_MAX}us, loss={LOSS_RATE*100:.1f}%")
print("=" * 110)

last_nf = -1
for s in stats:
    rd = s['rd']; nf = s['nf']
    near = rd in [250,251,252,253, 500,501,502,503, 750,751,752,753]
    if near or (nf != last_nf) or rd % 200 == 0:
        pgs = s['pgs']
        if nf <= 8: pg_str = 'pg:' + ' '.join(f'{p:.4f}' for p in pgs)
        else: pg_str = f'pg:[{s["pmin"]:.4f}-{s["pmax"]:.4f}]'
        print(f"  rd={rd:>4} nf={nf:>2} Q={s['q']:>8.0f}us  PG={s['pm']:.4f} std={s['ps']:.4f}  J={s['jain']:.4f}  {s['sts']}  {pg_str}")
    last_nf = nf

print(f"\n{'='*110}")
print("PHASE SUMMARY")
print(f"{'='*110}")
print(f"  {'Phase':<15} {'N':>3} {'Q_P50':>8} {'Q_P95':>8} {'PG':>6} {'std':>6} {'Jain':>6}")
for label, s0, s1 in [("1 flow", 300, 240), ("4 flows", 400, 490), ("8 flows", 600, 740), ("4 flows end", 800, 990)]:
    ss = [s for s in stats if s0 <= s['rd'] <= s1]
    if not ss: continue
    qs = sorted([s['q'] for s in ss]); nq = len(qs)
    q50 = qs[nq//2] if nq else 0; q95 = qs[min(nq-1, int(nq*0.95))] if nq else 0
    pm = sum(s['pm'] for s in ss)/len(ss)
    ps = sum(s['ps'] for s in ss)/len(ss)
    ja = sum(s['jain'] for s in ss)/len(ss)
    print(f"  {label:<15} {ss[0]['nf']:>3} {q50:>8.0f} {q95:>8.0f} {pm:>5.3f} {ps:>5.4f} {ja:>5.4f}")
