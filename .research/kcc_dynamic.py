# KCC v2.0 ULTIMATE VERIFICATION — Dynamic flow count, raw per-round trace, no aggregation
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

MSS=1448; BW=1260.0; BWbps=BW*1e6; BD=BWbps/8; TP=35000; BDB=BD*TP*1e-6; BDPp=BDB/MSS

class Flow:
    def __init__(self, fid, brtt, start_round):
        self.fid = fid; self.brtt = brtt; self.start_round = start_round
        self.pg = BBR_UNIT; self.cwnd_g = BBR_UNIT; self.st = S_PL
        self.ez = 0; self.fpr = 0; self.fpc = 0; self.plr = 0
        self.bws = 0; self.mbw = 0.0; self.depg = 999999999; self.drok = 0; self.mr = brtt

    def step(self, excess, bw):
        if bw > self.mbw: self.mbw = bw; self.bws = 0
        else: self.bws += 1
        if self.fpc > 0: self.fpc -= 1
        tp = max(1, self.mr); T = tp // TGT; D = tp // DR_DIV
        sq = lambda p: (p * p) // BBR_UNIT

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

# ============================================================
# DYNAMIC SIMULATION — flows arrive and depart over time
# ============================================================
rng = random.Random(42)
flows = []  # (flow, departure_round)
next_fid = 0

# Schedule: flows arrive at specific rounds to create clear transitions
# Round 0: 1 flow
# Round 100: +1 flow (total 2)
# Round 200: +2 flows (total 4)
# Round 300: +4 flows (total 8)
# Round 400: +8 flows (total 16)
# Round 500: +16 flows (total 32)
# Round 600: 16 oldest flows leave (back to 16)
# Round 700: 8 leave (back to 8)
# Round 800: 4 leave (back to 4)

add_events = [
    (0, 1), (100, 1), (200, 2), (300, 4), (400, 8), (500, 16),
]
remove_events = [
    (600, 16), (700, 8), (800, 4),
]

SIM_RNDS = 1000

stats = []
for rd in range(SIM_RNDS):
    # Add flows at scheduled rounds
    for (ar, count) in add_events:
        if rd == ar:
            for _ in range(count):
                brtt = max(3000, TP + rng.randint(-1000, 1000))
                flows.append(Flow(next_fid, brtt, rd))
                next_fid += 1

    # Remove oldest flows at scheduled rounds
    for (rr, count) in remove_events:
        if rd == rr:
            # Remove oldest 'count' flows
            flows.sort(key=lambda f: f.start_round)
            flows = flows[count:]

    nf = len(flows)
    if nf == 0:
        stats.append({'rd': rd, 'nf': 0, 'q_us': 0, 'pg_mean': 0, 'pg_std': 0,
                      'pg_min': 0, 'pg_max': 0, 'states': '--', 'pgs': []})
        continue

    # Fluid equilibrium
    total_pg = sum(f.pg for f in flows)
    avg_tp = sum(f.brtt for f in flows) // nf
    rtt_s = max(1e-9, avg_tp * 1e-6)
    for _ in range(8):
        tr = 0.0; ki = 0.0
        for f in flows:
            pacing = BWbps * f.pg / BBR_UNIT
            cwnd_r = (BDPp * f.cwnd_g / BBR_UNIT) * MSS * 8 / rtt_s
            rate = min(pacing, cwnd_r)
            tr += rate; ki += rate * rtt_s / 8 / MSS
        tr = min(tr, BWbps)
        qb = max(0.0, ki * MSS - BDB)
        rtt_s = avg_tp * 1e-6 + qb / BD

    q_us = qb / BD * 1e6
    excess = max(0.0, q_us - avg_tp)

    for f in flows:
        f.step(excess, BWbps * f.pg / BBR_UNIT / 1e6)

    pgs = [f.pg / BBR_UNIT for f in flows]
    pm = sum(pgs) / nf if nf > 0 else 0
    ps = math.sqrt(sum((p - pm)**2 for p in pgs) / nf) if nf > 1 else 0
    st_dist = {}
    for f in flows: st_dist[SN[f.st]] = st_dist.get(SN[f.st], 0) + 1
    st_str = ','.join(f'{k}{v}' for k, v in sorted(st_dist.items()))

    # Jain fairness from pg values (not from rate formula)
    jain = 1.0 if nf == 1 else (sum(pgs)**2 / (nf * sum(p*p for p in pgs))) if sum(p*p for p in pgs) > 0 else 0

    stats.append({'rd': rd, 'nf': nf, 'q_us': q_us, 'excess': excess,
                  'pg_mean': pm, 'pg_std': ps, 'pg_min': min(pgs) if pgs else 0,
                  'pg_max': max(pgs) if pgs else 0, 'states': st_str,
                  'pgs': [round(p, 4) for p in pgs], 'jain': jain})

# ============================================================
# PRINT RAW TRACE — every round for key transitions, every 10 otherwise
# ============================================================
print("=" * 110)
print("KCC v2.0 DYNAMIC VERIFICATION — Flow count changes, raw per-round trace")
print(f"  BW={BW}Mbps  T_prop={TP/1000:.0f}ms  PG_MIN={PG_MIN/BBR_UNIT:.4f}  dynamic DRAINING exit")
print("  Events: 1->2@100  2->4@200  4->8@300  8->16@400  16->32@500  -16@600  -8@700  -4@800")
print("=" * 110)

last_printed_nf = -1
for s in stats:
    rd = s['rd']; nf = s['nf']; q = s['q_us']
    pm = s['pg_mean']; ps = s['pg_std']; ja = s['jain']
    st = s['states']
    pgs = s['pgs']

    # Print every round during transitions (5 rounds before/after flow changes), else every 50
    near_transition = any(abs(rd - ev[0]) <= 8 for ev in add_events + remove_events)
    should_print = (near_transition or
                    rd % 100 == 0 or
                    (rd <= 20) or
                    nf != last_printed_nf)

    if should_print:
        # Show per-flow pg for up to 8 flows, else just stats
        if nf <= 8:
            pg_str = '  pg:' + ' '.join(f'{p:.4f}' for p in pgs)
        elif nf <= 32:
            pg_str = f'  pg(range):[{s["pg_min"]:.4f}-{s["pg_max"]:.4f}]'
        else:
            pg_str = f'  pg(mean/min/max):{pm:.4f}/{s["pg_min"]:.4f}/{s["pg_max"]:.4f}'

        print(f"  rd={rd:>4} nf={nf:>4} Q={q:>8.0f}us  PG={pm:.4f} std={ps:.4f}  J={ja:.4f}  st={st}{pg_str}")

    last_printed_nf = nf

# ============================================================
# PHASE SUMMARY
# ============================================================
print(f"\n{'='*110}")
print("PHASE SUMMARY (steady-state for each flow count period)")
print(f"{'='*110}")
print(f"  {'Phase':<20} {'Rounds':>10} {'N':>4} {'Q_P50':>8} {'Q_P95':>8} {'PG':>6} {'PG_std':>6} {'Jain':>6} {'States':>30}")
print(f"  {'-'*20} {'-'*10} {'-'*4} {'-'*8} {'-'*8} {'-'*6} {'-'*6} {'-'*6} {'-'*30}")

phases = [
    ("1 flow (start)", 50, 99),
    ("2 flows", 120, 190),
    ("4 flows", 240, 290),
    ("8 flows", 340, 390),
    ("16 flows", 440, 490),
    ("32 flows", 540, 590),
    ("16 flows (after -16)", 620, 690),
    ("8 flows (after -8)", 720, 790),
    ("4 flows (after -4)", 820, 900),
    ("end steady", 920, 990),
]

for label, start, end in phases:
    ss = [s for s in stats if start <= s['rd'] <= end]
    if not ss: continue
    nf_set = set(s['nf'] for s in ss)
    nf_str = ','.join(str(n) for n in sorted(nf_set))
    qs = sorted([s['q_us'] for s in ss]); nq = len(qs)
    q50 = qs[nq // 2] if nq else 0
    q95 = qs[min(nq - 1, int(nq * 0.95))] if nq else 0
    pm_avg = sum(s['pg_mean'] for s in ss) / len(ss)
    ps_avg = sum(s['pg_std'] for s in ss) / len(ss)
    j_avg = sum(s['jain'] for s in ss) / len(ss)
    # Aggregate states
    st_agg = {}
    for s in ss:
        for part in s['states'].split(','):
            if part:
                k = part[0]; v = int(part[1:])
                st_agg[k] = st_agg.get(k, 0) + v
    total_st = sum(st_agg.values())
    st_str = ' '.join(f'{k}={v/total_st*100:.0f}%' for k, v in sorted(st_agg.items()))
    print(f"  {label:<20} {start}-{end:<5} {nf_str:>4} {q50:>8.0f} {q95:>8.0f} {pm_avg:>5.3f} {ps_avg:>5.4f} {j_avg:>5.4f}  {st_str}")
