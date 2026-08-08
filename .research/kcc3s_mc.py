# KCC v2.0 Final: pg-only AIMD, wide floor, fluid bottleneck model
# pg ∈ [0.125, 1.05], cwnd_gain = pg. Convergence: pg → 1/N for N flows.
# py -3 this.py
import random, math, json

MSS = 1448; BW_Mbps = 1260.0; BW_bps = BW_Mbps * 1e6; BD_BPS = BW_bps / 8
T_PROP_US = 35000; T_PROP_S = T_PROP_US * 1e-6
BDP_bytes = BD_BPS * T_PROP_S; BDP_pkts = BDP_bytes / MSS
BBR_UNIT = 256

TARGET_US = T_PROP_US // 128   # 273us
DRAIN_TRIG = T_PROP_US // 32   # 1093us
PG_MIN = BBR_UNIT // 8         # 0.125 — allows up to 8 flows at zero queue
PG_MAX = int(BBR_UNIT * 1.05)

AI_N, AI_D = 1, 100            # +1%/RTT AI step
MD_N, MD_D = 1, 8              # pg -= pg * excess/(T_prop * 8)
DRAIN_DEC_N, DRAIN_DEC_D = 92, 100  # *0.92 per drain round

PROBE_CG = int(BBR_UNIT * 1.25); PROBE_G = 125; PROBE_D = 100

def sim_one(seed, n_flows, scenario):
    rng = random.Random(seed)
    pg = [BBR_UNIT] * n_flows
    cwnd_g = [BBR_UNIT] * n_flows
    excess_zero = [0] * n_flows
    probe_r = [0] * n_flows; probe_cd = [0] * n_flows
    drain_ok = [0] * n_flows
    bw_stable = [0] * n_flows; max_bw = [0.0] * n_flows
    state = [0] * n_flows  # 0=STEADY, 1=PROBING, 2=DRAINING

    NR = 800; stats = []

    for rd in range(NR):
        cross_frac = {'alone': 0.0, 'step': 0.5 if 200<=rd<600 else 0.0}.get(scenario, 0.0)
        avail_bw = BW_bps * (1.0 - cross_frac)

        # Compute equilibrium queue
        # Each flow: send_rate = min(pacing_rate, cwnd/RTT)
        # pacing_rate_i = BW_bps * pg[i] / BBR_UNIT
        # cwnd_i = BDP_pkts * cwnd_g[i] / BBR_UNIT
        total_send = 0.0; total_inflight = 0.0
        rtt_s = T_PROP_S  # start with base RTT
        for it in range(5):  # iterate to find equilibrium
            total_send = 0.0; total_inflight = 0.0
            for i in range(n_flows):
                pacing = BW_bps * pg[i] / BBR_UNIT
                cwnd_bps = (BDP_pkts * cwnd_g[i] / BBR_UNIT) * MSS * 8 / rtt_s
                rate = min(pacing, cwnd_bps)
                total_send += rate
                total_inflight += rate * rtt_s / 8 / MSS
            total_inflight_bytes = total_inflight * MSS
            queue_bytes = max(0.0, total_inflight_bytes - BDP_bytes)
            rtt_s = T_PROP_S + queue_bytes / BD_BPS

        queue_us = queue_bytes / BD_BPS * 1e6
        excess = max(0.0, queue_us - T_PROP_US)
        total_rate_mbps = min(total_send, avail_bw) / 1e6

        # Per-flow rates (proportional to pg — fairness mechanism)
        total_pg = sum(pg[i] for i in range(n_flows))
        flow_rates = [avail_bw * pg[i]/total_pg/1e6 if total_pg>0 else 0.0 for i in range(n_flows)]

        # Step each flow
        for i in range(n_flows):
            tprop = T_PROP_US  # use true tprop (geodesic converged)

            # BW tracking
            bw_now = flow_rates[i]
            if bw_now > max_bw[i]: max_bw[i] = bw_now; bw_stable[i] = 0
            else: bw_stable[i] += 1
            if probe_cd[i] > 0: probe_cd[i] -= 1

            if state[i] == 0:  # STEADY
                cwnd_g[i] = pg[i]  # cwnd = pacing
                if excess <= TARGET_US:
                    excess_zero[i] += 1
                    pg[i] = min(int(pg[i] + BBR_UNIT*AI_N//AI_D), PG_MAX)
                else:
                    excess_zero[i] = 0
                    # Multiplicative decrease proportional to excess
                    ratio = (int(excess) * BBR_UNIT * MD_N) // (tprop * MD_D)
                    reduction = min(ratio, BBR_UNIT // 4)
                    pg[i] = max(PG_MIN, int(pg[i] * (BBR_UNIT - int(reduction)) // BBR_UNIT))

                if excess >= DRAIN_TRIG:
                    state[i] = 2; drain_ok[i] = 0
                elif (pg[i] >= PG_MAX - BBR_UNIT//100 and excess_zero[i] >= 8
                      and probe_cd[i] == 0 and bw_stable[i] >= 12):
                    state[i] = 1; probe_r[i] = 0

            elif state[i] == 1:  # PROBING
                probe_r[i] += 1
                if excess > TARGET_US or probe_r[i] >= 6 or bw_stable[i] >= 4:
                    state[i] = 0; probe_cd[i] = 8
                    cg = PROBE_CG
                    for _ in range(probe_r[i] - 1): cg = (cg * PROBE_G)//PROBE_D
                    pg[i] = max(PG_MIN, min(cg, PG_MAX))
                    cwnd_g[i] = pg[i]
                    excess_zero[i] = 0
                else:
                    cg = PROBE_CG
                    for _ in range(probe_r[i] - 1): cg = (cg * PROBE_G)//PROBE_D
                    cwnd_g[i] = min(cg, int(BBR_UNIT * 2.0))
                    pg[i] = min(cwnd_g[i], PG_MAX)

            elif state[i] == 2:  # DRAINING
                pg[i] = max(PG_MIN, int(pg[i] * DRAIN_DEC_N // DRAIN_DEC_D))
                cwnd_g[i] = pg[i]
                if excess <= TARGET_US: drain_ok[i] += 1
                else: drain_ok[i] = 0
                if drain_ok[i] >= 4: state[i] = 0; excess_zero[i] = 0

        if rd >= 100:
            stats.append({'rd':rd,'queue_us':queue_us,'cross':cross_frac,
                'pgains':[p/BBR_UNIT for p in pg],'rates':flow_rates,
                'total_rate':total_rate_mbps,'states':list(state)})

    qs = [s['queue_us'] for s in stats]; qs.sort()
    nq = len(qs); avg_q = sum(qs)/nq if nq else 0
    p50 = qs[nq//2]; p95 = qs[min(nq-1,int(nq*0.95))]
    frates = [sum([s['rates'][fi] for s in stats[-200:] if fi<len(s['rates'])])/max(1,len([s for s in stats[-200:] if fi<len(s['rates'])])) for fi in range(n_flows)]
    total = sum(frates)
    jain = 1.0 if n_flows==1 else (total**2/(n_flows*sum(r*r for r in frates)) if sum(r*r for r in frates)>0 else 1.0)
    pg_avg = sum([sum(s['pgains'])/len(s['pgains']) for s in stats])/len(stats) if stats else 1.0

    # Count states
    st_cnt = {0:0,1:0,2:0}
    for s in stats:
        for stv in s['states']: st_cnt[stv] = st_cnt.get(stv,0)+1
    total_st = sum(st_cnt.values()); st_pct = {k:v/total_st*100 for k,v in st_cnt.items()} if total_st else {}

    return {'seed':seed,'sc':scenario,'nf':n_flows,'q_avg':avg_q,'q_p50':p50,'q_p95':p95,
        'rate':total,'jain':jain,'pg':pg_avg,'st_pct':st_pct,'frates':frates}

def st(arr):
    s=sorted(arr); n=len(s)
    return {'mean':sum(s)/n,'p50':s[n//2],'p5':s[max(0,n//20)],'p95':s[min(n-1,n*19//20)]} if n else {'mean':0,'p50':0,'p5':0,'p95':0}

if __name__=='__main__':
    NS=30
    print("="*85)
    print("KCC v2.0: pg-only AIMD, cwnd_gain=pg, pg∈[0.125,1.05]")
    print(f"  T_prop={T_PROP_US}us  BW={BW_Mbps}Mbps  AI={AI_N/AI_D*100:.1f}%  MD=*{MD_N}/{MD_D}")
    print("="*85)

    all_res=[]
    for nf in [1,3,5,8,12]:
        for sc in ['alone']:
            res=[sim_one(42+s,nf,sc) for s in range(NS)]
            all_res.extend(res)
            print(f"\n  {nf} flow(s) alone:")
            print(f"  {'Metric':<22} {'Mean':>8} {'P50':>8} {'P5':>8} {'P95':>8}")
            for name,key in [('Queue(us)','q_avg'),('Q_P50(us)','q_p50'),('Q_P95(us)','q_p95'),
                             ('Rate(Mbps)','rate'),('Jain','jain'),('PGain','pg')]:
                vals=[r[key] for r in res]; s=st(vals)
                print(f"  {name:<22} {s['mean']:>8.1f} {s['p50']:>8.1f} {s['p5']:>8.1f} {s['p95']:>8.1f}")
            for sn,sl in [(0,'STEADY'),(1,'PROBING'),(2,'DRAINING')]:
                pcts=[r['st_pct'].get(sn,0) for r in res]; s=st(pcts)
                print(f"  State_{sl:<16} {s['mean']:>8.1f}% {s['p50']:>8.1f}% {s['p5']:>8.1f}% {s['p95']:>8.1f}%")
            if nf>1:
                for fi in range(nf):
                    fr=[]; [fr.append(r['frates'][fi]) for r in res if fi<len(r.get('frates',[]))]
                    if fr: s=st(fr); print(f"  Flow{fi}_Mbps{' '*14} {s['mean']:>8.1f} {s['p50']:>8.1f} {s['p5']:>8.1f} {s['p95']:>8.1f}")

    print(f"\n{'='*85}\nSUMMARY")
    print(f"  {'N':>3} {'Q(us)':>8} {'Rate':>8} {'Jain':>7} {'PG':>7} {'Eff%':>7} {'vsBBR':>8}")
    bbr_q = {1:35000,3:175000,5:315000,8:525000,12:805000}
    for nf in [1,3,5,8,12]:
        ss=[r for r in all_res if r['nf']==nf]
        if not ss: continue
        q=st([r['q_p50'] for r in ss]); tr=st([r['rate'] for r in ss])
        ja=st([r['jain'] for r in ss]); pg=st([r['pg'] for r in ss])
        bq = bbr_q.get(nf,0)
        imp = f"{(1-q['p50']/bq)*100:.0f}%" if bq else "-"
        print(f"  {nf:>3} {q['p50']:>8.0f} {tr['p50']:>7.0f}  {ja['p50']:>6.3f} {pg['p50']:>6.3f} {tr['mean']/BW_Mbps*100:>6.1f}% {imp:>8}")
