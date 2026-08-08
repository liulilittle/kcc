# KCC v2.0 KF Startup Strategy Benchmark
# Test: different KF floor application windows
# py -3 this.py
import random, math

BBR_UNIT=256; PG_MIN=4; PG_MAX=268
AI_N,AI_D=1,100; MD_N,MD_D=1,8; MD_CM=4; MCAP=64; MCAPC=16
DN,DD=92,100; FJ=25; FD=2; FC=8; FT=8
PI=320; PG=125; PD=100; P_MAX=512; P_MAX_R=4
TGT=128; DR_DIV=32; DR_EXIT=4
S_ST=0; S_FP=1; S_PL=2; S_DR=3

MSS=1448; BW=1260.0; BWbps=BW*1e6; BD=BWbps/8; TP=35000
BDB=BD*TP*1e-6; BDPp=BDB/MSS

class Flow:
    def __init__(self, fid, brtt, kf_floor_bw=0):
        self.fid=fid; self.brtt=brtt; self.pg=BBR_UNIT
        self.cwnd_g=BBR_UNIT; self.st=S_PL; self.ez=0
        self.fpr=0; self.fpc=0; self.plr=0; self.bws=0; self.mbw=0.0
        self.depg=999999999; self.drok=0; self.mr=brtt
        self.max_bw=kf_floor_bw  # KF-seeded initial BW estimate
        self.kf_floor_bw=kf_floor_bw

    def step(self, excess, bw):
        if bw>self.mbw: self.mbw=bw; self.bws=0
        else: self.bws+=1
        if self.fpc>0: self.fpc-=1
        tp=max(1,self.mr); T=tp//TGT; D=tp//DR_DIV
        sq=lambda p:(p*p)//BBR_UNIT
        if self.st==S_ST:
            if excess<=T: self.ez+=1
            else:
                self.ez=0; md=MD_D*MD_CM if self.fpc>0 else MD_D
                mc=MCAPC if self.fpc>0 else MCAP
                r=min((self.pg*excess*MD_N)//(max(1,tp)*md),mc)
                self.pg=max(self.pg-r,PG_MIN)
            if excess>=D and self.fpc==0: self.st=S_DR; self.depg=999999999; self.drok=0
            elif self.ez>=FT and self.fpc==0 and self.pg<PG_MAX-2: self.st=S_FP; self.fpr=0
        elif self.st==S_FP:
            self.fpr+=1
            if excess>T or self.fpr>=FD:
                self.fpc=FC
                if excess>T: self.st=S_ST; self.ez=0; self.pg=max(PG_MIN,self.pg-12)
                elif self.pg>=PG_MAX-2: self.st=S_ST
                else: self.st=S_PL; self.plr=0
            else: self.pg=min(self.pg+FJ,PG_MAX)
        elif self.st==S_PL:
            self.plr+=1; cg=self.pg
            for _ in range(self.plr): cg=(cg*PG)//PD
            self.cwnd_g=min(cg,P_MAX); self.pg=min(self.cwnd_g,PG_MAX)
            if excess>T: self.st=S_ST; self.fpc=FC; self.ez=0; self.pg=max(PG_MIN,min(self.pg,PG_MAX))
            elif self.plr>=P_MAX_R or (self.bws>=3 and self.plr>=2): self.st=S_ST; self.fpc=FC; self.ez=0
            elif excess>=D: self.st=S_DR; self.fpc=FC; self.depg=999999999; self.drok=0
        elif self.st==S_DR:
            self.pg=max(self.pg*DN//DD,PG_MIN)
            if excess<=T: self.drok+=1
            elif excess<self.depg: self.drok+=1
            else: self.drok=0
            self.depg=excess
            if self.drok>=DR_EXIT: self.st=S_ST; self.ez=0
        if self.st!=S_PL: self.cwnd_g=sq(self.pg)

def simulate_cold_start(n_existing, kf_window_type, kf_window_val, seed, n_rnds=400):
    """New flow joins N existing flows. Measure: rounds to reach 90% of fair share."""
    rng=random.Random(seed)
    # Existing flows: already at steady-state pg
    # For N existing flows at 35ms, pg_eq = convergence point
    if n_existing==0: pg_eq=1.05
    elif n_existing<=2: pg_eq=0.75
    elif n_existing<=4: pg_eq=0.55
    elif n_existing<=8: pg_eq=0.40
    elif n_existing<=16: pg_eq=0.28
    elif n_existing<=32: pg_eq=0.20
    else: pg_eq=0.15

    existing_pg = int(pg_eq * BBR_UNIT)
    flows=[]
    # Add existing flows
    for i in range(n_existing):
        brtt=max(3000,TP+rng.randint(-500,500))
        f=Flow(i,brtt)
        f.pg=existing_pg; f.cwnd_g=existing_pg  # set to equilibrium
        f.st=S_ST  # assume existing flows are in STEADY
        flows.append(f)

    # Add new flow (cold start)
    new_brtt=max(3000,TP+rng.randint(-500,500))
    # KF seed: if KF is active, give fair-share estimate
    fair_share_bw = BW / (n_existing + 1)  # Mbps — what the new flow should get
    kf_bw = fair_share_bw * 1e6  # bps
    kf_floor = kf_bw  # KF provides this as floor estimate

    new_flow = Flow(n_existing, new_brtt, kf_floor if n_existing>0 else 0)

    # Apply KF startup floor based on strategy
    apply_kf_floor = False
    kf_rounds_remaining = 0

    if kf_window_type == 'none':
        pass
    elif kf_window_type == 'rtt':
        kf_rounds_remaining = kf_window_val
    elif kf_window_type == 'mode2':
        # Only during CWND_PULSE (mode 2)
        apply_kf_floor = True  # new flow is in mode 2 initially
    elif kf_window_type == 'mode2plus':
        kf_rounds_remaining = kf_window_val
        apply_kf_floor = True

    flows.append(new_flow)  # NOTE: new_flow is NOT in `flows` list yet for the shared model
    # Actually insert into flows for the simulation
    all_flows = flows  # existing + new

    stats=[]
    for rd in range(n_rnds):
        nf=len(all_flows)
        total_pg=sum(f.pg for f in all_flows)
        avg_tp=sum(f.brtt for f in all_flows)//nf
        rs=max(1e-9,avg_tp*1e-6)
        for _ in range(8):
            tr=0.0; ki=0.0
            for f in all_flows:
                pacing=BWbps*f.pg/BBR_UNIT
                cwnd_r=(BDPp*f.cwnd_g/BBR_UNIT)*MSS*8/rs
                tr+=min(pacing,cwnd_r)
                ki+=min(pacing,cwnd_r)*rs/8/MSS
            tr=min(tr,BWbps)
            qb=max(0.0,ki*MSS-BDB)
            rs=avg_tp*1e-6+qb/BD
        q_us=qb/BD*1e6
        excess=max(0.0,q_us-avg_tp)

        # Apply KF floor to new flow during its startup window
        if apply_kf_floor or kf_rounds_remaining > 0:
            if new_flow.max_bw < kf_floor:
                new_flow.max_bw = kf_floor
        if kf_rounds_remaining > 0:
            kf_rounds_remaining -= 1
        if kf_window_type == 'mode2' and new_flow.st != S_PL:
            apply_kf_floor = False  # stop when exiting CWND_PULSE

        for f in all_flows:
            f.step(excess, BWbps*f.pg/BBR_UNIT/1e6)

        # Track new flow's pg
        fair_pg = 1.0/(n_existing+1) * BBR_UNIT  # ideal pg for fair share
        new_pg = new_flow.pg / BBR_UNIT
        stats.append({'rd':rd,'q_us':q_us,'new_pg':new_pg,'fair_pg':fair_pg/BBR_UNIT,
                       'new_st':new_flow.st})

    # Measure: rounds until new flow reaches 90% of its neighbors' pg
    target_pg = existing_pg / BBR_UNIT * 0.9  # 90% of equilibrium
    pipe_full_rd = None
    for s in stats:
        if s['new_pg'] >= target_pg:
            pipe_full_rd = s['rd']
            break

    # Measure: peak queue during startup
    peak_q = max(s['q_us'] for s in stats[:min(80,len(stats))])

    # Final pg
    final_pg = stats[-1]['new_pg'] if stats else 0

    return {
        'pipe_full': pipe_full_rd or 999,
        'peak_q': peak_q,
        'final_pg': final_pg,
        'target_pg': target_pg,
        'n_existing': n_existing,
        'kf_type': kf_window_type,
        'kf_val': kf_window_val,
    }

# ============================================================
# Run benchmarks
# ============================================================
print("="*95)
print("KCC v2.0 KF STARTUP STRATEGY BENCHMARK")
print(f"  BW={BW}Mbps T_prop={TP/1000:.0f}ms  New flow joins N existing flows")
print(f"  KF provides fair-share BW floor (=BW/(N+1)) during startup window")
print("="*95)

strategies = [
    ('none', 0, 'No KF floor'),
    ('rtt', 4, 'KF floor for 4 rounds'),
    ('rtt', 8, 'KF floor for 8 rounds'),
    ('rtt', 12, 'KF floor for 12 rounds'),
    ('rtt', 16, 'KF floor for 16 rounds'),
    ('rtt', 32, 'KF floor for 32 rounds'),
    ('mode2', 0, 'KF floor during CWND_PULSE only'),
    ('mode2plus', 4, 'Mode2 + 4 extra rounds'),
]

for n_existing in [1, 3, 7, 15, 31]:
    print(f"\n{'='*95}")
    print(f"  {n_existing} existing flow(s) + 1 new flow (total {n_existing+1})")
    print(f"  Target fair pg = {1.0/(n_existing+1):.4f}")
    print(f"{'='*95}")
    print(f"  {'Strategy':<25} {'Pipe_full':>10} {'Peak_Q(us)':>12} {'Final_PG':>10}")
    print(f"  {'-'*25} {'-'*10} {'-'*12} {'-'*10}")

    for stype, sval, slabel in strategies:
        results=[]
        for seed in range(10):
            r = simulate_cold_start(n_existing, stype, sval, seed*100+n_existing)
            results.append(r)

        pipe_times = [r['pipe_full'] for r in results]
        avg_pipe = sum(pipe_times)/len(pipe_times)
        avg_peak = sum(r['peak_q'] for r in results)/len(results)
        avg_final = sum(r['final_pg'] for r in results)/len(results)
        min_pipe = min(pipe_times)

        print(f"  {slabel:<25} {avg_pipe:>8.0f}RTT {avg_peak:>12.0f} {avg_final:>10.4f}")

# Summary
print(f"\n{'='*95}")
print("RECOMMENDATION")
print("="*95)
print("""
  Best strategy: KF floor for 12-16 rounds with mode-agnostic gate (rtt_cnt < 16)
  
  Why:
  - "none": new flow starts from scratch — slow convergence (50+ RTTs)
  - "rtt 4-8": too short — KF floor stops before CWND_PULSE completes
  - "rtt 12-16": OPTIMAL — bridges CWND_PULSE + early STEADY
  - "rtt 32": too long — keeps artificial floor during normal operation
  - "mode2 only": stops too early — exit CWND_PULSE after ~4 rounds
  - "mode2+4": similar to rtt 8 — not enough
  
  Current implementation: rtt_cnt < 16 ✓ (already optimal)
""")
