# KCC v2.0 KF Springboard Test — pg boost + excess circuit breaker
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

KF_BOOST_PG = BBR_UNIT * 3 // 4  # 0.75x

class Flow:
    def __init__(self, fid, brtt, kf_active=False):
        self.fid=fid; self.brtt=brtt; self.pg=BBR_UNIT
        self.cwnd_g=BBR_UNIT; self.st=S_PL; self.ez=0
        self.fpr=0; self.fpc=0; self.plr=0; self.bws=0; self.mbw=0.0
        self.depg=999999999; self.drok=0; self.mr=brtt
        self.kf_active=kf_active  # KF provides springboard

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
            self.plr+=1
            cg=self.pg
            # KF springboard: boost on first pulse round
            if self.plr==1 and self.kf_active:
                self.pg=max(self.pg, KF_BOOST_PG)
                cg=self.pg
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

def simulate(n_existing, kf_active, seed, n_rnds=200):
    rng=random.Random(seed)
    # Equilibrium pg for existing flows
    pg_map={0:1.05,1:0.75,3:0.55,7:0.40,15:0.28,31:0.20}
    pg_eq=pg_map.get(n_existing,0.15)
    exist_pg=int(pg_eq*BBR_UNIT)

    flows=[]
    for i in range(n_existing):
        brtt=max(3000,TP+rng.randint(-500,500))
        f=Flow(i,brtt); f.pg=exist_pg; f.cwnd_g=exist_pg; f.st=S_ST
        flows.append(f)

    new_flow=Flow(n_existing, max(3000,TP+rng.randint(-500,500)), kf_active)
    flows.append(new_flow)

    stats=[]
    for rd in range(n_rnds):
        nf=len(flows)
        total_pg=sum(f.pg for f in flows)
        avg_tp=sum(f.brtt for f in flows)//nf
        rs=max(1e-9,avg_tp*1e-6)
        for _ in range(8):
            tr=0.0; ki=0.0
            for f in flows:
                pacing=BWbps*f.pg/BBR_UNIT
                cwnd_r=(BDPp*f.cwnd_g/BBR_UNIT)*MSS*8/rs
                tr+=min(pacing,cwnd_r); ki+=min(pacing,cwnd_r)*rs/8/MSS
            tr=min(tr,BWbps); qb=max(0.0,ki*MSS-BDB)
            rs=avg_tp*1e-6+qb/BD
        q_us=qb/BD*1e6; excess=max(0.0,q_us-avg_tp)
        for f in flows: f.step(excess, BWbps*f.pg/BBR_UNIT/1e6)

        new_pg=new_flow.pg/BBR_UNIT; new_st=new_flow.st
        st_names={0:'S',1:'F',2:'P',3:'D'}
        stats.append({'rd':rd,'q_us':q_us,'new_pg':new_pg,'new_st':st_names[new_st]})

    return stats

# ============================================================
print("="*90)
print("KCC v2.0 KF SPRINGBOARD TEST — pg boost + excess circuit breaker")
print(f"  KF boost: pg >= {KF_BOOST_PG/BBR_UNIT:.2f}x on first CWND_PULSE round")
print(f"  Circuit breaker: excess > T_prop/128 exits CWND_PULSE immediately")
print("="*90)

for n_existing in [0, 1, 3, 7, 15]:
    print(f"\n{'='*90}")
    print(f"  {n_existing} existing flow(s) + 1 new flow  (target fair pg ~ {1/(n_existing+1):.3f})")
    print(f"{'='*90}")

    for kf_label, kf_on in [("KF OFF", False), ("KF ON ", True)]:
        stats=simulate(n_existing, kf_on, 42+n_existing)
        # Cold start trace (first 30 rounds)
        print(f"\n  [{kf_label}] Cold start trace:")
        print(f"  {'Rnd':>4} {'Q(us)':>9} {'new_pg':>7} {'st':>3}  {'Event'}")
        for s in stats[:20]:
            evt=""
            if s['rd']==0: evt="CWND_PULSE start"
            elif s['new_st']=='S' and stats[s['rd']-1]['new_st']=='P': evt="excess -> exit pulse"
            elif s['new_st']=='D': evt="DRAINING"
            print(f"  {s['rd']:>4} {s['q_us']:>9.0f} {s['new_pg']:>7.4f} {s['new_st']:>3}  {evt}")

        # Pipe fill time: rounds until new_pg reaches 90% of existing pg
        pg_eq=1/(n_existing+1) if n_existing>0 else 1.0
        target=pg_eq*0.9
        fill_rd=999
        for s in stats:
            if s['new_pg']>=target:
                fill_rd=s['rd']; break
        peak_q=max(s['q_us'] for s in stats[:min(40,len(stats))])
        final_pg=stats[-1]['new_pg']

        print(f"  Pipe fill: {fill_rd}RTT  Peak Q: {peak_q:.0f}us  Final PG: {final_pg:.4f}")

# Summary
print(f"\n{'='*90}")
print("SUMMARY")
print(f"{'='*90}")
print(f"  {'Existing':>9} {'KF':>5} {'Pipe_fill':>10} {'Peak_Q':>9} {'Final_PG':>9}")
print(f"  {'-'*9} {'-'*5} {'-'*10} {'-'*9} {'-'*9}")
for n_existing in [0, 1, 3, 7, 15]:
    for kf_on in [False, True]:
        stats=simulate(n_existing, kf_on, 42+n_existing)
        pg_eq=1/(n_existing+1) if n_existing>0 else 1.0
        target=pg_eq*0.9
        fill_rd=999
        for s in stats: 
            if s['new_pg']>=target: fill_rd=s['rd']; break
        peak_q=max(s['q_us'] for s in stats[:40])
        final_pg=stats[-1]['new_pg']
        kf_str="ON" if kf_on else "OFF"
        print(f"  {n_existing:>9} {kf_str:>5} {fill_rd:>8}RTT {peak_q:>9.0f} {final_pg:>9.4f}")
