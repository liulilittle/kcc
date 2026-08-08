# KCC v2.0 POST-FIX SIMULATION — verifies all fixes
# py -3 this.py
import random, math

BBR_UNIT=256; PG_MIN=BBR_UNIT//4; PG_MAX=268
AI_N=2; AI_D=100
MD_N=1; MD_D=16; MD_CM=4; MCAP=64; MCAPC=16
DN=92; DD=100
FJ=25; FD=2; FC=8; FT=8
STARTUP_GAIN=int(BBR_UNIT*2.89)
CWND_MAX=int(BBR_UNIT*2); CWND_MAX_R=999
P_INIT=int(BBR_UNIT*1.25); P_G=125; P_D=100
TGT=128; DR_DIV=32; DR_EXIT=4
S_C=0; S_P=1; S_S=2; S_D=3
SN={0:'CRUISE',1:'PROBE',2:'STARTUP',3:'DRAIN'}

MSS=1448; BW=1260.0; BWbps=BW*1e6; BD=BWbps/8; TP=35000
BDB=BD*TP*1e-6; BDPp=BDB/MSS

class Flow:
    def __init__(self,fid,brtt):
        self.fid=fid; self.brtt=brtt; self.pg=STARTUP_GAIN; self.cwnd_g=STARTUP_GAIN
        self.st=S_S; self.ez=0; self.fpr=0; self.fpc=0; self.plr=0
        self.bws=0; self.mbw=0.0; self.depg=999999999; self.drok=0; self.mr=brtt
    def step(self,excess,bw):
        if bw>self.mbw: self.mbw=bw; self.bws=0
        else: self.bws+=1
        if self.fpc>0: self.fpc-=1
        tp=max(1,self.mr); T=tp//TGT; D=tp//DR_DIV
        sq=lambda p:(p*p)//BBR_UNIT
        if self.st==S_C:
            if excess<=T or excess<self.depg: self.ez+=1
            else:
                self.ez=0; md=MD_D*MD_CM if self.fpc>0 else MD_D
                mc=MCAPC if self.fpc>0 else MCAP
                r=min((self.pg*excess*MD_N)//(max(1,tp)*md),mc)
                self.pg=max(self.pg-r,PG_MIN)
            self.depg=excess
            if excess>=D and self.fpc==0: self.st=S_D; self.depg=999999999; self.drok=0
            elif self.ez>=FT and self.fpc==0 and self.pg<PG_MAX-2: self.st=S_P; self.fpr=0
        elif self.st==S_P:
            self.fpr+=1
            if excess>T or self.fpr>=FD:
                self.fpc=FC
                if excess>T: self.st=S_C; self.ez=0; self.pg=max(PG_MIN,self.pg-12)
                elif self.pg>=PG_MAX-2: self.st=S_C
                else: self.st=S_S; self.plr=0
            else: self.pg=min(self.pg+FJ,PG_MAX)
        elif self.st==S_S:
            self.plr+=1; cg=self.pg
            if self.plr==1: self.pg=STARTUP_GAIN; cg=self.pg
            for _ in range(self.plr): cg=(cg*P_G)//P_D
            if self.plr==1 and self.fpc==0: cg=min(cg,STARTUP_GAIN)
            else: cg=min(cg,CWND_MAX)
            self.cwnd_g=cg
            if self.fpc==0: self.pg=min(self.cwnd_g,STARTUP_GAIN)
            else: self.pg=min(self.cwnd_g,PG_MAX)
            if excess>T: self.st=S_C; self.fpc=FC; self.ez=0
            elif self.plr>=CWND_MAX_R or (self.bws>=3 and self.plr>=2): self.st=S_C; self.fpc=FC; self.ez=0
            elif excess>=D: self.st=S_D; self.fpc=FC; self.depg=999999999; self.drok=0
        elif self.st==S_D:
            self.pg=max(self.pg*DN//DD,PG_MIN)
            if excess<=T: self.drok+=1
            elif excess<self.depg: self.drok+=1
            else: self.drok=0
            self.depg=excess
            if self.drok>=DR_EXIT: self.st=S_C; self.ez=0
        if self.st!=S_S: self.cwnd_g=sq(self.pg)

def sim(nf,n_rnds=800,seed=42,leave_after=0):
    rng=random.Random(seed)
    brtts=[max(3000,TP+rng.randint(-500,500)) for _ in range(nf)]
    flows=[Flow(i,brtts[i]) for i in range(nf)]
    stats=[]
    for rd in range(n_rnds):
        if leave_after>0 and rd==leave_after:
            # Remove half the flows
            flows=flows[:nf//2]
        nf2=len(flows); tw=sum(f.pg for f in flows)
        at=sum(f.brtt for f in flows)//nf2; rs=max(1e-9,at*1e-6)
        for _ in range(8):
            tr=0.0; ki=0.0
            for f in flows:
                pa=BWbps*f.pg/BBR_UNIT; cw=(BDPp*f.cwnd_g/BBR_UNIT)*MSS*8/rs
                tr+=min(pa,cw); ki+=min(pa,cw)*rs/8/MSS
            tr=min(tr,BWbps); qb=max(0.0,ki*MSS-BDB)
            rs=at*1e-6+qb/BD
        qu=qb/BD*1e6; ex=max(0.0,qu-at)
        for f in flows: f.step(ex,BWbps*f.pg/BBR_UNIT/1e6)
        if rd%20==0 or rd<30:
            sc={}
            for f in flows: sc[f.st]=sc.get(f.st,0)+1
            sts=','.join(f'{SN[k][:1]}{v}' for k,v in sorted(sc.items()))
            pgs=[f.pg/BBR_UNIT for f in flows]; pm=sum(pgs)/nf2
            frates=[BWbps*f.pg/tw/1e6 if tw>0 else 0.0 for f in flows]
            total_rate=sum(frates)
            stats.append({'rd':rd,'qu':qu,'pm':pm,'sts':sts,'rate':total_rate,'nf':nf2})
    return stats

print("="*90)
print("KCC v2.0 POST-FIX VALIDATION")
print(f"  BW={BW}Mbps T_prop={TP/1000:.0f}ms pg_init=2.89x MD=1/{MD_D} AI={AI_N}%")
print("="*90)

# TEST 1: Single flow cold start
print("\n--- TEST 1: Single flow cold start ---")
s=sim(1,150)
for r in s[:10]:
    print(f"  rd={r['rd']:>4} Q={r['qu']:>8.0f}us PG={r['pm']:.3f} Rate={r['rate']:.0f}Mbps {r['sts']}")
print(f"  Final rate (rd 140): {s[-1]['rate']:.0f}Mbps")

# TEST 2: 4-flow cold start + probe detection
print("\n--- TEST 2: 4-flow cold start — does PROBE fire? ---")
s=sim(4,600)
probe_seen=False; last_rate=0
for r in s:
    if 'P' in r['sts']: probe_seen=True
    last_rate=r['rate']
print(f"  PROBE state seen: {probe_seen}")
print(f"  Final rate: {last_rate:.0f}Mbps")
# Show state transitions
print(f"  State timeline:")
for r in s:
    if r['rd']%100==0 or r['rd']<50:
        print(f"    rd={r['rd']:>4} Q={r['qu']:>8.0f}us Rate={r['rate']:.0f}Mbps {r['sts']}")

# TEST 3: Flow leave recovery
print("\n--- TEST 3: 4 flows, 2 leave at rd=300 ---")
s=sim(4,600,leave_after=300)
for r in s:
    if 295<=r['rd']<=310 or r['rd']%50==0:
        print(f"  rd={r['rd']:>4} Q={r['qu']:>8.0f}us nf={r['nf']} Rate={r['rate']:.0f}Mbps {r['sts']}")

# TEST 4: 4-flow vs BBR comparison (steady state)
print("\n--- TEST 4: Steady-state rate (last 200 rounds) ---")
s=sim(4,800)
steady=s[-200:]
qs=sorted([r['qu'] for r in steady]);nq=len(qs)
rates=[r['rate'] for r in steady]
avg_q=sum(qs)/nq; avg_r=sum(rates)/len(rates)
q50=qs[nq//2]; q95=qs[min(nq-1,int(nq*0.95))]
print(f"  Q_avg={avg_q:.0f}us Q_P50={q50:.0f}us Q_P95={q95:.0f}us")
print(f"  Rate_avg={avg_r:.0f}Mbps ({avg_r/BW*100:.1f}% utilization)")
print(f"  vs BBR: BBR gets ~1070Mbps (85%) with 4 flows")
print(f"  KCC  target: >= 1100Mbps (87%)")

# TEST 5: STARTUP exit timing  
print("\n--- TEST 5: STARTUP exit timing ---")
s=sim(1,100)
startup_end=None
for r in s:
    if 'S' not in r['sts'] and startup_end is None:
        startup_end=r['rd']
print(f"  STARTUP exited at rd={startup_end}")
print(f"  Rate at exit: {s[startup_end]['rate']:.0f}Mbps" if startup_end else "  Still in STARTUP")
