# KCC v2.0 FINAL FIX VALIDATION — match C code exactly
# py -3 this.py
import random, math

BBR_UNIT=256; PG_MIN=BBR_UNIT//4; PG_MAX=320
AI_N=1; AI_D=100
MD_N=1; MD_D=8; MD_CM=4; MCAP=64; MCAPC=16
DN=92; DD=100
FJ=25; FD=2; FC=8; FT=8
STARTUP_GAIN=int(BBR_UNIT*2.89)
CWND_MAX=BBR_UNIT*2
P_INIT=int(BBR_UNIT*1.25); P_G=125; P_D=100
TGT=128; DR_DIV=32; DR_EXIT=4; PG_MM=BBR_UNIT//100
S_C=0; S_P=1; S_S=2; S_D=3
SN={0:'CRUISE',1:'PROBE',2:'STARTUP',3:'DRAIN'}

MSS=1448; BW=1260.0; BWbps=BW*1e6; BD=BWbps/8
TP=35000; BDB=BD*TP*1e-6; BDPp=BDB/MSS

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

        # CRUISE
        if self.st==S_C:
            if excess<=T: self.ez+=1
            else:
                self.ez=0; md=MD_D*MD_CM if self.fpc>0 else MD_D
                mc=MCAPC if self.fpc>0 else MCAP
                r=min((self.pg*excess*MD_N)//(max(1,tp)*md),mc)
                self.pg=max(self.pg-r,PG_MIN)
            if excess>=D and self.fpc==0: self.st=S_D; self.depg=999999999; self.drok=0
            elif self.ez>=FT and self.fpc==0 and self.pg<PG_MAX-PG_MM: self.st=S_P; self.fpr=0
            elif self.fpc==0 and self.bws>=8 and self.pg<PG_MAX-PG_MM and excess<tp//4: self.st=S_P; self.fpr=0; self.bws=0

        # PROBE (FAST_PROBE)
        elif self.st==S_P:
            self.fpr+=1
            if excess>T or self.fpr>=FD:
                self.fpc=FC
                if excess>T: self.st=S_C; self.ez=0; self.pg=max(PG_MIN,self.pg-12)
                elif self.pg>=PG_MAX-PG_MM: self.st=S_C
                else: self.st=S_C; self.ez=0
            else: self.pg=min(self.pg+FJ,PG_MAX)

        # STARTUP
        elif self.st==S_S:
            self.plr+=1; cg=self.pg
            if self.plr==1: self.pg=STARTUP_GAIN; cg=self.pg
            for _ in range(self.plr): cg=(cg*P_G)//P_D
            cg=min(cg,STARTUP_GAIN) if self.plr==1 and self.fpc==0 else min(cg,CWND_MAX)
            self.cwnd_g=cg
            self.pg=min(self.cwnd_g,STARTUP_GAIN) if self.fpc==0 else min(self.cwnd_g,PG_MAX)
            if excess>T: self.st=S_C; self.fpc=FC; self.ez=0
            elif self.bws>=3 and self.plr>=2: self.st=S_C; self.fpc=FC; self.ez=0
            elif excess>=D: self.st=S_D; self.fpc=FC; self.depg=999999999; self.drok=0

        # DRAIN
        elif self.st==S_D:
            self.pg=max(self.pg*DN//DD,PG_MIN)
            if excess<=T: self.drok+=1
            elif excess<self.depg: self.drok+=1
            else: self.drok=0
            self.depg=excess
            if self.drok>=DR_EXIT: self.st=S_C; self.ez=0

        if self.st!=S_S: self.cwnd_g=sq(self.pg)

def sim(nf,n_rnds=600,seed=42):
    rng=random.Random(seed)
    brtts=[max(3000,TP+rng.randint(-500,500)) for _ in range(nf)]
    flows=[Flow(i,brtts[i]) for i in range(nf)]
    stats=[]
    for rd in range(n_rnds):
        nf2=len(flows); tw=sum(f.pg for f in flows)
        at=sum(f.brtt for f in flows)//nf2; rs=max(1e-9,at*1e-6)
        for _ in range(8):
            tr=0.0; ki=0.0
            for f in flows:
                pa=BWbps*f.pg/BBR_UNIT; cw=(BDPp*f.cwnd_g/BBR_UNIT)*MSS*8/rs
                tr+=min(pa,cw); ki+=min(pa,cw)*rs/8/MSS
            tr=min(tr,BWbps); qb=max(0.0,ki*MSS-BDB); rs=at*1e-6+qb/BD
        qu=qb/BD*1e6; ex=max(0.0,qu-at)
        for f in flows: f.step(ex,BWbps*f.pg/BBR_UNIT/1e6)
        if rd>=100:
            pgs=[f.pg/BBR_UNIT for f in flows]; pm=sum(pgs)/nf2
            sc={}
            for f in flows: sc[f.st]=sc.get(f.st,0)+1
            frates=[BWbps*f.pg/tw/1e6 if tw>0 else 0.0 for f in flows]
            stats.append({'rd':rd,'qu':qu,'pm':pm,'sts':dict(sc),'rate':sum(frates),
                          'pg_min':min(pgs),'pg_max':max(pgs)})
    return stats

def metrics(stats,nf):
    ss=stats[-200:]; nq=len(ss)
    qs=sorted([s['qu'] for s in ss]); q50=qs[nq//2]; q95=qs[min(nq-1,int(nq*0.95))]
    rates=[s['rate'] for s in ss]; avg_r=sum(rates)/nq
    pgs=[s['pm'] for s in ss]; avg_pg=sum(pgs)/nq
    sc={}
    for s in ss:
        for k,v in s['sts'].items(): sc[k]=sc.get(k,0)+v
    tot=sum(sc.values()); sts=' '.join(f'{SN[k]}={v/tot*100:.0f}%' for k,v in sorted(sc.items()))
    pq=max(s['qu'] for s in stats[:60])
    # PROBE count
    probe_ct=sum(1 for s in ss if S_P in s['sts'])
    return {'q50':q50,'q95':q95,'peak_q':pq,'rate':avg_r,'eff':avg_r/BW*100,
            'pg':avg_pg,'sts':sts,'probe_pct':probe_ct/nq*100}

print("="*90)
print("KCC v2.0 BENCHMARK — PG_MAX=1.25x, MD=1/8, AI=1%, pg_init=2.89x")
print("="*90)

for nf in [1,2,4]:
    stats=sim(nf,600)
    m=metrics(stats,nf)
    print(f"\n  N={nf}: Rate={m['rate']:.0f}Mbps ({m['eff']:.1f}%) PG={m['pg']:.3f} "
          f"Q50={m['q50']:.0f}us Q95={m['q95']:.0f}us PeakQ={m['peak_q']:.0f}us")
    print(f"  States: {m['sts']}")
    print(f"  PROBE rounds: {m['probe_pct']:.0f}%")
    # Time evolution
    for s in stats:
        if s['rd']%100==0 or (s['rd']<30 and s['rd']%5==0):
            sc=s['sts']; sts=','.join(f'{SN[k][:1]}{v}' for k,v in sorted(sc.items()))
            print(f"    rd={s['rd']:>4} Q={s['qu']:>8.0f}us PG={s['pm']:.3f} Rate={s['rate']:.0f}Mbps {sts}")

# vs BBR comparison
print(f"\n{'='*90}")
print("vs BBR (same config)")
print("  BBR 4f: ~1070Mbps (85%)  Q~175ms")
print("  KCC 4f: see above")
print(f"{'='*90}")
