# KCC v2.0 FINAL — 1-1024 flows, KF-ON/OFF, comprehensive stats
# py -3 this.py
import random, math

BBR_UNIT=256; PG_MIN=BBR_UNIT//64; PG_MAX=268
AI_N,AI_D=1,100; MD_N,MD_D=1,8; MD_CM=4; MCAP=64; MCAPC=16
DN,DD=92,100; FJ=25; FD=2; FC=8; FT=8
P_INIT=int(BBR_UNIT*1.25); P_G=125; P_D=100; P_MAX=int(BBR_UNIT*2); P_MAX_R=3
TGT=128; DR_DIV=32; DR_EXIT=4
S_C=0; S_P=1; S_S=2; S_D=3
SN={0:'CRUISE',1:'PROBE',2:'STARTUP',3:'DRAIN'}

MSS=1448; BW=1260.0; BWbps=BW*1e6; BD=BWbps/8; TP=35000
BDB=BD*TP*1e-6; BDPp=BDB/MSS

class Flow:
    def __init__(self,fid,brtt,kf_active=False):
        self.fid=fid; self.brtt=brtt; self.pg=BBR_UNIT; self.cwnd_g=BBR_UNIT
        self.st=S_S; self.ez=0; self.fpr=0; self.fpc=0; self.plr=0
        self.bws=0; self.mbw=0.0; self.depg=999999999; self.drok=0; self.mr=brtt
        self.kf=kf_active; self.spring_fired=False

    def step(self,excess,bw):
        if bw>self.mbw: self.mbw=bw; self.bws=0
        else: self.bws+=1
        if self.fpc>0: self.fpc-=1
        tp=max(1,self.mr); T=tp//TGT; D=tp//DR_DIV
        sq=lambda p:(p*p)//BBR_UNIT

        if self.st==S_C:
            if excess<=T: self.ez+=1
            else:
                self.ez=0; md=MD_D*MD_CM if self.fpc>0 else MD_D
                mc=MCAPC if self.fpc>0 else MCAP
                r=min((self.pg*excess*MD_N)//(max(1,tp)*md),mc)
                self.pg=max(self.pg-r,PG_MIN)
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
            # STARTUP springboard: round 1 = 2.89x unconditionally
            if self.plr==1: self.pg=int(BBR_UNIT*2.89); cg=self.pg; self.spring_fired=True
            for _ in range(self.plr): cg=(cg*P_G)//P_D
            # Cap: cold-start 2.89x; probing 2.0x
            if self.spring_fired and self.plr==1: cg=min(cg,int(BBR_UNIT*2.89))
            else: cg=min(cg,P_MAX)
            self.cwnd_g=cg; self.pg=min(self.cwnd_g,PG_MAX)
            if excess>T: self.st=S_C; self.fpc=FC; self.ez=0; self.pg=max(PG_MIN,min(self.pg,PG_MAX))
            elif self.plr>=P_MAX_R or (self.bws>=3 and self.plr>=2): self.st=S_C; self.fpc=FC; self.ez=0
            elif excess>=D: self.st=S_D; self.fpc=FC; self.depg=999999999; self.drok=0

        elif self.st==S_D:
            self.pg=max(self.pg*DN//DD,PG_MIN)
            if excess<=T: self.drok+=1
            elif excess<self.depg: self.drok+=1
            else: self.drok=0
            self.depg=excess
            if self.drok>=DR_EXIT: self.st=S_C; self.ez=0
        if self.st!=S_S: self.cwnd_g=sq(self.pg)

def sim(nf,kf_seed,n_rnds=800,seed=42):
    rng=random.Random(seed)
    brtts=[max(3000,TP+rng.randint(-500,500)) for _ in range(nf)]
    # If kf_seed: seed initial pg from "KF" estimate (fair-share)
    initial_pg = int(BBR_UNIT/(nf) if kf_seed and nf>0 else BBR_UNIT)
    initial_pg = max(PG_MIN, min(initial_pg, BBR_UNIT))
    flows=[Flow(i,brtts[i],kf_seed) for i in range(nf)]
    if kf_seed:
        for f in flows: f.pg=initial_pg; f.cwnd_g=initial_pg  # KF-seeded fair-share
    stats=[]
    for rd in range(n_rnds):
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
        if rd>=max(200,n_rnds//4):
            pgs=[f.pg/BBR_UNIT for f in flows]; pm=sum(pgs)/nf2
            ps=math.sqrt(sum((p-pm)**2 for p in pgs)/nf2) if nf2>1 else 0
            sc={}
            for f in flows: sc[f.st]=sc.get(f.st,0)+1
            frates=[BWbps*f.pg/tw/1e6 if tw>0 else 0.0 for f in flows]
            jn=1.0 if nf2==1 else (sum(frates)**2/(nf2*sum(r*r for r in frates))) if sum(r*r for r in frates)>0 else 0
            stats.append({'rd':rd,'qu':qu,'pm':pm,'ps':ps,'pmin':min(pgs),'pmax':max(pgs),
                          'sts':dict(sc),'jain':jn,'frates':frates})
    return stats

def metrics(stats,nf,bw):
    ss=stats[-200:]; nq=len(ss)
    qs=sorted([s['qu'] for s in ss]); q50=qs[nq//2]; q95=qs[min(nq-1,int(nq*0.95))]
    pq=max(s['qu'] for s in stats[:60])
    pm=sum(s['pm'] for s in ss)/nq; psa=sum(s['ps'] for s in ss)/nq
    pspread=sum(s['pmax']-s['pmin'] for s in ss)/nq
    ja=sum(s['jain'] for s in ss)/nq
    frates=[sum([s['frates'][fi] for s in ss if fi<len(s['frates'])])/max(1,len([s for s in ss if fi<len(s['frates'])])) for fi in range(nf)]
    total_rate=sum(frates); ef=total_rate/bw*100
    sc={}
    for s in ss:
        for k,v in s['sts'].items(): sc[k]=sc.get(k,0)+v
    tot=sum(sc.values()); st_pct={k:v/tot*100 for k,v in sc.items()} if tot else {}
    # Cold start: rounds until STOPS being in STARTUP (enters CRUISE/PROBE/DRAIN)
    cold_rounds=None
    for s in stats:
        sts=s['sts']
        if sts.get(S_S,0)<nf: cold_rounds=s['rd']; break
    return {'q50':q50,'q95':q95,'peak_q':pq,'pm':pm,'ps':psa,'spread':pspread,
            'jain':ja,'rate':total_rate,'eff':ef,'st_pct':st_pct,'cold':cold_rounds or 999,'frates':frates}

def st(arr):
    s=sorted(arr); n=len(s)
    return {'mean':sum(s)/n,'p50':s[n//2],'p5':s[max(0,n//20)],'p95':s[min(n-1,n*19//20)]} if n else {'mean':0,'p50':0,'p5':0,'p95':0}

if __name__=='__main__':
    import time
    N_SEEDS=20
    print("="*100)
    print("KCC v2.0 COMPREHENSIVE — 1-1024 flows, KF-ON/OFF, {N_SEEDS} seeds")
    print("="*100)

    all_results=[]
    for kf_label,kf_on in [("KF_OFF",False),("KF_ON",True)]:
        print(f"\n{'='*100}")
        print(f"   {kf_label}")
        print(f"{'='*100}")
        print(f"  {'N':>5} {'Q_P50':>8} {'Q_P95':>8} {'PeakQ':>8} {'PG':>6} {'std':>6} {'Spr':>6} {'Jain':>6} {'Eff%':>6} {'ColdR':>6} {'States'}")
        print(f"  {'-'*5} {'-'*8} {'-'*8} {'-'*8} {'-'*6} {'-'*6} {'-'*6} {'-'*6} {'-'*6} {'-'*6} {'-'*45}")
        for nf in [1,2,4,8,16,32,64,128,256,512,1024]:
            seeds_use=N_SEEDS if nf<=128 else max(5,N_SEEDS//4)
            res=[]
            for s in range(seeds_use):
                stats=sim(nf,kf_on,800,42+s)
                m=metrics(stats,nf,BW); m['nf']=nf; m['seed']=s; m['kf']=kf_on
                res.append(m); all_results.append(m)
            q=st([r['q50'] for r in res]); q95=st([r['q95'] for r in res])
            pq=st([r['peak_q'] for r in res]); pm=st([r['pm'] for r in res])
            ps=st([r['ps'] for r in res]); sp=st([r['spread'] for r in res])
            ja=st([r['jain'] for r in res]); ef=st([r['eff'] for r in res])
            cd=st([r['cold'] for r in res])
            sc_all={}
            for r in res:
                for k,v in r['st_pct'].items(): sc_all[k]=sc_all.get(k,0)+v
            tot=sum(sc_all.values()); sts=' '.join(f'{SN[k]}={v/tot*100:.0f}%' for k,v in sorted(sc_all.items()))
            print(f"  {nf:>5} {q['p50']:>8.0f} {q95['p50']:>8.0f} {pq['p50']:>8.0f} {pm['p50']:>6.3f} {ps['p50']:>6.4f} {sp['p50']:>6.4f} {ja['p50']:>6.4f} {ef['p50']:>5.1f}% {cd['p50']:>5.0f}  {sts}")
            # Per-flow equity for small N
            if nf<=8 and res:
                frs=list(zip(*[r['frates'] for r in res]))
                for fi in range(nf):
                    sfr=st(frs[fi]);
                    print(f"  F{fi} rate={sfr['p50']:.0f}Mbps  ",end='')
                print()

    print(f"\n{'='*100}")
    print("COMPARISON: KF-OFF vs KF-ON")
    print(f"{'='*100}")
    print(f"  {'N':>5} {'Q50_OFF':>8} {'Q50_ON':>8} {'PG_OFF':>7} {'PG_ON':>7} {'Cold_OFF':>8} {'Cold_ON':>8}")
    print(f"  {'-'*5} {'-'*8} {'-'*8} {'-'*7} {'-'*7} {'-'*8} {'-'*8}")
    for nf in [1,2,4,8,16,32,64,128,256,512,1024]:
        off=[r for r in all_results if r['nf']==nf and r['kf']==False]
        on=[r for r in all_results if r['nf']==nf and r['kf']==True]
        if off and on:
            qoff=st([r['q50'] for r in off]); qon=st([r['q50'] for r in on])
            poff=st([r['pm'] for r in off]); pon=st([r['pm'] for r in on])
            coff=st([r['cold'] for r in off]); con=st([r['cold'] for r in on])
            print(f"  {nf:>5} {qoff['p50']:>8.0f} {qon['p50']:>8.0f} {poff['p50']:>6.3f} {pon['p50']:>6.3f} {coff['p50']:>8.0f} {con['p50']:>8.0f}")
