#!/usr/bin/env python3
"""
Physical comparison: pg2 vs pg with bw_est='C' vs bw_est='C/N' models.
"""
import sys

AI=0.02; PG_MIN=0.75; PG_MAX=1.25; MD_NUM,MD_DEN=1,1; DR=64

class Flow:
    __slots__=('n','pg','gf','bwmodel','tp','is_bbr')
    def __init__(s,n,pg=1.,gf="pg",bwmodel="cn",is_bbr=False):
        s.n=n;s.pg=pg;s.gf=gf;s.bwmodel=bwmodel;s.tp=0;s.is_bbr=is_bbr
    def cg(s):
        if s.is_bbr: return 2.0
        if s.gf=="pg": return s.pg
        if s.gf=="pg2": return s.pg*s.pg
        return 1.0
    def rate_estimate(s, C, N):
        """bandwidth estimate for pacing: 'C'=full link, 'cn'=C/N fair share"""
        if s.bwmodel=="C": return C
        return C / max(N,1)
    def cwnd_bytes(s, C, T, N):
        """cwnd = cwnd_gain * bw_est * T_prop"""
        return s.cg() * s.rate_estimate(C,N) * T

def upd(fl,C,T,rnd):
    N=len(fl)
    tc=sum(f.cwnd_bytes(C,T,N) for f in fl)
    Q=max(0.,tc/C - T)
    for f in fl:
        if f.is_bbr: continue
        if Q<T/128.: f.pg=min(f.pg+AI,PG_MAX)
        else: md=f.pg*Q*MD_NUM/(T*MD_DEN);f.pg=max(f.pg-md,PG_MIN)
        if(rnd&63)==0:f.pg=0.75
    # BBR cycle
    cyc=[1.25,0.75,1.0,1.0,1.0,1.0,1.0,1.0]
    for f in fl:
        if f.is_bbr:f.pg=cyc[rnd&7]
    t2=sum(f.cwnd_bytes(C,T,N) for f in fl)
    Q2=max(0.,t2/C - T)
    rt=T+Q2
    for f in fl: f.tp=f.cwnd_bytes(C,T,N)/rt
    return Q2

def sim(N,C,T,R,nb=0,gf="pg",bw="cn",w=500):
    fl=[Flow("K%d"%i,gf=gf,bwmodel=bw)for i in range(N-nb)]+\
       [Flow("B%d"%i,is_bbr=True,bwmodel=bw)for i in range(nb)]
    for r in range(w):upd(fl,C,T,r)
    qs=[];pd={f.n:[]for f in fl};td=pd.copy()
    for r in range(R):
        Q=upd(fl,C,T,r+w);qs.append(Q)
        for f in fl:pd[f.n].append(f.pg);td[f.n].append(f.tp)
    aq=sum(qs)*1000/len(qs);pq=sorted(qs)[int(len(qs)*.95)]*1000
    dets=[]
    for f in fl:
        ap=sum(pd[f.n])/len(pd[f.n]);at=sum(td[f.n])/len(td[f.n])/1e6
        dets.append((f.n,ap,at,f.is_bbr))
    return aq,pq,dets

def main():
    C=1.26e9;T=0.060;R=3000;N=4
    print("%-5s %-6s %6s %6s %8s %8s %8s %8s" % ("bw_est","cwnd","Qavg","Qp95","tot_tp","min_tp","max_tp","BBR/KCC"))
    print("-"*75)
    for bw in ["cn","C"]:
        for gf in ["pg","pg2"]:
            for nb in [0,1,2]:
                aq,pq,dets=sim(N,C,T,R,nb=nb,gf=gf,bw=bw)
                ttl=sum(d[2]for d in dets)
                kccs=[d[2]for d in dets if not d[3]]
                bbrs=[d[2]for d in dets if d[3]]
                mink=min(kccs)if kccs else 0;maxb=max(bbrs)if bbrs else 0
                ratio="%.2f"%(maxb/mink) if kccs and bbrs else "--"
                print("%-5s %-6s %6.2f %6.2f %8.0f %8.0f %8.0f %8s"%(bw,gf,aq,pq,ttl,mink,maxb,ratio))

    # Theory
    print("\nTheory for bw_est=C/N (BBR standard):")
    print("  Q = (avg_cwnd_gain - 1)*T_prop")
    print("  pg:    inflight in [0.75, 1.25] BDP    Q in [0, 15ms]")
    print("  pg2:   inflight in [0.56, 1.56] BDP    Q in [0, 33.8ms]")
    print("\nTheory for bw_est=C (user's model):")
    print("  pg = 1/N, cwnd = pg2*BDP = BDP/N2")
    print("  N=4: total inflight = BDP/4 = 25% BDP  (75% headroom)")
    print("  N=1: total inflight = BDP = 100% BDP  (full)")

if __name__=="__main__":main()
