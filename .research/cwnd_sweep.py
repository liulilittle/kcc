#!/usr/bin/env python3
"""
Sweep cwnd_gain functions vs BBR competition.
"""
import sys

AI_RATE=0.02; PG_MIN=0.75; PG_MAX=1.25; MD_NUM,MD_DEN=1,1; DRAIN_PERIOD=64

class Flow:
    __slots__=('name','pg','gf','tp','is_bbr','loss')
    def __init__(s, n, pg=1.0, gf="pg", is_bbr=False):
        s.name=n;s.pg=pg;s.gf=gf;s.tp=0;s.is_bbr=is_bbr;s.loss=0
    def cg(s):
        if s.is_bbr: return 2.0
        if s.gf=="1": return 1.0
        if s.gf=="pg": return s.pg
        if s.gf=="pg2": return s.pg*s.pg
        if s.gf=="1.0+0.75*(pg-1)": return 1.0+0.75*(s.pg-1.0)  # pg=1.25->1.19, pg=0.75->0.81
        return 1.0

def advance_bbr(fl,rnd):
    cyc=[1.25,0.75,1.0,1.0,1.0,1.0,1.0,1.0]
    for f in fl:
        if f.is_bbr: f.pg=cyc[rnd&7]

def upd(fl,C,T,rnd):
    N=len(fl);b=C*T/N;tc=sum(f.cg()*b for f in fl);Q=max(0.,tc/C-T)
    for f in fl:
        if f.is_bbr: continue
        if Q<T/128.: f.pg=min(f.pg+AI_RATE,PG_MAX)
        else: md=f.pg*Q*MD_NUM/(T*MD_DEN);f.pg=max(f.pg-md,PG_MIN)
        if(rnd&63)==0:f.pg=0.75
    advance_bbr(fl,rnd)
    t2=sum(f.cg()*b for f in fl);Q2=max(0.,t2/C-T);rt=T+Q2
    for f in fl: f.tp=f.cg()*b/rt
    return Q2

def sim(N,C,T,R,nb=0,gf="pg",w=500):
    fl=[Flow("K%d"%i,gf=gf)for i in range(N-nb)]+[Flow("B%d"%i,is_bbr=True)for i in range(nb)]
    for r in range(w):upd(fl,C,T,r)
    qs=[];pd={f.name:[]for f in fl};td=pd.copy();ld=pd.copy()
    for r in range(R):
        Q=upd(fl,C,T,r+w);qs.append(Q)
        for f in fl:pd[f.name].append(f.pg);td[f.name].append(f.tp);ld[f.name].append(f.loss)
    return qs,pd,td

def main():
    C=1.26e9;T=0.060;R=3000;N=4
    print("%-12s %7s %7s %6s %20s" % ("gf","Qavg","Qp95","TP","flow details"))
    print("-"*80)
    for gf in ["pg","pg2"]:
        for nb in [0,1,2]:
            q,p,t=sim(N,C,T,R,nb=nb,gf=gf)
            aq=sum(q)*1000/len(q);pq=sorted(q)[int(len(q)*.95)]*1000
            ttl=0.0;dets=[]
            for f in sorted(p.keys()):
                ap=sum(p[f])/len(p[f]);at=sum(t[f])/len(t[f])/1e6;ttl+=at
                dets.append("%s %.4f %.0f"%(f,ap,at))
            print("%-12s %7.2f %7.2f %5.0f  %s"%(gf+" %dB"%nb,aq,pq,ttl,", ".join(dets)))

    print("\nTheory: Q = (avg_cwnd_gain - 1) * T_prop")
    print("  BBR (cg=2): max Q=%.0fms  |  KCC pg (cg=1.25): Q=%.0fms  |  KCC pg2 (cg=1.56): Q=%.0fms"%(
        60.,15.,33.75))

if __name__=="__main__":main()
