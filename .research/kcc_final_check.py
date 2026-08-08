#!/usr/bin/env python3
"""
Final KCC 2.0: cwnd=max(1.0, pg), MD=1/1, drain=128 rounds, PG_MIN=0.75/PG_MAX=1.25
Compare: throughput, queue, fairness vs BBR.
"""
import sys

AI=0.02; MD_NUM,MD_DEN=1,1
PG_MIN=0.75; PG_MAX=1.25
DRAIN_PERIOD=128  # was 64, doubled for less throughput tax

class Flow:
    __slots__=('n','pg','tp','bbr')
    def __init__(s,n,bbr=False):
        s.n=n;s.pg=1.;s.tp=0;s.bbr=bbr
    def cg(s):
        if s.bbr: return 2.0               # BBR cwnd_gain=2
        return max(1.0, s.pg)              # KCC: floor=1.0x
    def advance_bbr(fl,rnd):
        cyc=[1.25,0.75,1.0,1.0,1.0,1.0,1.0,1.0]
        for f in fl:
            if f.bbr: f.pg=cyc[rnd&7]
    advance_bbr=staticmethod(advance_bbr)

def step(fl,C,T,rnd):
    N=len(fl);b=C*T/N
    tc=sum(f.cg()*b for f in fl)
    Q=max(0.,tc/C-T)
    for f in fl:
        if f.bbr: continue
        if Q<T/128.: f.pg=min(f.pg+AI,PG_MAX)
        else: md=f.pg*Q*MD_NUM/(T*MD_DEN);f.pg=max(f.pg-md,PG_MIN)
        if(rnd&(DRAIN_PERIOD-1))==0:f.pg=0.75  # keep low pg for drain
    Flow.advance_bbr(fl,rnd)
    t2=sum(f.cg()*b for f in fl)
    Q2=max(0.,t2/C-T);rt=T+Q2
    for f in fl: f.tp=f.cg()*b/rt
    return Q2

def sim(N,C,T,R,nb=0,w=500):
    fl=[Flow("K%d"%i)for i in range(N-nb)]+[Flow("B%d"%i,True)for i in range(nb)]
    for r in range(w):step(fl,C,T,r)
    qs=[];pd={f.n:[]for f in fl};td={f.n:[]for f in fl}
    for r in range(R):
        Q=step(fl,C,T,r+w);qs.append(Q)
        for f in fl:pd[f.n].append(f.pg);td[f.n].append(f.tp)
    aq=sum(qs)*1e3/len(qs);pq=sorted(qs)[int(len(qs)*.95)]*1e3
    dets=[]
    for f in fl:
        ap=sum(pd[f.n])/len(pd[f.n]);at=sum(td[f.n])/len(td[f.n])/1e6
        dets.append((f.n,ap,at,f.bbr))
    return aq,pq,dets

def main():
    C=1.26e9;T=0.060;R=5000;N=4
    print("="*80)
    print("KCC 2.0 final: cwnd_gain=max(1.0,pg)  MD=1/1  drain=%dRTT  PG min=%.2f max=%.2f"%(
        DRAIN_PERIOD,PG_MIN,PG_MAX))
    print("C=%.2fGbps  T=%dms  BDP=%.1fMB  BBR cwnd_gain=2"%(
        C/1e9,T*1000,C*T/1e6))
    print("="*80)
    for nb in [0,1,2]:
        aq,pq,dets=sim(N,C,T,R,nb=nb)
        tt=sum(d[2]for d in dets)
        kccs=[d[2]for d in dets if not d[3]]
        bbrs=[d[2]for d in dets if d[3]]
        print("\n%2d KCC + %d BBR  |  Q avg=%.2fms  p95=%.2fms  tp=%.0fMbps  util=%.1f%%"%(
            N-nb,nb,aq,pq,tt,tt/(C/1e6)*100))
        for d in dets:
            isb="(BBR)"if d[3]else""
            print("  %-6s  pg=%.4f  tp=%.0f Mbps  %s"%(d[0],d[1],d[2],isb))
        if kccs and bbrs:
            print("  -> BBR/KCC = %.2fx %s"%(max(bbrs)/max(kccs),
                "(was 2.67x before)" if nb==1 else "(was 2.67x before)" if nb==2 else ""))

    print("\n--- Pure KCC efficiency ---")
    aq,pq,dets=sim(N,C,T,R,nb=0)
    tt=sum(d[2]for d in dets)
    print("  Throughput = %.0f/1260 = %.1f%% (was 1227/1260=97.3%% before)" % (tt,tt/(C/1e6)*100))

if __name__=="__main__":main()
