#!/usr/bin/env python3
""" Sweep AI rate 0.1%-10% for eco and turbo modes. Find max safe queue pressure. """
import sys

MD_NUM,MD_DEN=1,1; PG_MIN=0.75; PG_MAX=1.25; DR=128

class Flow:
    __slots__=('pg','tp','bbr','turbo','ai')
    def __init__(s,pg=1.,bbr=False,turbo=False,ai=0.02):
        s.pg=pg;s.tp=0;s.bbr=bbr;s.turbo=turbo;s.ai=ai
    def cg(s):
        if s.bbr: return 2.0
        floor=1.85 if s.turbo else 1.0
        return max(floor,s.pg)

def upd(fl,C,T,rnd,ai):
    N=len(fl);b=C*T/N
    tc=sum(f.cg()*b for f in fl);Q=max(0.,tc/C-T)
    for f in fl:
        if f.bbr: continue
        if Q<T/128:f.pg=min(f.pg+ai,PG_MAX)
        else:md=f.pg*Q*MD_NUM/(T*MD_DEN);f.pg=max(f.pg-md,PG_MIN)
        if(rnd&(DR-1))==0:f.pg=0.75
    cyc=[1.25,0.75,1,1,1,1,1,1]
    for f in fl:
        if f.bbr:f.pg=cyc[rnd&7]
    t2=sum(f.cg()*b for f in fl);Q2=max(0.,t2/C-T);rt=T+Q2
    for f in fl:f.tp=f.cg()*b/rt
    return Q2

def sim(N,C,T,R,turbo,ai,w=300):
    fl=[Flow(turbo=turbo,ai=ai)for _ in range(N)]
    for r in range(w):upd(fl,C,T,r,ai)
    qs=[];tps=[[] for _ in fl];pgs=[[] for _ in fl]
    for r in range(R):
        Q=upd(fl,C,T,r+w,ai);qs.append(Q)
        for i,f in enumerate(fl):tps[i].append(f.tp);pgs[i].append(f.pg)
    aq=sum(qs)/len(qs)*1e3
    p95=sorted(qs)[int(len(qs)*.95)]*1e3
    p99=sorted(qs)[int(len(qs)*.99)]*1e3
    pgt=sum(sum(p)/len(p)for p in pgs)/N
    tpt=sum(sum(t)/len(t)for t in tps)/1e6
    return aq,p95,p99,pgt,tpt

def main():
    C=1.26e9;T=0.060;N=4;R=1500
    print("ECO mode (cwnd floor=1.0x):")
    print("%-6s %8s %8s %8s %8s %6s %6s" % ("AI%","Q_avg","Q_p95","Q_p99","pg_avg","Util%","Tp_f"))
    for ai_pct in [0.1,0.2,0.5,1.0,1.5,2.0,3.0,4.0,5.0,7.0,10.0]:
        ai=ai_pct/100.0
        aq,p95,p99,pg,tp=sim(N,C,T,R,False,ai)
        util=tp/(C/1e6)*100
        mark="<-- " if ai_pct==2.0 else ""
        print("%5.1f%% %8.2f %8.2f %8.2f %8.4f %5.0f%% %6.0f %s"%(ai_pct,aq,p95,p99,pg,util,tp,mark))

    print("\nTURBO mode (cwnd floor=1.85x):")
    print("%-6s %8s %8s %8s %8s %6s %6s" % ("AI%","Q_avg","Q_p95","Q_p99","pg_avg","Util%","Tp_f"))
    for ai_pct in [0.1,0.2,0.5,1.0,1.5,2.0,3.0,4.0,5.0,7.0,10.0]:
        ai=ai_pct/100.0
        aq,p95,p99,pg,tp=sim(N,C,T,R,True,ai)
        util=tp/(C/1e6)*100
        mark="<-- " if ai_pct==2.0 else ""
        print("%5.1f%% %8.2f %8.2f %8.2f %8.4f %5.0f%% %6.0f %s"%(ai_pct,aq,p95,p99,pg,util,tp,mark))

    print("\nTheory: Q_eq = (pg_eq-1)*T  where pg*(pg-1) = AI_coeff")
    print("AI=2%% -> pg=1.02, Q=1.2ms  |  AI=5%% -> pg=1.05, Q=2.9ms  |  AI=10%% -> pg=1.10, Q=5.7ms")
    print("Turbo floor dominates: Q = (1.85-1)*60 = 51ms regardless of AI.")

if __name__=="__main__":main()
