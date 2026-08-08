#!/usr/bin/env python3
""" Sweep cwnd boost % (kcc_cwnd_floor_pct) for KCC/BBR fairness """
import sys

AI=0.02; MD_NUM,MD_DEN=1,1; PG_MIN=0.75; PG_MAX=1.25; DR=128

class Flow:
    __slots__=('n','pg','tp','bbr','bst')
    def __init__(s,n,bbr=False,bst=100):s.n=n;s.pg=1.;s.tp=0;s.bbr=bbr;s.bst=bst
    def cg(s):
        if s.bbr: return 2.0
        return max(1.0,s.pg)*s.bst/100.

def upd(fl,C,T,rnd):
    N=len(fl);b=C*T/N
    tc=sum(f.cg()*b for f in fl);Q=max(0.,tc/C-T)
    for f in fl:
        if f.bbr: continue
        if Q<T/128:f.pg=min(f.pg+AI,PG_MAX)
        else:md=f.pg*Q*MD_NUM/(T*MD_DEN);f.pg=max(f.pg-md,PG_MIN)
        if(rnd&(DR-1))==0:f.pg=0.75
    cyc=[1.25,0.75,1,1,1,1,1,1]
    for f in fl:
        if f.bbr:f.pg=cyc[rnd&7]
    t2=sum(f.cg()*b for f in fl);Q2=max(0.,t2/C-T);rt=T+Q2
    for f in fl:f.tp=f.cg()*b/rt
    return Q2

def run(fl,C,T,R,w=300):
    for r in range(w):upd(fl,C,T,r)
    qs=[];ts={f.n:[]for f in fl}
    for r in range(R):
        Q=upd(fl,C,T,r+w);qs.append(Q)
        for f in fl:ts[f.n].append(f.tp)
    aq=sum(qs)/len(qs)
    tps=[sum(ts[f.n])/len(ts[f.n])/1e6 for f in fl]
    return aq*1e3,tps

def main():
    C=1.26e9;T=0.060;R=2000
    print("PURE KCC (N=4):")
    print("%-5s %8s %8s %8s %8s" % ("bst%","Q_avg","Q_p95","Util%","KCC_tp"))
    for bst in [100,150,175,180,185,200]:
        fl=[Flow("K%d"%i,bst=bst)for i in range(4)]
        aq,tps=run(fl,C,T,R)
        pq=sorted([tps[0]]*10)[0];tt=sum(tps)
        print("%4d  %8.2f %8.2f %7.0f%% %8.0f"%(bst,aq,aq,tt/(C/1e9*1000)*100,tps[0]))

    print("\nMIXED 3KCC+1BBR:")
    print("%-5s %8s %8s %8s %8s %8s %s" % ("bst%","Q_avg","Util%","KCC_tp","BBR_tp","BBR/KCC","check>0.9"))
    for bst in [100,160,170,175,178,180,185,190,200]:
        fl=[Flow("K%d"%i,bst=bst)for i in range(3)]+[Flow("B0",bbr=True)]
        aq,tps=run(fl,C,T,R)
        ktp=tps[0];btp=tps[3];tt=sum(tps);r=btp/ktp if ktp>0 else 99
        print("%4d  %8.2f %7.0f%% %8.0f %8.0f %6.2f %s"%(bst,aq,tt/(C/1e9*1000)*100,ktp,btp,r,"YES" if r>=0.9 else "NO"))
    print("\nTarget: BBR/KCC <= 1.0/0.9 = 1.11 (reverse: KCC/BBR >= 0.9)")
    print("        BBR/KCC=%.2f means KCC gets %.0f%% of BBR share"%(
        r,ktp/btp*100 if btp>0 else 0))

if __name__=="__main__":main()
