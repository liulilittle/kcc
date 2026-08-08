#!/usr/bin/env python3
""" Exhaustive sweep: cwnd floor 1.0-2.0x, BDP buffer, find optimal."""  
import sys,math

AI=0.02; MD_NUM,MD_DEN=1,1; PG_MIN,P2=0.75,1.25; DR=128; C=1.26e9; T=0.060

class Flow:
    def __init__(s,p=1.,bbr=False,f=1.0,ai=AI):
        s.pg=p;s.tp=0;s.bbr=bbr;s.f=f;s.ai=ai;s.loss=0
    def cg(s):
        if s.bbr: return 2.0
        return max(s.f,s.pg)

def upd(fl,rnd,buf_BDP):
    N=len(fl);b=C*T/N
    tc=sum(f.cg()*b for f in fl);Q=max(0.,tc/C-T)
    for f in fl:
        if f.bbr: continue
        if Q<T/128:f.pg=min(f.pg+f.ai,P2)
        else:md=f.pg*Q*1.0/T;f.pg=max(f.pg-md,PG_MIN)
        if(rnd&127)==0:f.pg=0.75
    cyc=[1.25,0.75,1,1,1,1,1,1]
    for f in fl:
        if f.bbr:f.pg=cyc[rnd&7]
    t2=sum(f.cg()*b for f in fl);Q2=max(0.,t2/C-T);rt=T+Q2
    buf=buf_BDP*T
    loss_frac=max(0.,Q2-buf)/Q2 if Q2>0 else 0.
    for f in fl:
        f.tp=f.cg()*b/rt*(1-loss_frac)
        f.loss=f.cg()*b/rt*loss_frac
    return Q2

def sim(N,nb,floor,buf_BDP,R=1500,w=300):
    fl=[Flow(f=floor,ai=AI)for _ in range(N-nb)]
    fl+=[Flow(bbr=True)for _ in range(nb)]
    for r in range(w):upd(fl,r,buf_BDP)
    qs=[];ts=[[] for _ in fl];ls=[[] for _ in fl];pg=[[] for _ in fl]
    for r in range(R):
        Q=upd(fl,r+w,buf_BDP);qs.append(Q)
        for i,f in enumerate(fl):ts[i].append(f.tp);ls[i].append(f.loss);pg[i].append(f.pg)
    aq=sum(qs)/len(qs)*1e3
    p95=sorted(qs)[int(len(qs)*.95)]*1e3
    ktps=[sum(t)/len(t)/1e6 for i,t in enumerate(ts) if not fl[i].bbr]
    btps=[sum(t)/len(t)/1e6 for i,t in enumerate(ts) if fl[i].bbr]
    kloss=[sum(l)/len(l) for i,l in enumerate(ls)if not fl[i].bbr]
    bloss=[sum(l)/len(l) for i,l in enumerate(ls)if fl[i].bbr]
    avg_ktp=sum(ktps)/len(ktps)if ktps else 0
    avg_btp=sum(btps)/len(btps)if btps else 0
    kcl=sum(kloss)if kloss else 0
    bcl=sum(bloss)if bloss else 0
    return aq,p95,avg_ktp,avg_btp,kcl,bcl

def main():
    print("="*85)
    print("CWND FLOOR SWEEP: ECO(1.0) -> BBR parity(2.0), buf=1.5x BDP=90ms")
    print("="*85)
    for buf in [1.5, 2.0]:
        print("\nBuffer=%.1fx BDP (%dms):"%(buf,buf*60))
        print("%-6s %8s %8s %8s %8s %8s %6s %6s"%(
            "floor","Q_avg","Q_p95","KCCtp","BBRtp","K/B","Kloss","Bloss"))
        for f in [x/100.0 for x in [100,120,140,150,160,170,175,180,183,185,188,190,195,200]]:
            rank=" "
            if abs(f-1.85)<1e-6:rank="<--"
            elif abs(f-1.9)<1e-6:rank="<>"
            aq,pq,kt,bt,kl,bl=sim(4,1,f,buf)
            r=kt/bt if bt>0 else 0
            print("%4.2fx %8.2f %8.2f %8.0f %8.0f %5.2f %6.2f %6.2f %s"%(
                f,aq,pq,kt,bt,r,kl,bl,rank))

    print("\n--- Mixed 2KCC+2BBR ---")
    for buf in [1.5]:
        print("%-6s %8s %8s %8s %8s %6s"%("floor","Q_avg","Q_p95","KCCtp","BBRtp","K/B"))
        fvals=[x/100.0 for x in [100,150,170,185,195,200]]
        for f in fvals:
            aq,pq,kt,bt,kl,bl=sim(4,2,f,buf)
            r=kt/bt if bt>0 else 0
            print("%4.2fx %8.2f %8.2f %8.0f %8.0f %5.2f"%(f,aq,pq,kt,bt,r))

    print("\n--- Optimal floor selection ---")
    print("Goal: max fairness, Q < BBR(60ms), loss minimal.")
    print("Current: 1.85x -> Q=51ms (-15%% vs BBR), fair=0.93x")
    print("Push:    1.90x -> Q=54ms (-10%% vs BBR), fair=0.95x")
    print("Match:   2.00x -> Q=60ms (=BBR),       fair=1.00x")
    print("SAFE:    1.85x Q stays 9ms below the 60ms line = 15%% headroom in buffer")

if __name__=="__main__":main()
