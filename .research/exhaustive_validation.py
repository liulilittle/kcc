#!/usr/bin/env python
""" KCC 2.0 Exhaustive Validation: 180 scenarios across C,T,AI,mode,competition """
import sys,time

DR=128;R=1200;W=200

class Flow:
    __slots__=('pg','f','ai','tp','bbr')
    def __init__(s,pg=1.,f=1.0,ai=0.03125,bbr=False):
        s.pg=pg;s.f=f;s.ai=ai;s.tp=0;s.bbr=bbr
    def cg(s):
        if s.bbr:return 2.0
        return max(s.f,s.pg)

def upd(fl,C,T,R):
    N=len(fl);b=C*T/N
    tc=sum(f.cg()*b for f in fl);Q=max(0.,tc/C-T)
    for f in fl:
        if f.bbr:continue
        if Q<T/128:f.pg=min(f.pg+f.ai,1.25)
        else:md=f.pg*Q/T;f.pg=max(f.pg-md,0.75)
        if(R&127)==0:f.pg=0.75
    cyc=[1.25,0.75,1,1,1,1,1,1]
    for f in fl:
        if f.bbr:f.pg=cyc[R&7]
    t2=sum(f.cg()*b for f in fl);Q2=max(0.,t2/C-T);rt=T+Q2
    for f in fl:f.tp=f.cg()*b/rt
    return Q2

def sim(N,nb,C,T,floor,ai,R=1200,W=200):
    fl=[Flow(f=floor,ai=ai,bbr=i>=N-nb)for i in range(N)]
    for r in range(W):upd(fl,C,T,r)
    qs=[];tps=[[]for _ in fl]
    for r in range(R):
        Q=upd(fl,C,T,r+W);qs.append(Q)
        for i,f in enumerate(fl):tps[i].append(f.tp)
    aq=sum(qs)/len(qs)*1e3;p95=sorted(qs)[int(len(qs)*.95)]*1e3
    p99=sorted(qs)[int(len(qs)*.99)]*1e3
    ktps=[sum(tps[i])/len(tps[i])/1e6 for i in range(N-nb)]
    btps=[sum(tps[i+N-nb])/len(tps[i+N-nb])/1e6 for i in range(nb)]if nb>0 else[]
    avg_k=sum(ktps)/len(ktps)if ktps else 0
    avg_b=sum(btps)/len(btps)if btps else 0
    tt=sum(sum(t)/len(t)for t in tps)/1e6
    return aq,p95,p99,avg_k,avg_b,tt

def main():
    Cs=[1e9,1.26e9];Ts=[0.01,0.03,0.06,0.12,0.25]
    ai_nums=[16,25,40];ai_labels=['2%','3.125%','5%']
    turbos=[(False,1.0,'ECO'),(True,1.88,'TURBO')]
    N=4

    t0=time.time()
    results=[]
    for C in Cs:
        for T in Ts:
            for turb,floor,mode in turbos:
                for nb in [0,1,2]:
                    if N-nb<=0:continue
                    for ai_num,ai_lbl in zip(ai_nums,ai_labels):
                        ai=ai_num/800.
                        aq,pq,p99,kp,bp,tt=sim(N,nb,C,T,floor,ai)
                        util=tt/(C/1e6)*100
                        fairness=min(kp,bp)/max(kp,bp)if kp>0 and bp>0 else 1.0
                        results.append((C,T,mode,ai_lbl,nb,aq,pq,p99,kp,bp,tt,util,fairness))

    elapsed=time.time()-t0
    print('KCC 2.0 Exhaustive Validation: %d scenarios in %.0fs'%(len(results),elapsed))
    print('='*90)

    # Summary by T_prop (user's real network T=60ms)
    print('\n--- T=60ms (your hardware link) ---')
    print('%-5s %-6s %3s %7s %7s %7s %6s %6s %6s %5s %s'%('C','mode','AI','Q_avg','Q_p95','Q_p99','KCCtp','BBRtp','Util%','Fair',''))
    for r in results:
        C,T,mode,ai_lbl,nb,aq,pq,p99,kp,bp,tt,util,fair=r
        if abs(T-0.06)>1e-5 or abs(C-1e9)>1e5:continue
        bbr_str='%6.0f'%bp if nb>0 else '   ---'
        fair_str='%.3f'%fair if nb>0 else '  ---'
        print('%4.0fM %-6s %3s %7.2f %7.2f %7.2f %6.0f %s %5.0f%% %s %sbbr=%d'%(
            C/1e6,mode,ai_lbl,aq,pq,p99,kp,bbr_str,util,fair_str,'+' if nb>0 else ' ',nb))

    # Cross-T_prop check: verify Q scales linearly 
    print('\n--- AI=3.125% ECO, pure KCC, Q vs T_prop ---')
    print('%-6s %7s %7s %7s'%('T_ms','Q_avg','Q_pred','ratio'))
    for T in Ts:
        for r in results:
            C_,T_,mode,ai_lbl,nb,aq,pq,p99,kp,bp,tt,util,fair=r
            if abs(T_-T)<1e-5 and mode=='ECO' and ai_lbl=='3.125%' and nb==0:
                q_pred=(1.0292-1)*T*1e3
                print('%5d  %7.2f %7.2f  %.2f'%(T*1e3,aq,q_pred,aq/q_pred if q_pred>0 else 0))

    # TURBO Q vs T_prop
    print('\n--- AI=3.125% TURBO, pure KCC, Q vs T_prop ---')
    print('%-6s %7s %7s %7s'%('T_ms','Q_avg','Q_pred','ratio'))
    for T in Ts:
        for r in results:
            C_,T_,mode,ai_lbl,nb,aq,pq,p99,kp,bp,tt,util,fair=r
            if abs(T_-T)<1e-5 and mode=='TURBO' and ai_lbl=='3.125%' and nb==0:
                q_pred=(1.88-1)*T*1e3
                print('%5d  %7.2f %7.2f  %.2f'%(T*1e3,aq,q_pred,aq/q_pred if q_pred>0 else 0))

    # Fairness summary
    print('\n--- Fairness (KCC/BBR) at T=60ms, C=1000M ---')
    for r in results:
        C,T,mode,ai_lbl,nb,aq,pq,p99,kp,bp,tt,util,fair=r
        if abs(T-0.06)>1e-5 or abs(C-1e9)>1e5 or nb==0:continue
        print('  %s AI=%s bbr=%d: K/B=%.3f'%(mode,ai_lbl,nb,fair))

    print('\n--- Anomaly check ---')
    anomalies=0
    for r in results:
        C,T,mode,ai_lbl,nb,aq,pq,p99,kp,bp,tt,util,fair=r
        if util<95:print('LOW_UTIL:',r);anomalies+=1
        if fair<0.5 and nb>0:print('LOW_FAIR:',r);anomalies+=1
        if aq>500:print('HIGH_Q:',r);anomalies+=1
    print('Anomalies: %d (expect 0). Total scenarios: %d'%(anomalies,len(results)))

if __name__=='__main__':main()
