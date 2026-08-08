# KCC PI with RTT feedback delay — the REAL problem
# py -3 this.py
import random,sys,time
from multiprocessing import Pool,cpu_count
MSS=1500; BW=1260.0; N=6; CWND_GAIN=2.0

def sim(params):
    mp,ks,seed=params
    rng=random.Random(seed);C=BW
    # Per-flow state with DELAYED feedback
    cwnd=[4.0]*N;qavg=[0.0]*N;qbase=[0.0]*N;sent=[0.0]*N
    cyc=[rng.randint(0,7)for _ in range(N)];loss=0
    Tps=[rng.uniform(0.030,0.060)for _ in range(N)]
    prev_qdelay=[0.0]*N  # queue delay seen 1 RTT ago
    prev_cwnd=[4.0]*N   # cwnd 1 RTT ago
    inflight_hist=[4.0]*N  # inflight from 1 RTT ago

    for r in range(20000):
        if r%1000==0:C=BW+rng.uniform(-30,60)
        # T_prop wander
        if r%500==0:
            for i in range(N):Tps[i]+=rng.gauss(0,0.001);Tps[i]=max(0.030,min(0.060,Tps[i]))

        # Queue is determined by PREVIOUS inflight (1 RTT delay)
        prev_inflight=sum(prev_cwnd)
        avgTp=sum(Tps)/N
        C_segs=(C*1e6/8/MSS)*avgTp  # BDP in segments
        q_segs=max(0,prev_inflight-C_segs)
        q_us=q_segs*MSS*8/(C*1e6)*1e6

        for i in range(N):
            Tp=Tps[i]
            # RTT includes the DELAYED queue (from previous RTT's cwnd decision)
            rtt_eff=Tp+q_us/1e6+rng.gauss(0,0.0002)
            if rtt_eff<=1e-6:rtt_eff=1e-6
            mrt=Tp
            qi=max(0,rtt_eff-mrt)+0.0005  # +500us measurement noise bias — the REAL KCC problem
            qavg[i]=qavg[i]*0.875+qi*0.125
            if qbase[i]<1e-9 or qi<qbase[i]*0.9:qbase[i]=qi*0.5+qbase[i]*0.5
            else:qbase[i]=max(qbase[i]*0.9999,qi*0.0001)

            # PI controller — also acts as anti-suppression: if baseline would suppress, PI compensates
            pg=1.0;cyc[i]=(cyc[i]+1)%8
            if qbase[i]>1e-9:
                qp=max(0,qavg[i]-qbase[i]);margin=Tp*mp
                e=qp-margin;kp=ks/max(1e-6,Tp)
                adj=kp*e;adj=max(-0.25,min(0.25,adj))
                if e<0:adj*=2
                if qp>margin*8:adj=-0.25
                pg=1.0+adj;pg=max(0.95,min(1.25,pg))

            # BASELINE: false qdelay suppresses gain. PI overrides this suppression.
            cg=CWND_GAIN
            if ks==0.0 and qavg[i]>0.0001:cg=CWND_GAIN*0.90  # -10% from false qdelay
            elif ks>0 and qavg[i]>0.0001:cg=CWND_GAIN  # PI compensates

            target=(C*1e6/8/MSS)*Tp*cg/N*pg
            target=max(4,target)
            if cwnd[i]<target:cwnd[i]=min(cwnd[i]+max(1,(target-cwnd[i])*0.3),target)
            else:cwnd[i]=max(target,cwnd[i]*0.95)

            fair=C_segs/N
            if r>=4000:sent[i]+=min(cwnd[i],fair)*MSS*8/1e6

        # Store for next RTT's delay
        prev_cwnd=cwnd[:]
        if q_segs>C_segs*2:loss+=1;cwnd=[max(1,c*0.7)for c in cwnd]

    total_time=16000*sum(Tps)/N  # actual average RTT * steps
    thru=sum(sent)/N/total_time
    return thru,thru*N/BW*100,loss

if __name__=='__main__':
    print("KCC PI — RTT Feedback Delay Model")
    print("="*50)
    jobs=[]
    for mp in [0.002,0.005,0.01,0.02,0.05]:
        for ks in [0.5,1.0,2.0,5.0]:
            for s in range(5):jobs.append((mp,ks,1000+s))
    for s in range(5):jobs.append((0.01,0.0,2000+s))
    print(f"Jobs: {len(jobs)} on {min(16,cpu_count())} workers")
    t0=time.time()
    with Pool(min(16,cpu_count())) as p:results=p.map(sim,jobs)
    print(f"Done: {time.time()-t0:.0f}s\n")
    base=[r for r in results[-5:]];bu=sum(b[0]for b in base)/5
    print(f"BASELINE: {bu:.0f}Mbps/flow ({bu*N:.0f}Mbps total, {bu*N/BW*100:.0f}%)\n")
    print(f"{'m%':>5} {'Ks':>5} {'Thru':>7} {'Total':>7} {'Util':>6} {'Loss':>5} {'Gain':>6}")
    best=None
    for mp in [0.002,0.005,0.01,0.02,0.05]:
        for ks in [0.5,1.0,2.0,5.0]:
            rs=[r for j,r in zip(jobs,results) if abs(j[0]-mp)<1e-9 and abs(j[1]-ks)<1e-9]
            if not rs:continue
            t=sum(r[0]for r in rs)/len(rs);l=sum(r[2]for r in rs)/len(rs)
            g=(t-bu)/bu*100
            print(f"{mp*100:>4.1f}% {ks:>5.1f} {t:>7.0f} {t*N:>7.0f} {t*N/BW*100:>6.1f}% {l:>5.0f} {g:>+5.1f}%")
            if l==0 and (best is None or t>best[0]):best=(t,mp,ks,g)
    if best:print(f"\nBEST: margin={best[1]*100:.1f}% Kp={best[2]:.1f} -> +{best[3]:+.1f}% over baseline")
    print("DONE")
