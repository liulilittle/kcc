# KCC PI validated on working bottleneck model
# py -3 this.py
import random,sys,time
from multiprocessing import Pool,cpu_count
MSS=1500; BW=1260.0; N=6; Tp=0.045

def sim(params):
    mp,ks,seed=params
    rng=random.Random(seed);C=BW
    cwnd=[4.0]*N;qavg=[0.0]*N;qbase=[0.0]*N;sent=[0.0]*N
    cyc=[rng.randint(0,7) for _ in range(N)];loss=0
    for r in range(10000):
        if r%500==0:C=BW+rng.uniform(-30,60)
        C_segs=(C*1e6/8/MSS)*Tp
        inflight=sum(cwnd)
        q_segs=max(0,inflight-C_segs)
        q_us=q_segs*MSS*8/(C*1e6)*1e6
        for i in range(N):
            qi=max(0,Tp+q_us/1e6-Tp)  # simplified qdelay = queue/RTT proportion
            qi=q_us/1e6  # queue delay in seconds
            qavg[i]=qavg[i]*0.875+qi*0.125
            if qbase[i]<1e-9 or qi<qbase[i]*0.9:qbase[i]=qi*0.5+qbase[i]*0.5
            else:qbase[i]=max(qbase[i]*0.9999,qi*0.0001)
            # PI controller
            pg=1.0;cyc[i]=(cyc[i]+1)%8
            if qbase[i]>1e-9:
                qp=max(0,qavg[i]-qbase[i])
                margin=Tp*mp;e=qp-margin;kp=ks/max(1e-6,Tp)
                adj=kp*e;adj=max(-0.25,min(0.25,adj))
                if e<0:adj*=2
                if qp>margin*8:adj=-0.25
                pg=1.0+adj;pg=max(0.95,min(1.25,pg))
            target=(C*1e6/8/MSS)*Tp*2/N*pg
            target=max(4,target)
            if cwnd[i]<target:cwnd[i]=min(cwnd[i]+max(1,(target-cwnd[i])*0.3),target)
            else:cwnd[i]=max(target,cwnd[i]*0.95)
            fair=C_segs/N
            if r>=2000:sent[i]+=min(cwnd[i],fair)*MSS*8/1e6
        if q_segs>C_segs*2:loss+=1;cwnd=[max(1,c*0.7) for c in cwnd]
    thru=sum(sent)/N/(8000*Tp)
    return thru,thru/BW*100,loss,qavg,qbase

if __name__=='__main__':
    print("KCC PI — Working Bottleneck Model")
    print("="*45)
    jobs=[]
    for mp in [0.002,0.005,0.01,0.02,0.05]:
        for ks in [0.5,1.0,2.0,5.0]:
            for s in range(5):jobs.append((mp,ks,1000+s))
    for s in range(5):jobs.append((0.01,0.0,2000+s))
    print(f"Jobs: {len(jobs)} on {min(16,cpu_count())} workers")
    t0=time.time()
    with Pool(min(16,cpu_count())) as p:results=p.map(sim,jobs)
    print(f"Done: {time.time()-t0:.0f}s\n")
    base=[r for r in results[-5:]];bu=sum(b[0] for b in base)/5
    print(f"BASELINE(no PI): {bu:.0f}Mbps/flow ({bu*N:.0f} total, {bu*N/BW*100:.0f}%)\n")
    print(f"{'m%':>5} {'Ks':>5} {'Thru':>7} {'Total':>7} {'Util':>6} {'Loss':>5}")
    for mp in [0.002,0.005,0.01,0.02,0.05]:
        for ks in [0.5,1.0,2.0,5.0]:
            rs=[r for j,r in zip(jobs,results) if abs(j[0]-mp)<1e-9 and abs(j[1]-ks)<1e-9]
            if not rs:continue
            t=sum(r[0]for r in rs)/len(rs);l=sum(r[2]for r in rs)/len(rs)
            g=(t*N-bu*N)/BW*100
            print(f"{mp*100:>4.1f}% {ks:>5.1f} {t:>7.0f} {t*N:>7.0f} {t*N/BW*100:>6.1f}% {l:>5.0f} {'***'if g>5 else'++'if g>2 else'+'if g>0 else''}")
    print("DONE")
