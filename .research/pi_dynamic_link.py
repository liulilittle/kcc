# Dynamic international link: wandering capacity, churn, variable RTT
# py -3 this.py
import random,time
from multiprocessing import Pool,cpu_count
MSS=1500; BW_NOM=1260.0; CWND_GAIN=2.0

def sim(params):
    mp,ks,seed=params
    rng=random.Random(seed);C=BW_NOM
    # Dynamic: flows come and go, capacity wanders
    flows={}
    next_id=0;add_timer=0;sent={};qavg={};qbase={};cwnd={};cyc={};Tps={}
    prev_cwnd={};queue_s=0.0;total_thru=0.0;loss=0;steps=20000;warmup=4000

    for r in range(steps):
        dt=0.001  # 1ms timestep
        # Capacity wander (international link: BGP reroutes, congestion)
        if r%2000==0:C=BW_NOM+rng.uniform(-100,120)
        if r%5000==0:C=max(800,C*rng.uniform(0.85,1.15))  # occasional big shifts
        C=max(800,min(1500,C))

        # Flow churn: add flow every ~500ms, remove random flow occasionally
        if r%500==0:
            fid=next_id;next_id+=1
            flows[fid]=len(flows)
            sent[fid]=0.0;qavg[fid]=0.0;qbase[fid]=0.0;cwnd[fid]=4.0
            cyc[fid]=rng.randint(0,7)
            Tps[fid]=rng.uniform(0.025,0.065)
            prev_cwnd[fid]=4.0
        if r%3000==0 and len(flows)>4:
            fid=rng.choice(list(flows.keys()))
            del flows[fid],sent[fid],qavg[fid],qbase[fid],cwnd[fid],cyc[fid],Tps[fid],prev_cwnd[fid]

        N=len(flows)
        if N<3:continue
        # T_prop wander
        if r%200==0:
            for f in flows:
                Tps[f]+=rng.gauss(0,0.0005)
                Tps[f]=max(0.025,min(0.065,Tps[f]))

        # Per-flow update
        avgTp=sum(Tps.values())/N
        C_segs=(C*1e6/8/MSS)*avgTp
        prev_inflight=sum(prev_cwnd.values())
        q_segs=max(0,prev_inflight-C_segs)
        q_us=q_segs*MSS*8/(C*1e6)*1e6
        fair=C_segs/N

        for f in flows:
            Tp=Tps[f];rtt=Tp+q_us/1e6+rng.gauss(0,0.0002)
            if rtt<=1e-6:rtt=1e-6
            mrt=Tp  # perfect T_prop knowledge (geodesic)
            qi=max(0,rtt-mrt)+0.0005  # 500us measurement noise floor
            qavg[f]=qavg[f]*0.875+qi*0.125
            if qbase[f]<1e-9 or qi<qbase[f]*0.9:qbase[f]=qi*0.5+qbase[f]*0.5
            else:qbase[f]=max(qbase[f]*0.9999,qi*0.0001)

            # PI
            pg=1.0;cyc[f]=(cyc[f]+1)%8
            if qbase[f]>1e-9:
                qp=max(0,qavg[f]-qbase[f]);margin=Tp*mp
                e=qp-margin;kp=ks/max(1e-6,Tp)
                adj=kp*e;adj=max(-0.25,min(0.25,adj))
                if e<0:adj*=2
                if qp>margin*8:adj=-0.25
                pg=1.0+adj;pg=max(0.95,min(1.25,pg))

            # PI overrides baseline suppression by pushing pg above 1.0
            target=(C*1e6/8/MSS)*Tp*CWND_GAIN/N*pg
            target=max(4,target)
            if cwnd[f]<target:cwnd[f]=min(cwnd[f]+max(1,(target-cwnd[f])*0.3),target)
            else:cwnd[f]=max(target,cwnd[f]*0.95)

            if r>=warmup:sent[f]+=min(cwnd[f],fair)*MSS*8/1e6

        prev_cwnd={f:cwnd[f] for f in flows}
        if q_segs>C_segs*2:
            loss+=1
            for f in flows:cwnd[f]=max(1,cwnd[f]*0.7)

        if not sent:return 0,0,0
        nf=len(sent)
        tf=(steps-warmup)*0.001
    thru=sum(sent.values())/nf/tf
    return thru,thru*nf/BW_NOM*100,loss

if __name__=='__main__':
    print("DYNAMIC LINK — churn, wander, capacity shifts")
    print("="*50)
    jobs=[]
    for mp in [0.002,0.005,0.01,0.02]:
        for ks in [0.5,1.0,2.0,5.0]:
            for s in range(5):jobs.append((mp,ks,s))
    for s in range(5):jobs.append((0.01,0.0,s+100))
    print(f"Jobs: {len(jobs)} on {min(16,cpu_count())} workers")
    t0=time.time()
    with Pool(min(16,cpu_count())) as p:results=p.map(sim,jobs)
    print(f"Done: {time.time()-t0:.0f}s\n")
    base=[r for r in results[-5:]]
    bu=sum(b[0]for b in base if b[0]>0)/max(1,sum(1 for b in base if b[0]>0))
    print(f"BASELINE: {bu:.0f}Mbps/flow\n")
    print(f"{'m%':>5} {'Ks':>5} {'Thru':>7} {'Util':>6} {'Loss':>5} {'Gain':>6}")
    for mp in [0.002,0.005,0.01,0.02]:
        for ks in [0.5,1.0,2.0,5.0]:
            rs=[r for j,r in zip(jobs,results) if abs(j[0]-mp)<1e-9 and abs(j[1]-ks)<1e-9]
            if not rs:continue
            t=sum(r[0]for r in rs)/len(rs)
            u=sum(r[1]for r in rs)/len(rs) if rs else 0
            l=sum(r[2]for r in rs)/len(rs)
            g=(t-bu)/bu*100 if bu>0 else 0
            print(f"{mp*100:>4.1f}% {ks:>5.1f} {t:>7.0f} {u:>6.1f}% {l:>5.0f} {g:>+5.1f}%")
    print("DONE")
