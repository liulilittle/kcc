# KCC 2.0 full simulation — proper RTT-level model, aggressive PI
# py -3 this_file.py
import random,sys,time
from multiprocessing import Pool,cpu_count

MSS=1500; BW=1260.0; N_FLOWS=6; RTT_RANGE=(0.030,0.060)
JITTER=0.0002; CWND_GAIN=2.0; STEPS=20000; WARMUP=5000
CYCLE=[1.25,0.75,1.0,1.0,1.0,1.0,1.0,1.0]
PI_LO,PI_HI=0.75,1.25  # same as DRAIN-PROBE_UP

def sim_one(args):
    margin_pct,kp_scale,seed=args
    rng=random.Random(seed)
    # per-flow state (indexed by flow)
    tp0=[rng.uniform(*RTT_RANGE) for _ in range(N_FLOWS)]
    tp=tp0[:]; mr=tp[:]; x=[t for t in tp]; cnf=[0]*N_FLOWS
    csl=[0]*N_FLOWS; pd=[0]*N_FLOWS; cwnd=[4.0]*N_FLOWS
    cycle=[rng.randint(0,7) for _ in range(N_FLOWS)]
    qavg=[0.0]*N_FLOWS; qbase=[0.0]*N_FLOWS; total=[0.0]*N_FLOWS
    queue_s=0.0; loss=0; pgains=[1.0]*N_FLOWS; C=BW

    for step in range(STEPS):
        # capacity wander
        if step%2000==0: C=BW+rng.uniform(-30,60)
        # T_prop wander
        if step%1000==0:
            for i in range(N_FLOWS):
                tp[i]+=rng.gauss(0,0.001);tp[i]=max(RTT_RANGE[0],min(RTT_RANGE[1],tp[i]))

        # Each flow does one RTT update
        inflight_bytes=0.0
        for i in range(N_FLOWS):
            rtt=tp[i]+queue_s+rng.gauss(0,JITTER)
            if rtt<=0:rtt=1e-6
            # geodesic
            xv=x[i]; mv=mr[i]
            if rtt<=xv:xv=rtt
            else:xv=min(xv*1.12,rtt)
            x[i]=xv
            ft=mv*1.10;st=mv*1.05
            if xv>=ft:cnf[i]+=1;csl[i]+=1
            elif xv>=st:cnf[i]=0;csl[i]+=1
            else:cnf[i]=0
            if xv<=mv:cnf[i]=0;csl[i]=0
            if cnf[i]>=3:mr[i]=xv;cnf[i]=csl[i]=0
            elif csl[i]>=4:mr[i]=xv;cnf[i]=csl[i]=0
            if cnf[i]==0 and csl[i]==0:
                if xv<mv*0.95:pd[i]+=1
                else:pd[i]=0
                if pd[i]>=3:mr[i]=xv;pd[i]=0
            # model_rtt
            mrt=min(xv,mv)
            qi=max(0,rtt-mrt)
            qavg[i]=qavg[i]*0.875+qi*0.125
            # qdelay_base
            if qbase[i]<1e-9 or qi<qbase[i]*0.9:
                qbase[i]=qi*0.5+qbase[i]*0.5
            else:
                qbase[i]=max(qbase[i]*0.9999,qi*0.0001)
            # pacing_gain: cycle + PI
            pg=1.0; cycle[i]=(cycle[i]+1)%8
            pg_raw=CYCLE[cycle[i]&7]
            # Aggressive PI: replaces fixed 1.0 cruise, operates at any gain >= 1.0
            if pg_raw>=1.0 and qbase[i]>1e-9:
                qp=max(0,qavg[i]-qbase[i])
                margin=mrt*margin_pct  # adaptive margin
                e=qp-margin
                kp=kp_scale/max(1e-6,mrt)  # adaptive Kp
                adj=kp*e
                adj=max(-0.25,min(0.25,adj))  # +-25% full range
                if e<0:adj*=2  # 2x aggressive below margin
                if qp>margin*8:adj=-0.25  # hard brake
                pg=1.0+adj
                pg=max(PI_LO,min(PI_HI,pg))
            if pg_raw<1.0 and qi<mrt*0.05:pg=1.0  # drain-skip
            pgains[i]=pg
            # BDP-based cwnd
            bdp_bytes=C*1e6/8*mrt
            target=max(4,bdp_bytes/MSS*CWND_GAIN)
            if cwnd[i]<target:cwnd[i]=min(cwnd[i]+(target-cwnd[i])*0.3,target)
            else:cwnd[i]=max(target,cwnd[i]*0.95)
            inflight_bytes+=cwnd[i]*MSS

        # Bottleneck: serve at C, measure queue
        C_bytes_per_rtt=C*1e6/8*max(tp)
        served=min(inflight_bytes,C_bytes_per_rtt)
        queue_bytes=max(0,inflight_bytes-served)
        queue_s=queue_bytes/(C*1e6/8)
        if queue_s>max(tp)*4:loss+=1;queue_s*=0.3

        # Count throughput (only after warmup)
        if step>=WARMUP:
            fair_share=served/N_FLOWS
            for i in range(N_FLOWS):
                total[i]+=min(cwnd[i]*MSS,fair_share)

    thru=sum(total)/N_FLOWS/((STEPS-WARMUP)*max(tp))/1e6*8
    util=thru/BW*100; qp_avg=sum(qavg)/N_FLOWS; qb_avg=sum(qbase)/N_FLOWS
    return thru,util,loss,qp_avg*1e6,qb_avg*1e6

if __name__=='__main__':
    print("KCC AGGRESSIVE PI — RTT-level model")
    print("="*50)
    jobs=[]
    # Sweep margin_pct and kp_scale
    for mp in [0.002,0.005,0.01,0.02,0.05]:  # 0.2%-5% of T_prop
        for ks in [0.1,0.5,1.0,2.0,5.0]:      # Kp scale
            for s in range(5):
                jobs.append((mp,ks,1000+s))
    # Baseline (no PI: kp_scale=0)
    for s in range(5):
        jobs.append((0.01,0.0,2000+s))
    print(f"Jobs: {len(jobs)} on {min(16,cpu_count())} workers")
    t0=time.time()
    with Pool(min(16,cpu_count())) as p:
        results=p.map(sim_one,jobs)
    print(f"Done: {time.time()-t0:.0f}s\n")

    # Aggregate by (margin_pct, kp_scale)
    print(f"{'m%':>6} {'Kp':>5} {'Thru':>7} {'Util':>6} {'Loss':>5} {'Qavg':>6} {'Qbase':>6}")
    baselines=[r for r in results[125:] if r[0]>0]
    base_thru=sum(b[0] for b in baselines)/len(baselines)
    base_util=sum(b[1] for b in baselines)/len(baselines)
    print(f"{'base':>6} {'0':>5} {base_thru:>7.0f} {base_util:>6.1f}% {'-':>5} {'-':>6} {'-':>6}")

    best=None
    for mp in [0.002,0.005,0.01,0.02,0.05]:
        for ks in [0.1,0.5,1.0,2.0,5.0]:
            idx=[mp]*25+[ks]*5  # find results for this combo
            rs=[results[i] for i in range(len(jobs)) if abs(jobs[i][0]-mp)<1e-9 and abs(jobs[i][1]-ks)<1e-9]
            if not rs: continue
            avg_t=sum(r[0]for r in rs)/len(rs)
            avg_u=sum(r[1]for r in rs)/len(rs)
            avg_l=sum(r[2]for r in rs)/len(rs)
            gain=avg_u-base_util
            s="***" if gain>3 else("++" if gain>1 else("+" if gain>0.1 else " "))
            print(f"{mp*100:>5.1f}% {ks:>5.1f} {avg_t:>7.0f} {avg_u:>6.1f}% {avg_l:>5.0f} {s} {gain:+.1f}%")
            if avg_l==0 and (best is None or avg_t>best[0]):
                best=(avg_t,avg_u,mp,ks)
    if best:
        print(f"\nBEST: margin={best[2]*100:.1f}% RTT Kp_scale={best[3]:.1f} -> {best[0]:.0f}Mbps ({best[1]:.1f}%)")
        print(f"Gain: {best[1]-base_util:+.1f}% over baseline")
    print("DONE")
