# KCC 2.0 PI search — clean simulation model
# py -3 this_file.py
import random,sys,time
from multiprocessing import Pool,cpu_count

MSS=1500; BW=1260.0; N_FLOWS=6; DT=0.001; SEED_OFFSET=9999

def sim_one(args):
    margin,Kp,Ki,ff,seed=args
    rng=random.Random(seed)
    # Flow state
    tp=[rng.uniform(0.030,0.060) for _ in range(N_FLOWS)]
    mr, x_est = tp[:], tp[:]
    qavg=[0.0]*N_FLOWS; qbase=[0.0]*N_FLOWS; pi_int=[0.0]*N_FLOWS
    cwnd=[4.0]*N_FLOWS; cycle=[0]*N_FLOWS
    CYCLE=[1.25,0.75,1.0,1.0,1.0,1.0,1.0,1.0]
    total_served=[0.0]*N_FLOWS
    queue_bytes=0.0

    for step in range(120000):  # 120s total, 30s warmup
        # T_prop wander (slow drift)
        if step%5000==0:
            for i in range(N_FLOWS):
                tp[i]+=rng.gauss(0,0.001);tp[i]=max(0.030,min(0.060,tp[i]))
        # Capacity wander
        C_now=BW+rng.uniform(-30,60)

        # Drain queue
        drain=C_now*1e6/8*DT; queue_bytes=max(0,queue_bytes-drain)
        queue_s=queue_bytes/(C_now*1e6/8)

        total_sent=0.0
        for i in range(N_FLOWS):
            # RTT with noise
            rtt=tp[i]+queue_s+rng.gauss(0,0.0002)
            if rtt<=0:rtt=1e-6
            mrt=min(rtt,tp[i])  # simplified model_rtt
            qi=max(0,rtt-mrt)
            # EWMA qdelay
            qavg[i]=qavg[i]*0.875+qi*0.125
            # qdelay_base
            if qbase[i]<1e-9 or qi<qbase[i]*0.9:
                qbase[i]=qi*0.5+qbase[i]*0.5
            else:qbase[i]=max(qbase[i]*ff,qi*(1-ff))

            # Pacing gain: fixed cycle + PI on cruise
            pg=1.0; cycle[i]=(cycle[i]+1)%8
            pg_raw=CYCLE[cycle[i]&7]
            if abs(pg_raw-1.0)<0.01 and qbase[i]>1e-9:
                qp=max(0,qavg[i]-qbase[i]); e=qp-margin
                pi_int[i]+=e*Ki*tp[i]
                pi_int[i]=max(-0.001,min(0.001,pi_int[i]))
                if qp<=margin*3 and queue_s<tp[i]*0.15:
                    if abs(e)>20e-6:
                        a=Kp*e+pi_int[i];a=max(-0.05,min(0.05,a))
                        pg=1.0+a
                else:pi_int[i]=0.0
            if pg_raw<1.0 and qi<tp[i]*0.05:pg=1.0
            else:pg=pg_raw if pg_raw!=1.0 else pg
            pg=min(1.05,max(0.95,pg))

            # BDP-based cwnd
            bdp_segs=C_now*1e6/8*mrt/MSS
            target=bdp_segs*2.0*pg
            target=max(target,4)
            if cwnd[i]<target:cwnd[i]=min(cwnd[i]+max(1,(target-cwnd[i])*0.3),target)
            else:cwnd[i]=max(target,cwnd[i]*0.95)

            # Send: each flow sends cwnd/RTT rate
            rate=cwnd[i]*MSS/mrt
            sent=rate*DT
            total_sent+=sent
            if step>=30000:total_served[i]+=sent
        queue_bytes+=total_sent

    thru=sum(total_served)/N_FLOWS/90.0/1e6*8
    util=thru/BW*100; stable=1 if abs(util-100)<2 else 0
    return thru,util,stable

if __name__=='__main__':
    print("KCC 2.0 PI SEARCH")
    print("="*45)
    # Q1: Stability sweep
    jobs=[]
    for Kp in [0.3,0.5,0.7,1.0,1.5,2.0,3.0]:
        for Ki in [0.01,0.03,0.05,0.1,0.2]:
            for s in range(3):
                jobs.append((200e-6,Kp,Ki,0.9999,SEED_OFFSET+s))
    # Q2: Margin sweep
    for m in [50,100,150,200,300,500]:
        for s in range(3):
            jobs.append((m*1e-6,1.0,0.05,0.9999,SEED_OFFSET+100+s))
    # Baseline
    for s in range(3):
        jobs.append((200e-6,0.0,0.0,0.9999,SEED_OFFSET+200+s))
    print(f"Total: {len(jobs)} jobs on {min(16,cpu_count())} workers")
    t0=time.time()
    with Pool(min(16,cpu_count())) as p:
        results=p.map(sim_one,jobs)
    print(f"Done: {time.time()-t0:.0f}s\n")

    # Q1 results
    print("Q1: Stability (margin=200us)")
    print(f"{'Kp':>5} {'Ki':>6} {'Thru':>7} {'Util':>6} Stable")
    idx=0
    for Kp in [0.3,0.5,0.7,1.0,1.5,2.0,3.0]:
        for Ki in [0.01,0.03,0.05,0.1,0.2]:
            ts=[results[idx+s][0] for s in range(3)]
            us=[results[idx+s][1] for s in range(3)]
            st=[results[idx+s][2] for s in range(3)]
            avg_t,avg_u=sum(ts)/3,sum(us)/3
            ok="STABLE" if all(st) else "UNSTABLE"
            print(f"{Kp:>5.1f} {Ki:>6.3f} {avg_t:>7.0f} {avg_u:>6.1f}% {ok}")
            idx+=3
    # Q2
    print("\nQ2: Throughput vs margin (Kp=1.0,Ki=0.05)")
    offset=7*5*3
    for m in [50,100,150,200,300,500]:
        ts=[results[offset+(mi)*3+s][0] for mi,mu in enumerate([50,100,150,200,300,500]) if mu==m for s in range(3)]
        print(f"  m={m:>3}us -> {sum(ts)/3:.0f}Mbps")
    # Baseline
    boff=7*5*3+6*3
    bt=[results[boff+s][0] for s in range(3)]
    print(f"\nBASELINE: {sum(bt)/3:.0f}Mbps ({sum(bt)/3/BW*100:.1f}%)")
    best_t,best_i=max((r[0],i) for i,r in enumerate(results))
    best_job=jobs[best_i]
    print(f"BEST: m={int(best_job[0]*1e6)}us Kp={best_job[1]:.1f} Ki={best_job[2]:.3f} -> {best_t:.0f}Mbps")
    print("DONE")
