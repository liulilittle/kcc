# KCC 2.0 PI nonlinear search - stability + throughput + wander
# py -3 this_file.py
import random, sys, time
from multiprocessing import Pool, cpu_count

SCALE, SHIFT = 1024, 10; MSS = 1500
BW_MBPS=1260; BBR_UNIT=256; CWND_GAIN=2.0
FAST_N,SLOW_N,PD_N=3,4,3
CYCLE=[1.25,0.75,1.0,1.0,1.0,1.0,1.0,1.0]
STEPS_WARM,STEPS_TEST=60000,60000  # 60s + 60s at 1ms dt

def sim(params):
    margin,Kp,Ki,ff,nflows,seed=params
    rng=random.Random(seed)
    # flows state
    tp=[rng.uniform(0.030,0.060)for _ in range(nflows)]
    mr=tp[:]; x=[t*SCALE for t in tp]
    cnf=[0]*nflows; csl=[0]*nflows; pd=[0]*nflows
    cwnd=[4.0]*nflows; cyc=[0]*nflows
    qavg=[0.0]*nflows; qbase=[0.0]*nflows; pi=[0.0]*nflows
    bw_est=[BW_MBPS/nflows]*nflows
    full_bw=[False]*nflows; full_cnt=[0]*nflows
    total_sent=[0.0]*nflows; losses=[0]*nflows
    pgain=[1.0]*nflows
    queue_us=0.0; C=float(BW_MBPS)

    for step in range(STEPS_WARM+STEPS_TEST):
        # capacity wander
        if step%10000==0: C=BW_MBPS+rng.uniform(-30,60)
        # queue dynamics
        drain_bytes=C*1e6/8*0.001; queue_us=max(0,queue_us-0.001*1e6)
        bdp_bytes=C*1e6/8*max(tp)
        for i in range(nflows):
            # T_prop wander
            if step%int(tp[i]/0.001)==0:
                tp[i]+=rng.gauss(0,0.001); tp[i]=max(0.030,min(0.060,tp[i]))
                rtt=tp[i]+queue_us/1e6+rng.gauss(0,0.0002)
                if rtt<=0:rtt=1e-6
                # geodesic
                z=rtt; xv=x[i]
                if z<=xv:xv=z
                else:xv=min(xv*1.12,z)
                x[i]=xv; mv=mr[i]
                ft=mv*1.10;st=mv*1.05
                if xv>=ft:cnf[i]+=1;csl[i]+=1
                elif xv>=st:cnf[i]=0;csl[i]+=1
                else:cnf[i]=0
                if xv<=mv:cnf[i]=0;csl[i]=0
                if cnf[i]>=FAST_N:mr[i]=xv;cnf[i]=csl[i]=0
                elif csl[i]>=SLOW_N:mr[i]=xv;cnf[i]=csl[i]=0
                if cnf[i]==0 and csl[i]==0:
                    if xv<mv*0.95:pd[i]+=1
                    else:pd[i]=0
                    if pd[i]>=PD_N:mr[i]=xv;pd[i]=0
                # bandwidth estimation
                delivered=min(cwnd[i]*MSS,C*1e6/8*0.001/nflows)
                bw_est[i]=bw_est[i]*0.9+delivered*8/0.001*0.1
                # STARTUP detection
                if not full_bw[i]:
                    if bw_est[i]<bw_est[i]*1.25:full_cnt[i]+=1
                    else:full_cnt[i]=0
                    if full_cnt[i]>=3:full_bw[i]=True
                # qdelay
                mrt=min(xv,mv); qi=max(0,rtt-mrt)
                qavg[i]=qavg[i]*0.875+qi*0.125
                if qbase[i]<1e-6 or qi<min(qbase[i],mv*0.01):
                    qbase[i]=qi*0.5+qbase[i]*0.5
                else:qbase[i]=max(qbase[i]*ff,qi*(1-ff))
                # PI cruise
                pg=1.0; cyc[i]=(cyc[i]+1)%8
                pg_raw=CYCLE[cyc[i]&7]
                if abs(pg_raw-1.0)<0.01 and qbase[i]>1e-9:
                    qp=max(0,qavg[i]-qbase[i]); e=qp-margin
                    pi[i]+=e*Ki*mv
                    pi[i]=max(-0.001,min(0.001,pi[i]))
                    if qp<=margin*3 and queue_us<mv*0.15*1e6:
                        if abs(e)>20e-6:
                            a=Kp*e+pi[i]; a=max(-0.05,min(0.05,a))
                            pg=1.0+a
                    else:pi[i]=0.0
                if pg_raw<1.0 and qi<mv*0.05:pg=1.0
                else:pg=pg_raw if pg_raw!=1.0 else pg
                pg=min(1.05,max(0.95,pg)); pgain[i]=pg
                # cwnd
                tgt=mv*C*1e6/8/MSS*CWND_GAIN*pg
                tgt=max(tgt,4)
                if cwnd[i]<tgt:cwnd[i]=min(cwnd[i]+max(1,(tgt-cwnd[i])*0.3),tgt)
                else:cwnd[i]=max(tgt,cwnd[i]*0.95)
                # send
                s=min(cwnd[i]*MSS, C*1e6/8*0.001/nflows)
                if step>=STEPS_WARM:total_sent[i]+=s
            # queue inject from this flow's sending
        # total inflight -> queue delay
        inflight_bytes=sum(cwnd[i]*MSS for i in range(nflows))
        bdp_bytes=C*1e6/8*max(tp)
        extra=inflight_bytes-bdp_bytes
        if extra>0:queue_us+=extra*8/(C*1e6)*1e6
        else:queue_us*=0.9
        queue_us=max(0,min(queue_us,max(tp)*1e6))
        if queue_us>max(tp)*0.5*1e6:
            for i in range(nflows):losses[i]+=1
            queue_us*=0.3

    thru=sum(total_sent)/nflows/60.0/1e6*8
    util=thru/C*100; loss=sum(losses); qpeak=max(tp)*1e6
    osc=sum(abs(pgain[i]-1.0)for i in range(nflows))/nflows*100
    return thru,util,loss,qpeak,osc

if __name__=='__main__':
    print("KCC 2.0 PI NONLINEAR SEARCH")
    print("="*50)
    jobs=[]
    # Q1: stability - sweek Kp,Ki at margin=200
    for Kp in [0.1,0.3,0.5,0.7,1.0,1.5,2.0,3.0,5.0]:
        for Ki in [0.001,0.005,0.01,0.03,0.05,0.1,0.2,0.5]:
            jobs.append((200e-6,Kp,Ki,0.9999,6,0))
    # Q2: margin sweep with Kp=1.0,Ki=0.01
    for m in [50,100,150,200,250,300,400,500]:
        for s in range(3):
            jobs.append((m*1e-6,1.0,0.01,0.9999,6,1000+s))
    # Baseline
    for s in range(3):
        jobs.append((200e-6,0,0,0.9999,6,2000+s))
    print(f"Jobs: {len(jobs)} on {min(16,cpu_count())} workers")
    t0=time.time()
    with Pool(min(16,cpu_count())) as p:
        results=p.map(sim,jobs)
    print(f"Done in {time.time()-t0:.0f}s")
    # Show Q1 results
    print("\nQ1: Stability (margin=200us)")
    print(f"{'Kp':>5} {'Ki':>7} {'Thru':>7} {'Util':>6} {'Loss':>5} {'Osc%':>6}")
    for Kp in [0.1,0.3,0.5,0.7,1.0,1.5,2.0,3.0,5.0]:
        for Ki in [0.001,0.005,0.01,0.03,0.05,0.1,0.2,0.5]:
            idx=0; thru,util,loss,qp,osc=results[idx]; idx+=1
            s="STABLE" if osc<2.0 else("OK"if osc<5.0 else("OSC"if osc<10 else"UNSTABLE"))
            print(f"{Kp:>5.1f} {Ki:>7.4f} {thru:>7.0f} {util:>6.1f} {loss:>5.0f} {osc:>6.1f} {s}")
    # Show Q2 results
    print("\nQ2: Throughput vs margin (Kp=1.0,Ki=0.01)")
    for m in [50,100,150,200,250,300,400,500]:
        ts=[results[72+(i*3)+j][0]for i,mu in enumerate([50,100,150,200,250,300,400,500])if mu==m for j in range(3)]
        avg=sum(ts)/3 if ts else 0
        print(f"  margin={m:>3}us -> {avg:.0f}Mbps")
    # Baseline
    bt=[results[-3+j][0]for j in range(3)]
    print(f"\nBASELINE: {sum(bt)/3:.0f}Mbps")
    print("BEST:",max(r[0]for r in results),"Mbps")
    print("DONE")
