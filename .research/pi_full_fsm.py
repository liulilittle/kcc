# Full KCC FSM simulation — STARTUP/DRAIN/PROBE_BW + bandwidth estimation + PI
# py -3 this.py
import random,time
from multiprocessing import Pool,cpu_count

MSS=1500; BW=1260.0; N=6; CWND_GAIN=2.0
HIGH_GAIN=2.89; DRAIN_GAIN=0.35
PROBE_GAINS=[1.25,0.75,1.0,1.0,1.0,1.0,1.0,1.0]
FULL_BW_THRESH=1.25; FULL_BW_ROUNDS=3
PI_LO,PI_HI=0.75,1.25

class KCC:
    def __init__(self,tp0,seed):
        self.tp=tp0; self.mr=tp0; self.x=tp0  # geodesic
        self.cnf=self.csl=self.pd=0
        self.cwnd=4; self.phase=0  # PROBE_BW phase
        self.mode='STARTUP'
        self.max_bw=0.0  # BW estimate in Mbps
        self.no_growth_cnt=0
        self.drain_cnt=0
        self.qavg=0.0; self.qbase=0.0
        self.sent_total=0.0
        self.rng=random.Random(seed)
        self.rt=0.0  # round trip time estimate

    def step(self,C_mbps,q_us,mp,ks,Tp_now):
        self.tp=Tp_now
        rtt_eff=self.tp+q_us/1e6+self.rng.gauss(0,0.0002)
        if rtt_eff<=1e-6:rtt_eff=1e-6
        self.rt=rtt_eff

        # Geodesic G1/G2
        if rtt_eff<=self.x:self.x=rtt_eff
        else:self.x=min(self.x*1.12,rtt_eff)
        if rtt_eff<self.mr:self.mr=rtt_eff
        ft=self.mr*1.10;st=self.mr*1.05
        if self.x>=ft:self.cnf+=1;self.csl+=1
        elif self.x>=st:self.cnf=0;self.csl+=1
        else:self.cnf=0
        if self.x<=self.mr:self.cnf=self.csl=0
        if self.cnf>=3:self.mr=self.x;self.cnf=self.csl=0
        elif self.csl>=4:self.mr=self.x;self.cnf=self.csl=0
        if self.cnf==0 and self.csl==0:
            if self.x<self.mr*0.95:self.pd+=1
            else:self.pd=0
            if self.pd>=3:self.mr=self.x;self.pd=0

        mrt=min(self.x,self.mr)
        qi=max(0,rtt_eff-mrt)+0.0005  # measurement noise floor
        self.qavg=self.qavg*0.875+qi*0.125
        if self.qbase<1e-9 or qi<self.qbase*0.9:self.qbase=qi*0.5+self.qbase*0.5
        else:self.qbase=max(self.qbase*0.9999,qi*0.0001)

        # Bandwidth estimation: sliding max of delivery rate
        delivery=self.cwnd*MSS*8/rtt_eff/1e6  # Mbps
        if delivery>self.max_bw:self.max_bw=delivery;self.no_growth_cnt=0
        else:self.no_growth_cnt+=1

        # FSM state transitions
        if self.mode=='STARTUP':
            self.pgain=HIGH_GAIN
            bdp_segs=C_mbps*1e6/8*mrt/MSS
            target=bdp_segs*CWND_GAIN
            if self.cwnd<target:self.cwnd=self.cwnd+max(self.cwnd,target-self.cwnd)*0.5
            else:self.cwnd=target
            # Detect full_bw: 3 consecutive rounds without bw growth
            if self.no_growth_cnt>=FULL_BW_ROUNDS:
                self.mode='DRAIN';self.drain_cnt=0
        elif self.mode=='DRAIN':
            self.pgain=DRAIN_GAIN
            bdp_segs=C_mbps*1e6/8*mrt/MSS
            target=bdp_segs*CWND_GAIN
            if self.cwnd>target:self.cwnd-=(self.cwnd-target)*0.3
            elif self.cwnd<target:self.cwnd=target
            self.drain_cnt+=1
            if self.cwnd<=target*1.1 or self.drain_cnt>=4:
                self.mode='PROBE_BW';self.phase=0
        else:  # PROBE_BW
            pg_raw=PROBE_GAINS[self.phase&7];self.phase+=1
            # PI controller on cruise (gain==1.0)
            if abs(pg_raw-1.0)<0.01 and self.qbase>1e-9:
                qp=max(0,self.qavg-self.qbase)
                margin=mrt*mp
                e=qp-margin;kp=ks/max(1e-6,mrt)
                adj=kp*e;adj=max(-0.25,min(0.25,adj))
                if e<0:adj*=2  # aggressive below margin
                if qp>margin*8:adj=-0.25  # hard brake
                self.pgain=max(PI_LO,min(PI_HI,1.0+adj))
            else:
                self.pgain=pg_raw if pg_raw<1.0 else 1.0
            if pg_raw<1.0 and qi<mrt*0.05:self.pgain=1.0  # drain-skip
            bdp_segs=C_mbps*1e6/8*mrt/MSS
            target=bdp_segs*CWND_GAIN*self.pgain
            target=max(target,4)
            if self.cwnd<target:self.cwnd=min(self.cwnd+(target-self.cwnd)*0.3,target)
            else:self.cwnd=max(target,self.cwnd*0.95)
        return mrt

def sim_one(params):
    mp,ks,seed=params
    rng=random.Random(seed);C=BW
    Tps=[rng.uniform(0.030,0.060) for _ in range(N)]
    flows=[KCC(Tps[i],seed*N+i) for i in range(N)]
    sent=[0.0]*N;queue_s=0.0;loss=0;total_rtts=0

    for r in range(15000):
        # Capacity wander
        if r%1000==0:C=BW+rng.uniform(-50,80)
        C=max(800,min(1500,C))
        # T_prop wander
        if r%500==0:
            for i in range(N):Tps[i]+=rng.gauss(0,0.0005);Tps[i]=max(0.028,min(0.065,Tps[i]))

        # All flows step
        inflight=0.0
        for i in range(N):
            mrt=flows[i].step(C,queue_s,mp,ks,Tps[i])
            inflight+=flows[i].cwnd

        # Bottleneck
        avgTp=sum(Tps)/N
        C_segs=(C*1e6/8/MSS)*avgTp
        q_segs=max(0,inflight-C_segs)
        queue_s=q_segs*MSS*8/(C*1e6)*1e6
        fair=C_segs/N

        # Track throughput (after warmup)
        if r>=3000:
            for i in range(N):
                sent[i]+=min(flows[i].cwnd,fair)*MSS*8/1e6
            total_rtts+=1

        if q_segs>C_segs*3:
            loss+=1
            for i in range(N):flows[i].cwnd=max(1,flows[i].cwnd*0.7)

    if total_rtts==0:return 0,0,0
    avg_sent=sum(sent)/N
    total_time=total_rtts*avgTp
    thru=avg_sent/total_time
    util=thru*N/BW*100
    return thru,util,loss

if __name__=='__main__':
    print("FULL KCC FSM SIMULATION")
    print("="*45)
    jobs=[]
    for mp in [0.002,0.005,0.01,0.02]:
        for ks in [0.5,1.0,2.0,5.0]:
            for s in range(5):jobs.append((mp,ks,s))
    for s in range(5):jobs.append((0.01,0.0,s+100))
    print(f"Jobs: {len(jobs)} on {min(16,cpu_count())} workers")
    t0=time.time()
    with Pool(min(16,cpu_count())) as p:results=p.map(sim_one,jobs)
    print(f"Done: {time.time()-t0:.0f}s\n")

    base=[r for r in results[-5:] if r[0]>0]
    bu=sum(b[0]for b in base)/len(base)
    print(f"BASELINE: {bu:.0f}Mbps/flow ({bu*N/BW*100:.0f}% total)\n")
    print(f"{'m%':>5} {'Ks':>5} {'Thru':>7} {'Util':>6} {'Loss':>5} {'Gain':>6}")
    best=None
    for mp in [0.002,0.005,0.01,0.02]:
        for ks in [0.5,1.0,2.0,5.0]:
            rs=[r for j,r in zip(jobs,results) if abs(j[0]-mp)<1e-9 and abs(j[1]-ks)<1e-9]
            if not rs:continue
            t=sum(r[0]for r in rs)/len(rs);u=sum(r[1]for r in rs)/len(rs)
            l=sum(r[2]for r in rs)/len(rs)
            g=(t-bu)/bu*100 if bu>0 else 0
            print(f"{mp*100:>4.1f}% {ks:>5.1f} {t:>7.0f} {u:>6.1f}% {l:>5.0f} {g:>+5.1f}%")
            if l==0 and (best is None or t>best[0]):best=(t,u,mp,ks,g)
    if best:print(f"\nBEST: m={best[2]*100:.1f}% Kp={best[3]:.1f} -> {best[0]:.0f}Mbps ({best[1]:.1f}%) +{best[4]:+.1f}%")
    print("DONE")
