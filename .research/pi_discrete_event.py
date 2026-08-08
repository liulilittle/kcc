# KCC 2.0 discrete-event simulation — proper per-RTT, full FSM
# py -3 this.py
import random,heapq,sys,time
from multiprocessing import Pool,cpu_count

MSS=1500; BW=1260.0; N=6; DURATION=200.0
CWND_GAIN=2.0; PI_LO,PI_HI=0.75,1.25
CYCLE=[1.25,0.75,1.0,1.0,1.0,1.0,1.0,1.0]

class KCCFlow:
    def __init__(self,tp0,seed):
        self.tp=tp0; self.mr=tp0; self.x=tp0
        self.cnf=self.csl=self.pd=0; self.cwnd=4.0
        self.cyc=0; self.mode='STARTUP'; self.full_bw=False
        self.max_bw=0.0; self.bw_samples=[]
        self.rtt_cnt=0; self.drain_cnt=0
        self.qavg=0.0; self.qbase=0.0; self.pgain=1.0
        self.sent_total=0.0; self.loss=0
        self.rng=random.Random(seed)

    def step(self,C_mbps,queue_s,now):
        rtt=self.tp+queue_s+self.rng.gauss(0,0.0002)
        if rtt<=1e-6:rtt=1e-6

        # Geodesic
        if rtt<=self.x:self.x=rtt
        else:self.x=min(self.x*1.12,rtt)
        if rtt<self.mr:self.mr=rtt
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
        # qdelay
        qi=max(0,rtt-mrt)
        self.qavg=self.qavg*0.875+qi*0.125
        if self.qbase<1e-9 or qi<self.qbase*0.9:
            self.qbase=qi*0.5+self.qbase*0.5
        else:self.qbase=max(self.qbase*0.9999,qi*0.0001)

        self.rtt_cnt+=1
        # FSM
        if self.mode=='STARTUP':
            self.pgain=2.89  # high_gain
            bdp_segs=C_mbps*1e6/8*mrt/MSS
            target=bdp_segs*CWND_GAIN
            if self.cwnd<target:self.cwnd=self.cwnd+self.cwnd*0.5  # exponential
            else:self.cwnd=target
            # full_bw detection: 3 rounds without bw growth
            delivery_rate=self.cwnd*MSS*8/rtt/1e6
            if delivery_rate<=self.max_bw*1.25:self.drain_cnt+=1
            else:self.drain_cnt=0
            self.max_bw=max(self.max_bw,delivery_rate)
            if self.drain_cnt>=3:self.mode='DRAIN';self.drain_cnt=0
        elif self.mode=='DRAIN':
            self.pgain=0.35
            bdp_segs=C_mbps*1e6/8*mrt/MSS
            target=bdp_segs*CWND_GAIN
            if self.cwnd>target:self.cwnd*=0.7
            self.drain_cnt+=1
            if self.cwnd<=target or self.drain_cnt>=4:
                self.mode='PROBE_BW';self.cyc=0;self.drain_cnt=0
        else:  # PROBE_BW
            self.cyc=(self.cyc+1)%8;pg_raw=CYCLE[self.cyc&7]
            # Aggressive PI on cruise (gain>=1.0)
            if pg_raw>=1.0 and self.qbase>1e-9:
                qp=max(0,self.qavg-self.qbase)
                margin=mrt*0.01
                e=qp-margin;kp=1.0/max(1e-6,mrt)
                adj=kp*e;adj=max(-0.25,min(0.25,adj))
                if e<0:adj*=2
                if qp>margin*8:adj=-0.25
                self.pgain=max(PI_LO,min(PI_HI,1.0+adj))
            else:
                self.pgain=pg_raw if pg_raw<1.0 else 1.0
            if pg_raw<1.0 and qi<mrt*0.05:self.pgain=1.0
            bdp_segs=C_mbps*1e6/8*mrt/MSS
            target=bdp_segs*CWND_GAIN*self.pgain
            target=max(target,4)
            if self.cwnd<target:self.cwnd=min(self.cwnd+(target-self.cwnd)*0.3,target)
            else:self.cwnd=max(target,self.cwnd*0.95)

    def rate_mbps(self):return self.cwnd*MSS*8/self.tp/1e6

def sim_one(params):
    margin_pct,kp_scale,seed=params
    rng=random.Random(seed);C=BW
    base_rtts=[rng.uniform(0.030,0.060) for _ in range(N)]
    flows=[KCCFlow(base_rtts[i],seed*N+i) for i in range(N)]
    queue_s=0.0;total_thru=0.0;total_loss=0;started_at=[0.0]*N
    t=0.0;warmup=30.0

    while t<DURATION:
        if t%10.0<0.001:C=BW+rng.uniform(-30,60)
        # advance to next RTT event
        next_t=float('inf')
        for i in range(N):
            if t>=started_at[i]:
                started_at[i]=t+flows[i].tp+queue_s
            next_t=min(next_t,started_at[i])
        if next_t>t+0.001:
            # drain queue during idle
            drain_s=(next_t-t);queue_s=max(0,queue_s-drain_s)
        t=next_t

        # Update flows whose RTT timer fires
        inflight=0.0
        for i in range(N):
            if abs(t-started_at[i])<1e-6:
                flows[i].step(C,queue_s,t)
                flows[i].tp+=rng.gauss(0,0.001)
                flows[i].tp=max(0.030,min(0.060,flows[i].tp))
                started_at[i]=t+flows[i].tp+queue_s
            inflight+=flows[i].cwnd*MSS

        # Bottleneck: serve at C
        C_bytes_per_s=C*1e6/8
        avg_rtt=sum(f.tp for f in flows)/N
        served=min(inflight,C_bytes_per_s*avg_rtt)
        queue_bytes=max(0,inflight-served)
        queue_s=queue_bytes/(C*1e6/8)
        if queue_s>avg_rtt*4:total_loss+=1;queue_s*=0.3

        if t>=warmup:
            fair=served/N
            for i in range(N):
                flows[i].sent_total+=min(flows[i].cwnd*MSS,fair)

    thru=sum(f.sent_total for f in flows)/N/(DURATION-warmup)/1e6*8
    util=thru/BW*100
    return thru,util,total_loss

if __name__=='__main__':
    print("KCC DISCRETE-EVENT — full FSM, aggressive PI")
    print("="*50)
    jobs=[]
    for mp in [0.005,0.01,0.02,0.05]:
        for ks in [0.5,1.0,2.0,5.0]:
            for s in range(3):
                jobs.append((mp,ks,1000+s))
    for s in range(3):jobs.append((0.01,0.0,2000+s))
    print(f"Jobs: {len(jobs)} on {min(8,cpu_count())} workers")
    t0=time.time()
    with Pool(min(8,cpu_count())) as p:results=p.map(sim_one,jobs)
    print(f"Done: {time.time()-t0:.0f}s\n")
    base=[r for r in results[-3:]]
    bu=sum(b[0] for b in base)/3
    print(f"BASELINE: {bu:.0f}Mbps ({bu/BW*100:.1f}%)\n")
    print(f"{'m%':>5} {'Ks':>5} {'Thru':>7} {'Util':>6} {'Loss':>5}")
    best=None
    for mp in [0.005,0.01,0.02,0.05]:
        for ks in [0.5,1.0,2.0,5.0]:
            rs=[r for j,r in zip(jobs,results) if abs(j[0]-mp)<1e-9 and abs(j[1]-ks)<1e-9]
            if not rs:continue
            t=sum(r[0]for r in rs)/len(rs);u=sum(r[1]for r in rs)/len(rs)
            l=sum(r[2]for r in rs)/len(rs);g=u-bu/BW*100
            print(f"{mp*100:>4.1f}% {ks:>5.1f} {t:>7.0f} {u:>6.1f}% {l:>5.0f} {'***' if g>5 else '++' if g>1 else '+' if g>0 else ''} {g:+.1f}%")
            if l==0 and (best is None or t>best[0]):best=(t,u,mp,ks)
    if best:
        print(f"\nBEST: margin={best[2]*100:.1f}% Kp_scale={best[3]:.1f} -> {best[0]:.0f}Mbps ({best[1]:.1f}%) +{best[1]-bu/BW*100:.1f}% over baseline")
    print("DONE")
