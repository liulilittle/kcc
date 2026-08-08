"""PI controller validation - KCC 2.0 bottleneck scenario."""
import random, sys

SCALE, SHIFT = 1024, 10; MSS = 1500
JITTER_DIV = 100.0; BBR_UNIT = 256
CWND_GAIN = 2.0
G3_FAST, G3_SLOW, PD_N = 4, 4, 3
CYCLE = [1.25, 0.75, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
BW_MBPS = 1260
TPROPS = [30000, 31000, 31600, 32000, 33000, 40000, 43000, 45000, 58000, 59000, 200000, 300000, 336000]
SEEDS = 4; RTTS = 3000; WARMUP = 1000

class Flow:
    def __init__(self, tp, seed, margin, kp_num, ki_num, imax, amax):
        self.tp=tp; self.mr=tp; self.x=tp*SCALE; self.cnf=self.csl=self.pd=0
        self.cwnd=4; self.cycle=0; self.qavg=tp*0.01; self.qbase=0.0; self.pi_int=0.0
        self.rng=random.Random(seed); self.sent=0.0
        self.margin=margin; self.kp=kp_num; self.ki=ki_num; self.imax=imax; self.amax=amax

    def step(self, queue_us):
        tp=self.tp; rng=self.rng; mr=self.mr; x=self.x
        rtt=max(1,int(tp+queue_us+rng.gauss(0,max(1.0,tp/JITTER_DIV))))
        z=rtt*SCALE
        if z<=x: x=z
        else: x=min(x+x*122//1000,z)
        ft=mr*SCALE*11//10; st=mr*SCALE*21//20; bl=mr*SCALE; cnf=self.cnf; csl=self.csl
        if x>=ft: cnf+=1; csl+=1
        elif x>=st: cnf=0; csl+=1
        else: cnf=0
        if x<=bl: cnf=0; csl=0
        if cnf>=G3_FAST: mr=x>>SHIFT; cnf=csl=0
        elif csl>=G3_SLOW: mr=x>>SHIFT; cnf=csl=0
        xus=x>>SHIFT; pd=self.pd
        if cnf==0 and csl==0:
            if xus<mr*95//100: pd+=1
            else: pd=0
            if pd>=PD_N: mr=xus; pd=0
        qi=max(0,rtt-xus); qa=self.qavg*7/8+qi/8
        if self.qbase<=0 or qi<self.qbase: self.qbase=qi
        else: self.qbase=self.qbase*0.999+qi*0.001
        mrt=min(xus,mr); pgain=CYCLE[self.cycle&7]; self.cycle+=1
        if abs(pgain-1.0)<0.01 and self.qbase>0:
            qp=max(0,qa-self.qbase); e=qp-self.margin
            self.pi_int+=e*self.ki/max(1,tp)
            self.pi_int=max(-self.imax,min(self.imax,self.pi_int))
            if qp<=self.margin*2:
                a=self.kp*e/max(1,tp)+self.pi_int
                a=max(-self.amax,min(self.amax,a))
                pgain=1.0+a/BBR_UNIT
            else: self.pi_int=0.0
        if pgain<1.0 and queue_us<mrt*0.05: pgain=1.0
        if queue_us>mrt*0.15: self.pi_int=0.0
        target=mrt*BW_MBPS*1000/8/MSS*CWND_GAIN*pgain
        target=max(target,4)
        cw=self.cwnd
        if cw<target: cw+=max(1,(target-cw)*0.3)
        else: cw=max(target,cw-1)
        served=min(cw,BW_MBPS*1000*mrt/8/MSS/len(TPROPS)/SEEDS)
        self.sent+=served*MSS*8/1e6
        self.mr=mr; self.x=x; self.cnf=cnf; self.csl=csl; self.pd=pd
        self.qavg=qa; self.cwnd=cw

def sim(m, kp, ki, imax, amax):
    fl=[Flow(tp,s*7919+tp,m,kp,ki,imax,amax) for tp in TPROPS for s in range(SEEDS)]
    qu=0.0
    for _ in range(WARMUP+RTTS):
        total_cwnd=sum(f.cwnd for f in fl)
        bdp_segs=BW_MBPS*1000/8/MSS
        if total_cwnd>bdp_segs: qu+=(total_cwnd-bdp_segs)*MSS*8/(BW_MBPS*1e6)*1e6
        else: qu*=0.8
        qu=max(0,min(qu,50000.0))
        for f in fl: f.step(qu/2)
    thru=sum(f.sent for f in fl)/len(fl)
    return thru/(RTTS/1e6)/BW_MBPS*100

if __name__=='__main__':
    print("PI VALIDATION - 1.26Gbps")
    print("="*45)
    bu=sim(200,0,0,0,0)
    print(f"Baseline: {bu:.1f}%")
    best, bp = bu, None
    for m in [100,200,400]:
        for kp in [500, 786, 1000]:
            for ki in [100, 154, 250]:
                for imax in [50, 100]:
                    u=sim(m,kp,ki,imax,13)
                    g=u-bu; tag="***"if g>2 else("++"if g>1 else("+"if g>0.5 else" "))
                    print(f"  m={m:>3} kp={kp:>4} ki={ki:>3} im={imax:>3} -> {u:.1f}%{tag} ({g:+.1f}%)")
                    if u>best: best=u; bp=(m,kp,ki,imax)
    print(f"\nBEST: m={bp[0]} kp={bp[1]} ki={bp[2]} im={bp[3]} -> {best:.1f}% (+{best-bu:.1f}%)")
    print("DONE")
