"""PI controller — simple correct bottleneck model."""
import random

SCALE, SHIFT = 1024, 10; JITTER_DIV = 100.0; MSS = 1500
CYCLE_GAIN = [1.25, 0.75, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
CWND_GAIN, BBR_UNIT = 2.0, 256

def run(tp_us, bw_mbps, flows, Kp, Ki, margin, rtts):
    # Per-flow state
    mrs = [tp_us]*flows; xs = [tp_us*SCALE]*flows
    cnfs = [0]*flows; csls = [0]*flows; pds = [0]*flows
    cwnds = [4]*flows; cycles = [0]*flows
    qavgs = [0.0]*flows; qbases = [0.0]*flows; pi_ints = [0.0]*flows
    rngs = [random.Random(i*7919) for i in range(flows)]
    total_thru = [0.0]*flows
    queue_segs = 0

    for _ in range(rtts):
        bdp_segs = bw_mbps*1e6*tp_us/1e6/8/MSS
        qd_us = queue_segs * MSS * 8 / (bw_mbps*1e6) * 1e6

        for i in range(flows):
            tp, rng = tp_us, rngs[i]
            rtt = max(1, int(tp + qd_us + rng.gauss(0, max(1.0,tp/JITTER_DIV))))
            z = rtt*SCALE; x = xs[i]; mr = mrs[i]
            # G1/G2
            if z <= x: x = z
            else: x = min(x + x*122//1000, z)
            # G3
            ft = mr*SCALE*11//10; st = mr*SCALE*21//20; bl = mr*SCALE
            cnf, csl = cnfs[i], csls[i]
            if x >= ft: cnf += 1; csl += 1
            elif x >= st: cnf = 0; csl += 1
            else: cnf = 0
            if x <= bl: cnf = 0; csl = 0
            old_mr = mr
            if cnf >= 4: mr = x>>SHIFT; cnf = csl = 0
            elif csl >= 4: mr = x>>SHIFT; cnf = csl = 0
            xus = x>>SHIFT
            pd = pds[i]
            if cnf==0 and csl==0:
                if xus < mr*95//100: pd += 1
                else: pd = 0
                if pd >= 3: mr = xus; pd = 0
            # qdelay EWMA
            qi = max(0, rtt - xus)
            qa = qavgs[i]*7/8 + qi/8
            qb = qi if qbases[i]==0 or qi < qbases[i] else qbases[i]*0.999 + qi*0.001
            # PI
            mrt = min(xus, mr)
            pgain = CYCLE_GAIN[cycles[i]&7]; cycles[i] += 1
            if abs(pgain-1.0) < 0.01:
                qp = max(0, qa - qb)
                pi_ints[i] += (qp-margin)*Ki*(mrt/1e6)
                pi_ints[i] = max(-0.05, min(0.05, pi_ints[i]))
                pgain = 1.0 + max(-0.05, min(0.05, Kp*(qp-margin) + pi_ints[i]))
            if pgain < 1.0 and qd_us < mrt*0.05: pgain = 1.0
            if qd_us > mrt*0.15: pi_ints[i] = 0.0
            # cwnd
            target = bdp_segs * CWND_GAIN * pgain
            target = max(target, 4)
            cw = cwnds[i]
            if cw < target: cw += max(1.0, (target-cw)*0.3)
            else: cw = max(target, cw - 1)
            cw = min(cw, target)
            # save state
            xs[i]=x; mrs[i]=mr; cnfs[i]=cnf; csls[i]=csl; pds[i]=pd
            cwnds[i]=cw; qavgs[i]=qa; qbases[i]=qb

        # Bottleneck: total inflight vs BDP
        total_inflight = sum(cwnds)
        if total_inflight <= bdp_segs:
            for i in range(flows): total_thru[i] += cwnds[i]*MSS*8/1e6
            queue_segs = 0
        else:
            fair = bdp_segs / flows
            for i in range(flows): total_thru[i] += min(cwnds[i], fair)*MSS*8/1e6
            queue_segs = total_inflight - bdp_segs

    avg_thru = sum(total_thru)/flows/(rtts*tp_us/1e6)
    util = avg_thru/bw_mbps*100
    return avg_thru, util

if __name__ == '__main__':
    print("PI CONTROLLER SWEEP — Simple BDP Bottleneck")
    print("="*50)
    for Kp in [0.01, 0.02, 0.04]:
        for Ki in [0.001, 0.002, 0.005]:
            for m in [50, 100, 200]:
                pu = 0.0; bu = 0.0
                for tp in [5000, 10000, 45000]:
                    for fl in [1, 4]:
                        _, u = run(tp, 1000, fl, Kp, Ki, m, 3000)
                        pu += u
                        _, u = run(tp, 1000, fl, 0, 0, 0, 3000)
                        bu += u
                pu /= 6; bu /= 6
                g = pu - bu
                tag = "***" if g > 1.0 else ("++" if g > 0.5 else ("+" if g > 0 else " "))
                print(f"  Kp={Kp:.3f} Ki={Ki:.3f} m={m:>3}us  PI={pu:.1f}%  base={bu:.1f}%  gain={g:+.1f}% {tag}")
    print("DONE")
