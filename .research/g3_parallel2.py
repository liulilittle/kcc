#!/usr/bin/env python3

"""G3 FP: parallel, with warmup to eliminate cold-start."""
import random
from multiprocessing import Pool

SCALE, SHIFT = 1024, 10
FAST_TH, PD_N = 11, 3
SEEDS, RTTS, WARMUP = 200, 50000, 2000
TPROPS = [1000, 5000, 10000, 45000, 100000, 1000000]
NS = [2, 3, 4, 5, 6, 8, 10, 15, 20]

def run_one(args):
    tp, N = args
    fp, mx = 0, 0
    for s in range(SEEDS):
        rng = random.Random(s * 7919 + tp * 1000003)
        mr = tp; x = tp * SCALE
        cnf = csl = pd = 0
        jtr = tp / 100.0
        # warmup
        for _ in range(WARMUP):
            rtt = tp + rng.gauss(0, jtr)
            z = max(1, int(rtt)) * SCALE
            if z <= x: x = z
            else: x = min(x + x * 12 // 100, z)
            ft = mr * SCALE * FAST_TH // 10
            st = mr * SCALE * 21 // 20
            bl = mr * SCALE
            if x >= ft: cnf += 1; csl += 1
            elif x >= st: cnf = 0; csl += 1
            else: cnf = 0
            if x <= bl: cnf = 0; csl = 0
            if cnf >= 3: mr = x >> SHIFT; cnf = csl = 0
            elif csl >= N: mr = x >> SHIFT; cnf = csl = 0
            xus = x >> SHIFT
            if cnf == 0 and csl == 0:
                if xus < mr: pd += 1
                else: pd = 0
                if pd >= PD_N: mr = xus; pd = 0
        # test
        sm = 0
        for _ in range(RTTS):
            rtt = tp + rng.gauss(0, jtr)
            z = max(1, int(rtt)) * SCALE
            if z <= x: x = z
            else: x = min(x + x * 12 // 100, z)
            ft = mr * SCALE * FAST_TH // 10
            st = mr * SCALE * 21 // 20
            bl = mr * SCALE
            if x >= ft: cnf += 1; csl += 1
            elif x >= st: cnf = 0; csl += 1
            else: cnf = 0
            if x <= bl: cnf = 0; csl = 0
            if cnf >= 3: fp += 1; sm = max(sm, cnf); break
            elif csl >= N: fp += 1; sm = max(sm, csl); break
            if csl > sm: sm = csl
            xus = x >> SHIFT
            if cnf == 0 and csl == 0:
                if xus < mr: pd += 1
                else: pd = 0
                if pd >= PD_N: mr = xus; pd = 0
        mx = max(mx, sm)
    return tp, N, fp, mx

if __name__ == '__main__':
    jobs = [(tp, n) for tp in TPROPS for n in NS]
    print(f"Running {len(jobs)} jobs on 8 workers...")
    with Pool(8) as p:
        results = p.map(run_one, jobs)

    print(f"\n{'T_prop':>9} |", end="")
    for n in NS: print(f" N={n:>2}|", end="")
    print(" max_csl")
    print("-" * (12 + 8 * len(NS)))
    by_tp = {}
    for tp, n, fp, mx in results:
        by_tp.setdefault(tp, {})[n] = (fp, mx)
    for tp in TPROPS:
        print(f"{tp:>6} us |", end="")
        for n in NS:
            fp, mx = by_tp[tp][n]
            print(f" {fp:>3}|", end="")
        print(f" max={max(v[1] for v in by_tp[tp].values())}")
    print("\nDONE")
