#!/usr/bin/env python3

"""N=2,3,4,5 exhaustive FP rate computation. 100 T_props x 1000 seeds x 100k RTTs."""
import random, sys, math

SCALE, SHIFT = 1024, 10
FAST_TH, PD_N = 11, 3

TPROPS = [25, 50, 75, 100, 150, 200, 250, 300, 400, 500,
          750, 1000, 1500, 2000, 2500, 3000, 4000, 5000,
          7500, 10000, 15000, 20000, 25000, 35000, 45000,
          60000, 80000, 100000, 150000, 200000, 300000,
          400000, 500000, 750000, 1000000]
NS = [2, 3, 4, 5]
SEEDS = 1000
RTTS_PER = 100000
TOTAL = len(TPROPS) * SEEDS * RTTS_PER  # = 35 * 1000 * 100000 = 3.5 billion

def sim_h0(tp, N):
    """Return max_slow_ctr across seeds. Count FPs."""
    fp_count = 0
    max_ever = 0
    for s in range(SEEDS):
        rng = random.Random(s * 7919 + hash(str(tp)) % 9999991)
        mr = tp
        x = tp * SCALE
        cnf, csl, pd = 0, 0, 0
        jtr = max(1.0, tp / 100.0)
        seed_max = 0
        for _ in range(RTTS_PER):
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
            if cnf >= 3: mr = x >> SHIFT; cnf = 0; csl = 0; fp_count += 1; seed_max = max(seed_max, csl); break
            elif csl >= N: mr = x >> SHIFT; cnf = 0; csl = 0; fp_count += 1; seed_max = max(seed_max, csl); break
            if csl > seed_max: seed_max = csl
            xus = x >> SHIFT
            if cnf == 0 and csl == 0:
                if xus < mr: pd += 1
                else: pd = 0
                if pd >= PD_N: mr = xus; pd = 0
        max_ever = max(max_ever, seed_max)
    return fp_count, max_ever

print(f"G3 SLOW FP RATE: {len(TPROPS)} T_props x {SEEDS} seeds x {RTTS_PER} RTTs = {TOTAL/1e9:.1f}B samples")
print()
hdr = f"{'T_prop':>9} |" + "|".join(f" N={n} FP |max" for n in NS)
print(hdr)
print("-" * len(hdr))

for tp in TPROPS:
    row = f"{tp:>6} us |"
    for n in NS:
        fp, mx = sim_h0(tp, n)
        row += f"  {fp:>3}  {mx:>2}|"
    print(row)
    sys.stdout.flush()

print(f"\nTOTAL: {TOTAL/1e9:.1f}B samples")
