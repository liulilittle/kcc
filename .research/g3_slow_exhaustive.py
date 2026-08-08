#!/usr/bin/env python3

"""G3 slow-counter exhaustive sweep: find minimum safe N across 25us-1s."""
import random
import sys

SCALE = 1024
SHIFT = 10
JITTER_DIV = 100.0  # sigma = T_prop / 100, but bounded by 1us clock granularity
FAST_TH = 11
SLOW_TH = 21
PD_N = 3
H0_SEEDS = 1000
H0_RTTS  = 50000

TPROPS = [25, 50, 100, 250, 500, 1000, 2500, 5000, 10000,
          25000, 45000, 50000, 75000, 100000, 250000, 500000, 750000, 1000000]
NS = [2, 3, 4, 5, 6, 7, 8, 10, 12, 15, 20]

def run_h0(tp, N, seeds=200, rtts=50000):
    """Count false positives under pure noise."""
    fp = 0
    for s in range(seeds):
        rng = random.Random(s * 7919)
        mr = tp
        x = tp * SCALE
        cnf, csl, pd = 0, 0, 0
        jtr = max(1.0, tp / JITTER_DIV)
        for _ in range(rtts):
            rtt = tp + rng.gauss(0, jtr)
            z = max(1, int(rtt)) * SCALE
            if z <= x:
                x = z
            else:
                x = min(x + x * 122 // 1000, z)
            ft = mr * SCALE * FAST_TH // 10
            st = mr * SCALE * SLOW_TH // 20
            bl = mr * SCALE
            if x >= ft:
                cnf += 1
                csl += 1
            elif x >= st:
                cnf = 0
                csl += 1
            else:
                cnf = 0
            if x <= bl:
                cnf = 0
                csl = 0
            if cnf >= 4:
                mr = x >> SHIFT
                cnf = 0
                csl = 0
            elif csl >= N:
                mr = x >> SHIFT
                cnf = 0
                csl = 0
                fp += 1
                break
            xus = x >> SHIFT
            if cnf == 0 and csl == 0:
                if xus < mr:
                    pd += 1
                    if pd >= PD_N:
                        mr = xus
                        pd = 0
                else:
                    pd = 0
    return fp / seeds

def run_h1(tp, amp_pct, N, seeds=30, max_rtts=5000):
    """Measure detection delay for path increase."""
    delays = []
    for s in range(seeds):
        rng = random.Random(s * 7919)
        mr = tp
        x = tp * SCALE
        cnf, csl, pd = 0, 0, 0
        jtr = max(1.0, tp / JITTER_DIV)
        # warmup
        for _ in range(1000):
            rtt = tp + rng.gauss(0, jtr)
            z = max(1, int(rtt)) * SCALE
            if z <= x:
                x = z
            else:
                x = min(x + x*122//1000, z)
            ft = mr*SCALE*FAST_TH//10
            st = mr*SCALE*SLOW_TH//20
            bl = mr*SCALE
            if x >= ft:
                cnf += 1
                csl += 1
            elif x >= st:
                cnf = 0
                csl += 1
            else:
                cnf = 0
            if x <= bl:
                cnf = 0
                csl = 0
            if cnf >= 4:
                mr = x >> SHIFT
                cnf = 0
                csl = 0
            elif csl >= N:
                mr = x >> SHIFT
                cnf = 0
                csl = 0
            xus = x >> SHIFT
            if cnf == 0 and csl == 0:
                if xus < mr:
                    pd += 1
                    if pd >= PD_N:
                        mr = xus
                        pd = 0
                else:
                    pd = 0
        new_tp = int(tp * (1 + amp_pct/100))
        jtr = max(1.0, new_tp / JITTER_DIV)
        for i in range(1, max_rtts+1):
            rtt = new_tp + rng.gauss(0, jtr)
            z = max(1, int(rtt)) * SCALE
            if z <= x:
                x = z
            else:
                x = min(x + x*122//1000, z)
            ft = mr*SCALE*FAST_TH//10
            st = mr*SCALE*SLOW_TH//20
            bl = mr*SCALE
            if x >= ft:
                cnf += 1
                csl += 1
            elif x >= st:
                cnf = 0
                csl += 1
            else:
                cnf = 0
            if x <= bl:
                cnf = 0
                csl = 0
            if cnf >= 4:
                mr = x >> SHIFT
                cnf = 0
                csl = 0
                delays.append(i)
                break
            elif csl >= N:
                mr = x >> SHIFT
                cnf = 0
                csl = 0
                delays.append(i)
                break
            xus = x >> SHIFT
            if cnf == 0 and csl == 0:
                if xus < mr:
                    pd += 1
                    if pd >= PD_N:
                        mr = xus
                        pd = 0
                else:
                    pd = 0
        if len(delays) <= s:
            delays.append(max_rtts)
    return len([d for d in delays if d < max_rtts]), sorted(delays)

# ── H0 sweep ──
print("G3 SLOW THRESHOLD — H0 FALSE POSITIVE (200 seeds x 50k RTTs each)")
hdr = f"{'T_prop':>10} |" + "|".join(f"N={n:>2}" for n in NS)
print(hdr)
print("-" * len(hdr))
for tp in TPROPS:
    row = f"{tp:>6} us |"
    for n in NS:
        fp = run_h0(tp, n)
        row += f"{fp:>5.1f}%" if fp > 0 else "    0"
    print(row)
    sys.stdout.flush()

# ── H1 sweep ──
print("\nG3 SLOW — +5% DETECTION DELAY (median RTTs, 30 seeds)")
hdr = f"{'T_prop':>10} |" + "|".join(f"N={n:>2}" for n in NS[:6])
print(hdr)
print("-" * len(hdr))
for tp in TPROPS:
    row = f"{tp:>6} us |"
    for n in NS[:6]:
        ok, ds = run_h1(tp, 5, n)
        med = ds[len(ds)//2]
        row += f"{med:>5}"
    print(row)
    sys.stdout.flush()

# ── +3% sweep (key thresholds) ──
print("\nG3 SLOW — +3% DETECTION (N=3,4,5; 30 seeds)")
for n in [3, 4, 5]:
    print(f"N={n}: ", end="")
    for tp in [1000, 10000, 45000, 100000]:
        ok, ds = run_h1(tp, 3, n)
        med = ds[len(ds)//2] if ok>0 else "X"
        print(f"T={tp}:{med} ", end="")
    print()

print("\nDONE")
