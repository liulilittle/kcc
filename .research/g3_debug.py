#!/usr/bin/env python3

"""Debug: G3 slow counter distribution under H0."""
import random

SCALE, SHIFT = 1024, 10
PD_N = 3

def sim_one(tp, n_rtts=50000, seed=42):
    rng = random.Random(seed)
    mr = tp
    x = tp * SCALE
    cnf, csl, pd = 0, 0, 0
    jtr = max(1.0, tp / 100.0)
    max_csl = 0
    resets = 0
    incs = 0
    for _ in range(n_rtts):
        rtt = tp + rng.gauss(0, jtr)
        z = max(1, int(rtt)) * SCALE
        if z <= x:
            x = z
        else:
            x = min(x + x * 12 // 100, z)
        ft = mr * SCALE * 11 // 10
        st = mr * SCALE * 21 // 20
        bl = mr * SCALE
        prev_csl = csl
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
            if prev_csl > 0:
                resets += 1
        if cnf >= 3:
            mr = x >> SHIFT
            cnf = 0
            csl = 0
            return f"G3 FAST fired! mr={mr}"
        elif csl > max_csl:
            max_csl = csl
        if csl >= 4:
            return f"SLOW FIRED at {csl}"
        xus = x >> SHIFT
        if cnf == 0 and csl == 0:
            if xus < mr:
                pd += 1
                if pd >= PD_N:
                    mr = xus
                    pd = 0
            else:
                pd = 0
        incs += 1 if csl > 0 else 0
    return f"max_csl={max_csl} resets={resets} incs={incs}"

# Test several T_prop values
for tp in [100, 500, 1000, 5000, 10000]:
    results = [sim_one(tp, 50000, s) for s in range(50)]
    fired = sum(1 for r in results if "FIRED" in r or "SLOW" in r)
    maxes = [int(r.split("max_csl=")[1].split()[0]) for r in results if "max_csl=" in r]
    print(f"T={tp:>6}us: fired={fired}/50  max_csl_mean={sum(maxes)/len(maxes):.1f}  max_csl_max={max(maxes)}")

print("\nDONE")
