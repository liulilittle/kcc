#!/usr/bin/env python3
"""
G3 3/4 vs 4/5 definitive false-positive audit.
Distinguishes: corrections (mr was BELOW tp, G3 restores it) vs
               false positives (mr EXCEEDS tp due to G3).

Kernel-matched 3-band running-min. All T_prop scales, multiple noise levels.
"""
import random, math
random.seed(42)

KCC_SCALE = 1024; G2_N, G2_D = 122, 1000
G3_FN, G3_FD = 11, 10; G3_SN, G3_SD = 21, 20
STICKY_N, STICKY_D = 75, 100; FF_DIV = 4; FF_CNT = 5; BIT3 = 7
PD_N, PD_D = 95, 100; MS = 5; STALE = 128; RTT_MIN = 1

T_PROPS = [500, 1000, 5000, 10000, 50000, 100000, 500000, 1000000]
NOISES = [0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 15.0, 20.0]
MAX_ACKS = 100000
TRIALS = 5

def sim(tp, np, fc, sc, ma=MAX_ACKS):
    """Returns (g3_commits, g3_inflated, mr_above_tp_count, final_mr, max_mr, min_mr)."""
    mr = tp; xe = tp * KCC_SCALE; cc = csc = rc = mruc = mrffc = 0
    sigma = tp * np / 100.0
    commits = 0
    inflated_commits = 0   # commits that raised mr > tp * 1.01
    mr_above_tp = 0         # ACKs where mr > tp * 1.01
    max_mr = mr
    min_mr = mr

    for a in range(ma):
        ns = random.gauss(0, sigma)
        rtt = int(max(tp + ns, RTT_MIN))
        z = rtt * KCC_SCALE

        inno = z - xe
        if inno <= 0: xe = min(xe, z)
        else: xe = min(xe + xe * G2_N // G2_D, z)

        if rc - mruc >= STALE:
            ms_ = mr * KCC_SCALE
            if xe <= ms_ * G3_FN // G3_FD:
                xe = ms_ * PD_N // PD_D
                mruc = rc

        rd = (a > 0 and a % 20 == 0)
        if rd: rc += 1

        tf = mr * KCC_SCALE * G3_FN // G3_FD
        ts = mr * KCC_SCALE * G3_SN // G3_SD
        bl = mr * KCC_SCALE

        if xe >= tf: cc = min(cc+1,255); csc = min(csc+1,255)
        elif xe >= ts: cc = 0; csc = min(csc+1,255)
        else: cc = 0
        if xe <= bl: cc = 0; csc = 0

        # G3 commit
        commit = False
        if cc >= fc or csc >= sc:
            old_mr = mr
            mr = max(xe // KCC_SCALE, RTT_MIN)
            cc = csc = 0
            mruc = rc
            commits += 1
            commit = True
            # Was this commit inflationary (mr above true tp)?
            if mr > tp * 1.01 and old_mr <= tp * 1.01:
                inflated_commits += 1

        # 3-band running-min
        if not commit and cc == 0 and csc == 0 and rtt <= mr:
            rtc = max(rtt, RTT_MIN)
            if rtc < mr * STICKY_N // STICKY_D:
                if rtc < mr // FF_DIV: mr = rtc; mrffc = 0
                else: mrffc = min(mrffc+1, BIT3)
                if mrffc >= FF_CNT: mr = rtc; mrffc = 0
                elif rd: mr = max(RTT_MIN, mr * STICKY_N // STICKY_D)
            else: mr = rtc; mrffc = 0

        if sc >= MS and xe:
            k = xe // KCC_SCALE
            if k < mr and k < mr * PD_N // PD_D: mr = k; mruc = rc

        max_mr = max(max_mr, mr)
        min_mr = min(min_mr, mr)
        if mr > tp * 1.01:
            mr_above_tp += 1

    return commits, inflated_commits, mr_above_tp, mr, max_mr, min_mr


print("=" * 130)
print("G3 FALSE-POSITIVE AUDIT: 3/4 vs 4/5")
print("Key metric: 'INFLATED' = G3 commits that pushed mr above true T_prop")
print("(These are the only TRUE false positives — mr overshooting the baseline)")
print("'Corrections' = G3 pulls mr back up after running-min drifted it too low")
print("Kernel-matched 3-band running-min. %d ACKs each, %d trials." % (MAX_ACKS, TRIALS))
print("=" * 130)

# Table header
hdr = "T_prop    Noise    3/4:Cmts 3/4:Inf  3/4:Above 3/4:Final  3/4:MrMax  " \
      "4/5:Cmts 4/5:Inf  4/5:Above 4/5:Final  4/5:MrMax   Verdict"
print(hdr)
print("-" * 130)

global_fp34 = global_fp45 = 0
global_scenarios = 0

for tp in T_PROPS:
    for n in NOISES:
        c34_sum = i34_sum = a34_sum = mr34_sum = mx34_sum = 0
        c45_sum = i45_sum = a45_sum = mr45_sum = mx45_sum = 0

        for t in range(TRIALS):
            c, i, a, mr, mx, _ = sim(tp, n, 3, 4)
            c34_sum += c; i34_sum += i; a34_sum += a; mr34_sum += mr; mx34_sum += mx
            c, i, a, mr, mx, _ = sim(tp, n, 4, 5)
            c45_sum += c; i45_sum += i; a45_sum += a; mr45_sum += mr; mx45_sum += mx

        c34 = c34_sum / TRIALS; i34 = i34_sum / TRIALS; a34 = a34_sum / TRIALS
        mr34 = mr34_sum / TRIALS; mx34 = mx34_sum / TRIALS
        c45 = c45_sum / TRIALS; i45 = i45_sum / TRIALS; a45 = a45_sum / TRIALS
        mr45 = mr45_sum / TRIALS; mx45 = mx45_sum / TRIALS

        # Verdict: does this scenario have true false positives?
        fp34_flag = "FP!" if i34 > 0 else ""
        fp45_flag = "FP!" if i45 > 0 else ""
        if i34 > 0: global_fp34 += 1
        if i45 > 0: global_fp45 += 1
        global_scenarios += 1

        # Only print if noise is interesting or any FP
        if n in [0.5, 1.0, 2.0, 5.0, 10.0, 15.0] or i34 > 0 or i45 > 0:
            line = "%7d %6.1f%%  %7.0f %7.0f %8.0f %8.0fus %8.0fus  " \
                   "%7.0f %7.0f %8.0f %8.0fus %8.0fus  %s/%s" % (
                       tp, n, c34, i34, a34, mr34, mx34,
                       c45, i45, a45, mr45, mx45,
                       fp34_flag or "OK", fp45_flag or "OK")
            print(line)

print("=" * 130)
print("SUMMARY: True false-positive scenarios (mr inflated above tp):")
print("  3/4: %d / %d scenarios (%.1f%%)" % (global_fp34, global_scenarios,
      100*global_fp34/global_scenarios if global_scenarios else 0))
print("  4/5: %d / %d scenarios (%.1f%%)" % (global_fp45, global_scenarios,
      100*global_fp45/global_scenarios if global_scenarios else 0))
print()
print("NOTE: G3 'commits' at low noise are CORRECTIONS, not false positives.")
print("Running-min (band 3: rtt in [75%%, 100%%]) continuously drags mr DOWN.")
print("G3 restores mr to the true T_prop — this is the DESIGN INTENT.")
print("A TRUE false positive requires mr to EXCEED true T_prop by >1%%.")
print()
print("CONCLUSION:")
if global_fp34 == 0 and global_fp45 == 0:
    print("  BOTH 3/4 AND 4/5 HAVE ZERO TRUE FALSE POSITIVES at all tested noise levels.")
    print("  The 'commits' seen in raw counts are corrective (mr was below tp, G3 restored it).")
elif global_fp45 == 0 and global_fp34 > 0:
    print("  4/5 ELIMINATES the false positives that 3/4 exhibits.")
elif global_fp45 < global_fp34:
    print("  4/5 REDUCES false positives compared to 3/4.")
else:
    print("  Both thresholds produce false positives at high noise.")
