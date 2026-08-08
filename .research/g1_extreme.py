# KCC G1 staleness EXTREME test — 25us to 10s path change
# py -3 this.py
import math

SCALE = 1024
GROWTH_NUM = 122
GROWTH_DEN = 1000
G3_FAST_TH = 11  # 11/10 = 1.1x
G3_FAST_D = 10
G3_SLOW_TH = 21  # 21/20 = 1.05x  
G3_SLOW_D = 20
PD_NOISE_N = 95
PD_NOISE_D = 100

def geodesic_step(x_est, min_rtt_us, z_us, cnf, csl, pd):
    z = z_us * SCALE
    nu = z - x_est
    if nu <= 0:
        x_est = min(x_est, z)
    else:
        growth = x_est * GROWTH_NUM // GROWTH_DEN
        x_est = min(x_est + growth, z)

    ft = min_rtt_us * G3_FAST_TH * SCALE // G3_FAST_D
    st = min_rtt_us * G3_SLOW_TH * SCALE // G3_SLOW_D
    mr = min_rtt_us * SCALE

    if x_est >= ft: cnf += 1; csl += 1
    elif x_est >= st: cnf = 0; csl += 1
    else: cnf = 0
    if x_est <= mr: cnf = csl = 0

    g3_fired = False
    if cnf >= 3:
        min_rtt_us = x_est // SCALE; cnf = csl = 0; g3_fired = True
    elif csl >= 4:
        min_rtt_us = x_est // SCALE; cnf = csl = 0; g3_fired = True

    if cnf == 0 and csl == 0:
        if x_est < min_rtt_us * PD_NOISE_N * SCALE // PD_NOISE_D:
            pd += 1
        else: pd = 0
        if pd >= 3:
            min_rtt_us = x_est // SCALE; pd = 0; g3_fired = True

    return x_est, min_rtt_us, cnf, csl, pd, g3_fired

print("=" * 95)
print("KCC G1 STALENESS EXTREME TEST — 25us to 10s path change")
print("=" * 95)

for old_us, new_us, label in [
    (25, 10000000, "25us -> 10s (400000x)"),
    (100, 1000000, "100us -> 1s (10000x)"),
    (1000, 10000000, "1ms -> 10s (10000x)"),
    (35000, 10000000, "35ms -> 10s (286x)"),
    (100000, 10000000, "100ms -> 10s (100x)"),
    (500000, 10000000, "500ms -> 10s (20x)"),
]:
    x_est = old_us * SCALE
    min_rtt = old_us
    cnf = csl = pd = 0
    staleness = 0  # rounds since min_rtt last updated
    last_mr_update = 0
    
    # Run 200 rounds of geodesic
    convergence_round = None
    mr_update_rounds = []
    
    for rd in range(200):
        # Simulated RTT sample: after round 20, jump to new T_prop
        if rd < 20:
            z_us = old_us  # stable path
        else:
            z_us = new_us  # path changed
        
        old_min_rtt = min_rtt
        x_est, min_rtt, cnf, csl, pd, g3_fired = geodesic_step(x_est, min_rtt, z_us, cnf, csl, pd)
        
        if min_rtt != old_min_rtt:
            mr_update_rounds.append(rd)
            staleness = 0
            last_mr_update = rd
        else:
            staleness += 1
        
        # Staleness check: reset x_est if stuck
        if staleness >= 100:
            x_est = min_rtt * SCALE
            staleness = last_mr_update = rd  # fake update
        
        # Track convergence
        if convergence_round is None and min_rtt >= new_us * 0.9:
            convergence_round = rd - 20  # rounds after path change
    
    x_est_us = x_est // SCALE
    ratio = new_us / old_us
    predicted = math.ceil(math.log(ratio) / math.log(1.12))
    
    print(f"\n  {label}")
    print(f"    G2 predicted: {predicted} RTTs  |  Actual convergence: {convergence_round if convergence_round else 'NOT YET'} RTTs")
    print(f"    Final: x_est={x_est_us}us  min_rtt={min_rtt}us  staleness={staleness}")
    print(f"    G3 update rounds: {mr_update_rounds[:5]}{'...' if len(mr_update_rounds)>5 else ''}")
    
    if convergence_round:
        print(f"    Peak staleness during convergence: max={max(staleness_tracker) if 'staleness_tracker' in dir() else 'tracked per-round'}")

# Special: detailed trace for 25us->10s
print(f"\n{'='*95}")
print("DETAILED TRACE: 25us -> 10s (first 130 rounds after change)")
print(f"{'='*95}")
print(f"  {'Rnd':>4} {'x_est(us)':>12} {'mr(us)':>10} {'z(us)':>12} {'cnf':>4} {'csl':>4} {'stale':>6} {'Event'}")
print(f"  {'-'*4} {'-'*12} {'-'*10} {'-'*12} {'-'*4} {'-'*4} {'-'*6} {'-'*20}")

x_est = 25 * SCALE; min_rtt = 25; cnf = csl = pd = 0; stale = 0
for rd in range(150):
    z_us = 25 if rd < 20 else 10000000
    old_mr = min_rtt
    x_est, min_rtt, cnf, csl, pd, g3 = geodesic_step(x_est, min_rtt, z_us, cnf, csl, pd)
    if min_rtt != old_mr: stale = 0
    else: stale += 1
    if stale >= 100:
        x_est = min_rtt * SCALE; stale = 0
        event = "RESET!"
    elif g3:
        event = f"G3->mr={min_rtt}"
    elif rd == 20:
        event = "PATH CHANGE"
    else:
        event = ""
    
    if rd >= 18 and rd <= 30 or rd % 20 == 0 or g3 or stale >= 99:
        print(f"  {rd:>4} {x_est//SCALE:>12} {min_rtt:>10} {z_us:>12} {cnf:>4} {csl:>4} {stale:>6}  {event}")
