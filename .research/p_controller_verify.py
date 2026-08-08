# KCC 2.0 Continuous P-controller verification
# pressure = qdelay / T_prop, target = 1/128, adjust = error * SLOPE_NUM / SLOPE_DEN
# py -3 this.py
import math, sys

BBR_UNIT = 256
P_TARGET_SCALED = 8     # 1/128 << 10
P_SLOPE_NUM = 1
P_SLOPE_DEN = 4
P_ADJUST_MAX = BBR_UNIT // 20  # ±5% = ±12.8 → 12
GAIN_MIN = BBR_UNIT * 75 // 100  # 192 = 0.75x
GAIN_MAX = BBR_UNIT * 125 // 100 # 320 = 1.25x

def p_controller(tprop_us, qdelay_us):
    """Compute pacing_gain from queue pressure (all integer, matches C code)."""
    if tprop_us == 0:
        return BBR_UNIT
    pressure_scaled = (qdelay_us << 10) // tprop_us
    error = P_TARGET_SCALED - pressure_scaled
    adjust = (error * P_SLOPE_NUM) // P_SLOPE_DEN
    adjust = max(-P_ADJUST_MAX, min(adjust, P_ADJUST_MAX))
    pgain = max(GAIN_MIN, min(BBR_UNIT + adjust, GAIN_MAX))
    return pgain

def gain_to_ratio(g): return g / BBR_UNIT

# Test 1: Parameter sweep
print("=" * 70)
print("P-Controller Transfer Function")
print(f"  target = 1/128 = {P_TARGET_SCALED}/1024  (Q10)")
print(f"  slope = {P_SLOPE_NUM}/{P_SLOPE_DEN} = {P_SLOPE_NUM/P_SLOPE_DEN:.3f}")
print(f"  clamp = ±{P_ADJUST_MAX} BBR_UNIT = ±{gain_to_ratio(P_ADJUST_MAX)*100:.1f}%")
print(f"  range = [{gain_to_ratio(GAIN_MIN):.2f}, {gain_to_ratio(GAIN_MAX):.2f}]")
print("=" * 70)

for tprop_ms in [5, 10, 35, 100, 200]:
    tprop = tprop_ms * 1000
    print(f"\n  T_prop = {tprop_ms}ms ({tprop}us):")
    print(f"  {'qdelay(us)':>10} {'pressure':>10} {'error':>8} {'adjust':>8} {'gain':>8} {'ratio':>8}")
    print(f"  {'-'*10} {'-'*10} {'-'*8} {'-'*8} {'-'*8} {'-'*8}")

    for fraction in [0, 1/1024, 1/512, 1/256, 1/128, 1/64, 1/32, 1/16, 1/8, 1/4, 1/2, 1.0, 2.0]:
        qd = int(tprop * fraction)
        pg = p_controller(tprop, qd)
        press = qd / tprop if tprop else 0
        er = P_TARGET_SCALED - ((qd << 10) // max(tprop, 1))
        adj = max(-P_ADJUST_MAX, min(er * P_SLOPE_NUM // P_SLOPE_DEN, P_ADJUST_MAX))
        print(f"  {qd:>10} {press:>10.4f} {er:>8} {adj:>8} {pg:>8} {pg/BBR_UNIT:>8.4f}")

# Test 2: Dynamic simulation
print("\n" + "=" * 70)
print("Dynamic Simulation: 35ms path, flow competes with cross-traffic")
print("=" * 70)

import random
rng = random.Random(42)

T_PROP_US = 35000
BDP_QUEUE = T_PROP_US  # natural BDP queue from cwnd_gain=2

# Simulate 500 rounds, cross-traffic enters at round 100, leaves at round 400
pacing_gain = BBR_UNIT
n_rounds = 500
history = []

for rd in range(n_rounds):
    # Cross-traffic effect on queue
    if 100 <= rd < 300:
        cross_factor = 2.0  # another flow joins → queue doubles
    elif 300 <= rd < 400:
        cross_factor = 3.0  # two flows → queue triples
    else:
        cross_factor = 1.0

    # Actual queue (ground truth)
    true_queue = BDP_QUEUE * cross_factor * (pacing_gain / BBR_UNIT)

    # Per-round min measurement (with small noise)
    noise = rng.uniform(0, 30)  # 0-30us noise floor
    measured_qdelay = true_queue + noise

    # P-controller
    pg = p_controller(T_PROP_US, int(measured_qdelay))
    pacing_gain = pg

    # cwnd_gain=2 produces natural queue = pacing_gain * BDP
    delivered_rate = BBR_UNIT * pacing_gain / BBR_UNIT  # normalized

    history.append((rd, true_queue, measured_qdelay, pacing_gain / BBR_UNIT, cross_factor))

# Print phase summaries
def phase_stats(start, end, label):
    rows = history[start:end]
    if not rows:
        return
    avg_queue = sum(r[1] for r in rows) / len(rows)
    avg_gain = sum(r[3] for r in rows) / len(rows)
    last_gain = rows[-1][3]
    print(f"  {label:<20} avg_queue={avg_queue:>8.0f}us  avg_gain={avg_gain:.4f}  final_gain={last_gain:.4f}")

print("\nPhase Analysis:")
phase_stats(0, 99, "alone (0-99)")
phase_stats(100, 199, "1 cross (100-199)")
phase_stats(200, 299, "1 cross steady")
phase_stats(300, 399, "2 cross (300-399)")
phase_stats(400, 499, "alone again (400-499)")

# Test 3: Step response
print("\n" + "=" * 70)
print("Step Response: queue jumps from 0 to 2×BDP, controller response")
print("=" * 70)

pg = BBR_UNIT
response = []
tprop = T_PROP_US

# Queue step: 0 -> 2x BDP at round 20, then back to 0 at round 40
for rd in range(60):
    if rd < 20:
        qd = 0
    elif rd < 40:
        qd = tprop * 2
    else:
        qd = 0

    measured = qd + random.uniform(0, 25)
    pg = p_controller(tprop, int(measured))
    response.append((rd, qd, pg / BBR_UNIT))

print(f"  {'Round':>5} {'Queue(us)':>10} {'Gain':>8}")
for rd, qd, gr in response:
    bar = '+' * max(0, int((gr - 1.0) * 400)) if gr >= 1.0 else '-' * max(0, int((1.0 - gr) * 400))
    print(f"  {rd:>5} {qd:>10} {gr:>8.4f} {bar[:30]}")

# Test 4: Sensitivity to slope parameter
print("\n" + "=" * 70)
print("Slope Sensitivity: different SLOPE_NUM values at T_prop=35ms")
print("=" * 70)
for slope_num in [1, 2, 4, 8]:
    def test_slope(qd, sn=slope_num):
        pressure_scaled = (qd << 10) // T_PROP_US
        error = P_TARGET_SCALED - pressure_scaled
        adjust = (error * sn) // P_SLOPE_DEN
        adjust = max(-P_ADJUST_MAX, min(adjust, P_ADJUST_MAX))
        return max(GAIN_MIN, min(BBR_UNIT + adjust, GAIN_MAX))

    print(f"\n  SLOPE_NUM={slope_num}, SLOPE_DEN={P_SLOPE_DEN} (slope={slope_num/P_SLOPE_DEN:.2f}):")
    print(f"  {'qdelay':>10} {'gain':>8} {'ratio':>8}")
    for frac in [0, 1/256, 1/128, 1/64, 1/32, 1/16, 1/8]:
        qd = int(T_PROP_US * frac)
        pg = test_slope(qd)
        print(f"  {qd:>10} {pg:>8} {pg/BBR_UNIT:>8.4f}")
