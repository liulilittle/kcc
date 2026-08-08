# Quick debug: P-controller single-seed test
import random

MSS = 1448
BW_Mbps = 1260.0
BW_bps = BW_Mbps * 1e6
BD_BYTE_PER_S = BW_bps / 8
T_PROP_US = 35000
T_PROP_S = T_PROP_US * 1e-6
BDP_bytes = BW_bps * T_PROP_S / 8
BDP_pkts = BDP_bytes / MSS

BBR_UNIT = 256
CWND_GAIN = BBR_UNIT * 2
CYCLE = [int(BBR_UNIT * 1.25), int(BBR_UNIT * 0.75)] + [BBR_UNIT] * 6
GAIN_MIN = int(BBR_UNIT * 0.75)
GAIN_MAX = int(BBR_UNIT * 1.25)

P_TARGET_SCALED = 0
P_SLOPE_NUM = 1
P_SLOPE_DEN = 4
P_ADJUST_MAX = BBR_UNIT // 20

def p_control(tprop_us, round_qdelay_us):
    excess_us = max(0, int(round_qdelay_us) - int(tprop_us))
    pressure_scaled = (excess_us << 10) // max(1, int(tprop_us))
    error = P_TARGET_SCALED - pressure_scaled
    adjust = (error * P_SLOPE_NUM) // P_SLOPE_DEN
    adjust = max(-P_ADJUST_MAX, min(adjust, P_ADJUST_MAX))
    result = max(GAIN_MIN, min(BBR_UNIT + adjust, GAIN_MAX))
    print(f"  p_ctrl: qd={round_qdelay_us} excess={excess_us} press={pressure_scaled} "
          f"err={error} adj={adjust} gain={result}/{BBR_UNIT}={result/BBR_UNIT:.4f}")
    return result

# Compute natural BP queue at 3 flows with cwn=2x
print("Natural BP queue computation:")
print(f"  3 flows * 2x BP * 1.0 gain = 6x BP inflight")
print(f"  Queue = (6 - 1) * BP = 5 * 35ms = {5*35}ms")
print(f"  queue_us = {5*T_PROP_US}")

print("\nP-controller test:")
p_control(T_PROP_US, 5*T_PROP_US)  # 175ms queue, 35ms T_prop

print("\nP-controller transfer for 3-flow case:")
for gain_factor in [1.25, 1.0, 0.953, 0.75]:
    inflight_total = 3 * BDP_pkts * 2 * gain_factor
    queue_bytes = inflight_total * MSS - BDP_bytes
    queue_us = max(0, queue_bytes / BD_BYTE_PER_S * 1e6)
    print(f"  gain={gain_factor:.3f}: queue={queue_us:.0f}us  p_ctrl->", end='')
    p_control(T_PROP_US, queue_us)
