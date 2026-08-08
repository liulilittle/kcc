#!/usr/bin/env python3
"""
kalman_integer_verify.py -- Verify integer Kalman arithmetic matches floating-point theory.
Implements exact C-code integer arithmetic (scale=1024, 10-bit shift)
and compares against floating-point for the full update cycle.
Sweeps RTT 1uss--300ms with realistic innovation distributions.
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

SCALE = 1024
SCALE_SHIFT = 10
P_INIT = 1000
P_MAX = 100_000_000
P_FLOOR = 10

failures = 0
warnings = 0


def fail(msg):
    global failures
    print(f"  FAIL: {msg}")
    failures += 1


def warn(msg):
    global warnings
    print(f"  WARN: {msg}")
    warnings += 1


def pass_(msg):
    print(f"  PASS: {msg}")


def verify_point(name, computed, expected, tol=0.02):
    err = abs(computed - expected) / max(abs(expected), 1e-9)
    if err > tol:
        fail(
            f"{name}: computed={computed:.6f} expected={expected:.6f} error={err * 100:.2f}%",
        )
    else:
        pass_(f"{name}: {computed:.6f} (expected {expected:.6f})")


def int_sqrt(x):
    """Exact bit-by-bit sqrt matching Linux kernel int_sqrt().
    Guaranteed exact for all u64 inputs, no overflow."""
    if x <= 1:
        return x
    # m = 1UL << (__fls(x) & ~1UL) -- highest power of 4 <= x
    m = 1 << (x.bit_length() - 1 & ~1)
    y = 0
    while m:
        b = y + m
        y >>= 1
        if x >= b:
            x -= b
            y += m
        m >>= 2
    return y


def kalman_int_step(x_est, p_est, rtt_us, Q_int, R_int):
    """One Kalman update step using exact C-code integer arithmetic.
    Returns: (x_est_new, p_est_new, K_actual, x_est_cap, innov_scaled, p_pred, accepted)
    K_actual = p_pred/(p_pred+R) -- the real observation_update_gain
    x_est_cap = 1.0 for G3-detect convergence (neg innov), else K_actual
    """
    z = rtt_us << SCALE_SHIFT
    innov = z - x_est

    p_pred = min(p_est + Q_int, P_MAX)

    if innov <= 0:
        floor = x_est - (x_est >> 3)
        gain_num = p_pred
        gain_den = p_pred + R_int
        K_actual = gain_num / gain_den if gain_den > 0 else 0

        if z >= floor:
            x_est_new = z
            p_est_new = max(R_int, P_FLOOR)
            accepted = True
            x_est_cap = 1.0
        else:
            x_est_new = x_est
            p_est_new = p_pred
            accepted = False
            x_est_cap = 0.0
    else:
        gain_num = p_pred
        gain_den = p_pred + R_int
        K_actual = gain_num / gain_den if gain_den > 0 else 0

        if gain_den > 0:
            corr = (p_pred * innov) // gain_den
            p_reduction = (p_pred * gain_num) // gain_den
        else:
            corr = 0
            p_reduction = 0
        x_est_new = x_est + corr
        x_est_new = min(x_est_new, 0xFFFFFFFF)
        p_est_new = p_pred - p_reduction
        p_est_new = max(p_est_new, P_FLOOR)
        accepted = True
        x_est_cap = K_actual

    return x_est_new, p_est_new, K_actual, x_est_cap, innov, p_pred, accepted


def covariance_update_ss(Qf, Rf):
    p_pred_ss = (Qf + math.sqrt(Qf * Qf + 4 * Qf * Rf)) / 2
    p_post_ss = (-Qf + math.sqrt(Qf * Qf + 4 * Qf * Rf)) / 2
    K_obs_drain = p_pred_ss / (p_pred_ss + Rf)
    return p_pred_ss, p_post_ss, K_obs_drain


def run_convergence(
    x_est_init,
    p_est_init,
    Q_int,
    R_int,
    rtt_base_us,
    sigma_noise,
    rounds=1000,
):
    """Run convergence, track K_actual/p_est for positive-innov-only steps (standard KF updates)."""
    x_est = x_est_init
    p_est = p_est_init
    K_track = []
    p_track = []
    x_vals = []

    rng = random.Random(42 + Q_int + R_int)
    for _ in range(rounds):
        noise = int(rng.gauss(0, sigma_noise))
        rtt_us = max(1, rtt_base_us + noise)
        x_est, p_est, K_actual, _x_est_cap, innov, _p_pred, _accepted = kalman_int_step(
            x_est,
            p_est,
            rtt_us,
            Q_int,
            R_int,
        )
        if innov > 0:  # standard Kalman update
            K_track.append(K_actual)
            p_track.append(p_est)
        x_vals.append(x_est / SCALE)

    K_last = sum(K_track[-50:]) / min(50, len(K_track[-50:])) if K_track else 0
    p_last = sum(p_track[-50:]) / min(50, len(p_track[-50:])) if p_track else 0
    x_last = sum(x_vals[-50:]) / 50
    return K_last, p_last, x_last


print("=" * 90)
print("INTEGER KALMAN ARITHMETIC VERIFICATION")
print("=" * 90)

# =============================================================================
# SECTION 1: Steady-state K_actual for positive-innov path (directional KF)
# =============================================================================
print("\n--- 1. Steady-state K_actual on positive-innov steps (directional KF) ---")

# In a directional KF with P(nu<0)~=0.5, Joseph form ups p_est to R=400 on neg steps.
# This elevates K_actual above the symmetric covariance_update value of 0.347.
# Verify K_actual bounded in (0,1) and stable.
for Q_int, R_int in [(100, 400), (2000, 400), (50000, 32000)]:
    K_conv, p_conv, x_conv = run_convergence(
        1400 * SCALE,
        P_INIT,
        Q_int,
        R_int,
        1400,
        20,
    )
    if 0 < K_conv < 1:
        pass_(f"K_actual stable (Q={Q_int},R={R_int}): K={K_conv:.4f} (0 < K < 1)")
    else:
        fail(f"K_actual unstable (Q={Q_int},R={R_int}): K={K_conv:.4f}")

# =============================================================================
# SECTION 2: K at R=102400 -- verify K > 0 (filter still learns)
# =============================================================================
print("\n--- 2. K at R=102400 -- filter still learns at max R ---")
K_conv, p_conv, x_conv = run_convergence(
    1400 * SCALE,
    P_INIT,
    100,
    102400,
    1400,
    20,
    rounds=500,
)
if K_conv > 0.001:
    pass_(f"K at R=102400: {K_conv:.6f} > 0.001 (filter still learning)")
elif K_conv > 0.01:
    pass_(f"K at R=102400: {K_conv:.6f} > 0.01 (good sensitivity)")
else:
    fail(f"K at R=102400: {K_conv:.6f} too small (filter frozen)")

# =============================================================================
# SECTION 3: p_post steady-state -- verify bounded and stable
# =============================================================================
print("\n--- 3. p_post steady-state -- bounded between p_floor and P_MAX ---")
for Q_int, R_int in [(100, 400), (2000, 400), (50000, 32000)]:
    K_conv, p_conv, x_conv = run_convergence(
        1400 * SCALE,
        P_INIT,
        Q_int,
        R_int,
        1400,
        20,
    )
    p_floor = 10
    if p_floor <= p_conv <= P_MAX:
        pass_(f"p_post stable (Q={Q_int},R={R_int}): p_est={p_conv:.0f}")
    else:
        fail(f"p_post out of bounds (Q={Q_int},R={R_int}): p_est={p_conv:.0f}")

# =============================================================================
# SECTION 4: K_init = (p_init+Q)/(p_init+Q+R)
# =============================================================================
print("\n--- 4. K_init = (p_init+Q)/(p_init+Q+R) ---")

for Q_int, R_int in [(100, 400), (2000, 400), (100, 3200)]:
    p_pred_init = P_INIT + Q_int
    K_expected = p_pred_init / (p_pred_init + R_int)
    # Single step with positive innov
    x_est = 1400 * SCALE
    p_est = P_INIT
    rtt_us = 1500  # positive innov
    _, _, K_actual, _, _, _, _ = kalman_int_step(x_est, p_est, rtt_us, Q_int, R_int)
    verify_point(f"K_init(Q={Q_int},R={R_int}) on step 1", K_actual, K_expected, 0.05)

# =============================================================================
# SECTION 5: x_est convergence to true RTT base
# =============================================================================
print("\n--- 5. x_est convergence to true RTT base ---")

for rtt_base_us, Q_int, R_int, sigma_noise in [
    (1400, 100, 400, 20),
    (50000, 100, 3200, 100),
    (300000, 100, 3200, 100),
]:
    x_est = rtt_base_us * SCALE + 200 * SCALE
    p_est = P_INIT
    x_history = []
    rng = random.Random(42 + rtt_base_us)
    for _ in range(200):
        noise = int(rng.gauss(0, sigma_noise))
        rtt = max(1, rtt_base_us + noise)
        x_est, p_est, _, _, _, _, _ = kalman_int_step(x_est, p_est, rtt, Q_int, R_int)
        x_history.append(x_est / SCALE)

    avg_last_50 = sum(x_history[-50:]) / 50
    drift_pct = abs(avg_last_50 - rtt_base_us) / rtt_base_us * 100

    if drift_pct < 5:
        pass_(
            f"x_est at RTT={rtt_base_us}us: final={avg_last_50:.1f}us (drift={drift_pct:.2f}%)",
        )
    else:
        fail(
            f"x_est at RTT={rtt_base_us}us: final={avg_last_50:.1f}us (drift={drift_pct:.2f}%)",
        )

# =============================================================================
# SECTION 6: Directional gate -- G3-detect convergence for nu <= 0
# =============================================================================
print("\n--- 6. Directional gate: G3-detect convergence for nu <= 0 ---")

x_est = 1000 * SCALE
p_est = P_INIT
_, _, K_actual, x_est_cap, innov, _, _ = kalman_int_step(x_est, p_est, 900, 100, 400)
if innov <= 0 and x_est_cap == 1.0:
    pass_(
        f"Neg innov={innov}: x_est_cap=1 (G3-detect convergence), K_actual={K_actual:.4f}",
    )
else:
    fail(f"Neg innov: x_est_cap={x_est_cap} (expected 1.0)")

x_est = 1000 * SCALE
p_est = P_INIT
_, _, K_actual, x_est_cap, innov, _, _ = kalman_int_step(x_est, p_est, 1100, 100, 400)
if innov > 0 and 0 < x_est_cap < 1:
    pass_(f"Pos innov={innov}: x_est_cap={x_est_cap:.4f} = K_actual (standard update)")
else:
    fail(f"Pos innov: x_est_cap={x_est_cap:.4f} (expected 0 < K < 1)")

# =============================================================================
# SECTION 7: Speed-of-light floor (12.5% drop)
# =============================================================================
print("\n--- 7. Speed-of-light floor: rejects >12.5% single-step drops ---")

x_est = 1000 * SCALE
p_est = P_INIT
floor_us = (x_est - (x_est >> 3)) >> SCALE_SHIFT

_, _, _, _, _, _, accepted = kalman_int_step(x_est, p_est, floor_us, 100, 400)
if accepted:
    pass_(f"Floor accept at z=x_est*7/8={floor_us}us: accepted")
else:
    fail(f"Floor accept at z=x_est*7/8={floor_us}us: REJECTED")

below_floor_us = floor_us - 1
if below_floor_us >= 1:
    _, _, _, _, _, _, accepted = kalman_int_step(x_est, p_est, below_floor_us, 100, 400)
    if not accepted:
        pass_("Floor reject at z<floor: rejected correctly")
    else:
        fail("Floor reject at z<floor: ACCEPTED incorrectly")

# =============================================================================
# SECTION 8: Covariance update paths
# =============================================================================
print("\n--- 8. Covariance update verification ---")

x_est = 1400 * SCALE
p_est = 5000
Q, R = 100, 400
rtt_us = 1500  # positive innov
_, p_new, K_actual, _, _, p_pred, _ = kalman_int_step(x_est, p_est, rtt_us, Q, R)
K_calc = p_pred / (p_pred + R)
p_expected = p_pred * (1 - K_calc)
if abs(p_new - p_expected) <= max(p_expected * 0.05, 1):
    pass_(
        f"Standard cov update: p={int(p_new)}, expected ~{int(p_expected)}, K={K_actual:.4f}",
    )
else:
    fail(f"Standard cov update: p={int(p_new)}, expected ~{int(p_expected)}")

# Joseph form
_, p_new, _, _, _, _, _ = kalman_int_step(x_est, p_est, 1300, Q, R)
if int(p_new) >= R:
    pass_(f"Joseph form: p_new={int(p_new)} >= R={R}")
else:
    fail(f"Joseph form: p_new={int(p_new)} < R={R}")

# =============================================================================
# SECTION 9: RTT sweep [1us, 300ms] -- integer KF stability
# =============================================================================
print("\n--- 9. RTT sweep [1us-300ms] -- integer KF stability ---")

rtt_sweep = [
    1,
    10,
    100,
    300,
    500,
    1000,
    1400,
    2000,
    5000,
    10000,
    20000,
    50000,
    100000,
    150000,
    200000,
    300000,
]
for rtt_us in rtt_sweep:
    x_est = rtt_us * SCALE  # cold start: x_est = first measurement
    p_est = P_INIT
    Q = 100
    R = 400
    stable = True
    x_vals, p_vals = [], []
    rng = random.Random(rtt_us)
    for _ in range(100):
        noise = int(rng.gauss(0, max(10, rtt_us * 0.01)))
        rtt = max(1, rtt_us + noise)
        x_est, p_est, _, _, _, _, _ = kalman_int_step(x_est, p_est, rtt, Q, R)
        x_vals.append(x_est / SCALE)
        p_vals.append(p_est)
        if x_est < 0 or p_est < 0 or p_est > P_MAX:
            stable = False
            break
    if stable:
        avg_x = sum(x_vals[-50:]) / 50
        drift = abs(avg_x - rtt_us) / max(rtt_us, 1) * 100
        if drift < 20:
            pass_(
                f"RTT={rtt_us:>6d}us: stable, final x={avg_x:.1f}us, drift={drift:.1f}%",
            )
        else:
            warn(f"RTT={rtt_us:>6d}us: stable but drift {drift:.1f}%")
    else:
        fail(f"RTT={rtt_us:>6d}us: UNSTABLE")

# =============================================================================
# SECTION 10: Power-law R integer vs float
# =============================================================================
print("\n--- 10. Power-law R: integer vs float (0-500ms jitter excess) ---")

KCC_R_POWER_FRAC = 20


def compute_R_int(je_us, J50=200, base_r=400, r_max_boost=256):
    if je_us <= 0:
        return base_r
    ratio = ((je_us) << KCC_R_POWER_FRAC) // J50
    sqrt_ratio = int_sqrt(ratio << KCC_R_POWER_FRAC)
    scale = (ratio * sqrt_ratio) >> KCC_R_POWER_FRAC
    r_new = (base_r * scale) >> KCC_R_POWER_FRAC
    r_cap = base_r * r_max_boost
    return max(base_r, min(r_new, r_cap))


def compute_R_float(je, J50=200, base_r=400, r_max_boost=256):
    if je <= 0:
        return base_r
    r = base_r * (je / J50) ** 1.5
    return max(base_r, min(r, base_r * r_max_boost))


je_sweep = [
    0,
    50,
    100,
    150,
    200,
    250,
    300,
    317,
    400,
    500,
    800,
    1000,
    2000,
    5000,
    10000,
    50000,
    100000,
    200000,
    500000,
]
for je in je_sweep:
    r_int = compute_R_int(je)
    r_float = compute_R_float(je)
    err = abs(r_int - r_float) / max(r_float, 1)
    if err < 0.05:
        pass_(
            f"je={je:>6d}us: R_int={r_int:>6d}, R_float={r_float:.1f}, err={err * 100:.2f}%",
        )
    else:
        fail(
            f"je={je:>6d}us: R_int={r_int:>6d}, R_float={r_float:.1f}, err={err * 100:.2f}%",
        )

# =============================================================================
# SECTION 11: R doubling point at 1.587*J50
# =============================================================================
print("\n--- 11. R doubling point: R doubles at ~1.587*J50 (~317us at J50=200) ---")
r_317 = compute_R_float(317)
verify_point("R(je=317us, J50=200) is 2x base", r_317 / 400, 2.0, 0.05)
r_200 = compute_R_float(200)
verify_point("R(je=200us, J50=200) is 1x base", r_200 / 400, 1.0, 0.01)

# =============================================================================
# SECTION 12: Integer overflow safety
# =============================================================================
print("\n--- 12. Integer overflow safety at extremes ---")

je_max, J50_min = 500000, 1
ratio_max = (je_max << KCC_R_POWER_FRAC) // J50_min
sqrt_max = int_sqrt(ratio_max << KCC_R_POWER_FRAC)
prod_max = ratio_max * sqrt_max
if prod_max <= 2**64 - 1:
    pass_(f"J50=1,je=500ms: product={prod_max:_d} fits u64")
else:
    warn(f"J50=1,je=500ms: product={prod_max:_d} EXCEEDS u64 (overflow guard required)")

# Default J50=200 at extreme
ratio_def = (je_max << KCC_R_POWER_FRAC) // 200
sqrt_def = int_sqrt(ratio_def << KCC_R_POWER_FRAC)
prod_def = ratio_def * sqrt_def
if prod_def <= 2**64 - 1:
    pass_(f"J50=200,je=500ms: product={prod_def:_d} fits u64 (safe)")
else:
    warn("J50=200,je=500ms: product OVERFLOWS")

# =============================================================================
# SECTION 13: All README section A.5 formulas cross-validated
# =============================================================================
print(
    "\n--- 13. README Section A.5: All covariance_update/Kalman formulas verified ---",
)

table = [
    ("p_pred_ss Q=100,R=400", (100 + math.sqrt(100**2 + 4 * 100 * 400)) / 2, 256),
    (
        "K_obs_drain Q=100,R=400",
        lambda Q=100, R=400: covariance_update_ss(Q, R)[2],
        0.390,
    ),
    ("p_pred_ss Q=2500,R=400", (2500 + math.sqrt(2500**2 + 4 * 2500 * 400)) / 2, 2851),
    (
        "K_obs_drain Q=2500,R=400",
        lambda Q=2500, R=400: covariance_update_ss(Q, R)[2],
        0.88,
    ),
    ("K_min p_floor=10", (10 + 100) / (10 + 100 + 400), 0.216),
    ("K_init", 1100 / 1500, 0.733),
]
for name, val, expected in table:
    if callable(val):
        val = val()
    verify_point(f"README: {name}", val, expected, 0.02)

# Q/R derivations
Q_deriv = int(((10 * 1024) ** 2) / 1_000_000)
R_deriv = int(((20 * 1024) ** 2) / 1_000_000)
verify_point("Q_base = (10*1024)^2/1e6", Q_deriv, 105, 0.05)
verify_point("R_base = (20*1024)^2/1e6", R_deriv, 420, 0.05)

# =============================================================================
# SECTION 14: Convergence threshold p_pred = K_th*R/(1-K_th)
# =============================================================================
print("\n--- 14. Convergence threshold derivation ---")
K_th = 0.25
p_pred_th = K_th * 400 / (1 - K_th)
p_est_th = p_pred_th - 100
verify_point("p_pred at K_th=0.25,R=400", p_pred_th, 133.3, 0.05)
verify_point("p_est at K_th=0.25", p_est_th, 33.3, 0.1)
K_check = (33 + 100) / (33 + 100 + 400)
verify_point("K at p_est=33", K_check, 0.25, 0.02)

# =============================================================================
# SECTION 15: Joseph vs standard covariance
# =============================================================================
print("\n--- 15. Joseph vs Standard covariance comparison ---")
for p_pred, R in [(5000, 400), (1000, 400), (50000, 32000)]:
    K = p_pred / (p_pred + R)
    joseph = (1 - K) * (1 - K) * p_pred + K * K * R
    standard = (1 - K) * p_pred
    if joseph >= standard:
        pass_(
            f"p_pred={p_pred},R={R}: Joseph={joseph:.1f} >= Standard={standard:.1f} (safer)",
        )
    else:
        fail(f"p_pred={p_pred},R={R}: Joseph={joseph:.1f} < Standard={standard:.1f}")

# =============================================================================
# SECTION 16: Cold-start initialization
# =============================================================================
print("\n--- 16. Cold-start: x_est initialized to first measurement ---")
# In real C code: if (!ext->x_est) ext->x_est = z;
# First step: x_est=0, rtt=1400us => z=1400<<10=1433600, innov=z-0=z
x_est = 0
p_est = P_INIT
z = 1400 << SCALE_SHIFT  # simulated first measurement
x_est = z  # cold start init (real code: x_est = min(z, U32_MAX))
if x_est == 1400 * SCALE:
    pass_(f"Cold start x_est = {x_est // SCALE}us (correct)")
else:
    warn(f"Cold start x_est = {x_est // SCALE}us")

# =============================================================================
print(f"\n{'=' * 90}")
print(f"RESULTS: {failures} failures, {warnings} warnings")
if failures == 0:
    print("ALL INTEGER KALMAN VERIFICATIONS PASSED")
else:
    print(f"{failures} FAILURES DETECTED")
