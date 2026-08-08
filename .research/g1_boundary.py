# KCC G1 staleness — EXHAUSTIVE boundary enumeration
# Goal: find optimal threshold that detects all real T_prop increases,
#       prevents deadlock, and doesn't false-trigger from noise.
# py -3 this.py
import math

GROWTH = 1.12  # G2 growth per RTT

print("=" * 95)
print("KCC G1 STALENESS — EXHAUSTIVE BOUNDARY ANALYSIS")
print("=" * 95)

# ============================================================
# 1. G2 convergence: given old/new T_prop ratio, how many RTTs?
# ============================================================
print("\n1. G2 RECOVERY TIME (x_est grows from old_T_prop to new_T_prop)")
print(f"   Formula: RTTs = ceil(log(new/old) / log({GROWTH}))")
print(f"   {'Old→New':>12} {'Factor':>8} {'RTTs':>6} {'@5ms':>8} {'@35ms':>8} {'@200ms':>8} {'@1s':>8}")
print(f"   {'-'*12} {'-'*8} {'-'*6} {'-'*8} {'-'*8} {'-'*8} {'-'*8}")

for old_ms in [1, 5, 10, 35, 100]:
    for factor in [1.25, 1.5, 2, 3, 5, 10, 50, 100]:
        new_ms = old_ms * factor
        if new_ms > 1000: continue  # skip unrealistic (1ms→100ms = 10x)
        rtts = math.ceil(math.log(factor) / math.log(GROWTH))
        label = f"{old_ms}→{new_ms:.0f}ms"
        t5 = rtts * old_ms * 0.001
        t35 = rtts * 0.035
        t200 = rtts * 0.200
        t1s = rtts * 1.0
        print(f"   {label:>12} {factor:>8.1f}x {rtts:>6} {t5:>7.3f}s {t35:>7.2f}s {t200:>7.1f}s {t1s:>7.0f}s")

# ============================================================
# 2. Deadlock scenarios — what if G1 NEVER fires?
# ============================================================
print("\n2. DEADLOCK SCENARIOS")
print("   Scenario A: Path permanently increases (e.g., routing change)")
print("     G1 never fires (all new RTTs > old x_est)")
print("     G2 grows x_est at 12.2%/RTT toward new T_prop")
print("     G3 detects when x_est > 1.05×min_rtt (3 consecutive)")
print("     Convergence: handled by G3 after ~ceil(log(new/old)/log(1.12)) RTTs")
print("     DEADLOCK? No — G2+G3 handle it.")
print()
print("   Scenario B: Continuous noise floor (always some queue, no clean samples)")
print("     G1 never fires (all z > x_est, even the lowest z has some queue)")
print("     G2 grows x_est, gets capped at z (which includes noise)")
print("     G3 fires if x_est > 1.05×min_rtt")
print("     x_est converges to noise floor, min_rtt inflates")
print("     DEADLOCK? x_est drifts up — T_prop overestimate. Need staleness reset!")
print()
print("   Scenario C: G1 fires sporadically (occasional clean ACK)")
print("     Every clean ACK: x_est drops to z=clean_T_prop")
print("     Between clean ACKs: G2 grows x_est, capped at noisy_z")
print("     x_est oscillates around true T_prop")
print("     DEADLOCK? No — periodic G1 corrections prevent drift.")
print()
print("   Scenario D: Path decreases (shorter route)")
print("     G1 fires immediately on first clean sample")
print("     DEADLOCK? No — G1 instant convergence.")

# ============================================================
# 3. Optimal staleness threshold
# ============================================================
print("\n3. OPTIMAL STALENESS THRESHOLD")
print("   Must be: ")
print("   a) Long enough to not false-trigger during normal G2 growth")
print("   b) Short enough to prevent T_prop inflation from Scenario B")
print("   c) Shorter than any real path-increase convergence time")
print()
print(f"   {'Threshold':>10} {'MinFactor':>12} {'Covers(ScenA)':>15} {'FalseRisk':>12} {'@5ms':>8} {'@500ms':>8}")
print(f"   {'-'*10} {'-'*12} {'-'*15} {'-'*12} {'-'*8} {'-'*8}")

for thresh in [8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 128]:
    # Min detectable path increase: what factor change is just barely caught?
    # If G1 fires 0 times in 'thresh' rounds, and G2 grows during all of them:
    # x_est would grow by (1.12)^thresh factor
    # But x_est is capped at z (the observed RTT)
    # So the effective limit: if new_T_prop / old_T_prop > (1.12)^thresh, G2 ALONE converges
    # If the ratio is smaller, G2 might not reach it in time
    
    # The staleness check fires when x_est <= min_rtt * SCALE for 'thresh' consecutive rounds
    # This means x_est hasn't grown above min_rtt at all
    # G2 grows x_est by 12.2%/RTT from min_rtt
    # After N rounds: x_est = min_rtt * (1.12)^N, capped at observed z
    # If z > min_rtt, x_est grows until capped
    # If z ≈ min_rtt (always), x_est stays at min_rtt (capped)
    # Staleness fires when NO z > min_rtt has been observed for 'thresh' rounds
    
    # For Scenario B: x_est is always at min_rtt (all z have at least some queue)
    # After 'thresh' rounds of this → staleness fires → resets x_est = min_rtt
    # Problem: min_rtt might also be stale
    # But! If we reset x_est to min_rtt, and min_rtt is also high from noise,
    # the next G3 firing will update min_rtt further up. This is the wrong direction!
    
    # Better: reset x_est to min_rtt * 0.95 (pull-down threshold)
    # This forces x_est back below min_rtt, giving G3 a chance to detect if min_rtt is inflated
    
    # Min factor: at what old→new ratio can G2 converge within 'thresh' rounds?
    min_factor = GROWTH ** thresh
    covers = "Yes" if min_factor >= 5 else "No"
    false_risk = "Low" if thresh >= 32 else "High" if thresh <= 16 else "Med"
    t5 = thresh * 0.005; t500 = thresh * 0.500
    print(f"   {thresh:>10} {min_factor:>12.0f}x {covers:>15} {false_risk:>12} {t5:>7.3f}s {t500:>7.0f}s")

# ============================================================
# 4. Path increase detection latency (G2+G3 vs staleness)
# ============================================================
print("\n4. DETECTION LATENCY: G2+G3 vs staleness reset")
print(f"   For a 5x path increase:")
g2_rtts = math.ceil(math.log(5) / math.log(GROWTH))
print(f"     G2 growth: {g2_rtts} RTTs to reach new T_prop")
print(f"     G3 detection: +3 RTTs (consecutive above 1.1x)")
print(f"     Total G2+G3: {g2_rtts + 3} RTTs")
print(f"     Staleness reset: fires at 64 RTTs — doesn't interfere")
print()
print(f"   For a 2x path increase:")
g2_rtts = math.ceil(math.log(2) / math.log(GROWTH))
print(f"     G2 growth: {g2_rtts} RTTs, G3: +3, Total: {g2_rtts + 3} RTTs")
print(f"     Staleness doesn't fire before G3 completes — no conflict")

# ============================================================
# 5. Final recommendation
# ============================================================
print(f"\n5. RECOMMENDATION: 64 RTTs")
print(f"   a) Long enough: 0.64s@10ms, 32s@500ms — won't false-trigger")
print(f"   b) Short enough: catches Scenario B within seconds")
print(f"   c) Doesn't conflict with G2+G3: G3 detects 5x increase in 18 RTTs << 64")
print(f"   d) Covers 1412x growth: all physically possible path changes")
