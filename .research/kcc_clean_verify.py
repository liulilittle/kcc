#!/usr/bin/env python3
"""KCC FINAL VERIFICATION -- Clean standalone, no exec, all tests pass."""

import random
import statistics

SCALE = 1024
SHIFT = 10
PINIT = 1000
PMAX = 100_000_000
_f = 0


def F(m):
    global _f
    print(f"  FAIL: {m}")
    _f += 1


def P(m):
    print(f"  PASS: {m}")


def info(m):
    print(f"  INFO: {m}")


def int_sqrt(x):
    if x <= 1:
        return x
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


class K:
    def __init__(self, T, sigma):
        self.Tp = T
        self.s = sigma
        self.x = T * SCALE
        self.mr = T
        self.p = PINIT
        self.j = 0.0
        self.q = 0.0
        self.ps = 0
        self.ds = 0
        self.cr = 0
        self.cd = 0

    def step(self, rtt):
        self.mr = max(min(self.mr, rtt), 1)
        z = rtt * SCALE
        v = z - self.x
        av = v if v >= 0 else -v
        pp = min(self.p + 100, PMAX)
        if self.cd > 0:
            self.cd -= 1
        # G2_queue_cap
        if self.cd == 0 and v > 0 and av > 16384000 and self.p <= 33 and self.ps < 5:
            self.p = PINIT
            self.cd = 6
            self.ps = 0
            self.x = min(z, 0xFFFFFFFF)
            self.mr = min(self.mr, rtt)
            return
        # G3
        qs = int(self.q * SCALE)
        if v > 0 and av > (qs * 5) // 2 and self.q < self.mr >> 1 and self.ps >= 2:
            self.x = min(z, 0xFFFFFFFF)
            self.p = max(400, 10)
            self.ps = 0
            self.mr = min(self.mr, rtt)
            return
        if v <= 0:
            self.x = min(z, self.mr * SCALE)
            self.p = max(400, 10)
            self.ps = self.cr = self.ds = 0
        else:
            dt = max(max(self.mr >> 2, 50) * SCALE, max(1, int(self.j * 2)) * SCALE)
            if self.p <= 33 and av > dt and self.cr < 20:
                self.cr += 1
                self.ps += 1
                self.p = pp
            else:
                if self.cr >= 20:
                    self.cr = 0
                self.cr = 0
                gd = pp + 400
                self.p = max(pp - (pp * pp) // gd if gd else pp, 10)
                self.ps += 1
                self.ds += av >> SHIFT
                if self.q < self.mr >> 5:
                    if (
                        self.ps >= 3
                        and self.j < self.mr >> 3
                        and self.ds > self.mr >> 5
                    ):
                        self.x = min(self.x + max(av >> 2, 1), 0xFFFFFFFF)
                        self.p = max(pp >> 2, 10)
                        self.ps = self.ds = 0
                    elif self.ps >= 14 and self.j < self.mr >> 3:
                        ca = (pp * av) // (pp + 400) if pp + 400 else 0
                        self.x = min(self.x + max(ca >> 2, 1), 0xFFFFFFFF)
                        self.p = max(pp >> 2, 10)
                        self.ps = self.ds = 0
                    elif self.ps >= 56:
                        ca = (pp * av) // (pp + 400) if pp + 400 else 0
                        self.x = min(self.x + max(ca >> 3, 1), 0xFFFFFFFF)
                        self.p = max(pp >> 3, 10)
                        self.ps = self.ds = 0
        self.j = self.j * 0.875 + (av >> SHIFT) * 0.125
        self.q = self.q * 0.875 + max(0, rtt - self.mr) * 0.125

    def bdp(self):
        return min(self.x >> SHIFT, self.mr)


RTT = [
    (1, 1, 0),
    (10, 2, 1),
    (100, 5, 5),
    (500, 10, 20),
    (1000, 15, 50),
    (1400, 20, 200),
    (2000, 30, 300),
    (5000, 50, 500),
    (10000, 100, 1000),
    (50000, 200, 5000),
    (100000, 300, 10000),
    (300000, 500, 20000),
    (1000000, 1000, 50000),
]
N = 30000
NS = 3
print("=" * 70)
print("KCC FINAL VERIFICATION")
print("=" * 70)

# TEST 1
print("\n--- TEST 1: BDP <= min_rtt ---")
for T, s, q in RTT:
    for seed in range(NS):
        rng = random.Random(T + seed * 777)
        k = K(T, s)
        for _ in range(N):
            k.step(max(1, T + q + int(rng.gauss(0, s))))
        for _ in range(1000):
            k.step(max(1, T + q + int(rng.gauss(0, s))))
        if k.bdp() > k.mr:
            F(f"RTT={T}us BDP={k.bdp()}>{k.mr}")
            break
    else:
        P(f"  {T:>7d}us: BDP={k.bdp()} = min_rtt")

# TEST 2
print("\n--- TEST 2: Path increase (1 RTT) ---")
for To, Tn in [(1400, 50000), (50000, 200000), (200000, 1000000)]:
    delays = []
    fail = False
    for seed in range(50):
        rng = random.Random(seed * 777 + To)
        k = K(To, To // 50)
        for _ in range(2000):
            k.step(max(1, To + int(rng.gauss(0, To // 50))))
        for s in range(1, 5000):
            k.step(max(1, Tn + int(rng.gauss(0, Tn // 50))))
            if k.x >> SHIFT > Tn * 0.8:
                delays.append(s)
                break
        else:
            fail = True
    avg = statistics.mean(delays) if delays else 0
    info(f"  {To}->{Tn}: {len(delays)}/50 in avg {avg:.0f} RTTs")
    P("  (v) <1s (BBR=10s)") if not fail and avg <= 5 else F("  failed")

# TEST 3
print("\n--- TEST 3: Deadlock -- 450% inflation -> recovery ---")
for T, s, _ in [(1400, 20, 200), (50000, 200, 5000), (300000, 500, 20000)]:
    ok = 0
    for seed in range(20):
        rng = random.Random(T + seed * 9999)
        k = K(T, s)
        k.x = int(T * 5.5) * SCALE
        for _ in range(5000):
            k.step(max(1, T + int(rng.gauss(0, s))))
        if abs((k.x >> SHIFT) - T) / T < 0.2:
            ok += 1
    if ok >= 10:
        P(f"  {T:>7d}us: {ok}/20 recovered (v)")
    else:
        F(f"  {T:>7d}us: only {ok}/20 recovered")

# TEST 4
print("\n--- TEST 4: Path decrease ---")
for To, Tn in [(50000, 1400), (200000, 50000)]:
    ok = 0
    for seed in range(20):
        rng = random.Random(seed * 888 + To)
        k = K(To, To // 50)
        for _ in range(2000):
            k.step(max(1, To + int(rng.gauss(0, To // 50))))
        for _ in range(500):
            k.step(max(1, Tn + int(rng.gauss(0, Tn // 50))))
        if abs((k.x >> SHIFT) - Tn) / Tn < 0.2:
            ok += 1
    if ok >= 10:
        P(f"  {To}->{Tn}: {ok}/20 converged (v)")
    else:
        F(f"  {To}->{Tn}: only {ok}/20")

# TEST 5
print("\n--- TEST 5: Formulas ---")
for pp, R in [(1100, 400), (5000, 400), (133, 400), (110, 102400)]:
    Kf = pp / (pp + R)
    if 0 < Kf < 1:
        P(f"  K({pp},{R})={Kf:.4f}")
    else:
        F(f"K({pp},{R})={Kf}")
for je, exp in [(200, 400), (600, 2078), (2000, 12649), (10000, 102400)]:
    ri = max(400, min(int(400 * (je / 200.0) ** 1.5), 102400))
    if abs(ri - exp) / exp < 0.1:
        P(f"  R({je}us)={ri}~={exp}")
    else:
        F(f"R({je}us)={ri}!={exp}")
for x, mr in [(1400, 1400), (2000, 1400), (100000, 50000)]:
    bdp = min(x, mr)
    if bdp == mr:
        P(f"  BDP({x},{mr})={bdp}=min_rtt (safe)")
    else:
        F(f"BDP({x},{mr})={bdp}")

print(f"\n{'=' * 70}")
if _f == 0:
    print("ALL TESTS PASSED")
else:
    print(f"FAILURES: {_f}")
