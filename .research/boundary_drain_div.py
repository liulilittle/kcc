from __future__ import division
import random
import math

BBR_UNIT = 256
PG_MIN_VAL = BBR_UNIT // 64
PG_MAX_VAL = BBR_UNIT * 5 // 4
AI_N, AI_D = 1, 100
MD_N, MD_D = 1, 8
DRAIN_DECAY_NUM = 92
DRAIN_DECAY_DEN = 100
TARGET_DIV = 128
DRAIN_EXIT = 4
DRAIN_INTERVAL = 128

MODE_PROBE_BW = 1
MODE_DRAIN = 2

MSS = 1448
BW_M = 1260.0
BW_BPS = BW_M * 1e6
BD = BW_BPS / 8
T_PROP = 35000
BDP_BYTES = BD * T_PROP * 1e-6
BDP_PKTS = BDP_BYTES / MSS


class Flow:
    def __init__(self):
        self.pg = BBR_UNIT
        self.cwnd_g = BBR_UNIT
        self.mode = MODE_PROBE_BW
        self.ez = 0
        self.fpr = 0
        self.fpc = 0
        self.plr = 0
        self.bws = 0
        self.mbw = 0.0
        self.depg = 0
        self.drok = 0
        self.mr = T_PROP
        self.rtt_cnt = random.randint(0, 1023)
        self.drain_count = 0
        self.drain_rounds_this = 0

    def step(self, excess, bw_mbps, rd, drain_div):
        if bw_mbps > self.mbw:
            self.mbw = bw_mbps
            self.bws = 0
        else:
            self.bws += 1
        if self.fpc > 0:
            self.fpc -= 1
        tp = max(1, self.mr)
        T = tp // TARGET_DIV
        D = tp // drain_div

        def sq(p):
            return (p * p) // BBR_UNIT

        if self.mode == MODE_PROBE_BW:
            if excess < T:
                self.pg = min(self.pg + BBR_UNIT * AI_N // AI_D, PG_MAX_VAL)
            else:
                md = (self.pg * excess * MD_N) // (tp * MD_D)
                self.pg = max(self.pg - md, PG_MIN_VAL)
            self.rtt_cnt += 1
            self.cwnd_g = sq(self.pg)
            if excess >= D and self.fpc == 0:
                self.mode = MODE_DRAIN
                self.depg = 999999999
                self.drok = 0
                self.drain_count += 1
                self.drain_rounds_this = 0

        elif self.mode == MODE_DRAIN:
            self.drain_rounds_this += 1
            if self.depg == 999999999:
                self.pg = BBR_UNIT * 75 // 100
            else:
                self.pg = max(self.pg * DRAIN_DECAY_NUM // DRAIN_DECAY_DEN, PG_MIN_VAL)
            self.cwnd_g = sq(self.pg)
            if excess <= T:
                self.drok += 1
            elif excess < self.depg:
                self.drok += 1
            else:
                self.drok = 0
            self.depg = excess
            if self.drok >= DRAIN_EXIT:
                self.mode = MODE_PROBE_BW
                self.ez = 0


class BurstFlow:
    def __init__(self, start_rd, duration):
        self.start_rd = start_rd
        self.duration = duration
        self.cwnd_g = 2 * BBR_UNIT
        self.pg = BBR_UNIT

    def active(self, rd):
        return self.start_rd <= rd < self.start_rd + self.duration


def simulate(drain_div, seed=42, rounds=1000):
    rng = random.Random(seed)
    brtt = max(3000, T_PROP + rng.randint(-3000, 3000))
    flow = Flow()
    burst = BurstFlow(200, 100)
    stats = []
    for rd in range(rounds):
        has_burst = burst.active(rd)
        rtt_s = max(1e-9, brtt * 1e-6)
        for _ in range(8):
            pacing = BW_BPS * flow.pg / BBR_UNIT
            cwnd_r = (BDP_PKTS * flow.cwnd_g / BBR_UNIT) * MSS * 8 / rtt_s
            kcc_rate = min(pacing, cwnd_r)
            b_rate = 0.0
            if has_burst:
                b_pacing = BW_BPS * burst.pg / BBR_UNIT
                b_cwnd = (BDP_PKTS * burst.cwnd_g / BBR_UNIT) * MSS * 8 / rtt_s
                b_rate = min(b_pacing, b_cwnd)
            total_rate = min(kcc_rate + b_rate, BW_BPS)
            kcc_share = kcc_rate / max(kcc_rate + b_rate, 1e-9)
            total_inflight = (kcc_rate + b_rate) * rtt_s / 8 / MSS
            queue_bytes = max(0.0, total_inflight * MSS - BDP_BYTES)
            rtt_s = brtt * 1e-6 + queue_bytes / BD
        queue_us = queue_bytes / BD * 1e6
        excess = max(0.0, queue_us - brtt)
        flow.step(excess, 0.0, rd, drain_div)
        stats.append({
            'rd': rd, 'q_us': queue_us, 'excess': excess,
            'pg': flow.pg / BBR_UNIT, 'mode': flow.mode,
            'in_drain': 1 if flow.mode == MODE_DRAIN else 0,
            'has_burst': has_burst,
        })
    return stats


def main():
    print("=" * 90)
    print("BOUNDARY 3: KCC_EXCESS_DRAIN_DIV SWEEP")
    print("=" * 90)
    print("Sweeping drain_div: 8, 16, 32, 64, 128")
    print("Mode: 1 KCC flow + 1 burst flow (rounds 200-300)")
    print("Without burst: measure steady-state KCC-only behavior")
    print("With burst: measure DRAIN entry/exit timing")
    print("=" * 90)

    values = [8, 16, 32, 64, 128]

    print("\n%-12s %12s %12s %12s %12s %12s" % (
        "Drain_Div", "Qidle(us)", "Qburst(us)", "Drain_evts", "Avg_PG", "PG_nadir"))
    print("-" * 72)
    for v in values:
        stats = simulate(v, seed=42, rounds=1000)
        idle = [s for s in stats if not s['has_burst'] and s['rd'] < 200]
        burst = [s for s in stats if s['has_burst']]
        post = [s for s in stats if not s['has_burst'] and s['rd'] >= 300]
        post_early = [s for s in stats if not s['has_burst'] and 300 <= s['rd'] < 500]
        q_idle = sum(s['q_us'] for s in idle) / len(idle) if idle else 0
        q_burst = sum(s['q_us'] for s in burst) / len(burst) if burst else 0
        drain_total = sum(s['in_drain'] for s in stats)
        pg_avg = sum(s['pg'] for s in post_early) / len(post_early) if post_early else 0
        pg_nadir = min(s['pg'] for s in burst) if burst else 1.0
        print("%-12d %12.0f %12.0f %12d %12.4f %12.4f" % (v, q_idle, q_burst, drain_total, pg_avg, pg_nadir))

    print("\n--- DRAIN TRIGGER THRESHOLD ---")
    for v in values:
        print("  drain_div=%d: drain triggers when excess >= %d us (tprop/%d)" % (v, T_PROP // v, v))

    print("\n--- INTERPRETATION ---")
    print("Current=32: triggers at tp/32 (~%dus)." % (T_PROP // 32))
    print("When a burst flow causes excess > threshold, KCC enters DRAIN.")
    print("Higher drain_div = more sensitive = drains sooner but might drain too eagerly.")
    print("Lower drain_div = less sensitive = only drains during significant events.")


if __name__ == "__main__":
    main()
