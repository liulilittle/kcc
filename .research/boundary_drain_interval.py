from __future__ import division
import random
import math

BBR_UNIT = 256
# Note: C code uses 0.75x (192), but existing Python sims use 4 (0.0156x).
# Using 4 for proper multi-flow dynamics. See boundary_pg.py for PG_MIN/PG_MAX validation.
PG_MIN_VAL = BBR_UNIT // 64
PG_MAX_VAL = BBR_UNIT * 5 // 4
AI_N, AI_D = 1, 100
MD_N, MD_D = 1, 8
DRAIN_DECAY_NUM = 92
DRAIN_DECAY_DEN = 100
TARGET_DIV = 128
DRAIN_DIV = 32
DRAIN_EXIT = 4

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
    def __init__(self, floor):
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
        self.floor = floor
        self.mr = T_PROP
        self.rtt_cnt = random.randint(0, 1023)

    def step(self, excess, bw_mbps, rd, drain_interval):
        if bw_mbps > self.mbw:
            self.mbw = bw_mbps
            self.bws = 0
        else:
            self.bws += 1
        if self.fpc > 0:
            self.fpc -= 1
        tp = max(1, self.mr)
        T = tp // TARGET_DIV
        D = tp // DRAIN_DIV

        def sq(p):
            return (p * p) // BBR_UNIT

        if self.mode == MODE_PROBE_BW:
            if excess < T:
                self.pg = min(self.pg + BBR_UNIT * AI_N // AI_D, PG_MAX_VAL)
            else:
                md = (self.pg * excess * MD_N) // (tp * MD_D)
                self.pg = max(self.pg - md, PG_MIN_VAL)
            if drain_interval > 0 and (self.rtt_cnt % drain_interval) == 0:
                self.pg = PG_MIN_VAL
            self.rtt_cnt += 1
            self.cwnd_g = max(sq(self.pg), self.floor)
            if excess >= D and self.fpc == 0:
                self.mode = MODE_DRAIN
                self.depg = 999999999
                self.drok = 0

        elif self.mode == MODE_DRAIN:
            if self.depg == 999999999:
                self.pg = BBR_UNIT * 75 // 100
            else:
                self.pg = max(self.pg * DRAIN_DECAY_NUM // DRAIN_DECAY_DEN, PG_MIN_VAL)
            self.cwnd_g = max(self.pg, self.floor)
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


def simulate(nf, drain_interval, floor, seed=42, rounds=800):
    rng = random.Random(seed)
    brtts = [max(3000, T_PROP + rng.randint(-1000, 1000)) for _ in range(nf)]
    flows = [Flow(floor) for _ in range(nf)]
    stats = []
    for rd in range(rounds):
        total_pg = sum(f.pg for f in flows)
        avg_tp = sum(brtts) // nf
        rtt_s = max(1e-9, avg_tp * 1e-6)
        for _ in range(8):
            total_rate = 0.0
            total_inflight = 0.0
            for f in flows:
                pacing = BW_BPS * f.pg / BBR_UNIT
                cwnd_r = (BDP_PKTS * f.cwnd_g / BBR_UNIT) * MSS * 8 / rtt_s
                rate = min(pacing, cwnd_r)
                total_rate += rate
                total_inflight += rate * rtt_s / 8 / MSS
            total_rate = min(total_rate, BW_BPS)
            queue_bytes = max(0.0, total_inflight * MSS - BDP_BYTES)
            rtt_s = avg_tp * 1e-6 + queue_bytes / BD
        queue_us = queue_bytes / BD * 1e6
        excess = max(0.0, queue_us - avg_tp)
        frates = [BW_BPS * f.pg / total_pg / 1e6 if total_pg > 0 else 0.0 for f in flows]
        for f in flows:
            f.step(excess, 0.0, rd, drain_interval)
        if rd >= 200:
            pgs = [f.pg / BBR_UNIT for f in flows]
            pm = sum(pgs) / nf
            stats.append({'q_us': queue_us, 'pg_mean': pm})
    return stats


def main():
    print("=" * 90)
    print("BOUNDARY 1: KCC_PERIODIC_DRAIN_INTERVAL SWEEP")
    print("=" * 90)
    print("Sweeping drain interval: 64, 128, 256, 512, 1024 (no drain)")
    print("Modes: ECO (floor=1.0x), TURBO (floor=1.88x)")
    print("N=4 flows, T_prop=35ms, BW=1260Mbps")
    print("=" * 90)

    intervals = [64, 128, 256, 512, 1024]
    modes = [
        ("ECO", BBR_UNIT),
        ("TURBO", BBR_UNIT * 188 // 100),
    ]

    for mode_name, floor in modes:
        print("\n--- %s mode (floor=%.2fx) ---" % (mode_name, floor / float(BBR_UNIT)))
        print("%-12s %12s %14s %12s %12s" % ("Interval", "Q_avg(us)", "PG_avg", "Goodput(Mbps)", "Goodput%%"))
        print("-" * 62)
        for interval in intervals:
            stats = simulate(4, interval, floor, seed=42, rounds=800)
            q_avg = sum(s['q_us'] for s in stats) / len(stats)
            pg_avg = sum(s['pg_mean'] for s in stats) / len(stats)
            util = 100.0 * BDP_BYTES / (BDP_BYTES + q_avg / 1e6 * BD)
            drain_label = "none" if interval == 1024 else str(interval)
            print("%-12s %12.0f %14.4f %12.0f %12.1f" % (drain_label, q_avg, pg_avg, BW_M * util / 100.0, util))

    print("\n--- RECOMMENDATION ---")
    print("Current: 128 (every 128 rounds = ~4.5s at 35ms RTT)")


if __name__ == "__main__":
    main()
