from __future__ import division
import random
import math

BBR_UNIT = 256
PG_MIN_VAL = BBR_UNIT // 64
PG_MAX_VAL = BBR_UNIT * 5 // 4
AI_N, AI_D = 1, 100
MD_N, MD_D = 1, 8
TARGET_DIV = 128
DRAIN_DIV = 32
DRAIN_EXIT = 4
DRAIN_INTERVAL = 128
FP_COOLDOWN = 8

STARTUP_GAIN = BBR_UNIT * 289 // 100

MODE_STARTUP = 0
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
        self.pg = STARTUP_GAIN
        self.cwnd_g = STARTUP_GAIN
        self.mode = MODE_STARTUP
        self.ez = 0
        self.fpr = 0
        self.fpc = 0
        self.plr = 0
        self.bws = 0
        self.mbw = 0.0
        self.depg = 0
        self.drok = 0
        self.mr = T_PROP
        self.probe_round = 1
        self.probe_cooldown = 0
        self.drain_rounds = 0
        self.min_pg_reached = 1.0
        self.max_excess_in_drain = 0.0
        self.excess_at_exit = 0.0

    def step(self, excess, bw_mbps, rd, drain_decay_num):
        if bw_mbps > self.mbw:
            self.mbw = bw_mbps
            self.bws = 0
        else:
            self.bws += 1
        if self.fpc > 0:
            self.fpc -= 1
        if self.probe_cooldown > 0:
            self.probe_cooldown -= 1
        tp = max(1, self.mr)
        T = tp // TARGET_DIV
        D = tp // DRAIN_DIV

        def sq(p):
            return (p * p) // BBR_UNIT

        if self.mode == MODE_STARTUP:
            self.probe_round += 1
            if self.probe_round > 7:
                self.probe_round = 7
            cg = STARTUP_GAIN
            for _ in range(self.probe_round):
                cg = cg * 125 // 100
            self.cwnd_g = min(cg, BBR_UNIT * 2)
            if self.probe_cooldown == 0:
                self.pg = min(cg, STARTUP_GAIN)
            else:
                self.pg = min(cg, PG_MAX_VAL)
            if excess > T:
                self.probe_cooldown = FP_COOLDOWN
                self.mode = MODE_DRAIN
                self.depg = 999999999
                self.drok = 0
            elif excess >= D:
                self.mode = MODE_DRAIN
                self.probe_cooldown = FP_COOLDOWN
                self.depg = 999999999
                self.drok = 0

        elif self.mode == MODE_PROBE_BW:
            if excess < T:
                self.pg = min(self.pg + BBR_UNIT * AI_N // AI_D, PG_MAX_VAL)
            else:
                md = (self.pg * excess * MD_N) // (tp * MD_D)
                self.pg = max(self.pg - md, PG_MIN_VAL)
            if DRAIN_INTERVAL > 0 and (rd % DRAIN_INTERVAL) == 0:
                self.pg = PG_MIN_VAL
            self.cwnd_g = sq(self.pg)
            if excess >= D and self.fpc == 0:
                self.mode = MODE_DRAIN
                self.depg = 999999999
                self.drok = 0

        elif self.mode == MODE_DRAIN:
            if self.depg == 999999999:
                self.pg = BBR_UNIT * 75 // 100
            else:
                self.pg = max(self.pg * drain_decay_num // 100, PG_MIN_VAL)
            self.cwnd_g = sq(self.pg)
            if self.pg / BBR_UNIT < self.min_pg_reached:
                self.min_pg_reached = self.pg / BBR_UNIT
            self.drain_rounds += 1
            if excess > self.max_excess_in_drain:
                self.max_excess_in_drain = excess
            if excess <= T:
                self.drok += 1
            elif excess < self.depg:
                self.drok += 1
            else:
                self.drok = 0
            self.depg = excess
            if self.drok >= DRAIN_EXIT:
                self.excess_at_exit = excess
                self.mode = MODE_PROBE_BW
                self.ez = 0
                return True
        return False


def simulate_single(drain_decay_num, seed=42, rounds=500):
    rng = random.Random(seed)
    brtt = max(3000, T_PROP + rng.randint(-1000, 1000))
    flow = Flow()
    stats = []
    for rd in range(rounds):
        rtt_s = max(1e-9, brtt * 1e-6)
        for _ in range(8):
            pacing = BW_BPS * flow.pg / BBR_UNIT
            cwnd_r = (BDP_PKTS * flow.cwnd_g / BBR_UNIT) * MSS * 8 / rtt_s
            rate = min(pacing, cwnd_r)
            inflight = rate * rtt_s / 8 / MSS
            qb = max(0.0, inflight * MSS - BDP_BYTES)
            rtt_s = brtt * 1e-6 + qb / BD
        queue_us = qb / BD * 1e6
        excess = max(0.0, queue_us - brtt)
        flow.step(excess, 0.0, rd, drain_decay_num)
        stats.append({
            'rd': rd, 'q_us': queue_us, 'excess': excess,
            'pg': flow.pg / BBR_UNIT, 'mode': flow.mode,
        })
    return stats, flow


def main():
    print("=" * 90)
    print("BOUNDARY 4: KCC_DRAIN_DECAY_NUM SWEEP")
    print("=" * 90)
    print("Sweeping drain_decay_num: 75, 80, 85, 90, 92, 95")
    print("Mode: ECO single flow, DRAIN entry from STARTUP (pg=2.89x)")
    print("PG_MIN=0.0156x (BBR_UNIT//64) for proper decay visibility")
    print("Measure: rounds spent in DRAIN, minimum pg reached")
    print("=" * 90)

    values = [75, 80, 85, 90, 92, 95]

    print("\n%-14s %14s %16s %14s %14s" % (
        "Decay_Num", "Drain_Rounds", "Min_PG_reached", "Decay_factor", "Exit_Excess(us)"))
    print("-" * 72)
    for v in values:
        stats, flow = simulate_single(v, seed=42, rounds=500)
        drain_rd_count = 0
        for s in stats:
            if s['mode'] == MODE_DRAIN:
                drain_rd_count += 1
        min_pg = flow.min_pg_reached
        decay_factor = v / 100.0
        print("%-14d %14d %16.4f %14.3f %14.0f" % (v, drain_rd_count, min_pg, decay_factor, flow.excess_at_exit))

    print("\n--- INTERPRETATION ---")
    print("Current=92 (*0.92): decay factor 0.92 per round.")
    print("Lower = faster drain (fewer rounds, more aggressive queue clearance, lower min PG).")
    print("Higher = slower drain (gentler, less risk of underutilization).")
    print("Recommended: 90-92 balances drain speed with smooth exit.") 


if __name__ == "__main__":
    main()
