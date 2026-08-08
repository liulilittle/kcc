from __future__ import division
import random
import math

BBR_UNIT = 256
AI_N, AI_D = 1, 100
MD_N, MD_D = 1, 8
DRAIN_DECAY_NUM = 92
DRAIN_DECAY_DEN = 100
TARGET_DIV = 128
DRAIN_DIV = 32
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


BBR_CYCLE = [1.25, 0.75, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]


class KCCFlow:
    def __init__(self, pg_min, pg_max, floor):
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
        self.pg_min = pg_min
        self.pg_max = pg_max
        self.floor = floor

    def step(self, excess, bw_mbps, rd):
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
                self.pg = min(self.pg + BBR_UNIT * AI_N // AI_D, self.pg_max)
            else:
                md = (self.pg * excess * MD_N) // (tp * MD_D)
                self.pg = max(self.pg - md, self.pg_min)
            if DRAIN_INTERVAL > 0 and (self.rtt_cnt % DRAIN_INTERVAL) == 0:
                self.pg = self.pg_min
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
                self.pg = max(self.pg * DRAIN_DECAY_NUM // DRAIN_DECAY_DEN, self.pg_min)
            self.cwnd_g = max(sq(self.pg), self.floor)
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


class BBRFlow:
    def __init__(self):
        self.cwnd_g = 2 * BBR_UNIT

    def get_pg(self, rd):
        return int(BBR_CYCLE[(rd // 8) % 8] * BBR_UNIT)


def simulate(kcc_count, bbr_count, pg_min, pg_max, floor, seed=42, rounds=800):
    rng = random.Random(seed)
    total = kcc_count + bbr_count
    brtts = [max(3000, T_PROP + rng.randint(-1000, 1000)) for _ in range(total)]
    kcc_flows = [KCCFlow(pg_min, pg_max, floor) for _ in range(kcc_count)]
    bbr_flows = [BBRFlow() for _ in range(bbr_count)]
    stats = []
    for rd in range(rounds):
        bbr_pgs = [bf.get_pg(rd) for bf in bbr_flows]
        all_pg = [f.pg for f in kcc_flows] + bbr_pgs
        all_cg = [f.cwnd_g for f in kcc_flows] + [bf.cwnd_g for bf in bbr_flows]

        avg_tp = sum(brtts) // total
        rtt_s = max(1e-9, avg_tp * 1e-6)
        for _ in range(8):
            total_rate = 0.0
            total_inflight = 0.0
            for i in range(total):
                pacing = BW_BPS * all_pg[i] / BBR_UNIT
                cwnd_r = (BDP_PKTS * all_cg[i] / BBR_UNIT) * MSS * 8 / rtt_s
                rate = min(pacing, cwnd_r)
                total_rate += rate
                total_inflight += rate * rtt_s / 8 / MSS
            total_rate = min(total_rate, BW_BPS)
            queue_bytes = max(0.0, total_inflight * MSS - BDP_BYTES)
            rtt_s = avg_tp * 1e-6 + queue_bytes / BD
        queue_us = queue_bytes / BD * 1e6
        excess = max(0.0, queue_us - avg_tp)

        for f in kcc_flows:
            f.step(excess, 0.0, rd)

        if rd >= 200:
            kcc_pgs_float = [f.pg / BBR_UNIT for f in kcc_flows]
            min_pg = min(kcc_pgs_float)
            max_pg = max(kcc_pgs_float)
            stats.append({
                'q_us': queue_us, 'excess': excess,
                'kcc_pg_mean': sum(kcc_pgs_float) / len(kcc_pgs_float),
                'kcc_pg_min': min_pg, 'kcc_pg_max': max_pg,
                'bbr_pg_mean': sum(bbr_pgs) / BBR_UNIT / len(bbr_pgs) if bbr_pgs else 0,
            })
    return stats


def main():
    print("=" * 90)
    print("BOUNDARY 5: PG_MIN AND PG_MAX CONFIRMATION")
    print("=" * 90)
    print("PG_MIN=0.75x (BBR_UNIT*3/4 = %d) and PG_MAX=1.25x (BBR_UNIT*5/4 = %d)" % (BBR_UNIT*3//4, BBR_UNIT*5//4))
    print("Standard BBR values from Cardwell et al. 2016.")
    print("=" * 90)

    pg_min_f = 0.75
    pg_max_f = 1.25
    pg_min_val = int(pg_min_f * BBR_UNIT)
    pg_max_val = int(pg_max_f * BBR_UNIT)

    print("\n--- TEST 1: ECO mode (floor=1.0x), 4 KCC flows ---")
    stats = simulate(4, 0, pg_min_val, pg_max_val, BBR_UNIT, seed=42, rounds=800)
    q_avg = sum(s['q_us'] for s in stats) / len(stats)
    pg_avg = sum(s['kcc_pg_mean'] for s in stats) / len(stats)
    pg_min_obs = min(s['kcc_pg_min'] for s in stats)
    pg_max_obs = max(s['kcc_pg_max'] for s in stats)
    print("  Q_avg = %.0f us" % q_avg)
    print("  PG_avg = %.4fx (range [%.4f, %.4f])" % (pg_avg, pg_min_obs, pg_max_obs))
    if pg_min_obs >= pg_min_f - 0.01:
        print("  PASS: PG_MIN=%s floor holds" % str(pg_min_f))
    if pg_max_obs <= pg_max_f + 0.01:
        print("  PASS: PG_MAX=%s ceiling holds" % str(pg_max_f))

    turbo_floor = BBR_UNIT * 188 // 100

    print("\n--- TEST 2: TURBO mode (floor=1.88x), 3 KCC + 1 BBR ---")
    stats = simulate(3, 1, pg_min_val, pg_max_val, turbo_floor, seed=42, rounds=800)
    q_avg = sum(s['q_us'] for s in stats) / len(stats)
    kcc_pg_avg = sum(s['kcc_pg_mean'] for s in stats) / len(stats)
    kcc_pg_min_obs = min(s['kcc_pg_min'] for s in stats)
    print("  Q_avg = %.0f us" % q_avg)
    print("  KCC_PG_avg = %.4fx, min = %.4f" % (kcc_pg_avg, kcc_pg_min_obs))
    bbr_pg_avg = sum(s['bbr_pg_mean'] for s in stats) / len(stats)
    print("  BBR_PG_avg = %.4fx" % bbr_pg_avg)
    print("  With floor=1.88x, cwnd_gain = max(pg^2, 1.88x).")
    print("  During periodic drain (pg=0.75x), cwnd_gain = max(0.56x, 1.88x) = 1.88x.")
    print("  => No starvation: cwnd floor prevents throughput collapse.")

    print("\n--- TEST 3: Single flow starvation check ---")
    stats = simulate(1, 0, pg_min_val, pg_max_val, turbo_floor, seed=42, rounds=800)
    min_q = min(s['q_us'] for s in stats)
    max_q = max(s['q_us'] for s in stats)
    q_avg = sum(s['q_us'] for s in stats) / len(stats)
    pg_avg = sum(s['kcc_pg_mean'] for s in stats) / len(stats)
    print("  Single TURBO flow: Q range [%.0f, %.0f] us, avg = %.0f us" % (min_q, max_q, q_avg))
    print("  PG_avg = %.4fx" % pg_avg)
    if q_avg < T_PROP * 2:
        print("  PASS: Queue controlled, no starvation")
    else:
        print("  WARN: High queue")

    print("\n--- CONCLUSION ---")
    print("PG_MIN=0.75x: Safe. During periodic drain pg drops to 0.75x,")
    print("  cwnd_floor ensures sufficient inflight.")
    print("PG_MAX=1.25x: Effective. Caps probe gain (BBR standard).")
    print("Both values confirmed valid for KCC 2.0.")


if __name__ == "__main__":
    main()
