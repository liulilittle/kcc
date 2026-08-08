"""Corrected KCC 2.0 periodic drain interval sweep.
Uses actual KCC 2.0 parameters: PG_MIN=0.75x, AI=3.125%, MD=1/1."""

import random, math

BBR_UNIT = 256

PG_MIN = BBR_UNIT * 3 // 4       # 192 = 0.75x
PG_MAX = BBR_UNIT * 5 // 4       # 320 = 1.25x
AI_N, AI_D = 25, 800              # 3.125%
MD_N, MD_D = 1, 1                 # proportional 1/1
DRAIN_DECAY_N, DRAIN_DECAY_D = 92, 100
TARGET_DIV = 128
DRAIN_DIV = 32
DRAIN_EXIT = 4

MSS = 1448

MODE_PROBE_BW = 0
MODE_DRAIN = 1

class Flow:
    def __init__(self, cwnd_floor):
        self.pg = BBR_UNIT
        self.mode = MODE_PROBE_BW
        self.mr = None  # set externally
        self.floor = cwnd_floor
        self.rtt_cnt = random.randint(0, 1023)
        self.depg = 999999
        self.drok = 0
        self.fpc = 0

    def step(self, drain_interval):
        tp = max(1, self.mr)
        if self.fpc > 0:
            self.fpc -= 1

        if self.mode == MODE_PROBE_BW:
            if drain_interval > 0 and (self.rtt_cnt % drain_interval) == 0:
                self.pg = PG_MIN
                self.rtt_cnt += 1
            else:
                # AI: slow increase toward target
                self.pg = min(self.pg + BBR_UNIT * AI_N // AI_D, PG_MAX)
                self.rtt_cnt += 1

        elif self.mode == MODE_DRAIN:
            if self.depg == 999999:
                self.pg = PG_MIN
            else:
                self.pg = max(self.pg * DRAIN_DECAY_N // DRAIN_DECAY_D, PG_MIN)

        return self.pg


def simulate(nf, interval, floor, t_prop_us, bw_bps, seeds=None, rounds=1200):
    if seeds is None:
        seeds = [42 + i for i in range(nf)]
    flows = [Flow(floor) for _ in range(nf)]
    for f in flows:
        f.mr = t_prop_us

    bdp_bytes = (bw_bps / 8) * t_prop_us * 1e-6
    bdp_pkts = bdp_bytes / MSS

    stats = []
    for rd in range(rounds):
        # Collect pg
        pgs = [f.step(interval) for f in flows]
        avg_pg_bbru = sum(pgs) / nf
        avg_pg = avg_pg_bbru / BBR_UNIT

        # Compute queue from avg inflight
        total_cwnd_pkts = sum(f.floor / BBR_UNIT * bdp_pkts for f in flows)
        total_inflight_b = total_cwnd_pkts * MSS
        queue_b = max(0.0, total_inflight_b - bdp_bytes)
        queue_us = queue_b / (bw_bps / 8) * 1e6

        # throughput = min(bw_bps, total send rate)
        total_send = nf * avg_pg * bw_bps  # simplified
        throughput = min(bw_bps, total_send)

        if rd >= 200:
            stats.append({
                'q_us': queue_us,
                'pg': avg_pg,
                'tp': throughput,
            })
    q_avg = sum(s['q_us'] for s in stats) / len(stats) if stats else 0
    util = min(100.0, (sum(s['tp'] for s in stats) / len(stats)) / bw_bps * 100) if stats else 0
    return q_avg, util


def main():
    t_prop = 60000   # 60ms
    bw = 1000e6      # 1Gbps
    nf = 4
    floor_turbo = BBR_UNIT * 188 // 100  # 1.88x
    floor_eco = BBR_UNIT                 # 1.0x

    print("=" * 80)
    print("KCC 2.0 Periodic Drain Interval Sweep (corrected parameters)")
    print("T_prop=%dms, BW=%.0fMbps, N=%d flows" % (t_prop // 1000, bw / 1e6, nf))
    print("PG_MIN=0.75x, PG_MAX=1.25x, AI=%.1f%%, MD=1/1" % (AI_N * 100 / AI_D))
    print("=" * 80)

    for name, floor in [("TURBO (1.88x)", floor_turbo)]:
        print("\n--- %s ---" % name)
        print("%-10s %12s %12s %12s" % ("Interval", "Q_avg(us)", "Util%%", "Q/prop%%"))
        print("-" * 48)
        results = []
        for interval in [32, 48, 64, 80, 96, 128, 192, 256]:
            q, u = simulate(nf, interval, floor, t_prop, bw, rounds=1200)
            qpct = q / t_prop * 100
            results.append((interval, q, u, qpct))
            print("%-10d %12.1f %11.1f%% %11.2f%%" % (interval, q, u, qpct))

        print("\n--- Analysis ---")
        print("Overhead per drain: 1/%d round = %.1f%% throughput impact" %
              (128, 100.0 / 128))
        print("At 60ms RTT, 128 rounds = %.1fs between drains" %
              (128 * t_prop / 1e6))
        print("At 10ms RTT, 128 rounds = %.1fs between drains" %
              (128 * 10000 / 1e6))

    print("\n--- CONCLUSION ---")
    print("128 chosen for: 1) bitmask efficiency (127 = 2^7-1),")
    print("2) overhead 1/128 < 0.8%, 3) at 60ms -> 7.7s interval")
    print("complementing PROBE_RTT at 10s for clean min_rtt refresh.")


if __name__ == "__main__":
    main()
