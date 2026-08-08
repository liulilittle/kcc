"""Realistic KCC simulation with 8-phase cycle, pacing, queue dynamics."""
import random, sys

SCALE, SHIFT = 1024, 10
JITTER_DIV = 100.0
BBR_UNIT = 256
BW_SCALE = 24
USEC_PER_SEC = 1000000
MSS = 1500

# 8-phase cycle: [1.25, 0.75, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
CYCLE_GAIN = [1.25, 0.75, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
CWND_GAIN  = 2.0

class KCCReal:
    def __init__(self, tp_us, seed, use_boost=False):
        self.tp  = tp_us
        self.mr  = tp_us
        self.x   = tp_us * SCALE
        self.jtr = max(1.0, tp_us / JITTER_DIV)
        self.rng = random.Random(seed)
        self.cnf = self.csl = self.pd = 0
        self.cycle = 0        # PROBE_BW phase index
        self.phase_start = 0  # RTTs into current phase
        self.pacing_rate = 0  # bytes/sec
        self.cwnd = 4         # segments
        self.mr_down_rtt = -100  # RTT count when mr last dropped
        self.rtt_cnt = 0
        self.boost = use_boost
        self.boost_cnt = 0

    def step(self, bw_mbps, total_inflight_segs, bottleneck_buf_bytes):
        """Process one RTT. Returns pacing_rate, cwnd."""
        self.rtt_cnt += 1
        # Effective RTT = T_prop + queue_delay
        queue_segs = max(0, total_inflight_segs - bw_mbps * self.tp * 1e6 / 1e6 / 8 / MSS)
        qd_us = queue_segs * MSS * 8 / (bw_mbps * 1e6) * 1e6
        rtt = self.tp + qd_us + self.rng.gauss(0, self.jtr)
        rtt = max(1, int(rtt))
        z = rtt * SCALE

        # G1/G2
        if z <= self.x:
            self.x = z
        else:
            self.x = min(self.x + self.x * 12 // 100, z)

        # G3
        ft = self.mr * SCALE * 11 // 10
        st = self.mr * SCALE * 21 // 20
        bl = self.mr * SCALE
        if self.x >= ft: self.cnf += 1; self.csl += 1
        elif self.x >= st: self.cnf = 0; self.csl += 1
        else: self.cnf = 0
        if self.x <= bl: self.cnf = 0; self.csl = 0

        old_mr = self.mr
        if self.cnf >= 3: self.mr = self.x >> SHIFT; self.cnf = self.csl = 0
        elif self.csl >= 4: self.mr = self.x >> SHIFT; self.cnf = self.csl = 0

        xus = self.x >> SHIFT
        if self.cnf == 0 and self.csl == 0:
            if xus < self.mr * 95 // 100: self.pd += 1
            else: self.pd = 0
            if self.pd >= 3: self.mr = xus; self.pd = 0

        if self.mr < old_mr: self.mr_down_rtt = self.rtt_cnt

        # BDP
        model_rtt = min(self.x >> SHIFT, self.mr)
        bdp_bytes = bw_mbps * model_rtt * 1e6 / 1e6 / 8

        # 8-phase cycle
        self.phase_start += 1
        pgain = CYCLE_GAIN[self.cycle]
        if pgain < 1.0 and qd_us < model_rtt * 0.05:  # drain-skip: no queue -> skip 0.75x
            pgain = 1.0
        if self.phase_start >= 1:  # advance phase each RTT
            self.cycle = (self.cycle + 1) % 8
            self.phase_start = 0

        # Physics boost: mr just dropped -> 2 RTT window of extra cwnd
        cgain = CWND_GAIN
        if self.boost and self.rtt_cnt - self.mr_down_rtt <= 2:
            cgain = 2.5
            self.boost_cnt += 1

        target_inflight = bdp_bytes * cgain / MSS
        target_inflight = max(target_inflight, 4)

        # Pacing rate
        self.pacing_rate = (bdp_bytes / MSS) * pgain * MSS * 8 * USEC_PER_SEC / (model_rtt * 1e6 * BW_SCALE)
        self.pacing_rate = min(self.pacing_rate, bw_mbps * 1e6 / 8)
        self.pacing_rate = max(self.pacing_rate, 1)

        # Converge cwnd toward target
        if self.cwnd < target_inflight:
            self.cwnd = min(self.cwnd + (target_inflight - self.cwnd) * 0.3, target_inflight)
        else:
            self.cwnd = max(self.cwnd * 0.9, target_inflight)

        return self.cwnd

def run_bottleneck(tp_us, bw_mbps, flows, use_boost, rounds=5000):
    """Multi-flow bottleneck with realistic KCC."""
    kflows = [KCCReal(tp_us, i*7919, use_boost) for i in range(flows)]
    total_thru = [0.0] * flows

    for r in range(rounds):
        total_cwnd = sum(f.cwnd for f in kflows)
        # Bottleneck buffer (1 BDP)
        bdp_bytes = bw_mbps * 1e6 * tp_us / 1e6 / 8
        max_buf = bdp_bytes
        # Simulate: each flow independently steps
        for i, f in enumerate(kflows):
            queue = max(0, total_cwnd - bw_mbps * tp_us * 1e6 / 1e6 / 8 / MSS)
            cwnd = f.step(bw_mbps / flows, total_cwnd, max_buf)
            served = min(cwnd, bw_mbps * tp_us / 8 / MSS / max(1,flows))
            total_thru[i] += served * MSS * 8 / 1e6  # Mbps

    avg_thru = sum(total_thru) / flows / (rounds * tp_us / 1e6)
    return avg_thru

if __name__ == '__main__':
    print("PHYSICS BOOST — REALISTIC 8-PHASE BOTTLENECK")
    print("=" * 50)
    for tp in [1000, 5000, 10000, 45000]:
        for flows in [1, 4]:
            base  = run_bottleneck(tp, 1000, flows, False, 3000)
            boost = run_bottleneck(tp, 1000, flows, True, 3000)
            d = (boost - base) / base * 100 if base > 0 else 0
            tag = "+" if d > 0.5 else ("-" if d < -0.5 else "=")
            print(f"  T={tp:>5}us N={flows}  base={base:>6.0f}Mbps  boost={boost:>6.0f}Mbps  {tag} {d:+.1f}%")
    print("DONE")
