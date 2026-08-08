# KCC 2.0 PI controller grid search - 1.26 Gbps bottleneck
import random, sys, time

SCALE, SHIFT = 1024, 10; MSS = 1500; BW_MBPS = 1260; BBR_UNIT = 256
CWND_GAIN = 2.0; FAST_N = 3; SLOW_N = 4; PD_N = 3
CYCLE = [1.25, 0.75, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
DURATION_SEC = 120; DT_SEC = 0.001; WARMUP_SEC = 30
N_FLOWS = 6

def rtt_noise(): return random.gauss(0, 0.0002)  # 200us jitter

class KCCFlow:
    def __init__(self, margin, Kp, Ki, ff):
        self.base = random.uniform(0.030, 0.060)  # tp in seconds
        self.mr   = self.base; self.x  = self.base
        self.cnf  = 0; self.csl = 0; self.pd = 0
        self.cwnd = 4.0; self.cycle = 0; self.qavg = 0.0
        self.qbase = 0.0; self.pi = 0.0
        self.sent = 0.0; self.margin = margin; self.Kp = Kp
        self.Ki = Ki; self.ff = ff; self.loss = 0
        self.pgain = 1.0; self.bw_est = BW_MBPS / N_FLOWS

    def update_base_rtt(self):
        self.base += random.gauss(0, 0.001)
        self.base = max(0.030, min(0.060, self.base))

    def update_geodesic(self, rtt_s):
        z = rtt_s
        if z <= self.x: self.x = z  # G1
        else: self.x = min(self.x * 1.12, z)  # G2
        if z < self.mr: self.mr = z
        ft = self.mr * 1.10; st = self.mr * 1.05
        x_now = self.x
        if x_now >= ft: self.cnf += 1; self.csl += 1
        elif x_now >= st: self.cnf = 0; self.csl += 1
        else: self.cnf = 0
        if x_now <= self.mr: self.cnf = 0; self.csl = 0
        if self.cnf >= FAST_N: self.mr = x_now; self.cnf = self.csl = 0
        elif self.csl >= SLOW_N: self.mr = x_now; self.cnf = self.csl = 0
        if self.cnf == 0 and self.csl == 0:
            if x_now < self.mr * 0.95: self.pd += 1
            else: self.pd = 0
            if self.pd >= PD_N: self.mr = x_now; self.pd = 0

    def update_qdelay(self, rtt_s):
        mrt = min(self.x, self.mr)
        qi = max(0, rtt_s - mrt)
        self.qavg = self.qavg * 0.875 + qi * 0.125
        if self.qbase == 0 or qi < self.qbase * 0.9:
            self.qbase = qi * 0.5 + self.qbase * 0.5
        else:
            self.qbase = max(self.qbase * self.ff, qi * (1 - self.ff))
        return mrt

    def get_pacing_gain(self):
        pg = CYCLE[self.cycle & 7]; self.cycle += 1
        if abs(pg - 1.0) < 0.01 and self.qbase > 1e-9:
            qp = max(0, self.qavg - self.qbase)
            e = qp - self.margin
            self.pi += e * self.Ki * self.base
            self.pi = max(-0.001, min(0.001, self.pi))
            if qp <= self.margin * 3:
                a = self.Kp * e + self.pi
                a = max(-0.05, min(0.05, a))
                pg = 1.0 + a
            else: self.pi = 0.0
        pg = max(0.95, min(1.05, pg))
        self.pgain = pg
        return pg

    def update_cwnd(self, bdp_segs):
        pg = self.pgain
        target = bdp_segs * CWND_GAIN * pg
        target = max(target, 4)
        if self.cwnd < target: self.cwnd += max(1, (target - self.cwnd) * 0.3)
        else: self.cwnd = max(target, self.cwnd * 0.95)

def sim_one(margin, Kp, Ki, ff):
    flows = [KCCFlow(margin, Kp, Ki, ff) for _ in range(N_FLOWS)]
    queue_bytes = 0.0; total_sent = 0.0; total_loss = 0; peak_queue = 0
    steps = int((DURATION_SEC + WARMUP_SEC) / DT_SEC)
    warmup_steps = int(WARMUP_SEC / DT_SEC)

    for step in range(steps):
        # Queue drain
        drain_bytes = BW_MBPS * 1e6 / 8 * DT_SEC
        queue_bytes = max(0, queue_bytes - drain_bytes)
        queue_s = queue_bytes / (BW_MBPS * 1e6 / 8)
        peak_queue = max(peak_queue, queue_s)

        # Each flow
        total_rate = 0.0
        for f in flows:
            if step % int(f.base / DT_SEC) == 0:
                f.update_base_rtt()
                rtt = f.base + queue_s + rtt_noise()
                f.update_geodesic(rtt)
                f.update_qdelay(rtt)
                pg = f.get_pacing_gain()
                mrt = min(f.x, f.mr)
                bdp_bytes = BW_MBPS * 1e6 / 8 * mrt
                bdp_segs = bdp_bytes / MSS
                f.update_cwnd(bdp_segs)
                rate = f.bw_est * 1e6 / 8 * pg
                total_rate += rate
                # Track sent
                if step >= warmup_steps:
                    f.sent += min(f.cwnd * MSS, rate * DT_SEC)

        # Queue injection
        queue_bytes += total_rate * DT_SEC

        # Loss if queue exceeds 2 BDP
        avg_rtt = sum(f.mr for f in flows) / N_FLOWS
        bdp_ref = BW_MBPS * 1e6 / 8 * avg_rtt
        if queue_bytes > bdp_ref:
            overflow = queue_bytes - bdp_ref
            total_loss += overflow / MSS * 2  # approx loss segments
            queue_bytes = bdp_ref * 0.8  # drain some

    total_sent = sum(f.sent for f in flows)
    throughput = total_sent / DURATION_SEC / 1e6 * 8  # Mbps
    utilization = throughput / BW_MBPS * 100
    return utilization, throughput, total_loss, peak_queue * 1e6  # us

if __name__ == '__main__':
    print("KCC 2.0 PI GRID SEARCH — 1.26Gbps bottleneck")
    print("=" * 55)
    print(f"  Flows={N_FLOWS}  Duration={DURATION_SEC}s  Warmup={WARMUP_SEC}s")
    print()

    # Baseline (no PI: Kp=0, Ki=0)
    bu, bt, bl, bpq = sim_one(200e-6, 0, 0, 0.9999)
    print(f"BASELINE: {bt:.0f}Mbps ({bu:.1f}%) loss={bl:.0f} pq={bpq:.0f}us")
    print()

    results = []
    margins = [50e-6, 100e-6, 200e-6, 500e-6]
    Kps = [0.5, 1.0, 2.0, 5.0]
    Kis = [0.01, 0.05, 0.1, 0.2]
    ffs = [0.9999, 0.999, 0.99]

    total = len(margins) * len(Kps) * len(Kis) * len(ffs)
    done = 0
    best = None

    for m in margins:
        for Kp in Kps:
            for Ki in Kis:
                for ff in ffs:
                    u, t, l, pq = sim_one(m, Kp, Ki, ff)
                    gain = u - bu
                    results.append((u, t, l, pq, m, Kp, Ki, ff, gain))
                    done += 1
                    sys.stdout.write(f"\r  {done}/{total} m={int(m*1e6):>3}us Kp={Kp:3.1f} Ki={Ki:.3f} ff={ff:.4f} -> {t:.0f}Mbps {u:.1f}% (+{gain:+.1f}%) loss={l:.0f}   ")
                    sys.stdout.flush()
                    if l == 0 and (best is None or t > best[1]):
                        best = (u, t, l, pq, m, Kp, Ki, ff, gain)

    print("\n\nTop 10 (sorted by throughput, zero loss only):")
    ok = [(r) for r in results if r[2] == 0]
    ok.sort(key=lambda x: -x[1])
    for i, (u, t, l, pq, m, Kp, Ki, ff, gain) in enumerate(ok[:10]):
        print(f"  #{i+1}: m={int(m*1e6):>3}us Kp={Kp:3.1f} Ki={Ki:.3f} ff={ff:.4f} -> {t:.0f}Mbps {u:.1f}% (+{gain:+.1f}%) pq={pq:.0f}us")

    print(f"\nBEST: m={int(best[4]*1e6)}us Kp={best[5]:.1f} Ki={best[6]:.3f} ff={best[7]:.4f} -> {best[1]:.0f}Mbps ({best[0]:.1f}%)")
    print(f"Gain over baseline: {best[-1]:+.1f}%")
    print("DONE")
