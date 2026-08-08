#!/usr/bin/env python3
"""
PI vs Baseline Simulation for KCC Congestion Control.

Replicates exact C code mechanisms from tcp_kcc.c:
  1. Geodesic estimator G1/G2/G3 + pull-down + G3 lock
  2. kcc_get_model_rtt FILTER: min(x_est>>shift, min_rtt_us)
  3. kcc_bdp: (bw * model_rtt * gain) >> BBR_SCALE >> BW_SCALE
  4. BW sliding-window max over ~10 rounds
  5. Full FSM: STARTUP→DRAIN→PROBE_BW[1.25,0.75,1.0×6]
  6. qdelay_base update (exact C pattern from lines 8337-8347)
  7. PI controller on PROBE_BW cruise (exact C formula)
  8. Fluid queue model with random walk T_prop

The PI activates when min_rtt decreases (from geodesic pull-down tracking
the random-walk T_prop decreases), which pulls qdelay_base down and allows
qdelay_avg to exceed it in subsequent rounds.
"""

import random, itertools, time
from concurrent.futures import ProcessPoolExecutor, as_completed
from collections import defaultdict

BBR_SCALE = 8; BBR_UNIT = 1 << BBR_SCALE
BW_SCALE = 24; BW_UNIT = 1 << BW_SCALE
KCC_SCALE = 1024; SHIFT = 10

G2_N, G2_D = 122, 1000
G3F_N, G3F_D = 11, 10; G3S_N, G3S_D = 21, 20
PD_N, PD_D = 95, 100
G3_FC = 3; G3_SC = 4; PD_CNT = 3

PI_MAX_ADJ = 64
PI_GAIN_MIN = BBR_UNIT * 75 // 100
PI_GAIN_MAX = BBR_UNIT * 125 // 100

CYCLE_LEN = 8; FULL_BW_CNT = 3; DRAIN_TIMEOUT = 24; BW_WIN = 10

BW_MBPS = 1260.0; N_FLOWS = 6; MSS = 1500
C_BYTEPS = int(BW_MBPS * 1e6 / 8)
C_SEGPS = C_BYTEPS / MSS
T_PROP_BASE = 0.045
T_PROP_RANGE = (0.030, 0.060)
SIM_RTTS = 12000; WARMUP = 3000; EVAL = SIM_RTTS - WARMUP
MARGIN_PCT = [0.002, 0.005, 0.01, 0.02]
KP_SCALE = [0.5, 1.0, 2.0, 5.0]; N_SEEDS = 5


class Flow:
    def __init__(self, fid, rtt_prop, seed):
        self.fid = fid
        self.rng = random.Random(seed + fid * 9973)
        self.rtt_prop = rtt_prop

        # geodesic
        self.x_est = 0; self.samples = 0; self.jitter = 0
        self.qdelay_avg = 0; self.qdelay_base = 0

        # G3
        self.cf_cnt = 0; self.cf_slow = 0; self.pd_cnt = 0

        # FSM
        self.mode = 0; self.full_bw = 0; self.full_bw_cnt = 0
        self.full_bw_reached = False
        self.rnd = 0; self.round_start = False

        # BW
        self.bw_ring = []; self.cur_bw = 1

        # gains
        self.p_gain = BBR_UNIT; self.c_gain = BBR_UNIT; self.cycle = 0

        # min RTT
        self.mr_us = int(rtt_prop * 1e6)
        self.mr_stamp = 0; self.mr_snap = self.mr_us; self.ff_cnt = 0

        # DRAIN
        self.dr_cnt = 0; self.tgt_inf = 0

        # delivery
        self.cwnd = 10; self.total_segs = 0; self.losses = 0

    def model_rtt_us(self):
        if self.samples < 5 or self.x_est == 0:
            return self.mr_us
        return min(self.x_est >> SHIFT, self.mr_us)

    def bdp(self, bw_bwu, gain):
        mr = max(self.model_rtt_us(), 1)
        w = bw_bwu * mr
        r = (w * gain) >> BBR_SCALE
        return (r + BW_UNIT - 1) >> BW_SCALE

    # ── G1/G2 geodesic ─────────────────────────────────────────────────
    def geo_update(self, rtt_us):
        z = rtt_us << SHIFT
        if self.samples == 0:
            self.x_est = z; self.jitter = max(rtt_us >> 4, 1)
            self.qdelay_avg = 0; self.samples = 1; self.qdelay_base = 0; return
        if self.samples == 1 and self.mr_us:
            self.x_est = min(self.x_est, self.mr_us << SHIFT)
        innov = z - self.x_est
        if innov <= 0:
            self.x_est = z if z < self.x_est else self.x_est
        else:
            g = self.x_est * G2_N // G2_D; self.x_est = min(self.x_est + g, z)
        rj = abs(innov) >> SHIFT
        self.jitter = (self.jitter * 7 + rj) // 8 if self.samples > 1 else rj
        self.jitter = min(self.jitter, max(self.mr_us, 500000))
        qd = max(0, int((z - self.x_est) >> SHIFT))
        self.qdelay_avg = (self.qdelay_avg * 7 + qd) // 8 if self.samples > 1 else qd
        self.samples = min(self.samples + 1, 2**31 - 1)

    # ── G3 ──────────────────────────────────────────────────────────────
    def g3_update(self, rnd):
        x_us = self.x_est >> SHIFT
        ft = self.mr_us * G3F_N // G3F_D; st = self.mr_us * G3S_N // G3S_D
        if x_us >= ft:
            self.cf_cnt = min(self.cf_cnt + 1, 255); self.cf_slow = min(self.cf_slow + 1, 255)
        elif x_us >= st:
            self.cf_cnt = 0; self.cf_slow = min(self.cf_slow + 1, 255)
        else:
            self.cf_cnt = 0
        if x_us <= self.mr_us: self.cf_cnt = 0; self.cf_slow = 0
        if self.cf_cnt >= G3_FC or self.cf_slow >= G3_SC:
            self.mr_us = x_us; self.mr_stamp = rnd; self.cf_cnt = 0; self.cf_slow = 0; return False
        return self.cf_cnt > 0 or self.cf_slow > 0

    # ── pull-down ──────────────────────────────────────────────────────
    def pd_update(self, rnd):
        x_us = self.x_est >> SHIFT
        if x_us > 1 and x_us < self.mr_us * PD_N // PD_D:
            self.pd_cnt += 1
            if self.pd_cnt >= PD_CNT:
                self.mr_us = min(self.mr_us, max(x_us, int(self.mr_us * 0.9)))
                self.mr_stamp = rnd; self.pd_cnt = 0; return True
        else:
            self.pd_cnt = 0
        return False

    # ── qdelay_base (exact C pattern) ──────────────────────────────────
    def update_qdelay_base(self):
        lowered = (self.mr_us < self.mr_snap)
        if lowered:
            if self.qdelay_base > 0:
                self.qdelay_base = self.qdelay_avg * 90 // 100 + self.qdelay_base * 10 // 100
            else:
                self.qdelay_base = self.qdelay_avg
        elif self.qdelay_base > 0:
            self.qdelay_base = max(self.qdelay_base, self.qdelay_avg)
        self.mr_snap = self.mr_us

    # ── BW ──────────────────────────────────────────────────────────────
    def bw_update(self, segs, int_us):
        if int_us <= 0 or segs < 0: return
        bw = int((segs << BW_SCALE) / max(int_us, 1))
        self.bw_ring.append((self.rnd, bw))
        self.bw_ring = [s for s in self.bw_ring if s[0] >= self.rnd - BW_WIN]
        self.cur_bw = max((s[1] for s in self.bw_ring), default=1)

    # ── FSM ─────────────────────────────────────────────────────────────
    def fsm_update(self):
        if self.mode == 0 and not self.full_bw_reached and self.round_start:
            if self.full_bw == 0:
                self.full_bw = self.cur_bw
            else:
                thr = self.full_bw * 125 // 100
                if self.cur_bw >= thr:
                    self.full_bw = self.cur_bw; self.full_bw_cnt = 0
                else:
                    self.full_bw_cnt += 1
                    if self.full_bw_cnt >= FULL_BW_CNT:
                        self.full_bw_reached = True
        if self.mode == 0 and self.full_bw_reached:
            self.mode = 1; self.dr_cnt = 0
            self.tgt_inf = self.bdp(self.cur_bw, BBR_UNIT)
        if self.mode == 1:
            if self.cwnd <= self.tgt_inf or self.cwnd <= 4:
                self.mode = 2; self.cycle = self.rnd % CYCLE_LEN; self.dr_cnt = 0
            elif self.round_start:
                self.dr_cnt += 1
                if self.dr_cnt >= DRAIN_TIMEOUT:
                    self.mode = 2; self.cycle = self.rnd % CYCLE_LEN; self.dr_cnt = 0
        if self.round_start and self.mode == 2:
            self.cycle = (self.cycle + 1) & (CYCLE_LEN - 1)

    # ── gains + PI ─────────────────────────────────────────────────────
    def set_gains(self, kp_scale=0.0, margin_pct=0.01):
        if self.mode == 0:
            self.p_gain = int(2.885 * BBR_UNIT); self.c_gain = int(2.885 * BBR_UNIT)
        elif self.mode == 1:
            self.p_gain = int(0.347 * BBR_UNIT); self.c_gain = int(2.885 * BBR_UNIT)
        elif self.mode == 2:
            gtab = [int(1.25 * BBR_UNIT), int(0.75 * BBR_UNIT)] + [BBR_UNIT] * 6
            self.p_gain = gtab[self.cycle & 7]; self.c_gain = int(2.0 * BBR_UNIT)
            # PI on cruise (p_gain >= 1.0x) with qdelay_base established
            if self.p_gain >= BBR_UNIT and kp_scale > 0 and self.qdelay_base > 0:
                qp = max(0, self.qdelay_avg - self.qdelay_base)
                margin = max(100, self.mr_us // 100)
                marg = max(1, int(margin * margin_pct * 100))
                err = qp - marg
                if self.mr_us > 0:
                    base_kp = max(1, int(BBR_UNIT * 10 / max(self.mr_us, 1)))
                    kp = max(1, int(base_kp * kp_scale))
                    adj = kp * err // BBR_UNIT
                    adj = max(-PI_MAX_ADJ, min(PI_MAX_ADJ, adj))
                    if err < 0: adj = adj * 2
                    if qp > marg * 8: adj = -PI_MAX_ADJ
                    ng = self.p_gain + adj
                    self.p_gain = max(PI_GAIN_MIN, min(PI_GAIN_MAX, ng))


def simulate(args):
    margin_pct, kp_scale, seed = args
    rng = random.Random(seed)
    t_prop = T_PROP_BASE

    flows = [Flow(i, t_prop + (rng.random() - 0.5) * 0.005, seed) for i in range(N_FLOWS)]

    fair_bwu = int((C_SEGPS / N_FLOWS) * BW_UNIT)
    for f in flows:
        f.cur_bw = max(fair_bwu, 1)
        f.bw_ring = [(0, f.cur_bw)]

    queue = 0
    max_q = int(0.2 * C_BYTEPS)

    f_del = [0.0] * N_FLOWS; f_loss = [0] * N_FLOWS

    for rt in range(SIM_RTTS):
        if rt % 50 == 0:
            t_prop = max(T_PROP_RANGE[0], min(T_PROP_RANGE[1],
                         t_prop + rng.uniform(-0.002, 0.002)))

        rtt_s = t_prop + queue / max(C_BYTEPS, 1)
        rtt_s = max(rtt_s, 0.0001)

        # ── send ──
        total_s = 0; f_send = [0] * N_FLOWS
        for i, f in enumerate(flows):
            f.round_start = False
            bw_sp = f.cur_bw / BW_UNIT
            rate = bw_sp * (f.p_gain / BBR_UNIT) * rtt_s * 1e6
            bdp_t = f.bdp(f.cur_bw, f.c_gain)
            cw = max(4, bdp_t); f.cwnd = cw
            s = max(1, min(int(rate), cw))
            f_send[i] = s; total_s += s

        # ── bottleneck ──
        queue += total_s * MSS
        over = 0
        if queue > max_q: over = queue - max_q; queue = max_q
        serve = C_BYTEPS * rtt_s
        deliv = min(queue, serve); queue -= deliv; queue = max(queue, 0)

        # ── per-flow ──
        for i, f in enumerate(flows):
            share = f_send[i] / max(total_s, 1)
            sd = max(0, int(share * deliv / MSS))
            f.total_segs += sd; f_del[i] += sd
            if over > 0 and rng.random() < 0.03: f.losses += 1; f_loss[i] += 1

            f_rtt = t_prop + queue / max(C_BYTEPS, 1)
            noise = f.rng.gauss(0, f_rtt * 0.005)
            fu = int(max(100, (f_rtt + noise) * 1e6))

            # geodesic + G3
            f.geo_update(fu)

            locked = f.g3_update(rt)

            # min RTT window + pull-down (when not locked)
            pd_fired = False
            if not locked:
                if fu <= f.mr_us:
                    if fu < f.mr_us * 75 // 100:
                        if fu < f.mr_us // 4: f.ff_cnt = 0; f.mr_us = fu
                        else: f.ff_cnt += 1
                        if f.ff_cnt >= 3: f.mr_us = fu
                    f.mr_stamp = rt
                pd_fired = f.pd_update(rt)

            # qdelay_base
            f.update_qdelay_base()

            # BW
            bi = int(max(rtt_s, 0.001) * 1e6)
            f.bw_update(sd, bi)

            # round
            if f.rnd != rt:
                f.rnd = rt; f.round_start = True
                f.fsm_update(); f.set_gains(kp_scale=kp_scale, margin_pct=margin_pct)

    # ── throughput ──
    time_sec = EVAL * (t_prop + 0.005)
    if time_sec <= 0: time_sec = 1
    tputs = [f_del[i] * EVAL // max(SIM_RTTS, 1) * MSS * 8 / 1e6 / time_sec for i in range(N_FLOWS)]
    avg_t = sum(tputs) / N_FLOWS; util = sum(tputs) / BW_MBPS * 100; tot_l = sum(f_loss)
    return avg_t, util, tot_l


def main():
    configs = []
    for seed in range(N_SEEDS):
        configs.append((0.0, 0.0, seed + 1000))
    for mp, ks in itertools.product(MARGIN_PCT, KP_SCALE):
        for seed in range(N_SEEDS):
            configs.append((mp, ks, seed + 1000))

    agg = defaultdict(lambda: {'t': [], 'u': [], 'l': []})
    nw = min(16, len(configs))
    print(f"Running {len(configs)} sims ({nw} workers)...", flush=True)
    start = time.time()

    with ProcessPoolExecutor(max_workers=nw) as pool:
        fmap = {pool.submit(simulate, c): c for c in configs}
        done = 0
        for fut in as_completed(fmap):
            cfg = fmap[fut]
            try: t, u, lo = fut.result(); agg[(cfg[0], cfg[1])]['t'].append(t); agg[(cfg[0], cfg[1])]['u'].append(u); agg[(cfg[0], cfg[1])]['l'].append(lo)
            except Exception as e: print(f"  FAIL {cfg}: {e}", flush=True)
            done += 1
            if done % 10 == 0: print(f"  {done}/{len(configs)} ({time.time()-start:.1f}s)", flush=True)

    print(f"\nDone in {time.time()-start:.1f}s\n", flush=True)

    keys = sorted(agg.keys(), key=lambda k: (k[0] if k[0] > 0 else -1, k[1]))
    print(f"{'Type':>10} {'Kp':>8} {'TputMbps':>10} {'Util%':>8} {'Loss':>6}  N")
    print("-" * 52)
    for k in keys:
        v = agg[k]
        at = sum(v['t']) / len(v['t']) if v['t'] else 0
        au = sum(v['u']) / len(v['u']) if v['u'] else 0
        al = sum(v['l']) / len(v['l']) if v['l'] else 0
        label = "BASELINE" if k[0] == 0 else f"m={k[0]:.3f}"
        print(f"{label:>10} {k[1]:>8.1f} {at:>10.2f} {au:>8.1f} {al:>6.0f} {len(v['t']):>3}")

    base = agg[(0.0, 0.0)]
    bt = sum(base['t']) / len(base['t']) if base['t'] else 0
    bu = sum(base['u']) / len(base['u']) if base['u'] else 0
    print(f"\nBaseline: {bt:.2f} Mbps, {bu:.1f}% util")
    for k in keys:
        if k == (0.0, 0.0): continue
        v = agg[k]
        at = sum(v['t']) / len(v['t']) if v['t'] else 0
        diff = (at - bt) / bt * 100 if bt > 0 else 0
        print(f"  margin={k[0]:.3f} kp={k[1]:.1f}: {at:>8.2f} Mbps ({diff:>+5.1f}%)")


if __name__ == '__main__':
    main()
