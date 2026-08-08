#!/usr/bin/env python3
"""
Nonlinear cwnd pressure scaling: verify physical correctness.
KCC 2.0 PROBE_BW: cwnd_gain = pg (not 1x)

Physics:
  Q_total = max(0, (avg(cwnd_gain) - 1) * T_prop)    steady-state queue
  AI:   pg += 0.02 * BBR_UNIT                          when excess < T_prop/128
  MD:   pg -= pg * excess * 5 / (T_prop * 2)          when excess >= T_prop/128
"""

import sys, math, argparse

BBR_UNIT      = 1.0
PG_MIN        = 0.75
PG_MAX        = 1.25
AI_RATE       = 0.02
MD_NUM, MD_DEN = 5, 2
DRAIN_PERIOD  = 64

class Flow(object):
    __slots__ = ('name', 'pg', 'cwnd_fn', 'throughput', 'is_bbr')
    def __init__(self, name, pg=1.0, cwnd_fn="pg", is_bbr=False):
        self.name = name
        self.pg = pg
        self.cwnd_fn = cwnd_fn
        self.throughput = 0.0
        self.is_bbr = is_bbr

    def cwnd_gain(self):
        if self.cwnd_fn == "1":   return 1.0
        if self.cwnd_fn == "pg":  return self.pg
        if self.cwnd_fn == "pg2": return self.pg * self.pg
        return 1.0


def advance_bbr(flows, rtt_cnt):
    cycle = [1.25, 0.75, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    for f in flows:
        if f.is_bbr:
            f.pg = cycle[rtt_cnt & 7]


def update_round(flows, C, T_prop, rtt_cnt):
    N = len(flows)
    BDP = C * T_prop
    bdp_per = BDP / max(N, 1)

    total_cwnd = 0.0
    for f in flows:
        total_cwnd += f.cwnd_gain() * bdp_per
    Q = max(0.0, total_cwnd / C - T_prop)

    target_q = T_prop / 128.0
    for f in flows:
        if f.is_bbr:
            continue
        if Q < target_q:
            f.pg = min(f.pg + AI_RATE, PG_MAX)
        else:
            md = f.pg * Q * MD_NUM / (T_prop * MD_DEN)
            f.pg = max(f.pg - md, PG_MIN)
        if (rtt_cnt & (DRAIN_PERIOD - 1)) == 0:
            f.pg = 0.75

    advance_bbr(flows, rtt_cnt)

    total_cwnd2 = 0.0
    for f in flows:
        total_cwnd2 += f.cwnd_gain() * bdp_per
    Q2 = max(0.0, total_cwnd2 / C - T_prop)
    rtt = T_prop + Q2
    for f in flows:
        f.throughput = f.cwnd_gain() * bdp_per / rtt
    return Q2


def simulate(flows, C, T_prop, rounds, warmup=500):
    for r in range(warmup):
        update_round(flows, C, T_prop, r)
    pg_hist = {}
    q_hist = []
    tp_hist = {}
    for f in flows:
        pg_hist[f.name] = []
        tp_hist[f.name] = []
    for r in range(rounds):
        Q = update_round(flows, C, T_prop, r + warmup)
        q_hist.append(Q)
        for f in flows:
            pg_hist[f.name].append(f.pg)
            tp_hist[f.name].append(f.throughput)
    return pg_hist, q_hist, tp_hist


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--flows", type=int, default=4)
    ap.add_argument("--bbr", type=int, default=0)
    ap.add_argument("--C", type=float, default=1.26e9)
    ap.add_argument("--T", type=float, default=0.060)
    ap.add_argument("--rounds", type=int, default=5000)
    ap.add_argument("--warmup", type=int, default=500)
    ap.add_argument("--mode", type=str, default="all")
    args = ap.parse_args()

    C = args.C; T = args.T; BDP = C * T
    print("="*70)
    print("C=%.3f Gbps  T_prop=%d ms  BDP=%.2f MB" % (C/1e9, T*1000, BDP/1e6))
    print("%d flows  %d BBR  %d rounds" % (args.flows, args.bbr, args.rounds))
    print("AI=%d%%/RTT  MD=%d/%d*excess/tprop  cwnd_gain=pg" % (int(AI_RATE*100), MD_NUM, MD_DEN))
    print("="*70)

    modes = (["1", "pg", "pg2"] if args.mode == "all" else [args.mode])

    for m in modes:
        n_kcc = args.flows - args.bbr
        flows = [Flow("KCC%d" % i, pg=1.0, cwnd_fn=m) for i in range(n_kcc)]
        for i in range(args.bbr):
            flows.append(Flow("BBR%d" % i, pg=1.0, cwnd_fn="2", is_bbr=True))

        pg_d, q_d, tp_d = simulate(flows, C, T, args.rounds, warmup=args.warmup)

        q = q_d
        avg_q = sum(q) / len(q) * 1000
        srt = sorted(q)
        p95_q = srt[int(len(srt) * 0.95)] * 1000
        p99_q = srt[int(len(srt) * 0.99)] * 1000
        print("\n--- cwnd_gain = %s ---" % m)
        print("  Queue  avg=%.2f ms  p95=%.2f ms  p99=%.2f ms" % (avg_q, p95_q, p99_q))

        total_tp = 0.0
        for f in flows:
            nm = f.name
            avg_p = sum(pg_d[nm]) / len(pg_d[nm])
            avg_t = sum(tp_d[nm]) / len(tp_d[nm]) / 1e6
            total_tp += avg_t
            print("  %8s  pg=%.4f  tp=%.1f Mbps" % (nm, avg_p, avg_t))
        print("  Total  %.0f Mbps  util=%.1f%%" % (total_tp, total_tp/(C/1e6)*100))

    # theory check
    pg_eq = (1 + math.sqrt(1 + 8 * AI_RATE * MD_DEN / MD_NUM)) / 2
    q_eq = (pg_eq - 1) * T * 1000
    print("\n--- Theory ---")
    print("  pg_eq=%.4f  Q_eq=%.3f ms (AI=%.2f, MD_coeff=%.1f)" % (pg_eq, q_eq, AI_RATE, float(MD_NUM)/MD_DEN))
    print("  Q_max(1.25, cwnd=pg):  (1.25-1)*%dms = 15.0 ms" % (T*1000,))
    print("  Q_max(1.25, cwnd=pg2): (1.56-1)*%dms = 33.8 ms" % (T*1000,))


if __name__ == "__main__":
    main()
