#!/usr/bin/env python3
"""
KCC (cwnd_gain=pg, MD=1/1) vs BBR (cwnd_gain=2) mixed-flow competition.
"""
import sys

AI_RATE = 0.02; PG_MIN=0.75; PG_MAX=1.25
MD_NUM, MD_DEN = 1, 1
DRAIN_PERIOD = 64

class Flow:
    __slots__ = ('name','pg','cwnd_fn','throughput','is_bbr','loss')
    def __init__(self, name, pg=1.0, cwnd_fn="pg", is_bbr=False):
        self.name = name; self.pg = pg; self.cwnd_fn = cwnd_fn
        self.throughput = 0.0; self.is_bbr = is_bbr; self.loss = 0.0
    def cwnd_gain(self):
        if self.is_bbr: return 2.0
        if self.cwnd_fn == "1": return 1.0
        if self.cwnd_fn == "pg": return self.pg
        return 1.0

def advance_bbr(flows, rtt_cnt):
    cycle = [1.25, 0.75, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    for f in flows:
        if f.is_bbr:
            f.pg = cycle[rtt_cnt & 7]

def update_round(flows, C, T_prop, rtt_cnt):
    N = len(flows); bdp_per = C * T_prop / N
    total_cwnd = sum(f.cwnd_gain() * bdp_per for f in flows)
    Q = max(0.0, total_cwnd / C - T_prop)

    target_q = T_prop / 128.0
    for f in flows:
        if f.is_bbr: continue
        if Q < target_q:
            f.pg = min(f.pg + AI_RATE, PG_MAX)
        else:
            md = f.pg * Q * MD_NUM / (T_prop * MD_DEN)
            f.pg = max(f.pg - md, PG_MIN)
        if (rtt_cnt & (DRAIN_PERIOD - 1)) == 0:
            f.pg = 0.75

    advance_bbr(flows, rtt_cnt)

    total2 = sum(f.cwnd_gain() * bdp_per for f in flows)
    Q2 = max(0.0, total2 / C - T_prop)
    rtt = T_prop + Q2

    # Tail-drop loss: buffer = 1.5 * BDP
    buf = 1.5 * T_prop
    for f in flows: f.loss = 0.0
    if Q2 > buf:
        drop_fraction = (Q2 - buf) / Q2
        for f in flows:
            f.loss = f.cwnd_gain() * bdp_per * drop_fraction / rtt

    for f in flows:
        f.throughput = f.cwnd_gain() * bdp_per / rtt - f.loss
    return Q2

def sim(N, C, T, rounds, n_bbr=0, warmup=500):
    n_kcc = N - n_bbr
    flows = [Flow("KCC%d"%i) for i in range(n_kcc)]
    imports = [Flow("BBR%d"%i, is_bbr=True) for i in range(n_bbr)]
    flows += imports

    for r in range(warmup):
        update_round(flows, C, T, r)

    qs=[]; pgs={n:[] for n in [f.name for f in flows]}
    tps={n:[] for n in [f.name for f in flows]}
    losses = {n: [] for n in [f.name for f in flows]}
    for r in range(rounds):
        Q = update_round(flows, C, T, r+warmup)
        qs.append(Q)
        for f in flows:
            pgs[f.name].append(f.pg)
            tps[f.name].append(f.throughput)
            losses[f.name].append(f.loss)
    return qs, pgs, tps, losses

def fmt(name, pgs, tps, losses):
    ap = sum(pgs)/len(pgs); at = sum(tps)/len(tps)/1e6
    al = sum(losses)/len(losses)/1e6
    return (name, ap, at, al)

def main():
    C = 1.26e9; T = 0.060; rounds = 5000
    BDP = C * T
    print("="*70)
    print("KCC(cwnd=pg,MD=%d/%d) vs BBR(cwnd=2) competition  C=%.2fGbps T=%dms" % (MD_NUM,MD_DEN,C/1e9,T*1000))
    print("="*70)

    for n_bbr in [0, 1, 2, 4]:
        N = 4
        qs, pgs, tps, loss = sim(N, C, T, rounds, n_bbr=n_bbr)
        avg_q  = sum(qs)*1000/len(qs)
        p95_q  = sorted(qs)[int(len(qs)*0.95)]*1000
        print("\n--- %d KCC + %d BBR ---" % (N-n_bbr, n_bbr))
        print("  Q: avg=%.2f ms  p95=%.2f ms" % (avg_q, p95_q))
        total = 0.0
        for f_name in sorted(pgs.keys()):
            nm, ap, at, al = fmt(f_name, pgs[f_name], tps[f_name], loss[f_name])
            total += at
            is_bb = "BBR" in f_name
            print("  %-8s  pg=%.4f  tp=%.1f Mbps  loss=%.3f Mbps  %s" % (
                f_name, ap, at, al, "(BBR)" if is_bb else ""))
        print("  Total %.0f Mbps  util=%.1f%%" % (total, total/(C/1e6)*100))

if __name__ == "__main__":
    main()
