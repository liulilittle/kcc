#!/usr/bin/env python3
"""
Sweep MD coefficient to find stable range for cwnd_gain=pg.
"""
import sys, math

BBR_UNIT = 1.0; PG_MIN=0.75; PG_MAX=1.25; AI_RATE=0.02
DRAIN_PERIOD = 64

class Flow:
    __slots__ = ('pg','throughput')
    def __init__(self, pg=1.0):
        self.pg = pg; self.throughput = 0.0
    def cwnd_gain(self): return self.pg

def update_round(flows, C, T_prop, rtt_cnt, md_num, md_den):
    N = len(flows); bdp_per = C * T_prop / N
    total_cwnd = sum(f.cwnd_gain() * bdp_per for f in flows)
    Q = max(0.0, total_cwnd / C - T_prop)
    for f in flows:
        if Q < T_prop / 128.0:
            f.pg = min(f.pg + AI_RATE, PG_MAX)
        else:
            md = f.pg * Q * md_num / (T_prop * md_den)
            f.pg = max(f.pg - md, PG_MIN)
        if (rtt_cnt & (DRAIN_PERIOD - 1)) == 0:
            f.pg = 0.75
    total2 = sum(f.cwnd_gain() * bdp_per for f in flows)
    Q2 = max(0.0, total2 / C - T_prop)
    rtt = T_prop + Q2
    for f in flows: f.throughput = f.cwnd_gain() * bdp_per / rtt
    return Q2

def sim(N, C, T, rounds, md_num, md_den, warmup=500):
    flows = [Flow() for _ in range(N)]
    for r in range(warmup):
        update_round(flows, C, T, r, md_num, md_den)
    qs = []; pgs = [[] for _ in flows]; tps = [[] for _ in flows]
    for r in range(rounds):
        Q = update_round(flows, C, T, r + warmup, md_num, md_den)
        qs.append(Q)
        for i, f in enumerate(flows):
            pgs[i].append(f.pg); tps[i].append(f.throughput)
    avg_q = sum(qs)/len(qs)*1000
    p95_q = sorted(qs)[int(len(qs)*0.95)]*1000
    avg_pg = sum(sum(p)/len(p) for p in pgs) / N
    avg_tp = sum(sum(t)/len(t) for t in tps) / 1e6
    return avg_q, p95_q, avg_pg, avg_tp

def main():
    C = 1.26e9; T = 0.060; N = 4; rounds = 3000
    print("%-10s  %-8s  %-8s  %-8s  %-8s  %-8s" % ("MD_NUM/DEN","coeff","avgQ_ms","p95Q_ms","avgPG","tp_Mbps"))
    print("-"*58)
    for (num,den) in [(1,4),(1,3),(1,2),(1,1),(3,2),(2,1),(5,2)]:
        coeff = float(num)/den
        qa, qp, pg, tp = sim(N, C, T, rounds, num, den)
        print("  %d/%-6d  %-8.2f  %-8.2f  %-8.2f  %-8.4f  %-8.0f" % (num,den,coeff,qa,qp,pg,tp))

    print("\nTheory equilibrium (AI=2%, cwnd_gain=pg):")
    for (num,den) in [(1,4),(1,3),(1,2),(1,1),(3,2),(2,1),(5,2)]:
        c = float(num)/den
        # pg*(pg-1)*c = AI  =>  pg^2 - pg - AI/c = 0
        disc = 1 + 4 * AI_RATE / c
        if disc >= 0:
            pg_eq = (1 + math.sqrt(disc)) / 2
            q_eq = (pg_eq - 1) * T * 1000
            print("  coeff=%.2f  pg_eq=%.4f  Q_eq=%.2f ms" % (c, pg_eq, q_eq))

if __name__ == "__main__":
    main()
