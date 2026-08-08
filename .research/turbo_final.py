import sys
import math

class Flow:
    __slots__ = ('n','pg','tp','bbr','turbo')
    def __init__(self, n, bbr=False, turbo=False):
        self.n=n; self.pg=1.0; self.tp=0; self.bbr=bbr; self.turbo=turbo
    def cg(self):
        if self.bbr: return 2.0
        floor = 1.85 if self.turbo else 1.0
        return max(floor, self.pg)

def upd(fl, C, T, rnd):
    N=len(fl); b=C*T/N
    tc = sum(f.cg()*b for f in fl); Q = max(0., tc/C - T)
    for f in fl:
        if f.bbr: continue
        if Q < T/128: f.pg = min(f.pg + 0.02, 1.25)
        else: md = f.pg*Q*1.0/T; f.pg = max(f.pg - md, 0.75)
        if (rnd & 127) == 0: f.pg = 0.75
    cyc=[1.25,0.75,1,1,1,1,1,1]
    for f in fl:
        if f.bbr: f.pg=cyc[rnd&7]
    t2 = sum(f.cg()*b for f in fl); Q2 = max(0., t2/C - T)
    rt = T + Q2
    for f in fl: f.tp = f.cg()*b/rt
    return Q2

def run_pure(N, C, T, R, warmup, turbo):
    fl = [Flow(i, bbr=False, turbo=turbo) for i in range(N)]
    Qs = []
    loss_bytes = 0.0
    buf = C * T * 2
    for rnd in range(R):
        Q = upd(fl, C, T, rnd)
        if rnd >= warmup:
            Qs.append(Q)
            qb = Q * C
            if qb > buf:
                loss_bytes += (qb - buf)
    Q_avg = sum(Qs)/len(Qs)
    sQs = sorted(Qs)
    Q_p95 = sQs[int(len(sQs)*0.95)]
    total_tp = sum(f.tp for f in fl)
    Util = total_tp / C * 100
    total_time = (R - warmup) * T
    loss_Mbps = loss_bytes * 8 / total_time / 1e6 if total_time > 0 else 0
    return Q_avg*1000, Q_p95*1000, Util, loss_Mbps

def run_mixed(N, C, T, R, warmup, n_bbr, turbo):
    fl = []
    for i in range(N):
        fl.append(Flow(i, bbr=(i < n_bbr), turbo=(turbo and i >= n_bbr)))
    Qs = []
    pgs_log = []
    for rnd in range(R):
        Q = upd(fl, C, T, rnd)
        if rnd >= warmup:
            Qs.append(Q)
            if rnd % 10 == 0:
                pgs_log.append([f.pg for f in fl])
    kcc_tp = sum(f.tp for f in fl if not f.bbr) / max(1, len([f for f in fl if not f.bbr]))
    bbr_tp = sum(f.tp for f in fl if f.bbr) / max(1, len([f for f in fl if f.bbr]))
    fairness = kcc_tp / max(bbr_tp, 1e-12)
    Q_avg_ms = sum(Qs)/len(Qs)*1000
    return kcc_tp, bbr_tp, fairness, Q_avg_ms, pgs_log

def main():
    C = 1.26e9
    T = 0.060
    R = 1000
    warmup = 300
    flow_counts = [1,2,4,8,16,32,64,128,256,512,1024]

    print("="*120)
    print("KCC/BBR CONGESTION CONTROL SIMULATION")
    print("C=%.3f Gbps, T=%dms, R=%d rounds, warmup=%d" % (C/1e9, int(T*1000), R, warmup))
    print("AI=0.02/round, MD=1.0, PG_MIN=0.75, PG_MAX=1.25, DRAIN_PERIOD=%d" % 128)
    print("KCC ECO: cwnd_gain = max(1.0, pg)")
    print("KCC TURBO: cwnd_gain = max(1.85, pg)")
    print("="*120)

    all_results = []

    # === PURE KCC ===
    print("\n" + "="*120)
    print("SCENARIO 1: PURE KCC (All flows are KCC)")
    print("="*120)
    print("%-8s %-8s %-14s %-14s %-12s %-16s" % ("Flows", "Mode", "Q_avg(ms)", "Q_p95(ms)", "Util(%)", "Loss(Mbps)"))
    print("-"*120)
    for N in flow_counts:
        for mode, turbo in [("ECO", False), ("TURBO", True)]:
            Qa, Qp, Ut, Ls = run_pure(N, C, T, R, warmup, turbo)
            print("%-8d %-8s %-14.4f %-14.4f %-12.2f %-16.8f" % (N, mode, Qa, Qp, Ut, Ls))
            all_results.append(("pure", N, mode, Qa, Qp, Ut, Ls))

    # === MIXED 1 BBR + N-1 KCC ===
    print("\n" + "="*120)
    print("SCENARIO 2: MIXED (1 BBR + N-1 KCC)")
    print("="*120)
    print("%-8s %-8s %-18s %-18s %-18s %-14s" % ("Flows", "Mode", "KCC_tp_avg(bps)", "BBR_tp(bps)", "Fairness(K/B)", "Q_avg(ms)"))
    print("-"*120)
    for N in flow_counts:
        if N < 2: continue
        for mode, turbo in [("ECO", False), ("TURBO", True)]:
            ktp, btp, fair, qa, _ = run_mixed(N, C, T, R, warmup, 1, turbo)
            print("%-8d %-8s %-18.2e %-18.2e %-18.4f %-14.3f" % (N, mode, ktp, btp, fair, qa))
            all_results.append(("mixed_1bbr", N, mode, ktp, btp, fair, qa))

    # === MIXED N/2 BBR + N/2 KCC ===
    print("\n" + "="*120)
    print("SCENARIO 3: MIXED (N/2 BBR + N/2 KCC) - only N>=4")
    print("="*120)
    print("%-8s %-8s %-18s %-18s %-18s %-14s" % ("Flows", "Mode", "KCC_tp_avg(bps)", "BBR_tp(bps)", "Fairness(K/B)", "Q_avg(ms)"))
    print("-"*120)
    for N in flow_counts:
        if N < 4: continue
        n_bbr = N // 2
        for mode, turbo in [("ECO", False), ("TURBO", True)]:
            ktp, btp, fair, qa, _ = run_mixed(N, C, T, R, warmup, n_bbr, turbo)
            print("%-8d %-8s %-18.2e %-18.2e %-18.4f %-14.3f" % (N, mode, ktp, btp, fair, qa))
            all_results.append(("mixed_half", N, mode, ktp, btp, fair, qa))

    # === SUMMARY ANALYSIS ===
    print("\n" + "="*120)
    print("SUMMARY ANALYSIS")
    print("="*120)

    pure_results = [r for r in all_results if r[0] == "pure"]
    mixed_results = [r for r in all_results if r[0] != "pure"]

    # 1. Best mode for pure KCC
    eco_pure = [r for r in pure_results if r[2] == "ECO"]
    turbo_pure = [r for r in pure_results if r[2] == "TURBO"]
    eco_avg_util = sum(r[5] for r in eco_pure) / len(eco_pure) if eco_pure else 0
    turbo_avg_util = sum(r[5] for r in turbo_pure) / len(turbo_pure) if turbo_pure else 0
    eco_avg_q = sum(r[3] for r in eco_pure) / len(eco_pure) if eco_pure else 0
    turbo_avg_q = sum(r[3] for r in turbo_pure) / len(turbo_pure) if turbo_pure else 0
    eco_avg_loss = sum(r[6] for r in eco_pure) / len(eco_pure) if eco_pure else 0
    turbo_avg_loss = sum(r[6] for r in turbo_pure) / len(turbo_pure) if turbo_pure else 0

    print("\n1. BEST MODE FOR PURE KCC:")
    if abs(eco_avg_util - turbo_avg_util) < 0.5:
        if eco_avg_q < turbo_avg_q and eco_avg_loss <= turbo_avg_loss:
            print("   ECO mode recommended (lower avg Q=%.4fms vs TURBO %.4fms, util both ~%.2f%%)" % (eco_avg_q, turbo_avg_q, eco_avg_util))
        elif turbo_avg_q < eco_avg_q and turbo_avg_loss <= eco_avg_loss:
            print("   TURBO mode recommended (lower avg Q=%.4fms vs ECO %.4fms, util both ~%.2f%%)" % (turbo_avg_q, eco_avg_q, turbo_avg_util))
        else:
            print("   Similar: ECO (Q=%.4fms, util=%.2f%%, loss=%.8f) vs TURBO (Q=%.4fms, util=%.2f%%, loss=%.8f)" % (eco_avg_q, eco_avg_util, eco_avg_loss, turbo_avg_q, turbo_avg_util, turbo_avg_loss))
    elif eco_avg_util > turbo_avg_util:
        print("   ECO mode (util=%.2f%% vs TURBO %.2f%%)" % (eco_avg_util, turbo_avg_util))
    else:
        print("   TURBO mode (util=%.2f%% vs ECO %.2f%%)" % (turbo_avg_util, eco_avg_util))

    # 2. Best mode for mixed BBR
    eco_mixed = [r for r in mixed_results if r[2] == "ECO"]
    turbo_mixed = [r for r in mixed_results if r[2] == "TURBO"]
    eco_fair_vals = [r[5] for r in eco_mixed]
    turbo_fair_vals = [r[5] for r in turbo_mixed]
    eco_fair_avg = sum(eco_fair_vals)/len(eco_fair_vals) if eco_fair_vals else 0
    turbo_fair_avg = sum(turbo_fair_vals)/len(turbo_fair_vals) if turbo_fair_vals else 0
    eco_fair_min = min(eco_fair_vals) if eco_fair_vals else 1
    turbo_fair_min = min(turbo_fair_vals) if turbo_fair_vals else 1

    print("\n2. BEST MODE FOR MIXED BBR:")
    if eco_fair_avg > turbo_fair_avg + 0.05:
        print("   ECO mode (avg fairness=%.4f, min=%.4f) vs TURBO (avg=%.4f, min=%.4f)" % (eco_fair_avg, eco_fair_min, turbo_fair_avg, turbo_fair_min))
    elif turbo_fair_avg > eco_fair_avg + 0.05:
        print("   TURBO mode (avg fairness=%.4f, min=%.4f) vs ECO (avg=%.4f, min=%.4f)" % (turbo_fair_avg, turbo_fair_min, eco_fair_avg, eco_fair_min))
    else:
        print("   Similar fairness: ECO (avg=%.4f, min=%.4f) vs TURBO (avg=%.4f, min=%.4f)" % (eco_fair_avg, eco_fair_min, turbo_fair_avg, turbo_fair_min))

    # 3. Maximum queue
    max_q = 0.0
    max_q_desc = ""
    for r in all_results:
        if r[0] == "pure":
            q = r[3]
            desc = "Pure %s N=%d" % (r[2], r[1])
        else:
            q = r[6]
            desc = "%s %s N=%d" % (r[0], r[2], r[1])
        if q > max_q:
            max_q = q
            max_q_desc = desc

    print("\n3. MAXIMUM QUEUE (avg Q_avg) ACROSS ALL SCENARIOS: %.4f ms (%s)" % (max_q, max_q_desc))

    # 4. Minimum fairness
    min_fair = float('inf')
    min_fair_desc = ""
    for r in mixed_results:
        f = r[5]
        if f < min_fair:
            min_fair = f
            min_fair_desc = "%s %s N=%d" % (r[0], r[2], r[1])

    print("\n4. MINIMUM FAIRNESS (KCC/BBR) ACROSS ALL MIXED SCENARIOS: %.4f (%s)" % (min_fair, min_fair_desc))

    # 5. Anomalies
    print("\n5. ANOMALIES:")
    anomalies = []
    for r in pure_results:
        if r[5] < 95.0:
            anomalies.append("Low util in pure KCC: N=%d, Mode=%s, Util=%.2f%%" % (r[1], r[2], r[5]))
        if r[6] > 1.0:
            anomalies.append("High loss in pure KCC: N=%d, Mode=%s, Loss=%.4f Mbps" % (r[1], r[2], r[6]))
    for r in mixed_results:
        if r[5] < 0.5:
            anomalies.append("KCC starvation in %s: N=%d, Mode=%s, Fairness=%.4f" % (r[0], r[1], r[2], r[5]))
        if r[5] > 2.0:
            anomalies.append("KCC dominance in %s: N=%d, Mode=%s, Fairness=%.4f" % (r[0], r[1], r[2], r[5]))
    if not anomalies:
        print("   No significant anomalies detected.")
    else:
        for a in anomalies:
            print("   * " + a)

    # Check PG stuck for some scenarios
    print("\n   PG stuck check (spot-check N=64 mixed_1bbr):")
    for mode, turbo in [("ECO", False), ("TURBO", True)]:
        _, _, _, _, pgs_log = run_mixed(64, C, T, R, warmup, 1, turbo)
        kcc_pgs = [pgs[1] for pgs in pgs_log]  # first KCC flow pg values
        min_pg = min(kcc_pgs)
        max_pg = max(kcc_pgs)
        stuck_at_min = sum(1 for p in kcc_pgs if p <= 0.751)
        stuck_at_max = sum(1 for p in kcc_pgs if p >= 1.249)
        print("      %s: pg range [%.4f, %.4f], rounds at min=%d, at max=%d" % (mode, min_pg, max_pg, stuck_at_min, stuck_at_max))

    print("\n" + "="*120)
    print("END OF REPORT")
    print("="*120)

if __name__ == "__main__":
    main()
