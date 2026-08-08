"""KCC algorithm simulation: geodesic estimator + state machine.
Exact constants and formulas extracted from tcp_kcc.c."""

KCC_SCALE = 1024
KCC_SCALE_SHIFT = 10

KCC_G2_GROWTH_NUM = 122
KCC_G2_GROWTH_DEN = 1000

KCC_G3_FAST_TH_NUM = 11
KCC_G3_FAST_TH_DEN = 10
KCC_G3_SLOW_TH_NUM = 21
KCC_G3_SLOW_TH_DEN = 20

KCC_G3_FAST_CNT = 4
KCC_G3_SLOW_CNT = 5

KCC_STALENESS_RNDS = 128
KCC_PD_NOISE_GATE_NUM = 95
KCC_PD_NOISE_GATE_DEN = 100

KCC_RTT_MIN_FLOOR_US = 1
KCC_RTT_SAMPLE_MAX_US = 500000

KCC_P_EST_INIT = 1000
KCC_P_EST_FLOOR = 10
KCC_P_EST_DECAY_SHIFT = 4
KCC_P_EST_GROWTH_SHIFT = 3

KCC_EWMA_JITTER_NUM = 7
KCC_EWMA_JITTER_DEN = 8
KCC_MIN_SAMPLES = 5
KCC_JITTER_SEED_SHIFT = 2

KCC_MODE_STARTUP = 0
KCC_MODE_PROBE_BW = 1
KCC_MODE_DRAIN = 2

KCC_CYCLE_LEN = 8
BBR_UNIT = 256
KCC_HIGH_GAIN = BBR_UNIT * 2885 // 1000 + 1
KCC_DRAIN_GAIN = BBR_UNIT * 1000 // 2885
KCC_CWND_GAIN = BBR_UNIT * 2
KCC_FULL_BW_THRESH = 320
KCC_FULL_BW_CNT = 3

KCC_MINRTT_FAST_FALL_CNT = 5
KCC_MINRTT_FAST_FALL_DIV = 4
KCC_MINRTT_STICKY_NUM = 75
KCC_MINRTT_STICKY_DEN = 100
KCC_MINRTT_SRTT_GUARD_NUM = 90
KCC_MINRTT_SRTT_GUARD_DEN = 100

KCC_QDELAY_CLEAN_BP = 1000
KCC_QDELAY_CONG_BP = 2500
KCC_QDELAY_FLOOR_US = 500

KCC_PROBE_CWND_BONUS = 2


class KCCExt:
    def __init__(self):
        self.x_est = 0
        self.p_est = KCC_P_EST_INIT
        self.qdelay_avg = 0
        self.jitter_ewma = 0
        self.sample_cnt = 0
        self.mr_update_rtt_cnt = 0


class KCCState:
    def __init__(self, min_rtt_us=10000):
        self.min_rtt_us = min_rtt_us
        self.mode = KCC_MODE_STARTUP
        self.confirm_cnt = 0
        self.confirm_slow_cnt = 0
        self.full_bw = 0
        self.full_bw_cnt = 0
        self.full_bw_reached = False
        self.rtt_cnt = 0
        self.round_start = False
        self.cycle_idx = 0
        self.cycle_mstamp = 0
        self.ext = KCCExt()
        self.next_round_delivered = 0
        self.prior_cwnd = 0
        self.pacing_gain = BBR_UNIT
        self.cwnd_gain = KCC_CWND_GAIN
        self.lt_use_bw = False
        self.drain_enter_stamp = 0
        self.round_rtt_min = 0xFFFFFFFF
        self.prev_round_rtt_min = 0


def kcc_update(kcc, rtt_us):
    """G1 (downward instant) + G2 (upward capped growth)."""
    ext = kcc.ext
    rtt_us = max(rtt_us, KCC_RTT_MIN_FLOOR_US)
    z = rtt_us << KCC_SCALE_SHIFT

    if ext.sample_cnt == 0:
        ext.x_est = z
        ext.p_est = KCC_P_EST_INIT
        ext.qdelay_avg = 0
        ext.jitter_ewma = max(rtt_us >> KCC_JITTER_SEED_SHIFT, KCC_RTT_MIN_FLOOR_US)
        ext.sample_cnt = 1
        return

    if ext.sample_cnt == 1:
        if kcc.min_rtt_us:
            ceiling = kcc.min_rtt_us << KCC_SCALE_SHIFT
            if ext.x_est > ceiling:
                ext.x_est = ceiling

    innovation = z - ext.x_est
    abs_innov = abs(innovation)

    if innovation <= 0:
        ext.x_est = min(ext.x_est, z)
    else:
        growth = ext.x_est * KCC_G2_GROWTH_NUM // KCC_G2_GROWTH_DEN
        ext.x_est = min(ext.x_est + growth, z)

    ext.sample_cnt += 1

    ext.jitter_ewma = (ext.jitter_ewma * KCC_EWMA_JITTER_NUM +
                       abs_innov * (KCC_EWMA_JITTER_DEN - KCC_EWMA_JITTER_NUM)) // KCC_EWMA_JITTER_DEN


def kcc_update_min_rtt(kcc, rtt_us):
    """G3 dual-threshold path-increase detection + min_rtt update."""
    ext = kcc.ext

    kcc_update(kcc, rtt_us)

    mr_scaled = kcc.min_rtt_us << KCC_SCALE_SHIFT

    if ext.x_est >= mr_scaled * KCC_G3_FAST_TH_NUM // KCC_G3_FAST_TH_DEN:
        kcc.confirm_cnt = min(kcc.confirm_cnt + 1, 7)
        kcc.confirm_slow_cnt = min(kcc.confirm_slow_cnt + 1, 7)
    elif ext.x_est >= mr_scaled * KCC_G3_SLOW_TH_NUM // KCC_G3_SLOW_TH_DEN:
        kcc.confirm_cnt = 0
        kcc.confirm_slow_cnt = min(kcc.confirm_slow_cnt + 1, 7)
    else:
        kcc.confirm_cnt = 0

    if ext.x_est <= mr_scaled:
        kcc.confirm_cnt = 0
        kcc.confirm_slow_cnt = 0

    if kcc.confirm_cnt >= KCC_G3_FAST_CNT:
        kcc.min_rtt_us = ext.x_est >> KCC_SCALE_SHIFT
        kcc.confirm_cnt = 0
        kcc.confirm_slow_cnt = 0
        ext.p_est = KCC_P_EST_INIT
        return True
    elif kcc.confirm_slow_cnt >= KCC_G3_SLOW_CNT:
        kcc.min_rtt_us = ext.x_est >> KCC_SCALE_SHIFT
        kcc.confirm_cnt = 0
        kcc.confirm_slow_cnt = 0
        ext.p_est = KCC_P_EST_INIT
        return True

    if kcc.confirm_cnt > 0 or kcc.confirm_slow_cnt > 0:
        return False

    return False


def get_model_rtt(kcc):
    """Model RTT: min(x_est>>shift, min_rtt_us) with cold-start guard."""
    ext = kcc.ext
    if not ext or not ext.x_est or ext.sample_cnt < KCC_MIN_SAMPLES:
        return kcc.min_rtt_us
    return min(ext.x_est >> KCC_SCALE_SHIFT, kcc.min_rtt_us)


def pacing_gain_for_phase(kcc):
    """BBR pacing gain array."""
    gains = [
        BBR_UNIT * 5 // 4,
        BBR_UNIT * 3 // 4,
        BBR_UNIT, BBR_UNIT, BBR_UNIT,
        BBR_UNIT, BBR_UNIT, BBR_UNIT
    ]
    if kcc.mode == KCC_MODE_STARTUP:
        return KCC_HIGH_GAIN
    elif kcc.mode == KCC_MODE_DRAIN:
        return KCC_DRAIN_GAIN
    else:
        return gains[kcc.cycle_idx % KCC_CYCLE_LEN]


def run_geodesic_test(rtt_samples, true_t_prop=None, label=""):
    """Run the geodesic estimator on a sequence of RTT samples."""
    kcc = KCCState(min_rtt_us=10000)
    results = {"x_est": [], "min_rtt_us": [], "confirm_cnt": [], "confirm_slow_cnt": []}

    for rtt in rtt_samples:
        kcc_update_min_rtt(kcc, rtt)
        results["x_est"].append(kcc.ext.x_est >> KCC_SCALE_SHIFT)
        results["min_rtt_us"].append(kcc.min_rtt_us)
        results["confirm_cnt"].append(kcc.confirm_cnt)
        results["confirm_slow_cnt"].append(kcc.confirm_slow_cnt)

    return results
