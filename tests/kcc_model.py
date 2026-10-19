#!/usr/bin/env python3
"""
kcc_model.py -- reference model mirroring the key tcp_kcc.c paths (post-fix).

Purpose: the "system under test" for TDD.  Every model rule maps to a specific tcp_kcc.c line.
Whenever the C code changes, the model must change with it, so the tests reflect kernel behaviour.

Mapping (post-fix):
  kcc_update()            G1/G2(per round)/staleness/qdelay(min_rtt baseline)/jitter/p_est
  kcc_update_min_rtt()    locked(+release)/G3 per round prev_round_rtt_min/legacy/sticky/SRTT guard/takeover
  kcc_get_model_rtt()     G4: confirmed min_rtt only
  kcc_bdp() / kcc_quantization_budget() / kcc_inflight()
  kcc_is_next_cycle_phase() / kcc_check_drain() / kcc_ack_aggregation_cwnd()
"""
from __future__ import annotations

KCC_SCALE = 1024
KCC_SCALE_SHIFT = 10
BW_SCALE = 24
BW_UNIT = 1 << BW_SCALE
BBR_SCALE = 8
BBR_UNIT = 1 << BBR_SCALE
KCC_G2_GROWTH_NUM = 122
KCC_G2_GROWTH_DEN = 1000
KCC_G3_FAST_TH_NUM, KCC_G3_FAST_TH_DEN = 11, 10
KCC_G3_SLOW_TH_NUM, KCC_G3_SLOW_TH_DEN = 21, 20
KCC_G3_FAST_CNT = 6
KCC_G3_SLOW_CNT = 7
KCC_BITFIELD_3BIT_MAX = 7
KCC_LOCK_THRESH_US = 5000
KCC_FAST_ONLY_THRESH_US = 7500
KCC_STALENESS_RNDS = 128
KCC_PD_NOISE_GATE_NUM, KCC_PD_NOISE_GATE_DEN = 95, 100
KCC_MINRTT_STICKY_NUM, KCC_MINRTT_STICKY_DEN = 75, 100
KCC_MINRTT_FAST_FALL_DIV = 4
KCC_MINRTT_FAST_FALL_FAST_CNT = 2
KCC_MINRTT_FAST_FALL_CNT = 5
KCC_MINRTT_SRTT_GUARD_NUM, KCC_MINRTT_SRTT_GUARD_DEN = 90, 100
KCC_SRTT_SUPPRESS_RNDS = 24
KCC_RTT_MIN_FLOOR_US = 1
KCC_RTT_SAMPLE_MAX_US = 500000
KCC_MIN_SAMPLES = 5
KCC_P_EST_INIT = 1000
KCC_P_EST_FLOOR = 10
KCC_P_EST_DECAY_SHIFT = 4
KCC_P_EST_GROWTH_SHIFT = 3
KCC_P_EST_MAX = 1000000
KCC_EWMA_JITTER_NUM, KCC_EWMA_JITTER_DEN = 7, 8
KCC_EWMA_QDELAY_NUM, KCC_EWMA_QDELAY_DEN = 7, 8
KCC_JITTER_SEED_SHIFT = 2
KCC_CWND_GAIN = BBR_UNIT * 2
KCC_CWND_MIN_TARGET = 4
KCC_TSO_HEADROOM_MULT = 3
KCC_MIN_TSO_RATE = 1200000
KCC_MIN_TSO_RATE_DIV = 8
KCC_TSO_HIGH_JITTER_THRESH_US = 4000
KCC_TSO_SEGS_DEFAULT = 2
KCC_TSO_SEGS_LOW = 1
KCC_TSO_DIV_HALVE_SHIFT = 1
KCC_PROBE_CWND_BONUS = 2
KCC_EXTRA_ACKED_MAX_MS_RATIO = 100
KCC_DEFAULT_RTT_US = 1000
KCC_HIGH_GAIN = BBR_UNIT * 2885 // 1000 + 1
KCC_DRAIN_GAIN = BBR_UNIT * 1000 // 2885
KCC_FULL_BW_THRESH = 320
KCC_FULL_BW_CNT = 3
KCC_CYCLE_LEN = 8

MODE_STARTUP, MODE_PROBE_BW, MODE_DRAIN = 0, 1, 2


class Ext:
    def __init__(self):
        self.x_est = 0
        self.mr_update_rtt_cnt = 0
        self.p_est = KCC_P_EST_INIT
        self.qdelay_avg = 0
        self.sample_cnt = 0
        self.jitter_ewma = 0
        self.extra_acked = [0, 0]
        self.ack_epoch_mstamp = 0
        self.ack_epoch_acked = 0
        self.extra_acked_win_rtts = 0
        self.extra_acked_win_idx = 0
        self.x_est_fall_cnt = 0
        self.srtt_suppress_cnt = 0


class Kcc:
    def __init__(self, t_prop_us=20000):
        self.min_rtt_us = t_prop_us
        self.min_rtt_stamp = 0
        self.rtt_cnt = 0
        self.next_rtt_delivered = 0
        self.round_start = 0
        self.mode = MODE_STARTUP
        self.locked = 0
        self.locked_high_cnt = 0
        self.confirm_cnt = 0
        self.confirm_slow_cnt = 0
        self.srtt_guard_cnt = 0
        self.prior_in_flight = 0
        self.bw_now = 0
        self.min_rtt_fast_fall_cnt = 0
        self.cycle_idx = 0
        self.full_bw_reached = 0
        self.pacing_gain = KCC_HIGH_GAIN
        self.cwnd_gain = KCC_HIGH_GAIN
        self.round_rtt_min = 0xFFFFFFFF
        self.prev_round_rtt_min = 0xFFFFFFFF
        self.ext = Ext()
        self.srtt_us = 0
        self.snd_cwnd = 10
        self.delivered = 0
        self.mss_cache = 1448
        self.sk_pacing_rate = 0
        self.lt_use_bw = 0
        self.lt_bw = 0
        self.g3_fires = []
        self.has_seen_rtt = 0
        self.default_rtt = False   # True = min_rtt is the 1ms placeholder

    # ---------- kcc_update ----------
    def update(self, rtt_us: int):
        ext = self.ext
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
            if self.min_rtt_us:
                ceiling = self.min_rtt_us << KCC_SCALE_SHIFT
                if ext.x_est > ceiling:
                    ext.x_est = ceiling

        innovation = z - ext.x_est
        abs_innov = abs(innovation)

        # G1: instant downward min
        if innovation <= 0:
            ext.x_est = min(ext.x_est, z)
        # G2: once per round (12.2% per RTT)
        elif self.round_start:
            growth = ext.x_est * KCC_G2_GROWTH_NUM // KCC_G2_GROWTH_DEN
            ext.x_est = min(ext.x_est + growth, z)

        # staleness: downward-only correction, floored at 1us
        if self.rtt_cnt - ext.mr_update_rtt_cnt >= KCC_STALENESS_RNDS:
            mr_scaled = self.min_rtt_us << KCC_SCALE_SHIFT
            if ext.x_est <= mr_scaled * KCC_G3_FAST_TH_NUM // KCC_G3_FAST_TH_DEN:
                ext.x_est = max(KCC_SCALE,
                                min(ext.x_est,
                                    mr_scaled * KCC_PD_NOISE_GATE_NUM // KCC_PD_NOISE_GATE_DEN))
                ext.mr_update_rtt_cnt = self.rtt_cnt

        raw_jitter = min(abs_innov >> KCC_SCALE_SHIFT, 0xFFFFFFFF)
        if ext.sample_cnt > 1:
            ext.jitter_ewma = (ext.jitter_ewma * KCC_EWMA_JITTER_NUM + raw_jitter) // KCC_EWMA_JITTER_DEN
        else:
            ext.jitter_ewma = raw_jitter
        ext.jitter_ewma = min(ext.jitter_ewma, max(self.min_rtt_us, KCC_RTT_SAMPLE_MAX_US))

        # qdelay: stable baseline = min_rtt
        t_prop_scaled = self.min_rtt_us << KCC_SCALE_SHIFT
        qdelay_instant = ((z - t_prop_scaled) >> KCC_SCALE_SHIFT) if z > t_prop_scaled else 0
        if ext.sample_cnt == 1:
            ext.qdelay_avg = qdelay_instant
        else:
            ext.qdelay_avg = (ext.qdelay_avg * KCC_EWMA_QDELAY_NUM + qdelay_instant) // KCC_EWMA_QDELAY_DEN

        if ext.sample_cnt < 0xFFFFFFFF:
            ext.sample_cnt += 1

        if ext.sample_cnt >= KCC_MIN_SAMPLES:
            x_est_us = ext.x_est >> KCC_SCALE_SHIFT
            if (x_est_us <= self.min_rtt_us * KCC_G3_SLOW_TH_NUM // KCC_G3_SLOW_TH_DEN
                    and self.confirm_cnt == 0 and self.confirm_slow_cnt == 0):
                delta = (ext.p_est - KCC_P_EST_FLOOR) >> KCC_P_EST_DECAY_SHIFT if ext.p_est > KCC_P_EST_FLOOR else 0
                if ext.p_est > KCC_P_EST_FLOOR + delta:
                    ext.p_est -= max(delta, 1)
            elif x_est_us > self.min_rtt_us * KCC_G3_FAST_TH_NUM // KCC_G3_FAST_TH_DEN:
                delta = (KCC_P_EST_INIT - ext.p_est) >> KCC_P_EST_GROWTH_SHIFT if ext.p_est < KCC_P_EST_INIT else 0
                if ext.p_est + delta < KCC_P_EST_MAX:
                    ext.p_est += max(delta, 1)

    # ---------- kcc_update_min_rtt ----------
    def update_min_rtt(self, rtt_us: int, round_start: bool = False, is_ack_delayed: bool = False):
        if rtt_us <= 0:
            return
        self.round_start = 1 if round_start else 0
        rtt_clamped = rtt_us
        mr_snapshot = self.min_rtt_us

        self.update(rtt_clamped)

        # Cold start: adopt the first real measurement when init had no
        # RTT sample and min_rtt still holds the 1ms placeholder.
        if (self.ext.sample_cnt == 1 and not self.has_seen_rtt
                and self.min_rtt_us == KCC_DEFAULT_RTT_US):
            self.min_rtt_us = rtt_clamped

        if rtt_clamped < KCC_LOCK_THRESH_US and self.min_rtt_us <= KCC_LOCK_THRESH_US:
            self.locked = 1

        # ---- G3: per-round, decision statistic = prev_round_rtt_min ----
        if round_start:
            round_min = self.prev_round_rtt_min
            if round_min == 0 or round_min == 0xFFFFFFFF:
                self.confirm_cnt = 0
                self.confirm_slow_cnt = 0
            elif self.locked:
                if round_min >= KCC_FAST_ONLY_THRESH_US:
                    self.locked_high_cnt = min(self.locked_high_cnt + 1, KCC_BITFIELD_3BIT_MAX)
                    if self.locked_high_cnt >= KCC_BITFIELD_3BIT_MAX:
                        self.locked = 0
                        self.locked_high_cnt = 0
                        self.min_rtt_us = round_min
                        self.ext.srtt_suppress_cnt = KCC_SRTT_SUPPRESS_RNDS
                        self.srtt_guard_cnt = 0
                else:
                    self.locked_high_cnt = 0
                self.confirm_cnt = 0
                self.confirm_slow_cnt = 0
            else:
                stat = round_min << KCC_SCALE_SHIFT
                if round_min >= KCC_FAST_ONLY_THRESH_US:
                    if stat >= self.min_rtt_us * KCC_SCALE * KCC_G3_FAST_TH_NUM // KCC_G3_FAST_TH_DEN:
                        self.confirm_cnt = min(self.confirm_cnt + 1, KCC_BITFIELD_3BIT_MAX)
                    else:
                        self.confirm_cnt = 0
                    if stat >= self.min_rtt_us * KCC_SCALE * KCC_G3_SLOW_TH_NUM // KCC_G3_SLOW_TH_DEN:
                        self.confirm_slow_cnt = min(self.confirm_slow_cnt + 1, KCC_BITFIELD_3BIT_MAX)
                    else:
                        self.confirm_slow_cnt = 0
                elif round_min >= KCC_LOCK_THRESH_US:
                    if stat >= self.min_rtt_us * KCC_SCALE * KCC_G3_FAST_TH_NUM // KCC_G3_FAST_TH_DEN:
                        self.confirm_cnt = min(self.confirm_cnt + 1, KCC_BITFIELD_3BIT_MAX)
                    else:
                        self.confirm_cnt = 0
                    self.confirm_slow_cnt = 0
                else:
                    self.confirm_cnt = 0
                    self.confirm_slow_cnt = 0

                if stat <= self.min_rtt_us * KCC_SCALE:
                    self.confirm_cnt = 0
                    self.confirm_slow_cnt = 0

                if self.confirm_cnt >= KCC_G3_FAST_CNT:
                    self.min_rtt_us = round_min
                    self.confirm_cnt = 0
                    self.confirm_slow_cnt = 0
                    self.ext.p_est = KCC_P_EST_INIT
                    self.ext.mr_update_rtt_cnt = self.rtt_cnt
                    self.ext.srtt_suppress_cnt = KCC_SRTT_SUPPRESS_RNDS
                    self.srtt_guard_cnt = 0
                    self.g3_fires.append((self.rtt_cnt, "fast", self.min_rtt_us))
                elif self.confirm_slow_cnt >= KCC_G3_SLOW_CNT:
                    self.min_rtt_us = round_min
                    self.confirm_cnt = 0
                    self.confirm_slow_cnt = 0
                    self.ext.p_est = KCC_P_EST_INIT
                    self.ext.mr_update_rtt_cnt = self.rtt_cnt
                    self.ext.srtt_suppress_cnt = KCC_SRTT_SUPPRESS_RNDS
                    self.srtt_guard_cnt = 0
                    self.g3_fires.append((self.rtt_cnt, "slow", self.min_rtt_us))

        # ---- traditional min_rtt (no sticky pre-reduction) ----
        if rtt_clamped <= self.min_rtt_us:
            r = max(rtt_clamped, KCC_RTT_MIN_FLOOR_US)
            if r < self.min_rtt_us * KCC_MINRTT_STICKY_NUM // KCC_MINRTT_STICKY_DEN:
                if round_start:
                    self.min_rtt_fast_fall_cnt = min(self.min_rtt_fast_fall_cnt + 1, KCC_BITFIELD_3BIT_MAX)
                if r < self.min_rtt_us // KCC_MINRTT_FAST_FALL_DIV:
                    if self.min_rtt_fast_fall_cnt >= KCC_MINRTT_FAST_FALL_FAST_CNT:
                        self.min_rtt_us = r
                        self.min_rtt_fast_fall_cnt = 0
                elif self.min_rtt_fast_fall_cnt >= KCC_MINRTT_FAST_FALL_CNT:
                    self.min_rtt_us = r
                    self.min_rtt_fast_fall_cnt = 0
            else:
                self.min_rtt_us = r
                self.min_rtt_fast_fall_cnt = 0
            self.min_rtt_stamp = 1
        elif rtt_clamped >= self.min_rtt_us:
            self.min_rtt_fast_fall_cnt = 0

        # ---- SRTT guard (per-round, confirmed, suppressed after G3) ----
        if self.ext.srtt_suppress_cnt:
            if round_start:
                self.ext.srtt_suppress_cnt -= 1
            self.srtt_guard_cnt = 0
        elif self.srtt_us and self.min_rtt_us:
            srtt_shifted = max(self.srtt_us >> 3, KCC_RTT_MIN_FLOOR_US)
            if srtt_shifted < self.min_rtt_us * KCC_MINRTT_SRTT_GUARD_NUM // KCC_MINRTT_SRTT_GUARD_DEN:
                if round_start and self.srtt_guard_cnt < KCC_BITFIELD_3BIT_MAX:
                    self.srtt_guard_cnt += 1
                if self.srtt_guard_cnt >= KCC_MINRTT_FAST_FALL_CNT:
                    self.min_rtt_us = srtt_shifted
                    self.min_rtt_stamp = 1
                    self.srtt_guard_cnt = 0
            else:
                self.srtt_guard_cnt = 0

        # ---- downward path-change detection (per-round statistic) ----
        if (round_start and self.prev_round_rtt_min != 0
                and self.prev_round_rtt_min != 0xFFFFFFFF):
            rm = self.prev_round_rtt_min
            if rm < self.min_rtt_us and rm < self.min_rtt_us * KCC_PD_NOISE_GATE_NUM // KCC_PD_NOISE_GATE_DEN:
                if self.ext.x_est_fall_cnt < KCC_MINRTT_FAST_FALL_CNT:
                    self.ext.x_est_fall_cnt += 1
                if self.ext.x_est_fall_cnt >= KCC_MINRTT_FAST_FALL_CNT:
                    self.min_rtt_us = rm
                    self.ext.x_est_fall_cnt = 0
                    self.min_rtt_stamp = 1
                    self.ext.mr_update_rtt_cnt = self.rtt_cnt
            else:
                self.ext.x_est_fall_cnt = 0

        if self.min_rtt_us != mr_snapshot:
            self.ext.mr_update_rtt_cnt = self.rtt_cnt

    # ---------- kcc_get_model_rtt ----------
    def get_model_rtt(self) -> int:
        return max(self.min_rtt_us, KCC_RTT_MIN_FLOOR_US)

    # ---------- kcc_rate_bytes_per_sec ----------
    def rate_bytes_per_sec(self, bw_seg_us: int, gain: int,
                           mss: int = 1448) -> int:
        """Mirror kcc_rate_bytes_per_sec(): BW_UNIT -> bytes per second.

        bw_seg_us is in BW_UNIT (segments per usec << BW_SCALE).  The
        USEC_PER_SEC factor converts per-usec to per-second; without it the
        pacer runs 1e-6 times too slow and the connection stalls.
        """
        rate = bw_seg_us * mss
        rate = (rate * gain) >> BBR_SCALE
        rate -= rate // 100
        return (rate * 1000000) >> BW_SCALE

    # ---------- kcc_min_tso_segs ----------
    def min_tso_segs(self, pacing_rate: int, jitter_us: int) -> int:
        div = KCC_MIN_TSO_RATE_DIV
        if jitter_us > KCC_TSO_HIGH_JITTER_THRESH_US:
            div = max(1, div >> KCC_TSO_DIV_HALVE_SHIFT)
        thresh = max(1, KCC_MIN_TSO_RATE // div)
        return (KCC_TSO_SEGS_LOW if pacing_rate < thresh
                else KCC_TSO_SEGS_DEFAULT)

    # ---------- kcc_bdp ----------
    def bdp(self, bw_seg_us: int, gain: int) -> int:
        model_rtt = self.get_model_rtt()
        w = bw_seg_us * model_rtt
        bdp64 = (w * gain) >> BBR_SCALE
        bdp64 += BW_UNIT - 1
        return bdp64 >> BW_SCALE

    # ---------- kcc_quantization_budget ----------
    def quantization_budget(self, cwnd: int, tso_segs_goal: int = 2) -> int:
        cwnd += KCC_TSO_HEADROOM_MULT * tso_segs_goal
        cwnd = (cwnd + 1) & ~1
        return cwnd

    def cwnd_target(self, bw_seg_us: int, gain: int) -> int:
        target = self.quantization_budget(self.bdp(bw_seg_us, gain))
        if self.mode == MODE_PROBE_BW and self.cycle_idx == 0:
            target += KCC_PROBE_CWND_BONUS
        return target

    def inflight(self, bw_seg_us: int, gain: int) -> int:
        return self.quantization_budget(self.bdp(bw_seg_us, gain))

    # ---------- kcc_is_next_cycle_phase ----------
    def is_next_cycle_phase(self, prior_in_flight: int, losses: int,
                            is_full_length: bool, elapsed_us: int,
                            bw_seg_us: int) -> bool:
        if self.pacing_gain == BBR_UNIT:
            return is_full_length
        inflight = prior_in_flight
        if self.pacing_gain > BBR_UNIT:
            return (is_full_length and (
                losses > 0 or inflight >= self.inflight(bw_seg_us, self.pacing_gain))) \
                or elapsed_us > self.min_rtt_us * 2
        return ((is_full_length and inflight <= self.inflight(bw_seg_us, BBR_UNIT))
                or elapsed_us > self.min_rtt_us * 4)

    # ---------- kcc_check_drain ----------
    def check_drain(self, inflight: int, bw_seg_us: int, elapsed_us: int,
                    drain_and_or_mode: bool = True) -> bool:
        bdp = self.inflight(bw_seg_us, BBR_UNIT)
        drained = inflight <= bdp
        timeout = elapsed_us > self.min_rtt_us * 4
        one_rtt = elapsed_us > self.min_rtt_us
        if drain_and_or_mode:
            return (drained and one_rtt) or timeout
        return drained or timeout

    # ---------- kcc_ack_aggregation_cwnd ----------
    def ack_aggregation_cwnd(self, bw_seg_us: int, extra_acked_gain: int) -> int:
        if extra_acked_gain <= 0 or not self.full_bw_reached:
            return 0
        max_ms = KCC_EXTRA_ACKED_MAX_MS_RATIO * 1000
        product = bw_seg_us * max_ms
        max_aggr_cwnd = min(product >> BW_SCALE, 0xFFFFFFFF)
        aggr64 = (extra_acked_gain * max(self.ext.extra_acked[0], self.ext.extra_acked[1])) >> BBR_SCALE
        return min(aggr64, max_aggr_cwnd)

    # ---------- round boundary ----------
    def end_round(self):
        self.prev_round_rtt_min = self.round_rtt_min
        self.round_rtt_min = 0xFFFFFFFF
        self.rtt_cnt += 1

    def advance_cycle_phase(self):
        self.cycle_idx = (self.cycle_idx + 1) & (KCC_CYCLE_LEN - 1)
        self.cycle_mstamp = 0

    def sample_round_min(self, rtt_us: int):
        if 0 <= rtt_us < self.round_rtt_min:
            self.round_rtt_min = rtt_us
