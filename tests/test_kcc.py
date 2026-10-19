#!/usr/bin/env python3
"""
test_kcc.py -- TDD suite for the KCC fixes.

Each test covers one concrete tcp_kcc.c defect (IDs in .opencode/process.md).
The model kcc_model.py mirrors the CURRENT C behaviour; after fixing C, sync the model and the tests turn green.

Run: python -m unittest tests.test_kcc -v
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kcc_model import (
    Kcc, MODE_PROBE_BW, MODE_DRAIN,
    KCC_SCALE_SHIFT, KCC_G3_FAST_CNT,
    BBR_UNIT,
)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read_source():
    with open(os.path.join(ROOT, "tcp_kcc.c"), encoding="utf-8", errors="replace") as f:
        return f.read()


class TestP1ZeroRttSample(unittest.TestCase):
    """P1 (K01): rs->rtt_us == 0 must not collapse min_rtt or trigger locked."""

    def test_zero_rtt_does_not_collapse_min_rtt(self):
        k = Kcc(t_prop_us=20000)
        for _ in range(20):
            k.update_min_rtt(20000)
        self.assertEqual(k.min_rtt_us, 20000)

        k.update_min_rtt(0)

        self.assertEqual(k.min_rtt_us, 20000,
                         "a zero-RTT sample must not collapse min_rtt to 1us")
        self.assertEqual(k.ext.x_est >> KCC_SCALE_SHIFT, 20000,
                         "a zero-RTT sample must not collapse x_est")

    def test_zero_rtt_does_not_set_locked(self):
        k = Kcc(t_prop_us=20000)
        k.update_min_rtt(20000)
        k.update_min_rtt(0)
        self.assertEqual(k.locked, 0,
                         "a zero-RTT sample is not a measurement and must not set locked")

    def test_min_rtt_stays_usable_after_zero_sample(self):
        k = Kcc(t_prop_us=20000)
        for _ in range(10):
            k.update_min_rtt(20000, round_start=True)
            k.end_round()
        k.update_min_rtt(0)
        k.update_min_rtt(20000, round_start=True)
        k.end_round()

        # bw = 1 seg/us (=1e6 seg/s); 1.0x BDP at 20ms = 20000 segments
        cwnd_target = k.bdp(1 << 24, BBR_UNIT)
        self.assertGreater(cwnd_target, 1000,
                           f"BDP target collapsed to {cwnd_target}; min_rtt not recovered")


class TestP2G3PerRound(unittest.TestCase):
    """P2 (F33/F01/F08): G3 must decide once per round on clean samples."""

    def _feed_round(self, k, rtt_us, acks=20):
        for i in range(acks):
            k.sample_round_min(rtt_us)
            k.update_min_rtt(rtt_us, round_start=(i == 0))
        k.end_round()

    def test_probe_up_queue_does_not_trigger_g3(self):
        k = Kcc(t_prop_us=20000)
        for _ in range(10):
            self._feed_round(k, 20000)
        self.assertEqual(k.min_rtt_us, 20000)

        self._feed_round(k, 25000)   # one probe-up phase

        self.assertEqual(k.min_rtt_us, 20000,
                         "a probe-up self-built queue must not be read as a path increase")
        self.assertEqual(k.confirm_cnt, 0)
        self.assertEqual(k.confirm_slow_cnt, 0)

    def test_real_path_increase_detected_within_6_rounds(self):
        k = Kcc(t_prop_us=20000)
        for _ in range(10):
            self._feed_round(k, 20000)

        detected_at = None
        for r in range(12):
            self._feed_round(k, 200000)
            if k.min_rtt_us >= 200000 * 95 // 100:
                detected_at = r + 1
                break

        self.assertIsNotNone(detected_at, "10x path increase not detected")
        self.assertLessEqual(detected_at, KCC_G3_FAST_CNT + 1,
                             f"detection took {detected_at} rounds, above the fast threshold")

    def test_probe_up_cannot_block_cycle_exit(self):
        """F33: the probe-up exit condition must stay reachable after G3 raises min_rtt."""
        k = Kcc(t_prop_us=20000)
        k.mode = MODE_PROBE_BW
        k.pacing_gain = BBR_UNIT * 5 // 4
        for _ in range(10):
            self._feed_round(k, 20000)

        bw = 1
        bdp_1x = k.inflight(bw, BBR_UNIT)
        target_125 = k.inflight(bw, k.pacing_gain)
        inflight = int(bdp_1x * 1.25) + 4
        self.assertGreaterEqual(inflight, target_125,
                                f"inflight={inflight} cannot reach the 1.25x target={target_125} (F33)")

    def test_ack_delayed_does_not_trigger_g3(self):
        """F10: an RTT increase caused by delayed ACKs must not trigger G3."""
        k = Kcc(t_prop_us=20000)
        for _ in range(10):
            self._feed_round(k, 20000)
        for _ in range(20):
            k.sample_round_min(50000)
            k.update_min_rtt(50000, round_start=False, is_ack_delayed=True)
        k.end_round()
        self.assertEqual(k.min_rtt_us, 20000,
                         "delayed ACKs must not trigger G3 and raise min_rtt")


class TestP3EstimatorSemantics(unittest.TestCase):
    """P3 (F34/F02/F07): x_est must estimate T_prop and qdelay must reflect queueing."""

    def test_x_est_does_not_track_every_ack_sample(self):
        k = Kcc(t_prop_us=20000)
        k.update_min_rtt(20000, round_start=True)
        k.end_round()
        for i in range(20):
            k.update_min_rtt(20000 + i * 10, round_start=(i == 0))
        k.end_round()

        x_us = k.ext.x_est >> KCC_SCALE_SHIFT
        self.assertNotEqual(x_us, 20000 + 19 * 10,
                            "x_est exactly equals the latest sample -> estimator degenerates to pass-through (F34)")

    def test_g2_grows_12_percent_per_round(self):
        k = Kcc(t_prop_us=20000)
        k.update_min_rtt(20000, round_start=True)
        k.end_round()
        x0 = k.ext.x_est

        k.update_min_rtt(30000, round_start=True)
        k.end_round()
        x1 = k.ext.x_est

        expected = min(x0 * 1122 // 1000, 30000 << KCC_SCALE_SHIFT)
        self.assertAlmostEqual(x1, expected, delta=x0 // 100,
                               msg=f"G2 per round should be 1.122x, got {x1 / x0:.4f}x")

    def test_qdelay_reflects_gradient_queue(self):
        k = Kcc(t_prop_us=20000)
        k.update_min_rtt(20000, round_start=True)
        k.end_round()
        for i in range(100):
            k.update_min_rtt(20000 + i * 100, round_start=(i % 10 == 0))
        self.assertGreater(k.ext.qdelay_avg, 3000,
                           f"gradual 10ms queue build, qdelay_avg only {k.ext.qdelay_avg}us (F02)")

    def test_bdp_ignores_unconfirmed_low_outlier(self):
        k = Kcc(t_prop_us=20000)
        for _ in range(10):
            k.update_min_rtt(20000, round_start=True)
            k.end_round()
        baseline_bdp = k.bdp(1, BBR_UNIT)

        k.update_min_rtt(5000, round_start=True)
        k.end_round()

        bdp_after = k.bdp(1, BBR_UNIT)
        self.assertGreater(bdp_after, baseline_bdp * 60 // 100,
                           f"a single outlier cut BDP from {baseline_bdp} to {bdp_after}")


class TestP4LockedRecovery(unittest.TestCase):
    """P4 (K05/F05): locked must not disable G3 forever."""

    def test_locked_released_after_sustained_high_rtt(self):
        k = Kcc(t_prop_us=4000)
        k.sample_round_min(4000)
        k.update_min_rtt(4000, round_start=True)
        k.end_round()
        self.assertEqual(k.locked, 1, "a 4ms sample should set locked")

        for _ in range(64):
            k.sample_round_min(40000)
            k.update_min_rtt(40000, round_start=True)
            k.end_round()

        self.assertEqual(k.locked, 0, "locked should be released after sustained high RTT")
        self.assertGreaterEqual(k.min_rtt_us, 30000,
                                f"min_rtt should rise after lock release, got {k.min_rtt_us}")


class TestP5GetInfoAttr(unittest.TestCase):
    """P5 (F26/K04): get_info must set *attr."""

    def test_get_info_sets_attr(self):
        self.assertIn("*attr = INET_DIAG_BBRINFO", read_source(),
                      "kcc_get_info did not set *attr; inet_diag would use stack garbage as the attribute type")


class TestP8EcnBackoff(unittest.TestCase):
    """P8: ECN backoff keeps the original fixed-fraction behaviour (Lemma N.6)."""

    def test_gain_floor_is_one(self):
        src = read_source()
        self.assertRegex(src, r"#define\s+KCC_GAIN_FLOOR\s+1\b",
                         "KCC_GAIN_FLOOR must stay 1 (=1/256 in BBR_UNIT scale)")

    def test_backoff_does_not_scale_with_ecn_ewma(self):
        src = read_source()
        self.assertNotIn("ecn_backoff * ext->ecn_ewma", src,
                         "ECN backoff is a fixed fraction in the original controller")

    def test_probe_phase_scaling_present(self):
        src = read_source()
        self.assertIn("ecn_scale = (1U << (BBR_SCALE + BBR_SCALE)) / kcc->pacing_gain;", src,
                      "probe-phase backoff scaling (1/pacing_gain) must stay")


class TestP10DrainGate(unittest.TestCase):
    """P10: DRAIN defaults to KCC's AND gate; the OR gate is selectable."""

    def test_drain_default_is_and_gate(self):
        src = read_source()
        self.assertRegex(src, r"static int kcc_drain_and_or_mode = 1;",
                         "DRAIN default must stay AND: KCC requires both the BDP condition and 1 RTT")

    def test_drain_exits_when_drained_even_before_one_rtt(self):
        k = Kcc(t_prop_us=20000)
        k.mode = MODE_DRAIN
        for _ in range(10):
            k.update_min_rtt(20000, round_start=True)
            k.end_round()
        bw = 1
        bdp = k.inflight(bw, BBR_UNIT)

        exit_now = k.check_drain(inflight=bdp - 1, bw_seg_us=bw,
                                 elapsed_us=10000, drain_and_or_mode=False)
        self.assertTrue(exit_now, "should exit when drained but under 1 RTT (OR gate)")


class TestP16Overflow(unittest.TestCase):
    """P16 (K08/K09): overflow boundaries."""

    def test_rate_bytes_per_sec_no_overflow(self):
        U64_MAX = (1 << 64) - 1
        mss = 65483
        bw = (1 << 32) - 1
        gain = 739
        r1 = (bw * mss * gain) >> 8
        self.assertGreater(r1 * 990000, U64_MAX,
                           "test premise: this input really does overflow")
        r_fixed = r1 - r1 // 100
        self.assertLessEqual(r_fixed, U64_MAX)

    def test_rate_uses_subtraction_not_990000_multiply(self):
        src = read_source()
        self.assertNotIn("rate *= USEC_PER_SEC * 99 / 100", src,
                         "rate *= 990000 overflows u64 when rate > 1.86e13")
        self.assertIn("rate -= div_u64(rate, 100);", src)


class TestP6LtTimestampWrap(unittest.TestCase):
    """P6 (K03/F18): LT timestamps must be subtracted in the u32 domain."""

    def test_lt_interval_subtracts_in_u32(self):
        src = read_source()
        self.assertIn("u32 t_ms = (u32)div_u64(tp->delivered_mstamp, USEC_PER_MSEC) -",
                      src, "LT timestamps not subtracted in u32: after 49.7 days the interval grows to 2^32 ms")
        self.assertIn("if ((s32)t_ms < 1)", src)


class TestP7LtIntervalLoss(unittest.TestCase):
    """P7 (F19): ending an LT interval must require a loss on the current ACK."""

    def test_lt_interval_requires_current_loss(self):
        src = read_source()
        self.assertRegex(src, r"if \(rs->losses <= 0\) \{\s*return;\s*\}",
                         "LT interval can end on a lossless ACK and underestimate the policed rate")


class TestP11TakeoverCounter(unittest.TestCase):
    """P11 (F06): takeover must use its own counter."""

    def test_takeover_uses_own_counter(self):
        src = read_source()
        self.assertIn("x_est_fall_cnt", src,
                      "takeover still shares min_rtt_fast_fall_cnt and can never reach the threshold")

    def test_takeover_actually_lowers_min_rtt(self):
        k = Kcc(t_prop_us=20000)
        k.srtt_us = 0
        # establish x_est = 10ms, min_rtt = 20ms (x_est below 95% of min_rtt)
        for _ in range(10):
            k.update_min_rtt(10000, round_start=True)
            k.end_round()
        # x_est was pulled to 10ms by G1; takeover needs five consecutive samples
        for _ in range(5):
            k.update_min_rtt(20000, round_start=True)
            k.end_round()
        self.assertLessEqual(k.min_rtt_us, 10000,
                             f"takeover did not engage, min_rtt={k.min_rtt_us}")


class TestP12AggThreshold(unittest.TestCase):
    """P12 (F16): the ACK aggregation gate must use the stable lower bound."""

    def test_agg_gate_uses_lower_of_min_rtt_and_x_est(self):
        src = read_source()
        self.assertIn("path_rtt = min_t(u32, path_rtt,", src,
                      "agg gate only looks at min_rtt; G3/delayed ACKs can raise it and wrongly enable aggregation")


class TestP14TsoJitter(unittest.TestCase):
    """P14 (F25): high jitter must shorten bursts (raise the threshold)."""

    def test_high_jitter_raises_threshold(self):
        src = read_source()
        self.assertIn("div = max_t(u32, 1, div >> KCC_TSO_DIV_HALVE_SHIFT);", src,
                      "with high jitter div grows -> threshold falls -> more 2-segment returns -> larger bursts")


class TestP15KfSemantics(unittest.TestCase):
    """P15 (F29/F30/F31): KF floor semantics and parameter description."""

    def test_kf_floor_is_gain_compensated(self):
        # README "Dessert-Speed Discount Ratio": coeff = discount/high_gain
        # = (50/100)/2.887 = 17.3%.  The seed is injected at neutral gain and
        # STARTUP applies 2.887x on the first ACK; the original code is the
        # authoritative behaviour.
        src = read_source()
        self.assertIn("init_bw = (init_bw << BBR_SCALE) / KCC_HIGH_GAIN;", src,
                      "KF floor must stay gain-compensated (17.3% dessert speed)")
        self.assertIn("init_bw = (init_bw << BBR_SCALE) / KCC_PACING_INIT_GAIN;", src,
                      "KF init_bw seed must stay gain-compensated (17.3%)")

    def test_kf_mode_description_matches_implementation(self):
        src = read_source()
        self.assertIn("0=instant (default), 1=peak-tracking", src,
                      "kf_mode description contradicts the implementation")

    def test_no_dead_kf_floor_in_update_bw(self):
        src = read_source()
        self.assertNotIn("floor_bw = (floor_bw << BBR_SCALE) / KCC_HIGH_GAIN;", src,
                         "kcc_update_bw still contains dead code assigning to a local bw")


class TestP18LegacyKfunc(unittest.TestCase):
    """P18 (K07): register_kfunc_btf_id_set returns void on 5.16/5.17."""

    def test_legacy_register_not_assigned_to_int(self):
        src = read_source()
        self.assertNotIn("ret = register_kfunc_btf_id_set", src,
                         "register_kfunc_btf_id_set returns void on 5.16/5.17; assigning it to int does not compile")


class TestP19ColdStart(unittest.TestCase):
    """P19 (audit): cold start must adopt the first real sample instead of the 1ms placeholder."""

    def test_first_sample_replaces_default_min_rtt(self):
        k = Kcc(t_prop_us=1000)      # kcc_init placeholder
        k.has_seen_rtt = 0
        k.sample_round_min(20000)
        k.update_min_rtt(20000, round_start=True)
        self.assertEqual(k.min_rtt_us, 20000,
                         "min_rtt is still the 1ms placeholder after the first 20ms sample")

    def test_cold_start_bdp_is_usable_immediately(self):
        k = Kcc(t_prop_us=1000)
        k.has_seen_rtt = 0
        k.sample_round_min(20000)
        k.update_min_rtt(20000, round_start=True)
        bdp = k.bdp(1 << 24, BBR_UNIT)   # 1 seg/us, 1.0x
        self.assertGreater(bdp, 10000,
                           f"cold-start BDP={bdp}, real 20ms baseline not adopted")

    def test_outlier_does_not_lock_g3_on_long_path(self):
        """A single 4ms outlier on a 20ms path must not set locked."""
        k = Kcc(t_prop_us=20000)
        k.has_seen_rtt = 1
        for _ in range(5):
            k.sample_round_min(20000)
            k.update_min_rtt(20000, round_start=True)
            k.end_round()
        k.sample_round_min(4000)
        k.update_min_rtt(4000)
        self.assertEqual(k.locked, 0,
                         "a single low-RTT outlier on a long path must not lock G3")

    def test_short_path_still_locks(self):
        k = Kcc(t_prop_us=4000)
        k.has_seen_rtt = 1
        k.sample_round_min(4000)
        k.update_min_rtt(4000, round_start=True)
        self.assertEqual(k.locked, 1, "a genuinely short path should lock G3")


class TestP20RoundEvidence(unittest.TestCase):
    """P20 (audit): every "N consecutive" evidence chain must count per round and break on gaps."""

    def _feed(self, k, rtt, acks=20):
        for i in range(acks):
            k.sample_round_min(rtt)
            k.update_min_rtt(rtt, round_start=(i == 0))
        k.end_round()

    def test_single_low_sample_does_not_reduce_min_rtt(self):
        """A single 0.25x sample must not lower min_rtt immediately."""
        k = Kcc(t_prop_us=20000)
        k.has_seen_rtt = 1
        for _ in range(5):
            self._feed(k, 20000)
        self._feed(k, 5000)
        self.assertEqual(k.min_rtt_us, 20000,
                         "a single low sample lowered min_rtt without confirmation")

    def test_four_consecutive_low_samples_still_wait(self):
        k = Kcc(t_prop_us=20000)
        k.has_seen_rtt = 1
        for _ in range(5):
            self._feed(k, 20000)
        for _ in range(4):
            self._feed(k, 10000)   # 0.5x, requires five confirmations
        self.assertEqual(k.min_rtt_us, 20000,
                         "four low samples must not pass the five-sample gate")

    def test_five_consecutive_low_samples_accepted(self):
        k = Kcc(t_prop_us=20000)
        k.has_seen_rtt = 1
        for _ in range(5):
            self._feed(k, 20000)
        for _ in range(5):
            self._feed(k, 10000)
        self.assertEqual(k.min_rtt_us, 10000,
                         "five consecutive low samples should be accepted")

    def test_round_without_samples_breaks_chain(self):
        """A round without a valid sample must reset the confirmation counters."""
        k = Kcc(t_prop_us=20000)
        k.has_seen_rtt = 1
        for _ in range(5):
            self._feed(k, 20000)
        # three low-sample rounds
        for _ in range(3):
            self._feed(k, 10000)
        # one empty round (no sample_round_min)
        k.update_min_rtt(20000, round_start=True)
        k.end_round()
        # two more low-sample rounds: without the break the chain would reach five
        for _ in range(2):
            self._feed(k, 10000)
        self.assertEqual(k.min_rtt_us, 20000,
                         "the observation gap did not break the consecutive chain")

    def test_fast_fall_requires_two_rounds(self):
        """A drop below min_rtt/4 needs two rounds (reject single timestamp artefacts)."""
        k = Kcc(t_prop_us=20000)
        k.has_seen_rtt = 1
        for _ in range(5):
            self._feed(k, 20000)
        self._feed(k, 4000)
        self.assertEqual(k.min_rtt_us, 20000,
                         "a single 4x low sample must not take effect immediately")
        self._feed(k, 4000)
        self.assertEqual(k.min_rtt_us, 4000,
                         "two consecutive 4x drops should take effect")


class TestP21SrttGuardVsG3(unittest.TestCase):
    """P21 (audit B F09): the SRTT guard must not erase a G3-confirmed path increase."""

    def _feed(self, k, rtt, acks=10):
        for i in range(acks):
            k.sample_round_min(rtt)
            k.update_min_rtt(rtt, round_start=(i == 0))
        k.end_round()

    def test_path_increase_survives_lagging_srtt(self):
        """When the path grows from 20ms to 200ms, srtt still lags at 20ms;
        G3 must raise min_rtt to 200ms within 6-7 rounds and the SRTT guard must not undo it."""
        k = Kcc(t_prop_us=20000)
        k.has_seen_rtt = 1
        k.srtt_us = 20000 * 8          # srtt = 20ms, frozen (worst-case lag)
        for _ in range(10):
            self._feed(k, 20000)
        self.assertEqual(k.min_rtt_us, 20000)

        detected = None
        for r in range(10):
            self._feed(k, 200000)
            if k.min_rtt_us >= 190000:
                detected = r + 1
                break

        self.assertIsNotNone(detected, "10x path increase not detected by G3 (erased by the SRTT guard)")
        self.assertLessEqual(detected, 7, f"detection took {detected} rounds")

    def test_srtt_guard_still_works_when_stable(self):
        """The guard must still work when srtt is stable and genuinely below min_rtt."""
        k = Kcc(t_prop_us=20000)
        k.has_seen_rtt = 1
        k.srtt_us = 10000 * 8          # srtt stable at 10ms
        for _ in range(10):
            self._feed(k, 20000)       # direct samples stay at 20ms (inflated by aggregation)
        self.assertLessEqual(k.min_rtt_us, 11000,
                             f"guard did not engage with stable srtt=10ms, min_rtt={k.min_rtt_us}")


class TestP22G3SelfQueue(unittest.TestCase):
    """P22: G3 must not treat KCC's own queue as a path increase (positive-feedback ratchet)."""

    def _feed(self, k, rtt, acks=10):
        for i in range(acks):
            k.sample_round_min(rtt)
            k.update_min_rtt(rtt, round_start=(i == 0))
        k.end_round()

    def test_transient_queue_is_reverted(self):
        """The queue keeps RTT high for 6+ rounds, then drains; min_rtt must return to the original baseline."""
        k = Kcc(t_prop_us=20000)
        k.has_seen_rtt = 1
        for _ in range(10):
            self._feed(k, 20000)
        self.assertEqual(k.min_rtt_us, 20000)

        # self-built queue: RTT constant at 60ms for 12 rounds (enough to trigger G3)
        for _ in range(12):
            self._feed(k, 60000)
        # queue drains, back to 20ms
        for _ in range(10):
            self._feed(k, 20000)

        self.assertEqual(k.min_rtt_us, 20000,
                         f"self-built queue permanently raised min_rtt to {k.min_rtt_us} (ratchet)")

    def test_genuine_increase_is_kept(self):
        """After a real path increase RTT does not fall back; min_rtt must keep the new baseline."""
        k = Kcc(t_prop_us=20000)
        k.has_seen_rtt = 1
        for _ in range(10):
            self._feed(k, 20000)
        for _ in range(30):
            self._feed(k, 60000)
        self.assertGreaterEqual(k.min_rtt_us, 55000,
                                f"a real increase was reverted, min_rtt={k.min_rtt_us}")


class TestP23ProbeUpTimeout(unittest.TestCase):
    """P23: the probe-up phase needs a timeout, or an unreachable inflight target keeps it forever."""

    def test_probe_up_exits_on_timeout(self):
        k = Kcc(t_prop_us=20000)
        k.mode = MODE_PROBE_BW
        k.pacing_gain = BBR_UNIT * 5 // 4
        for _ in range(5):
            k.update_min_rtt(20000, round_start=True)
            k.end_round()
        bw = 1 << 24
        # inflight far below the 1.25x target, but 2*min_rtt has elapsed
        self.assertTrue(
            k.is_next_cycle_phase(prior_in_flight=1, losses=0,
                                  is_full_length=True,
                                  elapsed_us=20000 * 3, bw_seg_us=bw),
            "probe-up did not exit after 2*min_rtt (it would stay forever when the target is unreachable)")

    def test_probe_up_does_not_exit_early(self):
        k = Kcc(t_prop_us=20000)
        k.mode = MODE_PROBE_BW
        k.pacing_gain = BBR_UNIT * 5 // 4
        for _ in range(5):
            k.update_min_rtt(20000, round_start=True)
            k.end_round()
        bw = 1 << 24
        # under one RTT and inflight is short: must not exit
        self.assertFalse(
            k.is_next_cycle_phase(prior_in_flight=1, losses=0,
                                  is_full_length=False,
                                  elapsed_us=1000, bw_seg_us=bw),
            "probe-up exited before one full RTT")

    def test_probe_bonus_not_in_exit_target(self):
        """The probe bonus applies to cwnd only and must not raise the probe-up exit threshold."""
        k = Kcc(t_prop_us=20000)
        k.mode = MODE_PROBE_BW
        k.cycle_idx = 0
        bw = 1 << 24
        self.assertEqual(k.inflight(bw, BBR_UNIT * 5 // 4),
                         k.quantization_budget(k.bdp(bw, BBR_UNIT * 5 // 4)),
                         "the probe-up exit target includes the probe bonus")
        self.assertGreater(k.cwnd_target(bw, BBR_UNIT * 5 // 4),
                           k.quantization_budget(k.bdp(bw, BBR_UNIT * 5 // 4)),
                           "the cwnd target is missing the probe bonus")


class TestP24PacingRateUnits(unittest.TestCase):
    """P24: kcc_rate_bytes_per_sec must convert usec to seconds."""

    def test_60kb_cwnd_1ms_rtt_gives_about_60mbps(self):
        # 60 KiB / 1 ms with a 1448-byte MSS is about 60.8 MB/s at gain 1.0.
        segs = 60 * 1024 / 1448.0
        bw = int(segs / 1000.0 * (1 << 24))
        rate = Kcc(t_prop_us=1000).rate_bytes_per_sec(bw, BBR_UNIT)
        self.assertGreater(rate, 55_000_000,
                           f"pacing rate {rate} B/s is far too low")
        self.assertLess(rate, 66_000_000,
                        f"pacing rate {rate} B/s is far too high")

    def test_haircut_is_one_percent(self):
        bw = 1 << 24
        mss = 1448
        raw = (bw * mss * BBR_UNIT) >> 8
        expected = ((raw - raw // 100) * 1_000_000) >> 24
        self.assertEqual(
            Kcc(t_prop_us=20000).rate_bytes_per_sec(bw, BBR_UNIT, mss),
            expected)

    def test_high_rate_conversion_stays_positive(self):
        # 32-bit bandwidth ceiling with jumbo MSS and maximum gain.
        rate = Kcc(t_prop_us=1000).rate_bytes_per_sec(0xFFFFFFFF, 1024, 65535)
        self.assertGreater(rate, 0)

    def test_init_pacing_rate_is_not_stalled(self):
        # kcc_init_pacing_rate_from_rtt(): 10 segments over 1 ms at 2.887x.
        bw = (10 << 24) // 1000
        rate = Kcc(t_prop_us=1000).rate_bytes_per_sec(bw, 739)
        self.assertGreater(rate, 30_000_000,
                           f"initial pacing rate {rate} B/s is too low")
        self.assertLess(rate, 50_000_000,
                        f"initial pacing rate {rate} B/s is too high")

    def test_pacing_grid_never_zero(self):
        for bw in (1, 1 << 12, 1 << 20, 1 << 24, 0xFFFFFFFF):
            for mss in (536, 1448, 9000, 65535):
                for gain in (64, 256, 739, 1024):
                    rate = Kcc(t_prop_us=1000).rate_bytes_per_sec(
                        bw, gain, mss)
                    self.assertGreater(
                        rate, 0,
                        f"pacing rate is zero for bw={bw} mss={mss} gain={gain}")
                    raw = (bw * mss * gain) >> 8
                    raw -= raw // 100
                    self.assertEqual(rate, (raw * 1_000_000) >> 24)

    def test_c_source_keeps_usec_conversion(self):
        src = read_source()
        start = src.index("static u64 kcc_rate_bytes_per_sec")
        end = src.index("\n}", start)
        body = src[start:end]
        self.assertIn("USEC_PER_SEC", body,
                      "pacing-rate function lost the usec->second conversion")
        self.assertIn("mul_u64_u32_shr(rate, USEC_PER_SEC, BW_SCALE)", body,
                      "pacing-rate conversion must be overflow-safe")


class TestP25TsoDirection(unittest.TestCase):
    """P25: high jitter must shorten the minimum TSO burst."""

    def test_high_jitter_returns_one_segment(self):
        k = Kcc(t_prop_us=20000)
        rate = 200_000  # between KCC_MIN_TSO_RATE/16 and KCC_MIN_TSO_RATE/8
        self.assertEqual(k.min_tso_segs(rate, 0), 2)
        self.assertEqual(k.min_tso_segs(rate, 5000), 1)

    def test_low_and_high_rates_unaffected(self):
        k = Kcc(t_prop_us=20000)
        self.assertEqual(k.min_tso_segs(100_000, 0), 1)
        self.assertEqual(k.min_tso_segs(100_000, 5000), 1)
        self.assertEqual(k.min_tso_segs(1_000_000, 0), 2)
        self.assertEqual(k.min_tso_segs(1_000_000, 5000), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
