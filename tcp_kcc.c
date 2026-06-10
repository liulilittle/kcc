/* ---- tcp_kcc.c : TCP KCC (Kalman Congestion Control) v1.0 ------------------ */
/*
 * KCC: Kalman Congestion Control v1.0
 *
 * A TCP congestion control module for shared-bandwidth VPS environments
 * (e.g. 1-10 Gbps multi-tenant hosting) combining the BBRv1 state machine
 * (Cardwell et al. 2016) with a single-state Kalman filter (Kalman 1960)
 * for propagation-delay estimation.
 *
 * Kernel BBR practice (tcp_bbr.c) uses a sliding-window minimum of recent
 * RTT samples to estimate base propagation delay (`min_rtt_us` / `min_rtt_stamp`).
 * This approach is simple and O(1) per ACK, but exhibits three well-known
 * pathologies: (a) upward bias under persistent queue pressure, because
 * the window always includes queued RTTs; (b) winner-takes-all unfairness
 * among competing flows, since each flow tracks an independent min window;
 * and (c) slow convergence after path changes, because the window must
 * drain stale samples before the true minimum emerges.  KCC replaces the
 * sliding-window min with a single-state Kalman filter that estimates
 * true propagation delay as a latent state, separated from queuing delay
 * and measurement noise by the filter's stochastic model.
 *
 * DESIGN PRINCIPLES
 *
 *   Congestion control algorithms must balance throughput, latency,
 *   fairness, and loss tolerance.  KCC takes a pragmatic approach:
 *
 *   1. BBRv1 PROVIDES a well-understood state machine (STARTUP, DRAIN,
 *      PROBE_BW, PROBE_RTT) with proven pacing and gain-cycling
 *      strategies.  KCC adopts these mechanisms without modification,
 *      preserving compatibility with the BBR congestion-control framework
 *      and its empirical validation across diverse network paths.
 *
 *   2. The KALMAN FILTER augments the BBRv1 foundation by estimating
 *      true propagation delay separately from queuing delay and
 *      measurement jitter.  A more accurate min_rtt estimate yields
 *      tighter BDP computation, better-calibrated CWND, and more
 *      stable pacing — all of which improve bandwidth utilisation
 *      and reduce unnecessary queue buildup.  Kernel BBR's
 *      sliding-window min_rtt is simple and cheap but inherently
 *      biased under persistent queue; the Kalman filter provides an
 *      unbiased estimate with quantified uncertainty (p_est),
 *      enabling confidence-gated decisions that the windowed-min
 *      approach cannot support.
 *
 *   3. Intra-KCC fairness is actively maintained: the Kalman filter
 *      converges all KCC flows sharing a bottleneck to a consistent
 *      propagation-delay estimate, eliminating the winner-takes-all
 *      feedback loop that can cause severe unfairness in pure BBR
 *      multi-flow deployments.  In kernel BBR, each flow independently
 *      tracks its own min_rtt window; flows with lower apparent delay
 *      claim more bandwidth, creating persistent unfairness that grows
 *      with flow count.  KCC's shared-convergence design eliminates
 *      this pathology at the estimator level.  Inter-algorithm
 *      dynamics (BBR, CUBIC, Reno) are left to the standard TCP
 *      competitive equilibrium — KCC neither prioritises nor
 *      penalises external flows.
 *
 *   The metrics that guide development: throughput, latency (P95/P99),
 *   retransmit efficiency, convergence time, and jitter.  The Kalman
 *   extensions aim to improve each of these relative to the BBRv1
 *   baseline without sacrificing robustness.
 *
 * ARCHITECTURE
 *
 *   BBRv1 state machine augmented with a single-state Kalman filter
 *   for propagation-delay estimation.  Key deviations from kernel BBR
 *   (tcp_bbr.c) with rationale for each:
 *
 *   - Kalman filter for min_rtt (x_est, p_est) instead of sliding-window min.
 *     Kernel BBR: windowed min tracked as u32 with update-on-lower-sample.
 *     WHY: Windowed min is biased upward under persistent queue; cannot
 *     separate prop delay from queuing delay.  Kalman filter yields unbiased
 *     estimate with known error covariance, enabling confidence-gated qboost,
 *     dynamic PROBE_RTT, and proactive ECN backoff.
 *     BENEFIT: Lower baseline latency, tighter BDP, fairer multi-flow convergence.
 *
 *   - Directional state update: positive innovations (queue noise) are
 *     skipped — only negative innovations (clean samples) and
 *     qboost-triggered path-change updates enter the filter.
 *     Kernel BBR: every RTT sample (up or down) can update min_rtt when
 *     the window cycles.
 *     WHY: RTT increases are dominated by queuing delay, not prop-delay
 *     changes.  Filtering positive innovations avoids polluting x_est with
 *     queue noise.  Negative innovations (RTT drops) are likely cleaner
 *     samples of the true propagation delay.
 *     BENEFIT: More stable min_rtt estimate under variable queue depth.
 *
 *   - Confidence-gated qboost: large innovations only trigger gain reset
 *     when the filter is converged (p_est <= converged_p_est), with a
 *     configurable cooldown (kcc_kalman_qboost_cdwn, default 15 samples)
 *     between events to prevent runaway on lossy paths.
 *     Kernel BBR: no equivalent mechanism.
 *     WHY: When path delay drops significantly (route change), the filter
 *     must converge quickly.  Uncordinated gain resets on noisy paths
 *     cause oscillations.  The convergence gate and cooldown add hysteresis.
 *     BENEFIT: Fast route-change adaptation without oscillation on lossy paths.
 *
 *   - Adaptive process/measurement noise (Q, R) based on jitter and min_rtt,
 *     with BBR-S covariance-matched estimation as a slow calibration channel.
 *     Kernel BBR: no noise parameters — windowed-min approach has no
 *     stochastic model.
 *     WHY: Fixed Q/R require per-path tuning.  Following BBR-S (2021),
 *     adaptive noise estimates let the Kalman filter operate across diverse
 *     environments (datacenter to satellite) without reconfiguration.
 *     BENEFIT: Deployability across heterogeneous paths without manual tuning.
 *
 *   - Outlier gating with dynamic threshold derived from jitter_ewma.
 *     Kernel BBR: 10% outlier rejection in bandwidth estimator (bw filter),
 *     but no RTT outlier rejection beyond the window-min mechanism itself.
 *     WHY: Single spurious RTT samples (scheduling delay, TSO batching,
 *     interrupt coalescing) would corrupt a standard Kalman update.  The
 *     dynamic threshold tracked via jitter EWMA rejects outliers while
 *     adapting to path conditions.
 *     BENEFIT: Robustness to measurement artifacts without a fixed threshold.
 *
 *   - Hysteresis on Kalman takeover (3 confirming rounds).
 *     Kernel BBR: min_rtt always active — no handover needed.
 *     WHY: Prevents a single clean RTT from immediately overriding the
 *     windowed-min baseline before the filter has demonstrated consistent
 *     improvement.  Three consecutive confirming rounds provide statistical
 *     confidence.
 *     BENEFIT: Smooth transition without false positives on single clean samples.
 *
 *   - Cold-start overshoot correction at sample_cnt == 1.
 *     Kernel BBR: no equivalent — windowed min refills gradually.
 *     WHY: The first Kalman update after a long idle period may use stale
 *     or inflated initial x_est.  The correction bounds the initial cwnd
 *     to prevent burst-induced losses.
 *     BENEFIT: Avoids initial burst loss on long-idle connections.
 *
 *   - Gain decay in PROBE_BW (opt-in via kcc_cycle_decay_mask, disabled
 *     by default, conserving full probe intensity).
 *     Kernel BBR: pacing_gain in PROBE_BW always follows the 8-entry gain
 *     table (1.25, 0.75, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0) — no decay.
 *     WHY: In shallow-buffer paths, 1.25x probe gain can cause packet
 *     loss.  When p_est is low (filter converged) and qdelay is near-full,
 *     reducing probe intensity avoids unnecessary drops.  Disabled by
 *     default to match kernel BBR's probe aggressiveness.
 *     BENEFIT: Optional loss reduction on shallow-buffer paths.
 *
 *   - Proactive cwnd reduction when Kalman covariance is low and qdelay
 *     rises (ECN backoff).
 *     Kernel BBR: reacts to packet loss or ECN only after the event.
 *     WHY: With a converged filter (low p_est), the estimated qdelay is
 *     reliable enough to proactively reduce send rate before ECN marking
 *     or loss occurs.  Uses the same BBR_UNIT scaling for gain reduction.
 *     BENEFIT: Prevents queue buildup and ECN marking proactively.
 *
 *   - Dynamic PROBE_RTT interval based on Kalman covariance p_est.
 *     Kernel BBR: fixed 10-second PROBE_RTT interval (probe_rtt_duration_ms).
 *     WHY: Fixed interval is too aggressive for low-latency paths
 *     (unnecessary throughput dips) and not aggressive enough for
 *     high-jitter paths (stale min_rtt).  Scaling interval inversely with
 *     p_est lets converged filters probe less often, uncertain filters
 *     probe more often.
 *     BENEFIT: Adaptive PROBE_RTT frequency tuned to estimation uncertainty.
 *
 *   - Long-Term (LT) bandwidth estimation triggered on loss events.
 *     Kernel BBR: max-bw filter (`struct minmax bw`) collapses during
 *     sustained losses — the windowed max drops as old high-bw samples
 *     expire.
 *     WHY: LT-BW preserves a separate bandwidth estimate through lossy
 *     periods (e.g., wireless), then smoothly transitions back when the
 *     window recovers.  Similar to BBRv3's LT-BW approach.
 *     BENEFIT: Maintains send rate through lossy periods, faster recovery.
 *
 *   - Single-flow detection with automatic BBR-pure fallback.
 *     Kernel BBR: no equivalent — always uses windowed min regardless
 *     of flow count.
 *     WHY: When only one KCC flow is detected on a path, the Kalman
 *     convergence benefits are moot, and the filter overhead can be
 *     avoided.  KCC falls back to BBR-pure min_rtt tracking.
 *     BENEFIT: Reduced CPU overhead on single-flow paths.
 *
 * REFERENCES
 *
 *   [BBR]   Cardwell et al., "BBR: Congestion-Based Congestion Control",
 *           ACM Queue, Vol. 14 No. 5, 2016.
 *           https://dl.acm.org/doi/10.1145/3009824
 *
 *   [BBR-S] "BBR-S: A Low-Latency BBR Modification for Fast-Varying
 *           Connections", 2021.
 *           https://ieeexplore.ieee.org/document/9438951
 *
 *   [RBBR]  "RBBR: A Receiver-Driven BBR in QUIC for Low-Latency in
 *           Cellular Networks", 2022.
 *           https://ieeexplore.ieee.org/document/9703289
 *
 *   [ERCC]  "ERCC: Fine-grained RDMA Congestion Control via Kalman
 *           Filter-based Multi-bit ECN Feedback Reconstruction", 2025.
 *           https://dl.acm.org/doi/10.1145/3769270.3770124
 *
 *   [Kernel] Linux BBR reference implementation
 *            https://github.com/torvalds/linux/blob/master/net/ipv4/tcp_bbr.c
 *            https://github.com/google/bbr
 *
 *   [BBRplus] "BBRplus: Adaptive Cycle Randomization, Drain-to-Target,
 *             and ACK Aggregation Compensation for BBR Convergence
 *             and Stall Prevention"
 *             https://blog.csdn.net/dog250/article/details/80629551
 *
 *   [IETF101] "BBR Congestion Control Work at Google IETF 101 Update"
 *             https://datatracker.ietf.org/meeting/101/materials/slides-101-iccrg-an-update-on-bbr-work-at-google-00
 *
 * Copyright (c) 2017 ~ 2035 PPP PRIVATE NETWORK(TM) X
 * SPDX-License-Identifier: Dual BSD/GPL
 */

#include <linux/module.h>       /* module_init/module_exit, MODULE_LICENSE, MODULE_DESCRIPTION — kernel module boilerplate required by all loadable kernel modules; kernel BBR uses the identical macro set */
#include <linux/version.h>      /* KERNEL_VERSION(), LINUX_VERSION_CODE — preprocessor version gating for cross-kernel compatibility (get_random_u32_below vs prandom_u32_max, __bpf_kfunc availability) */
#include <net/tcp.h>            /* tcp_sock, tcp_congestion_ops, rate_sample — core TCP structures KCC, like kernel BBR, hooks into via struct tcp_congestion_ops callbacks */
#include <linux/inet_diag.h>    /* INET_DIAG_BBRINFO — enables ss -i to dump KCC state alongside BBR diagnostics; matches kernel BBR's diagnostic interface for tool compatibility */
#include <linux/win_minmax.h>   /* struct minmax, minmax_running_max — sliding-window max for bandwidth estimation; KCC retains this directly from kernel BBR's bw filter unchanged */
#include <linux/math64.h>       /* div_u64, mul_u64_u32_shr — 64-bit fixed-point helpers for BDP and Kalman arithmetic; kernel BBR relies on the same helpers for the same purpose */
#include <linux/random.h>       /* prandom_u32_max (pre-6.2) / get_random_u32_below (6.2+) — uniform random for PROBE_BW cycle-phase start offset randomization (Cardwell et al. 2016 Section 4.3) */

 /*
  * BTF (BPF Type Format) / kfunc support for struct_ops BPF programs.
  * KCC_KFUNC decorates callback functions that may be invoked by BPF
  * struct_ops dispatchers.  Pre-5.16 kernels lack kfunc infrastructure;
  * the macro is a no-op on those kernels.
  *
  * Kernel BBR (tcp_bbr.c) does not use kfunc decoration because it is
  * built into the kernel image, not loaded as a module.  KCC supports
  * optional BPF struct_ops attachment for observability and tuning.
  */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0) /* kernel 5.16+ has btf.h */
#include <linux/btf.h>               /* BTF ID macros for kfunc registration (kernel 5.16+ provides btf ID infrastructure) */
#include <linux/btf_ids.h>           /* BTF_ID / BTF_ID_FLAGS macro definitions (kernel 5.16+ provides set-annotation macros) */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0) /* 6.3+ requires __bpf_kfunc */
#define KCC_KFUNC __bpf_kfunc        /* decorate as BPF kernel function (6.3+); kernel 6.3+ requires explicit __bpf_kfunc tag for struct_ops kfunc registration */
#else
#define KCC_KFUNC                     /* no-op: kfunc attribute not required (pre-6.3 kernels accept kfunc registration without explicit decoration) */
#endif
#else                                 /* kernel < 5.16: no BTF/kfunc support */
#define KCC_KFUNC                     /* no-op: pre-5.16 kernel lacks BTF infrastructure entirely */
#endif
  /*
   * BTF set macros were renamed across kernel versions:
   *   6.9+: BTF_KFUNCS_START / BTF_KFUNCS_END
   *   6.0+: BTF_SET8_START / BTF_SET8_END
   *   5.16+: BTF_SET_START / BTF_SET_END
   *
   * KCC must support all three naming schemes for cross-kernel
   * compatibility.  Unlike kernel BBR (in-tree, version-locked), KCC
   * is an out-of-tree module that must compile against 5.16 through 6.9+.
   */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 9, 0) /* 6.9+: BTF_KFUNCS_START */
#define BTF_SETS_START(name) BTF_KFUNCS_START(name)   /* 6.9+ kfunc set start; kernel renamed from BTF_SET8 to BTF_KFUNCS in 6.9 */
#define BTF_SETS_END(name)   BTF_KFUNCS_END(name)     /* 6.9+ kfunc set end; paired with BTF_KFUNCS_START */
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0) /* 6.0+: BTF_SET8_START */
#define BTF_SETS_START(name) BTF_SET8_START(name)     /* 6.0+ kfunc set start; kernel renamed from BTF_SET to BTF_SET8 in 6.0 */
#define BTF_SETS_END(name)   BTF_SET8_END(name)       /* 6.0+ kfunc set end; paired with BTF_SET8_START */
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0) /* 5.16+: BTF_SET_START */
#define BTF_SETS_START(name) BTF_SET_START(name)       /* 5.16+ kfunc set start; original kernel naming introduced in 5.16 */
#define BTF_SETS_END(name)   BTF_SET_END(name)         /* 5.16+ kfunc set end; paired with BTF_SET_START */
#endif
   /*
    * Kernel 6.2+ renamed prandom_u32_max() to get_random_u32_below().
    * The wrapper kcc_random_below(x) provides a uniform interface.
    */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0) /* 6.2+ uses get_random_u32_below */
#define kcc_random_below(x) get_random_u32_below(x) /* uniform random [0, x); kernel 6.2+ API provides non-deterministic random with bounded range */
#else                                               /* pre-6.2 uses prandom_u32_max */
#define kcc_random_below(x) prandom_u32_max(x)      /* uniform random [0, x); pre-6.2 API uses pseudo-random with bounded range; functionally identical for our use (PROBE_BW cycle phase randomization) */
#endif
    /*
     * tcp_snd_cwnd_set() / tcp_snd_cwnd() were introduced in kernel 5.14.
     * Provide inline fallbacks for older kernels so KCC compiles against
     * pre-5.14 kernels without modification.
     *
     * Kernel BBR does not need these fallbacks because it is part of the
     * kernel tree and is always compiled against its own version.  KCC
     * as an out-of-tree module must bridge the gap.
     */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
static inline void tcp_snd_cwnd_set(struct tcp_sock* tp, u32 val) { WRITE_ONCE(tp->snd_cwnd, val); }
static inline u32 tcp_snd_cwnd(const struct tcp_sock* tp) { return READ_ONCE(tp->snd_cwnd); }
#endif
/* ---- Fixed-Point Scales --------------------------------------------- */
/*
 * KCC uses the exact fixed-point scales as kernel BBR (tcp_bbr.c) for
 * bandwidth and gain representation, ensuring arithmetic compatibility.
 *
 * BW_SCALE  = 24: bandwidth stored in units of BW_UNIT = segments*(1<<24) per
 *                 usec.  24 bits of fractional precision preserves accuracy
 *                 through BDP multiplication (bw * rtt_us >> BW_SCALE) without
 *                 64-bit overflow for realistic BDPs (< 10 Gbps, < 100 ms RTT).
 *                 Derived from kernel BBR's BW_SCALE (tcp_bbr.c line ~90).
 *
 * BBR_SCALE =  8: pacing_gain and cwnd_gain stored as fixed-point
 *                 multiples of BBR_UNIT = 256.  8 bits (256 = 1.0x) provides
 *                 granularity of ~0.39% per step, sufficient for the 1.25x/0.75x
 *                 probe gain table and ECN backoff fractions.
 *                 Derived from kernel BBR's BBR_SCALE (tcp_bbr.c).
 */
#define BW_SCALE 24            /* bitshift: BW_UNIT = segments*(1<<24) per usec; matches kernel BBR's BW_SCALE for identical BDP arithmetic */
#define BW_UNIT  (1 << BW_SCALE)   /* 16777216: fixed-point multiplier for BDP calc bw * rtt_us >> BW_SCALE; kernel BBR uses same unit */
#define BBR_SCALE 8            /* bitshift for BBR_UNIT: 256 = 1.0x gain; matches kernel BBR's BBR_SCALE for gain representation */
#define BBR_UNIT  (1 << BBR_SCALE) /* 256 = 1.0x gain reference; a gain of 2.0x = 2 * BBR_UNIT = 512; derived from Cardwell et al. 2016 */

 /*
  * KCC_GAIN_SLOTS = 256: number of phases in the PROBE_BW cycled gain table.
  * Kernel BBR uses an 8-entry gain table (BBR_GAIN_CYCLE_LEN = 8).  KCC
  * extends this to 256 entries to allow fine-grained gain-decay modulation
  * per cycle phase.  Each entry corresponds to one min_rtt-duration phase.
  * KCC_DECAY_MASK_WORDS = 8: bitmask covering 256 bits via 8 x 32-bit words.
  * Each bit controls whether the corresponding cycle phase enables
  * queuing-delay/jitter-based gain decay.  The 256-bit footprint is stored
  * in 8 x u32 words rather than a struct bitfield for simplicity.
  *
  * WHY 256 slots: 256 = 8 x 32 allows the decay mask to map directly
  * onto a word-index + bit-position scheme (KCC_DECAY_WORD_BITS = 5,
  * KCC_DECAY_BIT_MASK = 31).  This is an extension beyond kernel BBR;
  * kernel BBR has no equivalent decay mechanism.
  */
#define KCC_GAIN_SLOTS 256     /* total PROBE_BW gain table entries (0..255); each slot = 1 RTT-duration phase; extension beyond kernel BBR's 8-entry cycle */
#define KCC_DECAY_MASK_WORDS 8 /* 256 bits stored as 8 x 32-bit words for the opt-in gain-decay mask (disabled by default) */

  /*
   * KCC_KALMAN_INNOV_SQ_CAP: overflow guard in Kalman innovation squaring.
   * sqrt(S64_MAX) ≈ 3.037e9, cap at 3e9 for headroom.
   * The innovation is squared during the Kalman update denominator
   * calculation (innov^2 / S).  Without capping, a noise spike could
   * overflow the 64-bit signed range, corrupting the filter state.
   * No equivalent exists in kernel BBR (which has no Kalman filter).
   */
#define KCC_KALMAN_INNOV_SQ_CAP 3000000000ULL /* overflow guard: sqrt(S64_MAX) ≈ 3.037e9, capped to 3e9 for headroom in innov^2 computation */
   /* S64_MAX (2^63-1) stored as unsigned for min_t() type compatibility. */
#define KCC_S64_MAX ((u64)9223372036854775807ULL)

        /*
         * Aggregation confidence state constants (must precede all usages).
         * KCC's ACK-aggregation confidence FSM uses four monotonic states
         * to progressively enable compensation.
         *
         * Kernel BBR (tcp_bbr.c) does not have a confidence-based aggregation
         * model.  It uses a simpler "extra_acked" window that unconditionally
         * inflates cwnd when aggregation is detected.  KCC's confidence-gated
         * approach (inspired by BBRplus) scales compensation to avoid
         * over-inflation when the aggregation signal is ambiguous.
         *
         * State transition: IDLE -> SUSPECTED -> CONFIRMED -> TRUSTED.
         * Compensation intensity increases with state.
         */
#define KCC_AGG_IDLE      0  /* no aggregation detected or untrusted; no compensation applied; Kalman R uses default scale */
#define KCC_AGG_SUSPECTED 1  /* possible aggregation detected; only adjust Kalman R (increase measurement noise), no cwnd compensation yet */
#define KCC_AGG_CONFIRMED 2  /* confirmed aggregation across multiple epochs; adjust Kalman R + apply light cwnd compensation */
#define KCC_AGG_TRUSTED   3  /* highly trusted across sustained observation; full cwnd compensation and maximum Kalman R scaling */

         /*
          * KCC_DEFAULT_GAIN_CYCLE_LEN = 8: default number of gain-table entries
          * in a single PROBE_BW gain cycle.  Matches kernel BBR's 8-entry cycle
          * (BBR_GAIN_CYCLE_LEN = 8) so that when decay is disabled (default), the
          * gain cycle is identical to kernel BBR's.
          */
#define KCC_DEFAULT_GAIN_CYCLE_LEN 8   /* default gain-table entries per cycle; matches kernel BBR's BBR_GAIN_CYCLE_LEN = 8 */
          /*
           * KCC_MIN_RTT_UNINIT = ~0U (U32_MAX): sentinel indicating min_rtt_us
           * has not yet been measured.  Kernel BBR uses the same convention
           * (BBR_MIN_RTT_UNINIT = ~0U).  All guard branches check for inequality
           * before performing arithmetic on min_rtt_us to avoid underflow.
           */
#define KCC_MIN_RTT_UNINIT        ~0U /* sentinel: min_rtt_us not yet measured; all guards check != before arithmetic to avoid u32 underflow */
#define KCC_PCT_BASE              100 /* percentage base (100 = 100%) for ratio/fraction arithmetic in gain-decay and ECN backoff calculations */
#define KCC_MSTAMP_HI_SHIFT       32  /* shift for u64-timestamp hi/lo split/recombine; KCC splits tcp_mstamp into lo/hi for 32-bit cycle_mstamp fields to save bitfield space in struct kcc; kernel BBR uses the full 64-bit tcp_mstamp directly */
#define KCC_DECAY_MASK_LSB        1   /* LSB extraction for testing individual bits in gain-decay mask via (mask_word & KCC_DECAY_MASK_LSB) */
           /*
            * KCC_PROBE_RTT_JITTER_HASH_MASK: per-flow jitter hash mask.
            * KCC_PROBE_RTT_JITTER_DIV: per-flow jitter divisor.
            * Together they spread the PROBE_RTT jitter across flows using
            * sk->sk_hash to avoid synchronized PROBE_RTT drains (thundering-herd).
            * Kernel BBR uses a similar per-flow hash for PROBE_RTT jitter
            * (tcp_bbr.c: bbr_probe_rtt_jitter_ms) but with different constants.
            */
#define KCC_PROBE_RTT_JITTER_HASH_MASK 0xFF /* per-flow jitter hash mask (sk_hash & MASK); spreads PROBE_RTT timing to avoid synchronized drains across flows */
#define KCC_PROBE_RTT_JITTER_DIV       64   /* per-flow jitter divisor; produces jitter range ~0..2 min_rtt, sufficient to desynchronize without excessive PROBE_RTT delay */
            /*
             * KCC_DRAIN_SKIP_MIN_RTT_DIV: drain-skip guard threshold.
             * If min_rtt >> 3 is large enough (min_rtt > 8 * threshold), KCC
             * may skip DRAIN entirely to avoid an excessive throughput dip on
             * long-RTT paths.  Kernel BBR does not skip DRAIN — it always drains.
             * This is a KCC extension for high-latency paths where 0.35x drain
             * would cause unacceptable throughput reduction.
             */
#define KCC_DRAIN_SKIP_MIN_RTT_DIV     8    /* drain-skip guard: min_rtt >> 3 required before DRAIN may be skipped; extension beyond kernel BBR's always-drain policy */
             /*
              * KCC_DRAIN_TARGET_MAX_RTTS: drain-to-target safety timeout.
              * Maximum number of RTTs to remain in DRAIN before forcibly exiting.
              * If the estimated inflight does not drop to BDP within this limit,
              * KCC exits DRAIN anyway to avoid stalling.  Kernel BBR has no such
              * timeout — it relies on the inflight estimate converging naturally.
              */
#define KCC_DRAIN_TARGET_MAX_RTTS      4    /* drain-to-target safety timeout: max RTTs in DRAIN before forced exit; KCC safety net not present in kernel BBR */
              /*
               * KCC_JITTER_SEED_DIV: cold-start jitter EWMA initial seed divisor.
               * On first sample (when jitter_ewma == 0), the initial jitter is
               * seeded as abs(innov) / KCC_JITTER_SEED_DIV to avoid starting from
               * zero (which would make the first outlier gate trivially reject).
               * Kernel BBR has no jitter EWMA — no equivalent exists.
               */
#define KCC_JITTER_SEED_DIV            4    /* cold-start jitter EWMA init seed divisor; avoids zero initial jitter_ewma on first sample */
#define KCC_DECAY_WORD_BITS       5   /* bits per word in gain-decay mask (1<<5 = 32); used to compute word index: word = idx >> 5 */
#define KCC_DECAY_BIT_MASK        31  /* bit mask per word in gain-decay mask (32-1 = 31); used to compute bit position: bit = idx & 31 */
                 /*
                  * KCC_ALONE_CONFIRM_CNT_MAX: max u8 counter for alone-on-path
                  * confirmation rounds.  When a flow repeatedly qualifies as alone
                  * (no competing flows detected), this counter increments.  At max,
                  * KCC permanently transitions to BBR-pure mode.  255 rounds provides
                  * a long confirmation window to avoid premature fallback on bursty paths.
                  */
#define KCC_ALONE_CONFIRM_CNT_MAX  255 /* max u8 counter for alone-on-path confirmation rounds (0..255); long window avoids premature BBR-pure fallback */
                  /*
                   * KCC_EWMA_NEW_WEIGHT: implicit weight of new sample in EWMA formulas
                   * throughout KCC.  KCC follows the standard EWMA convention:
                   *   ewma = (1 - alpha) * ewma + alpha * sample
                   * where alpha = 1 / (1 << N) and the new-weight value encodes N.
                   * This define documents the pattern; actual alphas vary per EWMA.
                   * Kernel BBR uses the same implicit-weight convention.
                   */
#define KCC_EWMA_NEW_WEIGHT       1   /* implicit weight of new sample in EWMA formulas; follows kernel BBR's EWMA convention (new_weight = 1 for standard alpha) */
#define KCC_BITFIELD_2BIT_MAX      3    /* maximum value storable in a 2-bit bitfield (2^2 - 1 = 3); used for full_bw_cnt, prev_ca_state bitfields */
#define KCC_GAIN_MAX              1023 /* maximum value storable in a 10-bit bitfield (2^10 - 1 = 1023); used for pacing_gain, cwnd_gain; 10 bits covers 0..~4.0x gain at BBR_UNIT scale */
#define KCC_LT_RTT_CNT_MAX        4095 /* maximum value storable in a 12-bit bitfield (2^12 - 1 = 4095); used for lt_rtt_cnt; 4095 RTTs provides a ~40s interval at 10ms RTT */
#define KCC_PROBE_RTT_MAX_SEC     86400 /* maximum PROBE_RTT interval (seconds, 24h); safety ceiling for kcc_probe_rtt_base_jiffies module parameter */
                   /*
                    * KCC_DYN_PROBE_HYPER_NUM/DEN: dynamic PROBE_RTT interval
                    * hyper-converged multiplier.  When the Kalman filter indicates very
                    * low uncertainty (p_est near floor), the probe interval is multiplied
                    * by NUM/DEN = 3/2 = 1.5x as a bonus for converged state.
                    * No equivalent in kernel BBR (fixed 10s interval).
                    */
#define KCC_DYN_PROBE_HYPER_NUM    3     /* dynamic probe interval hyper-converged multiplier numerator; 3/2 = 1.5x bonus for converged Kalman filter */
#define KCC_DYN_PROBE_HYPER_DEN    2     /* dynamic probe interval hyper-converged multiplier denominator; paired with NUM=3 */
                    /*
                     * KCC_TSO_LOW/HIGH_JITTER_THRESH_US: TSO burst sizing thresholds.
                     * KCC adjusts TSO burst size based on jitter_ewma to avoid micro-bursts
                     * on low-jitter paths (where TSO bursts are visible as RTT spikes) and
                     * to maintain throughput on high-jitter paths.
                     *   jitter < 1ms → halve TSO divisor (smaller bursts)
                     *   jitter > 4ms → double TSO divisor (larger bursts to maintain pacing)
                     * No equivalent in kernel BBR — TSO sizing is fixed.
                     */
#define KCC_TSO_LOW_JITTER_THRESH_US   1000  /* TSO burst sizing: jitter_ewma below 1ms → halve TSO divisor for smaller, gentler bursts; no kernel BBR equivalent */
#define KCC_TSO_HIGH_JITTER_THRESH_US  4000  /* TSO burst sizing: jitter_ewma above 4ms → double TSO divisor for larger bursts to maintain pacing accuracy */
                     /*
                      * KCC_AGG_CONFIDENCE_MAX: ACK aggregation confidence score upper bound.
                      * The 4-factor confidence evaluation scores 0..1024, where 1024 = TRUSTED.
                      * 1024 provides ~10 bits of dynamic range for the multi-factor fuser
                      * (qdelay consistency, extra_acked magnitude, epoch duration, jitter).
                      * No equivalent in kernel BBR (which has no confidence scoring).
                      */
#define KCC_AGG_CONFIDENCE_MAX         1024  /* ACK aggregation confidence score upper bound (0..1024); 10-bit range for 4-factor confidence fusion; no kernel BBR equivalent */

                      /* ---- Global Kalman BDP Filter Constants -------------------------------- */
                      /*
                       * KCC_KF_CWND_SEGS_MAX: safety ceiling on CWND (in segments) injected
                       * by the Kalman BDP filter.  Prevents the Kalman filter from commanding
                       * an absurdly large cwnd if the bandwidth estimate spikes or min_rtt
                       * collapses.  No equivalent in kernel BBR (no Kalman BDP filter).
                       */
#define KCC_KF_CWND_SEGS_MAX       20000 /* max CWND segments from KF injection (safety ceiling); prevents Kalman-driven cwnd explosion */
                       /*
                        * KCC_KF_STARTUP_PROTECT_RTTS: rounds to protect STARTUP from premature
                        * full_bw exit.  During STARTUP, the Kalman filter may temporarily
                        * underestimate bandwidth due to the filter's convergence lag.  This
                        * constant prevents the KF from overriding BBR's full_bw detection
                        * during the first N RTTs of STARTUP.  No equivalent in kernel BBR
                        * (which uses only the windowed max-bw filter for full_bw detection).
                        */
#define KCC_KF_STARTUP_PROTECT_RTTS 8   /* STARTUP protection rounds: prevent Kalman filter from triggering premature full_bw exit during initial convergence */
                        /*
                         * Chi-squared innovation gate: prevents transient spikes from corrupting
                         * the global bandwidth estimate (used in KCC's Kalman BDP filter, not
                         * in the per-connection Kalman filter in struct kcc_ext).  num/den encodes
                         * the chi-squared rejection threshold in fixed-point.
                         *   default: 384/100 → 3.84σ (p ≈ 0.05 for 1 degree of freedom)
                         * 3.84σ corresponds to the 95th percentile of a χ²(1) distribution,
                         * a standard statistical outlier threshold.
                         * The bit shifts (INNOV_SHIFT, VAR_SHIFT) provide overflow protection for
                         * the 64-bit product in the (ν² / S) comparison.  The shifts cancel in
                         * the final ratio — they only prevent intermediate overflow.
                         * No equivalent in kernel BBR — BBR does not use a chi-squared gate.
                         */
#define KCC_KF_CHI2_NUM_DEFAULT      384 /* chi-squared gate: numerator (default 384/100 = 3.84σ, p ≈ 0.05 for 1 dof); standard statistical outlier threshold */
#define KCC_KF_CHI2_DEN_DEFAULT      100 /* chi-squared gate: denominator (paired with numerator) */
                         /*
                          * Bit-shift based overflow guards for the Kalman update:
                          *   - OVERFLOW_GUARD: rescale P+R to fit in 31 bits before multiplication
                          *   - INNOV_SHIFT:    downscale innovation before squaring (ν² >> 2·shift)
                          *   - VAR_SHIFT:      downscale variance  before ratio  (S >> shift)
                          * 2×INNOV_SHIFT must equal VAR_SHIFT so the shifts cancel in the ratio.
                          * Design constraint: the shifts are chosen so that
                          *   (ν² / S) = (ν² >> 2·INNOV_SHIFT) / (S >> 2·INNOV_SHIFT)
                          * preserving the ratio exactly while keeping intermediate products
                          * within 64-bit range.
                          * No equivalent in kernel BBR — no Kalman filter in kernel BBR.
                          */
#define KCC_KF_OVERFLOW_GUARD   (1ULL << 31) /* max unscaled P+R before fixed-point rescaling; keeps (P+R) in 31 bits for safe 32-bit multiplications */
#define KCC_KF_INNOV_SHIFT      10           /* innovation scaling shift: ν² >> 2·shift before ratio computation; prevents 64-bit overflow in ν² */
#define KCC_KF_VAR_SHIFT        (2 * KCC_KF_INNOV_SHIFT) /* variance scaling shift (must = 2·INNOV_SHIFT so numerator and denominator shifts cancel); derived constraint, not tunable */

                          /* ---- KCC FSM Modes (mirrors kernel BBR exactly) --------------------- */
                          /*
                           * KCC state machine mirrors kernel BBR (tcp_bbr.c) with four states,
                           * identical semantics and transition rules.  KCC does NOT introduce
                           * any new FSM states — the Kalman filter operates within this existing
                           * BBRv1 framework as a drop-in replacement for the min_rtt estimator.
                           *
                           * State machine identical to Cardwell et al. 2016 and kernel BBR:
                           *
                           *   STARTUP   — rapid exponential probing with pacing_gain approx 2.89x
                           *               (high_gain = 2885 / BBR_UNIT = 2.89).  Exits when the
                           *               bandwidth growth over one round-trip drops below the
                           *               full_bw_threshold (1.25x), indicating the pipe is full.
                           *               Kernel BBR: same logic, same constants.
                           *
                           *   DRAIN     — briefly drains the queue built during STARTUP.
                           *               Pacing_gain = 1 / high_gain ≈ 0.35x.  Exits when
                           *               estimated inflight drops to the BDP at 1.0x gain.
                           *               Kernel BBR: same drain factor, same exit condition.
                           *
                           *   PROBE_BW  — steady-state: cycles through the standard 8-entry
                           *               gain table (1.25, 0.75, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
                           *               where each phase lasts approx 1 min_rtt.  KCC extends
                           *               this to 256 phases (KCC_GAIN_SLOTS) for optional
                           *               gain-decay but the default cycle is identical to kernel
                           *               BBR's 8-entry sequence.
                           *               Kernel BBR: same 8-entry table, same duration per phase.
                           *
                           *   PROBE_RTT — periodically drains inflight to min_cwnd to obtain a
                           *               clean min_rtt sample.  Triggered when the PROBE_RTT
                           *               interval (default 10 seconds, matching kernel BBR's
                           *               probe_rtt_duration_ms = 5000 ms / 2) expires without
                           *               a min_rtt update.  KCC's dynamic interval adjustment
                           *               (based on p_est) is the only deviation.
                           *               Kernel BBR: fixed 10-second interval.
                           *
                           * WHY mirror BBR exactly: reusing the proven state machine minimizes
                           * risk and preserves compatibility with existing BBR deployments.
                           * The Kalman filter improves the quality of the min_rtt estimate
                           * consumed by these states without altering their dynamics.
                           */
enum kcc_mode {                    /* FSM operating mode, bit-for-bit identical to kernel BBR (tcp_bbr.c: bbr_mode) */
    KCC_STARTUP = 0,            /* initial rapid exponential probing, pacing_gain ≈ 2.89x; exits when full_bw_reached; identical to kernel BBR's BBR_STARTUP */
    KCC_DRAIN = 1,            /* drain queue built during STARTUP, pacing_gain ≈ 0.35x; identical to kernel BBR's BBR_DRAIN */
    KCC_PROBE_BW = 2,            /* steady-state: cycle through gain table probing bandwidth; identical to kernel BBR's BBR_PROBE_BW */
    KCC_PROBE_RTT = 3,            /* force min inflight (cwnd = 4 segments) to obtain clean min_rtt sample; identical to kernel BBR's BBR_PROBE_RTT */
};                                /* enum kcc_mode: 4-state FSM matching kernel BBR's 4-state design (Cardwell et al. 2016) */

/* ---- Extended State (heap, not size-constrained) --------------------- */
 /*
  * struct kcc_ext - Per-connection extended state (heap-allocated).
  *
  * The base struct kcc must fit within ICSK_CA_PRIV_SIZE (104 bytes on
  * x86_64).  Kalman filter state, queuing-delay EWMA, jitter EWMA,
  * ACK-aggregation epoch counters, and dynamic interval fields are
  * stored here because they exceed the available in-sock CA slot.
  *
  * Kernel BBR places all state in struct bbr (fits ICSK_CA_PRIV_SIZE)
  * with no external heap allocation because BBR has no Kalman filter
  * and uses smaller bitfield packing.  KCC's split design (struct kcc
  * for compact BBR-compatible fields + struct kcc_ext on heap for
  * Kalman/extension state) accommodates the additional state without
  * breaking the ICSK_CA_PRIV_SIZE constraint.
  */
struct kcc_ext {                                     /* extended per-connection state (heap); no size limit, holds Kalman + aggregation state beyond ICSK_CA_PRIV_SIZE */
    /* ---- Kalman filter state (Kalman 1960) ---- */
    /*
     * Kalman filter state estimate: the estimated true propagation delay,
     * scaled by kalman_scale (a fixed-point factor to preserve precision).
     * This replaces kernel BBR's min_rtt_us, which is a raw sliding-window
     * minimum of recent RTT samples.  Unlike BBR's min_rtt_us, x_est is
     * an unbiased estimate of the latent propagation delay, filtered from
     * queuing noise by the Kalman stochastic model.
     */
    u32 x_est;                                       /* State: true propagation delay (us * kalman_scale); replaces kernel BBR's windowed-min min_rtt_us */
    /*
     * Error covariance: quantifies uncertainty in x_est.  Bounded between
     * kcc_kalman_p_est_floor (minimum uncertainty floor, preventing
     * over-confidence) and kcc_kalman_p_est_max (maximum uncertainty cap,
     * preventing divergence).  Low p_est → filter is converged → KCC
     * trusts x_est more (enables qboost, aggressive qdelay backoff).
     * High p_est → filter is uncertain → KCC conservatively falls back
     * toward BBR-like behavior.  No equivalent in kernel BBR — BBR has
     * no uncertainty quantification for its min_rtt estimate.
     */
    u32 p_est;                                       /* Error covariance, bounded [kcc_kalman_p_est_floor, kcc_kalman_p_est_max]; quantifies trust in x_est; no kernel BBR equivalent */

    /*
     * EWMA-smoothed queuing delay (microseconds).
     * Computed as max(0, current_rtt - x_est / kalman_scale), then
     * smoothed via EWMA.  qdelay_avg is KCC's proxy for queue pressure;
     * it drives ECN backoff, gain decay, and aggregation detection.
     * Kernel BBR: no explicit queuing delay estimate — BBR infers queue
     * pressure indirectly from the difference between observed inflight
     * and estimated BDP.
     */
    u32 qdelay_avg;                                  /* EWMA-smoothed queuing delay (us); KCC's explicit queue-pressure proxy; no direct kernel BBR equivalent */

    /*
     * Number of accepted Kalman updates (i.e., samples that passed the
     * outlier gate and directional update rule).  Used for:
     *   - Cold-start correction (sample_cnt == 1)
     *   - Kalman takeover hysteresis (3 consecutive confirms)
     *   - Selecting between heuristic and covariance-matched Q/R
     * No equivalent in kernel BBR — BBR has no update counter for
     * its min_rtt estimator.
     */
    u32 sample_cnt;                                  /* Number of accepted Kalman updates; no kernel BBR equivalent */

    /*
     * EWMA-smoothed absolute innovation (microseconds).
     * Tracks the recent magnitude of RTT variability via an EWMA over
     * |innov| = |rtt_sample - x_est|.  Used to derive the dynamic
     * outlier gate threshold and to adapt process/measurement noise (Q, R).
     * Kernel BBR: no jitter EWMA.  BBR's min_rtt window is inherently
     * insensitive to jitter (it tracks the minimum, not the variation).
     */
    u32 jitter_ewma;                                 /* EWMA-smoothed absolute innovation (us); tracks RTT variability for dynamic outlier gating and noise adaptation; no kernel BBR equivalent */

    /*
     * Consecutive outlier rejection counter.
     * If too many consecutive samples are rejected by the outlier gate,
     * the dynamic threshold (which grows with jitter) could permanently
     * block all future samples, freezing the Kalman filter.
     * After kcc_kalman_max_consec_reject (default 25) consecutive
     * rejections, the next sample is force-accepted regardless of gate.
     * This is a safety net — without it, a sustained increase in path
     * jitter could permanently stall the Kalman filter.
     * No equivalent in kernel BBR — BBR does not use an outlier gate.
     */
    u32 consec_reject_cnt;                           /* consecutive outlier rejections; force-accept after kcc_kalman_max_consec_reject to prevent filter stall; no kernel BBR equivalent */

    /*
     * Covariance-matched noise estimates (BBR-S adaptive Q/R estimation,
     * Welch & Bishop 2006 covariance matching method).
     *
     * These are updated on every accepted Kalman sample using:
     *   q_est = (1-alpha) * q_est + alpha * (K * innov)^2 / S^2
     *   r_est = (1-beta)  * r_est + beta  * max(0, innov^2/S^2 - p_pred)
     *
     * They serve as a slow calibration channel; the final Q/R used
     * in the filter is max(heuristic_QR, covariance_matched_QR).
     *
     * Kernel BBR: no noise parameters — windowed-min approach has no
     * stochastic model to calibrate.  This is a KCC extension following
     * the BBR-S (2021) approach for adaptive tuning across heterogeneous
     * paths without manual reconfiguration.
     */
    u32 q_est;                                       /* covariance-matched process noise (same units as Q); BBR-S adaptive calibration; no kernel BBR equivalent */
    u32 r_est;                                       /* covariance-matched measurement noise (same units as R); BBR-S adaptive calibration; no kernel BBR equivalent */

    /*
     * ECN (Explicit Congestion Notification) state.
     * When enabled (kcc_ecn_enable != 0), CE-marked segments are tracked
     * via an EWMA of the ECN-mark ratio.  If ecn_ewma > 0 and Kalman
     * qdelay_avg exceeds kcc_ecn_qdelay_thresh_us, cwnd_gain and
     * pacing_gain are reduced proportionally by kcc_ecn_backoff.
     * Scaled to BBR_UNIT (256 = 100%).
     * Kernel BBR: ECN handling is limited to reducing cwnd on each
     * CE-marked ACK (like a loss).  KCC's approach is proactive:
     * it backs off proportionally to the ECN mark ratio, enabling
     * smoother rate reduction before loss occurs.
     */
    u32 ecn_ewma;                                    /* EWMA of ECN-CE mark ratio in BBR_UNIT (0..256); drives proactive gain backoff; kernel BBR reacts per-ACK, not via EWMA */
    u32 last_delivered_ce;                           /* tp->delivered_ce at last ECN EWMA update; difference from current gives CE-mark count since last update */

    /* ---- ACK aggregation epoch tracking (dual-window sliding max) ---- */
    /*
     * KCC extends kernel BBR's ACK-agg compensation (tcp_bbr.c:
     * bbr_ack_aggregation_cwnd) with a dual-window sliding max
     * approach.  Two alternating windows each spanning approx 5 RTTs,
     * sliding max over each window, using the maximum of the two as
     * the extra_acked bonus.
     *
     * Kernel BBR uses a single-window approach with extra_acked as a
     * bitfield in struct bbr.  KCC's dual-window design (inspired by
     * BBRplus) avoids the single-window reset problem: when one window
     * expires, the other still holds the recent peak, preventing
     * a cwnd cliff on window boundaries.
     *
     * extra_acked_win_rtts and extra_acked_win_idx are u32 here
     * (heap-allocated in struct kcc_ext) rather than bitfields in
     * BBR's inet_csk_ca slot because the bitfield budget in struct kcc
     * is consumed by Kalman-related flags.
     */
    u64 ack_epoch_mstamp;                            /* tcp_mstamp at current epoch start; used to detect epoch boundaries (epoch > ~5 RTTs triggers window switch); matches kernel BBR's approach */
    u32 extra_acked[2];                              /* dual-window sliding max (segments per ACK); index 0 and 1 alternate; max of both windows is the effective extra_acked */
    u32 ack_epoch_acked;                             /* bytes ACKed in current epoch (capped to ~1M segments to bound memory); used to compute extra_acked = max_acked - delivered */
    u32 extra_acked_win_rtts;                        /* RTTs elapsed in current window (0..31); when this exceeds the window target (~5 RTTs), KCC switches windows; stored as u32 (heap) not bitfield */
    u32 extra_acked_win_idx;                         /* which window is currently active (0 or 1); toggles on window switch; stored as u32 for simplicity */

    /*
     * ACK aggregation confidence-based compensation (BBRplus-inspired).
     * Unlike BBRplus which directly adds extra_acked to cwnd, KCC uses
     * extra_acked as a signal-quality indicator: high aggregation reduces
     * Kalman filter trust in RTT samples (by scaling up measurement noise R)
     * and only enables cwnd compensation at high confidence levels
     * (KCC_AGG_CONFIRMED and above).
     *
     * Kernel BBR: unconditional extra_acked cwnd inflation when aggregation
     * is detected.  KCC's confidence-gated approach prevents aggressive
     * cwnd inflation on ambiguous aggregation signals, which can cause
     * overshoot and loss in shallow-buffer paths.
     *
     * All fields guarded by kcc_agg_enable module param (default 1).
     */
    u32 agg_extra_acked;                             /* current window extra_acked estimate (segments); raw per-ACK aggregation for confidence evaluation */
    u32 agg_extra_acked_max;                         /* windowed maximum extra_acked (dual-slot sliding max); used as the compensation amount at CONFIRMED/TRUSTED states */
    u16 agg_confidence;                              /* confidence score 0..KCC_AGG_CONFIDENCE_MAX (1024); fused from 4 factors (qdelay consistency, extra_acked magnitude, epoch duration, jitter); no kernel BBR equivalent */
    u8  agg_state;                                   /* confidence FSM state: 0=IDLE, 1=SUSPECTED, 2=CONFIRMED, 3=TRUSTED; progressive compensation levels; no kernel BBR equivalent */
    u8  agg_comp_duration;                           /* consecutive RTTs with compensation active; watchdog counter to limit sustained compensation and prevent cwnd overshoot */
    u32 agg_r_scaled;                                /* persisting Kalman R noise scale for aggregation-state hysteresis; scaled to BBR_UNIT (256 = 1.0x, no scaling); prevents R scale from oscillating on state transitions */

    /*
     * Dynamic PROBE_RTT interval in jiffies.
     * 0 → use global defaults (kcc_probe_rtt_base_jiffies).
     * Set by kcc_update_dyn_probe_interval() based on p_est.
     * Kernel BBR: fixed 10-second PROBE_RTT interval.
     * KCC deviation: when p_est is low (filter converged), the interval
     * is extended (fewer throughput dips); when p_est is high (filter
     * uncertain), the interval is shortened (more frequent min_rtt
     * re-sampling).  Scaled by KCC_DYN_PROBE_HYPER_NUM/DEN for the
     * hyper-converged bonus (3/2 = 1.5x).
     */
    u32 dyn_probe_rtt_interval_jiffies;               /* per-connection dynamic PROBE_RTT interval; 0 = use global default; set from p_est; replaces kernel BBR's fixed 10s interval */

    /* ---- Single-flow detection (hysteresis) ---- */
    /*
     * alone_confirm_cnt: consecutive rounds that qualify as alone-on-path.
     * Incremented each round where no competing flows are detected
     * (based on RTT variance and Kalman convergence pattern).
     * When this reaches KCC_ALONE_CONFIRM_CNT_MAX (255), KCC permanently
     * transitions to BBR-pure min_rtt tracking (no Kalman filter) to
     * reduce CPU overhead.  Kernel BBR: no equivalent — always uses
     * windowed min regardless of flow count.
     */
    u8  alone_confirm_cnt;                            /* consecutive rounds qualifying as alone (0..255); transition to BBR-pure fallback at max; no kernel BBR equivalent */
    /*
     * qboost_cdwn: cooldown counter for qboost events.
     * After a qboost-triggered gain reset, this counter starts at
     * kcc_kalman_qboost_cdwn (default 15) and decrements each
     * accepted Kalman sample.  While > 0, no new qboost can fire.
     * Prevents runaway qboost events on lossy paths where large
     * innovations would otherwise trigger rapid-fire gain resets.
     * Kernel BBR: no qboost mechanism — no equivalent.
     */
    u8  qboost_cdwn;                                 /* cooldown counter: minimum accepted samples between qboost events; prevents runaway gain resets on lossy paths; no kernel BBR equivalent */
};

/*
 * CONCURRENCY & SAFETY MODEL:
 *
 * KCC follows kernel BBR exactly: only socket-layer fields accessed
 * from outside the CA module (e.g., tp->snd_cwnd) use READ_ONCE/
 * WRITE_ONCE for lock-free access from the BPF or diag paths.
 * Module parameters are read directly — a transiently stale value
 * from parallel parameter writes is harmless for congestion control.
 *
 * Kernel BBR (tcp_bbr.c) follows the same pattern: no explicit
 * synchronization beyond READ_ONCE/WRITE_ONCE on shared fields.
 * KCC's struct kcc_ext is always accessed from the socket's
 * softirq context, so no additional locking is needed.
 */
 /* ---- struct kcc_ext (end) ---- */

 /* ---- Per-Connection State (fits ICSK_CA_PRIV_SIZE = 104) ------------ */
 /*
  * struct kcc - Per-connection congestion-control state.
  *
  * Must fit within ICSK_CA_PRIV_SIZE (typically 104 bytes on x86_64).
  * Uses bitfields and careful packing.  Extended state (Kalman, etc.)
  * lives in struct kcc_ext on the heap, pointed to by kcc->ext.
  */
struct kcc {                                          /* per-connection CA state, fits ICSK_CA_PRIV_SIZE; mirrors kernel BBR's struct bbr in tcp_bbr.c but with bitfield packing to accommodate Kalman-ext pointer */
    /* core measurement state */
    u32 min_rtt_us;                                   /* Windowed-minimum RTT (us), corresponds to kernel BBR's bbr->min_rtt_us; replaced by Kalman x_est when converged in FILTER mode; range [0..U32_MAX], BBR default: initialized to ~0U (uninit sentinel) */
    u32 min_rtt_stamp;                                /* tcp_jiffies32 when min_rtt_us last updated; matches kernel BBR's bbr->min_rtt_stamp; used to determine PROBE_RTT interval expiry; range [0..U32_MAX] jiffies */
    u32 probe_rtt_done_stamp;                         /* tcp_jiffies32 deadline to exit PROBE_RTT, 0 = not entered; mirrors kernel BBR's bbr->probe_rtt_done_stamp; used to enforce minimum PROBE_RTT dwell time (default 200ms); range [0..U32_MAX] jiffies */

    struct minmax bw;                                 /* Sliding-window max bandwidth tracker (win_minmax.h), identical to kernel BBR's bbr->bw; window length = kcc_bw_rt_cycle_len (default 10 rounds); stores BW in BW_UNIT */

    u32 rtt_cnt;                                      /* Monotonic round-trip counter, matches kernel BBR's bbr->rtt_cnt; incremented each time delivered reaches next_rtt_delivered; range [0..U32_MAX] */
    u32 next_rtt_delivered;                           /* tp->delivered at next expected round boundary, matches kernel BBR's bbr->next_rtt_delivered; triggers rtt_cnt++ and round_start when tp->delivered >= this value */

    u32 cycle_mstamp_lo;                              /* Low 32 bits of PROBE_BW cycle phase MSTAMP (tcp_mstamp); KCC splits the full 64-bit tcp_mstamp into hi/lo to save bitfield space vs kernel BBR's full u64 bbr->cycle_mstamp; reconstructed via kcc_get_cycle_mstamp() */
    u32 cycle_mstamp_hi;                              /* High 32 bits of cycle MSTAMP; paired with cycle_mstamp_lo; see KCC_MSTAMP_HI_SHIFT (32) for the split/recombine logic */

    /* ---- Bitfield word 1: 32 bits (mode + flags + counters) ---- */
    struct {
        u32 mode : 2;                             /* enum kcc_mode (0..3), identical to kernel BBR's bbr->mode; 4 values: STARTUP=0, DRAIN=1, PROBE_BW=2, PROBE_RTT=3; range [0..3] */
        u32 prev_ca_state : 3;                    /* last TCP CA state before entering recovery, matches kernel BBR's bbr->prev_ca_state; used for cwnd save/restore; maps to enum tcp_ca_state: Open=0, Disorder=1, CWR=2, Recovery=3, Loss=4; range [0..7] */
        u32 round_start : 1;                      /* 1 = this ACK begins a new round-trip, matches kernel BBR's bbr->round_start; set when tp->delivered >= next_rtt_delivered; used for per-round updates (full_bw detection, EWMA decay) */
        u32 idle_restart : 1;                     /* 1 = flow was app-limited and needs restart logic, matches kernel BBR's bbr->idle_restart; triggers exponential cwnd ramp and pacing-rate reset on TX_START after idle */
        u32 probe_rtt_round_done : 1;             /* 1 = one round-trip has elapsed inside PROBE_RTT mode, matches kernel BBR's bbr->probe_rtt_round_done; used to determine when PROBE_RTT has obtained a clean min_rtt sample */
        u32 packet_conservation : 1;              /* 1 = in recovery, enforce packet conservation (cwnd = flightsize), matches kernel BBR's bbr->packet_conservation; set on loss recovery entry */
        u32 lt_is_sampling : 1;                   /* 1 = actively collecting LT BW samples within a loss-triggered interval, KCC extension beyond kernel BBR; no equivalent in kernel BBR's struct bbr; enables policer-detection mode */
        u32 lt_rtt_cnt : 12;                      /* RTT counter for LT-BW sampling interval (0..4095), KCC extension; no kernel BBR equivalent; tracks elapsed RTTs since LT interval start; max 4095 ensures wraparound safety; range [0..4095] */
        u32 min_rtt_fast_fall_cnt : 2;            /* sticky 2-bit counter for fast min_rtt drops, KCC extension to kernel BBR; when new RTT < min_rtt_us/fast_fall_div, this counter increments and triggers immediate min_rtt update on reaching kcc_minrtt_fast_fall_cnt (default 3); range [0..3] */
        u32 cycle_idx : 8;                        /* PROBE_BW cycle phase index (0..255), matches kernel BBR's bbr->cycle_idx but widened from 3 bits to 8 bits to support the 256-slot extended gain table; range [0..255]; BBR default: 8-slot cycle uses indices 0..7 */
    };

    /* ---- Bitfield word 2: 32 bits (flags + gains in BBR_SCALE) ---- */
    u32 full_bw_reached : 1;                          /* 1 = pipe capacity detected (Cardwell et al. 2016), identical to kernel BBR's bbr->full_bw_reached; gates exit from STARTUP to DRAIN and enables PROBE_BW steady-state */
    u32 full_bw_cnt : 2;                              /* consecutive rounds below growth threshold (0..3), matches kernel BBR's bbr->full_bw_cnt; triggers full_bw_reached=1 when >= kcc_full_bw_cnt (default 3); 2-bit field fits BBR range [1..3] */
    u32 has_seen_rtt : 1;                             /* 1 = tp->srtt_us has been sampled at least once, matches kernel BBR's bbr->has_seen_rtt; gates bootstrap pacing-rate initialization from RTT */
    u32 probe_rtt_restored : 1;                       /* 1 = cwnd has been restored after PROBE_RTT exit, matches kernel BBR's bbr->probe_rtt_restored; prevents double-restore if multiple ACKs arrive before the restore action completes */
    u32 lt_use_bw : 1;                                /* 1 = pace using lt_bw instead of max_bw, KCC extension; no kernel BBR equivalent; activated when policer-limited bandwidth is detected to provide stable pacing below an enforced rate ceiling */
    u32 unused_lt_restore : 5;                        /* unused, kept for struct layout compatibility */
    u32 pacing_gain : 10;                             /* Current pacing gain (0..1023, BBR_UNIT=256=1.0x), matches kernel BBR's bbr->pacing_gain; 10-bit field allows max ~4.0x; BBR default: 2885/1000*256≈739 in STARTUP, drops to ~88 in DRAIN, cycles 320/192/256 in PROBE_BW */
    u32 cwnd_gain : 10;                               /* Current cwnd gain (0..1023, BBR_UNIT=256=1.0x), matches kernel BBR's bbr->cwnd_gain; 10-bit field; BBR default: 2x BDP (512) in PROBE_BW/STARTUP, 1x (256) in DRAIN/PROBE_RTT */
    u32 alone_on_path : 1;                            /* 1 = single-flow detected, bypass Kalman and ECN guards, KCC extension; no kernel BBR equivalent; set via alone-confirmation hysteresis (kcc_alone_confirm_rounds); when active, model_rtt falls back to raw min_rtt_us for pure-BBR minimum */

    /* standalone u32 fields */
    u32 prior_cwnd;                                   /* cwnd saved before entering recovery or PROBE_RTT, matches kernel BBR's bbr->prior_cwnd; used to restore cwnd on exit from those states; range [0..U32_MAX] segments */

    u32 full_bw;                                      /* Peak bandwidth snapshot when full_bw_reached was set, matches kernel BBR's bbr->full_bw; used as the reference point for full_bw threshold comparisons; stored in BW_UNIT */

    /* ---- LT BW (Long-Term Bandwidth) estimation state ---- */
    /*
     * Activated on loss events when not in lt_use_bw mode.
     * Tracks a stable lower-bound bandwidth estimate over an interval.
     * KCC extension beyond kernel BBR: adds policer-detection capability
     * for rate-limited paths (VPN shapers, ISP throttling, WiFi capacity drops).
     * When lt_bw is consistent over multiple intervals, lt_use_bw = 1
     * and pacing switches to this stable estimate, preventing cwnd
     * oscillation above the policed rate.
     */
    u32 lt_bw;                                        /* Current LT bandwidth estimate (BW_UNIT units), KCC extension; no kernel BBR equivalent; computed as the minimum bandwidth seen over a loss-triggered interval; used when lt_use_bw=1; range [0..U32_MAX] BW_UNIT */
    u32 lt_last_delivered;                            /* tp->delivered at start of current LT interval, KCC extension; marks the beginning of the sampling window; range [0..U32_MAX] */
    u32 lt_last_stamp;                                /* Timestamp at LT interval start (jiffies), KCC extension; used with lt_last_delivered to compute bandwidth over the interval; range [0..U32_MAX] jiffies */
    u32 lt_last_lost;                                 /* tp->lost at start of current LT interval, KCC extension; used to compute loss ratio over the interval; range [0..U32_MAX] */

    struct kcc_ext* ext;                                    /* Heap-allocated extended state (Kalman filter, ECN EWMA, ACK aggregation, etc.); may be NULL if allocation failed; KCC-only: kernel BBR stores all state in the in-sock slot with no heap allocation; why: Kalman filter + confidence scoring exceed ICSK_CA_PRIV_SIZE */
};                                                     /* struct kcc */

/* ---- Module Parameters (num/den pairs, BBR core + Kalman) ---------- */
/*
 * All module parameters are exposed under /proc/sys/net/kcc/.
 * Writing any parameter triggers kcc_init_module_params() which
 * validates, clamps, and computes derived values.
 *
 * Two callback types are used:
 *   kcc_param_ops    — for scalar int params: set → kcc_init_module_params()
 *   kcc_gain_proc_handler — for array params (gain tables, decay mask):
 *                            set → kcc_rebuild_gain_table()
 */

static void kcc_init_module_params(void);             /* forward declaration: clamp + compute derived values */
static void kcc_rebuild_gain_table(void);              /* forward declaration: recompute kcc_cycle_gain_table[] */

/*
 * kcc_param_set_int - Wrapper around param_set_int.
 * After writing a raw parameter, calls kcc_init_module_params() to
 * recompute all derived values (gain tables, jiffies conversions, etc.).
 */
static int kcc_param_set_int(const char* val, const struct kernel_param* kp) /* custom param setter */
{
    int ret = param_set_int(val, kp);                 /* delegate to kernel int setter */
    if (ret == 0) {                                   /* write succeeded */
        kcc_init_module_params();                     /* recompute derived on successful write */
    }

    return ret;                                       /* return set result (0 = success) */
}
static const struct kernel_param_ops kcc_param_ops = { /* parameter ops for scalar int sysctl entries */
    .set = kcc_param_set_int,                         /* custom setter: validate then recompute */
    .get = param_get_int,                             /* standard kernel int getter */
};                                                     /* kcc_param_ops */

/* ---- PROBE_RTT intervals (seconds) ----------------------------------- */
/*
 * kcc_probe_rtt_base_sec  — Base interval between PROBE_RTT episodes (s).
 *                            Used when Kalman confidence is low or Kalman
 *                            not yet converged.  Default 10.
 * kcc_probe_rtt_max_sec   — Maximum interval when min_rtt is "long"
 *                            (> kcc_probe_rtt_long_rtt_us). Default 15.
 * kcc_probe_rtt_dyn_max_sec — Maximum dynamic interval when Kalman
 *                            p_est is at/below converged threshold.
 *                            Default 30.  If 0, dynamic interval disabled.
 */
static int kcc_probe_rtt_base_sec = 10;               /* base PROBE_RTT interval (seconds); corresponds to kernel BBR's fixed 10s probe interval (BBR_PROBE_RTT_TIMEOUT = msecs_to_jiffies(5000) * 2 in tcp_bbr.c); used when Kalman is not converged; range [1, 86400], BBR default: 10s */
module_param_cb(kcc_probe_rtt_base_sec, &kcc_param_ops, &kcc_probe_rtt_base_sec, 0644); /* sysctl: kcc_probe_rtt_base_sec */
static int kcc_probe_rtt_max_sec = 15;                /* max PROBE_RTT interval for long-RTT paths (seconds); KCC extension: kernel BBR has a single fixed interval; caps the interval when min_rtt > kcc_probe_rtt_long_rtt_us; range [1, 86400], BBR default: N/A (fixed 10s) */
module_param_cb(kcc_probe_rtt_max_sec, &kcc_param_ops, &kcc_probe_rtt_max_sec, 0644); /* sysctl: kcc_probe_rtt_max_sec */
static int kcc_probe_rtt_dyn_max_sec = 30;             /* max dynamic PROBE_RTT interval when Kalman converged (seconds); KCC extension: extends probe interval when p_est is low (filter trust is high), reducing throughput dips; 0 disables dynamic interval; range [0, 86400], BBR default: N/A (no dynamic interval) */
module_param_cb(kcc_probe_rtt_dyn_max_sec, &kcc_param_ops, &kcc_probe_rtt_dyn_max_sec, 0644); /* sysctl: kcc_probe_rtt_dyn_max_sec */

/* ---- Congestion window gain (num/den, default 2x) -------------------- */
/*
 * kcc_cwnd_gain_num / kcc_cwnd_gain_den — Target cwnd multiplier for
 * PROBE_BW mode.  Default num=2, den=1 → 2.0x BDP.
 */
static int kcc_cwnd_gain_num = 2;                     /* CWND gain numerator for PROBE_BW steady-state; corresponds to kernel BBR's bbr_cwnd_gain = 2 (2x BDP); effective gain = num/den * BBR_UNIT, clamped to 10-bit field (max 1023); range [0, 100000], BBR default: 2 */
module_param_cb(kcc_cwnd_gain_num, &kcc_param_ops, &kcc_cwnd_gain_num, 0644); /* sysctl: kcc_cwnd_gain_num */
static int kcc_cwnd_gain_den = 1;                     /* CWND gain denominator; paired with numerator; range [1, 100000], BBR default: 1 */
module_param_cb(kcc_cwnd_gain_den, &kcc_param_ops, &kcc_cwnd_gain_den, 0644); /* sysctl: kcc_cwnd_gain_den */

/* ---- ACK aggregation compensation gain (num/den, default 1x) --------- */
/*
 * kcc_extra_acked_gain_num / kcc_extra_acked_gain_den — Scaling factor
 * applied to the max extra_acked window value when computing the cwnd
 * bonus for ACK aggregation compensation.
 * Default num=1, den=1 → 1.0x.  Set num=0 to disable compensation.
 */
static int kcc_extra_acked_gain_num = 1;              /* ACK aggregation compensation gain numerator; scales the windowed max extra_acked for cwnd bonus; corresponds to kernel BBR's extra_acked gain logic in bbr_ack_aggregation_cwnd() but KCC extends with confidence-gated compensation; 0 disables compensation; range [0, 100000], BBR default: effectively 1x */
module_param_cb(kcc_extra_acked_gain_num, &kcc_param_ops, &kcc_extra_acked_gain_num, 0644); /* sysctl: kcc_extra_acked_gain_num */
static int kcc_extra_acked_gain_den = 1;              /* ACK-agg gain denominator; paired with numerator; range [1, 100000], BBR default: 1 */
module_param_cb(kcc_extra_acked_gain_den, &kcc_param_ops, &kcc_extra_acked_gain_den, 0644); /* sysctl: kcc_extra_acked_gain_den */

/* ---- PROBE_BW 256-slot gain table + decay mask (num/den) ------------- */
/*
 * kcc_gain_num[i] / kcc_gain_den[i]: pacing gain for phase i of PROBE_BW
 * cycle.  The effective gain = min((num/den)*BBR_UNIT, 1023) is stored
 * in kcc_cycle_gain_table[i].
 *
 * kcc_cycle_decay_mask[]: 256-bit mask (8x32-bit words).
 * If bit i = 1, the gain for phase i is eligible for queuing-delay and
 * jitter-based decay (reduction toward 1.0x).
 * Default: disabled (all zeros).  A common pattern is 0x01010101 per word
 * (every 8th slot, 32 slots total) for selective decay on high-gain phases.
 */
static int kcc_gain_num[KCC_GAIN_SLOTS];              /* PROBE_BW gain numerator array (256 entries) */
static int kcc_gain_den[KCC_GAIN_SLOTS];              /* PROBE_BW gain denominator array (256 entries) */
static bool kcc_gain_table_defaulted = true;            /* true until user writes gain_num/den via sysctl */
static int kcc_cycle_decay_mask[KCC_DECAY_MASK_WORDS] = { /* decay mask: 8x32-bit = 256 bits — default disabled */
    0, 0, 0, 0,                                         /* word[0..3]: no decay slots by default */
    0, 0, 0, 0                                          /* word[4..7]: no decay slots by default */
};                                                      /* 0 decay slots: probing phase preserved at full 1.25x */
/*
 * Custom array setter for gain/decay-mask parameters.
 * Uses the kernel's standard param_array_ops to parse comma-separated
 * integers; after a successful write, calls kcc_rebuild_gain_table()
 * to keep kcc_cycle_gain_table[] consistent.
 * This ensures writes via /sys/module/tcp_kcc/parameters/ and
 * /proc/sys/net/kcc/ (handled by kcc_gain_proc_handler) both
 * trigger a gain-table rebuild.
 */
extern const struct kernel_param_ops param_array_ops;

static int kcc_gain_array_set(const char* val, const struct kernel_param* kp)  /* custom setter: standard parse + rebuild */
{
    int ret = param_array_ops.set(val, kp);                                      /* delegate to kernel array setter */
    if (ret == 0) {                                                               /* write succeeded */
        kcc_gain_table_defaulted = false;                                          /* user explicitly configured gains */
        kcc_rebuild_gain_table();                                                  /* recompute kcc_cycle_gain_table[] */
    }

    return ret;                                                                      /* propagate error or success */
}
/*
 * Custom kernel_param_ops for array parameters.
 * .set wraps param_array_ops.set + kcc_rebuild_gain_table().
 * .get and .free forward to the kernel's standard param_array_ops
 * via local wrapper functions (necessary because param_array_ops
 * is an extern symbol and its members are not compile-time
 * constants for static initializers).
 */
static int kcc_gain_array_get(char* buffer, const struct kernel_param* kp)       /* forwarding wrapper */
{
    return param_array_ops.get(buffer, kp);                                        /* delegate to standard getter */
}

static void kcc_gain_array_free(void* arg)                                        /* forwarding wrapper (noop) */
{
    param_array_ops.free(arg);                                                      /* delegate to standard free */
}

static const struct kernel_param_ops kcc_gain_array_ops = {                     /* custom ops: set triggers rebuild */
    .set = kcc_gain_array_set,                                                  /* custom setter with rebuild hook */
    .get = kcc_gain_array_get,                                                  /* wrapper around param_array_ops.get */
    .free = kcc_gain_array_free,                                                 /* wrapper around param_array_ops.free */
};                                                                               /* kcc_gain_array_ops */

/* kparam_array descriptors: tell the kernel the array layout */
static struct kparam_array __param_arr_kcc_gain_num = {                         /* descriptor for kcc_gain_num[] */
    .max = KCC_GAIN_SLOTS,                                                      /* element count */
    .num = NULL,                                                                /* no runtime count tracking */
    .ops = &param_ops_int,                                                      /* per-element int setter/getter */
    .elem = kcc_gain_num,                                                        /* array base */
};                                                                               /* __param_arr_kcc_gain_num */
static struct kparam_array __param_arr_kcc_gain_den = {                         /* descriptor for kcc_gain_den[] */
    .max = KCC_GAIN_SLOTS,                                                      /* element count */
    .num = NULL,                                                                /* no runtime count tracking */
    .ops = &param_ops_int,                                                      /* per-element int setter/getter */
    .elem = kcc_gain_den,                                                        /* array base */
};                                                                               /* __param_arr_kcc_gain_den */
static struct kparam_array __param_arr_kcc_cycle_decay_mask = {                 /* descriptor for kcc_cycle_decay_mask[] */
    .max = KCC_DECAY_MASK_WORDS,                                                /* element count (8 words) */
    .num = NULL,                                                                /* no runtime count tracking */
    .ops = &param_ops_int,                                                      /* per-element int setter/getter */
    .elem = kcc_cycle_decay_mask,                                                /* array base */
};                                                                               /* __param_arr_kcc_cycle_decay_mask */

module_param_cb(kcc_gain_num, &kcc_gain_array_ops, &__param_arr_kcc_gain_num, 0644);                /* /sys/module + rebuild */
module_param_cb(kcc_gain_den, &kcc_gain_array_ops, &__param_arr_kcc_gain_den, 0644);                /* /sys/module + rebuild */
module_param_cb(kcc_cycle_decay_mask, &kcc_gain_array_ops, &__param_arr_kcc_cycle_decay_mask, 0644); /* /sys/module + rebuild */

/* ---- Kalman filter base noise (raw integer, scaled by kalman_scale) -- */
/*
 * kcc_kalman_q — Base process noise covariance Q (Kalman 1960).
 *                Internally adapted as Q' = Q * max(q_min_factor, min_rtt_us/1000).
 *                Default 100.
 *
 * kcc_kalman_r — Base measurement noise covariance R (Kalman 1960).
 *                Internally adapted as R' = R + (jitter - jr_thresh) * R / jr_scale.
 *                Default 400.
 */
static int kcc_kalman_q = 100;                        /* base process noise covariance Q (Kalman 1960); Q controls how much the filter trusts the model vs new measurements; larger Q = faster tracking but noisier estimates; KCC-only: kernel BBR has no Kalman filter; internally adapted as Q' = Q * max(q_min_factor, min_rtt_us/1000); range [0, 100000], BBR default: N/A (no Kalman filter) */
module_param_cb(kcc_kalman_q, &kcc_param_ops, &kcc_kalman_q, 0644); /* sysctl: kcc_kalman_q */
static int kcc_kalman_r = 400;                        /* base measurement noise covariance R (Kalman 1960); R controls trust in each new RTT sample; larger R = more smoothing, slower reaction; KCC-only; internally adapted as R' = R + (jitter - jr_thresh)*R/jr_scale; range [0, 100000], BBR default: N/A */
module_param_cb(kcc_kalman_r, &kcc_param_ops, &kcc_kalman_r, 0644); /* sysctl: kcc_kalman_r */

/*
 * kcc_kalman_q_rtt_div — RTT-to-ms divisor for adaptive Q scaling.
 *     Q scaled by max(q_min_factor, min_rtt_us / div).  On a 100ms path
 *     with div=1000, yields 100× scaling.  default 1000.
 */
static int kcc_kalman_q_rtt_div = 1000;             /* Q adaptation RTT divisor: Q_adapted = Q * max(q_min_factor, min_rtt_us / div); converts min_rtt_us from us to ms for adaptive Q scaling; e.g., 100ms path yields 100x scaling with div=1000; KCC-only; range [1, 1000000], BBR default: N/A */
module_param_cb(kcc_kalman_q_rtt_div, &kcc_param_ops, &kcc_kalman_q_rtt_div, 0644); /* sysctl: kcc_kalman_q_rtt_div */

/* ---- STARTUP high / drain gains (num/den, permille style) ------------ */
/*
 * kcc_high_gain_num / kcc_high_gain_den — pacing gain during STARTUP.
 *   Default: 2885/1000 = 2.885x (BBRv1 standard, Cardwell et al. 2016).
 *
 * kcc_drain_gain_num / kcc_drain_gain_den — pacing gain during DRAIN.
 *   Default: 347/1000 approx 0.347x (BBRv1 drain factor, Cardwell et al. 2016).
 */
static int kcc_high_gain_num = 2885;                  /* STARTUP pacing gain numerator; corresponds to kernel BBR's BBR_UNIT * 2885 / 1000 ≈ 2.885x high_gain (Cardwell et al. 2016); effective gain = num/den * BBR_UNIT, clamped to 10-bit field; range [0, 100000], BBR default: 2885 */
module_param_cb(kcc_high_gain_num, &kcc_param_ops, &kcc_high_gain_num, 0644); /* sysctl: kcc_high_gain_num */
static int kcc_high_gain_den = 1000;                  /* STARTUP pacing gain denominator; range [1, 100000], BBR default: 1000 */
module_param_cb(kcc_high_gain_den, &kcc_param_ops, &kcc_high_gain_den, 0644); /* sysctl: kcc_high_gain_den */
static int kcc_drain_gain_num = 347;                  /* DRAIN pacing gain numerator; corresponds to kernel BBR's drain_gain = 1/high_gain ≈ 0.347x (Cardwell et al. 2016); drains queue built during STARTUP; range [0, 100000], BBR default: 347 (1000/2885 ≈ 0.347) */
module_param_cb(kcc_drain_gain_num, &kcc_param_ops, &kcc_drain_gain_num, 0644); /* sysctl: kcc_drain_gain_num */
static int kcc_drain_gain_den = 1000;                 /* DRAIN pacing gain denominator; range [1, 100000], BBR default: 1000 */
module_param_cb(kcc_drain_gain_den, &kcc_param_ops, &kcc_drain_gain_den, 0644); /* sysctl: kcc_drain_gain_den */

/* ---- PROBE_BW cycle length (int) ------------------------------------- */
/*
 * kcc_probe_bw_cycle_len — Number of phases per PROBE_BW cycle.
 * Rounded up to a power of two (for efficient wrapping via mask).
 * Default 8, range [2, 256].
 */
static int kcc_probe_bw_cycle_len = 8;                /* PROBE_BW cycle length (number of phases per cycle); corresponds to kernel BBR's BBR_GAIN_CYCLE_LEN = 8; KCC allows [2..256] vs BBR's fixed 8; rounded up to power-of-two for efficient masking; range [2, 256], BBR default: 8 */
module_param_cb(kcc_probe_bw_cycle_len, &kcc_param_ops, &kcc_probe_bw_cycle_len, 0644); /* sysctl: kcc_probe_bw_cycle_len */

/* ---- Full BW detection threshold (num/den, default 125/100) ---------- */
/*
 * kcc_full_bw_thresh_num / kcc_full_bw_thresh_den — Growth threshold
 * for full_bw detection.  When max_bw >= full_bw * threshold, bandwidth
 * is still growing.  Default 125/100 = 1.25x (BBRv1, Cardwell et al. 2016).
 *
 * kcc_full_bw_cnt — Number of consecutive rounds below the growth
 * threshold required to declare full_bw_reached.  Default 3.
 */
static int kcc_full_bw_thresh_num = 125;              /* full-BW threshold numerator (growth ratio); corresponds to kernel BBR's bbr_full_bw_thresh = BBR_UNIT * 125 / 100 = 1.25x (Cardwell et al. 2016); when bandwidth growth is below this ratio for kcc_full_bw_cnt consecutive rounds, pipe is declared full; range [0, 100000], BBR default: 125 */
module_param_cb(kcc_full_bw_thresh_num, &kcc_param_ops, &kcc_full_bw_thresh_num, 0644); /* sysctl: kcc_full_bw_thresh_num */
static int kcc_full_bw_thresh_den = 100;              /* full-BW threshold denominator; range [1, 100000], BBR default: 100 */
module_param_cb(kcc_full_bw_thresh_den, &kcc_param_ops, &kcc_full_bw_thresh_den, 0644); /* sysctl: kcc_full_bw_thresh_den */
static int kcc_full_bw_cnt = 3;                       /* consecutive rounds without growth to declare full_bw_reached; corresponds to kernel BBR's bbr_full_bw_cnt = 3; value clamped to [1..3] to fit 2-bit bitfield; range [1, 3], BBR default: 3 */
module_param_cb(kcc_full_bw_cnt, &kcc_param_ops, &kcc_full_bw_cnt, 0644); /* sysctl: kcc_full_bw_cnt */

/* ---- Pacing margin (num/den, default 0/100 = 0%) --------------------- */
/*
 * kcc_pacing_margin_num / kcc_pacing_margin_den — Pacing rate headroom.
 * Effective divisor = 100 - (num*100/den).
 * Default 1/100 -> divisor = 99 -> rate = raw_rate * 99% (matches BBR's 1% margin).
 * Num is capped at 50 to prevent negative divisor.
 */
static int kcc_pacing_margin_num = 1;                 /* pacing margin numerator; corresponds to kernel BBR's 1% pacing margin (bbr_pacing_margin = 1% in tcp_bbr.c); effective divisor = 100 - (num*100/den); with default 1/100 => divisor=99 => rate = raw_rate * 99%; num capped at 50 to prevent negative divisor; range [0, 50], BBR default: 1 */
module_param_cb(kcc_pacing_margin_num, &kcc_param_ops, &kcc_pacing_margin_num, 0644); /* sysctl: kcc_pacing_margin_num */
static int kcc_pacing_margin_den = 100;               /* pacing margin denominator; range [1, 100000], BBR default: 100 */
module_param_cb(kcc_pacing_margin_den, &kcc_param_ops, &kcc_pacing_margin_den, 0644); /* sysctl: kcc_pacing_margin_den */

/* ---- Inflight gain bounds (num/den, percent style) ------------------- */
/*
 * Inflight bounds clamp the per-ACK cwnd target to [lo, hi] × BDP
 * in non-STARTUP modes (PROBE_BW / DRAIN / PROBE_RTT).
 *
 * ──────────────────────────── LO: 1.0× ─────────────────────────────
 *
 * kcc_inflight_low_gain_num / kcc_inflight_low_gain_den — Lower bound
 * on inflight as multiples of BDP.  Target cwnd ≥ bdp × low_gain.
 *
 *                                                ──────────────────
 * Default 100/100 = 1.0× (BBRv1 parity; deviates from BBRv2's 1.25×).
 *                                                ──────────────────
 *
 * ☆ Why 1.0×, not BBRv2's 1.25×? ☆
 *
 * BBRv2 chose inflight_lo = 1.25× to keep cwnd artificially high on
 * under-buffered paths (cellular / shallow-switch links), where the
 * extra 25 % padding compensates for transient ACK gaps and rapid
 * buffer drain.  Two implicit assumptions:
 *
 *   1. A single flow is the dominant consumer.
 *   2. Path buffers are smaller than 1 BDP.
 *
 * Neither holds for a general-purpose CC algorithm on modern
 * inter-datacentre / inter-continent links (adequate buffer sizing,
 * tens to hundreds of concurrent flows).
 *
 * The 1.25× aggregate-overcommitment pathology:
 *
 *   bottleneck_BW = B, concurrent flows = N.
 *
 *   Each flow's BDP estimate converges to ≈ B / N.
 *   With inflight_lo = 1.25×:   Σ inflight_floor = 1.25 × B
 *   With inflight_lo = 1.0 ×:   Σ inflight_floor = 1.00 × B
 *
 *   The 25 % per-flow padding MULTIPLIES with N — 8 flows produce
 *   200 % aggregate excess inflight.  The bottleneck queue never
 *   drains, inflight rarely drops below 1.25×BDP per flow, and
 *   the 1.25× floor backfires: it causes persistent queueing loss
 *   that costs more throughput than the padding was meant to protect.
 *
 * ─────────────────────────────────────────────────────────────────────
 * Multi-round A/B test (8 × flow, 60 s each, ~1 Gbps bottleneck):
 *
 *   1.0×  Run 1: 1.06 Gbps     Run 2: 1.12 Gbps     (mean ≈ 1.09 Gbps)
 *   1.25× Run 1: 1.02 Gbps     Run 2: 1.02 Gbps     (mean ≈ 1.02 Gbps)
 *                     Δ ≈ +7 % aggregate throughput at 1.0×.
 *
 *   Retransmission counts are comparable (±5 %), confirming that
 *   1.0× does not sacrifice loss resilience — it eliminates the
 *   self-inflicted loss from aggregate overcommitment.
 * ─────────────────────────────────────────────────────────────────────
 *
 * On single-flow paths (N = 1) the inflight_low bound is a floor,
 * not a ceiling.  PROBE_BW's up-phase pacing gain (1.25×) still
 * drives cwnd above BDP; the 1.0× floor only acts during the
 * down-phase to prevent cwnd from collapsing below the true pipe
 * capacity.  Single-flow throughput is therefore unaffected by
 * this change.
 *
 * In addition, KCC already compensates for cold-start / under-
 * buffered conditions through the Kalman-Filter bandwidth injection
 * (dessert-speed startup; see kcc_kf_get_init_bw), making a
 * hard-coded 1.25× cwnd pad redundant and counterproductive.
 *
 * ───────────────────────────── HI: 2.0× ──────────────────────────────
 *
 * kcc_inflight_high_gain_num / kcc_inflight_high_gain_den — Upper bound
 * on inflight in non-STARTUP modes.  Target cwnd ≤ bdp × high_gain.
 * Default 200/100 = 2.0× (BBRv1 standard).
 */
static int kcc_inflight_low_gain_num = 100;           /* inflight lower bound numerator (100/100 = 1.0x BDP); clamps the minimum per-ACK cwnd target in non-STARTUP modes to at least bdp * low_gain; KCC defaults to 1.0x vs BBRv2's 1.25x (see block comment for rationale); range [0, 100000], BBR default: BBRv1 has no explicit inflight floor (implicitly ~1.0x) */
module_param_cb(kcc_inflight_low_gain_num, &kcc_param_ops, &kcc_inflight_low_gain_num, 0644); /* sysctl: kcc_inflight_low_gain_num */
static int kcc_inflight_low_gain_den = 100;           /* inflight lower bound denominator; range [1, 100000], BBR default: 100 */
module_param_cb(kcc_inflight_low_gain_den, &kcc_param_ops, &kcc_inflight_low_gain_den, 0644); /* sysctl: kcc_inflight_low_gain_den */
static int kcc_inflight_high_gain_num = 200;          /* inflight upper bound numerator (200/100 = 2.0x BDP); caps the maximum per-ACK cwnd target in non-STARTUP modes; corresponds to kernel BBR's 2x BDP inflight cap (bbr_inflight_hi = 2*BDP); range [0, 100000], BBR default: 200 (2x) */
module_param_cb(kcc_inflight_high_gain_num, &kcc_param_ops, &kcc_inflight_high_gain_num, 0644); /* sysctl: kcc_inflight_high_gain_num */
static int kcc_inflight_high_gain_den = 100;          /* inflight upper bound denominator; range [1, 100000], BBR default: 100 */
module_param_cb(kcc_inflight_high_gain_den, &kcc_param_ops, &kcc_inflight_high_gain_den, 0644); /* sysctl: kcc_inflight_high_gain_den */

/* ---- Kalman filter bounds (int) -------------------------------------- */
/*
 * kcc_kalman_p_est_max      — Absolute upper bound on p_est.  Default 1,000,000.
 * kcc_kalman_converged_p_est — Threshold for filter convergence (Kalman 1960).
 *                             When p_est < this, cwnd reduction and dynamic
 *                             PROBE_RTT interval logic activate.  Default 500.
 * kcc_kalman_q_boost_mult   — Multiplier for Q-boost threshold.
 *   threshold = mult * kcc_kalman_q_boost_ms * 1000 * kalman_scale.
 *   Default 4.
 * kcc_kalman_q_max          — Ceiling on adaptive Q.  Default 2000.
 * kcc_kalman_q_scale_cap    — Cap on Q adaptation factor (min_rtt_us/1000).
 *                            Default 20.
 * kcc_kalman_min_samples    — Minimum Kalman updates before the filter
 *                            may overwrite min_rtt_us.  Default 5.
 */
static int kcc_kalman_p_est_max = 1000000;            /* absolute upper bound on p_est (error covariance); prevents divergence after extreme innovation; KCC-only: kernel BBR has no Kalman covariance; when p_est reaches this ceiling, the filter forces re-convergence; range [1, 100000000], BBR default: N/A */
module_param_cb(kcc_kalman_p_est_max, &kcc_param_ops, &kcc_kalman_p_est_max, 0644); /* sysctl: kcc_kalman_p_est_max */
static int kcc_kalman_converged_p_est = 500;          /* p_est convergence threshold: when p_est < this value, filter is considered converged; KCC-only; enables qboost, aggressive qdelay backoff, and dynamic PROBE_RTT interval extension; range [1, 1000000], BBR default: N/A */
module_param_cb(kcc_kalman_converged_p_est, &kcc_param_ops, &kcc_kalman_converged_p_est, 0644); /* sysctl: kcc_kalman_converged_p_est */

/*
 * kcc_drain_skip_qdelay_us — When the Kalman filter is converged AND
 *     qdelay_avg is below this threshold (us), skip the drain phase
 *     of the PROBE_BW cycle entirely.  This leverages Kalman's trusted
 *     zero-queue detection to avoid wasting 1/8 of each cycle on
 *     unnecessary draining, converting the drain phase into an
 *     additional cruise phase.  Default 1000 us (1 ms).
 */
static int kcc_drain_skip_qdelay_us = 1000;          /* qdelay (us) below which the DRAIN phase is skipped entirely; KCC extension: when Kalman is converged and qdelay is near zero, skipping DRAIN converts it into an extra cruise phase, avoiding unnecessary throughput dips; kernel BBR never skips DRAIN; range [0, 100000], BBR default: N/A */
module_param_cb(kcc_drain_skip_qdelay_us, &kcc_param_ops, &kcc_drain_skip_qdelay_us, 0644); /* sysctl: kcc_drain_skip_qdelay_us */

/*
 * kcc_kalman_probe_band_mult — Upper bound multiplier for PROBE_RTT
 *     interval transition band.  When p_est is between converged_p_est
 *     and mult × converged_p_est, interval is linearly interpolated.
 *     Above this band, uses base (conservative) interval.  default 4.
 */
static int kcc_kalman_probe_band_mult = 4;          /* PROBE_RTT interval transition band multiplier; when p_est is between converged_p_est and mult*converged_p_est, the probe interval is linearly interpolated between dyn_max and base; KCC-only; range [1, 32], BBR default: N/A */
module_param_cb(kcc_kalman_probe_band_mult, &kcc_param_ops, &kcc_kalman_probe_band_mult, 0644); /* sysctl: kcc_kalman_probe_band_mult */
static int kcc_kalman_q_boost_mult = 4;               /* Q-boost threshold multiplier: threshold = mult * q_boost_ms * 1000 * kalman_scale; when |innovation| exceeds this threshold, p_est resets to p_est_init for rapid re-convergence; KCC-only; range [1, 10000], BBR default: N/A */
module_param_cb(kcc_kalman_q_boost_mult, &kcc_param_ops, &kcc_kalman_q_boost_mult, 0644); /* sysctl: kcc_kalman_q_boost_mult */
static int kcc_kalman_q_max = 2000;                   /* maximum adaptive Q ceiling; clamps the adapted Q' = Q * max(q_min_factor, min_rtt_us/1000); prevents Q from growing unbounded on long-RTT paths; KCC-only; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_kalman_q_max, &kcc_param_ops, &kcc_kalman_q_max, 0644); /* sysctl: kcc_kalman_q_max */
static int kcc_kalman_q_scale_cap = 20;               /* cap on Q adaptation factor (min_rtt_us/1000); clamps the RTT-based scaling factor to prevent extreme Q values on very long RTT paths (e.g., satellite); KCC-only; range [1, 10000], BBR default: N/A */
module_param_cb(kcc_kalman_q_scale_cap, &kcc_param_ops, &kcc_kalman_q_scale_cap, 0644); /* sysctl: kcc_kalman_q_scale_cap */
static int kcc_kalman_min_samples = 5;                /* minimum Kalman accepted samples before min_rtt_us may be overwritten by the Kalman estimate; provides convergence guard against premature takeover; KCC-only; range [3, 20], BBR default: N/A (BBR uses windowed min_rtt exclusively) */
module_param_cb(kcc_kalman_min_samples, &kcc_param_ops, &kcc_kalman_min_samples, 0644); /* sysctl: kcc_kalman_min_samples */

/* ---- Global Kalman BDP filter (cross-connection bandwidth estimation) ---- */
/*
 * kcc_kf_enable — Master enable switch for the cross-connection
 *     Global Kalman BDP filter.  When 0 (default), each connection
 *     uses its own per-connection Kalman estimate for BDP calculation.
 *     When 1, the global KF estimate overrides per-connection BDP,
 *     enabling fair-share bandwidth allocation across all connections
 *     sharing a bottleneck.  Default 0 (off).
 */
static int kcc_kf_enable = 0;                         /* master enable switch for cross-connection Global Kalman BDP filter (0=off, 1=on); KCC-only: kernel BBR has no cross-connection bandwidth sharing; when enabled, the global KF estimate overrides per-connection BDP for fair-share bandwidth allocation across all connections sharing a bottleneck; range [0, 1], BBR default: N/A (no cross-connection KF) */
module_param_cb(kcc_kf_enable, &kcc_param_ops, &kcc_kf_enable, 0644); /* sysctl: kcc_kf_enable */
/*
 * kcc_kf_startup_r_pct — Measurement noise R as percentage of the
 *     measurement during the startup (pre-convergence) phase.  Higher
 *     R values make the KF trust the model more than new measurements,
 *     preventing wild swings before the estimate has stabilized.
 *     Default 20 (%).
 */
static int kcc_kf_startup_r_pct = 20;                 /* measurement noise R pct for startup (pre-convergence) phase; higher values = smoother but slower initial estimates; KCC-only: kernel BBR has no global KF; range [1, 100], BBR default: N/A */
module_param_cb(kcc_kf_startup_r_pct, &kcc_param_ops, &kcc_kf_startup_r_pct, 0644); /* sysctl: kcc_kf_startup_r_pct */
/*
 * kcc_kf_steady_r_pct — Measurement noise R as percentage of the
 *     measurement during steady-state (post-convergence) operation.
 *     Lower than startup R to allow faster reaction to genuine
 *     bandwidth changes once the filter has converged.
 *     Default 5 (%).
 */
static int kcc_kf_steady_r_pct = 5;                   /* measurement noise R percentage for steady-state (post-convergence) operation; lower than startup R to allow faster reaction to genuine bandwidth changes once the filter has converged; KCC-only; range [1, 100], BBR default: N/A */
module_param_cb(kcc_kf_steady_r_pct, &kcc_param_ops, &kcc_kf_steady_r_pct, 0644); /* sysctl: kcc_kf_steady_r_pct */
/*
 * kcc_kf_q_shift — Process noise shift.  Q = 1 << shift.
 *     Controls how quickly the KF "forgets" old estimates in favour
 *     of new measurements.  Higher shifts increase Q, making the
 *     filter more responsive to genuine bandwidth changes at the
 *     cost of estimate smoothness.  Default 20 (Q = 1,048,576).
 */
static int kcc_kf_q_shift = 20;                       /* global KF process noise shift: Q = 1 << shift; higher shift = more responsive to bandwidth changes but noisier estimates; KCC-only: kernel BBR has no global KF; range [0, 30], BBR default: N/A */
module_param_cb(kcc_kf_q_shift, &kcc_param_ops, &kcc_kf_q_shift, 0644); /* sysctl: kcc_kf_q_shift */
/*
 * kcc_kf_chi2_num — Chi-squared innovation gate numerator.
 * kcc_kf_chi2_den — Chi-squared innovation gate denominator.
 *     Measurements whose squared innovation exceeds the gate
 *     threshold (num/den) are rejected as outliers, preventing
 *     transient spikes from corrupting the global estimate.
 *     Default 384/100 = 3.84 (chi-squared 95% confidence, 1 DOF).
 */
static int kcc_kf_chi2_num = 384;                     /* chi-squared innovation gate numerator (outlier rejection for global KF); threshold = num/den ≈ 3.84σ (p≈0.05 for 1 dof); KCC-only: kernel BBR has no innovation gate; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_kf_chi2_num, &kcc_param_ops, &kcc_kf_chi2_num, 0644); /* sysctl: kcc_kf_chi2_num */
static int kcc_kf_chi2_den = 100;                     /* chi-squared innovation gate denominator; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_kf_chi2_den, &kcc_param_ops, &kcc_kf_chi2_den, 0644); /* sysctl: kcc_kf_chi2_den */
/*
 * kcc_kf_discount_num — Global fair-share discount numerator.
 * kcc_kf_discount_den — Global fair-share discount denominator.
 *
 * When multiple connections share a bottleneck link, the global KF
 * estimate of total available BW is discounted by this ratio before
 * being injected into each new connection, preventing over-allocation
 * across competing flows.
 *
 * --- Dessert-speed design rationale ---
 *
 * The effective initial-speed coefficient is:
 *
 *   coeff = (discount_ratio) / high_gain
 *         = (num / den) / 2.89
 *
 * where high_gain ≈ 2.89 is the BBR STARTUP pacing multiplier.
 * This places the initial injection at a conservatively low fraction
 * of the estimated fair-share bandwidth: enough to accelerate
 * cold-start meaningfully, low enough that STARTUP's 2.89× gain
 * does not overshoot into bufferbloat within the first 2–4 RTTs.
 *
 * Note that in practice the first 1–2 RTTs cannot physically saturate
 * even a low dessert speed: tcp_write_xmit enforces an initial CWND
 * of TCP_INIT_CWND (10 segments, ≈ 15 KB) for every new connection.
 * CWND only grows when remote ACKs arrive, so the dessert speed is
 * an upper bound on pacing rate — the actual throughput is CWND-limited
 * until sufficient ACKs have been received to open the window.
 *
 * The adjustable range is 35–75 (num / den = 35–75% of fair share):
 *
 *   num | coeff  | characteristic
 *   ----|--------|-------------------------------
 *    35 | 12.1%  | maximum safety, worst-path
 *    50 | 17.3%  | centre axis (default)
 *    75 | 25.9%  | mathematical dessert sweet spot
 *    80 | 27.6%  | mathematical rate ceiling (should not exceed)
 *
 * At 50 % centre: 50 / 100 / 2.89 ≈ 17.3 %.  This is the midpoint
 * between "too slow to help" and "too fast to be safe" — the KCC
 * state machine can adjust up or down within 2–4 RTTs from here.
 * Tune toward 35 for highly contended or fragile paths, toward 75
 * for clean high-BDP links.  Default 50.
 */
static int kcc_kf_discount_num = 50;                  /* global KF fair-share discount numerator (dessert-speed); initial BW injected into new connections = KF_estimate * discount_num / discount_den / high_gain; default 50/100/2.89 ≈ 17.3% of fair share; KCC-only; range [0, 100], BBR default: N/A */
module_param_cb(kcc_kf_discount_num, &kcc_param_ops, &kcc_kf_discount_num, 0644); /* sysctl: kcc_kf_discount_num */
static int kcc_kf_discount_den = 100;                 /* global KF fair-share discount denominator; paired with numerator; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_kf_discount_den, &kcc_param_ops, &kcc_kf_discount_den, 0644); /* sysctl: kcc_kf_discount_den */

/* ---- RTT sample bounds (us, int) ------------------------------------- */
/*
 * kcc_rtt_sample_max_us — RTT samples exceeding this value are discarded
 * by the Kalman filter to prevent extreme outliers from distorting x_est.
 * Default 500,000 us = 500 ms.
 */
static int kcc_rtt_sample_max_us = 500000;            /* maximum valid RTT sample (us): samples exceeding this are discarded by the Kalman filter to prevent extreme outliers from distorting x_est; KCC-only: kernel BBR has no Kalman filter; range [1, 10000000], BBR default: N/A */
module_param_cb(kcc_rtt_sample_max_us, &kcc_param_ops, &kcc_rtt_sample_max_us, 0644); /* sysctl: kcc_rtt_sample_max_us */

/*
 * kcc_kalman_rtt_dyn_mult — Dynamic RTT ceiling multiplier.
 *     rtt_max = max(kcc_rtt_sample_max_us, min_rtt_us * mult).
 *     default 2 → GEO satellite (600ms RTT) lifts 500ms floor to 1.2s.
 */
static int kcc_kalman_rtt_dyn_mult = 2;              /* dynamic RTT sample ceiling multiplier: rtt_max = max(kcc_rtt_sample_max_us, min_rtt_us * mult); on satellite paths (600ms RTT) lifts the 500ms floor to 1.2s; KCC-only; range [1, 100], BBR default: N/A */
module_param_cb(kcc_kalman_rtt_dyn_mult, &kcc_param_ops, &kcc_kalman_rtt_dyn_mult, 0644); /* sysctl: kcc_kalman_rtt_dyn_mult */

/* ---- Min-RTT tracking (num/den, percent style) ----------------------- */
/*
 * kcc_minrtt_fast_fall_cnt   — Consecutive samples below min_rtt_us/4
 *                              needed to force immediate min_rtt drop.
 *                              Default 3 (must fit in 2 bits: 0..3).
 * kcc_minrtt_sticky_num/den  — Sticky ratio for gradual min_rtt decreases.
 *   If new_rtt < min_rtt * sticky_num/sticky_den, min_rtt is reduced
 *   by sticky_num/sticky_den per sample.  Default 75/100 = 0.75.
 * kcc_minrtt_srtt_guard_num/den — SRTT sanity guard: if the smoothed RTT
 *   (SRTT/8) < min_rtt * guard_ratio, min_rtt is overridden by SRTT/8.
 *   Default 90/100 = 0.90.
 */
static int kcc_minrtt_fast_fall_cnt = 3;              /* consecutive fast-fall samples needed to force immediate min_rtt drop; when new RTT < min_rtt/fast_fall_div, this counter increments; KCC extension beyond kernel BBR's simple min_rtt tracking; fitted in 2-bit field (max 3); range [0, 3], BBR default: N/A (BBR directly updates min_rtt on any smaller sample) */
module_param_cb(kcc_minrtt_fast_fall_cnt, &kcc_param_ops, &kcc_minrtt_fast_fall_cnt, 0644); /* sysctl: kcc_minrtt_fast_fall_cnt */
static int kcc_minrtt_sticky_num = 75;                /* sticky ratio numerator for gradual min_rtt decrease: if new_rtt < min_rtt * num/den, min_rtt reduces by num/den per sample; KCC extension added to avoid over-reacting to transient RTT dips that kernel BBR would directly commit; range [0, 1000], BBR default: N/A */
module_param_cb(kcc_minrtt_sticky_num, &kcc_param_ops, &kcc_minrtt_sticky_num, 0644); /* sysctl: kcc_minrtt_sticky_num */
static int kcc_minrtt_sticky_den = 100;               /* sticky ratio denominator; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_minrtt_sticky_den, &kcc_param_ops, &kcc_minrtt_sticky_den, 0644); /* sysctl: kcc_minrtt_sticky_den */
static int kcc_minrtt_srtt_guard_num = 90;            /* SRTT guard ratio numerator: if SRTT/8 < min_rtt * num/den, min_rtt is overridden by SRTT/8; KCC extension as a sanity guard; kernel BBR has no SRTT guard; range [0, 1000], BBR default: N/A */
module_param_cb(kcc_minrtt_srtt_guard_num, &kcc_param_ops, &kcc_minrtt_srtt_guard_num, 0644); /* sysctl: kcc_minrtt_srtt_guard_num */
static int kcc_minrtt_srtt_guard_den = 100;           /* SRTT guard ratio denominator; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_minrtt_srtt_guard_den, &kcc_param_ops, &kcc_minrtt_srtt_guard_den, 0644); /* sysctl: kcc_minrtt_srtt_guard_den */

/*
 * kcc_minrtt_fast_fall_div — Divisor for fast-fall threshold.
 *     When new RTT < min_rtt_us / div, immediately commit (bypass sticky).
 *     default 4 → trigger at 25% of min_rtt.
 */
static int kcc_minrtt_fast_fall_div = 4;              /* min_rtt fast-fall threshold divisor: when new RTT < min_rtt_us / div, immediately commit the new minimum (bypass sticky logic); default 4 => 25% of current min_rtt triggers immediate update; KCC extension; range [1, 256], BBR default: N/A */
module_param_cb(kcc_minrtt_fast_fall_div, &kcc_param_ops, &kcc_minrtt_fast_fall_div, 0644); /* sysctl: kcc_minrtt_fast_fall_div */

/* ---- BDP calculation bounds (us, int) -------------------------------- */
/*
 * kcc_bdp_min_rtt_us — Floor for min_rtt_us in BDP calculation.
 * If model_rtt < this (and Kalman not yet converged), BDP returns
 * TCP_INIT_CWND.  Default 1 us (effectively disabled; matches BBR behavior).
 */
static int kcc_bdp_min_rtt_us = 1;                   /* BDP min-RTT floor (us): if min_rtt_us < this (and Kalman not converged), kcc_bdp() returns TCP_INIT_CWND; KCC extension to prevent BDP collapse on sub-millisecond paths; kernel BBR has no explicit min-RTT floor; range [0, 100000], BBR default: effectively 0 (no floor) */
module_param_cb(kcc_bdp_min_rtt_us, &kcc_param_ops, &kcc_bdp_min_rtt_us, 0644); /* sysctl: kcc_bdp_min_rtt_us */

/* ---- TSO/quantization (int) ------------------------------------------ */
/*
 * kcc_probe_cwnd_bonus — Extra segments added to cwnd target during
 * PROBE_BW phase 0 (highest-gain probe).  Helps discover bandwidth
 * above the sliding-window max.  Default 2.
 */
static int kcc_probe_cwnd_bonus = 2;                  /* extra segments added to cwnd target during PROBE_BW phase 0 (highest-gain probe phase); helps discover bandwidth above the sliding-window max; corresponds to kernel BBR's bbr_quantization_budget() probe_bonus (but BBR uses a compile-time constant of 2); range [0, 100], BBR default: 2 */
module_param_cb(kcc_probe_cwnd_bonus, &kcc_param_ops, &kcc_probe_cwnd_bonus, 0644); /* sysctl: kcc_probe_cwnd_bonus */

/* ---- EDT near-now threshold (ns) ------------------------------------- */
/*
 * kcc_edt_near_now_ns — If earliest departure time (EDT) is within this
 * many nanoseconds of now, kcc_packets_in_net_at_edt() treats delivered-
 * at-edt as zero (no packets will drain before next send).
 * Default 1000 ns = 1 us.
 */
static int kcc_edt_near_now_ns = 1000;                /* EDT near-now threshold (ns): if earliest departure time is within this many ns of now, kcc_packets_in_net_at_edt() treats delivered-at-edt as zero (no packets will drain before next send); KCC-only: kernel BBR uses a different approach for inflight estimation; range [0, 10000000], BBR default: N/A (BBR uses bbr_packets_in_net_at_edt with implicit threshold) */
module_param_cb(kcc_edt_near_now_ns, &kcc_param_ops, &kcc_edt_near_now_ns, 0644); /* sysctl: kcc_edt_near_now_ns */

/* ---- TSO rate/segs (int) --------------------------------------------- */
/*
 * kcc_min_tso_rate — Pacing rate (bytes/s) below which min_tso_segs()
 * returns 1 instead of 2.  Reduces bursts on slow paths.
 * Default 1,200,000 bytes/s (approx 9.6 Mbps).
 *
 * kcc_tso_max_segs — Maximum TSO segments per GSO skb.
 * Default 127.
 */
static int kcc_min_tso_rate = 1200000;                /* pacing rate threshold (bytes/s) below which kcc_min_tso_segs() returns kcc_tso_segs_low (default 1) instead of kcc_tso_segs_default (default 2); reduces TSO bursts on slow paths; KCC extends kernel BBR's fixed threshold with divisor adaptation; range [1, 1000000000], BBR default: effectively ~1.2M bytes/s (implicit in BBR min_tso_segs logic) */
module_param_cb(kcc_min_tso_rate, &kcc_param_ops, &kcc_min_tso_rate, 0644); /* sysctl: kcc_min_tso_rate */
static int kcc_tso_max_segs = 127;                    /* maximum TSO segments per GSO skb; corresponds to kernel BBR's sk->sk_gso_max_segs cap (127 default); range [1, 65535], BBR default: 127 */
module_param_cb(kcc_tso_max_segs, &kcc_param_ops, &kcc_tso_max_segs, 0644); /* sysctl: kcc_tso_max_segs */
/*
 * kcc_tso_segs_low — TSO segments returned by kcc_min_tso_segs() on low-rate
 *     paths (below kcc_min_tso_rate).  Default 1.
 */
static int kcc_tso_segs_low = 1;                      /* TSO segments returned by kcc_min_tso_segs() on low-rate paths (pacing below kcc_min_tso_rate); KCC extension to make the low-rate TSO behavior tunable; kernel BBR hardcodes min_tso_segs = 1 implicitly; range [1, 65535], BBR default: effectively 1 */
module_param_cb(kcc_tso_segs_low, &kcc_param_ops, &kcc_tso_segs_low, 0644); /* sysctl: kcc_tso_segs_low */
/*
 * kcc_tso_segs_default — TSO segments returned by kcc_min_tso_segs() on
 *     normal-rate paths.  Default 2.
 */
static int kcc_tso_segs_default = 2;                  /* TSO segments returned by kcc_min_tso_segs() on normal-rate paths (pacing above kcc_min_tso_rate); KCC extension for tunable normal-rate TSO; kernel BBR hardcodes min_tso_segs = 2 for normal paths; range [1, 65535], BBR default: 2 */
module_param_cb(kcc_tso_segs_default, &kcc_param_ops, &kcc_tso_segs_default, 0644); /* sysctl: kcc_tso_segs_default */

/*
 * kcc_tso_headroom_mult — TSO/GSO headroom multiplier for cwnd target.
 *     cwnd += mult × tso_segs_goal(sk).  default 3 (BBR standard).
 *     Setting 0 disables TSO headroom.
 */
static int kcc_tso_headroom_mult = 3;               /* TSO/GSO headroom multiplier for cwnd target: cwnd += mult * tso_segs_goal(sk); corresponds to kernel BBR's 3x TSO headroom in bbr_quantization_budget(); 0 disables TSO headroom; range [0, 1000], BBR default: 3 */
module_param_cb(kcc_tso_headroom_mult, &kcc_param_ops, &kcc_tso_headroom_mult, 0644); /* sysctl: kcc_tso_headroom_mult */

/*
 * kcc_min_tso_rate_div — Divisor for min_tso_rate comparison.
 *     kcc_min_tso_segs returns 1 if pacing < rate/div, else 2.
 *     default 8 (more generous than BBR's /2).
 */
static int kcc_min_tso_rate_div = 8;                /* TSO rate threshold divisor: kcc_min_tso_segs returns kcc_tso_segs_low if pacing < kcc_min_tso_rate/div, else kcc_tso_segs_default; KCC extension: BBR uses a fixed /2 divisor; default 8 is more generous (larger envelope where 1-seg TSO is used); range [1, 256], BBR default: effectively /2 */
module_param_cb(kcc_min_tso_rate_div, &kcc_param_ops, &kcc_min_tso_rate_div, 0644); /* sysctl: kcc_min_tso_rate_div */

/* ---- Jitter/Qdelay probe scaling (us) -------------------------------- */
/*
 * kcc_jitter_probe_thresh_us — Jitter threshold above which PROBE_BW
 *     gain decay activates.  Default 4000 us.
 * kcc_jitter_probe_scale_us — Scaling divisor for jitter-based gain
 *     reduction = (jitter - threshold) * BBR_UNIT / scale.  Default 16000 us.
 * kcc_qdelay_probe_thresh_us — Queuing delay threshold for gain decay.
 *     Default 5000 us.
 * kcc_qdelay_probe_scale_us — Scaling divisor for qdelay-based gain
 *     reduction.  Default 20000 us.
 * kcc_jitter_r_thresh_us — Jitter threshold above which measurement noise
 *     R is increased: R' = R + (jitter - thresh) * R / scale.
 *     Default 2000 us.
 * kcc_jitter_r_scale — Scaling divisor for adaptive R.  Default 8000.
 * kcc_kalman_r_max_boost — Maximum multiplier for jitter-based R boost.
 *     R_boost = (jitter - thresh) * base_R / scale, capped at
 *     base_R * r_max_boost.  Prevents extreme R values from freezing
 *     the Kalman gain on paths with persistent high jitter (e.g., WiFi
 *     bursts).  Default 8 → max R ≤ 9× base_R, keeping K ≥ ~10%.
 */
static int kcc_jitter_probe_thresh_us = 4000;         /* jitter threshold (us) above which PROBE_BW gain decay activates; when jitter_ewma exceeds this, pacing_gain is reduced toward 1.0x; KCC extension for jitter-adaptive probing; kernel BBR has no jitter-based gain decay; range [0, 100000], BBR default: N/A */
module_param_cb(kcc_jitter_probe_thresh_us, &kcc_param_ops, &kcc_jitter_probe_thresh_us, 0644); /* sysctl: kcc_jitter_probe_thresh_us */
static int kcc_jitter_probe_scale_us = 16000;          /* jitter scaling divisor (us) for gain decay: gain_reduction = (jitter - threshold) * BBR_UNIT / scale; larger = gentler decay; KCC extension; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_jitter_probe_scale_us, &kcc_param_ops, &kcc_jitter_probe_scale_us, 0644); /* sysctl: kcc_jitter_probe_scale_us */
static int kcc_qdelay_probe_thresh_us = 5000;          /* queuing delay threshold (us) for PROBE_BW gain decay; when qdelay_avg exceeds this, pacing_gain is reduced toward 1.0x; KCC extension for congestion-adaptive probing; kernel BBR has no qdelay-based gain decay; range [0, 100000], BBR default: N/A */
module_param_cb(kcc_qdelay_probe_thresh_us, &kcc_param_ops, &kcc_qdelay_probe_thresh_us, 0644); /* sysctl: kcc_qdelay_probe_thresh_us */
static int kcc_qdelay_probe_scale_us = 20000;          /* qdelay scaling divisor (us) for gain decay; larger = gentler decay; KCC extension; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_qdelay_probe_scale_us, &kcc_param_ops, &kcc_qdelay_probe_scale_us, 0644); /* sysctl: kcc_qdelay_probe_scale_us */
static int kcc_jitter_r_thresh_us = 2000;               /* jitter threshold (us) for adaptive Kalman measurement noise R: R' = R + (jitter - thresh) * R / scale; when jitter_ewma exceeds this, R increases to reduce trust in noisy samples; KCC-only; range [0, 100000], BBR default: N/A */
module_param_cb(kcc_jitter_r_thresh_us, &kcc_param_ops, &kcc_jitter_r_thresh_us, 0644); /* sysctl: kcc_jitter_r_thresh_us */
static int kcc_jitter_r_scale = 8000;                   /* scaling divisor for jitter-based adaptive R; larger = slower R increase with jitter; KCC-only; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_jitter_r_scale, &kcc_param_ops, &kcc_jitter_r_scale, 0644); /* sysctl: kcc_jitter_r_scale */
static int kcc_kalman_r_max_boost = 8;                 /* maximum multiplier for jitter-based R boost: R_boost = min((jitter - thresh)*base_R/scale, base_R * r_max_boost); prevents extreme R values from freezing the Kalman gain on persistently jittery paths (e.g., WiFi); KCC-only; default 8 => max R <= 9x base_R, keeping K >= ~10%; range [1, 1000], BBR default: N/A */
module_param_cb(kcc_kalman_r_max_boost, &kcc_param_ops, &kcc_kalman_r_max_boost, 0644); /* sysctl: kcc_kalman_r_max_boost */

/* ---- PROBE_RTT trigger thresholds (us) ------------------------------- */
/*
 * kcc_probe_rtt_long_rtt_us — When min_rtt_us exceeds this value,
 * the PROBE_RTT interval is divided by kcc_probe_rtt_long_interval_div
 * (default 1 = no scaling, matching BBR's fixed 10s interval).
 * Default 20,000 us = 20 ms.
 */
static int kcc_probe_rtt_long_rtt_us = 20000;           /* long-RTT threshold (us): when min_rtt_us exceeds this, the PROBE_RTT interval may be divided by kcc_probe_rtt_long_interval_div; KCC extension to adapt probe interval to path latency; kernel BBR uses a fixed 10s probe interval regardless of RTT; range [0, 10000000], BBR default: N/A (fixed interval) */
module_param_cb(kcc_probe_rtt_long_rtt_us, &kcc_param_ops, &kcc_probe_rtt_long_rtt_us, 0644); /* sysctl: kcc_probe_rtt_long_rtt_us */

/*
 * kcc_probe_rtt_long_interval_div — Divisor for PROBE_RTT interval on long-RTT
 *     paths.  Interval = base / div when min_rtt > long_rtt_threshold.
 *     default 1 (1 = no scaling, match BBR fixed 10s).  div=1 disables.
 */
static int kcc_probe_rtt_long_interval_div = 1;        /* PROBE_RTT interval divisor for long-RTT paths: interval = base / div when min_rtt > kcc_probe_rtt_long_rtt_us; default 1 = no scaling (matches kernel BBR's fixed 10s interval); div=1 effectively disables; KCC extension; range [1, 1000], BBR default: N/A (fixed) */
module_param_cb(kcc_probe_rtt_long_interval_div, &kcc_param_ops, &kcc_probe_rtt_long_interval_div, 0644); /* sysctl: kcc_probe_rtt_long_interval_div */


/* ---- LT BW (Long-Term Bandwidth) ------------------------------------ */
/*
 * kcc_lt_intvl_min_rtts — Minimum RTTs before an LT BW estimate
 *     can be produced.  Default 4.
 * kcc_lt_loss_thresh — Minimum loss ratio (BBR_UNIT units) for the
 *     LT sampling interval to be valid.  Default 15 (approx 5.9%).
 *     Suitable for WiFi/4G/5G (1-5% loss), satellite, and high-
 *     interference links.  Raise to 25-50 for very high loss paths
 *     where LT should rarely trigger.
 * kcc_lt_bw_ratio_num/den — Relative tolerance for LT BW update.
 *     |bw - lt_bw| <= ratio * lt_bw -> accept new estimate.
 *     Default 1/8 = 12.5%.
 * kcc_lt_bw_diff — Absolute byte-rate tolerance for LT BW update.
 *     Default 500 bytes/s.
 * kcc_lt_bw_max_rtts — Maximum RTTs with LT BW active before reset.
 *     Must fit in 12-bit bitfield (< 4095).  Default 48.
 */
static int kcc_lt_intvl_min_rtts = 4;                     /* minimum RTTs required before an LT BW estimate can be produced; ensures the sampling window has enough data; KCC extension: kernel BBR has no LT BW mechanism; range [1, 127], BBR default: N/A */
module_param_cb(kcc_lt_intvl_min_rtts, &kcc_param_ops, &kcc_lt_intvl_min_rtts, 0644); /* sysctl: kcc_lt_intvl_min_rtts */
static int kcc_lt_loss_thresh = 15;                       /* minimum loss ratio (BBR_UNIT units, 15 ≈ 5.9%) for an LT sampling interval to be considered valid; KCC extension; range [1, 65535], BBR default: N/A */
module_param_cb(kcc_lt_loss_thresh, &kcc_param_ops, &kcc_lt_loss_thresh, 0644); /* sysctl: kcc_lt_loss_thresh */
static int kcc_lt_intvl_max_mult = 4;                    /* LT BW sampling timeout multiplier: timeout = mult * kcc_lt_intvl_min_rtts; prevents an LT interval from persisting indefinitely; KCC extension; range [1, 32], BBR default: N/A */
module_param_cb(kcc_lt_intvl_max_mult, &kcc_param_ops, &kcc_lt_intvl_max_mult, 0644); /* sysctl: kcc_lt_intvl_max_mult */
static int kcc_lt_bw_ratio_num = 1;                       /* LT BW relative tolerance numerator: |bw - lt_bw| <= num/den * lt_bw => accept new estimate; KCC extension; range [0, 100000], BBR default: N/A */
module_param_cb(kcc_lt_bw_ratio_num, &kcc_param_ops, &kcc_lt_bw_ratio_num, 0644); /* sysctl: kcc_lt_bw_ratio_num */
static int kcc_lt_bw_ratio_den = 8;                       /* LT BW relative tolerance denominator; default 1/8 = 12.5%; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_lt_bw_ratio_den, &kcc_param_ops, &kcc_lt_bw_ratio_den, 0644); /* sysctl: kcc_lt_bw_ratio_den */
static int kcc_lt_bw_diff = 500;                          /* LT BW absolute byte-rate tolerance (bytes/s): |bw - lt_bw| <= diff => accept new estimate; KCC extension; range [0, 100000], BBR default: N/A */
module_param_cb(kcc_lt_bw_diff, &kcc_param_ops, &kcc_lt_bw_diff, 0644); /* sysctl: kcc_lt_bw_diff */
/*
 * kcc_lt_bw_ema_num / _den — LT BW EMA update coefficients.
 *     lt_bw = (new * num + old * (den - num)) / den.
 *     Default 1/2 gives exponential moving average.
 */
static int kcc_lt_bw_ema_num = 1;                         /* LT BW EMA update numerator: lt_bw = (new*num + old*(den-num))/den; KCC extension; range [0, 100], BBR default: N/A */
module_param_cb(kcc_lt_bw_ema_num, &kcc_param_ops, &kcc_lt_bw_ema_num, 0644); /* sysctl: kcc_lt_bw_ema_num */
static int kcc_lt_bw_ema_den = 2;                         /* LT BW EMA update denominator; default 1/2 gives exponential moving average; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_lt_bw_ema_den, &kcc_param_ops, &kcc_lt_bw_ema_den, 0644); /* sysctl: kcc_lt_bw_ema_den */
static int kcc_lt_bw_inst_qdelay_thresh_us = 5000;      /* LT BW gate: instantaneous queuing delay threshold (us); if qdelay exceeds this, LT BW sampling is suppressed to avoid false policer detection during transient bloat; KCC extension; range [0, 100000], BBR default: N/A */
module_param_cb(kcc_lt_bw_inst_qdelay_thresh_us, &kcc_param_ops, &kcc_lt_bw_inst_qdelay_thresh_us, 0644); /* sysctl: kcc_lt_bw_inst_qdelay_thresh_us */
static int kcc_lt_bw_max_rtts = 48;                     /* maximum RTTs with LT BW active before forced reset; must fit in 12-bit lt_rtt_cnt bitfield (< 4095); default 48 (~0.5s at 10ms RTT); KCC extension; range [1, 4094], BBR default: N/A */
module_param_cb(kcc_lt_bw_max_rtts, &kcc_param_ops, &kcc_lt_bw_max_rtts, 0644); /* sysctl: kcc_lt_bw_max_rtts */



/* ---- Kalman filter core constants (Kalman 1960) -------------------- */
/*
 * kcc_kalman_p_est_init   — Initial p_est on cold start or Q-boost reset.
 *                           Default 1000.
 * kcc_kalman_p_est_floor  — Lower bound for posterior p_est.
 *                           Default 10.
 * kcc_kalman_outlier_ms   — Base outlier threshold in milliseconds.
 *   Effective threshold = max(outlier_ms * 1000 * scale,
 *                             jitter_ewma * outlier_jitter_mult * scale).
 *   Default 5 ms.
 * kcc_kalman_q_boost_ms   — Time constant for Q-boost threshold (ms).
 *   Default 1 ms.
 * kcc_kalman_scale        — Fixed-point scaling factor for the Kalman state.
 *   x_est = measured in rtt_us * scale units.
 *   Rounded up to next power of two for fast division via shift.
 *   Default 1024.
 */
static int kcc_kalman_p_est_init = 1000;                  /* initial p_est (error covariance) on cold start or Q-boost reset; higher = more initial uncertainty, faster initial convergence rate; KCC-only; range [1, 10000000], BBR default: N/A */
module_param_cb(kcc_kalman_p_est_init, &kcc_param_ops, &kcc_kalman_p_est_init, 0644); /* sysctl: kcc_kalman_p_est_init */
static int kcc_kalman_p_est_floor = 10;                   /* lower bound for posterior p_est; prevents over-confidence (Kalman gain from going to zero); KCC-only; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_kalman_p_est_floor, &kcc_param_ops, &kcc_kalman_p_est_floor, 0644); /* sysctl: kcc_kalman_p_est_floor */
static int kcc_kalman_outlier_ms = 5;                     /* base outlier gate threshold (ms): effective threshold = max(outlier_ms*1000*scale, jitter_ewma*outlier_jitter_mult*scale); KCC-only: kernel BBR has no outlier gate; range [0, 10000], BBR default: N/A */
module_param_cb(kcc_kalman_outlier_ms, &kcc_param_ops, &kcc_kalman_outlier_ms, 0644); /* sysctl: kcc_kalman_outlier_ms */
static int kcc_kalman_q_boost_ms = 1;                     /* Q-boost time constant (ms): multiplied by q_boost_mult and kalman_scale to derive the innovation threshold that triggers p_est reset; KCC-only; range [1, 5000], BBR default: N/A */
module_param_cb(kcc_kalman_q_boost_ms, &kcc_param_ops, &kcc_kalman_q_boost_ms, 0644); /* sysctl: kcc_kalman_q_boost_ms */
static int kcc_kalman_qboost_cdwn = 15;                  /* Q-boost cooldown: minimum accepted Kalman samples between consecutive qboost events; prevents runaway gain resets on lossy paths with large innovations; KCC-only; range [1, 255], BBR default: N/A */
module_param_cb(kcc_kalman_qboost_cdwn, &kcc_param_ops, &kcc_kalman_qboost_cdwn, 0644); /* sysctl: kcc_kalman_qboost_cdwn */
static int kcc_kalman_scale = 1024;                       /* Kalman fixed-point scaling factor (power-of-two); x_est = rtt_us * scale in fixed point; rounded up to power-of-two for fast division via shift; KCC-only; range [64, 1048576], BBR default: N/A */
module_param_cb(kcc_kalman_scale, &kcc_param_ops, &kcc_kalman_scale, 0644); /* sysctl: kcc_kalman_scale */

/* ---- Kalman filter extra num/den tunables (Kalman 1960) ------------ */
/*
 * kcc_kalman_outlier_jitter_mult_num/den — Jitter multiplier for
 *     dynamic outlier threshold.  Default 4/1 = 4.
 * kcc_kalman_q_min_factor_num/den — Minimum multiplier for adaptive Q.
 *     Q_adapted = Q * max(factor, min_rtt_us/1000).  Default 10/1 = 10.
 * kcc_kalman_p_est_init_rtt_div_num/den — Alternate p_est initializer
 *     in terms of RTT: p_est = max(p_est_init, rtt_us / div).
 *     Default 10/1 = 10.
 */
static int kcc_kalman_outlier_jitter_mult_num = 4;       /* outlier jitter multiplier numerator: dynamic outlier threshold = max(base, jitter_ewma * num/den * scale); KCC-only; default 4/1 = 4x; range [0, 1000], BBR default: N/A */
module_param_cb(kcc_kalman_outlier_jitter_mult_num, &kcc_param_ops, &kcc_kalman_outlier_jitter_mult_num, 0644); /* sysctl: kcc_kalman_outlier_jitter_mult_num */
static int kcc_kalman_outlier_jitter_mult_den = 1;       /* outlier jitter multiplier denominator; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_kalman_outlier_jitter_mult_den, &kcc_param_ops, &kcc_kalman_outlier_jitter_mult_den, 0644); /* sysctl: kcc_kalman_outlier_jitter_mult_den */
static int kcc_kalman_q_min_factor_num = 10;             /* Q min factor numerator: Q_adapted = Q * max(factor, min_rtt_us/1000); default 10/1 = 10 ensures minimum Q scaling even on low-RTT paths; KCC-only; range [0, 1000], BBR default: N/A */
module_param_cb(kcc_kalman_q_min_factor_num, &kcc_param_ops, &kcc_kalman_q_min_factor_num, 0644); /* sysctl: kcc_kalman_q_min_factor_num */
static int kcc_kalman_q_min_factor_den = 1;              /* Q min factor denominator; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_kalman_q_min_factor_den, &kcc_param_ops, &kcc_kalman_q_min_factor_den, 0644); /* sysctl: kcc_kalman_q_min_factor_den */
static int kcc_kalman_p_est_init_rtt_div_num = 10;       /* p_est init RTT divisor numerator: initial p_est = max(p_est_init, rtt_us * den/num) when deriving from RTT; KCC-only; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_kalman_p_est_init_rtt_div_num, &kcc_param_ops, &kcc_kalman_p_est_init_rtt_div_num, 0644); /* sysctl: kcc_kalman_p_est_init_rtt_div_num */
static int kcc_kalman_p_est_init_rtt_div_den = 1;        /* p_est init RTT divisor denominator; default 10/1 = 10; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_kalman_p_est_init_rtt_div_den, &kcc_param_ops, &kcc_kalman_p_est_init_rtt_div_den, 0644); /* sysctl: kcc_kalman_p_est_init_rtt_div_den */

/*
 * kcc_ewma_qdelay_num/den — EWMA for qdelay smoothing.
 *   qdelay_avg = (qdelay_avg * num + new * 1) / den.
 *   Default 7/8 -> weight 0.875 old, 0.125 new.
 *
 * kcc_ewma_jitter_num/den — EWMA for jitter smoothing.
 *   jitter_avg = (jitter_avg * num + new * 1) / den.
 *   Default 7/8 -> weight 0.875 old, 0.125 new.
 */
static int kcc_ewma_qdelay_num = 7;                      /* EWMA queuing-delay numerator (old weight): qdelay_avg = (qdelay_avg*num + new*1)/den; default 7/8 = 0.875 old, 0.125 new weight; KCC-only: kernel BBR has no explicit qdelay EWMA; range [0, 100], BBR default: N/A */
module_param_cb(kcc_ewma_qdelay_num, &kcc_param_ops, &kcc_ewma_qdelay_num, 0644); /* sysctl: kcc_ewma_qdelay_num */
static int kcc_ewma_qdelay_den = 8;                      /* EWMA queuing-delay denominator; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_ewma_qdelay_den, &kcc_param_ops, &kcc_ewma_qdelay_den, 0644); /* sysctl: kcc_ewma_qdelay_den */
static int kcc_ewma_jitter_num = 7;                      /* EWMA jitter numerator (old weight): jitter_avg = (jitter_avg*num + new*1)/den; default 7/8 = 0.875 old, 0.125 new weight; KCC-only: kernel BBR has no jitter EWMA; range [0, 100], BBR default: N/A */
module_param_cb(kcc_ewma_jitter_num, &kcc_param_ops, &kcc_ewma_jitter_num, 0644); /* sysctl: kcc_ewma_jitter_num */
static int kcc_ewma_jitter_den = 8;                      /* EWMA jitter denominator; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_ewma_jitter_den, &kcc_param_ops, &kcc_ewma_jitter_den, 0644); /* sysctl: kcc_ewma_jitter_den */

/* ---- BBR-S Covariance-Matched Noise Estimation (num/den ratios) ----- */
/*
 * kcc_kalman_noise_alpha_num / kcc_kalman_noise_alpha_den — Learning rate
 *     for covariance-matched Q estimation (BBR-S method).  alpha controls
 *     how quickly q_est adapts: q_est = q_est*(1-alpha) + alpha*(K*innov)^2.
 *     Default 1/10 = 0.1.
 *
 * kcc_kalman_noise_beta_num / kcc_kalman_noise_beta_den — Learning rate
 *     for covariance-matched R estimation.
 *     Default 1/10 = 0.1.
 *
 * kcc_kalman_q_est_max — Upper bound on q_est.  Default 1,000,000,000.
 *     q_est is in (us * kalman_scale)^2 units (same implicit scale as Q).
 *     For a 10us innovation at K~0.5: (K*innov)^2 ~ 2.6e7, well within bound.
 * kcc_kalman_r_est_max — Upper bound on r_est.  Default 1,000,000,000.
 *     r_est is in (us * kalman_scale)^2 units (same implicit scale as R).
 * kcc_kalman_q_est_floor — Lower bound on q_est.  Default 1.
 * kcc_kalman_r_est_floor — Lower bound on r_est.  Default 1.
 *
 * kcc_kalman_noise_mode — Selects how covariance-matched estimates
 *     combine with heuristic Q/R:
 *       0 = disabled (use heuristic only)
 *       1 = max(heuristic, matched)  — conservative (default)
 *       2 = weighted blend (num/den configurable via noise_avg) — default (1/2) avg
 */
static int kcc_kalman_noise_alpha_num = 1;               /* BBR-S covariance-matched Q learning rate numerator: q_est = q_est*(1-alpha) + alpha*(K*innov)^2; default 1/10 = 0.1; KCC-only; range [0, 100], BBR default: N/A */
module_param_cb(kcc_kalman_noise_alpha_num, &kcc_param_ops, &kcc_kalman_noise_alpha_num, 0644); /* sysctl: kcc_kalman_noise_alpha_num */
static int kcc_kalman_noise_alpha_den = 10;              /* BBR-S adaptive Q learning rate denominator; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_kalman_noise_alpha_den, &kcc_param_ops, &kcc_kalman_noise_alpha_den, 0644); /* sysctl: kcc_kalman_noise_alpha_den */
static int kcc_kalman_noise_beta_num = 1;                /* BBR-S covariance-matched R learning rate numerator: r_est = r_est*(1-beta) + beta*max(0, innov^2/S^2 - p_pred); default 1/10 = 0.1; KCC-only; range [0, 100], BBR default: N/A */
module_param_cb(kcc_kalman_noise_beta_num, &kcc_param_ops, &kcc_kalman_noise_beta_num, 0644); /* sysctl: kcc_kalman_noise_beta_num */
static int kcc_kalman_noise_beta_den = 10;               /* BBR-S adaptive R learning rate denominator; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_kalman_noise_beta_den, &kcc_param_ops, &kcc_kalman_noise_beta_den, 0644); /* sysctl: kcc_kalman_noise_beta_den */
static int kcc_kalman_q_est_max = 1000000000;            /* upper bound on covariance-matched Q estimate; prevents unbounded Q growth from numerical artifacts; KCC-only; range [1, 2000000000], BBR default: N/A */
module_param_cb(kcc_kalman_q_est_max, &kcc_param_ops, &kcc_kalman_q_est_max, 0644); /* sysctl: kcc_kalman_q_est_max */
static int kcc_kalman_r_est_max = 1000000000;            /* upper bound on covariance-matched R estimate; prevents unbounded R growth; KCC-only; range [1, 2000000000], BBR default: N/A */
module_param_cb(kcc_kalman_r_est_max, &kcc_param_ops, &kcc_kalman_r_est_max, 0644); /* sysctl: kcc_kalman_r_est_max */
static int kcc_kalman_q_est_floor = 1;                   /* lower bound on covariance-matched Q estimate; prevents floor from hitting zero; KCC-only; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_kalman_q_est_floor, &kcc_param_ops, &kcc_kalman_q_est_floor, 0644); /* sysctl: kcc_kalman_q_est_floor */
static int kcc_kalman_r_est_floor = 1;                   /* lower bound on covariance-matched R estimate; prevents floor from hitting zero; KCC-only; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_kalman_r_est_floor, &kcc_param_ops, &kcc_kalman_r_est_floor, 0644); /* sysctl: kcc_kalman_r_est_floor */
static int kcc_kalman_noise_mode = 1;                    /* noise combination mode: 0=use heuristic Q/R only (disabled), 1=max(heuristic, covariance-matched) [default, conservative], 2=weighted blend (num/den configurable via noise_avg); KCC-only; range [0, 2], BBR default: N/A */
module_param_cb(kcc_kalman_noise_mode, &kcc_param_ops, &kcc_kalman_noise_mode, 0644); /* sysctl: kcc_kalman_noise_mode */
/*
 * kcc_rtt_mode — Selects the model RTT strategy for BDP calculation.
 *
 * Mode 1 (FILTER, default): use x_est_us directly (the raw Kalman / sliding-
 *      window filter estimate).  Recommended for production WAN / VPS
 *      deployments because:
 *
 *      a) Route-change resilience — When a BGP reroute increases physical RTT
 *         (e.g., 50 ms → 100 ms), the Kalman gain K_k reacts within a few
 *         RTTs and pulls x_est_us to the new path latency.  MIN mode deadlocks
 *         on the old min_rtt_us (50 ms) until the window expires; BDP is
 *         computed from the stale minimum, effectively cutting the pipe in
 *         half and creating a throughput cliff until the window rotates.
 *
 *      b) Built-in defenses — Outlier gating rejects RTT samples inflated by
 *         queue spikes (CoDel pass-through, ACK compression) before they
 *         enter the innovation step.  Adaptive Q/R noise matching reduces
 *         K_k when measurement noise is high, so the filter's gain naturally
 *         damps during transient bloat and keeps the estimate near the true
 *         propagation delay.
 *
 *      c) PROBE_RTT decoupling — FILTER mode computes BDP from x_est_us, not
 *         min_rtt_us, so the periodic PROBE_RTT drain is unnecessary for BDP
 *         accuracy.  The kcc_probe_rtt_decouple mechanism suppresses PROBE_RTT
 *         when the Kalman filter is healthy and reactivates it as an on-demand
 *         safety net when p_est signals divergence.
 *
 *      Result: zero throughput cliffs, no global PROBE_RTT sync collapses,
 *      application-level latency spikes eliminated.
 *
 * Mode 0 (MIN): return min(x_est_us, min_rtt_us).  Conservative: guarantees
 *      BDP never inflates beyond the windowed minimum.  Use when verifying
 *      kernel-module stability before switching to FILTER, or on links
 *      where RTT is known to be truly static.
 *
 * Runtime switch: echo {0,1} > /proc/sys/net/kcc/kcc_rtt_mode
 */
static int kcc_rtt_mode = 1;                   /* model RTT strategy for BDP calculation: 1=FILTER (default, use x_est_us directly for faster path-change adaptation), 0=MIN (use min(x_est_us, min_rtt_us) for conservative windowed-min clamp); KCC-only: kernel BBR always uses the sliding-window minimum; see block comment for detailed rationale; range [0, 1], BBR default: N/A (BBR always uses windowed min_rtt) */
module_param_cb(kcc_rtt_mode, &kcc_param_ops, &kcc_rtt_mode, 0644); /* sysctl: kcc_rtt_mode */
/*
 * kcc_probe_rtt_decouple — When kcc_rtt_mode == FILTER (1), replaces the
 *      periodic PROBE_RTT "self-mutilation" cycle with a smart Kalman-health
 *      heuristic.
 *
 * BBRv1's PROBE_RTT is a necessary evil for the window-based min_rtt
 * approach — the window cannot track true propagation delay unless the pipe
 * is periodically drained to 4 packets.  In FILTER mode the Kalman filter
 * replaces the window entirely, so the periodic self-mutilation loses its
 * mathematical necessity.
 *
 * Mode 0: Traditional PROBE_RTT — every 10 s, cwnd is clamped
 *      to 4 packets for ~200 ms to drain the pipe and measure min_rtt.
 *      This causes a periodic throughput cliff (the BBR "sawtooth").
 *
 * Mode 1 (decouple, default): When kcc_rtt_mode == FILTER, the PROBE_RTT entry
 *      trigger evaluates Kalman filter health via kcc_kalman_needs_recalibration():
 *
 *      - Kalman healthy (p_est <= kcc_recal_p_est_thresh):
 *        PROBE_RTT is suppressed.  The filter tracks true propagation
 *        delay via outlier gating and adaptive Q/R noise estimation
 *        without draining the pipe.  In normal operation p_est converges
 *        to ~p_est_floor (4--10), far below the 250k threshold, so
 *        PROBE_RTT suppression is the steady state.  Zero throughput
 *        cliffs, zero PROBE_RTT sync collapses.
 *
 *      - Kalman diverged (p_est > kcc_recal_p_est_thresh):
 *        PROBE_RTT is reactivated as a safety net.  This triggers when
 *        p_est exceeds kcc_recal_p_est_thresh (default 250k = 25 % of
 *        p_est_max).  A single traditional 4-packet drain provides a
 *        fresh min_rtt baseline; the filter re-converges and decoupling
 *        resumes automatically — an "on-demand" calibration rather than
 *        a "blind periodic" one.
 *
 *      Requires kcc_rtt_mode == 1 to take effect.  No-op in MIN mode.
 *
 * Runtime switch: echo {0,1} > /proc/sys/net/kcc/kcc_probe_rtt_decouple
 */
static int kcc_probe_rtt_decouple = 1;         /* PROBE_RTT decouple mode: 1 = skip PROBE_RTT when kcc_rtt_mode==FILTER and Kalman is healthy (p_est <= kcc_recal_p_est_thresh); eliminates the periodic throughput cliff of BBR's PROBE_RTT; 0 = traditional periodic PROBE_RTT; KCC-only: kernel BBR always uses periodic PROBE_RTT; range [0, 1], BBR default: N/A (always periodic) */
module_param_cb(kcc_probe_rtt_decouple, &kcc_param_ops, &kcc_probe_rtt_decouple, 0644); /* sysctl: kcc_probe_rtt_decouple */
/*
 * kcc_recal_p_est_thresh — Kalman error-covariance threshold for recalibration.
 * When kcc_probe_rtt_decouple is active and filter_expired fires, the Kalman
 * health is checked: if p_est > kcc_recal_p_est_thresh, the filter has lost
 * confidence and a traditional PROBE_RTT drain is triggered as a safety net.
 * Otherwise the probe is suppressed (the filter is tracking accurately).
 *
 * Default 250000 = 25 % of p_est_max (1,000,000).  During normal operation,
 * p_est converges to ~p_est_floor (typically 4--10).  A rising p_est signals
 * divergence (path change, noise model mismatch), at which point a hard
 * min_rtt re-measurement restores the filter baseline.
 */
static int kcc_recal_p_est_thresh = 250000;    /* Kalman error-covariance threshold for PROBE_RTT recalibration; default 250000 = 25% of p_est_max (1,000,000); when kcc_probe_rtt_decouple is active and p_est exceeds this, a PROBE_RTT drain is triggered as a safety net; KCC-only; range [1, 100000000], BBR default: N/A */
module_param_cb(kcc_recal_p_est_thresh, &kcc_param_ops, &kcc_recal_p_est_thresh, 0644); /* sysctl: kcc_recal_p_est_thresh */
/*
 * kcc_kalman_noise_avg_num / _den — Weighted blend ratio for noise mode 2.
 *     blend = (heuristic × (den−num) + matched × num) / den.
 *     Default 1/2 gives simple average (heuristic + matched) / 2.
 */
static int kcc_kalman_noise_avg_num = 1;               /* noise averaging numerator for mode=2 (weighted blend): blend = (heuristic*(den-num) + matched*num)/den; default 1/2 = simple average; KCC-only; range [0, 100], BBR default: N/A */
module_param_cb(kcc_kalman_noise_avg_num, &kcc_param_ops, &kcc_kalman_noise_avg_num, 0644); /* sysctl: kcc_kalman_noise_avg_num */
static int kcc_kalman_noise_avg_den = 2;               /* noise averaging denominator for mode=2; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_kalman_noise_avg_den, &kcc_param_ops, &kcc_kalman_noise_avg_den, 0644); /* sysctl: kcc_kalman_noise_avg_den */

/* ---- Single-flow detection hysteresis --------------------------------- */
/*
 * kcc_alone_confirm_rounds — Number of consecutive qualifying rounds
 *     (low qdelay + low jitter + no ECN + no agg + no LT BW) before
 *     activating single-flow mode.  Higher values add hysteresis to
 *     prevent oscillation during brief quiet periods in multi-flow
 *     competition.  Default 3 rounds.  Range [1, 32].
 */
static int kcc_alone_confirm_rounds = 3;             /* consecutive qualifying rounds before activating alone_on_path; adds hysteresis to prevent oscillation during brief quiet periods in multi-flow competition; KCC extension: kernel BBR has no single-flow detection; range [1, 32], BBR default: N/A */
module_param_cb(kcc_alone_confirm_rounds, &kcc_param_ops, &kcc_alone_confirm_rounds, 0644); /* sysctl: kcc_alone_confirm_rounds */

/*
 * kcc_alone_qdelay_thresh_us — Queuing delay upper bound (us) for
 *     single-flow detection.  qdelay_avg must be strictly below this
 *     value for the flow to qualify as alone.  Default 1000 us (1 ms).
 *     Lower = stricter (only enters on truly idle paths); higher =
 *     more permissive (enters even with minor queue).
 *     Range [0, 100000] us.
 */
static int kcc_alone_qdelay_thresh_us = 1000;          /* max queuing delay (us) for single-flow detection; qdelay_avg must be strictly below this for the flow to qualify as alone; lower = stricter; KCC extension; range [0, 100000], BBR default: N/A */
module_param_cb(kcc_alone_qdelay_thresh_us, &kcc_param_ops, &kcc_alone_qdelay_thresh_us, 0644); /* sysctl: kcc_alone_qdelay_thresh_us */

/*
 * kcc_alone_jitter_thresh_us — Jitter upper bound (us) for
 *     single-flow detection.  jitter_ewma must be strictly below this
 *     value.  Competing flows induce inter-packet timing variance;
 *     a quiet single-flow path shows only ACK-clock micro-jitter.
 *     Default 2000 us (2 ms).  Range [0, 100000] us.
 */
static int kcc_alone_jitter_thresh_us = 2000;           /* max jitter (us) for single-flow detection; jitter_ewma must be strictly below this; KCC extension; range [0, 100000], BBR default: N/A */
module_param_cb(kcc_alone_jitter_thresh_us, &kcc_param_ops, &kcc_alone_jitter_thresh_us, 0644); /* sysctl: kcc_alone_jitter_thresh_us */

/*
 * kcc_alone_agg_state_level — ACK aggregation strictness for single-flow
 *     detection.  Controls how much aggregation is tolerated before
 *     disqualifying the flow from alone mode:
 *       0 = IDLE only     (strict: zero aggregation; highest safety)
 *       1 = ≤ SUSPECTED   (moderate: allow transient agg; default)
 *       2 = < CONFIRMED   (permissive: block only persistent agg)
 *     Default 1.  Range [0, 2].
 */
static int kcc_alone_agg_state_level = 1;              /* ACK aggregation strictness for single-flow detection: 0=IDLE only (strict, highest safety), 1=<=SUSPECTED (moderate, allow transient agg, default), 2=<CONFIRMED (permissive, block only persistent agg); KCC extension; range [0, 2], BBR default: N/A */
module_param_cb(kcc_alone_agg_state_level, &kcc_param_ops, &kcc_alone_agg_state_level, 0644); /* sysctl: kcc_alone_agg_state_level */

/*
 * kcc_alone_bypass_ecn — When alone_on_path is active, skip ECN backoff
 *     (default 1).  On a single-flow path there is no competing sender
 *     to share ECN marks with; any marks are false positives from an
 *     over-sensitive AQM.  Bypassing matches BBR's zero-ECN behavior
 *     and recovers the throughput gap in single-flow scenarios.
 *     Set to 0 to keep ECN backoff active even when alone (conservative).
 *     Range [0, 1].
 */
static int kcc_alone_bypass_ecn = 1;                   /* when alone_on_path is active, skip ECN backoff (default: 1 = bypass); on single-flow paths, ECN marks are likely false positives from over-sensitive AQM; KCC extension; range [0, 1], BBR default: N/A */
module_param_cb(kcc_alone_bypass_ecn, &kcc_param_ops, &kcc_alone_bypass_ecn, 0644); /* sysctl: kcc_alone_bypass_ecn */

/* ---- ECN (Explicit Congestion Notification) --------------------------- */
/*
 * kcc_ecn_enable — Master switch for ECN-aware backoff.
 *     0 = disabled (no ECN tracking, zero overhead).
 *     1 = enabled (default).  Reads tp->delivered_ce from the TCP stack.
 *
 * kcc_ecn_backoff_num / kcc_ecn_backoff_den — Fraction by which
 *     cwnd_gain and pacing_gain (if > 1.0x) are reduced when ECN
 *     conditions are met (see kcc_ecn_backoff() for full trigger logic).
 *     Default (20/100) × BBR_UNIT ≈ 20% reduction.
 *
 * kcc_ecn_qdelay_thresh_us — Qdelay threshold (us) for ECN backoff
 *     activation via queue buildup.  Default 2000 us.
 * kcc_ecn_ewma_retained / kcc_ecn_ewma_total — EWMA weights for ECN
 *     mark ratio.  Default 3/4 -> weight 0.75 old, 0.25 new.
 */

static int kcc_ecn_enable = 1;                           /* ECN master switch: 0=disabled (no ECN tracking, zero overhead), 1=enabled (reads tp->delivered_ce); KCC extension for proactive ECN backoff; kernel BBR's ECN handling is limited to reducing cwnd per CE-marked ACK (reactive); range [0, 1], BBR default: N/A (BBR doesn't use ECN EWMA) */
module_param_cb(kcc_ecn_enable, &kcc_param_ops, &kcc_ecn_enable, 0644); /* sysctl: kcc_ecn_enable */
static int kcc_ecn_backoff_num = 20;                     /* ECN backoff percentage numerator: cwnd_gain and pacing_gain are reduced by num/den when ECN conditions are met; default 20/100 = 20% reduction; KCC extension for proportional ECN backoff; kernel BBR uses per-ACK cwnd reduction on CE; range [0, 100], BBR default: N/A */
module_param_cb(kcc_ecn_backoff_num, &kcc_param_ops, &kcc_ecn_backoff_num, 0644); /* sysctl: kcc_ecn_backoff_num */
static int kcc_ecn_backoff_den = 100;                    /* ECN backoff percentage denominator; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_ecn_backoff_den, &kcc_param_ops, &kcc_ecn_backoff_den, 0644); /* sysctl: kcc_ecn_backoff_den */
static int kcc_ecn_qdelay_thresh_us = 2000;              /* qdelay threshold (us) for ECN backoff activation via queue buildup; ECN backoff only activates when qdelay_avg exceeds this; KCC extension; range [0, 100000], BBR default: N/A */
module_param_cb(kcc_ecn_qdelay_thresh_us, &kcc_param_ops, &kcc_ecn_qdelay_thresh_us, 0644); /* sysctl: kcc_ecn_qdelay_thresh_us */
static int kcc_ecn_ewma_retained = 3;                    /* ECN EWMA retained weight (old part): ecn_ewma = (ecn_ewma*retained + instant)/total; default 3/4 = 0.75 old, 0.25 new weight; KCC extension; range [0, 100], BBR default: N/A */
module_param_cb(kcc_ecn_ewma_retained, &kcc_param_ops, &kcc_ecn_ewma_retained, 0644); /* sysctl: kcc_ecn_ewma_retained */
static int kcc_ecn_ewma_total = 4;                       /* ECN EWMA total weight (old + new); must be >= retained; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_ecn_ewma_total, &kcc_param_ops, &kcc_ecn_ewma_total, 0644); /* sysctl: kcc_ecn_ewma_total */

/*
 * kcc_ecn_idle_decay_num / kcc_ecn_idle_decay_den — Per-ACK gentle decay
 *     rate applied to ecn_ewma on every ACK where no new CE marks are
 *     detected (ce_delta == 0).  This is much slower than the round_start
 *     decay (which uses kcc_ecn_ewma_retained/total) and prevents ecn_ewma
 *     from persisting indefinitely on steady connections where round
 *     boundaries are infrequent.
 *     Default 31/32 -> ~3.2% decay per ACK,
 *     ~28% per typical RTT of 10 ACKs, halving in ~2 RTTs.
 */
static int kcc_ecn_idle_decay_num = 31;                 /* per-ACK ECN idle decay numerator: applied on every ACK without new CE marks; much slower than round-start decay; default 31/32 ≈ 3.2% decay per ACK, ~28% per RTT of 10 ACKs; KCC extension; range [1, den-1], BBR default: N/A */
module_param_cb(kcc_ecn_idle_decay_num, &kcc_param_ops, &kcc_ecn_idle_decay_num, 0644); /* sysctl: kcc_ecn_idle_decay_num */
static int kcc_ecn_idle_decay_den = 32;                 /* per-ACK ECN idle decay denominator; must be >= 2; range [2, 100000], BBR default: N/A */
module_param_cb(kcc_ecn_idle_decay_den, &kcc_param_ops, &kcc_ecn_idle_decay_den, 0644); /* sysctl: kcc_ecn_idle_decay_den */

/* ---- Kalman outlier rejection limit (int) ----------------------------- */
/*
 * kcc_kalman_max_consec_reject — Maximum consecutive outlier rejections
 *     before the Kalman filter force-accepts the next sample.  Prevents
 *     a self-reinforcing lock-in where jitter increases the dynamic
 *     rejection threshold, causing more rejections.  Default 25.
 */
static int kcc_kalman_max_consec_reject = 25;           /* maximum consecutive outlier rejections before force-accepting the next sample; prevents self-reinforcing lock-in where jitter increases the dynamic rejection threshold; KCC-only: kernel BBR has no outlier gate; range [1, 1000], BBR default: N/A */
module_param_cb(kcc_kalman_max_consec_reject, &kcc_param_ops, &kcc_kalman_max_consec_reject, 0644); /* sysctl: kcc_kalman_max_consec_reject */

/* ---- PROBE_BW cycle rand tunable ------------------------------------- */
/*
 * kcc_probe_bw_cycle_rand — Random offset range for initializing
 * PROBE_BW cycle phase on entry.  cycle_idx starts at
 * (cycle_len - 1 - rand[0, probe_bw_cycle_rand)).
 * Default 8.  Randomization prevents phase synchronization across flows
 * (Cardwell et al. 2016).
 */
static int kcc_probe_bw_cycle_rand = 8;                  /* random offset range for initializing PROBE_BW cycle phase on entry: cycle_idx starts at (cycle_len - 1 - rand[0, probe_bw_cycle_rand)); prevents phase synchronization across flows (Cardwell et al. 2016); corresponds to kernel BBR's per-flow cycle_idx randomization; range [1, cycle_len], BBR default: 8 */
module_param_cb(kcc_probe_bw_cycle_rand, &kcc_param_ops, &kcc_probe_bw_cycle_rand, 0644); /* sysctl: kcc_probe_bw_cycle_rand */
static int kcc_probe_bw_up_limit = 0;                  /* limit PROBE_BW up-phase exit conditions: 0=off (default, no restriction on up-phase exit), 1=on (restricts exit from probe phase); KCC extension for controlling probe behavior; kernel BBR has no such mechanism; range [0, 1], BBR default: N/A */
module_param_cb(kcc_probe_bw_up_limit, &kcc_param_ops, &kcc_probe_bw_up_limit, 0644); /* sysctl: kcc_probe_bw_up_limit */

/* ---- ACK aggregation max time window ms (num/den) -------------------- */
/*
 * kcc_extra_acked_max_ms_num/den — Maximum ACK aggregation epoch
 * duration in ms.  extra CWND cap = (bw * max_ms * 1000) / BW_UNIT.
 * Default 150/1 = 150 ms.
 */
static int kcc_extra_acked_max_ms_num = 150;              /* ACK aggregation maximum epoch duration numerator (ms): cap = (bw * max_ms * 1000) / BW_UNIT; default 150/1 = 150ms; KCC extension: kernel BBR's extra_acked window is in RTT units; range [0, 100000], BBR default: N/A (uses RTT-based window) */
module_param_cb(kcc_extra_acked_max_ms_num, &kcc_param_ops, &kcc_extra_acked_max_ms_num, 0644); /* sysctl: kcc_extra_acked_max_ms_num */
static int kcc_extra_acked_max_ms_den = 1;                /* ACK aggregation max window denominator; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_extra_acked_max_ms_den, &kcc_param_ops, &kcc_extra_acked_max_ms_den, 0644); /* sysctl: kcc_extra_acked_max_ms_den */

/* ---- ACK Aggregation Confidence-based Compensation (BBRplus-inspired) - */
/*
 * kcc_agg_enable — Master switch: 0 = disabled, 1 = enabled (default).
 *     When enabled, extra_acked signals feed into Kalman noise adjustment
 *     (always) and cwnd compensation (only at confidence >= threshold).
 *
 * kcc_agg_confidence_thresh — Minimum confidence score (0..1024) to
 *     enable cwnd compensation.  Default 512 (CONFIRMED state).
 *     Set > kcc_agg_confidence_max to disable cwnd compensation
 *     while keeping signal layer active.
 *
 * kcc_agg_max_comp_ratio — Maximum cwnd compensation as percentage of BDP.
 *     Default 75 (75% of BDP).  0 = no cwnd compensation.
 *
 * kcc_agg_max_comp_duration — Maximum consecutive RTTs with compensation
 *     active before watchdog forces confidence downgrade.  Default 8.
 *     Prevents stale extra_acked from persisting beyond the event.
 *
 * kcc_agg_r_hysteresis — R recovery hysteresis percentage.
 *     R increases immediately; recovery decays at (100-pct)% per RTT.
 *     Default 75 (25% decay per RTT, ~4 RTTs to return to baseline).
 *
 * kcc_agg_r_multiplier_min / max — Range for Kalman R noise scaling.
 *     256 = 1x (no scaling), 512 = 2x, 2048 = 8x.  Default 256..2048.
 */
static int kcc_agg_enable = 1;                           /* ACK aggregation confidence-based compensation master switch: 0=disabled, 1=enabled (default); when enabled, extra_acked signals feed into Kalman noise adjustment and cwnd compensation; KCC extension (BBRplus-inspired); kernel BBR uses unconditional extra_acked; range [0, 1], BBR default: 0 (no confidence gating) */
module_param_cb(kcc_agg_enable, &kcc_param_ops, &kcc_agg_enable, 0644); /* sysctl: kcc_agg_enable */
static int kcc_agg_confidence_thresh = 512;              /* minimum confidence score (0..1024) to enable cwnd compensation; default 512 = CONFIRMED state; set > kcc_agg_confidence_max to disable cwnd comp while keeping signal layer active; KCC extension; range [0, 10000], BBR default: N/A */
module_param_cb(kcc_agg_confidence_thresh, &kcc_param_ops, &kcc_agg_confidence_thresh, 0644); /* sysctl: kcc_agg_confidence_thresh */
static int kcc_agg_max_comp_ratio = 75;                 /* maximum cwnd compensation as percentage of BDP; default 75 = 75% of BDP; 0 = no cwnd compensation; KCC extension; range [0, 100], BBR default: N/A (BBR uses direct extra_acked addition) */
module_param_cb(kcc_agg_max_comp_ratio, &kcc_param_ops, &kcc_agg_max_comp_ratio, 0644); /* sysctl: kcc_agg_max_comp_ratio */
static int kcc_agg_max_comp_duration = 8;               /* maximum consecutive RTTs with active compensation before watchdog forces confidence downgrade; prevents stale extra_acked from persisting beyond the aggregation event; KCC extension; range [1, 128], BBR default: N/A */
module_param_cb(kcc_agg_max_comp_duration, &kcc_param_ops, &kcc_agg_max_comp_duration, 0644); /* sysctl: kcc_agg_max_comp_duration */
static int kcc_agg_r_hysteresis = 75;                   /* R recovery hysteresis: Kalman measurement noise R increases immediately when aggregation is detected; recovery decays at (100-pct)% per RTT; default 75 = 25% decay per RTT, ~4 RTTs to return to baseline; KCC extension; range [0, 100], BBR default: N/A */
module_param_cb(kcc_agg_r_hysteresis, &kcc_param_ops, &kcc_agg_r_hysteresis, 0644); /* sysctl: kcc_agg_r_hysteresis */
static int kcc_agg_r_multiplier_min = 256;              /* Kalman R noise scaling floor (256 = 1x = no scaling); minimum multiplier when aggregation is detected; KCC extension; range [1, 10000], BBR default: N/A */
module_param_cb(kcc_agg_r_multiplier_min, &kcc_param_ops, &kcc_agg_r_multiplier_min, 0644); /* sysctl: kcc_agg_r_multiplier_min */
static int kcc_agg_r_multiplier_max = 2048;            /* Kalman R noise scaling ceiling (2048 = 8x); maximum multiplier when aggregation is strongly detected; KCC extension; range [1, 10000], BBR default: N/A */
module_param_cb(kcc_agg_r_multiplier_max, &kcc_param_ops, &kcc_agg_r_multiplier_max, 0644); /* sysctl: kcc_agg_r_multiplier_max */

/*
 * kcc_agg_factor3_qdelay_us — Queue delay threshold for confidence
 *     Factor 3: RTT is considered "near min_rtt" if within this margin.
 *     Default 2000 us (2ms).
 *
 * kcc_agg_factor4_ratio_num / kcc_agg_factor4_ratio_den — Maximum ratio
 *     of current extra_acked to windowed max for Factor 4 to score.
 *     Default 3/2 = 1.5x.  Values within this ratio are not transient spikes.
 *
 * kcc_agg_safety_qdelay_us — Max allowed RTT above min_rtt before
 *     safety guard 1 triggers and blocks compensation.  Default 4000 us.
 *
 * kcc_agg_safety_bdp_mult — BDP multiplier for cwnd ceiling in safety
 *     guards 3 and 4.  Default 3 (3x BDP).  Compensation is blocked
 *     if cwnd or inflight exceeds this multiple of BDP.
 *
 * kcc_agg_max_window_ms — Time window (ms) for the extra_acked cap
 *     in kcc_measure_ack_aggregation: cap = bw * window_ms.
 *     Default 100 ms.
 *
 * kcc_agg_max_decay_pct — Percentage of agg_extra_acked_max retained
 *     per RTT decay in the watchdog.  75 means 25% decay per RTT.
 *     Default 75.
 *
 * kcc_agg_max_per_ack_decay — Gentle per-ACK decay of agg_extra_acked_max
 *     at round fractions (out of 128).  Prevents a single transient spike
 *     from inflating Factor 4 for an entire long RTT.  128 = no per-ACK
 *     decay (default).  127 = ~0.8% per ACK, reaching ~50% after ~87 ACKs.
 */
static int kcc_agg_factor3_qdelay_us = 2000;         /* confidence Factor 3 qdelay margin (us): RTT is considered "near min_rtt" if within this margin of min_rtt; contributes to aggregation confidence score when true; KCC extension; range [0, 100000], BBR default: N/A */
module_param_cb(kcc_agg_factor3_qdelay_us, &kcc_param_ops, &kcc_agg_factor3_qdelay_us, 0644); /* sysctl: kcc_agg_factor3_qdelay_us */
static int kcc_agg_factor4_ratio_num = 3;            /* confidence Factor 4 ratio numerator: maximum ratio of current extra_acked to windowed max for non-spike scoring; default 3/2 = 1.5x; values within this ratio indicate stable aggregation, not transient spikes; KCC extension; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_agg_factor4_ratio_num, &kcc_param_ops, &kcc_agg_factor4_ratio_num, 0644); /* sysctl: kcc_agg_factor4_ratio_num */
static int kcc_agg_factor4_ratio_den = 2;            /* confidence Factor 4 ratio denominator; range [1, 100000], BBR default: N/A */
module_param_cb(kcc_agg_factor4_ratio_den, &kcc_param_ops, &kcc_agg_factor4_ratio_den, 0644); /* sysctl: kcc_agg_factor4_ratio_den */
static int kcc_agg_safety_qdelay_us = 4000;          /* Safety Guard 1: max allowed RTT above min_rtt (us) before blocking compensation; if current RTT - min_rtt exceeds this, compensation is blocked to prevent cwnd overshoot; KCC extension; range [0, 100000], BBR default: N/A */
module_param_cb(kcc_agg_safety_qdelay_us, &kcc_param_ops, &kcc_agg_safety_qdelay_us, 0644); /* sysctl: kcc_agg_safety_qdelay_us */
static int kcc_agg_safety_bdp_mult = 3;              /* Safety Guard 3/4: BDP multiplier for cwnd ceiling; compensation is blocked if cwnd or inflight exceeds this multiple of BDP; default 3 = 3x BDP; KCC extension; range [1, 100], BBR default: N/A */
module_param_cb(kcc_agg_safety_bdp_mult, &kcc_param_ops, &kcc_agg_safety_bdp_mult, 0644); /* sysctl: kcc_agg_safety_bdp_mult */
static int kcc_agg_max_window_ms = 100;              /* extra_acked cap window (ms): cap = bw * window_ms used in kcc_measure_ack_aggregation(); KCC extension; range [1, 10000], BBR default: N/A */
module_param_cb(kcc_agg_max_window_ms, &kcc_param_ops, &kcc_agg_max_window_ms, 0644); /* sysctl: kcc_agg_max_window_ms */
static int kcc_agg_max_decay_pct = 75;               /* watchdog decay percentage: agg_extra_acked_max retained per RTT in the watchdog; 75 = 25% decay per RTT; KCC extension; range [0, 100], BBR default: N/A */
module_param_cb(kcc_agg_max_decay_pct, &kcc_param_ops, &kcc_agg_max_decay_pct, 0644); /* sysctl: kcc_agg_max_decay_pct */
static int kcc_agg_max_per_ack_decay = 128;          /* per-ACK gentle decay of agg_extra_acked_max: 128=no decay (default), 127=~0.8%/ACK; prevents a single transient spike from inflating Factor 4 for an entire long RTT; KCC extension; range [0, 128], BBR default: N/A */
module_param_cb(kcc_agg_max_per_ack_decay, &kcc_param_ops, &kcc_agg_max_per_ack_decay, 0644); /* sysctl: kcc_agg_max_per_ack_decay */
static int kcc_agg_max_per_ack_decay_den = 128;      /* per-ACK decay denominator: decay = value/den; range [1, 65535], BBR default: N/A */
module_param_cb(kcc_agg_max_per_ack_decay_den, &kcc_param_ops, &kcc_agg_max_per_ack_decay_den, 0644); /* sysctl: kcc_agg_max_per_ack_decay_den */
static int kcc_agg_window_rotation_rtts = 5;         /* ACK aggregation dual-window rotation period (RTTs): after this many RTTs, the active window switches to the other slot, keeping the max of both windows; KCC extension (BBRplus-inspired); kernel BBR uses a single window; range [1, 65535], BBR default: N/A */
module_param_cb(kcc_agg_window_rotation_rtts, &kcc_param_ops, &kcc_agg_window_rotation_rtts, 0644); /* sysctl: kcc_agg_window_rotation_rtts */
static int kcc_extra_acked_win_rtts_max = 31;        /* maximum RTTs for dual-window ACK aggregation rotation before the secondary window must take over; bounds the window switch to prevent stale windows persisting indefinitely; KCC extension; range [1, 65535], BBR default: N/A */
module_param_cb(kcc_extra_acked_win_rtts_max, &kcc_param_ops, &kcc_extra_acked_win_rtts_max, 0644); /* sysctl: kcc_extra_acked_win_rtts_max */

/*
 * kcc_agg_factor_weight — Per-factor confidence score increment.
 *     default 256; with 4 factors, max total = 4 × 256 = 1024.
 *
 * kcc_agg_confidence_max — Confidence scaling denominator (max range).
 *     default 1024; maps confidence [0, max] to [0, r_max - r_min] range.
 *
 * kcc_agg_thresh_suspected — Confidence >= this → SUSPECTED state (default 256).
 * kcc_agg_thresh_confirmed — Confidence >= this → CONFIRMED state (default 512).
 * kcc_agg_thresh_trusted — Confidence >= this → TRUSTED state (default 768).
 */
static int kcc_agg_factor_weight = 256;            /* confidence score increment per satisfied factor; with 4 factors, max total = 4 * 256 = 1024 = KCC_AGG_CONFIDENCE_MAX; KCC extension; range [1, KCC_AGG_CONFIDENCE_MAX], BBR default: N/A */
module_param_cb(kcc_agg_factor_weight, &kcc_param_ops, &kcc_agg_factor_weight, 0644); /* sysctl: kcc_agg_factor_weight */
static int kcc_agg_confidence_max = 1024;          /* confidence scaling denominator (max range); maps confidence [0, max] to [0, r_max - r_min] for Kalman R scaling; KCC extension; range [1, 65536], BBR default: N/A */
module_param_cb(kcc_agg_confidence_max, &kcc_param_ops, &kcc_agg_confidence_max, 0644); /* sysctl: kcc_agg_confidence_max */
static int kcc_agg_thresh_suspected = 256;         /* SUSPECTED state threshold: confidence >= this enters SUSPECTED state (transient aggregation suspected but not confirmed); KCC extension; range [1, 1024], BBR default: N/A */
module_param_cb(kcc_agg_thresh_suspected, &kcc_param_ops, &kcc_agg_thresh_suspected, 0644); /* sysctl: kcc_agg_thresh_suspected */
static int kcc_agg_thresh_confirmed = 512;         /* CONFIRMED state threshold: confidence >= this enters CONFIRMED state (aggregation confirmed, cwnd compensation may activate); KCC extension; range [2, 1024], BBR default: N/A */
module_param_cb(kcc_agg_thresh_confirmed, &kcc_param_ops, &kcc_agg_thresh_confirmed, 0644); /* sysctl: kcc_agg_thresh_confirmed */
static int kcc_agg_thresh_trusted = 768;           /* TRUSTED state threshold: confidence >= this enters TRUSTED state (aggregation is stable and persistent, full compensation); KCC extension; range [3, 1024], BBR default: N/A */
module_param_cb(kcc_agg_thresh_trusted, &kcc_param_ops, &kcc_agg_thresh_trusted, 0644); /* sysctl: kcc_agg_thresh_trusted */

/* ---- PROBE_RTT mode duration ms (num/den) ---------------------------- */
/*
 * kcc_probe_rtt_mode_ms_num/den — Minimum time (ms) spent in PROBE_RTT
 * after inflight drops to min_target.  Default 200/1 = 200 ms (BBRv1,
 * Cardwell et al. 2016).
 */
static int kcc_probe_rtt_mode_ms_num = 200;               /* PROBE_RTT minimum stay duration numerator (ms): minimum time spent in PROBE_RTT after inflight drops to min_target; default 200/1 = 200ms; corresponds to kernel BBR's probe_rtt_duration_ms = 200ms (Cardwell et al. 2016); range [1, 100000], BBR default: 200ms */
module_param_cb(kcc_probe_rtt_mode_ms_num, &kcc_param_ops, &kcc_probe_rtt_mode_ms_num, 0644); /* sysctl: kcc_probe_rtt_mode_ms_num */
static int kcc_probe_rtt_mode_ms_den = 1;                 /* PROBE_RTT stay duration denominator; range [1, 100000], BBR default: 1 */
module_param_cb(kcc_probe_rtt_mode_ms_den, &kcc_param_ops, &kcc_probe_rtt_mode_ms_den, 0644); /* sysctl: kcc_probe_rtt_mode_ms_den */

/* ---- Other misc constants -------------------------------------------- */
/*
 * kcc_bw_rt_cycle_len — Number of round-trip windows for the sliding max
 * bandwidth filter (minmax).  Default 10 (BBRv1, Cardwell et al. 2016).
 * kcc_cwnd_min_target — Absolute minimum cwnd floor in segments.
 * Default 4 (BBRv1, Cardwell et al. 2016).
 */
static int kcc_bw_rt_cycle_len = 10;                      /* sliding-window max bandwidth filter window length (round-trips); corresponds to kernel BBR's bbr_bw_cycle_len = 10 (Cardwell et al. 2016); used in struct minmax for bandwidth tracking; range [2, 256], BBR default: 10 */
module_param_cb(kcc_bw_rt_cycle_len, &kcc_param_ops, &kcc_bw_rt_cycle_len, 0644); /* sysctl: kcc_bw_rt_cycle_len */
static int kcc_cwnd_min_target = 4;                       /* absolute minimum cwnd floor (segments); corresponds to kernel BBR's BBR_MIN_CWND = 4 (Cardwell et al. 2016); used as the inflight target during PROBE_RTT drain; range [1, 1000], BBR default: 4 */
module_param_cb(kcc_cwnd_min_target, &kcc_param_ops, &kcc_cwnd_min_target, 0644); /* sysctl: kcc_cwnd_min_target */

/*
 * kcc_sndbuf_expand_factor — Send buffer = N × cwnd (BBR standard: 3x).
 *     default 3, range 2-100.
 */
static int kcc_sndbuf_expand_factor = 3;              /* send buffer expansion factor: sk_sndbuf = factor * cwnd * MSS; corresponds to kernel BBR's 3x sndbuf factor (bbr_sndbuf_expand); KCC makes this tunable vs BBR's compile-time constant; range [2, 100], BBR default: 3 */
module_param_cb(kcc_sndbuf_expand_factor, &kcc_param_ops, &kcc_sndbuf_expand_factor, 0644); /* sysctl: kcc_sndbuf_expand_factor */

/*
 * kcc_ack_epoch_max — Epoch byte accumulator cap (~1M default).
 *     Prevents u32 overflow in extra_acked = ack_epoch_acked - expected_acked.
 *     When approaching this cap, the epoch resets.  default 0xFFFFF ≈ 1M.
 */
static int kcc_ack_epoch_max = 0xFFFFF;               /* ACK aggregation epoch byte accumulator cap (~1M bytes); prevents u32 overflow in extra_acked = ack_epoch_acked - expected_acked; when approaching this cap, the epoch resets; KCC extension: kernel BBR uses a different mechanism for bounding the epoch; range [65536, 0x7FFFFFFF], BBR default: N/A */
module_param_cb(kcc_ack_epoch_max, &kcc_param_ops, &kcc_ack_epoch_max, 0644); /* sysctl: kcc_ack_epoch_max */

/* ---- Internal Derived Variables -------------------------------------- */
/*
 * These are computed by kcc_init_module_params() from the raw module
 * parameters.  No concurrent-write protection needed — see the
 * "CONCURRENCY & SAFETY MODEL" comment at struct kcc_ext for details.
 */
static u32 kcc_probe_rtt_base_jiffies;                    /* kcc_probe_rtt_base_sec * HZ (computed at module init) */
static u32 kcc_probe_rtt_max_jiffies;                     /* kcc_probe_rtt_max_sec * HZ (computed at module init) */
static u32 kcc_probe_rtt_dyn_max_jiffies;                 /* kcc_probe_rtt_dyn_max_sec * HZ (computed at module init) */

static u32 kcc_cwnd_gain_val;                             /* num/den * BBR_UNIT (clamped 0..1023) (computed at module init) */
static u32 kcc_cycle_gain_table[KCC_GAIN_SLOTS];           /* pre-computed PROBE_BW gains (read-on-connect) */

/*
 * kcc_rebuild_gain_table - Recompute the 256-slot cycle gain table.
 *
 * For each slot i:
 *   effective_gain = min( BBR_UNIT * num[i] / den[i] , 1023 )
 *
 * den is floored at 1, num at 0 to prevent invalid values.
 * Called at module init and whenever kcc_gain_num[] or kcc_gain_den[]
 * is written via sysctl (via kcc_gain_proc_handler).
 *
 * Note: raw array params are not clamped at sysctl write time; the
 * effective gain is capped at KCC_GAIN_MAX (1023) at computation time.
 */
static void kcc_rebuild_gain_table(void)                    /* recompute kcc_cycle_gain_table[] from num/den arrays */
{
    int i;                                                  /* loop index over KCC_GAIN_SLOTS */
    for (i = 0; i < KCC_GAIN_SLOTS; i++) {                  /* iterate through all 256 gain slots */
        int num = kcc_gain_num[i];
        int den = kcc_gain_den[i];
        if (den < 1) {
            den = 1;
        }                           /* floor: prevent div-by-zero */
        if (num < 0) {
            num = 0;
        }                           /* floor: gain cannot be negative */
        kcc_cycle_gain_table[i] = (u32)min_t(u64, ((u64)BBR_UNIT * (u32)num) / (u32)den, KCC_GAIN_MAX);
        /* Use corrected locals; do NOT write back to raw params */
    }
}
/*
 * kcc_cycle_decay_enabled - Query whether the decay bit is set for a given
 * cycle phase index.
 *
 * The decay mask is a 256-bit bitmap stored as 8 x 32-bit words.
 *   idx >> 5  -> word index (0..7)
 *   idx & 31  -> bit index (0..31)
 */
static inline bool kcc_cycle_decay_enabled(u32 idx)         /* check if decay bit is set for phase idx */
{
    return ((unsigned int)kcc_cycle_decay_mask[(idx >> KCC_DECAY_WORD_BITS) & (KCC_DECAY_MASK_WORDS - 1)] >> (idx & KCC_DECAY_BIT_MASK)) & KCC_DECAY_MASK_LSB;
}
/*
 * kcc_gain_proc_handler - Custom sysctl handler for the three array-type
 * parameters: kcc_gain_num[], kcc_gain_den[], kcc_cycle_decay_mask[].
 *
 * Delegates to proc_dointvec() for array read/write, then calls
 * kcc_rebuild_gain_table() after any successful write to refresh the
 * kcc_cycle_gain_table[] cache.
 */
static int kcc_gain_proc_handler(struct ctl_table* ctl, int write, /* custom sysctl handler for gain arrays */
    void* buffer, size_t* lenp, loff_t* ppos)         /* standard sysctl callback signature */
{
    int ret = proc_dointvec(ctl, write, buffer, lenp, ppos);       /* delegate array read/write to kernel */
    if (write && ret == 0) {                                       /* write succeeded */
        kcc_gain_table_defaulted = false;                           /* user explicitly configured via sysctl */
        kcc_rebuild_gain_table();                                  /* refresh pre-computed gain table */
    }

    return ret;                                                    /* return result from proc_dointvec */
}
/* Derived scalars — all populated by kcc_init_module_params() */
static u32 kcc_extra_acked_gain_val;                       /* ACK-agg gain in BBR_UNIT (computed at module init) */
static u32 kcc_high_gain_val;                              /* STARTUP pacing gain in BBR_UNIT (computed at module init) */
static u32 kcc_drain_gain_val;                             /* DRAIN pacing gain in BBR_UNIT (computed at module init) */
static u32 kcc_probe_bw_cycle_len_val;                     /* clamped & power-of-two cycle length (computed at module init) */
static u32 kcc_full_bw_thresh_val;                         /* full-BW detection threshold in BBR_UNIT (computed at module init) */
static u32 kcc_full_bw_cnt_val;                            /* clamped full-BW round count (computed at module init) */

static u32 kcc_inflight_low_gain_val;                      /* inflight lower bound in BBR_UNIT (computed at module init) */
static u32 kcc_inflight_high_gain_val;                     /* inflight upper bound in BBR_UNIT (computed at module init) */
static u32 kcc_kalman_p_est_max_val;                       /* clamped p_est max (computed at module init) */
static u32 kcc_kalman_converged_p_est_val;                  /* clamped convergence p_est (computed at module init) */
static u32 kcc_recal_p_est_thresh_val;                     /* clamped recalibration p_est threshold (computed at module init) */
static u32 kcc_drain_skip_qdelay_us_val;                   /* clamped drain skip qdelay (computed at module init) */
static u32 kcc_kalman_q_boost_thresh_val;                    /* computed Q-boost threshold (computed at module init) */
static u32 kcc_kalman_qboost_cdwn_val;                       /* clamped Q-boost cooldown (computed at module init) */
static int kcc_kalman_q_max_val;                            /* clamped Q max (computed at module init) */
static int kcc_kalman_q_scale_cap_val;                      /* clamped Q scale cap (computed at module init) */
static int kcc_kalman_min_samples_val;                      /* clamped min samples for takeover (computed at module init) */
static u32 kcc_rtt_sample_max_us_val;                       /* clamped max RTT sample (computed at module init) */
static int kcc_minrtt_fast_fall_cnt_val;                    /* clamped fast-fall count (computed at module init) */
static u32 kcc_minrtt_sticky_num_val;                       /* cached sticky num (computed at module init) */
static u32 kcc_minrtt_sticky_den_val;                       /* cached sticky den (computed at module init) */
static u32 kcc_minrtt_srtt_guard_num_val;                   /* cached SRTT guard num (computed at module init) */
static u32 kcc_minrtt_srtt_guard_den_val;                   /* cached SRTT guard den (computed at module init) */
static u32 kcc_bdp_min_rtt_us_val;                          /* clamped BDP min RTT (computed at module init) */
static u32 kcc_probe_cwnd_bonus_val;                        /* clamped probe cwnd bonus (computed at module init) */
static int kcc_edt_near_now_ns_val;                         /* clamped EDT near-now threshold (computed at module init) */
static int kcc_min_tso_rate_val;                            /* clamped min TSO rate (computed at module init) */
static int kcc_tso_max_segs_val;                            /* clamped max TSO segs (computed at module init) */
static int kcc_agg_enable_val;                           /* clamped agg enable flag (computed at module init) */
static int kcc_agg_confidence_thresh_val;                /* clamped confidence threshold (computed at module init) */
static int kcc_agg_max_comp_ratio_val;                   /* clamped max comp ratio (computed at module init) */
static int kcc_agg_max_comp_duration_val;                /* clamped max comp duration (computed at module init) */
static int kcc_agg_r_hysteresis_val;                     /* clamped R hysteresis (computed at module init) */
static u32 kcc_agg_r_multiplier_min_val;                 /* clamped R mult min (computed at module init) */
static u32 kcc_agg_r_multiplier_max_val;                 /* clamped R mult max (computed at module init) */
static int kcc_agg_factor3_qdelay_us_val;            /* clamped Factor 3 qdelay (computed at module init) */
static int kcc_agg_factor4_ratio_num_val;            /* snapped Factor 4 ratio num (computed at module init) */
static int kcc_agg_factor4_ratio_den_val;            /* snapped Factor 4 ratio den (computed at module init) */
static int kcc_agg_safety_qdelay_us_val;             /* clamped safety qdelay (computed at module init) */
static int kcc_agg_safety_bdp_mult_val;              /* clamped safety BDP mult (computed at module init) */
static int kcc_agg_max_window_ms_val;                /* clamped max window ms (computed at module init) */
static int kcc_agg_max_decay_pct_val;                /* clamped max decay pct (computed at module init) */
static int kcc_agg_max_per_ack_decay_val;            /* clamped per-ACK decay (computed at module init) */
static int kcc_agg_max_per_ack_decay_den_val;        /* clamped per-ACK decay denom (computed at module init) */
static int kcc_agg_window_rotation_rtts_val;         /* clamped window rotation RTTs (computed at module init) */
static int kcc_lt_bw_ema_num_val;                       /* cached LT BW EMA numerator (computed at module init) */
static int kcc_lt_bw_ema_den_val;                       /* cached LT BW EMA denominator (computed at module init) */
static int kcc_kalman_noise_avg_num_val;                 /* cached noise avg numerator (computed at module init) */
static int kcc_kalman_noise_avg_den_val;                 /* cached noise avg denominator (computed at module init) */
static int kcc_tso_segs_low_val;                         /* cached TSO segs low (computed at module init) */
static int kcc_tso_segs_default_val;                     /* cached TSO segs default (computed at module init) */
static int kcc_extra_acked_win_rtts_max_val;              /* cached extra acked win rtts max (computed at module init) */

static u32 kcc_kalman_noise_alpha_complement;           /* precomputed alpha_d - alpha_n (computed at module init) */
static u32 kcc_kalman_noise_beta_complement;             /* precomputed beta_d - beta_n (computed at module init) */
static bool kcc_agg_per_ack_decay_active;                 /* precomputed: per_ack_decay < den (computed at module init) */

static int kcc_jitter_probe_thresh_us_val;                  /* clamped jitter threshold for gain decay (computed at module init) */
static int kcc_jitter_probe_scale_us_val;                   /* clamped jitter scale for gain decay (computed at module init) */
static int kcc_qdelay_probe_thresh_us_val;                  /* clamped qdelay threshold for gain decay (computed at module init) */
static int kcc_qdelay_probe_scale_us_val;                   /* clamped qdelay scale for gain decay (computed at module init) */
static int kcc_jitter_r_thresh_us_val;                      /* clamped jitter threshold for adaptive R (computed at module init) */
static int kcc_jitter_r_scale_val;                          /* clamped jitter scale for adaptive R (computed at module init) */
static int kcc_kalman_r_max_boost_val;                  /* clamped R max boost (computed at module init) */
static int kcc_probe_rtt_long_rtt_us_val;                   /* clamped long-RTT threshold (computed at module init) */
static u32 kcc_pacing_margin_div_val;                       /* computed pacing divisor [1, 100] (computed at module init) */

static u32 kcc_kalman_p_est_init_val;                       /* clamped initial p_est (computed at module init) */
static u32 kcc_kalman_p_est_floor_val;                      /* clamped p_est floor (computed at module init) */
static u32 kcc_kalman_outlier_ms_val;                       /* clamped outlier base (ms) (computed at module init) */
static u32 kcc_kalman_scale_shift_val;                    /* ilog2(kalman_scale) for shift optimization (computed at module init) */
static u64 kcc_kalman_outlier_thresh_scaled_val;     /* precomputed scaled outlier base threshold (computed at module init) */
static u64 kcc_kalman_shift_cap_val;                 /* precomputed U64_MAX >> scale_shift (computed at module init) */
static int kcc_kalman_noise_alpha_num_val;               /* snapped noise alpha numerator (computed at module init) */
static int kcc_kalman_noise_alpha_den_val;               /* snapped noise alpha denominator (computed at module init) */
static int kcc_kalman_noise_beta_num_val;                /* snapped noise beta numerator (computed at module init) */
static int kcc_kalman_noise_beta_den_val;                /* snapped noise beta denominator (computed at module init) */
static int kcc_kalman_q_est_max_val;                     /* clamped Q estimate max (computed at module init) */
static int kcc_kalman_r_est_max_val;                     /* clamped R estimate max (computed at module init) */
static int kcc_kalman_q_est_floor_val;                   /* clamped Q estimate floor (computed at module init) */
static int kcc_kalman_r_est_floor_val;                   /* clamped R estimate floor (computed at module init) */
/* Cached clamped Q/R for hot-path use (avoid raw-param read race during sysctl) */
static int kcc_kalman_q_val;                             /* clamped Kalman Q (computed at module init) */
static int kcc_kalman_r_val;                             /* clamped Kalman R (computed at module init) */
static int kcc_kalman_noise_mode_val;                    /* clamped noise combination mode (computed at module init) */
static int kcc_alone_confirm_rounds_val;               /* clamped alone confirmation rounds (computed at module init) */
static int kcc_alone_qdelay_thresh_us_val;              /* clamped alone qdelay threshold (computed at module init) */
static int kcc_alone_jitter_thresh_us_val;              /* clamped alone jitter threshold (computed at module init) */
static int kcc_alone_agg_state_level_val;               /* clamped alone agg state level (computed at module init) */
static int kcc_alone_bypass_ecn_val;                    /* clamped alone bypass ECN flag (computed at module init) */

static int kcc_ecn_enable_val;                           /* clamped ECN enable flag (computed at module init) */
static u32 kcc_ecn_backoff_val;                          /* derived ECN backoff ratio in BBR_UNIT (computed at module init) */
static int kcc_ecn_qdelay_thresh_us_val;                 /* clamped ECN qdelay threshold (computed at module init) */
static int kcc_ecn_ewma_retained_val;                    /* cached ECN EWMA retained weight (computed at module init) */
static int kcc_ecn_ewma_total_val;                       /* cached ECN EWMA total weight (computed at module init) */
static int kcc_kalman_max_consec_reject_val;             /* clamped consec reject limit (computed at module init) */
static int kcc_ecn_idle_decay_num_val;                   /* snapped per-ACK ECN idle decay num (computed at module init) */
static int kcc_ecn_idle_decay_den_val;                   /* snapped per-ACK ECN idle decay den (computed at module init) */
static int kcc_ewma_qdelay_num_val;                         /* cached EWMA qdelay numerator (computed at module init) */
static int kcc_ewma_qdelay_den_val;                         /* cached EWMA qdelay denominator (computed at module init) */
static int kcc_ewma_jitter_num_val;                         /* cached EWMA jitter numerator (computed at module init) */
static int kcc_ewma_jitter_den_val;                         /* cached EWMA jitter denominator (computed at module init) */
static int kcc_probe_bw_cycle_rand_val;                     /* clamped PROBE_BW rand range (computed at module init) */
static int kcc_probe_bw_up_limit_val;                      /* clamped PROBE_BW up-phase limit flag (computed at module init) */
static int kcc_lt_intvl_min_rtts_val;                       /* clamped LT interval min RTTs (computed at module init) */
static int kcc_lt_intvl_max_mult_val;                      /* clamped LT interval max multiplier (computed at module init) */
static u32 kcc_lt_loss_thresh_val;                          /* clamped LT loss threshold (computed at module init) */
static u32 kcc_lt_bw_ratio_val;                             /* derived LT BW ratio in BBR_UNIT (computed at module init) */
static u32 kcc_lt_bw_diff_val;                              /* clamped LT BW absolute diff (computed at module init) */
static int kcc_lt_bw_max_rtts_val;                          /* clamped LT BW max RTTs (< 4095) (computed at module init) */
static int kcc_lt_bw_inst_qdelay_thresh_us_val;              /* clamped LT BW instant qdelay thresh (computed at module init) */

static u32 kcc_extra_acked_max_ms_val;                       /* derived ACK max aggregation ms (computed at module init) */
static u32 kcc_probe_rtt_mode_ms_val;                        /* derived PROBE_RTT stay-duration ms (computed at module init) */
static u32 kcc_minrtt_fast_fall_div_val;                    /* clamped minrtt fast-fall divisor (computed at module init) */
static u32 kcc_kalman_probe_band_mult_val;                  /* clamped probe interval band multiplier (computed at module init) */
static u32 kcc_kalman_q_rtt_div_val;                        /* clamped Q RTT division factor (computed at module init) */
static u32 kcc_kalman_rtt_dyn_mult_val;                     /* clamped rtt dynamic multiplier (computed at module init) */
static u32 kcc_tso_headroom_mult_val;                       /* derived TSO headroom multiplier (computed at module init) */
static u32 kcc_min_tso_rate_div_val;                        /* derived min TSO rate divisor (computed at module init) */
static int kcc_probe_rtt_long_interval_div_val;             /* snapped long interval divisor (computed at module init) */
static int kcc_agg_factor_weight_val;                       /* snapped factor weight [0,100] (computed at module init) */
static int kcc_agg_confidence_max_val;                      /* clamped confidence max (computed at module init) */
static int kcc_agg_thresh_suspected_val;                    /* snapped suspected threshold (computed at module init) */
static int kcc_agg_thresh_confirmed_val;                    /* snapped confirmed threshold (computed at module init) */
static int kcc_agg_thresh_trusted_val;                      /* snapped trusted threshold (computed at module init) */
static u32 kcc_kalman_outlier_jitter_mult_val;              /* clamped outlier jitter multiplier (computed at module init) */
static u32 kcc_kalman_q_min_factor_val;                     /* clamped Q-min factor (computed at module init) */
static u32 kcc_kalman_p_est_init_rtt_div_val;               /* clamped p_est init RTT divisor (computed at module init) */
static u32 kcc_bw_rt_cycle_len_val;                         /* clamped BW cycle length (computed at module init) */
static u32 kcc_cwnd_min_target_val;                          /* clamped cwnd min target (computed at module init) */
static u32 kcc_sndbuf_expand_factor_val;                     /* clamped sndbuf expand factor (computed at module init) */
static u32 kcc_ack_epoch_max_val;                            /* clamped ACK epoch max (computed at module init) */
static u32 kcc_kf_enable_val;                            /* cached global KF enable flag [0,1] (computed at module init) */
static u32 kcc_kf_startup_r_pct_val;                     /* cached startup-phase measurement noise R pct (computed at module init) */
static u32 kcc_kf_steady_r_pct_val;                      /* cached steady-state measurement noise R pct (computed at module init) */
static u32 kcc_kf_q_shift_val;                           /* cached process noise shift (Q = 1 << shift) (computed at module init) */
static u64 kcc_kf_chi2_num_val;                          /* cached chi2 innovation gate numerator (computed at module init) */
static u64 kcc_kf_chi2_den_val;                          /* cached chi2 innovation gate denominator (computed at module init) */
static u32 kcc_kf_discount_num_val;                      /* cached fair-share discount numerator (computed at module init) */
static u32 kcc_kf_discount_den_val;                      /* cached fair-share discount denominator (computed at module init) */

/*
 * kcc_init_module_params - Validate all raw module parameters against
 * their legal ranges, clamp out-of-bounds values, and compute all
 * derived fixed-point quantities.
 *
 * Called at module load and whenever any scalar parameter is written
 * via sysctl.  Also called by kcc_rebuild_gain_table's caller path.
 *
 * No concurrent-write protection needed — see the
 * "CONCURRENCY & SAFETY MODEL" comment at struct kcc_ext for details.
 */
static void kcc_init_module_params(void)                          /* clamp all params + compute derived values */
{
    int i;                                                                  /* loop index for decay mask init */
    /*
     * Clamp all raw module-parameter integers to their legal ranges.
     * For denominator-style params, lo=1 prevents division by zero.
     * For numerator-style params, lo=0 allows disabling the feature.
     */
    kcc_probe_rtt_base_sec = clamp(kcc_probe_rtt_base_sec, 1, KCC_PROBE_RTT_MAX_SEC);   /* PROBE_RTT base interval [1s, 24h] */
    kcc_probe_rtt_max_sec = clamp(kcc_probe_rtt_max_sec, 1, KCC_PROBE_RTT_MAX_SEC);      /* PROBE_RTT max interval [1s, 24h] */
    kcc_probe_rtt_dyn_max_sec = clamp(kcc_probe_rtt_dyn_max_sec, 0, KCC_PROBE_RTT_MAX_SEC); /* PROBE_RTT dyn max [0, 24h] */

    kcc_cwnd_gain_num = clamp(kcc_cwnd_gain_num, 0, 100000);            /* cwnd gain numerator [0, 100k] */
    kcc_cwnd_gain_den = clamp(kcc_cwnd_gain_den, 1, 100000);            /* cwnd gain denominator [1, 100k] */
    kcc_extra_acked_gain_num = clamp(kcc_extra_acked_gain_num, 0, 100000); /* ACK-agg gain num [0, 100k] */
    kcc_extra_acked_gain_den = clamp(kcc_extra_acked_gain_den, 1, 100000); /* ACK-agg gain den [1, 100k] */

    kcc_kalman_q = clamp(kcc_kalman_q, 0, 100000);        /* process noise Q [0, 100k] (Kalman 1960) */
    kcc_kalman_r = clamp(kcc_kalman_r, 0, 100000);        /* measurement noise R [0, 100k] (Kalman 1960) */

    kcc_high_gain_num = clamp(kcc_high_gain_num, 0, 100000);            /* STARTUP gain numerator [0, 100k] */
    kcc_high_gain_den = clamp(kcc_high_gain_den, 1, 100000);            /* STARTUP gain denominator [1, 100k] */
    kcc_drain_gain_num = clamp(kcc_drain_gain_num, 0, 100000);          /* DRAIN gain numerator [0, 100k] */
    kcc_drain_gain_den = clamp(kcc_drain_gain_den, 1, 100000);          /* DRAIN gain denominator [1, 100k] */

    /* PROBE_BW cycle length: [2, 256], must be power-of-two
     * so that cycle_idx & (len-1) wraps correctly. */
    kcc_probe_bw_cycle_len = clamp(kcc_probe_bw_cycle_len, 2, 256);     /* cycle length [2, 256] */
    kcc_probe_bw_cycle_len = roundup_pow_of_two(kcc_probe_bw_cycle_len); /* round up to power of two */

    kcc_full_bw_thresh_num = clamp(kcc_full_bw_thresh_num, 0, 100000); /* full-BW threshold num [0, 100k] */
    kcc_full_bw_thresh_den = clamp(kcc_full_bw_thresh_den, 1, 100000); /* full-BW threshold den [1, 100k] */
    kcc_full_bw_cnt = clamp(kcc_full_bw_cnt, 1, 3);                    /* full-BW rounds [1, 3], 2-bit field */

    kcc_pacing_margin_num = clamp(kcc_pacing_margin_num, 0, 50);        /* pacing margin num [0, 50], prevent >=100% */
    kcc_pacing_margin_den = clamp(kcc_pacing_margin_den, 1, 100000);    /* pacing margin den [1, 100k] */

    kcc_inflight_low_gain_num = clamp(kcc_inflight_low_gain_num, 0, 100000);   /* inflight lo num [0, 100k] */
    kcc_inflight_low_gain_den = clamp(kcc_inflight_low_gain_den, 1, 100000);   /* inflight lo den [1, 100k] */
    kcc_inflight_high_gain_num = clamp(kcc_inflight_high_gain_num, 0, 100000); /* inflight hi num [0, 100k] */
    kcc_inflight_high_gain_den = clamp(kcc_inflight_high_gain_den, 1, 100000); /* inflight hi den [1, 100k] */

    kcc_kalman_p_est_max = clamp(kcc_kalman_p_est_max, 1, 100000000);  /* p_est max [1, 100M] */
    kcc_kalman_converged_p_est = clamp(kcc_kalman_converged_p_est, 1, 1000000); /* convergence p_est [1, 1M] */
    kcc_recal_p_est_thresh = clamp(kcc_recal_p_est_thresh, 1, 100000000); /* recal p_est thresh [1, 100M] */
    kcc_drain_skip_qdelay_us = clamp(kcc_drain_skip_qdelay_us, 0, 100000);  /* drain skip qdelay [0, 100k us] */
    kcc_kalman_q_boost_mult = clamp(kcc_kalman_q_boost_mult, 1, 10000); /* Q-boost mult [1, 10k] */
    kcc_kalman_q_max = clamp(kcc_kalman_q_max, 1, 100000);              /* Q max [1, 100k] */
    kcc_kalman_q_scale_cap = clamp(kcc_kalman_q_scale_cap, 1, 10000);   /* Q scale cap [1, 10k] */
    kcc_kalman_min_samples = clamp(kcc_kalman_min_samples, 3, 20);      /* min Kalman samples [3, 20] */

    kcc_rtt_sample_max_us = clamp(kcc_rtt_sample_max_us, 1, 10000000);  /* max RTT sample [1, 10M us] */
    kcc_minrtt_fast_fall_cnt = clamp(kcc_minrtt_fast_fall_cnt, 0, 3);   /* fast-fall count [0, 3], 2-bit field */
    kcc_minrtt_sticky_num = clamp(kcc_minrtt_sticky_num, 0, 1000);      /* sticky ratio num [0, 1000] */
    kcc_minrtt_sticky_den = clamp(kcc_minrtt_sticky_den, 1, 100000);    /* sticky ratio den [1, 100k] */
    kcc_minrtt_sticky_num = min_t(int, kcc_minrtt_sticky_num, kcc_minrtt_sticky_den);
    kcc_minrtt_srtt_guard_num = clamp(kcc_minrtt_srtt_guard_num, 0, 1000); /* SRTT guard num [0, 1000] */
    kcc_minrtt_srtt_guard_den = clamp(kcc_minrtt_srtt_guard_den, 1, 100000); /* SRTT guard den [1, 100k] */
    kcc_minrtt_srtt_guard_num = min_t(int, kcc_minrtt_srtt_guard_num, kcc_minrtt_srtt_guard_den);

    kcc_bdp_min_rtt_us = clamp(kcc_bdp_min_rtt_us, 0, 100000);          /* BDP min-RTT floor [0, 100k us] */
    kcc_probe_cwnd_bonus = clamp(kcc_probe_cwnd_bonus, 0, 100);          /* probe cwnd bonus [0, 100] */
    kcc_edt_near_now_ns = clamp(kcc_edt_near_now_ns, 0, 10000000);       /* EDT near-now [0, 10M ns] */
    kcc_min_tso_rate = clamp(kcc_min_tso_rate, 1, 1000000000);           /* min TSO rate [1, 1B bytes/s] */
    kcc_tso_max_segs = clamp(kcc_tso_max_segs, 1, 65535);                /* max TSO segs [1, 65535] */
    kcc_tso_segs_low = clamp(kcc_tso_segs_low, 1, 65535);
    kcc_tso_segs_default = clamp(kcc_tso_segs_default, 1, 65535);
    kcc_jitter_probe_thresh_us = clamp(kcc_jitter_probe_thresh_us, 0, 100000);   /* jitter probe thresh [0, 100k] */
    kcc_jitter_probe_scale_us = clamp(kcc_jitter_probe_scale_us, 1, 100000);     /* jitter probe scale [1, 100k] */
    kcc_qdelay_probe_thresh_us = clamp(kcc_qdelay_probe_thresh_us, 0, 100000);   /* qdelay probe thresh [0, 100k] */
    kcc_qdelay_probe_scale_us = clamp(kcc_qdelay_probe_scale_us, 1, 100000);     /* qdelay probe scale [1, 100k] */
    kcc_jitter_r_thresh_us = clamp(kcc_jitter_r_thresh_us, 0, 100000);   /* adaptive R jitter thresh [0, 100k] */
    kcc_jitter_r_scale = clamp(kcc_jitter_r_scale, 1, 100000);           /* adaptive R scale [1, 100k] */
    kcc_kalman_r_max_boost = clamp(kcc_kalman_r_max_boost, 1, 1000);    /* R max boost [1, 1000] */
    kcc_probe_rtt_long_rtt_us = clamp(kcc_probe_rtt_long_rtt_us, 0, 10000000); /* long-RTT thresh [0, 10M] */

    /* Kalman scale must be power-of-two for shift-based division (Kalman 1960) */
    kcc_kalman_p_est_init = clamp(kcc_kalman_p_est_init, 1, 10000000);   /* p_est init [1, 10M] */
    kcc_kalman_p_est_floor = clamp(kcc_kalman_p_est_floor, 1, 100000);   /* p_est floor [1, 100k] */
    kcc_kalman_p_est_floor = min(kcc_kalman_p_est_floor, kcc_kalman_p_est_max); /* floor <= max invariant */
    kcc_kalman_p_est_init = min(kcc_kalman_p_est_init, kcc_kalman_p_est_max);   /* init <= max invariant */
    kcc_kalman_outlier_ms = clamp(kcc_kalman_outlier_ms, 0, 10000);      /* outlier ms [0, 10k] */
    kcc_kalman_q_boost_ms = clamp(kcc_kalman_q_boost_ms, 1, 5000);       /* Q-boost ms [1, 5000] */
    kcc_kalman_qboost_cdwn = clamp(kcc_kalman_qboost_cdwn, 1, 255);      /* Q-boost cooldown [1, 255] */
    kcc_kalman_scale = clamp(kcc_kalman_scale, 64, 1048576);              /* kalman scale [64, 1M] */
    kcc_kalman_scale = roundup_pow_of_two(kcc_kalman_scale);              /* round up to power of two */

    kcc_kalman_outlier_jitter_mult_num = clamp(kcc_kalman_outlier_jitter_mult_num, 0, 1000);     /* jitter mult num [0, 1000] */
    kcc_kalman_outlier_jitter_mult_den = clamp(kcc_kalman_outlier_jitter_mult_den, 1, 100000);   /* jitter mult den [1, 100k] */
    kcc_kalman_q_min_factor_num = clamp(kcc_kalman_q_min_factor_num, 0, 1000);                    /* Q min factor num [0, 1000] */
    kcc_kalman_q_min_factor_den = clamp(kcc_kalman_q_min_factor_den, 1, 100000);                  /* Q min factor den [1, 100k] */
    kcc_kalman_p_est_init_rtt_div_num = clamp(kcc_kalman_p_est_init_rtt_div_num, 1, 100000);      /* p_est init RTT div num [1, 100k] */
    kcc_kalman_p_est_init_rtt_div_den = clamp(kcc_kalman_p_est_init_rtt_div_den, 1, 100000);      /* p_est init RTT div den [1, 100k] */

    kcc_ewma_qdelay_num = clamp(kcc_ewma_qdelay_num, 0, 100);              /* EWMA qdelay num [0, 100] */
    kcc_ewma_qdelay_den = clamp(kcc_ewma_qdelay_den, 1, 100000);           /* EWMA qdelay den [1, 100k] */
    kcc_ewma_qdelay_num = min_t(int, kcc_ewma_qdelay_num, kcc_ewma_qdelay_den);
    kcc_ewma_jitter_num = clamp(kcc_ewma_jitter_num, 0, 100);              /* EWMA jitter num [0, 100] */
    kcc_ewma_jitter_den = clamp(kcc_ewma_jitter_den, 1, 100000);           /* EWMA jitter den [1, 100k] */
    kcc_ewma_jitter_num = min_t(int, kcc_ewma_jitter_num, kcc_ewma_jitter_den);

    /* BBR-S covariance-matched noise estimation params */
    kcc_kalman_noise_alpha_num = clamp(kcc_kalman_noise_alpha_num, 0, 100);    /* alpha num [0, 100] */
    kcc_kalman_noise_alpha_den = clamp(kcc_kalman_noise_alpha_den, 1, 100000); /* alpha den [1, 100k] */
    kcc_kalman_noise_alpha_num = min_t(int, kcc_kalman_noise_alpha_num, kcc_kalman_noise_alpha_den); /* alpha_num <= alpha_den */
    kcc_kalman_noise_beta_num = clamp(kcc_kalman_noise_beta_num, 0, 100);     /* beta num [0, 100] */
    kcc_kalman_noise_beta_den = clamp(kcc_kalman_noise_beta_den, 1, 100000);  /* beta den [1, 100k] */
    kcc_kalman_noise_beta_num = min_t(int, kcc_kalman_noise_beta_num, kcc_kalman_noise_beta_den);   /* beta_num <= beta_den */
    kcc_kalman_q_est_max = clamp(kcc_kalman_q_est_max, 1, 2000000000);          /* Q est max [1, 2e9] */
    kcc_kalman_r_est_max = clamp(kcc_kalman_r_est_max, 1, 2000000000);          /* R est max [1, 2e9] */
    kcc_kalman_q_est_floor = clamp(kcc_kalman_q_est_floor, 1, 100000);        /* Q est floor [1, 100k] */
    kcc_kalman_r_est_floor = clamp(kcc_kalman_r_est_floor, 1, 100000);        /* R est floor [1, 100k] */
    kcc_kalman_noise_mode = clamp(kcc_kalman_noise_mode, 0, 2);               /* mode [0=off, 1=max, 2=avg] */
    kcc_kalman_noise_avg_num = clamp(kcc_kalman_noise_avg_num, 0, 100);
    kcc_kalman_noise_avg_den = clamp(kcc_kalman_noise_avg_den, 1, 100000);
    kcc_kalman_noise_avg_num = min_t(int, kcc_kalman_noise_avg_num, kcc_kalman_noise_avg_den);

    /* Single-flow detection hysteresis */
    kcc_alone_confirm_rounds = clamp(kcc_alone_confirm_rounds, 1, 32);       /* alone confirm [1, 32] */
    kcc_alone_qdelay_thresh_us = clamp(kcc_alone_qdelay_thresh_us, 0, 100000); /* alone qdelay [0, 100k] us */
    kcc_alone_jitter_thresh_us = clamp(kcc_alone_jitter_thresh_us, 0, 100000); /* alone jitter [0, 100k] us */
    kcc_alone_agg_state_level = clamp(kcc_alone_agg_state_level, 0, 2);         /* alone agg level [0, 2] */
    kcc_alone_bypass_ecn = clamp(kcc_alone_bypass_ecn, 0, 1);                   /* alone bypass ECN [0, 1] */

    /* ECN params */
    kcc_ecn_enable = clamp(kcc_ecn_enable, 0, 1);                                /* ECN enable [0, 1] */
    kcc_ecn_backoff_num = clamp(kcc_ecn_backoff_num, 0, 100);                    /* ECN backoff num [0, 100] */
    kcc_ecn_backoff_den = clamp(kcc_ecn_backoff_den, 1, 100000);                 /* ECN backoff den [1, 100k] */
    kcc_ecn_qdelay_thresh_us = clamp(kcc_ecn_qdelay_thresh_us, 0, 100000);       /* ECN qdelay thresh [0, 100k] */
    kcc_ecn_ewma_retained = clamp(kcc_ecn_ewma_retained, 0, 100);                /* ECN EWMA retained [0, 100] */
    kcc_ecn_ewma_total = clamp(kcc_ecn_ewma_total, 1, 100000);                   /* ECN EWMA total [1, 100k] */
    /* EWMA formula requires retained <= total, otherwise new-sample weight > 1 */
    kcc_ecn_ewma_retained = min_t(int, kcc_ecn_ewma_retained, kcc_ecn_ewma_total); /* enforce retained <= total */
    /* ECN idle decay: must stay in [1, den] region, and num < den to guarantee actual decay */
    kcc_ecn_idle_decay_den = clamp(kcc_ecn_idle_decay_den, 2, 100000);              /* den [2, 100k], prevents div-by-zero */
    kcc_ecn_idle_decay_num = clamp(kcc_ecn_idle_decay_num, 1, kcc_ecn_idle_decay_den - 1); /* num [1, den-1], decay factor < 1.0 */

    /* Kalman outlier rejection limit */
    kcc_kalman_max_consec_reject = clamp(kcc_kalman_max_consec_reject, 1, 1000);   /* consec reject [1, 1000] */

    kcc_probe_bw_cycle_rand = clamp(kcc_probe_bw_cycle_rand, 1, kcc_probe_bw_cycle_len);   /* rand range [1, cycle_len] */
    kcc_probe_bw_up_limit = clamp(kcc_probe_bw_up_limit, 0, 1);                            /* PROBE_BW up-phase limit [0, 1] */

    /* LT BW: max RTTs must fit in 12-bit counter (< 4095) */
    kcc_lt_intvl_min_rtts = clamp(kcc_lt_intvl_min_rtts, 1, 127);          /* LT min RTTs [1, 127] */
    kcc_lt_loss_thresh = clamp(kcc_lt_loss_thresh, 1, 65535);              /* LT loss thresh [1, 65535] */
    kcc_lt_bw_ratio_num = clamp(kcc_lt_bw_ratio_num, 0, 100000);           /* LT BW ratio num [0, 100k] */
    kcc_lt_bw_ratio_den = clamp(kcc_lt_bw_ratio_den, 1, 100000);           /* LT BW ratio den [1, 100k] */
    kcc_lt_bw_diff = clamp(kcc_lt_bw_diff, 0, 100000);                     /* LT BW diff [0, 100k] */
    kcc_lt_bw_ema_num = clamp(kcc_lt_bw_ema_num, 0, 100);
    kcc_lt_bw_ema_den = clamp(kcc_lt_bw_ema_den, 1, 100000);
    kcc_lt_bw_ema_num = min_t(int, kcc_lt_bw_ema_num, kcc_lt_bw_ema_den);
    kcc_lt_bw_max_rtts = clamp(kcc_lt_bw_max_rtts, 1, 4094);               /* LT BW max RTTs [1, 4094], 12-bit field max = 4095 */
    kcc_lt_bw_inst_qdelay_thresh_us = clamp(kcc_lt_bw_inst_qdelay_thresh_us, 0, 100000); /* LT BW inst qdelay thresh [0, 100k us] */
    kcc_lt_intvl_max_mult = clamp(kcc_lt_intvl_max_mult, 1, 32);            /* LT timeout mult [1, 32] */

    kcc_extra_acked_max_ms_num = clamp(kcc_extra_acked_max_ms_num, 0, 100000);          /* ACK-agg max ms num [0, 100k] */
    kcc_extra_acked_max_ms_den = clamp(kcc_extra_acked_max_ms_den, 1, 100000);          /* ACK-agg max ms den [1, 100k] */

    kcc_probe_rtt_mode_ms_num = clamp(kcc_probe_rtt_mode_ms_num, 1, 100000);            /* PROBE_RTT mode ms num [1, 100k] */
    kcc_probe_rtt_mode_ms_den = clamp(kcc_probe_rtt_mode_ms_den, 1, 100000);            /* PROBE_RTT mode ms den [1, 100k] */

    kcc_bw_rt_cycle_len = clamp(kcc_bw_rt_cycle_len, 2, 256);                            /* BW sliding window [2, 256] */
    kcc_cwnd_min_target = clamp(kcc_cwnd_min_target, 1, 1000);                            /* min cwnd target [1, 1000] */

    /* ACK agg confidence-based compensation params */
    kcc_agg_enable = clamp(kcc_agg_enable, 0, 1);                          /* agg enable [0, 1] */
    kcc_agg_confidence_thresh = clamp(kcc_agg_confidence_thresh, 0, 10000); /* confidence thresh [0, 10k] */
    kcc_agg_max_comp_ratio = clamp(kcc_agg_max_comp_ratio, 0, 100);        /* max comp ratio [0, 100] */
    kcc_agg_max_comp_duration = clamp(kcc_agg_max_comp_duration, 1, 128);  /* max comp duration [1, 128] */
    kcc_agg_r_hysteresis = clamp(kcc_agg_r_hysteresis, 0, 100);            /* R hysteresis [0, 100] */
    kcc_agg_r_multiplier_min = clamp(kcc_agg_r_multiplier_min, 1, 10000);  /* R mult min [1, 10000] */
    kcc_agg_r_multiplier_max = clamp(kcc_agg_r_multiplier_max, 1, 10000);  /* R mult max [1, 10000] */
    kcc_agg_r_multiplier_max = max(kcc_agg_r_multiplier_max, kcc_agg_r_multiplier_min);  /* ensure max >= min */
    kcc_agg_factor3_qdelay_us = clamp(kcc_agg_factor3_qdelay_us, 0, 100000);     /* factor3 qdelay [0, 100k] */
    kcc_agg_factor4_ratio_num = clamp(kcc_agg_factor4_ratio_num, 1, 100000);    /* factor4 num [1, 100k] */
    kcc_agg_factor4_ratio_den = clamp(kcc_agg_factor4_ratio_den, 1, 100000);    /* factor4 den [1, 100k] */
    kcc_agg_safety_qdelay_us = clamp(kcc_agg_safety_qdelay_us, 0, 100000);      /* safety qdelay [0, 100k] */
    kcc_agg_safety_bdp_mult = clamp(kcc_agg_safety_bdp_mult, 1, 100);           /* safety bdp mult [1, 100] */
    kcc_agg_max_window_ms = clamp(kcc_agg_max_window_ms, 1, 10000);             /* max window ms [1, 10k] */
    kcc_agg_max_decay_pct = clamp(kcc_agg_max_decay_pct, 0, 100);               /* max decay pct [0, 100] */
    kcc_agg_max_per_ack_decay = clamp(kcc_agg_max_per_ack_decay, 0, 128);       /* per-ACK decay [0, 128]; 128=disabled */
    kcc_agg_max_per_ack_decay_den = clamp(kcc_agg_max_per_ack_decay_den, 1, 65535); /* per-ACK decay den [1, 65535] */
    kcc_agg_window_rotation_rtts = clamp(kcc_agg_window_rotation_rtts, 1, 65535);   /* window rotation RTTs [1, 65535] */
    kcc_extra_acked_win_rtts_max = clamp(kcc_extra_acked_win_rtts_max, 1, 65535);
    kcc_agg_window_rotation_rtts = min_t(int, kcc_agg_window_rotation_rtts, kcc_extra_acked_win_rtts_max);

    /* Bitmask values: all 32 bits per word are valid (256 phases across 8 words).
     * Cast through unsigned to preserve bit 31 (which would be negative as signed int). */
    for (i = 0; i < KCC_DECAY_MASK_WORDS; i++) {
        kcc_cycle_decay_mask[i] = (int)((u32)kcc_cycle_decay_mask[i]);
    }

    /*
     * Compute derived values and assign to the _val cache.
     * No concurrent-read protection needed — see "CONCURRENCY & SAFETY MODEL"
     * at struct kcc_ext for details.
     */

     /* PROBE_RTT intervals: sec * HZ -> jiffies, guarded against overflow */
    kcc_probe_rtt_base_jiffies = (u32)min_t(u64, (u64)kcc_probe_rtt_base_sec * HZ, U32_MAX);         /* sec * HZ capped at U32_MAX */
    kcc_probe_rtt_max_jiffies = (u32)min_t(u64, (u64)kcc_probe_rtt_max_sec * HZ, U32_MAX);          /* sec * HZ capped at U32_MAX */
    /* dyn_max must be > base_sec for valid interpolation range in kcc_update_dyn_probe_interval().
     * Enforce this constraint on the derived jiffies value without mutating the raw sysctl param. */
    {
        int dyn_sec = kcc_probe_rtt_dyn_max_sec;
        if (dyn_sec != 0 && dyn_sec <= kcc_probe_rtt_base_sec) {
            dyn_sec = (kcc_probe_rtt_base_sec < KCC_PROBE_RTT_MAX_SEC) ? (kcc_probe_rtt_base_sec + 1) : kcc_probe_rtt_base_sec;
        }
        kcc_probe_rtt_dyn_max_jiffies = (u32)min_t(u64, (u64)dyn_sec * HZ, U32_MAX);
    }

    /*
     * CWND gain: num/den * BBR_UNIT, clamped to fit 10-bit pacing_gain field.
     * ACK-agg gain is not clamped (read as multiplier, fits u32).
     */
    kcc_cwnd_gain_val = min_t(u32, (u32)((u64)BBR_UNIT * (u32)kcc_cwnd_gain_num / (u32)kcc_cwnd_gain_den), KCC_GAIN_MAX); /* clamp to 10 bits */
    kcc_extra_acked_gain_val = (u32)((u64)BBR_UNIT * (u32)kcc_extra_acked_gain_num / (u32)kcc_extra_acked_gain_den); /* num/den * BBR_UNIT */

    /*
     * STARTUP high_gain: ceiling division so that 2885/1000 maps to
     * ceil(2885 * 256 / 1000) = 739 (approx 2.887x BBR_UNIT)
     * (Cardwell et al. 2016).
     * Both high_gain and drain_gain are capped at 1023 to prevent bitfield
     * overflow in the 10-bit pacing_gain field.
     */
    kcc_high_gain_val = min_t(u32, (u32)((u64)BBR_UNIT * (u32)kcc_high_gain_num / (u32)kcc_high_gain_den) + 1, KCC_GAIN_MAX); /* floor+1: match BBR 256*2885/1000+1=739 */
    kcc_drain_gain_val = min_t(u32, (u32)((u64)BBR_UNIT * (u32)kcc_drain_gain_num / (u32)kcc_drain_gain_den), KCC_GAIN_MAX); /* floor: match BBR truncation, 347/1000=0.347→88 (BBR: 1000/2885=0.3467→88) */

    /* Cycle length and full-BW threshold */
    kcc_probe_bw_cycle_len_val = (u32)kcc_probe_bw_cycle_len;     /* publish cycle length */
    kcc_full_bw_thresh_val = (u32)((u64)BBR_UNIT * (u32)kcc_full_bw_thresh_num / (u32)kcc_full_bw_thresh_den); /* num/den * BBR_UNIT */
    kcc_full_bw_cnt_val = (u32)kcc_full_bw_cnt;                                  /* publish full-BW round count (already clamped to [1,3] at line 1638, fits 2-bit field) */

    /* Inflight bounds as BBR_UNIT multiples */
    kcc_inflight_low_gain_val = (u32)((u64)BBR_UNIT * (u32)kcc_inflight_low_gain_num / (u32)kcc_inflight_low_gain_den); /* num/den * BBR_UNIT */
    kcc_inflight_high_gain_val = (u32)((u64)BBR_UNIT * (u32)kcc_inflight_high_gain_num / (u32)kcc_inflight_high_gain_den); /* num/den * BBR_UNIT */

    /* Kalman clamped scalars (Kalman 1960) */
    kcc_kalman_p_est_max_val = (u32)kcc_kalman_p_est_max;         /* publish p_est max */
    kcc_kalman_converged_p_est_val = (u32)kcc_kalman_converged_p_est; /* publish converged p_est threshold */
    kcc_recal_p_est_thresh_val = (u32)kcc_recal_p_est_thresh;     /* publish recalibration p_est threshold */
    kcc_drain_skip_qdelay_us_val = (u32)kcc_drain_skip_qdelay_us;   /* publish drain skip qdelay */
    /* Cache clamped Q/R for hot-path use (avoids raw-param read race) */
    kcc_kalman_q_val = kcc_kalman_q;                                /* publish clamped Q */
    kcc_kalman_r_val = kcc_kalman_r;                                /* publish clamped R */

    /*
     * Q-boost threshold: when |innovation| exceeds this value, the filter
     * resets p_est to p_est_init, re-entering the high-gain phase to
     * rapidly track the changed path characteristic.
     * Formula: boost_mult * boost_ms * 1000 us/ms * kalman_scale.
     */
     /* Q-boost threshold: guard each multiply against u64 overflow.
      * At extreme parameters (mult=10000, ms=5000, scale=1048576) the product
      * exceeds U64_MAX; we check each step and saturate to U64_MAX safely. */
    {
        u64 qbt = (u64)kcc_kalman_q_boost_mult;
        if (qbt <= U64_MAX / (u64)kcc_kalman_q_boost_ms) {
            qbt *= (u64)kcc_kalman_q_boost_ms;
        }
        else {
            qbt = U64_MAX;
        }
        if (qbt <= U64_MAX / USEC_PER_MSEC) {
            qbt *= USEC_PER_MSEC;
        }
        else {
            qbt = U64_MAX;
        }
        if (qbt <= U64_MAX / (u64)kcc_kalman_scale) {
            qbt *= (u64)kcc_kalman_scale;
        }
        else {
            qbt = U64_MAX;
        }
        kcc_kalman_q_boost_thresh_val = (u32)min_t(u64, qbt, U32_MAX);
    }
    kcc_kalman_qboost_cdwn_val = (u32)kcc_kalman_qboost_cdwn;     /* publish Q-boost cooldown */
    kcc_kalman_q_max_val = kcc_kalman_q_max;                       /* publish Q max ceiling */
    kcc_kalman_q_scale_cap_val = kcc_kalman_q_scale_cap;           /* publish Q scale cap */
    kcc_kalman_min_samples_val = kcc_kalman_min_samples;           /* publish min Kalman samples */
    kcc_rtt_sample_max_us_val = (u32)kcc_rtt_sample_max_us;        /* publish max RTT sample */
    kcc_minrtt_fast_fall_cnt_val = kcc_minrtt_fast_fall_cnt;       /* publish fast-fall count */
    kcc_minrtt_fast_fall_div = clamp(kcc_minrtt_fast_fall_div, 1, 256);       /* prevent div-by-zero */
    kcc_minrtt_fast_fall_div_val = kcc_minrtt_fast_fall_div;       /* publish fast-fall divisor */
    kcc_kalman_probe_band_mult = clamp(kcc_kalman_probe_band_mult, 1, 32);    /* prevent u32 overflow */
    kcc_kalman_probe_band_mult_val = kcc_kalman_probe_band_mult;    /* publish probe band mult */
    kcc_kalman_q_rtt_div = clamp(kcc_kalman_q_rtt_div, 1, 1000000);           /* prevent div-by-zero */
    kcc_kalman_q_rtt_div_val = kcc_kalman_q_rtt_div;                /* publish Q RTT divisor */
    kcc_kalman_rtt_dyn_mult = clamp(kcc_kalman_rtt_dyn_mult, 1, 100);         /* prevent u32 overflow */
    kcc_kalman_rtt_dyn_mult_val = kcc_kalman_rtt_dyn_mult;          /* publish RTT dyn mult */

    /* Cache min-RTT sticky ratio and SRTT guard ratio as num/den pairs */
    {
        u32 snum = (u32)kcc_minrtt_sticky_num;                          /* read sticky num to local */
        u32 sden = (u32)kcc_minrtt_sticky_den;                          /* read sticky den to local */
        kcc_minrtt_sticky_num_val = snum;                     /* publish cached sticky num */
        kcc_minrtt_sticky_den_val = sden;                     /* publish cached sticky den */
        snum = (u32)kcc_minrtt_srtt_guard_num;                       /* read SRTT guard num to local */
        sden = (u32)kcc_minrtt_srtt_guard_den;                       /* read SRTT guard den to local */
        kcc_minrtt_srtt_guard_num_val = snum;                  /* publish cached SRTT guard num */
        kcc_minrtt_srtt_guard_den_val = sden;                  /* publish cached SRTT guard den */
    }
    kcc_bdp_min_rtt_us_val = (u32)kcc_bdp_min_rtt_us;             /* publish BDP min-RTT floor */
    kcc_probe_cwnd_bonus_val = kcc_probe_cwnd_bonus;              /* publish probe cwnd bonus */
    kcc_edt_near_now_ns_val = kcc_edt_near_now_ns;                /* publish EDT near-now threshold */
    kcc_min_tso_rate_val = kcc_min_tso_rate;                       /* publish min TSO rate */
    kcc_tso_max_segs_val = kcc_tso_max_segs;                       /* publish max TSO segs */
    kcc_tso_segs_low_val = kcc_tso_segs_low;
    kcc_tso_segs_default_val = kcc_tso_segs_default;
    kcc_tso_headroom_mult = clamp(kcc_tso_headroom_mult, 0, 1000);
    kcc_tso_headroom_mult_val = kcc_tso_headroom_mult;             /* publish TSO headroom mult */
    kcc_min_tso_rate_div = clamp(kcc_min_tso_rate_div, 1, 256);               /* prevent div-by-zero */
    kcc_min_tso_rate_div_val = kcc_min_tso_rate_div;               /* publish min TSO rate divisor */
    kcc_jitter_probe_thresh_us_val = kcc_jitter_probe_thresh_us;  /* publish jitter probe thresh */
    kcc_jitter_probe_scale_us_val = kcc_jitter_probe_scale_us;    /* publish jitter probe scale */
    kcc_qdelay_probe_thresh_us_val = kcc_qdelay_probe_thresh_us;  /* publish qdelay probe thresh */
    kcc_qdelay_probe_scale_us_val = kcc_qdelay_probe_scale_us;    /* publish qdelay probe scale */
    kcc_jitter_r_thresh_us_val = kcc_jitter_r_thresh_us;          /* publish adaptive R jitter thresh */
    kcc_jitter_r_scale_val = kcc_jitter_r_scale;                   /* publish adaptive R jitter scale */
    kcc_kalman_r_max_boost_val = kcc_kalman_r_max_boost;          /* publish R max boost */
    kcc_probe_rtt_long_rtt_us_val = kcc_probe_rtt_long_rtt_us;     /* publish long-RTT threshold */
    kcc_probe_rtt_long_interval_div = clamp(kcc_probe_rtt_long_interval_div, 1, 1000); /* prevent div-by-zero */
    kcc_probe_rtt_long_interval_div_val = kcc_probe_rtt_long_interval_div; /* publish long-RTT interval div */

    /* ACK agg confidence compensation: publish clamped values */
    kcc_agg_enable_val = kcc_agg_enable;                        /* publish agg enable */
    kcc_agg_confidence_thresh_val = kcc_agg_confidence_thresh;  /* publish confidence thresh */
    kcc_agg_max_comp_ratio_val = kcc_agg_max_comp_ratio;        /* publish max comp ratio */
    kcc_agg_max_comp_duration_val = kcc_agg_max_comp_duration;  /* publish max comp duration */
    kcc_agg_r_hysteresis_val = kcc_agg_r_hysteresis;            /* publish R hysteresis */
    kcc_agg_r_multiplier_min_val = (u32)kcc_agg_r_multiplier_min; /* publish R mult min */
    kcc_agg_r_multiplier_max_val = (u32)kcc_agg_r_multiplier_max; /* publish R mult max */
    kcc_agg_factor3_qdelay_us_val = kcc_agg_factor3_qdelay_us;       /* publish factor3 qdelay */
    kcc_agg_factor4_ratio_num_val = kcc_agg_factor4_ratio_num;       /* publish factor4 ratio num */
    kcc_agg_factor4_ratio_den_val = kcc_agg_factor4_ratio_den;       /* publish factor4 ratio den */
    kcc_agg_safety_qdelay_us_val = kcc_agg_safety_qdelay_us;         /* publish safety qdelay */
    kcc_agg_safety_bdp_mult_val = kcc_agg_safety_bdp_mult;           /* publish safety bdp mult */
    kcc_agg_max_window_ms_val = kcc_agg_max_window_ms;               /* publish max window ms */
    kcc_agg_max_decay_pct_val = kcc_agg_max_decay_pct;               /* publish max decay pct */
    kcc_agg_max_per_ack_decay_val = kcc_agg_max_per_ack_decay;       /* publish per-ACK decay */
    kcc_agg_max_per_ack_decay_den_val = kcc_agg_max_per_ack_decay_den; /* publish per-ACK decay denom */
    kcc_agg_window_rotation_rtts_val = kcc_agg_window_rotation_rtts; /* publish window rotation RTTs */
    kcc_extra_acked_win_rtts_max_val = kcc_extra_acked_win_rtts_max;
    kcc_agg_per_ack_decay_active = ((u32)kcc_agg_max_per_ack_decay_val < (u32)kcc_agg_max_per_ack_decay_den_val); /* precompute per-ACK decay gate */
    kcc_agg_factor_weight = clamp(kcc_agg_factor_weight, 1, KCC_AGG_CONFIDENCE_MAX);            /* clamp per-factor weight */
    /* kcc_agg_factor_weight_val intentionally stays 0: confidence-based Kalman R
     * scaling and ACK-agg compensation are experimental.  Enable by publishing
     * the raw param value here once validated. */
    kcc_agg_thresh_suspected = clamp(kcc_agg_thresh_suspected, 1, KCC_AGG_CONFIDENCE_MAX);      /* clamp SUSPECTED threshold */
    kcc_agg_thresh_confirmed = clamp(kcc_agg_thresh_confirmed, 2, KCC_AGG_CONFIDENCE_MAX);      /* clamp CONFIRMED threshold */
    kcc_agg_thresh_trusted = clamp(kcc_agg_thresh_trusted, 3, KCC_AGG_CONFIDENCE_MAX);          /* clamp TRUSTED threshold */
    kcc_agg_confidence_max = clamp(kcc_agg_confidence_max, 1, 65536);                           /* clamp confidence max (prevent div-zero) */
    kcc_agg_confidence_max_val = kcc_agg_confidence_max;                                /* publish confidence max */
    /* enforce strict ordering: suspected < confirmed < trusted, clamp within [1..1024] */
    kcc_agg_thresh_confirmed = min_t(u32, max(kcc_agg_thresh_confirmed, kcc_agg_thresh_suspected + 1), 1024);
    kcc_agg_thresh_trusted = min_t(u32, max(kcc_agg_thresh_trusted, kcc_agg_thresh_confirmed + 1), 1024);
    kcc_agg_thresh_suspected_val = kcc_agg_thresh_suspected;        /* publish SUSPECTED threshold */
    kcc_agg_thresh_confirmed_val = kcc_agg_thresh_confirmed;        /* publish CONFIRMED threshold */
    kcc_agg_thresh_trusted_val = kcc_agg_thresh_trusted;          /* publish TRUSTED threshold */

    /*
     * Pacing margin: divisor = 100 - (num * 100 / den).
     * With num=1, den=100 -> divisor=99: rate = raw_rate * 99 / 100.
     * Clamped [1, 100] to prevent overflow in rate_bytes_per_sec.
     */
    {
        u32 num = (u32)kcc_pacing_margin_num;                               /* read pacing margin num */
        u32 den = (u32)kcc_pacing_margin_den;                               /* read pacing margin den */
        s32 margin;
        num = min_t(u32, num, den);
        margin = 100 - (s32)((u64)num * 100 / (u64)max_t(u32, den, 1));                        /* compute margin percentage */
        kcc_pacing_margin_div_val = (u32)clamp(margin, 1, 100);   /* publish divisor clamped [1, 100] */
    }
    /* Kalman core cached values (Kalman 1960) */
    kcc_kalman_p_est_init_val = (u32)kcc_kalman_p_est_init;       /* publish p_est init */
    kcc_kalman_p_est_floor_val = (u32)kcc_kalman_p_est_floor;     /* publish p_est floor */
    kcc_kalman_outlier_ms_val = (u32)kcc_kalman_outlier_ms;       /* publish outlier ms */
    kcc_kalman_scale_shift_val = (u32)ilog2(kcc_kalman_scale);      /* publish Kalman shift (scale = 1<<shift) */

    /* Precompute Kalman outlier rejection constants.
     * These are invariant between sysctl writes — recomputing them
     * on every Kalman invocation is wasteful on the hot path. */
    {
        u64 ms_us = (u64)kcc_kalman_outlier_ms_val * USEC_PER_MSEC;
        u32 shift = kcc_kalman_scale_shift_val;
        kcc_kalman_shift_cap_val = U64_MAX >> shift;
        kcc_kalman_outlier_thresh_scaled_val = min_t(u64, ms_us, kcc_kalman_shift_cap_val) << shift;
    }

    /* Derived Kalman ratios (num/den -> single value) (Kalman 1960) */
    {
        u32 num = (u32)kcc_kalman_outlier_jitter_mult_num;                  /* read outlier jitter mult num */
        u32 den = (u32)kcc_kalman_outlier_jitter_mult_den;                  /* read outlier jitter mult den */
        kcc_kalman_outlier_jitter_mult_val = num / den; /* publish (num/den) */
    }
    {
        u32 num = (u32)kcc_kalman_q_min_factor_num;                         /* read Q min factor num */
        u32 den = (u32)kcc_kalman_q_min_factor_den;                         /* read Q min factor den */
        kcc_kalman_q_min_factor_val = num / den;        /* publish (num/den) */
    }
    {
        u32 num = (u32)kcc_kalman_p_est_init_rtt_div_num;                   /* read p_est init RTT div num */
        u32 den = (u32)kcc_kalman_p_est_init_rtt_div_den;                   /* read p_est init RTT div den */
        u32 val = max_t(u32, num / den, 1U);                      /* compute divisor, floor at 1 (den clamped >= 1) */
        kcc_kalman_p_est_init_rtt_div_val = val;                  /* publish p_est init RTT divisor */
    }
    /* EWMA coefficients */
    kcc_ewma_qdelay_num_val = kcc_ewma_qdelay_num;                /* publish EWMA qdelay num */
    kcc_ewma_qdelay_den_val = kcc_ewma_qdelay_den;                /* publish EWMA qdelay den */
    kcc_ewma_jitter_num_val = kcc_ewma_jitter_num;                /* publish EWMA jitter num */
    kcc_ewma_jitter_den_val = kcc_ewma_jitter_den;                /* publish EWMA jitter den */

    /* BBR-S covariance-matched noise estimation: publish snapped/clamped values */
    kcc_kalman_noise_alpha_num_val = kcc_kalman_noise_alpha_num;   /* publish noise alpha num */
    kcc_kalman_noise_alpha_den_val = kcc_kalman_noise_alpha_den;   /* publish noise alpha den */
    kcc_kalman_noise_beta_num_val = kcc_kalman_noise_beta_num;     /* publish noise beta num */
    kcc_kalman_noise_beta_den_val = kcc_kalman_noise_beta_den;     /* publish noise beta den */
    kcc_kalman_q_est_max_val = kcc_kalman_q_est_max;               /* publish Q est max */
    kcc_kalman_r_est_max_val = kcc_kalman_r_est_max;               /* publish R est max */
    kcc_kalman_q_est_floor_val = kcc_kalman_q_est_floor;           /* publish Q est floor */
    kcc_kalman_r_est_floor_val = kcc_kalman_r_est_floor;           /* publish R est floor */
    kcc_kalman_noise_mode_val = kcc_kalman_noise_mode;             /* publish noise mode */
    kcc_kalman_noise_avg_num_val = kcc_kalman_noise_avg_num;
    kcc_kalman_noise_avg_den_val = kcc_kalman_noise_avg_den;

    /* Precompute noise estimation complements for hot-path use (BBR-S) */
    kcc_kalman_noise_alpha_complement = (u32)kcc_kalman_noise_alpha_den_val - (u32)kcc_kalman_noise_alpha_num_val; /* alpha_d - alpha_n */
    kcc_kalman_noise_beta_complement = (u32)kcc_kalman_noise_beta_den_val - (u32)kcc_kalman_noise_beta_num_val;   /* beta_d - beta_n */

    /* ECN: publish clamped values and derived backoff ratio */
    kcc_alone_confirm_rounds_val = kcc_alone_confirm_rounds;         /* publish alone confirm rounds */
    kcc_alone_qdelay_thresh_us_val = kcc_alone_qdelay_thresh_us;    /* publish alone qdelay threshold */
    kcc_alone_jitter_thresh_us_val = kcc_alone_jitter_thresh_us;    /* publish alone jitter threshold */
    kcc_alone_agg_state_level_val = kcc_alone_agg_state_level;      /* publish alone agg state level */
    kcc_alone_bypass_ecn_val = kcc_alone_bypass_ecn;                /* publish alone bypass ECN flag */
    kcc_ecn_enable_val = kcc_ecn_enable;                            /* publish ECN enable */
    kcc_ecn_backoff_val = (u32)((u64)BBR_UNIT * (u32)kcc_ecn_backoff_num / (u32)kcc_ecn_backoff_den); /* backoff = BBR_UNIT * num / den */
    kcc_ecn_qdelay_thresh_us_val = kcc_ecn_qdelay_thresh_us;       /* publish ECN qdelay threshold */
    kcc_ecn_ewma_retained_val = kcc_ecn_ewma_retained;              /* publish ECN EWMA retained */
    kcc_ecn_ewma_total_val = kcc_ecn_ewma_total;                    /* publish ECN EWMA total */

    /* ECN idle decay: publish snapped values */
    kcc_ecn_idle_decay_num_val = kcc_ecn_idle_decay_num;             /* publish ECN idle decay num */
    kcc_ecn_idle_decay_den_val = kcc_ecn_idle_decay_den;             /* publish ECN idle decay den */

    /* Kalman outlier rejection: publish clamped consecutive reject limit */
    kcc_kalman_max_consec_reject_val = kcc_kalman_max_consec_reject; /* publish consec reject limit */

    /* PROBE_BW random offset and LT BW derived values */
    kcc_probe_bw_cycle_rand_val = kcc_probe_bw_cycle_rand;        /* publish PROBE_BW rand range */
    kcc_probe_bw_up_limit_val = kcc_probe_bw_up_limit;          /* publish PROBE_BW up-phase limit flag */
    kcc_lt_intvl_min_rtts_val = kcc_lt_intvl_min_rtts;            /* publish LT min RTTs */
    kcc_lt_intvl_max_mult_val = kcc_lt_intvl_max_mult;             /* publish LT timeout mult */
    kcc_lt_loss_thresh_val = (u32)kcc_lt_loss_thresh;                  /* publish LT loss threshold */
    kcc_lt_bw_ratio_val = (u32)((u64)BBR_UNIT * (u32)kcc_lt_bw_ratio_num / (u32)kcc_lt_bw_ratio_den); /* num/den * BBR_UNIT */
    kcc_lt_bw_diff_val = (u32)kcc_lt_bw_diff;                     /* publish LT BW absolute diff */
    kcc_lt_bw_max_rtts_val = kcc_lt_bw_max_rtts;                  /* publish LT BW max RTTs */
    kcc_lt_bw_inst_qdelay_thresh_us_val = kcc_lt_bw_inst_qdelay_thresh_us; /* publish LT BW inst qdelay thresh */
    kcc_lt_bw_ema_num_val = kcc_lt_bw_ema_num;
    kcc_lt_bw_ema_den_val = kcc_lt_bw_ema_den;

    /* Pacing rate double threshold, ACK aggregation max, PROBE_RTT mode duration */
    {
        u32 num = (u32)kcc_extra_acked_max_ms_num;                           /* read ACK-agg max ms num */
        u32 den = (u32)kcc_extra_acked_max_ms_den;                           /* read ACK-agg max ms den */
        kcc_extra_acked_max_ms_val = num / den;          /* publish (num/den) (den clamped >= 1) */
    }
    {
        u32 num = (u32)kcc_probe_rtt_mode_ms_num;                            /* read PROBE_RTT mode ms num */
        u32 den = (u32)kcc_probe_rtt_mode_ms_den;                            /* read PROBE_RTT mode ms den */
        u32 val = max_t(u32, num / den, 1U);                      /* compute stay duration, floor at 1 (den clamped >= 1) */
        kcc_probe_rtt_mode_ms_val = val;                           /* publish PROBE_RTT mode ms */
    }
    /*
     * If the gain table has never been explicitly configured (still at
     * static-zero defaults), populate the first cycle_len entries with
     * BBRv1-compatible values.  This ensures KCC probes for bandwidth
     * at 1.25x gain on phase 0 and drains at 0.75x on phase 1, matching
     * the BBR PROBE_BW cycle shape that is essential for discovering
     * available bandwidth above the sliding-window maximum.
     *
     * The pattern repeats modulo cycle_len over the full 256-slot table.
     */
    if (kcc_gain_table_defaulted) {
        int k, phase = 0;
        for (k = 0; k < KCC_GAIN_SLOTS; k++) {
            switch (phase) {
            case 0:  kcc_gain_num[k] = 5; kcc_gain_den[k] = 4; break; /*  5/4 = 1.25x probe */
            case 1:  kcc_gain_num[k] = 3; kcc_gain_den[k] = 4; break; /*  3/4 = 0.75x drain */
            default: kcc_gain_num[k] = 1; kcc_gain_den[k] = 1; break; /*  1/1 = 1.0x cruise */
            }
            if (++phase >= kcc_probe_bw_cycle_len) {
                phase = 0;
            }
        }
    }
    /* Rebuild the cycle gain table from the (possibly updated) arrays */
    kcc_rebuild_gain_table();                                                 /* recompute kcc_cycle_gain_table[] */

    kcc_kf_enable = clamp(kcc_kf_enable, 0, 1);                              /* global KF enable [0, 1] */
    kcc_kf_startup_r_pct = clamp(kcc_kf_startup_r_pct, 1, 100);              /* startup R pct [1, 100] */
    kcc_kf_steady_r_pct = clamp(kcc_kf_steady_r_pct, 1, 100);                /* steady R pct [1, 100] */
    kcc_kf_q_shift = clamp(kcc_kf_q_shift, 0, 30);                           /* process noise shift [0, 30] */
    kcc_kf_chi2_num = clamp(kcc_kf_chi2_num, 1, 100000);                     /* chi2 num [1, 100k] */
    kcc_kf_chi2_den = clamp(kcc_kf_chi2_den, 1, 100000);                     /* chi2 den [1, 100k] */
    kcc_kf_discount_num = clamp(kcc_kf_discount_num, 0, 100);                /* discount num [0, 100] */
    kcc_kf_discount_den = clamp(kcc_kf_discount_den, 1, 100000);             /* discount den [1, 100k] */
    kcc_kf_enable_val = (u32)kcc_kf_enable;                                  /* publish KF enable */
    kcc_kf_startup_r_pct_val = (u32)kcc_kf_startup_r_pct;                    /* publish startup R pct */
    kcc_kf_steady_r_pct_val = (u32)kcc_kf_steady_r_pct;                      /* publish steady R pct */
    kcc_kf_q_shift_val = (u32)kcc_kf_q_shift;                                /* publish Q shift */
    kcc_kf_chi2_num_val = (u64)kcc_kf_chi2_num;                              /* publish chi2 num */
    kcc_kf_chi2_den_val = (u64)kcc_kf_chi2_den;                              /* publish chi2 den */
    kcc_kf_discount_num_val = (u32)kcc_kf_discount_num;                      /* publish discount num */
    kcc_kf_discount_den_val = (u32)kcc_kf_discount_den;                      /* publish discount den */

    kcc_bw_rt_cycle_len_val = kcc_bw_rt_cycle_len;                 /* publish BW RT cycle length */
    kcc_cwnd_min_target_val = kcc_cwnd_min_target;                 /* publish min cwnd target */
    kcc_sndbuf_expand_factor = clamp(kcc_sndbuf_expand_factor, 2, 100);       /* clamp sndbuf factor */
    kcc_sndbuf_expand_factor_val = kcc_sndbuf_expand_factor;        /* publish sndbuf expand factor */
    kcc_ack_epoch_max = clamp(kcc_ack_epoch_max, 65536, 0x7FFFFFFF);         /* clamp epoch cap [64K, 2G] */
    kcc_ack_epoch_max_val = (u32)kcc_ack_epoch_max;                 /* publish epoch cap */
}                                                                             /* kcc_init_module_params */

/* ---- Global Kalman BDP atomic state (cross-connection) ---------------- */
/*
 * KF atomic globals: shared across all KCC connections on this host.
 * Using atomic types because the Kalman filter runs in softirq context
 * without per-connection locks — the estimation target (available
 * bandwidth on the bottleneck) is a shared resource.
 *   kcc_kf_x      — posterior state estimate (BW_UNIT)
 *   kcc_kf_P      — posterior error covariance
 *   kcc_kf_active — 1 = filter has been seeded with at least one sample
 */
static atomic64_t kcc_kf_x = ATOMIC64_INIT(0);          /* global available BW estimate (BW_UNIT) */
static atomic64_t kcc_kf_P = ATOMIC64_INIT(0);          /* error covariance (initial uncertainty) */
static atomic_t kcc_kf_active = ATOMIC_INIT(0);         /* 1 = filter has been seeded (cold-start guard) */

/* ---- Forward Declarations -------------------------------------------- */
static void kcc_check_probe_rtt_done(struct sock* sk);                        /* forward: check PROBE_RTT exit */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)                            /* kernel 6.10+ adds ack/flags to cong_control */
void kcc_main(struct sock* sk, u32 ack __maybe_unused, int flags __maybe_unused, const struct rate_sample* rs); /* main entry (6.10+ sig) */
#else                                                                          /* pre-6.10 signature */
void kcc_main(struct sock* sk, const struct rate_sample* rs);                /* main entry (legacy sig) */
#endif
static void kcc_update_model(struct sock* sk, const struct rate_sample* rs,   /* forward: per-ACK model update */
    struct kcc_ext* ext);                                        /* extended state (may be NULL) */
static void kcc_alone_on_path_eval(struct sock* sk, struct kcc_ext* ext); /* forward: single-flow detection */
static void kcc_apply_cwnd_constraints(struct sock* sk, struct kcc_ext* ext); /* forward: apply cwnd gain caps */
static u32 kcc_ack_aggregation_cwnd(struct sock* sk, struct kcc_ext* ext, u32 bw);    /* forward: ACK agg cwnd bonus */
/* ACK aggregation confidence module forward declarations */
static u32 kcc_measure_ack_aggregation(struct sock* sk, const struct rate_sample* rs, struct kcc_ext* ext);
static u16 kcc_evaluate_agg_confidence(struct sock* sk, struct kcc_ext* ext, u32 extra_acked);
static u8 kcc_agg_state_from_confidence(u16 confidence);
static u32 kcc_agg_cwnd_compensation(struct sock* sk, struct kcc_ext* ext, u32 extra_acked, u16 confidence, u32 bw);
/* KCC_KFUNC functions — non-static for BTF kfunc registration */
void kcc_init(struct sock* sk);                                                /* per-connection init */
u32 kcc_min_tso_segs(struct sock* sk);                                          /* minimum TSO segments */
void kcc_cwnd_event(struct sock* sk, enum tcp_ca_event event);                  /* congestion event handler */
u32 kcc_sndbuf_expand(struct sock* sk);                                          /* send buffer expansion factor */
u32 kcc_undo_cwnd(struct sock* sk);                                               /* cwnd undo on spurious loss */
u32 kcc_ssthresh(struct sock* sk);                                                 /* ssthresh query */
void kcc_set_state(struct sock* sk, u8 new_state);                                  /* CA state transition handler */
/* Global Kalman BDP filter (cross-connection bandwidth estimation) forward declarations */
static u64 kcc_kf_compute_R(u64 z, u32 pct);                   /* compute measurement noise covariance */
static u64 kcc_kf_update(u64 z, u32 r_pct, bool check);        /* one-step Kalman BW update */
static u64 kcc_kf_get_init_bw(struct sock* sk);                  /* bootstrapped initial BW */

/* ---- Extended State Helpers ------------------------------------------- */

static inline struct kcc_ext* kcc_ext_get(const struct sock* sk)
{
    return ((struct kcc*)inet_csk_ca(sk))->ext;
}

/*
 * kcc_ext_destruct - Null the ext pointer and free the extended-state block.
 * @sk: TCP socket.
 *
 * Called from kcc_release() on socket close (non-softirq context).
 * No RCU needed -- see "CONCURRENCY & SAFETY MODEL" at struct kcc_ext.
 */
static void kcc_ext_destruct(struct sock* sk)
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);
    struct kcc_ext* ext = kcc->ext;

    if (!ext) {
        return;
    }

    kcc->ext = NULL;
    kfree(ext);
}
/*
 * kcc_release - Release callback invoked when a KCC connection is closed.
 * @sk: TCP socket.
 */
static void kcc_release(struct sock* sk)                                     /* socket close callback */
{
    kcc_ext_destruct(sk);                                                    /* destroy extended state */
}
/*
 * kcc_full_bw_reached - Query whether pipe-fill has been detected.
 *
 * @sk: TCP socket.
 *
 * Returns the 1-bit full_bw_reached flag.  Once set, KCC transitions
 * from STARTUP to DRAIN, and subsequent PROBE_BW uses this flag to
 * enable cwnd-to-target convergence (vs. exponential growth).
 *
 * Corresponds directly to kernel BBR v5.4 (net/ipv4/tcp_bbr.c):
 * bbr_full_bw_reached().  Identical semantics and implementation.
 * No KCC deviation — the bitfield and accessor are verbatim copies.
 *
 * Type note: returns bool, stored as u32:1 bitfield.  Kernel BBR stores
 * the same flag as u8:1 in struct bbr; the return type is identical.
 */
static bool kcc_full_bw_reached(const struct sock* sk)                       /* check if pipe capacity detected */
{
    return ((struct kcc*)inet_csk_ca(sk))->full_bw_reached;                  /* return full_bw_reached bitfield */
}
/*
 * kcc_max_bw - Return the sliding-window maximum bandwidth estimate.
 *
 * @sk: TCP socket.
 *
 * Reads the max from the struct minmax running over kcc_bw_rt_cycle_len
 * round-trip windows.  Stored in BW_UNIT (segments * (1<<24) per usec).
 *
 * Corresponds to kernel BBR v5.4: bbr_max_bw().  Identical usage of
 * struct minmax and minmax_get().  No KCC deviation.
 *
 * Type note: u32 return matches kernel BBR's bbr_max_bw() return type.
 * The minmax struct stores entries as u32; BW_UNIT fixed-point ensures
 * sufficient precision for BDP multiplication without 64-bit overflow.
 */
static u32 kcc_max_bw(const struct sock* sk)                                  /* get sliding-window max BW */
{
    return minmax_get(&((struct kcc*)inet_csk_ca(sk))->bw);                 /* extract current max from minmax */
}
/*
 * kcc_bw - Return the active bandwidth estimate (either max_bw or lt_bw).
 *
 * @sk: TCP socket.
 *
 * When lt_use_bw == 1 (long-term BW is active and consistent), returns
 * the LT BW estimate.  Otherwise returns the sliding-window max.
 *
 * Corresponds to kernel BBR v5.4: bbr_bw().  Kernel BBR's bw() always
 * returns minmax_get(&bbr->bw).  KCC extends this with an LT-BW override
 * for lossy paths — see kcc_lt_bw_sampling for the full mechanism.
 *
 * KCC deviation: LT-BW override (bbr_bw has no such path).
 * BBR practice: bbr_bw() unconditionally returns the sliding-window max.
 * WHY: on lossy paths (WiFi, cellular), the max-bw window collapses
 * as old high-bw samples expire.  LT-BW preserves a stable estimate
 * through loss periods, then transitions back transparently.
 * BENEFIT: maintain send rate through lossy periods; faster recovery.
 *
 * Type note: u32 return in BW_UNIT.  Both branches converge to u32
 * (kcc->lt_bw is u32, kcc_max_bw returns u32).
 */
static u32 kcc_bw(const struct sock* sk)                                      /* get active BW estimate */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                          /* get CA private area */

    return kcc->lt_use_bw ? kcc->lt_bw : kcc_max_bw(sk);                      /* LT BW active ? lt_bw : max_bw */
}
/*
 * kcc_get_cycle_mstamp - Reconstruct the 64-bit cycle timestamp from the
 * hi/lo 32-bit halves stored in the kcc struct.
 * @kcc: per-connection KCC state.
 */
static inline u64 kcc_get_cycle_mstamp(const struct kcc* kcc)                 /* reconstruct 64-bit cycle mstamp */
{
    return ((u64)kcc->cycle_mstamp_hi << KCC_MSTAMP_HI_SHIFT) | kcc->cycle_mstamp_lo;          /* combine hi<<KCC_MSTAMP_HI_SHIFT | lo */
}
/*
 * kcc_set_cycle_mstamp - Store a 64-bit tcp_mstamp as two 32-bit halves.
 * @kcc: per-connection KCC state.
 * @val: the 64-bit timestamp value.
 *
 * Used to record the start of each PROBE_BW cycle phase.  The hi/lo split
 * saves two u32 words in the size-constrained struct kcc.
 */
static inline void kcc_set_cycle_mstamp(struct kcc* kcc, u64 val)             /* store 64-bit mstamp as hi+lo */
{
    kcc->cycle_mstamp_hi = (u32)(val >> KCC_MSTAMP_HI_SHIFT);                                  /* store high 32 bits */
    kcc->cycle_mstamp_lo = (u32)(val);                                       /* store low 32 bits */
}
/* ---- Pacing and Rate Helpers ----------------------------------------- */

/*
 * kcc_rate_bytes_per_sec - Convert bandwidth (BW_UNIT) * gain (BBR_UNIT)
 * into a pacing rate in bytes/second, with pacing margin applied.
 *
 * @sk:   TCP socket (for mss_cache).
 * @rate: bandwidth in BW_UNIT (segments * (1<<24) per usec).
 * @gain: pacing gain in BBR_UNIT units (256 = 1.0x).
 *
 * Formula (standard BBR pacing rate calc, Cardwell et al. 2016):
 *   step1:  (rate * gain) >> BBR_SCALE           -> gain-adjusted BW
 *   step2:  step1 * mss_cache                     -> raw bytes/interval
 *   step3:  step2 * USEC_PER_SEC >> BW_SCALE      -> bytes per second
 *   step4:  step3 * pacing_margin_div / 100       -> apply margin (e.g., 99/100)
 *
 * Corresponds to kernel BBR v5.4: bbr_rate_bytes_per_sec().
 * Identical formula and scaling.  No KCC deviation.
 *
 * Type note: 'gain' is 'int' rather than u32 because kernel BBR stores
 * pacing_gain as a signed int in struct bbr (to simplify gain-delta
 * arithmetic).  KCC's bitfield store is u32:10, but the function parameter
 * follows kernel BBR's int convention for compatibility.  'rate' is u64
 * because the intermediate product (rate * mss) can exceed U32_MAX on
 * multi-gigabit paths.
 *
 * Overflow guard: before step4, cap at U64_MAX / divisor.
 */
static u64 kcc_rate_bytes_per_sec(struct sock* sk, u64 rate, int gain)        /* compute paced bytes/sec */
{
    unsigned int mss = tcp_sk(sk)->mss_cache;

    rate *= mss;
    rate = mul_u64_u32_shr(rate, gain, BBR_SCALE);
    rate = mul_u64_u32_shr(rate, USEC_PER_SEC, BW_SCALE);
    rate = rate * kcc_pacing_margin_div_val / KCC_PCT_BASE;
    return rate;
}
/*
 * kcc_bw_to_pacing_rate - Compute sk_pacing_rate from BW and gain,
 * capped by sk_max_pacing_rate (socket-level upper bound from
 * e.g. SO_MAX_PACING_RATE).
 *
 * @sk:   TCP socket.
 * @bw:   bandwidth in BW_UNIT units.
 * @gain: gain in BBR_UNIT units.
 *
 * Corresponds to kernel BBR v5.4: bbr_bw_to_pacing_rate().
 * Identical delegation pattern (wraps kcc_rate_bytes_per_sec with
 * a socket-level cap).  No KCC deviation.
 *
 * Type note: 'bw' is u32 (BW_UNIT) and 'gain' is int (BBR_UNIT),
 * matching kernel BBR's parameter types.  The u64 return type is
 * required for pacing rates above 4 Gbps.
 */
static u64 kcc_bw_to_pacing_rate(struct sock* sk, u32 bw, int gain)           /* convert BW+gain to pacing rate */
{
    return min_t(u64, kcc_rate_bytes_per_sec(sk, bw, gain),                    /* computed bytes/sec */
        sk->sk_max_pacing_rate);                                          /* cap at socket max */
}
/*
 * kcc_init_pacing_rate_from_rtt - Bootstrap pacing rate from cwnd and SRTT
 * before any bandwidth samples are available.
 *
 * @sk: TCP socket.
 *
 * If SRTT is known: rate = (cwnd * BW_UNIT / srtt_us) * high_gain.
 * Otherwise: uses a 1 ms fallback RTT.
 * This ensures the connection has a valid pacing rate from the first ACK.
 *
 * Corresponds to kernel BBR v5.4: bbr_init_pacing_rate_from_rtt().
 * Identical bootstrap logic.  KCC's only addition is the Global Kalman
 * BDP injection in kcc_init(), which pre-seeds the bandwidth tracker
 * and sets has_seen_rtt = 1, causing this function to be skipped on
 * KF-initialised connections.  No KCC deviation in this function itself.
 *
 * Type note: rtt_us is kept as u32 throughout; BW_UNIT scaling via
 * div_u64() ensures the 64-bit intermediate (cwnd * BW_UNIT) does not
 * overflow on paths with cwnd up to ~2^18 segments.
 */
static void kcc_init_pacing_rate_from_rtt(struct sock* sk)                    /* bootstrap pacing from cwnd+SRTT */
{
    struct tcp_sock* tp = tcp_sk(sk);                                        /* get TCP socket state */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                          /* get KCC CA state */
    u32 rtt_us;                                                              /* RTT estimate in us */
    u64 bw;                                                                  /* computed bandwidth */

    if (tp->srtt_us) {                                                       /* SRTT sample available */
        /* SRTT is stored as 8 * smoothed RTT in us; >>3 recovers actual us */
        rtt_us = max_t(u32, tp->srtt_us >> 3, 1U);                            /* extract smoothed RTT, floor at 1 */
        kcc->has_seen_rtt = 1;                                               /* mark: SRTT has been sampled */
    }
    else {                                                                 /* no SRTT yet */
        rtt_us = USEC_PER_MSEC;                                              /* fallback: 1 ms */
    }

    /* bw = cwnd * BW_UNIT / rtt_us  (bandwidth proxy from BDP) */
    bw = (u64)tcp_snd_cwnd(tp) * BW_UNIT;                                         /* cwnd in BW_UNIT scale */
    bw = div_u64(bw, rtt_us);                                   /* 64-bit / 32-bit division: bw in BW_UNIT */

    WRITE_ONCE(sk->sk_pacing_rate, kcc_bw_to_pacing_rate(sk, bw, kcc_high_gain_val));
}
/*
 * kcc_set_pacing_rate - Set the socket pacing rate.
 *
 * @sk:   TCP socket.
 * @bw:   bandwidth in BW_UNIT units.
 * @gain: pacing gain in BBR_UNIT units.
 *
 * Rate application policy (matches BBR, Cardwell et al. 2016):
 *   - full_bw_reached: apply ALL rate changes immediately.  The pipe
 *     capacity is known; the pacing engine tracks bw directly.
 *   - STARTUP / DRAIN (not yet full_bw): only apply rate INCREASES.
 *     Transient dips are ignored — the bandwidth estimate should only
 *     grow during pipe-filling.
 *   - No rate smoothing is applied.  Smoothing acts as a low-pass filter
 *     that prevents bandwidth discovery from the 1.25x probe phase.
 *
 * Corresponds to kernel BBR v5.4: bbr_set_pacing_rate().
 * Identical policy on rate-increase-only during STARTUP, instant apply
 * after full_bw_reached.  No rate smoothing in either implementation.
 *
 * KCC deviation (within the function body, not in the policy logic):
 * Steps 3-4 add a Global Kalman BDP pacing-rate floor during STARTUP,
 * ensuring the pacing rate never drops below the global fair-share
 * estimate.  Kernel BBR has no cross-connection bandwidth sharing and
 * thus no equivalent floor.  See kcc_kf_get_init_bw() for the full
 * Global KF mechanism.
 * WHY: On a shared bottleneck, individual connections that under-sample
 * bandwidth (app-limited, late-starting) would otherwise pace below the
 * fair-share rate, causing throughput unfairness.  The KF floor corrects
 * this by providing a common lower bound derived from all connections.
 * BENEFIT: Improved flow fairness on shared bottlenecks without per-flow
 * coordination.
 *
 * Type note: 'bw' is u32 (BW_UNIT), 'gain' is int (BBR_UNIT), matching
 * kernel BBR's parameter types for bbr_set_pacing_rate().
 */
static void kcc_set_pacing_rate(struct sock* sk, u32 bw, int gain)            /* set sk_pacing_rate (no smoothing — matches BBR, Cardwell et al. 2016) */
{
    struct tcp_sock* tp = tcp_sk(sk);                                        /* get TCP socket state */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                          /* get KCC CA state */
    u64 rate = kcc_bw_to_pacing_rate(sk, bw, gain);                            /* compute target pacing rate */

    /* Step 3 (Global Kalman BDP): pacing-rate floor during STARTUP.
     * Even with the defensive floor in kcc_update_bw, kcc_set_pacing_rate
     * can still produce a low pacing rate if the input bw is the global
     * floor value but the gain is below BBR_UNIT (e.g. during LT BW fallback
     * or DRAIN).  This guard computes the global-Kalman bootstrap rate at
     * BBR_UNIT gain and enforces it as a hard floor during STARTUP.
     * NOTE: inlined instead of calling kcc_kf_get_init_bw() because the
     * latter contains a cwnd-sanity guard that rejects calls after cwnd
     * has already been expanded by Step 1. */
    if (kcc_kf_enable_val && atomic_read(&kcc_kf_active) && kcc->mode == KCC_STARTUP) {   /* KF active + STARTUP */
        u64 kf_bw = (u64)atomic64_read(&kcc_kf_x);                               /* read global fair-share estimate */
        if (kf_bw > 0) {                                                         /* valid estimate */
            u64 init_bw = kf_bw * (u64)kcc_kf_discount_num_val / (u64)kcc_kf_discount_den_val; /* apply safety discount */
            u64 kf_rate;                                                         /* neutral-gain KF pacing rate */

            init_bw = init_bw * BBR_UNIT / (u64)kcc_high_gain_val;               /* gain-compensate: ÷ high_gain */
            kf_rate = kcc_bw_to_pacing_rate(sk, init_bw, BBR_UNIT);               /* compute neutral-gain KF rate */
            if (rate < kf_rate) {                                                     /* computed rate below KF floor */
                rate = kf_rate;                                                       /* enforce global-Kalman pacing floor */
            }
        }
    }

    /* Bootstrap: on the first SRTT sample, initialize pacing from RTT */
    if (unlikely(!kcc->has_seen_rtt && tp->srtt_us)) {                        /* first SRTT sample available */
        kcc_init_pacing_rate_from_rtt(sk);                                    /* bootstrap pacing from RTT */
    }

    /*
     * BBR rule (Cardwell et al. 2016): in steady state (full_bw_reached),
     * apply ALL rate changes immediately - the pipe capacity is known and
     * the pacing engine should track the bandwidth estimate directly.
     * Smoothing during steady state acts as a low-pass filter that
     * prevents bandwidth discovery from the 1.25x probe phase.
     *
     * During STARTUP (full_bw not yet reached), only apply increases.
     */
    if (likely(kcc_full_bw_reached(sk))) {
        /* Steady state: instant apply, matching BBR behavior */
        WRITE_ONCE(sk->sk_pacing_rate, rate);
        return;
    }

    /*
     * STARTUP / DRAIN / early PROBE_BW (full_bw not yet reached):
     * Only apply rate INCREASES, matching BBR's behavior:
     *   if (bbr_full_bw_reached(sk) || rate > sk_pacing_rate)
     *       WRITE_ONCE(sk->sk_pacing_rate, rate);
     * Decreases are ignored: during STARTUP the bandwidth estimate should
     * only grow, and transient dips should not pull down the pacing rate.
     */
    if (rate > sk->sk_pacing_rate) {                               /* only apply increases */
        WRITE_ONCE(sk->sk_pacing_rate, rate);
    }
}
/*
 * kcc_min_tso_segs - Minimum TSO segments for the current pacing rate.
 *
 * @sk: TCP socket.
 *
 * Returns 1 for low pacing rates (< kcc_min_tso_rate / divisor), 2 otherwise.
 * The divisor is adaptive: halved (4) when Kalman is converged with low jitter
 * (larger TSO bursts for high-confidence clean paths), doubled (16) when jitter
 * is high (smaller bursts for jittery paths).  Default base divisor is 8.
 *
 * Corresponds to kernel BBR v5.4: bbr_min_tso_segs().
 * Kernel BBR uses a fixed threshold comparison with no divisor adaptation
 * (hardcodes the /2 divisor and the rate threshold).
 *
 * KCC deviation: adaptive divisor based on Kalman p_est and jitter_ewma.
 * BBR practice: bbr_min_tso_segs(sk) returns 1 if pacing_rate < 1.2Mbps,
 * else 2.  Fixed threshold, no path awareness.
 * WHY: On converged, low-jitter paths, halving the divisor permits larger
 * TSO bursts, reducing per-packet overhead and improving CPU efficiency.
 * On jittery paths, doubling the divisor forces smaller bursts, preventing
 * micro-bursts from amplifying RTT variance.
 * BENEFIT: CPU efficiency on clean paths; jitter mitigation on noisy paths.
 *
 * Type note: u32 return matches kernel BBR's bbr_min_tso_segs().
 * Marked KCC_KFUNC for BPF struct_ops attachment.
 */
KCC_KFUNC u32 kcc_min_tso_segs(struct sock* sk)                        /* compute minimum TSO segments */
{
    u32 div = kcc_min_tso_rate_div_val;                                        /* base divisor (default 8) */
    u32 tso_rate_thresh;                                                       /* rate threshold */
    struct kcc_ext* ext = kcc_ext_get(sk);                                     /* extended state (may be NULL) */
    if (ext) {
        if (ext->p_est < kcc_kalman_converged_p_est_val &&          /* Kalman converged + low jitter */
            ext->jitter_ewma < KCC_TSO_LOW_JITTER_THRESH_US) {                                         /* jitter < 1ms: halve divisor */
            div = max_t(u32, 2, div >> 1);                                    /* 8 → 4: larger TSO bursts (div/2) */
        }
        else if (ext->jitter_ewma > KCC_TSO_HIGH_JITTER_THRESH_US) {                                    /* jitter > 4ms: double divisor */
            div = min_t(u32, 32, div << 1);                                   /* 8 → 16: smaller TSO bursts (div*2) */
        }
    }
    tso_rate_thresh = max_t(u32, 1, kcc_min_tso_rate_val / div);               /* compute threshold */
    return READ_ONCE(sk->sk_pacing_rate) < tso_rate_thresh
        ? (u32)kcc_tso_segs_low_val
        : (u32)kcc_tso_segs_default_val;
}
/*
 * kcc_tso_segs_goal - Target number of TSO segments for GSO skb creation.
 *
 * @sk: TCP socket.
 *
 * Formula (BBR standard, Cardwell et al. 2016):
 *   1. bytes = min(pacing_rate >> pacing_shift, GSO_MAX_SIZE - 1 - MAX_TCP_HEADER)
 *   2. segs  = max(bytes / mss_cache, kcc_min_tso_segs)
 *   3. segs  = min(segs, tso_max_segs)
 *
 * pacing_rate >> pacing_shift converts the byte-per-second rate into the
 * byte budget for one pacing interval (approx 1 ms).
 *
 * Corresponds to kernel BBR v5.4: bbr_tso_segs_goal().
 * Identical formula, same constants (GSO_MAX_SIZE, MAX_TCP_HEADER).
 * KCC calls kcc_min_tso_segs() (which has Kalman-based adaptation)
 * instead of kernel BBR's bbr_min_tso_segs() — see that function
 * for the deviation details.
 *
 * Type note: u32 return; intermediate bytes/segs are computed as
 * unsigned long for compatibility with sk_pacing_rate >> sk_pacing_shift.
 */
static u32 kcc_tso_segs_goal(struct sock* sk)                                 /* compute TSO segment target */
{
    struct tcp_sock* tp = tcp_sk(sk);                                        /* get TCP socket state */
    u32 bytes, segs;                                                         /* intermediate bytes and segs */

    bytes = min_t(unsigned long,                                             /* compute byte budget per pacing interval */
        READ_ONCE(sk->sk_pacing_rate) >> READ_ONCE(sk->sk_pacing_shift),           /* rate -> bytes per interval */
        GSO_MAX_SIZE - 1 - MAX_TCP_HEADER);                             /* cap at GSO max minus headers */
    if (unlikely(!tp->mss_cache)) {
        return kcc_min_tso_segs(sk);
    }

    segs = max_t(u32, bytes / tp->mss_cache, kcc_min_tso_segs(sk));           /* convert to segments, floor at min */
    return min_t(u32, segs, kcc_tso_max_segs_val);                  /* cap at configured max TSO segs */
}
/* ---- CWND Save/Restore ----------------------------------------------- */

/*
 * kcc_save_cwnd - Save the current cwnd for later restoration.
 *
 * @sk: TCP socket.
 *
 * BBR logic (Cardwell et al. 2016): when entering recovery or PROBE_RTT,
 * record cwnd so it can be restored afterward.  If already in a recovery
 * state, keep the maximum of prior_cwnd and current cwnd (since recovery
 * may have already reduced cwnd, we want to restore to the pre-recovery peak).
 *
 * Corresponds to kernel BBR v5.4: bbr_save_cwnd().
 * Identical logic and semantics.  No KCC deviation.
 *
 * Type note: prior_cwnd is stored as standalone u32 in struct kcc
 * (kernel BBR stores it as u32 in struct bbr).  The bitfield packing
 * of struct kcc does not include prior_cwnd because it requires the
 * full 32-bit range and is accessed on every recovery transition.
 */
static void kcc_save_cwnd(struct sock* sk)                                    /* save cwnd for later restore */
{
    struct tcp_sock* tp = tcp_sk(sk);                                        /* get TCP socket state */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                          /* get KCC CA state */

    if (kcc->prev_ca_state < TCP_CA_Recovery && kcc->mode != KCC_PROBE_RTT) { /* not already in recovery/ProbeRTT */
        kcc->prior_cwnd = tcp_snd_cwnd(tp);                                       /* first entry to recovery/ProbeRTT: save cwnd */
    }
    else {                                                                 /* already in recovery/ProbeRTT */
        kcc->prior_cwnd = max_t(u32, kcc->prior_cwnd, tcp_snd_cwnd(tp));          /* keep max of saved and current */
    }
}
/*
 * kcc_cwnd_event - Handle TCP CA events.
 *
 * @sk:    TCP socket.
 * @event: congestion event type (e.g., CA_EVENT_TX_START).
 *
 * On TX_START when app_limited (connection was idle):
 *   - Sets idle_restart = 1 (triggers exponential cwnd ramp).
 *   - Resets ACK aggregation epoch.
 *   - In PROBE_BW: resets pacing to 1.0x of current bw estimate.
 *   - In PROBE_RTT: checks if probe can end (pipe already drained by idle).
 *
 * Corresponds to kernel BBR v5.4: bbr_cwnd_event().
 * Identical event handling for CA_EVENT_TX_START.  KCC additionally
 * resets the ACK aggregation epoch (ack_epoch_mstamp, ack_epoch_acked)
 * on idle restart — kernel BBR resets its extra_acked state implicitly
 * through the sliding-window mechanism; KCC's explicit reset prevents
 * stale aggregation data from inflating cwnd after an idle period.
 *
 * KCC deviation: explicit ACK epoch reset on idle restart.
 * BBR practice: bbr_cwnd_event() does not reset ACK-agg state.
 * WHY: After an idle period, the saved extra_acked from before the idle
 * represents a different traffic pattern; reusing it would inflate cwnd
 * when the connection resumes, potentially causing a burst.
 * BENEFIT: Prevents post-idle cwnd burst from stale ACK-agg data.
 *
 * Type note: 'event' is enum tcp_ca_event, matching kernel BBR's
 * bbr_cwnd_event() signature exactly.  Marked KCC_KFUNC for BPF
 * struct_ops compatibility.
 */
KCC_KFUNC void kcc_cwnd_event(struct sock* sk, enum tcp_ca_event event) /* handle CA event */
{
    struct tcp_sock* tp = tcp_sk(sk);                                        /* get TCP socket state */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                          /* get KCC CA state */

    if (event == CA_EVENT_TX_START && tp->app_limited) {                       /* idle restart on TX_START */
        struct kcc_ext* ext = kcc_ext_get(sk);                               /* retrieve ext (with UAF guard) */

        kcc->idle_restart = 1;                                               /* set idle restart flag */
        if (ext) {                                                            /* ext available */
            ext->ack_epoch_mstamp = tp->tcp_mstamp;                           /* reset agg time base */
            ext->ack_epoch_acked = 0;                                         /* reset agg byte count */
        }
        /* BBR rule: reset pacing rate to 1.0x of current bw estimate on idle
         * restart.  This prevents the connection from bursting at the old
         * PROBE_BW phase's gain (e.g., 1.25x probes would overshoot into an
         * idle pipe, 0.75x drain would under-utilize it).  BBR does this via
         * bbr_set_pacing_rate(sk, bbr_bw(sk), BBR_UNIT). */
        if (kcc->mode == KCC_PROBE_BW) {
            kcc_set_pacing_rate(sk, kcc_bw(sk), BBR_UNIT);
        }
        /* In PROBE_RTT, the idle period naturally drained the pipe.
         * Check if we can exit PROBE_RTT early to avoid a redundant wait. */
        if (kcc->mode == KCC_PROBE_RTT) {
            kcc_check_probe_rtt_done(sk);
        }
    }
}

/*
 * kcc_get_model_rtt - Return the RTT estimate used for BDP calculation.
 * @sk:  TCP socket.
 * @ext: extended state (may be NULL if allocation failed).
 *
 * Priority:
 *   0. If alone_on_path: return min_rtt_us — pure BBR minimum for
 *      single-flow scenarios, bypassing all Kalman logic.
 *   1. If Kalman filter has converged (x_est valid + sample_cnt >= min_samples)
 *      and kcc_rtt_mode == 1 (FILTER, default):
 *      return x_est_us directly — faster adaptation to abrupt path changes
 *      (route failover, LEO handover, mobile cell switch); Kalman's
 *      outlier gating and adaptive noise estimation prevent queue-bloat
 *      from polluting the estimate.
 *   2. If Kalman filter has converged and kcc_rtt_mode == 0 (MIN):
 *      return min(x_est, min_rtt_us).
 *      The min_rtt clamp prevents any Kalman inflation but may stall BDP
 *      after a real path-latency increase (e.g., BGP reroute).
 *   3. Otherwise: return the sliding-window min_rtt_us.
 */
static u32 kcc_get_model_rtt(const struct sock* sk,                            /* get model RTT for BDP */
    const struct kcc_ext* ext)                                   /* extended state (may be NULL) */
{
    const struct kcc* kcc = (const struct kcc*)inet_csk_ca(sk);              /* get const KCC CA state */

    /*
     * When alone on path: use min_rtt_us directly, matching BBR's pure
     * minimum.  The Kalman smoothed estimate has a small positive bias
     * from one-sided measurement noise (queues only add, never subtract),
     * which inflates BDP and causes deeper queues in single-flow scenarios.
     */
    if (kcc->alone_on_path) {
        return kcc->min_rtt_us;                                              /* BBR-style exact minimum */
    }

    if (unlikely(!ext || !ext->x_est || ext->sample_cnt < kcc_kalman_min_samples_val)) { /* Kalman not converged */
        return kcc->min_rtt_us;                                              /* fall back to window min_rtt */
    }

    {
        u32 x_est_us = ext->x_est >> kcc_kalman_scale_shift_val;           /* descale Kalman estimate to µs */
        /*
         * In FILTER mode (default): model_rtt = x_est_us — faster path-change
         * adaptation, with Kalman outlier gating and adaptive Q/R as defense.
         * In MIN mode: model_rtt = min_t(x_est_us, min_rtt_us) — the
         * windowed-min clamp prevents BDP inflation but may stall after a
         * real RTT increase (e.g., BGP reroute from 50→100 ms).
         */
        u32 model_rtt;
        if (kcc_rtt_mode) {                                          /* FILTER mode: skip min_rtt clamp for faster adaptation */
            model_rtt = x_est_us;                                           /* raw Kalman/windowed-filter RTT estimate */
        }
        else {                                                           /* MIN mode (default): clamp against windowed minimum */
            model_rtt = min_t(u32, x_est_us, kcc->min_rtt_us);
        }

        if (model_rtt < 1) {
            model_rtt = 1;
        }

        return model_rtt;
    }
}

/* ---- BDP Calculation (Cardwell et al. 2016) ------------------------- */

/*
 * kcc_bdp - Compute the bandwidth-delay product in segments.
 *
 * @sk:   TCP socket.
 * @bw:   bandwidth in BW_UNIT (segments * (1<<24) per usec).
 * @gain: cwnd gain in BBR_UNIT (256 = 1.0x).  Type 'int' matches kernel
 *        BBR convention where gains are signed to simplify delta arithmetic.
 * @ext:  extended state (for Kalman RTT via kcc_get_model_rtt).
 *
 * Formula (standard BBR BDP calculation with ceiling):
 *   w       = bw * model_rtt_us
 *   bdp_raw = (w * gain) >> BBR_SCALE
 *   bdp_seg = ceil(bdp_raw / BW_UNIT) = (bdp_raw + BW_UNIT - 1) >> BW_SCALE
 *
 * Returns TCP_INIT_CWND (approx 10) if min_rtt_us is invalid or below floor.
 *
 * Corresponds to kernel BBR v5.4: bbr_bdp().
 * Identical formula and fixed-point scaling (BW_SCALE, BBR_SCALE, BW_UNIT).
 *
 * KCC deviation: model_rtt selection via kcc_get_model_rtt(), which may
 * return the Kalman estimate x_est_us instead of windowed min_rtt_us.
 * Kernel BBR's bbr_bdp() always uses bbr->min_rtt_us (the windowed min).
 * See kcc_get_model_rtt() for the full selection logic and rationale.
 *
 * Additional KCC deviation: BDP floor logic.  When the Kalman filter has
 * NOT converged and model_rtt < kcc_bdp_min_rtt_us, the RTT is floored to
 * kcc_bdp_min_rtt_us (default 1 us, effectively disabled).  Kernel BBR has
 * no explicit BDP min-RTT floor — it relies on the fact that min_rtt_us
 * is always measured from real RTT samples.  The floor prevents BDP
 * collapse on sub-microsecond-RTT paths where clock quantization could
 * produce min_rtt_us = 0.
 * BENEFIT: Robustness on loopback and datacenter paths with < 1 us RTT.
 *
 * Type note: 'bw' is u32 (BW_UNIT), 'gain' is int (BBR_UNIT).  The
 * intermediate product w = bw * model_rtt is u64 to prevent overflow
 * (bw up to ~2^32 * seg/us, model_rtt up to 10^7 us).  Return is u32
 * (segments), matching kernel BBR's bbr_bdp().
 */
static u32 kcc_bdp(struct sock* sk, u32 bw, int gain,                        /* compute BDP in segments */
    struct kcc_ext* ext)                                              /* extended state */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                          /* get KCC CA state */
    u32 model_rtt;                                                           /* chosen RTT estimate */
    u64 w;                                                                   /* intermediate product */
    u64 bdp64;                                                               /* ceiling-safe BDP in BW_UNIT scale */

    /* Get effective RTT estimate first — this way the floor check
     * sees the RTT that will actually be used in the BDP calculation.
     * This is critical: after Kalman takeover, min_rtt_us holds the
     * Kalman estimate which may be below kcc_bdp_min_rtt_us_val on
     * low-latency paths (e.g., 200us < 1000us default floor).
     * The model RTT is already floored at min_rtt_us, so it is a
     * superset of our knowledge. */
    model_rtt = kcc_get_model_rtt(sk, ext);

    /* Floor check: only applies when Kalman has NOT taken over.
     * The Kalman filter provides a converged estimate that is trusted
     * below the traditional floor.  The traditional window-based
     * min_rtt may not have found the true minimum yet, so a floor
     * prevents under-estimation. */
    if (unlikely(kcc->min_rtt_us == KCC_MIN_RTT_UNINIT)) {
        return TCP_INIT_CWND;
    }

    /* Traditional floor: only for non-Kalman paths.
     * Instead of bailing out with TCP_INIT_CWND, floor the model_rtt to the
     * configured minimum.  Returning TCP_INIT_CWND starves cwnd growth during
     * STARTUP on low-latency paths where the windowed min_rtt sits below the
     * configured floor. */
    {
        u32 bdp_floor = kcc_bdp_min_rtt_us_val;                             /* single volatile read for both condition and body */

        if (unlikely(!(ext && ext->x_est &&
            ext->sample_cnt >= kcc_kalman_min_samples_val) &&
            model_rtt < bdp_floor)) {
            model_rtt = bdp_floor;                                      /* cached floor value */
        }
    }

    /* w = bw (seg*BW_UNIT/usec) * rtt_us (us) -> intermediate segments */
    w = (u64)bw * model_rtt;                                                 /* bandwidth-delay product intermediate */
    /* Match BBR: (w * gain >> BBR_SCALE + BW_UNIT - 1) / BW_UNIT */
    bdp64 = mul_u64_u32_shr(w, gain, BBR_SCALE);
    bdp64 += BW_UNIT - 1;
    return (u32)(bdp64 >> BW_SCALE);
}
/*
 * kcc_quantization_budget - Add headroom for TSO/GSO bursts, delayed ACK,
 * and probing bonuses (Cardwell et al. 2016).
 *
 * @sk:   TCP socket.
 * @cwnd: base cwnd in segments.
 *
 * Headroom breakdown (BBR standard):
 *   1. +3 * tso_segs_goal: TSO/GSO burst accommodation.
 *   2. Round to even: accommodate standard delayed-ACK factor of 2.
 *   3. +probe_cwnd_bonus during PROBE_BW phase 0: extra headroom for
 *      the highest-gain probe phase to push past the sliding-window max.
 *   4. Clamp to snd_cwnd_clamp (socket-level upper bound).
 *
 * Corresponds to kernel BBR v5.4: bbr_quantization_budget().
 * Identical formula and constants (tso_headroom_mult = 3, round-to-even,
 * probe_cwnd_bonus = 2 in both implementations).
 *
 * Type note: u32 cwnd in, u32 out.  Intermediate additions are safe
 * because tso_headroom_mult * tso_segs_goal ≤ 3 * 127 ≪ U32_MAX.
 * Clamping to snd_cwnd_clamp (u32) provides the final safety check.
 */
static u32 kcc_quantization_budget(struct sock* sk, u32 cwnd)                 /* add headroom to cwnd target */
{
    struct tcp_sock* tp = tcp_sk(sk);                                        /* get TCP socket state */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                          /* get KCC CA state */

    cwnd += kcc_tso_headroom_mult_val * kcc_tso_segs_goal(sk);   /* TSO/GSO burst headroom */
    cwnd = (cwnd + 1) & ~1U;                                                 /* round to even for delayed-ACK */
    if (kcc->mode == KCC_PROBE_BW && (kcc->cycle_idx & (kcc_probe_bw_cycle_len_val - 1)) == 0) {                  /* highest-gain probe phase */
        cwnd += kcc_probe_cwnd_bonus_val;                         /* add extra probe cwnd bonus */
    }

    cwnd = min_t(u32, cwnd, tp->snd_cwnd_clamp);                             /* clamp to socket max */
    return cwnd;                                                             /* return budgeted cwnd */
}
/* ---- ECN (Explicit Congestion Notification) ---------------------------- */

/*
 * kcc_update_ecn_ewma - Update the EWMA of the ECN-CE mark ratio.
 * @sk:  TCP socket.
 * @rs:  rate sample from this ACK.
 * @ext: extended state (ecn_ewma, last_delivered_ce).
 *
 * Reads tp->delivered_ce from the TCP stack (RFC 3168, cumulative count
 * of CE-marked segments delivered to the receiver).  Computes the delta
 * since the last update and converts to a ratio scaled to BBR_UNIT.
 *
 * EWMA: ecn_ewma = (ecn_ewma * retained + instant) / total.
 * Default 3/4 -> 75% old, 25% new weight.
 *
 * On round boundaries with no new CE marks, a strong decay at the EWMA rate
 * is applied; on non-round ACKs, a gentle per-ACK idle decay prevents
 * ecn_ewma from persisting indefinitely on steady connections.
 */
static void kcc_update_ecn_ewma(struct sock* sk, const struct rate_sample* rs, /* update ECN-CE EWMA (function signature split) */
    struct kcc_ext* ext)                            /* parameter: extended state */
{
    struct tcp_sock* tp = tcp_sk(sk);                                            /* TCP socket */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                             /* KCC CA state */
    u32 ce_delta, instant = 0;                                        /* CE delta, instantaneous ratio */
    u32 cur_ce;                                                       /* snapshot of cumulative delivered_ce counter */
    u64 total_u64;                                                    /* total pkts in interval (delivered + losses) */

    if (!ext || !kcc_ecn_enable_val) {                                /* ECN disabled or no ext */
        return;                                                                   /* no-op */
    }

    cur_ce = tp->delivered_ce;                                                     /* read cumulative CE counter (RFC 3168) */
    if (rs->delivered == 0) {
        return;
    }

    total_u64 = (u64)rs->delivered + rs->losses;
    if (total_u64 == 0) {  /* defensive: unreachable when delivered>0 but guards against integer wrap */
        return;
    }

    ce_delta = cur_ce - ext->last_delivered_ce;                                    /* CE delta: unsigned wrap well-defined modulo 2^32 */
    ext->last_delivered_ce = cur_ce;                                               /* save for next delta computation */

    if (ce_delta > 0) {                                                            /* CE marks in this interval */
        /* instant = ce_delta * BBR_UNIT / total (BBR_UNIT = 256 = 100%) */
        u64 inst64 = ((u64)ce_delta * BBR_UNIT) / total_u64;
        instant = (u32)min_t(u64, inst64, BBR_UNIT);
        if (ext->ecn_ewma == 0) {                                                   /* first CE sample */
            ext->ecn_ewma = min_t(u32, instant, BBR_UNIT);                           /* initialize directly */
        }
        else {                                                                     /* EWMA update */
            u32 v = (ext->ecn_ewma * kcc_ecn_ewma_retained_val + instant) / /* old * retained + new */
                kcc_ecn_ewma_total_val;                                   /* divided by total */
            ext->ecn_ewma = min_t(u32, v, BBR_UNIT);                                    /* clamp to max */
        }
    }
    else {                                                                       /* no CE marks in this interval */
        if (ext->ecn_ewma > 0) {                                                    /* non-zero EWMA */
            if (kcc->round_start) {                                                  /* round boundary: strong decay */
                /* Fast decay at round boundaries: ecn_ewma *= retained / total.
                 * Default 3/4 -> 25% reduction per round. */
                ext->ecn_ewma = ext->ecn_ewma * kcc_ecn_ewma_retained_val / /* decay by retained/total */
                    kcc_ecn_ewma_total_val;                         /* gradual reduction */
            }
            else {                                                                 /* non-round-boundary: gentle per-ACK decay */
                /* Slow per-ACK decay to prevent ecn_ewma from persisting
                 * indefinitely on steady connections with infrequent
                 * round boundaries.  Default 31/32 -> ~3.2% per ACK,
                 * halving in ~2 RTTs at 10 ACKs/RTT. */
                ext->ecn_ewma = (u32)((u64)ext->ecn_ewma *                                /* numerator */
                    kcc_ecn_idle_decay_num_val /                                  /* * idle decay num */
                    (u64)kcc_ecn_idle_decay_den_val);                               /* / idle decay den */
            }
        }
    }
}
/*
 * kcc_ecn_backoff - Reduce cwnd_gain on ECN congestion signal.
 * @sk:  TCP socket.
 * @ext: extended state (ecn_ewma, qdelay_avg).
 *
 * Activation conditions (all must be true):
 *   1. kcc_ecn_enable != 0.
 *   2. ext valid and Kalman filter converged (p_est < converged_p_est,
 *      sample_cnt >= min_samples).
 *   3. ecn_ewma > 0 (CE marks have been observed).
 *   4. qdelay_avg > kcc_ecn_qdelay_thresh_us (queue buildup confirms
 *       congestion).
 *   5. Not in PROBE_BW mode (cwnd_gain remains at 2x matching BBR).
 *   6. During probing, backoff is graduated (scaled to BBR_UNIT/gain)
 *      rather than fully suppressed — severe ECN marks can still
 *      partially reduce cwnd during probe phases.
 *
 * When triggered, cwnd_gain is reduced by the configured backoff factor.
 * BBR has no ECN backoff; KCC adds this as an intelligent response to
 * confirmed congestion signals.
 *
 * The reduction is proportional to the configured backoff percentage
 * (default 20%).  This drains the queue earlier than loss-based
 * congestion signals, improving P99 latency on ECN-enabled paths.
 */
static void kcc_ecn_backoff(struct sock* sk, struct kcc_ext* ext)                /* ECN-aware proactive backoff */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                             /* KCC CA state */
    u32 ecn_backoff, factor;                                           /* backoff fraction and remaining scaling factor */

    /* When alone on path and ECN bypass is enabled (default),
     * skip ECN backoff — ECN marks on a single-flow path are
     * false positives from an over-sensitive AQM.  Bypassing
     * matches BBR's zero-ECN behavior and recovers the throughput
     * gap in single-flow scenarios.
     * When kcc_alone_bypass_ecn = 0, ECN backoff remains active
     * even when alone (conservative mode). */
    if (kcc->alone_on_path && kcc_alone_bypass_ecn_val) {
        return;
    }

    if (!kcc_ecn_enable_val || !ext) {                                 /* ECN disabled or no ext */
        return;                                                                     /* no-op */
    }

    if (ext->p_est >= kcc_kalman_converged_p_est_val) {               /* filter not converged */
        return;                                                                     /* wait for convergence */
    }

    if (ext->sample_cnt < (u32)kcc_kalman_min_samples_val) {               /* insufficient samples */
        return;                                                                     /* wait for min samples */
    }

    if (ext->ecn_ewma == 0) {                                                      /* no CE marks observed */
        return;                                                                     /* nothing to react to */
    }

    /* ECN backoff: reduces cwnd_gain when queue is confirmed (qdelay > threshold).
     * Suppressed during PROBE_BW entirely — cwnd_gain stays at 2x matching BBR.
     * During probing (pacing_gain > BBR_UNIT), backoff is graduated: the
     * suppression scales from 0% (at BBR_UNIT) to 80% (at 5/4 probe) to
     * ~65% (at 2.89x high_gain).  Severe ECN marks can still partially
     * reduce cwnd during probing — binary suppression was too conservative
     * and ignored real congestion signals during probe phases. */
    ecn_backoff = kcc_ecn_backoff_val;                                     /* single volatile read hoisted above branch */
    if (!ecn_backoff) {                                                            /* backoff disabled (zero) */
        return;
    }

    if (kcc->pacing_gain > BBR_UNIT) {                                             /* probing: graduated suppression */
        u32 ecn_scale = BBR_UNIT * BBR_UNIT / kcc->pacing_gain;                    /* BBR_UNIT^2 / gain: 1.0x @ cruise, 0.8x @ 1.25x, 0.35x @ 2.89x */
        ecn_backoff = ecn_backoff * ecn_scale >> BBR_SCALE;                          /* scale backoff by probe factor */
    }

    /* factor = BBR_UNIT - min(ecn_backoff, BBR_UNIT-1) */
    factor = BBR_UNIT - min_t(u32, ecn_backoff, BBR_UNIT);

    /* cwnd_gain: only reduce when queue is confirmed (qdelay > threshold)
     * AND we are NOT in PROBE_BW.  BBR never reduces cwnd_gain in PROBE_BW
     * — it is fixed at 2x.  During PROBE_BW, ECN signals affect pacing_gain
     * (via the probing-suppression gate above) but never the inflight ceiling. */
    if (kcc->mode != KCC_PROBE_BW &&
        ext->qdelay_avg > (u32)kcc_ecn_qdelay_thresh_us_val) {
        kcc->cwnd_gain = min_t(u32, kcc->cwnd_gain,
            max_t(u32, 1U,
                kcc->cwnd_gain * factor >> BBR_SCALE));
    }
}
/* ---- PROBE_BW Cycle Phase --------------------------------------------- */

/*
 * kcc_get_cycle_pacing_gain - Return the pacing gain for the current PROBE_BW
 * cycle phase, after applying optional queuing-delay/jitter decay.
 * @sk:  TCP socket.
 * @ext: extended state (provides qdelay_avg and jitter_ewma).
 *
 * Algorithm:
 *   1. Base gain = kcc_cycle_gain_table[cycle_idx & (cycle_len − 1)].
 *   2. If decay is enabled for this phase (bit in kcc_cycle_decay_mask set)
 *      AND base_gain > 1.0x (BBR_UNIT) AND ext is valid:
 *      a. Qdelay reduction (Cardwell et al. 2016):
 *         r_q = (qdelay_avg - qdelay_thresh) * BBR_UNIT / qdelay_scale
 *         base_gain -= min(r_q, max_red).
 *      b. Jitter reduction (additive):
 *         r_j = (jitter_ewma - jitter_thresh) * BBR_UNIT / jitter_scale
 *         base_gain -= min(r_j, max_red).
 *   3. Floor at BBR_UNIT (1.0x): the gain never drops below cruise level.
 *
 * The linear reduction model is:
 *   gain = max(1.0, probe_gain - alpha*(qdelay-T_q) - beta*(jitter-T_j))
 * where alpha = BBR_UNIT / qdelay_scale, beta = BBR_UNIT / jitter_scale.
 */
static u32 kcc_get_cycle_pacing_gain(const struct sock* sk,                   /* get decay-adjusted cycle gain */
    struct kcc_ext* ext)                                    /* extended state */
{
    const struct kcc* kcc = (const struct kcc*)inet_csk_ca(sk);              /* get const KCC CA state */
    u32 idx = (u32)kcc->cycle_idx & (kcc_probe_bw_cycle_len_val - 1);     /* len is power-of-two, mask is equivalent to % */
    u32 base_gain = kcc_cycle_gain_table[idx];                     /* read base gain for this phase */

    /*
     * Gain decay via queuing delay and jitter.
     * If ext is NULL (e.g., allocation failed at init), decay is skipped.
     * This is benign because the first real ACK processed after ext
     * becomes available will apply decay on the next phase advance.
     *
     * IMPORTANT: Floor decayed probing gains at BBR_UNIT (1.0x) so that
     * decay never pushes a probe below cruise.  Drain phases (gain < 1.0x
     * by design, e.g. 0.75x BBR_UNIT) are NOT decayed (the decay guard
     * `base_gain > BBR_UNIT` prevents entry for drain phases), so they
     * pass through unchanged at their native sub-1.0x value.
     */
    if (kcc_cycle_decay_enabled(idx) && base_gain > BBR_UNIT && ext) {        /* decay enabled + above cruise + ext valid */
        u32 max_red = base_gain - BBR_UNIT;                                   /* maximum reduction budget: down to 1.0x */
        u32 qthresh = (u32)kcc_qdelay_probe_thresh_us_val;              /* read qdelay threshold */
        u32 qscale = (u32)kcc_qdelay_probe_scale_us_val;                /* read qdelay scale divisor */
        u32 conv = (u32)kcc_kalman_converged_p_est_val;           /* cache converged threshold (single read for consistency) */

        /* Qdelay reduction: linearly proportional to excess qdelay.
         * Scale by Kalman confidence: when p_est > converged, the filter
         * is uncertain and qdelay_avg may be noise — reduce decay impact
         * proportionally.  When converged, full decay applies. */
        u32 conf_scale = BBR_UNIT;
        if (ext->p_est > conv) {
            u32 p_max = kcc_kalman_p_est_max_val;
            if (p_max > conv) {
                conf_scale = (u32)((u64)BBR_UNIT * (p_max - ext->p_est) / (p_max - conv));
            }
            else {
                conf_scale = 0;
            }
        }
        if (conf_scale > 0 && ext->qdelay_avg > qthresh) {                 /* qdelay exceeds threshold */
            u32 raw = min_t(u32, ((u64)(ext->qdelay_avg - qthresh) * BBR_UNIT) / qscale, max_red); /* qdelay reduction */
            u32 r = raw * conf_scale >> BBR_SCALE;                               /* scale by confidence */
            base_gain -= r; max_red -= r;                                     /* apply qdelay reduction, update budget */
        }
        /* Jitter reduction: any remaining max_red budget, also confidence-scaled */
        if (max_red > 0 && conf_scale > 0) {                                    /* remaining reduction budget */
            u32 jitter = ext->jitter_ewma;                                    /* read jitter EWMA */
            u32 jthresh = (u32)kcc_jitter_probe_thresh_us_val;           /* read jitter threshold */
            u32 jscale = (u32)kcc_jitter_probe_scale_us_val;             /* read jitter scale divisor */
            if (jitter > jthresh) {                                      /* jitter exceeds threshold */
                u32 jr = min_t(u32, ((u64)(jitter - jthresh) * BBR_UNIT) / jscale, max_red); /* jitter reduction */
                u32 jr_scaled = jr * conf_scale >> BBR_SCALE;                  /* scale by confidence */
                base_gain -= jr_scaled;                                         /* apply jitter reduction */
            }
        }
        /* Floor: decay should never push a probe below 1.0x cruise level.
         * Deliberate drain phases (base_gain < BBR_UNIT) never enter this
         * block (guard: base_gain > BBR_UNIT), so they pass through below. */
        base_gain = max_t(u32, base_gain, BBR_UNIT);
    }
    return base_gain;                                                         /* return gain (may be < BBR_UNIT for deliberate drain phases) */
}
/*
 * kcc_advance_cycle_phase - Transition to the next PROBE_BW cycle phase
 * (Cardwell et al. 2016).
 *
 * @sk:  TCP socket.
 *
 * Increments cycle_idx (wraps via mask), records the phase-start timestamp.
 * Note: pacing_gain is NOT set here — it is set by kcc_update_model() after
 * all phase-advance and mode-transition logic completes, matching kernel
 * BBR's division of labour.
 *
 * Corresponds to kernel BBR v5.4: bbr_advance_cycle_phase().
 * Identical behaviour: increment cycle_idx with power-of-two wrap, record
 * phase-start timestamp from tp->delivered_mstamp.  No KCC deviation.
 *
 * Type note: cycle_idx is a u32:8 bitfield in struct kcc.  The cycle length
 * is rounded up to a power of two (kcc_probe_bw_cycle_len_val), so the
 * modulo operation is implemented as a bitwise AND mask for efficiency.
 * Kernel BBR uses the same mask-based wrap for its 8-entry cycle.
 */
static void kcc_advance_cycle_phase(struct sock* sk)                           /* advance to next cycle phase */
{
    struct tcp_sock* tp = tcp_sk(sk);                                        /* get TCP socket state */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                          /* get KCC CA state */

    /* Wrap cycle_idx: (idx + 1) % cycle_len, using mask since len is power-of-two */
    kcc->cycle_idx = (kcc->cycle_idx + 1) & (kcc_probe_bw_cycle_len_val - 1); /* advance + wrap */
    kcc_set_cycle_mstamp(kcc, tp->delivered_mstamp);                          /* mark phase start time */

    /* pacing_gain is set by the mode-specific block in kcc_update_model()
     * after all phase-advance and mode-transition logic completes.
     * Setting it here is redundant (always overwritten by the caller)
     * and wastes hot-path cycles on every PROBE_BW phase advance. */
}
/*
 * kcc_update_dyn_probe_interval - Recompute the dynamic PROBE_RTT interval
 * based on Kalman error covariance p_est (Kalman 1960).
 * @ext: extended state (modified in-place).
 *
 * Mapping function (linear interpolation):
 *
 *   p_est <= p_est_floor                        -> interval = 2.5x dyn_max (75s, hyper-converged)
 *   p_est_floor < p_est <= converged_p_est      -> linear from 2.5x → 1.0x dyn_max
 *   converged_p_est < p_est < band*conv          -> linear from dyn_max → base
 *   p_est >= band * converged_p_est              -> interval = base (conservative)
 *
 *   interval = base + (max_jif - base) * (4*conv - p_est) / (3*conv)
 *
 * Rationale: when the Kalman filter has high confidence (low p_est),
 * the propagation delay estimate is stable, and PROBE_RTT can be
 * performed less frequently — reducing periodic throughput drops.
 * At extreme confidence (p_est near floor), the interval extends to
 * 2.5x dyn_max (75s), further reducing the performance penalty of
 * periodic min_rtt probing.  When p_est is high (low confidence), the
 * interval reverts to the base (conservative) value.
 */
static void kcc_update_dyn_probe_interval(struct kcc_ext* ext)                 /* recompute dynamic PROBE_RTT interval */
{
    u32 base = kcc_probe_rtt_base_jiffies;                         /* read base interval (jiffies) */
    u32 max_jif = kcc_probe_rtt_dyn_max_jiffies;                   /* read max dynamic interval (jiffies) */

    if (max_jif == 0 || !ext) {                            /* dynamic disabled OR no ext */
        return;                                                               /* skip: nothing to compute */
    }

    /* Guard: if dyn_max <= base, the linear interpolation produces nonsense.
     * Clamp dyn_max to at least base+1 to guarantee valid interpolation range. */
    if (max_jif <= base) { max_jif = (base < U32_MAX) ? base + 1 : base; }                              /* prevent underflow in (max - base) */
    {
        u32 conv = kcc_kalman_converged_p_est_val;                  /* convergence threshold p_est */
        u32 high = (u32)((u64)kcc_kalman_probe_band_mult_val * conv); /* u64 intermediate prevents overflow */
        u32 p = ext->p_est;                                                   /* current p_est value */
        u32 interval;                                                         /* computed dynamic interval */

        if (p <= conv) {                                                      /* fully converged: high confidence */
            u32 p_floor = kcc_kalman_p_est_floor_val;              /* Kalman floor (near-certainty) */
            if (p <= p_floor && p_floor < conv) {                              /* hyper-converged: extend to 2.5x */
                interval = max_jif + (u32)((u64)max_jif * KCC_DYN_PROBE_HYPER_NUM / KCC_DYN_PROBE_HYPER_DEN);             /* 30s → 75s at extreme confidence */
            }
            else if (p_floor < conv) {                                       /* converged but not floor */
                interval = max_jif + (u32)(((u64)max_jif * KCC_DYN_PROBE_HYPER_NUM / KCC_DYN_PROBE_HYPER_DEN) *
                    (conv - p) / (conv - p_floor));                              /* linear from 2.5x → 1x */
            }
            else {
                interval = max_jif;                                              /* use maximum interval */
            }
        }
        else if (p >= high) {                                               /* low confidence: at or above upper band */
            interval = base;                                                  /* use base (conservative) interval */
        }
        else {                                                              /* in transition band: linear interp */
            /* Linear interpolation: closer to conv -> closer to max_jif */
            if (high <= conv) {
                high = conv + 1;
            }

            interval = base + (u32)((u64)(max_jif - base) *                    /* base + (max-base)*(high-p)/(high-conv) */
                (high - p) / (high - conv));                           /* linear interpolation ratio */
        }

        ext->dyn_probe_rtt_interval_jiffies = interval;                       /* store dynamic interval in ext */
    }
}
/*
 * kcc_get_probe_rtt_interval - Determine the current PROBE_RTT interval
 * in jiffies, preferring the dynamic interval from the Kalman filter
 * when available (Kalman 1960).
 * @sk: TCP socket.
 *
 * Priority:
 *   1. If Kalman-converged && dyn_probe_rtt_interval_jiffies > 0:
 *      use dynamic interval (set by kcc_update_dyn_probe_interval).
 *   2. Fallback (classic BBRv1, Cardwell et al. 2016):
 *      base_interval = kcc_probe_rtt_base_jiffies.
 *      If min_rtt_us > long_rtt_threshold, halve (long paths need more
 *      frequent min_rtt re-verification).
 *      Cap at kcc_probe_rtt_max_jiffies.
 */
static u32 kcc_get_probe_rtt_interval(const struct sock* sk,                    /* get effective PROBE_RTT interval */
    struct kcc_ext* ext)                                                  /* ext from caller (avoids redundant kcc_ext_get) */
{
    const struct kcc* kcc = (const struct kcc*)inet_csk_ca(sk);               /* get const KCC CA state */

    /*
     * Dynamic interval: Kalman-converged -> wider probe gap.
     * Requires ext, valid x_est, sufficient sample count, and
     * a non-zero dynamic interval.
     */
    if (ext && ext->x_est &&
        ext->sample_cnt >= kcc_kalman_min_samples_val &&
        ext->dyn_probe_rtt_interval_jiffies > 0) {

        return ext->dyn_probe_rtt_interval_jiffies;
    }

    /* Classic BBRv1 fallback (Cardwell et al. 2016) */
    {
        u64 interval = kcc_probe_rtt_base_jiffies;                  /* start with base interval */

        /* BBRv1: long-RTT paths probe more frequently — interval
         * divided by the configured divisor (default 1 = no scaling
         * to match BBR, div=2 emulates BBRv1 halving) */
        if (kcc->min_rtt_us > (u32)kcc_probe_rtt_long_rtt_us_val) { /* RTT exceeds long-RTT threshold */
            interval = max_t(u64, interval / kcc_probe_rtt_long_interval_div_val, 1);       /* shrink by configured divisor */
        }

        return (u32)min_t(u64, interval, kcc_probe_rtt_max_jiffies); /* cap at max interval */
    }
}

/* ---- CWND Constraints ------------------------------------------------- */

/*
 * kcc_apply_cwnd_constraints - Apply cwnd_gain constraints.
 * Applies ECN-aware backoff (kcc_ecn_backoff) when ECN-CE marks
 * coincide with elevated queuing delay.  This is the only runtime
 * cwnd_gain reduction mechanism — BBR's 2x cwnd_gain in PROBE_BW
 * is proven optimal and is preserved in all non-ECN paths.
 * @sk:  TCP socket.
 * @ext: extended state (for ECN EWMA).
 */
static void kcc_apply_cwnd_constraints(struct sock* sk,                        /* apply cwnd_gain caps */
    struct kcc_ext* ext)                                   /* extended state */
{
    /* BBR maintains full 2.89x throughout STARTUP with no loss-based
     * reduction.  KCC matches this — the full_bw detector already
     * handles the STARTUP->DRAIN transition when the pipe is full
     * (3 rounds without 25% bw growth).  Loss events naturally
     * trigger LT BW sampling and eventual mode transitions.
     *
     * In PROBE_BW mode, reducing cwnd_gain below 2x is unsafe: inflight
     * shrinks after each probe cycle, creating a bandwidth floor that
     * prevents the next 1.25x probe from discovering more bandwidth.
     * BBR always uses 2x in PROBE_BW for this reason (Cardwell et al.
     * 2016).  ECN backoff (kcc_ecn_backoff) provides congestion-responsive
     * cwnd reduction that is compatible with the gain cycle.
     */

     /* ECN-aware backoff: reduces cwnd_gain when ECN-CE marks coincide
      * with elevated queuing delay.
      */
    kcc_ecn_backoff(sk, ext);                                                       /* ECN proactive backoff (RFC 3168) */
}
/*
 * kcc_inflight - Compute the target inflight in segments for the given
 * bandwidth, gain, and model RTT (Cardwell et al. 2016).
 *
 * @sk:   TCP socket.
 * @bw:   bandwidth in BW_UNIT.
 * @gain: cwnd gain in BBR_UNIT.  Signed 'int' matches kernel BBR's
 *        bbr_inflight() parameter type; gains may be negative in
 *        intermediate calculations (though KCC never passes negative
 *        cwnd_gain in practice).
 * @ext:  extended state (for Kalman RTT via kcc_bdp).
 *
 * inflight = kcc_quantization_budget(kcc_bdp(bw, gain, ext)).
 *
 * Corresponds to kernel BBR v5.4: bbr_inflight().
 * Identical delegation pattern — both are thin wrappers that compose
 * bdp() and quantization_budget().  No KCC deviation in this wrapper.
 * KCC's differences in the underlying kcc_bdp() and kcc_quantization_budget()
 * are documented in those functions' headers.
 *
 * Type note: u32 return (segments), matching kernel BBR's bbr_inflight().
 */
static u32 kcc_inflight(struct sock* sk, u32 bw, int gain,                      /* compute target inflight */
    struct kcc_ext* ext)                                                /* extended state */
{
    return kcc_quantization_budget(sk, kcc_bdp(sk, bw, gain, ext));              /* BDP + quantization headroom */
}
/*
 * kcc_packets_in_net_at_edt - Estimate inflight at the earliest departure time
 * (Cardwell et al. 2016).
 *
 * @sk:           TCP socket.
 * @inflight_now: current tcp_packets_in_flight().
 * @bw:           bandwidth estimate in BW_UNIT for computing delivered-at-EDT.
 *
 * Formula: delivered_at_edt = bw * (edt - now) in us >> BW_SCALE.
 *          inflight_at_edt = inflight_now + (pacing_gain > 1x ? tso_segs_goal : 0)
 *          return max(0, inflight_at_edt - delivered_at_edt).
 *
 * This is the BBR "is the pipe still full?" check, used for deciding
 * when to advance to the next PROBE_BW cycle phase.  When pacing_gain > 1x
 * (probing up), we add one TSO burst to estimate inflight at edt's send time.
 *
 * If EDT is within kcc_edt_near_now_ns, treat delivered_at_edt = 0
 * (the pipe won't drain at all before the next send window).
 *
 * Corresponds to kernel BBR v5.4: bbr_packets_in_net_at_edt().
 * Identical formula and logic.  Both implementations use a near-now
 * threshold (EDT within ~1 us of now ⇒ delivered = 0) to avoid
 * division-by-zero in the bandwidth computation.
 *
 * KCC deviation: the near-now threshold is configurable via sysctl
 * (kcc_edt_near_now_ns, default 1000 ns).  Kernel BBR uses a hardcoded
 * equivalent (~1 us implicit).  The configurable threshold allows
 * tuning for high-precision HW timestamps (where 1000 ns may be too
 * coarse) vs. low-precision clocks (where a larger threshold avoids
 * spurious zero returns).
 * BENEFIT: Tuning range across hardware with different clock precision.
 *
 * Type note: u32 inflight_now and u32 bw; the intermediate delta_us
 * is computed as u64 via div_u64().  u32 return matches kernel BBR's
 * bbr_packets_in_net_at_edt().
 */
static u32 kcc_packets_in_net_at_edt(struct sock* sk, u32 inflight_now, u32 bw)          /* estimate inflight at EDT */
{
    struct tcp_sock* tp = tcp_sk(sk);                                           /* get TCP socket state */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                             /* get KCC CA state */
    u64 now_ns = tp->tcp_clock_cache;                                            /* current time in ns */
    u64 edt_ns = max_t(u64, tp->tcp_wstamp_ns, now_ns);                          /* earliest departure time, >= now */
    u32 delivered;                                                               /* estimated delivered-at-EDT */
    u32 inflight_at_edt = inflight_now;                                          /* inflight at EDT, start with now */

    /* EDT within "near now" threshold -> pipe hasn't drained at all */
    if (edt_ns <= now_ns || (edt_ns - now_ns) <= (u64)kcc_edt_near_now_ns_val) {                  /* EDT is effectively "now" */
        delivered = 0;                                                            /* nothing drains by EDT */
    }
    else {                                                                      /* EDT is in the future */
        /* delivered = bw * (edt - now) >> BW_SCALE, matching BBR's pattern */
        u64 delta_us = div_u64(edt_ns - now_ns, NSEC_PER_USEC);
        u64 delivered64;
        delivered64 = ((u64)bw * delta_us) >> BW_SCALE;
        delivered = (u32)delivered64;
    }

    /* When probing above 1x gain, add one TSO burst to the estimate */
    if (kcc->pacing_gain > BBR_UNIT) {                                             /* probing up: add TSO burst */
        inflight_at_edt += kcc_tso_segs_goal(sk);                                    /* add one TSO burst worth of segs */
    }

    if (delivered >= inflight_at_edt) {                                            /* pipe will empty by EDT */
        return 0;                                                                  /* return 0 inflight at EDT */
    }

    return inflight_at_edt - delivered;                                            /* remaining inflight at EDT */
}
/* ---- Recovery Entry/Exit ---------------------------------------------- */

/*
 * kcc_set_cwnd_to_recover_or_restore - Handle cwnd adjustments on TCP
 * recovery entry and exit (Cardwell et al. 2016).
 *
 * @sk:       TCP socket.
 * @rs:       rate sample (for losses).
 * @acked:    bytes ACKed (u32, from rs->acked_sacked).
 * @new_cwnd: [out] computed cwnd value (u32 pointer).
 *
 * Returns true if in packet-conservation mode (recovery with cwnd pinned
 * to inflight + acked).
 *
 * On recovery entry:
 *   - Enable packet_conservation flag.
 *   - Set cwnd = inflight + acked (conservative; don't send more than in flight).
 * On recovery exit:
 *   - Restore cwnd to max(current, prior_cwnd).
 * If losses present: subtract losses from cwnd.
 *
 * Corresponds to kernel BBR v5.4: bbr_set_cwnd_to_recover_or_restore().
 * Identical logic.  Both implementations follow the same three-branch
 * pattern: entry (non-recovery → recovery), exit (recovery → non-recovery),
 * and loss-reduction path.  No KCC deviation.
 *
 * Type note: 'acked' is u32 (from rs->acked_sacked).  'new_cwnd' is u32*
 * (output parameter).  Returns bool (true = packet conservation active).
 * All types match kernel BBR's implementation exactly.
 */
static bool kcc_set_cwnd_to_recover_or_restore(                                 /* handle recovery cwnd transitions */
    struct sock* sk, const struct rate_sample* rs, u32 acked, u32* new_cwnd)     /* input params + output cwnd */
{
    struct tcp_sock* tp = tcp_sk(sk);                                             /* get TCP socket state */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                               /* get KCC CA state */
    u8 prev_state = kcc->prev_ca_state, state = inet_csk(sk)->icsk_ca_state;      /* previous and current CA states */
    u32 cwnd = tcp_snd_cwnd(tp);                                                       /* start with current cwnd */

    /* Loss: reduce cwnd by the number of lost segments (floor at 1) */
    if (rs->losses > 0) {                                                           /* losses present */
        if (cwnd > rs->losses) {
            cwnd -= rs->losses;
        }
        else {
            cwnd = 1;
        }
    }

    /* Recovery entry: transition from non-recovery -> recovery */
    if (state == TCP_CA_Recovery && prev_state != TCP_CA_Recovery) {                 /* entering recovery */
        kcc->packet_conservation = 1;                                                 /* enable packet conservation */
        kcc->next_rtt_delivered = tp->delivered;                                      /* start round now (match BBR, Cardwell et al. 2016) */
        cwnd = tcp_packets_in_flight(tp) + acked;                                     /* conservative cwnd = inflight + acked */
    }
    /* Recovery exit: transition from recovery -> non-recovery */
    else if (prev_state >= TCP_CA_Recovery && state < TCP_CA_Recovery) {              /* exiting recovery */
        cwnd = max_t(u32, cwnd, kcc->prior_cwnd);                                      /* restore to at least pre-recovery cwnd */
        kcc->packet_conservation = 0;                                                   /* disable packet conservation */
    }

    /* Update tracked previous CA state only on actual transition
     * to avoid unnecessary cache-line writes on every ACK. */
    if (state != prev_state) {
        kcc->prev_ca_state = state;
    }

    if (kcc->packet_conservation) {                                                       /* in packet conservation mode */
        *new_cwnd = max_t(u32, cwnd, tcp_packets_in_flight(tp) + acked);                  /* cwnd >= inflight + acked */
        return true;                                                                        /* return true: in conservation */
    }

    *new_cwnd = cwnd;                                                                       /* store computed cwnd */
    return false;                                                                             /* return false: not in conservation */
}
/* ---- CWND Setting (Cardwell et al. 2016) ----------------------------- */

/*
 * kcc_set_cwnd - Update the congestion window on each ACK.
 *
 * @sk:    TCP socket.
 * @rs:    rate sample.
 * @acked: segments ACKed (rs->acked_sacked, not bytes).  u32 matches
 *         the type of rs->acked_sacked in the kernel's rate_sample struct.
 * @bw:    bandwidth estimate in BW_UNIT.
 * @gain:  current cwnd_gain in BBR_UNIT.  'int' matches kernel BBR's
 *         convention for gain parameters (signed for delta arithmetic).
 * @ext:   extended state (for Kalman RTT in kcc_bdp, ACK-agg compensation).
 *
 * Algorithm (standard BBR per-ACK cwnd update, Cardwell et al. 2016):
 *   1. Skip if no data ACKed (acked_segs == 0).
 *   2. Handle recovery entry/exit via kcc_set_cwnd_to_recover_or_restore().
 *   3. Compute BDP target = quantization_budget(BDP(bw, gain, ext)).
 *   4. Apply inflight bounds:
 *      - STARTUP: lower-bound target at BDP * inflight_low_gain (no upper bound).
 *      - Other modes: clamp target between [lo, hi].
 *   5. Add ACK aggregation bonuses (standard + confidence-gated).
 *   6. CWND progression:
 *      - full_bw_reached: cwnd = min(cwnd + acked, target)  (convergent).
 *      - STARTUP:         cwnd = cwnd + acked               (exponential probe).
 *   7. Floor at cwnd_min_target.
 *   8. If PROBE_RTT just ended (probe_rtt_restored): restore prior_cwnd.
 *   9. Clamp to snd_cwnd_clamp.
 *  10. In PROBE_RTT mode: enforce cap at cwnd_min_target (min inflight).
 *
 * Corresponds to kernel BBR v5.4: bbr_set_cwnd().
 * The core logic (steps 1-4, 6-10) is identical to kernel BBR.
 *
 * KCC deviations (within the same overall algorithm):
 *   a) Inflight bound: KCC's inflight_low_gain defaults to 1.0x BDP vs.
 *      BBRv2's 1.25x.  See the long block comment at the inflight gain
 *      parameters (around line 1093) for the multi-flow overcommitment
 *      rationale and A/B test results.
 *   b) ACK aggregation compensation includes a confidence-gated second
 *      layer (kcc_agg_cwnd_compensation) that only activates at high
 *      confidence (agg_state >= CONFIRMED).  Kernel BBR unconditionally
 *      adds extra_acked to the cwnd target.  The confidence gate prevents
 *      over-inflation on ambiguous aggregation signals.
 *   c) BDP target uses kcc_bdp() which may return a Kalman-derived RTT
 *      estimate (see kcc_get_model_rtt).  Kernel BBR always uses the
 *      windowed min_rtt_us.
 *   BENEFIT of (a): Eliminates per-flow 1.25x aggregate overcommitment
 *   on multi-flow paths (each flow's inflated floor sums to N*0.25x BDP
 *   excess inflight).  BENEFIT of (b): Avoids cwnd overshoot on paths
 *   with ambiguous ACK-agg signals.  BENEFIT of (c): Tighter BDP through
 *   unbiased Kalman propagation-delay estimate.
 *
 * Type note: 'bw' is u32 (BW_UNIT), 'gain' is int (BBR_UNIT), 'acked'
 * is u32.  All match kernel BBR's bbr_set_cwnd() parameter types.
 */
static void kcc_set_cwnd(struct sock* sk, const struct rate_sample* rs,             /* update snd_cwnd */
    u32 acked, u32 bw, int gain,                                           /* input parameters */
    struct kcc_ext* ext)                                                   /* extended state */
{
    struct tcp_sock* tp = tcp_sk(sk);                                                /* get TCP socket state */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                  /* get KCC CA state */
    u32 cwnd = tcp_snd_cwnd(tp), target;                                                   /* current cwnd and target */

    if (unlikely(!acked)) {                                                                        /* no data ACKed: uncommon case */
        goto done;                                                                         /* skip to clamp enforcement */
    }

    if (kcc_set_cwnd_to_recover_or_restore(sk, rs, acked, &cwnd)) {                       /* handle recovery transitions */
        goto done;                                                                          /* packet-conservation mode */
    }

    /* Core BDP target (segments) — without quantization budget.
     * Quantization headroom (TSO/even-round/probe) is added AFTER
     * inflight bounds to avoid stripping it (see below). */
    target = kcc_bdp(sk, bw, gain, ext);

    /* Inflight bounds (BBR standard, Cardwell et al. 2016).
     * Apply bounds to the *raw* BDP before adding quantization and
     * ACK-aggregation bonuses.  This matches BBR behavior where
     * bbr_quantization_budget() adds headroom on top of the
     * already-bounded target_cwnd. */
    {
        bool bdp_ready = (kcc->min_rtt_us != KCC_MIN_RTT_UNINIT && bw > 0);   /* min_rtt_us > 0 implied by != UNINIT (UNINIT = ~0U) */
        if (likely(bdp_ready)) {
            u64 bdp = ((u64)bw * kcc->min_rtt_us) >> BW_SCALE;
            u32 lo = max_t(u32, TCP_INIT_CWND,
                mul_u64_u32_shr(bdp, kcc_inflight_low_gain_val, BBR_SCALE));
            if (kcc->mode == KCC_STARTUP) {
                target = max_t(u32, target, lo);
            }
            else {
                u32 hi_gain = max_t(u32, kcc->cwnd_gain, kcc_inflight_high_gain_val);  /* ≥ runtime cwnd_gain for safety */
                u32 hi = max_t(u32, lo,
                    mul_u64_u32_shr(bdp, hi_gain, BBR_SCALE));
                target = clamp(target, lo, hi);
            }
        }
        /* Now add quantization headroom on top of the bounded target.
         * This matches BBR's order: bdp() → bounded → quantization(). */
        target = kcc_quantization_budget(sk, target);
        /* Inflight bounds valid: add ACK aggregation bonuses */
        if (likely(bdp_ready)) {
            target += kcc_ack_aggregation_cwnd(sk, ext, bw);

            /* ACK aggregation compensation: confidence-gated second layer */
            if (kcc_agg_enable_val && ext && ext->agg_state >= KCC_AGG_CONFIRMED) {
                u32 agg_comp = kcc_agg_cwnd_compensation(sk, ext, ext->agg_extra_acked, ext->agg_confidence, bw);
                target = min_t(u32, target + agg_comp, tp->snd_cwnd_clamp);
            }
        }
    }

    /*
     * CWND progression policy:
     * - full_bw_reached: converge to target.
     *   cwnd = min(cwnd + acked, target)
     * - STARTUP (not yet full): exponential ramp.
     *   cwnd = cwnd + acked (unbounded above; inflight_bounds prevent overshoot)
     */
    if (likely(kcc_full_bw_reached(sk))) {                                                              /* pipe full: converge to target */
        cwnd = min(cwnd + acked, target);                                                          /* converge upward to target */
    }
    else if (unlikely(cwnd < target || tp->delivered < TCP_INIT_CWND)) {                                        /* STARTUP: growth needed */
        cwnd = cwnd + acked;                                                                               /* exponential ramp */
    }

    cwnd = max(cwnd, kcc_cwnd_min_target_val);                                         /* floor at min cwnd */

done:
    tcp_snd_cwnd_set(tp, min(cwnd, tp->snd_cwnd_clamp));

    /* PROBE_RTT enforcement: keep cwnd at minimum to drain the pipe */
    if (unlikely(kcc->mode == KCC_PROBE_RTT)) {
        tcp_snd_cwnd_set(tp, min(tcp_snd_cwnd(tp), kcc_cwnd_min_target_val));
    }
}
/* ---- Cycle Phase Check (Cardwell et al. 2016) ------------------------ */

/*
 * kcc_is_next_cycle_phase - Determine whether the current PROBE_BW phase
 * has completed (should advance to the next phase).
 *
 * @sk:  TCP socket.
 * @rs:  rate sample.
 * @ext: extended state (for Kalman drain-skip gate).
 *
 * Decision logic (from BBRv1, adapted for KCC gain decay):
 *   1. Phase must have lasted at least one min_rtt (is_full_length).
 *   2. Additional termination conditions depend on pacing_gain:
 *      - gain > 1.0x (probing up):   exit if loss occurred OR
 *                                     inflight_at_edt >= target_inflight.
 *      - gain < 1.0x (probing down): drain-to-target.
 *        Exit when is_full_length AND inflight_at_edt <=
 *        target at 1x gain, with a safety timeout after
 *        KCC_DRAIN_TARGET_MAX_RTTS (default 4) RTTs.
 *        Prevents premature drain exit that leaves
 *        residual inflight on multi-flow paths.
 *      - gain == 1.0x (cruise):      exit when is_full_length.
 *
 * Corresponds to kernel BBR v5.4: bbr_is_next_cycle_phase().
 * Kernel BBR uses the same three-branch dispatch (probe-up, probe-down,
 * cruise) with is_full_length as the base condition.
 *
 * KCC deviations:
 *   a) Probe-up (gain > 1.0x): KCC adds kcc_probe_bw_up_limit exits
 *      (app-limited or empty send queue) when the sysctl is enabled
 *      (default: off).  Kernel BBR exits on loss OR inflight target
 *      only.  The up-limit prevents futile probing when the application
 *      cannot fill the pipe.
 *      WHY: On app-limited connections, 1.25x probing beyond the app's
 *      send rate only inflates the queue without discovering bandwidth.
 *      The up-limit (disabled by default) is a safety valve, not a
 *      behavioural change.
 *   b) Probe-down (gain < 1.0x): KCC replaces BBR's OR-gate
 *      (is_full_length || drained) with an AND-gate + safety timeout
 *      (is_full_length && drained) || timeout.  This fixes BBRv1's
 *      premature drain exit on multi-flow paths (documented in detail
 *      in the inline block comment above the drain-to-target section).
 *      Kernel BBR drains exit when EITHER one RTT has elapsed OR
 *      inflight drops to BDP — whichever comes first.  On multi-flow
 *      paths, a single RTT is insufficient to drain the aggregate queue
 *      from N simultaneous 1.25x probes, causing residual inflight that
 *      accumulates cycle over cycle until loss.
 *      BENEFIT: Eliminates the BBRv1 multi-flow residual-inflight
 *      pathology that causes throughput oscillations of 550-1300 Mbps
 *      on 8-flow, 1 Gbps bottlenecks.
 *   c) Kalman drain-skip: when the Kalman filter is converged and
 *      qdelay_avg < drain_skip_qdelay_us (default 1 ms), the drain
 *      phase is skipped entirely — converted to an early cruise.
 *      Kernel BBR never skips DRAIN.
 *      WHY: The preceding probe did not create standing queue, so
 *      draining is unnecessary.  Skipping DRAIN converts 1/8 of the
 *      cycle to productive cruise bandwidth.
 *      BENEFIT: Converts unproductive drain phases to cruise on
 *      zero-queue paths, increasing throughput by ~1-2% in steady state.
 *
 * Type note: 'delta' is s64 (from tcp_stamp_us_delta) because the
 * timestamp delta may be negative if the monotonic clock was reordered
 * (rare but possible on some kernel/NIC combinations).  s64 delta
 * handles negative values gracefully in the comparison with min_rtt_us
 * (u32).  Returns bool, matching kernel BBR's bbr_is_next_cycle_phase().
 */
static bool kcc_is_next_cycle_phase(struct sock* sk,                              /* check if phase should advance */
    const struct rate_sample* rs,                                 /* rate sample */
    struct kcc_ext* ext)                                          /* extended state */
{
    struct tcp_sock* tp = tcp_sk(sk);                                              /* get TCP socket state */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                /* get KCC CA state */
    s64 delta = tcp_stamp_us_delta(tp->delivered_mstamp, kcc_get_cycle_mstamp(kcc));
    bool is_full_length;                                                               /* flag: at least one min_rtt elapsed */

    if (unlikely(kcc->min_rtt_us == KCC_MIN_RTT_UNINIT)) {
        return false;
    }

    /* is_full_length: strictly GREATER than one min_rtt (not >=).
     * BBR uses `>`, not `>=`: the phase must last strictly longer
     * than min_rtt to avoid advancing the cycle on a sample that
     * lands exactly at the min_rtt boundary (Cardwell et al. 2016). */
    is_full_length = delta > kcc->min_rtt_us;

    if (kcc->pacing_gain > BBR_UNIT) {                                                   /* probing up (> 1x): rare (~1/8 phases) */
        u32 etd_bw = kcc_bw(sk);                                                         /* active BW for delivered-at-EDT (match BBR using bbr_bw internally) */
        u32 max_bw = kcc_max_bw(sk);                                                     /* max BW for inflight target (match BBR: bw = bbr_max_bw(sk)) */
        u32 inet_edt = kcc_packets_in_net_at_edt(sk, rs->prior_in_flight, etd_bw);       /* hoist: single call for both paths */
        /*
         * Multi-condition probe-up exit:
         *
         * PROBE_UP (1.25x pacing) tries to discover untapped
         * bandwidth by raising inflight above BDP.  The
         * original BBRv1 exits when either loss occurs or
         * inflight reaches the 1.25x target.
         *
         * When kcc_probe_up_limit is enabled (default: off),
         * two additional conditions prevent futile probing:
         *
         *   rs->is_app_limited:
         *     The application is not writing fast enough to
         *     saturate the pipe.  Inflating inflight at 1.25x
         *     pacing cannot raise delivery rate beyond what
         *     the app provides — the probe cannot discover
         *     more bandwidth and only inflates the queue.
         *
         *   !tcp_send_head(sk):
         *     The sender has no data ready to transmit (send
         *     queue empty).  Inflight cannot grow regardless
         *     of pacing gain — continuing PROBE_UP serves no
         *     purpose and adds queuing delay.
         *
         * Disabled by default — on high-throughput multi-flow
         * VPS paths the standard BBRv1 probe-up heuristic
         * (loss or inflight target only) performs better.
         */
        return is_full_length &&                                                            /* RTT elapsed AND */
            (rs->losses ||                                                                 /* loss occurred OR */
                inet_edt >=                                                            /* inflight at EDT >= */
                kcc_inflight(sk, max_bw, kcc->pacing_gain, ext) ||                           /* target inflight at max_bw */
                (kcc_probe_bw_up_limit_val &&                                      /* up-phase limit enabled AND */
                    (rs->is_app_limited || !tcp_send_head(sk))));              /* app-limited or no data queued */
    }

    if (kcc->pacing_gain < BBR_UNIT) {                                                           /* probing down (< 1x): rare (~1/8 phases) */
        u32 etd_bw = kcc_bw(sk);                                                         /* active BW for delivered-at-EDT (match BBR) */
        u32 max_bw = kcc_max_bw(sk);                                                     /* max BW for inflight target (match BBR) */
        /* Kalman drain skip: when Kalman converged and qdelay near zero,
         * the probe-up phase did not create standing queue.  Skip the
         * drain phase — the path is already empty.  This saves 1/8 cycle
         * of unnecessary 0.75x pacing on truly clean paths.
         *
         * KCC optimisation; kernel BBR always drains. */
        if (ext && ext->p_est < kcc_kalman_converged_p_est_val &&
            ext->qdelay_avg < kcc_drain_skip_qdelay_us_val &&
            delta > kcc->min_rtt_us / KCC_DRAIN_SKIP_MIN_RTT_DIV) {
            return true;
        }
        /*
         * Drain-to-target: fix BBRv1's premature drain exit.
         *
         * BBRv1 drain mechanism:
         *
         *   PROBE_UP (1.25x) ─── creates standing queue ───►
         *   DRAIN (0.75x) ─── empties the queue ───►
         *   CRUISE (1.0x) ─── maintains BDP-level inflight
         *
         * In the original BBRv1 (Cardwell et al. 2016), drain
         * exits when EITHER one min_rtt has elapsed OR inflight
         * drops to BDP — whichever comes first:
         *
         *   return is_full_length || (inet_edt <= BDP_target);
         *
         * This is broken on multi-flow paths:
         *
         *   1. Eight flows share a 1 Gbps bottleneck.
         *   2. During PROBE_UP they collectively overshoot, building
         *      a queue that can exceed 1–2× BDP per flow.
         *   3. DRAIN at 0.75× pacing begins.  After 1 RTT, inflight
         *      has only partially drained — residual queue remains
         *      because the aggregate drain rate (8×0.75=6×) cannot
         *      clear 8×BDP of standing queue in a single RTT.
         *   4. The `is_full_length` branch fires.  Drain exits
         *      prematurely.  Residual inflight carries over into
         *      the next cycle.
         *   5. The next PROBE_UP starts from an already-elevated
         *      inflight baseline — overshoot happens earlier and
         *      harder — loss spikes — CWND collapses — throughput
         *      oscillates 550–1300 Mbps within 1–2 seconds.
         *
         *   Timeline (8-flow, 212 ms RTT, 1 Gbps shared bottleneck):
         *
         *     ┌──────────────────────────────────────────────────────┐
         *     │ PROBE_UP (1.25x)                                      │
         *     │   queue builds: inflight climbs to 2–3× BDP          │
         *     │   aggregate = 8×1.25 = 10× → 25% over line rate      │
         *     ├──────────────────────────────────────────────────────┤
         *     │ DRAIN (0.75x)                                         │
         *     │   t=0.00:  inflight = 2.5× BDP (post-probe peak)    │
         *     │   t=0.21:  inflight ≈ 1.8× BDP (1 RTT elapsed)     │
         *     │            ┌── BBRv1 OR-gate: is_full_length ✓ →     │
         *     │            │   EXITS DRAIN immediately                │
         *     │            │   residual = 0.8× BDP above target       │
         *     ├────────────┴─────────────────────────────────────────┤
         *     │ CRUISE (1.0x)  [residual inflight persists]          │
         *     │   queue never fully cleared                          │
         *     ├──────────────────────────────────────────────────────┤
         *     │ PROBE_UP (1.25x)  [next cycle]                       │
         *     │   starts from 1.8× BDP baseline → immediate loss     │
         *     │   CWND cut → throughput collapse                     │
         *     └──────────────────────────────────────────────────────┘
         *
         * KCC fix — drain-to-target (AND-gate):
         *
         *   return (is_full_length && drained) || safety_timeout;
         *
         *   ┌──────────────────────────────────────────────────────┐
         *   │ DRAIN (0.75x)                                         │
         *   │   t=0.00:  inflight = 2.5× BDP (post-probe peak)    │
         *   │   t=0.21:  inflight ≈ 1.8× BDP (1 RTT elapsed)     │
         *   │            is_full_length ✓  but drained ✗ →          │
         *   │            CONTINUES DRAINING                         │
         *   │   t=0.42:  inflight ≈ 1.3× BDP (2 RTTs)             │
         *   │            drained ✗ → CONTINUES                      │
         *   │   t=0.63:  inflight ≈ 1.0× BDP (3 RTTs)             │
         *   │            drained ✓ → EXITS to CRUISE                │
         *   │            queue genuinely empty                      │
         *   └──────────────────────────────────────────────────────┘
         *
         * Safety timeout (4× min_rtt, ~848 ms at 212 ms RTT):
         * prevents infinite drain on paths where inflight can
         * never reach BDP due to persistent cross-traffic.
         */


        {
            bool drained = kcc_packets_in_net_at_edt(sk, rs->prior_in_flight, etd_bw) <=
                kcc_inflight(sk, max_bw, BBR_UNIT, ext);
            return (is_full_length && drained) ||
                delta > kcc->min_rtt_us * KCC_DRAIN_TARGET_MAX_RTTS;
        }
    }

    /* Cruise (== 1x): advance after one min_rtt */
    return is_full_length;                                                                                /* single condition: full RTT */
}
/*
 * kcc_update_cycle_phase - Check and advance the PROBE_BW cycle phase.
 *
 * @sk:  TCP socket.
 * @rs:  rate sample.
 * @ext: extended state (passed through to kcc_is_next_cycle_phase).
 *
 * Only acts in PROBE_BW mode.  Calls kcc_advance_cycle_phase() when
 * kcc_is_next_cycle_phase() returns true.
 *
 * Corresponds to kernel BBR v5.4: bbr_update_cycle_phase().
 * Identical dispatch pattern: mode check → phase check → advance.
 * No KCC deviation.
 */
static void kcc_update_cycle_phase(struct sock* sk,                                   /* check + advance PROBE_BW phase */
    const struct rate_sample* rs,                                      /* rate sample */
    struct kcc_ext* ext)                                               /* extended state */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                    /* get KCC CA state */
    if (likely(kcc->mode == KCC_PROBE_BW) && kcc_is_next_cycle_phase(sk, rs, ext)) {             /* PROBE_BW + time to advance */
        kcc_advance_cycle_phase(sk);                                                  /* advance to next phase */
    }
}
/*
 * kcc_reset_mode - Transition to STARTUP or PROBE_BW after DRAIN completes
 * or after exiting PROBE_RTT (Cardwell et al. 2016).
 *
 * @sk: TCP socket.
 *
 * If full_bw_reached:
 *   -> PROBE_BW, with randomized initial cycle phase (decorrelation via random
 *     offset to prevent phase synchronization across flows sharing a
 *     bottleneck).
 * Else:
 *   -> STARTUP (pipe is not yet fully characterized).
 *
 * Corresponds to kernel BBR v5.4: bbr_reset_mode().
 * Kernel BBR has the same two-branch decision: full_bw_reached → PROBE_BW
 * with randomized cycle_idx, else → STARTUP.  Identical logic.
 *
 * KCC note: kernel BBR's bbr_reset_mode() is the single reset function
 * handling both the STARTUP→PROBE_BW and PROBE_RTT→PROBE_BW transitions.
 * KCC follows the same pattern; there are no separate kcc_reset_startup_mode
 * or kcc_reset_probe_bw_mode functions.  Those names would correspond to
 * the two branches within this function.
 *
 * KCC deviation: when ext is NULL (allocation failure), KCC falls back to
 * cycle_idx = 0 and default gains instead of randomising.  Kernel BBR assumes
 * the in-sock CA slot is always available.  The fallback ensures KCC can
 * operate degraded but without crashing when kzalloc fails.
 * BENEFIT: Graceful degradation on memory pressure.
 *
 * Type note: cycle_idx is a u32:8 bitfield.  random_below() returns u32;
 * the subtraction and mask keep the result within the valid phase range
 * [0, cycle_len-1].  The cycle_len is guaranteed to be a power of two.
 */
static void kcc_reset_mode(struct sock* sk)                                            /* transition from DRAIN/ProbeRTT */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                     /* get KCC CA state */
    struct kcc_ext* ext = kcc_ext_get(sk);                                                 /* get ext (with UAF guard) */

    if (!kcc_full_bw_reached(sk)) {                                                         /* pipe not yet full */
        kcc->mode = KCC_STARTUP;                                                              /* re-enter STARTUP */
    }
    else {                                                                                    /* pipe full: enter PROBE_BW */
        kcc->mode = KCC_PROBE_BW;                                                                 /* set PROBE_BW mode */
        /* Random start phase: cycle_idx = len - 1 - rand(range).
         * Spreads flows across phases to reduce correlation. */
        if (ext) {
            kcc->cycle_idx = kcc_probe_bw_cycle_len_val - 1 -                               /* start near end */
                kcc_random_below(kcc_probe_bw_cycle_rand_val);                            /* randomized offset */
            /* BBR calls bbr_advance_cycle_phase() after setting cycle_idx,
             * which (a) increments cycle_idx, (b) records cycle_mstamp, and
             * (c) sets pacing_gain.  Match this behavior exactly so the
             * first PROBE_BW phase has a valid timestamp for is_full_length. */
            kcc_advance_cycle_phase(sk);                                                   /* flip to next phase + set mstamp */
        }
        else {
            kcc->cycle_idx = 0;                                                              /* fallback: phase 0 */
            kcc->pacing_gain = BBR_UNIT;                                                       /* cruise at 1.0x */
            kcc->cwnd_gain = kcc_cwnd_gain_val;                                                  /* baseline cwnd gain */
            kcc_set_cycle_mstamp(kcc, tcp_sk(sk)->delivered_mstamp);                             /* seed phase timestamp */
        }
    }
}
/* ---- LT BW (Long-Term Bandwidth) ---------------------------------------- */

/*
 * kcc_reset_lt_bw_sampling_interval - Reset the interval counters for a
 * new LT BW sampling episode.
 *
 * @sk: TCP socket.
 *
 * Records the current delivered, lost, and timestamp for the start of
 * a new sampling interval.  The lt_rtt_cnt is reset to 0.
 *
 * This is a KCC extension with no direct kernel BBR equivalent.
 * Kernel BBR has no LT-BW sampling — it uses only the sliding-window
 * max-bw filter (struct minmax).  LT-BW is a KCC addition for
 * policer-detection on rate-limited paths (VPN shapers, ISP throttling).
 *
 * Type note: lt_last_stamp stores jiffies as millisecond timestamps
 * (delivered_mstamp in us ÷ 1000).  u32 is sufficient because the
 * maximum LT interval is bounded (default 48 RTTs, typically < 5 s).
 */
static void kcc_reset_lt_bw_sampling_interval(struct sock* sk)                    /* start new LT BW interval */
{
    struct tcp_sock* tp = tcp_sk(sk);                                              /* get TCP socket state */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                /* get KCC CA state */

    kcc->lt_last_stamp = div_u64(tp->delivered_mstamp, (u32)USEC_PER_MSEC);        /* record interval start (ms) */
    kcc->lt_last_delivered = tp->delivered;                                          /* record delivered at interval start */
    kcc->lt_last_lost = tp->lost;                                                      /* record lost at interval start */
    kcc->lt_rtt_cnt = 0;                                                                 /* reset RTT counter */
}
/*
 * kcc_reset_lt_bw_sampling - Fully disable LT BW sampling and clear
 * the LT estimate and use flag.
 *
 * @sk: TCP socket.
 *
 * Clears lt_bw, lt_use_bw, lt_is_sampling, and resets the interval
 * counters.  After this call, LT-BW state returns to the inactive
 * (not sampling) state.
 *
 * This is a KCC extension with no direct kernel BBR equivalent.
 * See kcc_reset_lt_bw_sampling_interval for the BBR comparison.
 *
 * Type note: all three flags (lt_bw, lt_use_bw, lt_is_sampling) are
 * bitfields in struct kcc.  lt_bw is u32 for bandwidth in BW_UNIT.
 * The interval counters (lt_last_delivered, lt_last_lost, lt_last_stamp)
 * are reset by the delegated kcc_reset_lt_bw_sampling_interval().
 */
static void kcc_reset_lt_bw_sampling(struct sock* sk)                               /* disable LT BW + clear state */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                  /* get KCC CA state */

    kcc->lt_bw = 0;                                                                     /* clear LT BW estimate */
    kcc->lt_use_bw = 0;                                                                   /* disable LT BW pacing */
    kcc->lt_is_sampling = 0;                                                               /* disable sampling flag */
    kcc_reset_lt_bw_sampling_interval(sk);                                                   /* reset interval counters */
}
/*
 * kcc_lt_bw_interval_done - Process a completed LT BW interval.
 *
 * @sk: TCP socket.
 * @bw: bandwidth estimate for the just-completed interval (BW_UNIT, u64
 *      from the caller's 64-bit arithmetic in kcc_lt_bw_sampling).
 *
 * Consistency check: the new estimate bw must be within a certain
 * tolerance of the existing lt_bw (if lt_bw > 0):
 *   - Relative: |bw - lt_bw| <= ratio * lt_bw  (default ratio = 1/8).
 *   - Absolute: byte-rate diff <= kcc_lt_bw_diff (default 500 bytes/s).
 *
 * If consistent: update lt_bw to the exponential moving average of
 * (bw + lt_bw) / 2, set lt_use_bw = 1, reset pacing to 1.0x.
 * If inconsistent: replace lt_bw with the new estimate, restart interval.
 *
 * This is a KCC extension with no direct kernel BBR equivalent.
 * Kernel BBR's loss handling reduces cwnd but does not maintain a
 * separate long-term bandwidth estimate for policed paths.
 *
 * KCC deviation from BBR practice:
 * BBR practice: bbr uses minmax_running_max() for bandwidth estimation,
 * which collapses during sustained loss as old high-bw samples expire.
 * WHY KCC diverges: On policed paths (VPN shapers, ISP throttling, WiFi),
 * the max-bw window drops below the true policed rate after a loss
 * episode, forcing STARTUP re-probing that hits the policer again.
 * LT-BW preserves the policed rate as a stable lower bound, preventing
 * this oscillation.  The consistency check (relative + absolute tolerance)
 * prevents LT-BW from locking onto noise; the queue guard (qdelay_avg or
 * instant qdelay check) ensures LT-BW does not activate during KCC's own
 * congestion (self-inflicted loss), only during policer-limited loss.
 * BENEFIT: Stable send rate through policed loss periods, faster recovery
 * without re-probing into the policer ceiling.  Queue guard prevents
 * death-spiral where LT-BW locks in a low rate caused by KCC's own queue.
 *
 * Type note: 'bw' is u64 because the caller (kcc_lt_bw_sampling) computes
 * interval bandwidth via 64-bit division.  Internally, the function
 * clamps to u32 when storing to kcc->lt_bw.  The consistency comparison
 * uses u64 arithmetic to avoid overflow when multiplying lt_bw by the
 * ratio numerator.  The byte-rate conversion (kcc_rate_bytes_per_sec)
 * also produces u64, preserving precision through the diff comparison.
 */
static void kcc_lt_bw_interval_done(struct sock* sk, u64 bw)                        /* process completed LT BW interval */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                  /* get KCC CA state */
    u64 diff;                                                                          /* absolute bandwidth difference (u64: may exceed 2^32) */

    if (kcc->lt_bw) {                                                                    /* existing LT BW estimate */
        diff = (bw > kcc->lt_bw) ? bw - kcc->lt_bw : kcc->lt_bw - bw;                   /* absolute difference */
        /* Check both relative tolerance (BBR_UNIT ratio) and absolute diff */
        if (((u64)diff * BBR_UNIT <= (u64)kcc_lt_bw_ratio_val * kcc->lt_bw) ||  /* within relative tolerance */
            (kcc_rate_bytes_per_sec(sk, (u64)diff, BBR_UNIT) <=                            /* OR within absolute tolerance */
                (u64)kcc_lt_bw_diff_val)) {                                         /* bytes/s diff check */
            /* Consistent: smooth update using EMA */
                    {
                        u32 en = kcc_lt_bw_ema_num_val;
                        u32 ed = kcc_lt_bw_ema_den_val;
                        kcc->lt_bw = (u32)min_t(u64,
                            (bw * en + (u64)kcc->lt_bw * (ed - en)) / ed, U32_MAX);
                    }
                    /*
                     * Only activate LT BW when the loss is from a bandwidth
                     * policer, not from self-inflicted congestion.  When
                     * qdelay_avg is elevated, the queue is building from
                     * KCC's own over-sending — capping the bandwidth here
                     * would lock the flow into a death spiral where low
                     * bandwidth prevents recovery from the very congestion
                     * that triggered LT BW.
                     *
                     * Two congestion signals (either is sufficient):
                     * 1. qdelay_avg > ecn_qdelay_thresh: persistent EWMA queue (needs ext)
                     * 2. srtt - min_rtt > inst_thresh: instantaneous burst queue
                     *    (works without ext, protects against allocation failure)
                     */
                    {
                        struct kcc_ext* ext = kcc_ext_get(sk);
                        u32 qthresh = (u32)kcc_ecn_qdelay_thresh_us_val;
                        u32 ithresh = (u32)kcc_lt_bw_inst_qdelay_thresh_us_val;
                        struct tcp_sock* tp = tcp_sk(sk);
                        u32 srtt_us = tp->srtt_us >> 3;                 /* SRTT in µs (kernel stores as 8x) */

                        if (ext && ext->qdelay_avg > qthresh) {          /* persistent queue → congestion */
                            kcc_reset_lt_bw_sampling(sk);                 /* abort LT BW activation */
                            return;
                        }
                        if (srtt_us > kcc->min_rtt_us + ithresh) {       /* burst queue > threshold → congestion */
                            kcc_reset_lt_bw_sampling(sk);                 /* abort: works even without ext */
                            return;
                        }
                    }
                    kcc->lt_use_bw = 1;                                                                /* enable LT BW for pacing */
                    kcc->pacing_gain = BBR_UNIT;                                                         /* reset to cruise gain */
                    kcc->lt_rtt_cnt = 0;                                                                  /* reset RTT counter */
                    return;                                                                                /* done: consistent update */
        }
    }

    /* First estimate or inconsistent: start fresh */
    kcc->lt_bw = (u32)min_t(u64, bw, U32_MAX);                                                      /* store new LT BW estimate, clamp to u32 */
    kcc_reset_lt_bw_sampling_interval(sk);                                                           /* restart interval */
}
/*
 * kcc_lt_bw_sampling - Main LT BW sampling state machine, called per-ACK.
 *
 * @sk: TCP socket.
 * @rs: rate sample.
 *
 * Two modes:
 * A) lt_use_bw == 1 (LT BW active):
 *    - Count round trips.  After lt_bw_max_rtts rounds in PROBE_BW,
 *      reset LT BW and mode (periodically re-probe the path).
 *
 * B) lt_use_bw == 0 (not active):
 *    - Sampling triggers on first loss event.
 *    - Collects up to 4 * lt_intvl_min_rtts rounds of data.
 *    - After at least lt_intvl_min_rtts rounds, if loss ratio >= threshold,
 *      compute bw = delivered * BW_UNIT / interval_time and call
 *      kcc_lt_bw_interval_done().
 *
 * Exits: on app_limited, after timeout (4* min_rtts), or on bad timestamp.
 *
 * This is a KCC extension with no direct kernel BBR equivalent.
 * Kernel BBR handles loss by reducing cwnd (in bbr_set_cwnd) but does not
 * maintain a separate long-term bandwidth state machine.  KCC's LT-BW
 * is similar in spirit to BBRv3's LT-BW proposal (preserve a bandwidth
 * floor through lossy intervals), implemented independently.
 *
 * Type note: interval duration t_us is u64 (not u32) because KCC's LT
 * interval parameters are runtime sysctls, not compile-time constants.
 * Kernel BBR's LT timeout (4 * bbr_lt_intvl_min_rtts = 16 RTTs) fits in
 * u32 even after ms→us conversion.  KCC allows up to 32 * 127 = 4064 RTTs,
 * which at 10 s/RTT would overflow u32; hence u64 and div64_u64().
 * See the inline block comment at the bandwidth computation for the
 * detailed overflow analysis.
 */
static void kcc_lt_bw_sampling(struct sock* sk, const struct rate_sample* rs)        /* LT BW sampling state machine */
{
    struct tcp_sock* tp = tcp_sk(sk);                                                   /* get TCP socket state */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                     /* get KCC CA state */
    u32 lost, delivered;                                                                   /* interval lost and delivered */
    u64 bw;                                                                                 /* computed interval bandwidth */
    u64 t_us;                                                                               /* interval duration (us), u64 guards against overflow with extreme sysctl configs */

    /* ---- Mode A: LT BW already active ---- */
    if (kcc->lt_use_bw) {                                                                       /* LT BW is active */
        /* Periodically re-probe: reset after lt_bw_max_rtts rounds in PROBE_BW */
        if (kcc->mode == KCC_PROBE_BW && kcc->round_start) {                                     /* PROBE_BW + new round */
            u32 cnt = kcc->lt_rtt_cnt + 1;
            if (cnt >= KCC_LT_RTT_CNT_MAX) {
                cnt = KCC_LT_RTT_CNT_MAX;
            }

            kcc->lt_rtt_cnt = cnt;
            if (cnt >= kcc_lt_bw_max_rtts_val) {
                kcc_reset_lt_bw_sampling(sk);                                                            /* clear LT BW state */
                kcc_reset_mode(sk);                                                                        /* restart from PROBE_BW */
            }
        }
        return;                                                                                        /* done: LT BW active path */
    }

    /* ---- Mode B: Not active; trigger on loss ---- */
    if (!kcc->lt_is_sampling) {                                                                         /* not yet sampling */
        if (!rs->losses) {                                                                                /* no loss this ACK */
            return;                                                                                        /* wait for first loss */
        }

        kcc_reset_lt_bw_sampling_interval(sk);                                                             /* start sampling episode */
        kcc->lt_is_sampling = 1;                                                                            /* set sampling flag */
    }

    /* Abort if app-limited (cannot trust bw estimate) */
    if (rs->is_app_limited) {                                                                               /* app-limited ACK */
        kcc_reset_lt_bw_sampling(sk);                                                                        /* abort sampling */
        return; /* early return */
    }

    /* Count RTT boundaries */
    if (kcc->round_start) {                                                                                   /* round boundary */
        u32 cnt = kcc->lt_rtt_cnt + 1;
        if (cnt >= KCC_LT_RTT_CNT_MAX) {
            cnt = KCC_LT_RTT_CNT_MAX;
        }
        kcc->lt_rtt_cnt = cnt;                                                  /* increment and saturate RTT counter */
    }

    /* Too few RTTs yet; wait for lt_intvl_min_rtts rounds */
    if (kcc->lt_rtt_cnt < (u32)kcc_lt_intvl_min_rtts_val) {                                         /* insufficient rounds */
        return; /* early return */
    }

    /* Timeout: max_mult * min_rtts without enough loss -> abort */
    {
        u32 lt_to = kcc_lt_intvl_max_mult_val * kcc_lt_intvl_min_rtts_val;
        if (kcc->lt_rtt_cnt >= lt_to) { /* exceeded max interval */
            kcc_reset_lt_bw_sampling(sk);                                                                            /* abort: timeout */
            return;
        }
    }

    /* ---- Compute loss ratio over the interval ---- */
    lost = tp->lost - kcc->lt_last_lost;                                                                           /* interval lost pkts */
    delivered = tp->delivered - kcc->lt_last_delivered;                                                              /* interval delivered pkts */

    /* Require some delivered data AND loss ratio >= threshold (BBR_UNIT).
     * Comparison uses scaled integer math: compare (lost*256) < (threshold*delivered).
     * Parenthesize << — C precedence makes << bind looser than <. */
    if (!delivered || ((u64)lost << BBR_SCALE) < ((u64)kcc_lt_loss_thresh_val * delivered)) {
        return; /* early return */
    }

    /* ---- Compute bandwidth over the interval ---- */
    /*
     * BBR uses u32 for the interval because its LT timeout is a compile-time
     * constant (4 * bbr_lt_intvl_min_rtts = 16 RTTs), so t *= USEC_PER_MSEC
     * never overflows u32.
     *
     * KCC's kcc_lt_intvl_max_mult and kcc_lt_intvl_min_rtts are runtime sysctl
     * parameters (capped at 32 and 127 respectively). Worst case:
     *   4064 RTTs * 10 s/RTT * 1000 = 40,640,000 ms * 1000 > U32_MAX
     * Therefore the interval must use u64 to avoid overflow, and the
     * divisor must use u64 div64_u64() instead of BBR's faster u32 do_div().
     */
    t_us = div_u64(tp->delivered_mstamp, USEC_PER_MSEC) - kcc->lt_last_stamp;
    if ((s64)t_us < 1) {                                                                                                    /* interval less than 1 ms, wait for more data */
        return; /* early return */
    }

    t_us *= USEC_PER_MSEC;                                                                                                     /* convert ms -> us, u64 prevents overflow with extreme sysctl configs */
    bw = (u64)delivered * BW_UNIT;                                                                                            /* delivered in BW_UNIT scale */
    bw = div64_u64(bw, t_us);                                                                                                     /* bw = delivered * BW_UNIT / interval_us, u64 divisor required because t_us may exceed u32 range */
    kcc_lt_bw_interval_done(sk, bw);                                                                                            /* process interval result */
}
/* ---- Bandwidth Update (Cardwell et al. 2016) ------------------------- */

/*
 * kcc_update_bw - Update the sliding-window max bandwidth estimate.
 *
 * @sk:  TCP socket.
 * @rs:  rate sample from the current ACK.
 * @ext: extended state (unused here, for consistent API with kcc_update_model).
 *
 * Per-ACK updates:
 *   1. Validate the rate sample (interval > 0, delivered >= 0).
 *   2. Detect round boundaries: when prior_delivered >= next_rtt_delivered,
 *      a new round starts.  On round start:
 *        - Increment rtt_cnt.
 *        - Reset packet_conservation (exit recovery mode at round boundary).
 *   3. Run LT BW sampling state machine.
 *   4. Compute instantaneous bw = delivered * BW_UNIT / interval_us.
 *   5. Feed into the sliding-window max via minmax_running_max().
 *      Window length = kcc_bw_rt_cycle_len (default 10 rounds).
 *      If app-limited: only update if new bw >= existing max (BBR rule).
 *
 * Corresponds to kernel BBR v5.4: bbr_update_bw().
 * Identical validation, round-boundary detection, BW computation, and
 * minmax update logic.  The BW formula (delivered * BW_UNIT / interval_us)
 * and the app-limited exclusion rule match kernel BBR exactly.
 *
 * KCC deviations:
 *   a) LT BW sampling: interleaved at step 3 (kernel BBR has no LT-BW).
 *   b) Global Kalman BDP floor: Step 2 (inside the function body) enforces
 *      a defensive BW floor from the global KF estimate during STARTUP,
 *      preventing the first few dirty samples from overwriting the KF
 *      injection.  Kernel BBR has no cross-connection bandwidth sharing.
 *      See kcc_kf_get_init_bw() for the full Global KF mechanism.
 *   BENEFIT of (b): Prevents KF-injected initial bandwidth from being
 *   immediately overwritten by pre-fill low-rate samples.
 *
 * Type note: 'bw' is computed as u64 via div_u64() before being cast to
 * u32 for minmax_running_max().  The u64 intermediate is necessary because
 * delivered * BW_UNIT can exceed U32_MAX on high-speed paths with large
 * ACK coalescing.  The ext parameter is declared for API consistency but
 * unused in this function — it is consumed by kcc_lt_bw_sampling which is
 * called within.
 */
static void kcc_update_bw(struct sock* sk, const struct rate_sample* rs,               /* update sliding-window max BW */
    struct kcc_ext* ext)                                                      /* extended state (unused) */
{
    struct tcp_sock* tp = tcp_sk(sk);                                                     /* get TCP socket state */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                       /* get KCC CA state */
    u64 bw;                                                                                  /* instantaneous bandwidth */

    /* Match BBR, Cardwell et al. 2016: reset round_start at top, before any early
     * return.  This ensures stale round_start=1 from a previous ACK is cleared
     * even if this rate sample is invalid and we return early. */
    kcc->round_start = 0;                                                                        /* reset round_start flag */

    /* Validate rate sample — match BBR exactly (bbr_update_bw:765).
     * BBR rejects when delivered < 0 (negative) OR interval_us <= 0 (zero or negative).
     * Zero delivered IS valid: the ACK carries no new data but may still cross a
     * round boundary.  Skipping zero-delivered ACKs would delay round counting and
     * full_bw detection.  interval_us <= 0 catches both zero and negative timestamps. */
    if (unlikely(rs->delivered < 0 || rs->interval_us <= 0)) {                                    /* invalid sample: negative delivery or non-positive interval */
        return;
    }

    /* Round boundary detection (BBR round counting).
     * Uses BBR's !before() unsigned comparison (Cardwell et al. 2016):
     *   next_rtt_delivered is initialized to 0 in kcc_init() — matching
     *   stock BBR — so the very first data ACK always starts round 1
     *   regardless of handshake segment delivery.
     *
     *   unsigned before(a,b) is equivalent to (s32)(a - b) < 0 for
     *   monotonic sequence numbers.  prior_delivered is s32, delivered
     *   wraps at 2^31 (4B packets).  The unsigned comparison used by
     *   BBR matches Linux's monotonic sequence number arithmetic and
     *   is safe because next_rtt_delivered is always ahead of or equal
     *   to prior_delivered within a single round.
     *
     * On round boundary: advance rtt_cnt, update next_rtt_delivered
     * baseline, mark round_start, and exit packet_conservation. */
    if (!before(rs->prior_delivered, kcc->next_rtt_delivered)) {                             /* prior_delivered >= next_rtt_delivered */
        kcc->next_rtt_delivered = tp->delivered;                                                      /* update next round baseline */
        kcc->rtt_cnt++;                                                                                /* increment round counter */
        kcc->round_start = 1;                                                                           /* mark round start */
        kcc->packet_conservation = 0;                                                                    /* exit packet conservation at round boundary */
    }
    /* LT BW sampling (must run before bw update to use raw rs) */
    kcc_lt_bw_sampling(sk, rs);                                                                          /* run LT BW state machine */

    /* Instantaneous bandwidth: delivered segments * BW_UNIT / interval_us */
    bw = div_u64((u64)rs->delivered * BW_UNIT, rs->interval_us);                                           /* compute instant bandwidth */

    /* Step 2 (Global Kalman BDP): defensive floor during STARTUP.
     * The first few RTTs of a new connection produce low-bandwidth
     * delivery-rate samples because the pipe hasn't filled yet.  Without
     * a floor, these dirty samples overwrite the global Kalman injection.
     * The floor is init_bw (fair × discount × BBR_UNIT ÷ high_gain), not
     * the raw KF estimate — using the raw value would inflate max_bw and
     * cause overdosing when STARTUP high_gain is applied. */
    if (kcc_kf_enable_val && atomic_read(&kcc_kf_active) && kcc->mode == KCC_STARTUP) { /* KF active + STARTUP mode */
        u64 kf_bw = (u64)atomic64_read(&kcc_kf_x);                               /* read global fair-share estimate */
        u64 floor_bw = kf_bw * (u64)kcc_kf_discount_num_val / (u64)kcc_kf_discount_den_val; /* apply safety discount */

        floor_bw = floor_bw * BBR_UNIT / (u64)kcc_high_gain_val;                 /* gain-compensate: ÷ high_gain */
        if (bw < floor_bw) {                                                     /* raw bw below defensive floor */
            bw = floor_bw;                                                       /* enforce global-Kalman floor */
        }
    }

    /* BBR rule: if not app-limited OR new bw >= existing max, update sliding max.
     * App-limited samples are excluded unless they record a new peak. */
    if (!rs->is_app_limited || bw >= kcc_max_bw(sk)) {                                                       /* acceptable sample */
        minmax_running_max(&kcc->bw, kcc_bw_rt_cycle_len_val, kcc->rtt_cnt, (u32)bw);                       /* feed to sliding max */
    }
}
/* ---- Full BW Reached Detection (Cardwell et al. 2016) ---------------- */

/*
 * kcc_check_full_bw_reached - Detect when the pipe has been filled to
 * capacity (STARTUP -> DRAIN transition criterion).
 *
 * @sk: TCP socket.
 * @rs: rate sample.
 *
 * Algorithm (BBRv1, Cardwell et al. 2016):
 *   - Skip if already full, not at round_start, or app-limited.
 *   - Compute bw_thresh = full_bw * full_bw_threshold (default 1.25x).
 *   - If max_bw >= bw_thresh -> bandwidth still growing; update full_bw.
 *   - Else: increment full_bw_cnt.
 *   - When full_bw_cnt >= full_bw_cnt_val (default 3 rounds without growth):
 *     set full_bw_reached = 1.
 *
 * Corresponds to kernel BBR v5.4: bbr_check_full_bw_reached().
 * Identical growth-detection logic (bw_thresh comparison, full_bw_cnt
 * saturation at 3, threshold at 1.25x).  The same three-skip conditions
 * (already full, not round_start, app-limited) in the same order.
 *
 * KCC deviation: Global Kalman BDP STARTUP protection guard inserted
 * BEFORE the standard skip-check (lines after the function start).  When
 * the global KF is active and the connection is in early STARTUP (rtt_cnt
 * < KCC_KF_STARTUP_PROTECT_RTTS = 8), if max_bw is still at or above the
 * KF floor, the function resets full_bw_cnt = 0 and returns early, keeping
 * STARTUP alive.  This prevents premature full_bw detection on the first
 * few app-limited ACKs that BBR would tolerate but KCC accelerates via
 * the Global KF bandwidth injection.
 * BBR practice: bbr has no cross-connection bandwidth injection and thus
 * no equivalent guard.  App-limited early ACKs gradually build full_bw_cnt
 * and can trigger premature full_bw_reached on idle-start connections.
 * WHY: Without this guard, the Global KF-injected initial bandwidth is
 * interpreted as "pipe already full" by the standard growth check, causing
 * KCC to exit STARTUP before the connection has actually filled the pipe.
 * BENEFIT: Preserves STARTUP probing for the full pipe-filling duration
 * even when the global KF provides a high initial bandwidth estimate.
 *
 * Type note: full_bw_cnt is a u32:2 bitfield (max 3), saturating at 3.
 * full_bw_thresh_val is stored as a precomputed (num/den * BBR_UNIT) value.
 * bw_thresh is u32, computed as (full_bw * thresh_val) >> BBR_SCALE.
 * All types and operations match kernel BBR's bbr_check_full_bw_reached().
 */
static void kcc_check_full_bw_reached(struct sock* sk,                               /* check if pipe is full */
    const struct rate_sample* rs)                                  /* rate sample */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                   /* get KCC CA state */
    u32 bw_thresh;                                                                       /* bandwidth growth threshold */

    /* KCC+ Global Kalman BDP guard: must run BEFORE the is_app_limited
     * early-return below.  The first few ACKs during iperf3 (control
     * message exchange) are app-limited — without this guard, the
     * full_bw_cnt would tick each round and force STARTUP→DRAIN before
     * any bulk data is sent.  Resetting full_bw_cnt here prevents the
     * premature mode transition.
     *
     * Guard: if max_bw is still at or above the KF floor (init_bw),
     * the pipe has not yet been filled — reset full_bw_cnt to zero
     * and keep STARTUP running. */
    if (kcc->round_start && kcc->rtt_cnt < KCC_KF_STARTUP_PROTECT_RTTS &&          /* early startup rounds */
        kcc_kf_enable_val && atomic_read(&kcc_kf_active) && kcc->mode == KCC_STARTUP) { /* KF active + STARTUP */
        u64 kf_bw = (u64)atomic64_read(&kcc_kf_x);                               /* read global fair-share estimate */
        if (kf_bw > 0) {                                                         /* valid estimate */
            u64 init_floor = kf_bw * (u64)kcc_kf_discount_num_val / (u64)kcc_kf_discount_den_val; /* apply discount */
            init_floor = init_floor * BBR_UNIT / (u64)kcc_high_gain_val;         /* gain-compensate */
            if (kcc_max_bw(sk) >= (u32)init_floor) {                             /* max_bw still at/above KF floor */
                kcc->full_bw = kcc_max_bw(sk);                                   /* record current peak */
                kcc->full_bw_cnt = 0;                                            /* reset stagnation counter */
                return;                                                          /* keep STARTUP: pipe not yet full */
            }
        }
    }

    if (likely(kcc_full_bw_reached(sk) || !kcc->round_start || rs->is_app_limited)) {             /* skip if already full or invalid */
        return; /* early return */
    }

    /* bw_thresh = full_bw * full_bw_thresh_val >> BBR_SCALE (125% default) */
    bw_thresh = (u32)(((u64)kcc->full_bw * kcc_full_bw_thresh_val) >> BBR_SCALE);         /* compute growth threshold */
    {
        u32 cur_max = kcc_max_bw(sk);                                                     /* hoist: single minmax read */
        if (cur_max >= bw_thresh) {                                                       /* bandwidth still growing */
            kcc->full_bw = cur_max;                                                         /* record new peak bandwidth */
            kcc->full_bw_cnt = 0;                                                                    /* reset stagnation counter */
            return; /* early return */
        }
    }

    /* No growth this round: increment stagnation counter */
    kcc->full_bw_cnt = min_t(u32, kcc->full_bw_cnt + 1, KCC_BITFIELD_2BIT_MAX);                     /* saturate at 2-bit field max */
    /* After configured rounds without growth: declare pipe full */
    kcc->full_bw_reached = (kcc->full_bw_cnt >= kcc_full_bw_cnt_val);                      /* set full_bw_reached if threshold met */
}
/* ---- Drain Check (Cardwell et al. 2016) ------------------------------ */

/*
 * kcc_check_drain - Handle STARTUP -> DRAIN transition and DRAIN -> PROBE_BW exit.
 *
 * @sk:  TCP socket.
 * @rs:  rate sample.
 * @ext: extended state (used for inflight target computation via kcc_inflight).
 *
 * STARTUP -> DRAIN: full_bw_reached triggers mode change to DRAIN,
 *   followed by setting ssthresh to the BDP at 1.0x gain.
 *
 * DRAIN -> PROBE_BW: when estimated inflight at EDT <= target inflight
 *   at 1.0x gain, the queue has been drained; reset mode to PROBE_BW.
 *
 * Corresponds to kernel BBR v5.4: bbr_check_drain().
 * Identical state transitions and drain exit condition (inflight_at_edt <=
 * target_inflight at BBR_UNIT).  Kernel BBR's ssthresh setting follows the
 * same pattern: ssthresh = bbr_inflight(sk, max_bw, BBR_UNIT).
 *
 * KCC deviation: explicit qdelay_avg reset when transitioning STARTUP→DRAIN.
 * Resetting qdelay_avg to 0 prevents the queue accumulated during STARTUP
 * from carrying over into PROBE_BW and triggering unjustified ECN backoff
 * or gain decay.
 * BBR practice: bbr has no qdelay_avg (it's a KCC Kalman extension), so no
 * reset is needed.
 * BENEFIT: Clean state on DRAIN entry; no false congestion signals in the
 * following PROBE_BW cycle from the STARTUP queue that is being drained.
 *
 * Type note: DRAIN exit uses kcc_packets_in_net_at_edt() which returns u32;
 * the comparison with kcc_inflight() (u32) is straightforward and matches
 * kernel BBR's type usage.
 */
static void kcc_check_drain(struct sock* sk, const struct rate_sample* rs,            /* handle drain transitions */
    struct kcc_ext* ext)                                                   /* extended state */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                     /* get KCC CA state */

    if (unlikely(kcc->mode == KCC_STARTUP && kcc_full_bw_reached(sk))) {                              /* STARTUP -> DRAIN */
        kcc->mode = KCC_DRAIN;                                                                /* transition: pipe full -> drain excess */
        tcp_sk(sk)->snd_ssthresh = kcc_inflight(sk, kcc_max_bw(sk),                            /* set ssthresh = BDP at 1x */
            BBR_UNIT, ext);                                                  /* cwnd_gain = BBR_UNIT, ext state */

        /* Reset qdelay_avg to prevent the STARTUP queue buildup from
         * persisting into PROBE_BW and triggering unjustified cwnd reduction.
         * The DRAIN phase ensures the actual queue is emptied before PROBE_BW. */
        if (ext) {
            ext->qdelay_avg = 0;
        }
    }

    if (unlikely(kcc->mode == KCC_DRAIN)) {                                                              /* in DRAIN mode */
        u32 max_bw = kcc_max_bw(sk);                                                            /* hoist max BW for drain */
        if (kcc_packets_in_net_at_edt(sk, tcp_packets_in_flight(tcp_sk(sk)),
            max_bw) <=                                                               /* inflight at EDT <= */
            kcc_inflight(sk, max_bw, BBR_UNIT, ext)) {                                         /* target at 1.0x */
            kcc_reset_mode(sk);                                                                           /* queue drained -> enter PROBE_BW */
        }
    }
}
/* ---- Kalman Recalibration Heuristic ---- */

/*
 * kcc_kalman_needs_recalibration - Detect Kalman filter confidence loss.
 * @ext: extended state (may be NULL).
 *
 * Returns true when the Kalman filter's error covariance p_est exceeds
 * kcc_recal_p_est_thresh, signalling that the filter has diverged from
 * the true path RTT (e.g., after a BGP reroute, LEO handover, or
 * sustained noise-model mismatch).
 *
 * In steady-state operation on a stable path, p_est converges to
 * p_est_floor (~4--10), orders of magnitude below the threshold.  A
 * rising p_est is an early warning signal — the filter's internal model
 * no longer explains the observations, typically because the path has
 * materially changed.
 *
 * Called from the PROBE_RTT entry guard in kcc_update_model.  When a
 * recalibration is triggered, a single traditional PROBE_RTT drain
 * provides a fresh min_rtt_us sample that restores the filter baseline,
 * after which the Kalman re-converges and decoupling resumes.
 *
 * Never returns true if Kalman has not converged — an unconverged
 * filter has high p_est by design; let min_rtt_us handle BDP.
 */
static bool kcc_kalman_needs_recalibration(const struct kcc_ext* ext)      /* check Kalman health */
{
    if (!ext || !ext->x_est) {
        return false;
    }

    if (ext->sample_cnt < (u32)kcc_kalman_min_samples_val) {
        return false;                                                       /* not yet converged */
    }

    if (ext->p_est > kcc_recal_p_est_thresh_val) {
        return true;                                                        /* p_est diverged: need recal */
    }

    return false;
}

/* ---- PROBE_RTT Done Check (Cardwell et al. 2016) --------------------- */

/*
 * kcc_check_probe_rtt_done - Check whether PROBE_RTT should end.
 *
 * @sk: TCP socket.
 *
 * Conditions for exit:
 *   - probe_rtt_done_stamp is set (we entered stay period).
 *   - tcp_jiffies32 > probe_rtt_done_stamp (stay duration elapsed).
 *
 * On exit:
 *   - Update min_rtt_stamp (fresh sample obtained).
 *   - Restore cwnd to at least prior_cwnd.
 *   - Set probe_rtt_restored flag (cwnd restore happens in kcc_set_cwnd).
 *   - Reset to PROBE_BW mode.
 *   - Override pacing rate with high_gain to quickly refill pipe.
 *
 * Corresponds to kernel BBR v5.4: bbr_check_probe_rtt_done().
 * Identical exit conditions (jiffies past done_stamp) and exit actions
 * (min_rtt_stamp update, cwnd restore, reset_mode, high_gain override).
 * No KCC deviation.
 *
 * Type note: probe_rtt_done_stamp is u32 jiffies.  after() macro for
 * unsigned jiffies comparison handles wraparound correctly.  The pacing
 * override uses kcc_bw_to_pacing_rate (u64) capped by sk_max_pacing_rate.
 */
static void kcc_check_probe_rtt_done(struct sock* sk)                               /* check if PROBE_RTT can exit */
{
    struct tcp_sock* tp = tcp_sk(sk);                                                  /* get TCP socket state */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                     /* get KCC CA state */

    if (unlikely(!kcc->probe_rtt_done_stamp ||                                                      /* stay stamp not set OR */
        !after(tcp_jiffies32, kcc->probe_rtt_done_stamp))) {                                  /* stay duration not elapsed */
        return;                                                                                 /* not yet time to exit */
    }

    kcc->min_rtt_stamp = tcp_jiffies32;                                                        /* fresh min_rtt obtained */
    tp->snd_cwnd = max(tp->snd_cwnd, kcc->prior_cwnd);                        /* restore cwnd to pre-probe level */

    kcc_reset_mode(sk);                                                                            /* transition to PROBE_BW */
}
/* ---- Kalman Filter (Kalman 1960) -------------------------------------
 *
 * Single-state Kalman filter for propagation-delay estimation.
 *
 * State-space model:
 *   State equation:  x[k] = x[k-1] + w   (random walk; w ~ N(0, Q))
 *   Observation:     z[k] = x[k] + v     (v ~ N(0, R))
 *
 * where:
 *   x = true propagation delay (us * kalman_scale)
 *   z = observed RTT = rtt_us * kalman_scale
 *   Q = process noise covariance (adaptive)
 *   R = measurement noise covariance (adaptive)
 *
 * Standard Kalman filter equations (predict + update):
 *   Predict:
 *     x_pred = x_est          (identity state transition)
 *     p_pred = p_est + Q      (predicted error covariance)
 *
 *   Update (upon receiving z):
 *     innovation = z - x_pred
 *     K = p_pred / (p_pred + R)           (Kalman gain)
 *     x_est = x_pred + K * innovation     (state update)
 *     p_est = (1 - K) * p_pred            (covariance update)
 *
 * The above scalar K is implemented as gain_num/gain_den:
 *     K = gain_num / gain_den = p_pred / (p_pred + R)
 *
 * Enhancements over standard Kalman:
 *   - Adaptive Q: scaled by min_rtt_us/1000 to account for path length.
 *   - Adaptive R: increased when jitter exceeds threshold.
 *   - Q-boost: resets p_est to p_est_init when innovation is very large
 *     (path change recovery).
 *   - Outlier gating: rejects samples where |innovation| exceeds a
 *     dynamic threshold, preventing pollution of x_est by transient spikes.
 *   - Consecutive rejection guard: force-accepts after max consecutive
 *     rejections to prevent self-reinforcing lockout.
 *   - BBR-S covariance-matched noise estimation (Welch & Bishop 2006):
 *     online Q and R estimation via innovation and Kalman gain statistics.
 *   - EWMA smoothing of qdelay (queuing delay) and jitter for use in
 *     gain decay, cwnd reduction, and PROBE_RTT interval adjustment.
 */
static void kcc_kalman_update(struct sock* sk, u32 rtt_us,                           /* Kalman filter update */
    struct kcc_ext* ext)                                                /* extended state */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                    /* get KCC CA state */
    u64 z;                                                                                /* measurement in scaled units */
    u32 gain_num, gain_den, q, r, p_pred;                                                  /* Kalman gain + noise + predicted cov */
    u32 rtt_max;                                                                            /* dynamic RTT sample rejection ceiling */

    if (unlikely(!ext)) {                                                                         /* no ext: skip update */
        return;
    }

    /*
     * Zero RTT sample: at line rates ≥ 25 Gbps, the serialization
     * time of a 1500-byte packet falls below 1 µs.  The kernel's
     * microsecond-granularity RTT clock can legitimately read 0 µs
     * when consecutive ACKs land within the same microsecond tick.
     *
     * Packet serialization time (1500 bytes, 10^9 bps = 1 Gbps):
     *
     *   Rate (Gbps)   Serialization (ns)
     *   ───────────   ─────────────────
     *   10            1200
     *   25             480
     *   40             300
     *   50             240
     *   100            120
     *   200             60
     *   400             30
     *   800             15
     *   1200            10
     *
     * At 25 Gbps, a 1500-byte frame serializes in 480 ns — well under
     * 1 µs.  Consecutive ACKs (data → ACK → next ACK) can thus land
     * in the same microsecond, producing a legitimate rtt_us = 0
     * measurement.  Discarding such samples distorts state estimation
     * on high-speed paths.
     *
     * We floor rtt_us at 1 µs — the smallest representable meaningful
     * delay — to bound distortion while preserving measurement
     * existence (the round-trip occurred and produced a valid ACK).
     */
    rtt_us = max_t(u32, rtt_us, 1U);

    /* Measurement z = rtt_us * kalman_scale (fixed-point scale) */
    z = (u64)rtt_us << kcc_kalman_scale_shift_val;                                             /* convert to scaled units (shift = ilog2(scale)) */

    /* ---- Cold start: initialize state directly from first sample ---- */
    if (unlikely(ext->sample_cnt == 0)) {                                                           /* no prior state estimate */
        ext->x_est = (u32)min_t(u64, z, U32_MAX);                                                        /* set x_est = first measurement, clamp u64->u32 */

        /* Initialize Kalman covariance: p_est = max(p_est_init, rtt_us / divisor) */
        ext->p_est = max_t(u32, kcc_kalman_p_est_init_val,
            rtt_us / max_t(u32, kcc_kalman_p_est_init_rtt_div_val, 1U));
        ext->qdelay_avg = 0;                                                                              /* no qdelay on first sample */
        ext->jitter_ewma = max_t(u32, rtt_us / KCC_JITTER_SEED_DIV, 1U);                                                       /* seed jitter from rtt_us/4, floor 1us */
        ext->sample_cnt = 1;                                                                                /* first sample accepted */
        return;                                                                                             /* cold start complete */
    }

    /*
     * One-time cold-start overshoot correction: if the first sample was
     * inflated by queue delay (x_est > min_rtt_us * scale), cap it now.
     * The bootstrapped min_rtt_us (from 3WHS) provides a realistic upper
     * bound for the propagation delay.  After this single correction, the
     * directional update (negative innovations only) prevents queue-noise
     * drift, and qboost freely tracks genuine path changes upward.
     */
    if (unlikely(ext->sample_cnt == 1) &&
        kcc->min_rtt_us != KCC_MIN_RTT_UNINIT) {
        u32 ceiling = (u32)min_t(u64,
            (u64)kcc->min_rtt_us << kcc_kalman_scale_shift_val, U32_MAX);
        if (ext->x_est > ceiling) {
            ext->x_est = ceiling;
        }
    }

    /* Discard excessively large RTT samples.  The threshold is dynamic:
     * kcc_rtt_sample_max_us_val is the configured floor (default 500ms),
     * but for paths with baseline RTT > half the floor (e.g. GEO satellite
     * with 600ms RTT), the effective threshold lifts to min_rtt_us * 2
     * so Kalman can still converge.  Cold start has already returned
     * above, so sample_cnt > 0 is guaranteed here. */
    rtt_max = kcc_rtt_sample_max_us_val;                                       /* configured floor: 500ms default */
    if (kcc->min_rtt_us != KCC_MIN_RTT_UNINIT && kcc->min_rtt_us > 0 &&                                      /* valid min_rtt */
        kcc->min_rtt_us * (u64)kcc_kalman_rtt_dyn_mult_val > rtt_max) {              /* u64 prevents overflow */
        rtt_max = (u32)min_t(u64, (u64)kcc->min_rtt_us * (u64)kcc_kalman_rtt_dyn_mult_val, U32_MAX);           /* lift to dynamic threshold */
    }

    if (unlikely(rtt_us > rtt_max)) {                                                           /* cap exceeded: discard */
        return;
    }

    /* ---- Adaptive Q: scaled by min_rtt_us / divisor (Kalman 1960) ---- */
    /*
     * Base Q is multiplied by max(q_min_factor, min_rtt_us / kcc_kalman_q_rtt_div)
     * Q = Q_base * max(q_min_factor, min_rtt_us / q_rtt_div)
     * Capped at Q_base * q_scale_cap to prevent runaway on very long paths.
     * The scaling accounts for the fact that random-walk variance on a
     * longer path is proportionally larger.
     */
    {
        u64 q64;                                                                                           /* 64-bit Q accumulator */
        u32 q_rtt_factor = (kcc->min_rtt_us != KCC_MIN_RTT_UNINIT) ? kcc->min_rtt_us / (u32)kcc_kalman_q_rtt_div_val : 0;
        q64 = (u64)kcc_kalman_q_val;                                                                  /* base process noise Q (clamped cache) */
        q64 *= (u64)max_t(u32, kcc_kalman_q_min_factor_val,                                         /* multiply by max(factor, */
            q_rtt_factor);                  /* min_rtt_us / configured divisor */
        q64 = min_t(u64, q64,                                                                                     /* cap 1: Q_base * q_scale_cap */
            (u64)kcc_kalman_q_scale_cap_val * (u64)kcc_kalman_q_val);                  /* cap = cap_val * Q_base */
        q64 = min_t(u64, q64, (u64)kcc_kalman_q_max_val);                                         /* cap 2: absolute Q upper bound */
        q = (u32)q64;                                                                                               /* store adaptive Q */
    }
    /* ---- Adaptive R: increased by jitter (Kalman 1960) ---- */
    /*
     * R = base_R + min(max(0, jitter - jr_thresh) * base_R / jr_scale,
     *                  base_R * kcc_kalman_r_max_boost)
     * Boost capped to prevent gain freeze on paths with persistent high
     * Measurement noise increases when RTT jitter increases,
     * causing the Kalman gain to decrease (the filter trusts
     * measurements less when jitter is high).
     */
    {
        u32 base_r = max_t(u32, kcc_kalman_r_val, 1U);                                                                     /* base measurement noise R (clamped cache) */
        u32 jitter = ext->jitter_ewma;                                                                              /* current jitter EWMA */
        u32 jr_thresh = (u32)kcc_jitter_r_thresh_us_val;                                                       /* jitter threshold for R increase */
        u32 jr_scale = (u32)kcc_jitter_r_scale_val;                                                             /* scaling divisor for R increase */

        if (jitter > (u32)jr_thresh) {                                                                                   /* jitter exceeds threshold */
            u64 r_boost = (u64)(jitter - jr_thresh) * (u64)base_r / (u64)jr_scale;                       /* linear R boost in u64 to avoid truncation */
            u64 r_cap = (u64)base_r * (u32)kcc_kalman_r_max_boost_val;                                 /* cap: base_R * max_boost (u64 safe) */
            r = base_r + (u32)min_t(u64, r_boost, r_cap);                                                                       /* capped R = base + min(boost, cap) */
        }
        else {                                                                                                           /* jitter within threshold */
            r = base_r;                                                                                                      /* use base R unchanged */
        }
    }

    /*
     * Combine heuristic Q/R with covariance-matched estimates (BBR-S method).
     * Mode 0 (disabled): use heuristic Q/R only.
     * Mode 1 (max):       use max(heuristic, matched) — conservative, default.
     * Mode 2 (avg):       use (heuristic + matched) / 2 — balanced.
     * The matched estimates (q_est, r_est) are initialized at module-init
     * to the base Q/R values and updated by covariance matching on each
     * accepted Kalman sample (Welch & Bishop 2006).
     */
    {
        int mode = kcc_kalman_noise_mode_val;                                        /* noise combination mode */
        if (mode > 0 && ext->q_est > 0 && ext->r_est > 0) {                                    /* enabled and estimates valid */
            if (mode == 1) {                                                                    /* mode 1: max(heuristic, matched) */
                q = max_t(u32, q, ext->q_est);                                                  /* Q = max(adaptive_Q, matched_Q) */
                r = max_t(u32, r, ext->r_est);                                                  /* R = max(adaptive_R, matched_R) */
            }
            else if (mode == 2) {                                                                 /* mode 2: weighted average */
                {
                    u32 na = kcc_kalman_noise_avg_num_val;
                    u32 da = kcc_kalman_noise_avg_den_val;
                    q = (u32)(((u64)q * (da - na) + (u64)ext->q_est * na) / da);
                    r = (u32)(((u64)r * (da - na) + (u64)ext->r_est * na) / da);
                }
            }
        }
    }

    /* ACK aggregation noise adjustment with hysteresis (BBRplus-inspired).
     * R increases instantly when confidence warrants (fast response to
     * detected aggregation).  R recovers gradually at round boundaries
     * by decaying agg_r_scaled toward min_mult, preventing Kalman
     * filter oscillation from abrupt R changes.
     *
     * NOTE: Confidence evaluation runs BEFORE kcc_update_model in kcc_main,
     * so agg_confidence read here reflects the current ACK's evaluation.
     * The Kalman R scale now updates synchronously with detection. */
    {
        u16 conf = ext->agg_confidence;                                                    /* ext is non-NULL after early return guard at top of function */
        u32 r_min = kcc_agg_r_multiplier_min_val;
        u32 r_max = kcc_agg_r_multiplier_max_val;

        if (conf >= (u16)kcc_agg_thresh_suspected_val) {
            /* R scales up instantly with confidence: linear interpolation between min and max */
            u32 target_mult = r_min + (r_max - r_min) * (u32)conf / (u32)kcc_agg_confidence_max_val;
            /* Clamp the linear interpolation result to [r_min, r_max] to prevent
             * overshoot when conf exceeds confidence_max (possible when factor_weight
             * is configured independently from confidence_max). */
            ext->agg_r_scaled = clamp_t(u32, target_mult, r_min, r_max);  /* instant increase, clamped */
        }
        else if (kcc->round_start && ext->agg_r_scaled > r_min) {
            /* Round boundary: decay R toward baseline at hysteresis rate.
             * Default 75% retained = 25% decay per RTT, ~4 RTTs to baseline. */
            u32 hysteresis_pct = kcc_agg_r_hysteresis_val;
            ext->agg_r_scaled = (ext->agg_r_scaled * hysteresis_pct + r_min * (KCC_PCT_BASE - hysteresis_pct)) / KCC_PCT_BASE;
        }

        /* Apply the persisting R scale to current measurement noise */
        if (ext->agg_r_scaled > BBR_UNIT) {
            r = (u32)min_t(u64, ((u64)r * ext->agg_r_scaled) >> BBR_SCALE, U32_MAX);  /* u64 intermediate, clamp to u32 */
        }
    }

    /* ---- Core Kalman update (Kalman 1960) ---- */
    {
        /* innovation = z - x_est (in scaled units; may be negative) */
        s64 innovation = (s64)z - (s64)ext->x_est;                                                                           /* innovation = measurement - state */
        u64 abs_innov = innovation >= 0 ? (u64)innovation : (u64)(-(innovation + 1)) + 1;                                       /* absolute value of innovation */
        u64 corr_abs = 0;                                                                                                      /* correction magnitude (hoisted for covariance matching) */
        bool qboost_fired = false;                                                                                                  /* Q-boost flag: skip outlier gate */

        /*
         * Q-boost: if innovation exceeds the boost threshold AND the
         * filter has converged (p_est <= converged_p_est) AND the
         * cooldown has expired, reset p_est to p_est_init.  This
         * causes the Kalman gain to spike, allowing the filter to
         * rapidly track a genuine path change (e.g., route change,
         * mobility event).
         *
         * Confidence gate: only fire qboost when the filter is
         * confident.  When p_est is large (filter uncertain —
         * recent qboost or startup), large innovations are treated
         * as noise/jitter and pass through normal outlier rejection.
         *
         * Cooldown: after each qboost, block further qboost for
         * kcc_kalman_qboost_cdwn_val (default 15) samples.  On noisy paths
         * with RTT jitter > qboost_thresh (4ms), the confidence gate
         * alone is insufficient — p_est drops below converged after
         * ~1-2 samples (high Kalman gain from the p_est reset), and
         * the next jitter spike would re-trigger qboost.  The
         * cooldown forces a minimum observation window between
         * qboost events (~1 RTT at 200ms, 5 samples), ensuring
         * only persistent path changes (not transient jitter) can
         * defeat the directional-update guard.
         *
         * These gates together prevent the Kalman filter from
         * degrading to a simple low-pass on lossy paths:
         *   1. p_est stays converged (no perpetual reset),
         *   2. Outlier rejection gates transient spikes,
         *   3. x_est tracks propagation delay, not queued RTT,
         *   4. Flow fairness is preserved (all flows converge to
         *      similar BDP through shared min_rtt).
         * The reset happens BEFORE outlier rejection so that large
         * innovations from path changes can enter the filter.
         */
        if (unlikely(ext->qboost_cdwn == 0 &&
            ext->p_est <= kcc_kalman_converged_p_est_val &&
            abs_innov > kcc_kalman_q_boost_thresh_val)) {                                        /* cooldown expired + converged + large innovation: path change */
            ext->p_est = kcc_kalman_p_est_init_val;                                                                      /* reset covariance for high gain */
            ext->qboost_cdwn = (u8)kcc_kalman_qboost_cdwn_val;                                                                   /* start cooldown */
            qboost_fired = true;                                                                                                     /* mark Q-boost: skip outlier gate below */
        }
        else if (ext->qboost_cdwn > 0) {                                                                                         /* cooldown active */
            ext->qboost_cdwn--;                                                                                                       /* decrement toward expiration */
        }

        /* ---- Prediction step: p_pred = p_est + Q (Kalman 1960) ---- */
        /* p_est ≤ 100M clamped, q ≤ 100k clamped, sum ≤ 100.1M << U32_MAX */
        p_pred = min_t(u32, ext->p_est + q, kcc_kalman_p_est_max_val);                                               /* p_pred = min(p_est+Q, p_est_max) */
        /* ---- Outlier rejection (Kalman 1960) ---- */
        /*
         * Dynamic threshold = max(outlier_ms * 1000 * scale,
         *                         jitter_ewma * outlier_jitter_mult * scale)
         *
         * If abs(innovation) > threshold AND p_pred <= converged_p_est
         * (filter is confident enough that this is truly an outlier, not
         *  a genuine path change), reject the sample.
         *
         * When rejected:
         *   - sample_cnt is NOT incremented (filter state unchanged).
         *   - jitter_ewma is updated even on rejection, to prevent the
         *     dynamic threshold from locking in at an old value.
         *   - During high jitter, the Kalman-min_rtt takeover is
         *     intentionally delayed (needs min_samples clean updates).
         * When force-accepted (after max_consec_reject):
         *   - Falls through to full Kalman update below.
         *   - sample_cnt, x_est, p_est, jitter_ewma, qdelay_avg are
         *     all updated normally on the force-accepted sample.
         */
        {
            u64 dyn_thresh = kcc_kalman_outlier_thresh_scaled_val; /* base outlier threshold (precomputed) */
            u64 jitter_prod = (u64)ext->jitter_ewma * (u64)kcc_kalman_outlier_jitter_mult_val;
            u64 jitter_thresh = min_t(u64, jitter_prod, kcc_kalman_shift_cap_val) << kcc_kalman_scale_shift_val;

            if (jitter_thresh > dyn_thresh) {                                                                                                /* jitter threshold exceeds base */
                dyn_thresh = jitter_thresh;                                                                                                  /* use jitter-based threshold */
            }

            if (unlikely(!qboost_fired && abs_innov > dyn_thresh && p_pred <= kcc_kalman_converged_p_est_val)) {                          /* outlier + confident AND Q-boost not fired */
                /*
                 * Force-accept if too many consecutive rejections have occurred.
                 * Without this, a self-reinforcing cycle can lock in: high
                 * jitter raises the dynamic threshold, causing more rejections,
                 * which raise jitter further (jitter is updated even on rejection).
                 * After kcc_kalman_max_consec_reject_val consecutive rejections,
                 * the gate is bypassed and the sample enters the filter.
                 */
                u32 raw_jitter;                                                                                          /* jitter in us (C90: decl before stmt) */
                if (ext->consec_reject_cnt < (u32)kcc_kalman_max_consec_reject_val) {                  /* still within rejection budget */
                    ext->consec_reject_cnt++;                                                                    /* increment rejection counter */
                    /* Reject outlier: do not update x_est or p_est.
                     * Update jitter for threshold dynamics but skip state update. */
                    raw_jitter = (u32)min_t(u64, abs_innov >> kcc_kalman_scale_shift_val, U32_MAX);           /* |innov| back to us */
                    ext->jitter_ewma = ext->jitter_ewma ?                                                         /* if existing EWMA */
                        ((u64)ext->jitter_ewma * kcc_ewma_jitter_num_val +                                  /* old * num + */
                            raw_jitter * KCC_EWMA_NEW_WEIGHT) / kcc_ewma_jitter_den_val :                                      /* new * 1 / den */
                        raw_jitter;                                                                                    /* first sample: use raw */
                    return;                                                                                            /* rejection: no further processing */
                }

                /* Fall through: force-accept after max consecutive rejections.
                 * Jitter EWMA will be updated by the normal Kalman post-update
                 * path below; this prevents the stale outlier-jitter feedback. */
            }

            /* Reset rejection counter on any accepted sample */
            if (ext->consec_reject_cnt) { ext->consec_reject_cnt = 0; }                                                                              /* clear: sample passed gate */
        }

        /* ---- Compute Kalman gain: K_num/K_den = p_pred / (p_pred + R) (Kalman 1960) ---- */
        gain_num = p_pred;                                                                                                                         /* numerator = predicted covariance */

        /* gain_den = p_pred + r.  With clamped params, p_pred ≤ 100M and r ≤ 100M,
         * so p_pred + r ≤ 200M << U32_MAX.  No overflow guard needed.
         * gain_den >= 1 is guaranteed by p_pred >= p_est_floor >= 1 and r >= 1. */
        gain_den = p_pred + r;                                                                                                                  /* denominator = p_pred + meas noise */

        /* ---- State update: x_est = x_est + K * innovation (Kalman 1960) ---- */
        /*
         * correction = (abs_innov * gain_num) / gain_den
         * x_est = x_est +/- correction (sign follows innovation)
         *
         * Directional update policy (KCC-specific):
         *
         * The Kalman model z = x + v assumes zero-mean measurement noise,
         * but on a congested path the observation z = x + q + v includes
         * non-negative queue delay q.  Positive innovations (z > x_est)
         * may be either path changes or queue buildup — the filter
         * cannot distinguish.  Updating on queue noise causes x_est to
         * drift toward the AVERAGE RTT rather than the propagation delay.
         *
         * Therefore:
         *   - Negative innovation (z < x_est): always update.  The observed
         *     RTT is below the current estimate — the propagation delay
         *     has likely decreased.  Pull x_est DOWN toward the clean sample.
         *   - Positive innovation + Q-boost fired: update.  The innovation
         *     is large enough to indicate a genuine path change (route
         *     switch, mobility event).  Pull x_est UP toward the new path.
         *   - Positive innovation, no Q-boost: SKIP state update.  The
         *     innovation is queue delay, not a propagation delay change.
         *     The covariance update (below) still runs — the measurement
         *     provides information about filter uncertainty.
         *
         * This transforms the Kalman into an asymmetric estimator that
         * only tracks propagation delay DECREASES (plus Q-boosted
         * increases), matching the physical constraint that queue delay
         * is non-negative and propagation delay changes are rare.
         *
         * Overflow guard: if abs_innov * gain_num would overflow u64,
         * cap the product at U64_MAX.
         */
        {
            u64 prod;                                                                                                                                  /* product abs_innov * gain_num */
            s64 correction;                                                                                                                               /* signed correction */

            if (gain_num > 0 && abs_innov > U64_MAX / (u64)gain_num) {                                                                                     /* overflow check */
                prod = U64_MAX;                                                                                                                              /* cap product at max */
            }
            else {                                                                                                                                         /* safe to multiply */
                prod = abs_innov * (u64)gain_num;                                                                                                              /* product = |innov| * gain_num */
            }

            /* Compute correction magnitude unconditionally:
             * needed for both state update AND BBR-S covariance-matched
             * noise estimation (Welch & Bishop 2006).  When the state
             * update is skipped (positive innovation = queue noise), the
             * correction magnitude still informs matched Q/R estimation. */
            corr_abs = div_u64(prod, gain_den);                                                                                                                /* absolute correction = prod / den */

            if (innovation < 0 || qboost_fired) {
                /* Downward update (clean sample) OR Q-boost (path change):
                 * apply correction in the direction of the innovation. */
                correction = (innovation >= 0) ? (s64)min_t(u64, corr_abs, KCC_S64_MAX) : -(s64)min_t(u64, corr_abs, KCC_S64_MAX);                                     /* sign follows innovation (saturate to avoid -(s64)U64_MAX UB) */

                {                                                                                                                                                      /* x_est update: ext->x_est is u32, correction is s64 bounded to ±S64_MAX */
                    s64 new_x = (s64)ext->x_est + correction;                                                                     /* cannot overflow s64: u32 max + S64_MAX < S64_MAX + 4e9 ≤ S64_MAX */
                    if (new_x > 0) {                                                                                             /* x_est must stay positive */
                        ext->x_est = (u32)min_t(s64, new_x, (s64)U32_MAX);                                                       /* clamp to u32 range */
                    }
                    else {                                                                                                                                              /* correction would make state <= 0 */
                        u64 floor = (kcc->min_rtt_us != KCC_MIN_RTT_UNINIT) ?                                                                                    /* guard: min_rtt_us valid */
                            min_t(u64, (u64)kcc->min_rtt_us << kcc_kalman_scale_shift_val, U32_MAX) : 0;                                              /* floor = min_rtt * scale or 0 */
                        ext->x_est = (u32)max_t(u64,                                                                                                                         /* floor = */
                            (u64)1U << kcc_kalman_scale_shift_val,                                                                                                 /* max(scale = 1<<shift, */
                            floor);                                                                                                                                            /* min_rtt * scale) */
                    }
                }
            }
            else {
                /* Positive innovation without Q-boost: queue noise.
                 * Skip state update — x_est stays at current estimate.
                 * The covariance update (below) still runs to reflect
                 * that we incorporated a measurement. */
            }
        }

        /* Anti-drift: the directional update (above) skips positive
         * innovations (queue noise), so x_est can only drift DOWN
         * from clean samples or UP via qboost (path changes).
         *
         * No hard cap is needed:
         *  - Cold-start overshoot is corrected once (sample_cnt==1).
         *  - Queue noise cannot lift x_est (positive innovations skipped).
         *  - Qboost-driven path-change tracking is desirable: when
         *    the innovation exceeds the qboost threshold, a genuine
         *    path change is likely, and x_est should be allowed to
         *    exceed min_rtt_us temporarily.
         *
         * Self-correction: a false-positive qboost overshoot is
         * pulled back down by subsequent negative innovations
         * (RTT samples below the over-estimate). */

         /* ---- Update jitter EWMA from accepted innovation ---- */
         /*
          * raw_jitter = |innovation| / scale (back to us)
          * jitter_ewma = (jitter_ewma * num + raw_jitter) / den
          */
        {
            u32 raw_jitter = (u32)min_t(u64, abs_innov >> kcc_kalman_scale_shift_val, U32_MAX);                                                               /* jitter in us */
            ext->jitter_ewma = ext->jitter_ewma ?                                                                                                                     /* if existing jitter EWMA */
                (u32)(((u64)ext->jitter_ewma * kcc_ewma_jitter_num_val +                                                                                               /* old * num + */
                    raw_jitter * KCC_EWMA_NEW_WEIGHT) / kcc_ewma_jitter_den_val) :                                                                                                 /* new / den */
                raw_jitter;                                                                                                                                              /* first sample */
        }
        /* ---- Covariance update: p_est = (1 - K) * p_pred (Kalman 1960) ---- */
        /*
         * p_new = p_pred - p_pred * gain_num / gain_den
         *       = p_pred * (1 - K)
         * Floor at p_est_floor_val to prevent filter lock-in.
         */
        {
            u64 p_new = (u64)p_pred -                                                                                                                                    /* p_pred - correction */
                div_u64((u64)p_pred * gain_num, gain_den);                                                                                                              /* p_pred * K */
            ext->p_est = max_t(u32, (u32)p_new, kcc_kalman_p_est_floor_val);                                                                                      /* floor at configurable minimum */
        }
        /* ---- Update EWMA queuing delay ---- */
        /*
         * qdelay_instant = max(0, (z - x_est) / scale)
         *   i.e., observed RTT minus estimated prop delay, zero if negative.
         * qdelay_avg = (qdelay_avg * num + instant) / den
         */
        {
            u32 qdelay_instant = (z > ext->x_est) ?                                                                                                                          /* if measurement > estimate */
                (u32)((z - ext->x_est) >> kcc_kalman_scale_shift_val) : 0;                                                                                                 /* u64 division, cast to u32 after */
            if (ext->sample_cnt == 1) {                                                                                                                                        /* second sample (first after init) */
                ext->qdelay_avg = qdelay_instant;                                                                                                                               /* init qdelay EWMA directly */
            }
            else {                                                                                                                                                             /* normal EWMA update */
                ext->qdelay_avg = (u32)(((u64)ext->qdelay_avg * kcc_ewma_qdelay_num_val +                                                                                        /* old * num + */
                    qdelay_instant * KCC_EWMA_NEW_WEIGHT) / kcc_ewma_qdelay_den_val);                                                                                              /* new * 1 / den */
            }
        }

        if (ext->sample_cnt < U32_MAX) {                                        /* accepted update: saturating increment */
            ext->sample_cnt++;
        }
        /*
         * BBR-S covariance-matched noise estimation (Welch & Bishop 2006).
         * Updates q_est and r_est using the latest innovation and Kalman gain.
         * Only runs on accepted samples (after outlier gate and Q-boost check).
         *
         * Q estimate: q_est = (1-alpha) * q_est + alpha * (K * innov)^2
         * R estimate: r_est = (1-beta)  * r_est + beta  * max(0, innov^2 - p_pred)
         *
         * innov and K*innov are in scaled units (us * kalman_scale).
         * Their squares are in (us * S)^2, while p_est, Q, and R are in
         * plain µs².  Shift right by 2*scale_shift (= S²) to convert the
         * matched estimates back to µs² before blending with q_est/r_est.
         * Without this, the matched terms dominate by ~10⁶×, saturating
         * q_est at its ceiling and disabling outlier rejection.
         */
        if (kcc_kalman_noise_mode_val > 0) {                                                             /* noise estimation enabled */
            u64 corr = corr_abs;                                                                                    /* K * |innov| in scaled units */
            u64 innov_sq, keps_sq;                                                                                  /* squared innovation and squared correction */
            s64 r_contrib;                                                 /* R noise contribution (signed, may be negative) */
            u64 q_new, r_new;                                              /* new Q and R estimate accumulators for EWMA blend */
            u32 alpha_n = (u32)kcc_kalman_noise_alpha_num_val;                                                /* alpha numerator */
            u32 alpha_d = (u32)kcc_kalman_noise_alpha_den_val;                                                /* alpha denominator */
            u32 beta_n = (u32)kcc_kalman_noise_beta_num_val;                                                 /* beta numerator */
            u32 beta_d = (u32)kcc_kalman_noise_beta_den_val;                                                 /* beta denominator */
            u32 q_max = (u32)kcc_kalman_q_est_max_val;                                                      /* Q est upper bound */
            u32 r_max = (u32)kcc_kalman_r_est_max_val;                                                      /* R est upper bound */
            u32 q_floor = (u32)kcc_kalman_q_est_floor_val;                                                    /* Q est lower bound */
            u32 r_floor = (u32)kcc_kalman_r_est_floor_val;                                                    /* R est lower bound */

            /* Innov^2: guard against overflow with extreme configs (scale up to 1M, RTT up to 10s).
             * Cap abs_innov at sqrt(S64_MAX) ~ 3e9 before squaring to keep innov^2 within s64 range. */
            if (abs_innov > KCC_KALMAN_INNOV_SQ_CAP) {                                              /* overflow guard: sqrt(S64_MAX) */
                abs_innov = KCC_KALMAN_INNOV_SQ_CAP;
            }

            innov_sq = (u64)abs_innov * abs_innov;                                                                  /* innov^2 in (us*S)^2 units, fits in s64 */
            innov_sq >>= (u32)kcc_kalman_scale_shift_val * 2;                                                                /* rescale (us*S)^2 → µs² (divide by S²) */

            /* (K*innov)^2: cap corr like abs_innov before squaring. */
            if (corr > KCC_KALMAN_INNOV_SQ_CAP) {                                                    /* overflow guard: sqrt(S64_MAX) */
                corr = KCC_KALMAN_INNOV_SQ_CAP;
            }

            keps_sq = corr * corr;                                                                                  /* (K*innov)^2 in (us*S)^2 units */
            keps_sq >>= (u32)kcc_kalman_scale_shift_val * 2;                                                                /* rescale (us*S)^2 → µs² (divide by S²) */

            /* Q estimate (covariance matching): Q = (1-alpha)*Q + alpha * (K*innov)^2
             * Cap keps_sq before multiplication to prevent u64 overflow.
             * After S² rescale (>> 2*shift), keps_sq fits well within u64;
             * this guard is a belt-and-suspenders for extreme scale configs. */
            if (keps_sq > U64_MAX / (u64)max_t(u32, alpha_n, 1U)) {
                keps_sq = U64_MAX / (u64)max_t(u32, alpha_n, 1U);
            }

            {
                u64 t1 = (u64)ext->q_est * (u64)kcc_kalman_noise_alpha_complement;
                u64 t2 = (u64)alpha_n * keps_sq; u64 s = (t1 > U64_MAX - t2) ? U64_MAX : t1 + t2;
                q_new = s / (u64)alpha_d;
            }          /* EWMA blend: old*(1-alpha) + alpha*sample */
            ext->q_est = (u32)clamp_t(u64, q_new, (u64)q_floor, (u64)q_max);                                       /* publish Q estimate, clamped */

            /* R estimate: R = (1-beta)*R + beta * max(0, innov^2 - p_pred)
             * Cap r_contrib before multiplication to prevent u64 overflow.
             * After S² rescale, innov_sq fits well within u64;
             * this guard is a belt-and-suspenders for extreme scale configs. */
            r_contrib = (s64)innov_sq - (s64)p_pred;                                                                /* E[innov^2] = P + R, so R_contrib = innov^2 - P */
            if (r_contrib < 0) {                                                                     /* floor at 0: negative contribution */
                r_contrib = 0;
            }

            {
                u64 r_contrib_u64 = min_t(u64, (u64)r_contrib, U64_MAX / (u64)max_t(u32, beta_n, 1U));
                r_contrib = (r_contrib_u64 > KCC_S64_MAX) ? (s64)KCC_S64_MAX : (s64)r_contrib_u64;
            }

            {
                u64 t1 = (u64)ext->r_est * (u64)kcc_kalman_noise_beta_complement; u64 t2 = (u64)beta_n * (u64)r_contrib;
                u64 s = (t1 > U64_MAX - t2) ? U64_MAX : t1 + t2;
                r_new = s / (u64)beta_d;
            }       /* EWMA blend: old*(1-beta) + beta*sample */
            ext->r_est = (u32)clamp_t(u64, r_new, (u64)r_floor, (u64)r_max);                                       /* publish R estimate, clamped */
        }
    }
}
/* ---- Min RTT Update ---------------------------------------------------- */

/*
 * kcc_update_min_rtt - Update the min_rtt_us estimate using both the
 * traditional window-based filter and the Kalman filter (Kalman 1960).
 *
 * @sk:  TCP socket.
 * @rs:  rate sample from this ACK.
 * @ext: extended state (for Kalman filter update via kcc_kalman_update).
 *
 * Processing sequence:
 *   1. Track delayed-ACK status.
 *   2. Check if PROBE_RTT filter interval has expired.
 *   3. Run Kalman filter update (feeds rtt_us into kcc_kalman_update).
 *   4. Traditional min_rtt window update (only when Kalman has NOT taken over):
 *      - Sticky fall: gradual reduction using sticky_num/sticky_den ratio.
 *      - Fast fall: immediate reduction when rtt < min_rtt / 4 for fast_fall_cnt
 *        consecutive samples.
 *   5. SRTT guard: override min_rtt if SRTT < min_rtt * guard_ratio.
 *   6. PROBE_RTT entry: if filter_expired and not idle_restart and mode != PROBE_RTT.
 *   7. PROBE_RTT management: determine stay period and exit conditions.
 *   8. Kalman takeover: when x_est is valid and sample_cnt >= min_samples,
 *      replace min_rtt_us with x_est / kalman_scale, and compute dynamic
 *      PROBE_RTT interval.
 *
 * NOTE: min_rtt_stamp is NOT refreshed when Kalman sets min_rtt_us,
 * so that PROBE_RTT can still periodically re-probe the true path
 * propagation delay (the Kalman estimate may drift on path changes).
 *
 * Corresponds to kernel BBR v5.4: bbr_update_min_rtt().
 * The core PROBE_RTT management (steps 1, 2, 6, 7) is identical.
 *
 * KCC deviations (major):
 *   a) Step 3: Kalman filter update (kcc_kalman_update).  Kernel BBR does
 *      not use a Kalman filter — it relies solely on the sliding-window
 *      minimum and PROBE_RTT drains.  KCC runs the Kalman update on every
 *      valid RTT sample to maintain x_est and p_est.
 *   b) Step 4: Sticky fall and fast fall logic for the sliding-window
 *      min_rtt.  Kernel BBR updates min_rtt_us directly on any smaller
 *      sample (simple min operation).  KCC adds hysteresis to prevent
 *      transient RTT dips from permanently deflating min_rtt_us.  The
 *      sticky ratio (default 0.75) requires multiple consecutive dips
 *      below the threshold before committing the new minimum.  The fast
 *      fall bypass (default rtt < min_rtt / 4) provides an immediate
 *      commit for very large drops (e.g., route change).
 *   c) Step 5: SRTT guard — kernel BBR has no equivalent.  If the
 *      smoothed RTT (SRTT/8) drops below min_rtt * guard_ratio (default
 *      0.90), min_rtt is overridden by SRTT/8.  This prevents min_rtt_us
 *      from becoming stale when the path latency genuinely decreases but
 *      no single RTT sample falls below the old minimum.
 *   d) Step 8: Kalman min-rtt pull-down with hysteresis.  After the
 *      Kalman filter converges (sample_cnt >= min_samples), if x_est is
 *      consistently below min_rtt_us for minrtt_fast_fall_cnt consecutive
 *      rounds, min_rtt_us is replaced by the Kalman estimate.  Kernel
 *      BBR has no Kalman filter and no equivalent mechanism.
 *
 * BENEFIT of (b): Prevents BDP deflation from transient RTT dips on
 * bursty paths (WiFi, cellular).  BENEFIT of (c): Guarantees min_rtt_us
 * tracks genuine path-latency decreases even when no single sample
 * undercuts the old min.  BENEFIT of (d): Kalman's unbiased estimate
 * provides a tighter lower bound than the windowed min, improving BDP
 * accuracy on paths with persistent queue noise.
 *
 * Type note: 'filter_expired' gate uses u32 jiffies arithmetic with the
 * after() macro.  Per-flow jitter is computed from sk->sk_hash (u32) to
 * desynchronise PROBE_RTT entry across flows.  rtt_clamped is u32,
 * floored at 1 us.  All types match kernel BBR's bbr_update_min_rtt().
 */
static void kcc_update_min_rtt(struct sock* sk, const struct rate_sample* rs,        /* update min_rtt + Kalman + PROBE_RTT */
    struct kcc_ext* ext)                                                /* extended state */
{
    struct tcp_sock* tp = tcp_sk(sk);                                                    /* get TCP socket state */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                       /* get KCC CA state */
    bool filter_expired;                                                                     /* flag: PROBE_RTT interval expired */

    u32 now, rtt_clamped;                                                                    /* hoisted: jiffies timestamp and clamped RTT */

    now = tcp_jiffies32;                                                                    /* cache volatile jiffies for entire function */
    rtt_clamped = rs->rtt_us >= 0 ? (u32)min_t(s64, rs->rtt_us, U32_MAX) : 0;               /* hoist clamped RTT: used 4x below */
    /*
     * PROBE_RTT entry guard: filter_expired determines whether the
     * 10-second min_rtt filter window has elapsed since the last
     * min_rtt_us update.  When expired, the connection enters
     * PROBE_RTT mode to re-measure true propagation delay.
     *
     * Kernel BBR (v5.4 net/ipv4/tcp_bbr.c) computes:
     *   filter_expired = after(tcp_jiffies32,
     *       bbr->min_rtt_stamp + bbr_min_rtt_win_sec * HZ);
     *
     * KCC deviates from the kernel in two ways:
     *
     * 1. Per-flow jitter (BBR1 multi-flow fix):
     *
     *    The kernel's static window causes all co-existing flows
     *    sharing a bottleneck to enter PROBE_RTT simultaneously.
     *    N flows simultaneously drain to 4 packets (aggregate ~2*N
     *    Mbps) and then simultaneously refill at 2.89x high_gain,
     *    creating an N× overshoot.  The last flow to complete
     *    refill faces severe congestion and can enter RTO with zero
     *    throughput for seconds.
     *
     *    Per-flow jitter, derived from the stable per-socket hash
     *    (sk->sk_hash), spreads the PROBE_RTT entry window across
     *    0..2× min_rtt_us (0..~744 us at 372 us min_rtt).  At any
     *    instant, at most ~1 flow is in PROBE_RTT, eliminating the
     *    synchronised drain/refill overshoot.
     *
     * 2. Dynamic PROBE_RTT interval (KCC Kalman optimisation):
     *
     *    The kernel uses a fixed 10-second interval regardless of
     *    path stability.  When KCC's Kalman filter is converged
     *    (p_est < kcc_kalman_converged), the state estimate x_est
     *    tracks true propagation delay continuously from every
     *    valid RTT sample.  The periodic PROBE_RTT drain, which
     *    cuts cwnd to 4 segments for 200 ms, becomes unnecessary
     *    overhead — the Kalman filter already maintains an accurate
     *    min_rtt without queue-draining.
     *
     *    The interval extends from the base 10 s to
     *    kcc_probe_rtt_dyn_max_sec (default 30 s) when converged,
     *    reducing throughput dips from ~2 % (200 ms / 10 s) to
     *    ~0.7 % (200 ms / 30 s).  When the filter diverges
     *    (p_est > 4× threshold), the interval shortens to 5 s for
     *    urgent recalibration.
     */
    {
        u32 interval = kcc_get_probe_rtt_interval(sk, ext);
        u32 jitter_jif = 0;

        if (kcc->min_rtt_us != KCC_MIN_RTT_UNINIT && kcc->min_rtt_us > 0) {
            jitter_jif = usecs_to_jiffies(
                (u32)(sk->sk_hash & KCC_PROBE_RTT_JITTER_HASH_MASK) *
                kcc->min_rtt_us / KCC_PROBE_RTT_JITTER_DIV);
        }

        filter_expired = after(now,
            kcc->min_rtt_stamp + interval + jitter_jif);
    }

    /* Kalman filter update: feed every valid RTT sample into the filter (Kalman 1960) */
    if (likely(rs->rtt_us >= 0)) {                                                                         /* valid RTT sample */
        kcc_kalman_update(sk, rtt_clamped, ext);                             /* run Kalman update */
    }

    /* ---- Traditional min_rtt window update ---- */
    /*
     * Only apply window-based min_rtt tracking when the Kalman filter
     * has NOT taken over (i.e., either ext is NULL, x_est is 0, or
     * sample_cnt < min_samples).
     *
     * Conditions for updating min_rtt:
     *   - rtt_us < min_rtt_us (new minimum), OR
     *   - filter expired AND not delayed ACK (re-probe the min)
     */
    if (rs->rtt_us >= 0 &&                                                                           /* valid RTT */
        (rtt_clamped <= kcc->min_rtt_us ||                                                        /* new minimum OR */
            (filter_expired && !rs->is_ack_delayed))) {                                                   /* expired filter + valid sample */
        rtt_clamped = max_t(u32, rtt_clamped, 1U);                          /* floor at 1 us (kernel clock granularity) */
        if (kcc->min_rtt_us == KCC_MIN_RTT_UNINIT) {
            kcc->min_rtt_us = rtt_clamped;
            kcc->min_rtt_stamp = now;
            goto done_min_rtt;
        }

        if (rtt_clamped < (u64)kcc->min_rtt_us * kcc_minrtt_sticky_num_val /                          /* rtt < min_rtt * sticky_ratio */
            kcc_minrtt_sticky_den_val) {                                                     /* below sticky threshold */
            /*
             * Sticky fall: new RTT is significantly lower than current
             * min_rtt (e.g., 25% lower at 0.75 ratio).  Two sub-cases:
             *   1. Very large drop (> 75%): immediate update (fast fall reset).
             *   2. Moderate drop: count consecutive sticky samples;
             *      after fast_fall_cnt, commit the drop.
             */
            if (rtt_clamped < kcc->min_rtt_us / (u32)kcc_minrtt_fast_fall_div_val) {      /* rtt < min_rtt / div: fast fall */
                kcc->min_rtt_us = rtt_clamped;                                                                 /* immediate update */
                kcc->min_rtt_fast_fall_cnt = 0;                                                                  /* reset fast-fall counter */
            }
            else {                                                                                               /* moderate drop: sticky */
                kcc->min_rtt_fast_fall_cnt = min_t(u32, kcc->min_rtt_fast_fall_cnt + 1, KCC_BITFIELD_2BIT_MAX); /* saturate at 2-bit field max */
                if (kcc->min_rtt_fast_fall_cnt >= kcc_minrtt_fast_fall_cnt_val) {                          /* counter reached threshold */
                    kcc->min_rtt_us = rtt_clamped;                                                                     /* commit the drop */
                    kcc->min_rtt_fast_fall_cnt = 0;                                                                      /* reset counter */
                }
                else {                                                                                                  /* still counting */
                    /* Partial decrease for first sticky sample per round */
                    if (kcc->round_start) {
                        kcc->min_rtt_us = max_t(u32, 1U,
                            (u64)kcc->min_rtt_us *
                            kcc_minrtt_sticky_num_val /
                            kcc_minrtt_sticky_den_val);
                    }
                }
            }
        }
        else {                                                                                                               /* normal update */
            kcc->min_rtt_us = rtt_clamped;                                                                                       /* straightforward min_rtt update */
            kcc->min_rtt_fast_fall_cnt = 0;                                                                                        /* reset fast-fall counter */
        }

        kcc->min_rtt_stamp = now;                                                                                        /* record update time */
    }
    else if (rs->rtt_us >= 0 && !filter_expired &&                                                                /* valid RTT + filter not expired */
        rtt_clamped >= kcc->min_rtt_us) {                                                                                      /* no new minimum: end of fast-fall episode */
        kcc->min_rtt_fast_fall_cnt = 0;                                                                                                /* reset fast-fall counter */
    }

done_min_rtt:
    /* ---- SRTT guard ---- */
    /*
     * If the smoothed RTT (SRTT/8) is anomalously lower than min_rtt_us,
     * it means min_rtt_us has become stale.  Override it with SRTT/8.
     * Guard ratio default: 90% -> SRTT < 90% of min_rtt triggers override.
     * Apply to min_rtt_us regardless of Kalman state — SRTT below
     * min_rtt means our estimate is stale in all cases.
     */
    if (
        tp->srtt_us && kcc->min_rtt_us && kcc->min_rtt_us != KCC_MIN_RTT_UNINIT &&                                                /* SRTT + min_rtt valid + init */
        (tp->srtt_us >> 3) < (u64)kcc->min_rtt_us *                                                                                     /* SRTT/8 < min_rtt * ratio */
        kcc_minrtt_srtt_guard_num_val / kcc_minrtt_srtt_guard_den_val) {                                     /* SRTT guard ratio check */
        kcc->min_rtt_us = tp->srtt_us >= 8 ? tp->srtt_us >> 3 : 1;                                                                                          /* override with smoothed RTT */
        kcc->min_rtt_stamp = now;                                                                                          /* refresh stamp */
    }

    /* ---- PROBE_RTT entry (Cardwell et al. 2016, with Kalman decoupling) ---- */
    /*
      * Decision matrix when the PROBE_RTT filter interval expires.
      *
      * The PROBE_RTT mechanism exists to refresh min_rtt_us.  In FILTER mode,
      * BDP is computed from x_est_us (not min_rtt_us), so PROBE_RTT is
      * unnecessary for BDP accuracy.  However, the Kalman filter can diverge
      * after major path changes — when it does, a single PROBE_RTT drain
      * restores the baseline.  This matrix implements that adaptive logic.
      *
      *   kcc_rtt_mode | kcc_probe_rtt_decouple | Action
      *   --------------------------------------------------------------
      *   MIN (0)      | any                    | Traditional PROBE_RTT:
      *               |                        | enter, drain to 4 pkts.
      *   FILTER (1)   | 0 (disabled)          | Traditional PROBE_RTT:
      *               |                        | enter, drain to 4 pkts.
      *   FILTER (1)   | 1 (enabled)           | Smart decoupling:
      *               |                        |   a) Kalman healthy:
      *               |                        |      suppress PROBE_RTT
      *               |                        |      (no throughput cliff).
      *               |                        |   b) Kalman diverged:
      *               |                        |      enter PROBE_RTT as
      *               |                        |      a safety net — single
      *               |                        |      drain restores the
      *               |                        |      filter baseline.
      *
      * When suppressed: the Kalman filter continuously tracks true
      * propagation delay via outlier gating and adaptive noise
      * estimation — the periodic 4-packet drain is unnecessary.
      * This eliminates the BBR throughput cliff and PROBE_RTT
      * synchronisation collapses on multi-flow links.
      *
      * When Kalman p_est rises above kcc_recal_p_est_thresh, the
      * filter has lost confidence (path change, noise-model mismatch).
      * A single traditional PROBE_RTT drain provides a fresh min_rtt
      * baseline; the Kalman re-converges and decoupling resumes.
      *
      * Note: idle_restart flag is cleared on any data delivery
      * (rs->delivered > 0), so an idle-restarted connection
      *     correctly skips PROBE_RTT entry on the first ACK post-restart.
      */
    if (unlikely(filter_expired && !kcc->idle_restart
        && kcc->mode != KCC_PROBE_RTT)) {                                                                                        /* interval expired + valid state */
        if (kcc_rtt_mode && kcc_probe_rtt_decouple                                                                       /* FILTER + decouple active AND */
            && !kcc_kalman_needs_recalibration(ext)) {                                                                /* Kalman filter is healthy */
            goto skip_probe_rtt;                                                                                              /* suppress PROBE_RTT: no throughput cliff */
        }
        /* Kalman degrading OR traditional mode: enter PROBE_RTT */
        kcc->mode = KCC_PROBE_RTT;                                                                                                     /* enter PROBE_RTT */
        kcc_save_cwnd(sk);                                                                                                              /* save cwnd for later restore */
        kcc->probe_rtt_done_stamp = 0;                                                                                                   /* clear: stay period not yet started */
    }
skip_probe_rtt:

    /* ---- PROBE_RTT management ---- */
    if (unlikely(kcc->mode == KCC_PROBE_RTT)) {                                                                                                      /* active PROBE_RTT mode */
        /* app_limited = delivered + inflight; ensures app-limited is nonzero
         * so the pacing engine doesn't think the connection is idle */
        u32 app_limited_val = (u32)((u64)tp->delivered + tcp_packets_in_flight(tp));                                                     /* compute app-limited value */
        tp->app_limited = app_limited_val ? app_limited_val : 1;                                                                              /* set app_limited (never 0) */

        if (!kcc->probe_rtt_done_stamp) {                                                                                                      /* stay period not yet entered */
            if (tcp_packets_in_flight(tp) <= kcc_cwnd_min_target_val ||                                                              /* inflight at min OR */
                kcc->round_start) {                                                                                                              /* round boundary reached */
                /* Inflight has dropped to minimum OR we are at a round boundary.
                 * Start the stay timer (default 200ms). */
                kcc->probe_rtt_done_stamp = now +                                                                                      /* now + stay duration */
                    msecs_to_jiffies(kcc_probe_rtt_mode_ms_val);                                                                       /* convert ms to jiffies */
                kcc->probe_rtt_round_done = 0;                                                                                                     /* clear round done flag */
                kcc->next_rtt_delivered = tp->delivered;                                                                                           /* reset round baseline */
            }
        }
        else {                                                                                                                                     /* in stay period */
            if (kcc->round_start) {                                                                                                                      /* new round boundary */
                kcc->probe_rtt_round_done = 1;                                                                                                           /* mark round done */
            }
            if (kcc->probe_rtt_round_done) {                                                                                                              /* at least one round elapsed */
                kcc_check_probe_rtt_done(sk);                                                                                                              /* check exit conditions */
            }
        }
    }

    /* Clear idle_restart on any data delivery — enables PROBE_RTT entry
     * on next expired filter (the PROBE_RTT entry guard at line 4716 checks
     * !kcc->idle_restart). */
    if (rs->delivered > 0) {                                                                                                                               /* data delivered */
        kcc->idle_restart = 0;                                                                                                                               /* clear idle_restart */
    }

    /* ---- Kalman min-rtt pull-down (Kalman 1960) ---- */
    /*
     * When the Kalman filter has converged (valid x_est and sufficient
     * samples), allow it to pull min_rtt_us DOWN if its estimate is
     * lower than the windowed min.  The windowed min (updated above)
     * provides an upper-bound safety net against Kalman upward drift.
     *
     * Hysteresis: require kcc_minrtt_fast_fall_cnt consecutive Kalman
     * estimates below min_rtt_us before committing the pull-down.
     * On a long-RTT path (212ms) the Kalman gain K ≈ 0.86 produces
     * corrections up to ~8ms per sample from statistical jitter — a
     * single-sample overshoot would permanently lower min_rtt_us
     * (directional update prevents upward correction) and deflate
     * BDP by several percent for up to 10 seconds until the next
     * PROBE_RTT window expiry.
     *
     * Reuses min_rtt_fast_fall_cnt as a shared confirmation counter:
     * both the sliding-window sticky-fall and the Kalman takeover
     * agree that RTT is trending lower — the counter accumulates
     * evidence from both sources and commits when the threshold is
     * reached.  Default threshold = 3 consecutive confirming rounds.
     *
     * Update min_rtt_stamp so the next PROBE_RTT entry is governed
     * by the normal filter_expired window (10s or dynamic interval),
     * not by the age of a pre-takeover stamp.  This prevents premature
     * PROBE_RTT: a lower Kalman estimate improves min_rtt_us without
     * forcing an immediate bandwidth crash.
     *
     * Does NOT apply during PROBE_RTT (we want the raw min in that mode).
     */
    if (ext && ext->x_est && ext->sample_cnt >= kcc_kalman_min_samples_val &&                                                                  /* Kalman converged */
        kcc->mode != KCC_PROBE_RTT) {                                                                                                                          /* not in PROBE_RTT mode */
        u32 krtt = (u32)min_t(u64, ext->x_est >> kcc_kalman_scale_shift_val, U32_MAX);                                                                        /* Kalman RTT estimate */
        if (krtt < kcc->min_rtt_us) {                                                                                                                           /* Kalman lower than windowed min */
            kcc->min_rtt_fast_fall_cnt = min_t(u32,                                                                                                                 /* saturating increment */
                kcc->min_rtt_fast_fall_cnt + 1, KCC_BITFIELD_2BIT_MAX);                                                                                               /* 2-bit ceiling = 3 */
            if (kcc->min_rtt_fast_fall_cnt >= (u32)kcc_minrtt_fast_fall_cnt_val) {                                                                                     /* N consecutive confirming rounds */
                kcc->min_rtt_us = krtt;                                                                                                                             /* commit Kalman pull-down */
                kcc->min_rtt_fast_fall_cnt = 0;                                                                                                                      /* reset counter after commit */
                kcc->min_rtt_stamp = now;                                                                                                                            /* refresh stamp: prevent stale-stamp immediate PROBE_RTT */
                kcc_update_dyn_probe_interval(ext);                                                                                                                  /* recompute dynamic interval */
            }
        }
        else {                                                                                                                                                     /* Kalman estimate NOT lower */
            kcc->min_rtt_fast_fall_cnt = 0;                                                                                                                          /* reset counter: trend broken */
        }
    }
}
/* ---- ACK Aggregation Compensation (Cardwell et al. 2016) ------------ */

/*
 * kcc_update_ack_aggregation - Track extra ACKed data beyond the bandwidth
 * estimate to compensate for ACK aggregation (delayed/stretched ACKs).
 *
 * @sk:  TCP socket.
 * @rs:  rate sample from this ACK.
 * @ext: extended state (ack epoch tracking fields in struct kcc_ext).
 *
 * Algorithm (dual-window sliding max, inspired by BBRplus):
 *   - Two windows (indices 0 and 1) each spanning approx 5 RTTs.
 *   - Within each window, track the maximum extra_acked value observed.
 *     The effective extra_acked is max(win[0], win[1]).
 *
 * On each ACK:
 *   1. If round_start: increment window RTT counter, rotate windows at 5 RTTs.
 *   2. Compute epoch elapsed time and expected_acked = bw * epoch_us.
 *   3. If expected_acked >= ack_epoch_acked (more expected than received):
 *      reset the epoch (prevents accumulating stale extra_acked).
 *   4. Compute extra_acked = ack_epoch_acked - expected_acked.
 *   5. Update the sliding max in the current window.
 *
 * Epoch reset conditions:
 *   - ack_epoch_acked <= expected_acked (ACKs caught up), OR
 *   - ack_epoch_acked + this_acked >= 1M (epoch cap; prevents overflow).
 *
 * Corresponds to kernel BBR v5.4: bbr_update_ack_aggregation().
 * Kernel BBR uses a similar epoch-based mechanism with a single window.
 * The expected_acked formula and epoch reset logic are identical.
 *
 * KCC deviations:
 *   a) Dual-window sliding max (two alternating windows) instead of
 *      kernel BBR's single-window approach.
 *      BBR practice: bbr_update_ack_aggregation tracks extra_acked in
 *      a single window that is reset every ~5 RTTs.  The reset clears
 *      the accumulated max, causing a cwnd "cliff" at window boundaries.
 *      WHY: Dual windows preserve the peak across window boundaries:
 *      when one window resets, the other still holds the recent peak.
 *      This prevents the cwnd cliff that occurs every 5 RTTs in BBR.
 *      BENEFIT: Smoother cwnd trajectory, no periodic cwnd drops from
 *      window resets.
 *
 *   b) The tracking fields (extra_acked[], ack_epoch_acked, etc.) are
 *      in the heap-allocated kcc_ext rather than bitfields in struct kcc.
 *      Kernel BBR stores extra_acked as a u32 bitfield in struct bbr.
 *      KCC uses kcc_ext because the in-sock CA slot (ICSK_CA_PRIV_SIZE)
 *      is consumed by Kalman-filter state and bitfield packing.
 *      The dual-window array would not fit in the available bitfield
 *      budget.  This is an implementation constraint, not a semantic
 *      deviation.
 *
 * Type note: epoch_us is s64 (from tcp_stamp_us_delta) to handle
 * potential monotonic clock reordering gracefully.  Clamped to 0 if
 * negative.  expected_acked is u32, capped at U32_MAX if the product
 * (bw * epoch_us) would overflow.  The epoch cap (kcc_ack_epoch_max_val)
 * prevents unbounded accumulation of ack_epoch_acked.
 */
static void kcc_update_ack_aggregation(struct sock* sk,                                        /* track ACK aggregation */
    const struct rate_sample* rs,                                           /* rate sample */
    struct kcc_ext* ext)                                                    /* extended state */
{
    struct tcp_sock* tp = tcp_sk(sk);                                                            /* get TCP socket state */
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                               /* get KCC CA state */
    u64 epoch_us; u32 expected_acked, extra_acked;                                                       /* epoch vars */

    if (!ext || !kcc_extra_acked_gain_val) {                                                /* disabled or no ext */
        return; /* early return */
    }

    if (rs->acked_sacked == 0 || rs->delivered < 0 || rs->interval_us <= 0) {                                               /* invalid sample */
        return; /* early return */
    }

    /* Window rotation: each window lasts approx 5 RTTs */
    if (kcc->round_start) {                                                                            /* new round boundary */
        ext->extra_acked_win_rtts = min_t(u32, ext->extra_acked_win_rtts + 1, (u32)kcc_extra_acked_win_rtts_max_val);                      /* increment window RTT count */
        if (ext->extra_acked_win_rtts >= (u32)kcc_agg_window_rotation_rtts_val) {                     /* configured RTTs elapsed */
            ext->extra_acked_win_rtts = 0;                                                                 /* reset RTT counter */
            ext->extra_acked_win_idx = ext->extra_acked_win_idx ? 0 : 1;                                    /* rotate to other window */
            ext->extra_acked[ext->extra_acked_win_idx] = 0;                                                  /* clear new window max */
        }
    }

    /* Epoch elapsed time since last reset (us). Guard against negative delta
     * (monotonic clock reorder on some kernels/NIC drivers). */
    epoch_us = max_t(s64, tcp_stamp_us_delta(tp->delivered_mstamp, ext->ack_epoch_mstamp), 0);

    /* Expected ACKed data based on bandwidth estimate and epoch duration */
    {
        u64 bw_val = kcc_bw(sk);
        if (unlikely(epoch_us > U64_MAX / max_t(u64, bw_val, 1ULL))) {
            expected_acked = U32_MAX;
        }
        else {
            expected_acked = (u32)min_t(u64, (bw_val * epoch_us) >> BW_SCALE, U32_MAX);
        }
    }

    /*
     * Epoch reset: either we've received less than expected (ACKs caught up),
     * or we're approaching the configured epoch cap (prevents u32 overflow).
     */
    if (ext->ack_epoch_acked <= expected_acked ||                                                              /* ACKs caught up OR */
        ext->ack_epoch_acked >= kcc_ack_epoch_max_val) {               /* epoch cap reached (direct comparison) */
        ext->ack_epoch_acked = 0;                                                                                    /* reset acked counter */
        ext->ack_epoch_mstamp = tp->delivered_mstamp;                                                                 /* start new epoch */
        expected_acked = 0;                                                                                            /* reset expected */
    }

    {
        u64 new_acked = (u64)ext->ack_epoch_acked + rs->acked_sacked;
        ext->ack_epoch_acked = (u32)min_t(u64, kcc_ack_epoch_max_val, new_acked);
    } /* accumulate acked (capped) */

    extra_acked = (ext->ack_epoch_acked > expected_acked) ?
        ext->ack_epoch_acked - expected_acked : 0;                                                               /* excess beyond expected */
    extra_acked = min_t(u32, extra_acked, tp->snd_cwnd);                                                                /* cap at current cwnd */

    /* Sliding max over the current window */
    if (extra_acked > ext->extra_acked[ext->extra_acked_win_idx]) {                                                       /* new window max */
        ext->extra_acked[ext->extra_acked_win_idx] = min_t(u32, extra_acked, U32_MAX);                                  /* store as u32 */
    }
}
/*
 * kcc_ack_aggregation_cwnd - Compute the ACK aggregation cwnd bonus
 * (Cardwell et al. 2016).
 *
 * @sk:  TCP socket.
 * @ext: extended state (dual-window extra_acked array).
 * @bw:  bandwidth estimate in BW_UNIT (used to compute the max-agg-cwnd cap).
 *
 * Bonus = gain * max(extra_acked[0], extra_acked[1]) / BBR_UNIT.
 * Capped at max_aggr_cwnd = bw * max_ms * 1000 / BW_UNIT (default 100ms worth of data).
 *
 * Returns 0 if aggregation compensation is disabled (gain == 0),
 * full_bw not reached, or ext is NULL.
 *
 * Corresponds to kernel BBR v5.4: bbr_ack_aggregation_cwnd().
 * Identical formula: gain * max_extra_acked >> BBR_SCALE, capped at
 * bw * max_agg_window (kernel BBR uses the same cap with a compile-time
 * window constant).  Both implementations check full_bw_reached and gain
 * before computing the bonus.
 *
 * KCC deviation: dual-window max (max(extra_acked[0], extra_acked[1]))
 * vs. kernel BBR's single-window extra_acked.  See kcc_update_ack_aggregation
 * for the rationale on why dual windows prevent cwnd cliffs.
 *
 * Additionally, KCC applies the standard ACK-agg bonus BEFORE the
 * confidence-gated second layer (kcc_agg_cwnd_compensation) in
 * kcc_set_cwnd.  The confidence-gated layer is separate and only
 * activates at high agg_state.  Kernel BBR has no confidence gating.
 *
 * Type note: 'bw' is u32 (BW_UNIT).  Internal multiplication uses u64
 * to prevent overflow of (bw * max_ms * USEC_PER_MSEC).  The cap
 * max_aggr_cwnd is computed as u32 by shifting the u64 product right by
 * BW_SCALE.  Return is u32 (segments of cwnd bonus).
 */
static u32 kcc_ack_aggregation_cwnd(struct sock* sk, struct kcc_ext* ext, u32 bw)                    /* compute ACK-agg cwnd bonus */
{
    u32 max_aggr_cwnd = 0, aggr_cwnd = 0;                                                          /* max cap and computed bonus */

    if (kcc_extra_acked_gain_val && kcc_full_bw_reached(sk) && ext) {                   /* enabled + full_bw + ext valid */
        { /* saturating multiply: bw * max_ms * USEC_PER_MSEC */
            u64 max_ms = (u64)kcc_extra_acked_max_ms_val * USEC_PER_MSEC;
            u64 product;
            if (max_ms == 0 || bw > U64_MAX / max_ms) {
                product = U64_MAX;
            }
            else {
                product = bw * max_ms;
            }

            max_aggr_cwnd = (u32)min_t(u64, product >> BW_SCALE, U32_MAX);
        }

        {
            u64 aggr64 = ((u64)kcc_extra_acked_gain_val *
                max_t(u32, ext->extra_acked[0], ext->extra_acked[1])) >> BBR_SCALE;
            aggr_cwnd = (u32)min_t(u64, aggr64, max_aggr_cwnd);
        }
    }
    return aggr_cwnd;                                                                                      /* return bonus (0 if disabled) */
}

/* ---- ACK Aggregation Confidence-based Compensation --------------------
 *
 * BBRplus-inspired enhancement: uses extra_acked as a signal-quality
 * indicator for the Kalman filter rather than a direct cwnd adder.
 *
 * Five modules:
 *   1. kcc_measure_ack_aggregation: compute extra_acked estimate
 *   2. kcc_evaluate_agg_confidence: score 0..1024 based on 4 factors
 *   3. kcc_agg_cwnd_compensation: safe cwnd bonus with safety valve
 *   4. kcc_agg_safety_check: four-guard validation before compensation
 *   5. kcc_agg_watchdog: demote confidence after N RTTs + decay max
 */

 /*
  * kcc_measure_ack_aggregation - Compute the excess ACKed data beyond
  * the bandwidth expectation.  Returns extra segments (not bytes).
  */
static u32 kcc_measure_ack_aggregation(struct sock* sk, const struct rate_sample* rs, /* socket + rate sample */
    struct kcc_ext* ext)                                              /* extended state for agg tracking */
{
    struct tcp_sock* tp = tcp_sk(sk);
    u32 expected_acked, extra;
    u32 cur_bw;

    if (!ext || rs->interval_us <= 0) {
        return 0;
    }

    cur_bw = kcc_bw(sk);

    /* expected_acked = bw * interval_us / BW_UNIT (segments) */
    expected_acked = (u32)(((u64)cur_bw * rs->interval_us) >> BW_SCALE);

    if (rs->acked_sacked > expected_acked) {
        extra = rs->acked_sacked - expected_acked;
    }
    else {
        extra = 0;
    }

    /* Cap 1: not more than current cwnd */
    extra = min_t(u32, extra, tcp_snd_cwnd(tp));

    /* Cap 2: not more than bw * window_ms worth of data */
    {
        u64 max_ms2 = (u64)kcc_agg_max_window_ms_val * USEC_PER_MSEC;
        u64 bw_prod;
        u64 bw_cap;
        if (max_ms2 == 0 || (u64)cur_bw > U64_MAX / max_ms2) {
            bw_prod = U64_MAX;
        }
        else {
            bw_prod = (u64)cur_bw * max_ms2;
        }

        bw_cap = bw_prod >> BW_SCALE;
        extra = min_t(u32, extra, (u32)min_t(u64, bw_cap, U32_MAX));
    }

    /* Update dual-slot windowed maximum */
    if (extra > ext->agg_extra_acked_max) {
        ext->agg_extra_acked_max = extra;
    }

    ext->agg_extra_acked = extra;
    return extra;
}

/*
 * kcc_evaluate_agg_confidence - Score the trustworthiness of the current
 * extra_acked signal on a 0..1024 scale using four orthogonal factors,
 * each contributing 256 points.  Any single false signal cannot reach
 * CONFIRMED (512) alone.
 */
static u16 kcc_evaluate_agg_confidence(struct sock* sk, struct kcc_ext* ext, /* CA state + extended state */
    u32 extra_acked)                                                    /* current ACK's extra_acked estimate */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);
    u16 conf = 0;

    if (!ext) {
        return 0;
    }

    /* Factor 1: Kalman filter converged (estimate is reliable). Also requires
     * minimum sample count to avoid scoring before the filter has meaningful data. */
    if (ext->p_est < kcc_kalman_converged_p_est_val &&
        ext->sample_cnt >= kcc_kalman_min_samples_val) {
        conf += (u16)kcc_agg_factor_weight_val;  /* + configured weight */
    }

    /* Factor 2: No loss signal (no real congestion) */
    if (inet_csk(sk)->icsk_ca_state < TCP_CA_Recovery) {
        conf += (u16)kcc_agg_factor_weight_val;  /* + configured weight */
    }

    /* Factor 3: No sustained queue delay (x_est near min_rtt).
     * Requires valid Kalman state — cold start scores 0, not a free pass. */
    if (ext->x_est > 0 && kcc->min_rtt_us != KCC_MIN_RTT_UNINIT && kcc->min_rtt_us > 0) {
        u32 est_rtt = ext->x_est >> kcc_kalman_scale_shift_val;
        if (est_rtt <= kcc->min_rtt_us + (u32)kcc_agg_factor3_qdelay_us_val) {  /* within configurable margin */
            conf += (u16)kcc_agg_factor_weight_val;  /* + configured weight */
        }
    }
    /* No else: cold start with no estimate scores 0 for this factor */

    /* Factor 4: extra_acked magnitude check vs history (not a transient spike) */
    if (extra_acked == 0 || ext->agg_extra_acked_max == 0 ||
        (u64)extra_acked * (u64)kcc_agg_factor4_ratio_den_val <=
        (u64)ext->agg_extra_acked_max * (u64)kcc_agg_factor4_ratio_num_val) {
        conf += (u16)kcc_agg_factor_weight_val;  /* +configured weight, within configurable ratio of windowed max */
    }

    return conf;  /* 0..1024 */
}

/*
 * kcc_agg_state_from_confidence - Map confidence score to state enum.
 */
static u8 kcc_agg_state_from_confidence(u16 confidence)                 /* confidence score 0..1024 */
{
    if (confidence >= (u16)kcc_agg_thresh_trusted_val) {
        return KCC_AGG_TRUSTED;
    }

    if (confidence >= (u16)kcc_agg_thresh_confirmed_val) {
        return KCC_AGG_CONFIRMED;
    }

    if (confidence >= (u16)kcc_agg_thresh_suspected_val) {
        return KCC_AGG_SUSPECTED;
    }

    return KCC_AGG_IDLE;
}

/*
 * kcc_agg_safety_check - Four-guard validation before cwnd compensation.
 * Returns true if compensation is safe.
 */
static bool kcc_agg_safety_check(struct sock* sk, struct kcc_ext* ext, u32 bw) /* CA state + ext state + bw (BW_UNIT) */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);
    struct tcp_sock* tp = tcp_sk(sk);
    u32 safe_cwnd;
    u64 bdp_est;

    if (!ext) {
        return false;
    }

    /* Guard 1: Queue delay rising? Skip if Kalman cold (x_est == 0). */
    if (kcc->min_rtt_us != KCC_MIN_RTT_UNINIT && kcc->min_rtt_us > 0 && ext->x_est > 0) {
        u32 est_rtt = ext->x_est >> kcc_kalman_scale_shift_val;
        if ((u64)est_rtt > (u64)kcc->min_rtt_us + (u64)kcc_agg_safety_qdelay_us_val) {  /* >configurable margin */
            return false;  /* queue building, stop compensation */
        }
    }

    /* Guard 2: In loss recovery? */
    if (inet_csk(sk)->icsk_ca_state >= TCP_CA_Recovery) {
        return false;
    }

    /* Guard 3: CWND already > N x BDP?  (hard ceiling) */
    if (kcc->min_rtt_us == KCC_MIN_RTT_UNINIT || kcc->min_rtt_us == 0) {                              /* guard: min_rtt not yet measured */
        return false;
    }

    bdp_est = ((u64)bw * kcc->min_rtt_us) >> BW_SCALE;                /* u64 cast prevents overflow */
    safe_cwnd = (u32)min_t(u64, bdp_est * kcc_agg_safety_bdp_mult_val, U32_MAX);        /* u64 for safety against large BDP */
    if (tp->snd_cwnd >= safe_cwnd) {
        return false;
    }

    /* Guard 4: Inflight already excessive? */
    if (tcp_packets_in_flight(tp) >= (u64)safe_cwnd + kcc_tso_segs_goal(sk)) {
        return false;
    }

    return true;
}

/*
 * kcc_agg_cwnd_compensation - Compute safe cwnd bonus from aggregation signal.
 * Five-layer safety: confidence gate -> safety check -> progressive scaling
 * -> hard cap at BDP/2 -> watchdog timer.
 * Returns extra cwnd segments to add (0 = no compensation).
 */
static u32 kcc_agg_cwnd_compensation(struct sock* sk, struct kcc_ext* ext, /* socket + extended state */
    u32 extra_acked, u16 confidence, u32 bw)                            /* current extra_acked + confidence score + bw (BW_UNIT) */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);
    u32 comp = 0, agg_est = 0, bdp = 0;
    int thr;

    if (!ext || !kcc_agg_enable_val) {
        return 0;
    }

    /* Single cached read of threshold for both gate and computation. */
    thr = kcc_agg_confidence_thresh_val;                    /* dynamic threshold (clamped cache) */

    /* Layer 1: Confidence must reach CONFIRMED (512) */
    if (confidence < (u16)thr) {
        return 0;
    }

    /* Layer 2: Safety check must pass */
    if (!kcc_agg_safety_check(sk, ext, bw)) {
        return 0;
    }

    /* Layer 3: Progressive scaling: maps [threshold, confidence_max] → [0, agg_est].
     * Uses the configured threshold (not hardcoded 512) for both gating
     * and scaling range.  Denominator is (confidence_max - threshold) with div-by-zero
     * guard for threshold ≥ confidence_max. */
    agg_est = max_t(u32, extra_acked, ext->agg_extra_acked_max);
    {
        u32 conf_max = (u32)kcc_agg_confidence_max_val;          /* configured max confidence range */
        if (likely(thr < (int)conf_max)) {                                   /* threshold in valid range */
            comp = (u32)((u64)agg_est * (u32)(confidence - thr) / (conf_max - (u32)thr)); /* proportional: [thr,conf_max] → [0,agg_est] */
        }
        else {
            comp = 0;
        }
    }

    /* Layer 4: Hard cap at max_comp_ratio % of BDP */
    {
        u64 bdp64 = ((u64)bw * kcc->min_rtt_us) >> BW_SCALE;
        bdp = (u32)min_t(u64, bdp64, U32_MAX);
    }                      /* u64 cast prevents overflow */

    {
        u32 max_comp = (u32)((u64)bdp * (u32)kcc_agg_max_comp_ratio_val / KCC_PCT_BASE);  /* u64 for safety */
        comp = min_t(u32, comp, max_comp);
    }

    return comp;
}

/*
 * kcc_agg_watchdog - Demote confidence if compensation persists too long.
 * Called at round boundaries only (kcc->round_start == 1).
 * Also decays agg_extra_acked_max to prevent one spike from permanently
 * boosting confidence via Factor 4.
 * Does not return (state is modified in-place).
 */
static void kcc_agg_watchdog(struct sock* sk, struct kcc_ext* ext)
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);
    int max_dur;

    if (!ext || !kcc_agg_enable_val) {                            /* disabled or no ext */
        return;
    }

    /* Per-ACK gentle decay of windowed max: prevents a transient spike
     * (e.g. sudden burst) from inflating Factor 4 for an entire RTT
     * round.  Decays at value/denominator per ACK (both configurable).
     * Default 128/128 = 1.0 (no per-ACK decay). */
    {
        u32 per_ack = (u32)kcc_agg_max_per_ack_decay_val;
        u32 per_ack_den = (u32)kcc_agg_max_per_ack_decay_den_val;
        if (per_ack < per_ack_den && per_ack_den > 0) {
            ext->agg_extra_acked_max = (u32)((u64)ext->agg_extra_acked_max * per_ack / per_ack_den);
        }
    }

    if (!kcc->round_start) {                                                  /* only act on round boundaries */
        return;
    }

    /* Decay windowed max: 25% reduction per RTT to expire stale peaks */
    {
        u32 pct = (u32)kcc_agg_max_decay_pct_val;
        ext->agg_extra_acked_max = (u32)((u64)ext->agg_extra_acked_max * pct / KCC_PCT_BASE); /* u64 cast prevents overflow */
    }

    max_dur = kcc_agg_max_comp_duration_val;

    if (ext->agg_state >= KCC_AGG_CONFIRMED) {
        if (ext->agg_comp_duration < U8_MAX) {
            ext->agg_comp_duration++;
        }

        if ((u32)ext->agg_comp_duration > (u32)max_dur) {
            /* Demote: may be undetected congestion */
            ext->agg_state = KCC_AGG_SUSPECTED;
            ext->agg_comp_duration = 0;
        }
    }
    else {
        ext->agg_comp_duration = 0;
    }
}

/* ---- Model Update Pipeline (Cardwell et al. 2016) -------------------- */

/*
 * kcc_update_model - Execute the full per-ACK model update pipeline.
 *
 * @sk:  TCP socket.
 * @rs:  rate sample from the current ACK.
 * @ext: extended state (may be NULL if kzalloc failed at init).
 *
 * Processing order (reflects data dependencies):
 *   1. Bandwidth update (sliding-window max + LT BW).
 *   2. ECN-CE EWMA update (RFC 3168).
 *   3. ACK aggregation tracking.
 *   4. Cycle phase advance check (PROBE_BW only).
 *   5. Full BW reached detection.
 *   6. Drain check (STARTUP -> DRAIN -> PROBE_BW transition).
 *   7. Min RTT update (includes Kalman filter, traditional window, PROBE_RTT).
 *   8. Set pacing_gain and cwnd_gain based on current mode:
 *      - STARTUP:   high_gain / high_gain.
 *      - DRAIN:     drain_gain / high_gain (cwnd stays aggressive during drain).
 *      - PROBE_BW:  cycle_gain (or BBR_UNIT if LT BW active) / cwnd_gain_val.
 *      - PROBE_RTT: BBR_UNIT / BBR_UNIT (cruise, min inflight).
 *   9. Single-flow heuristic evaluation.
 *
 * Corresponds to kernel BBR v5.4: bbr_update_model().
 * Kernel BBR's pipeline order is identical (bw update, ack agg, cycle phase,
 * full_bw check, drain check, min_rtt, gain assignment).  KCC interleaves
 * ECN EWMA and single-flow evaluation at the same logical points.
 *
 * KCC deviations:
 *   a) ECN-CE EWMA (step 2b): kernel BBR does not maintain an ECN EWMA.
 *      BBR reacts to ECN per-ACK by reducing cwnd, similar to loss.
 *      KCC's EWMA enables proportional, graduated backoff rather than
 *      binary cwnd reduction.
 *   b) LT BW override in PROBE_BW gain assignment (step 8): when
 *      lt_use_bw is active, pacing_gain is locked at BBR_UNIT (1.0x)
 *      matching kernel BBR's behaviour exactly.
 *   c) Single-flow heuristic evaluation (step 9): kernel BBR has no
 *      equivalent alone_on_path detection.
 *
 * Type note: 'ext' may be NULL.  All sub-functions in the pipeline handle
 * NULL ext gracefully by falling back to no-Kalman-operation.  The gain
 * assignment switch uses bitfield reads for kcc->mode, kcc->cycle_idx,
 * etc.  Gains (kcc->pacing_gain, kcc->cwnd_gain) are u32:10 bitfields,
 * assigned from precomputed module-parameter caches (kcc_high_gain_val,
 * etc.) which are already in BBR_UNIT scale.
 */
static void kcc_update_model(struct sock* sk, const struct rate_sample* rs,            /* per-ACK model pipeline */
    struct kcc_ext* ext)                                                    /* extended state */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                       /* get KCC CA state */

    kcc_update_bw(sk, rs, ext);                                                              /* 1. sliding-window max bw */
    kcc_update_ecn_ewma(sk, rs, ext);                                                         /* 2b. ECN-CE mark ratio EWMA (RFC 3168) */
    kcc_update_ack_aggregation(sk, rs, ext);                                                   /* 3. ACK agg tracking */
    kcc_update_cycle_phase(sk, rs, ext);                                                        /* 4. PROBE_BW phase advance */
    kcc_check_full_bw_reached(sk, rs);                                                           /* 5. pipe-full detection */
    kcc_check_drain(sk, rs, ext);                                                                /* 6. drain transitions */
    kcc_update_min_rtt(sk, rs, ext);                                                             /* 7. min-RTT + Kalman + PROBE_RTT */

    /* Mode-specific gain assignment (Cardwell et al. 2016) */
    switch (kcc->mode) {                                                                          /* dispatch based on FSM mode */
    case KCC_STARTUP:                                                                          /* STARTUP mode */
        kcc->pacing_gain = kcc_high_gain_val;                                        /* pacing_gain approx 2.89x */
        kcc->cwnd_gain = kcc_high_gain_val;                                        /* cwnd_gain approx 2.89x */
        break;                                                                                   /* exit switch */
    case KCC_DRAIN:                                                                              /* DRAIN mode */
        kcc->pacing_gain = kcc_drain_gain_val;                                        /* pacing_gain approx 0.35x */
        kcc->cwnd_gain = kcc_high_gain_val;                                            /* cwnd at high_gain to keep cwnd (match BBR, Cardwell et al. 2016) */
        break;                                                                                     /* exit switch */
    case KCC_PROBE_BW:                                                                             /* PROBE_BW mode */
        if (kcc->lt_use_bw) {
            /* LT BW active: lock pacing at 1.0x to avoid exceeding
             * the policed rate.  Matches kernel BBR (net/ipv4/tcp_bbr.c):
             *   bbr->pacing_gain = (bbr->lt_use_bw ? BBR_UNIT :
             *       bbr_pacing_gain[bbr->cycle_idx]); */
            kcc->pacing_gain = BBR_UNIT;
        }
        else {                                                                                          /* normal PROBE_BW */
            if (likely(ext)) {
                kcc->pacing_gain = kcc_get_cycle_pacing_gain(sk, ext);                                   /* cycle gain with decay */
            }
            else {
                u32 idx = (u32)kcc->cycle_idx & (kcc_probe_bw_cycle_len_val - 1);           /* no ext: read table directly */
                kcc->pacing_gain = kcc_cycle_gain_table[idx];                              /* base gain, no decay possible */
            }
        }

        kcc->cwnd_gain = kcc_cwnd_gain_val;                                                /* baseline 2x */
        break;                                                                                           /* exit switch */
    case KCC_PROBE_RTT:                                                                                    /* PROBE_RTT mode */
        kcc->pacing_gain = BBR_UNIT;                                                                       /* cruise at 1.0x */
        kcc->cwnd_gain = BBR_UNIT;                                                                       /* cruise at 1.0x */
        break;                                                                                              /* exit switch */
    }

    /* Re-evaluate single-flow heuristic after all stats are fresh */
    kcc_alone_on_path_eval(sk, ext);
}
/*
 * kcc_alone_on_path_eval - Detect single-flow scenario for BBR-pure bypass.
 * @sk:  TCP socket.
 * @ext: extended state (for qdelay, jitter, ECN, agg state).
 *
 * Runs once per round boundary.  When all queues are nearly empty,
 * no ECN marks exist, and no ACK aggregation is confirmed, the flow
 * is likely alone on the bottleneck.  In this scenario, KCC's
 * protective mechanisms (Kalman model_rtt positive bias, ECN backoff)
 * reduce single-flow throughput compared to BBR.
 *
 * Evaluation is gated to cruise phase only (pacing_gain == BBR_UNIT).
 * Probe-up (1.25x) intentionally pushes the link — its queue pressure
 * is self-induced and not a competition signal.  Gating to cruise
 * eliminates the oscillation where self-induced probe pressure
 * falsely triggers alone-mode exit.
 *
 * When alone_on_path is set:
 *   - kcc_get_model_rtt returns kcc->min_rtt_us (BBR-style exact min),
 *     bypassing the Kalman smoothed estimate which has a small positive
 *     bias from one-sided measurement noise.
 *   - kcc_ecn_backoff returns immediately (no ECN reaction needed).
 *
 * The flag is cleared when any queue, ECN, or aggregation signal
 * appears during a cruise-phase evaluation — restoring full KCC
 * protection.
static void kcc_alone_on_path_eval(struct sock* sk,                            /* evaluate single-flow heuristic */
    struct kcc_ext* ext)                                                       /* extended state */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                            /* KCC CA state */

    /* Only re-evaluate on round boundaries for hysteresis.
     * Per-ACK evaluation would cause noise in the confirmation
     * counter and lead to oscillating back and forth between
     * normal and single-flow mode within a single RTT.  Round-boundary
     * evaluation provides ~1 RTT of natural hysteresis. */
    if (!kcc->round_start) {                                                    /* not a round boundary */
        return;                                                                 /* keep current state */
    }

    /* Without extended state (allocation failure at init), we cannot
     * evaluate the required signals (qdelay, jitter, ECN, agg).
     * Fall back to not-alone — safe, preserves KCC protection.
     * Clear the flag directly here; the counter lives in ext so
     * there is nothing to reset when ext is NULL. */
    if (!ext) {                                                                  /* no extended state available */
        kcc->alone_on_path = 0;                                                   /* exit single-flow mode */
        return;                                                                    /* no counter to reset */
    }

    /* Gain gating: only evaluate single-flow signals during cruise
     * phase (pacing_gain == BBR_UNIT = 1.0x).  Probe-up (1.25x)
     * intentionally pushes the bottleneck buffer — self-induced
     * queue pressure is expected behaviour, not a competition
     * signal.  Drain (0.75x) artificially suppresses the queue —
     * not a reliable reading.  STARTUP (2.89x) and DRAIN (0.35x)
     * are transient acceleration/deceleration — evaluation during
     * these phases is meaningless.
     *
     * Cruise is the steady-state equilibrium where BBR runs at the
     * estimated link capacity.  Clean signals during cruise genuinely
     * indicate no competing bulk traffic; dirty signals during cruise
     * indicate persistent queue pressure from another flow.
     *
     * By gating on cruise only, self-induced probe-up pressure does
     * not cause false exits — the oscillation "probe → queue pressure
     * → exit alone → conservative → clean → re-enter alone → probe"
     * is structurally damped. */
    if (kcc->pacing_gain != BBR_UNIT) {
        return;
    }

    /*
     * Single-flow indicators (five orthogonal signals):
     *
     * 0. sample_cnt >= kcc_kalman_min_samples_val:
     *    Kalman filter must have converged before we can trust
     *    qdelay_avg and jitter_ewma as meaningful queue signals.
     *    During early startup, these values are a random walk —
     *    acting on them would produce false positives.
     *
     * 1. qdelay_avg < kcc_alone_qdelay_thresh_us_val (default 1000 us):
     *    Queue must be nearly empty (< 1 ms by default).  BBR-style
     *    1 % pacing margin plus one TSO burst create ~1 - 2 ms of
     *    queue on a loaded path; a sub-millisecond queue means there
     *    is no competing bulk traffic.
     *
     * 2. jitter_ewma < kcc_alone_jitter_thresh_us_val (default 2000 us):
     *    Low packet - timing variance (< 2 ms by default).  Competing
     *    flows induce inter - packet gaps that push jitter well above
     *    this threshold.  A quiet single - flow path shows only
     *    ACK - clock micro - jitter.
     *
     * 3. ecn_ewma == 0:
     *    Zero congestion marks.  Any AQM(CoDel, FQ - CoDel, RED)
     *    marks packets when queue exceeds the marking threshold.
     *    Absence of marks implies an empty bottleneck buffer.
     *
     * 4. agg_state <= max per kcc_alone_agg_state_level_val:
     *    Configurable ACK aggregation strictness for alone detection.
     *    TCP delayed-ACK produces natural SUSPECTED-state aggregation
     *    even on a quiet single-flow path — requiring IDLE blocks
     *    alone mode in practice.  Three levels via sysctl:
     *      0 = IDLE only      (strict: zero aggregation)
     *      1 = ≤ SUSPECTED    (moderate: allow transient agg; default)
     *      2 = ≤ CONFIRMED    (permissive: block only trusted/persistent agg)
     *    Because CONFIRMED=2 and SUSPECTED=1 in KCC's enum, level 1
     *    and <CONFIRMED are equivalent; level 2 uses <TRUSTED for a
     *    genuinely more permissive tier.
     *
     * Entry: requires N consecutive qualifying rounds (hysteresis).
     * The confirmation counter increments once per round boundary
     * when all five conditions hold.  This prevents oscillation
     * during brief quiet windows in multi-flow competition —
     * "conservative to accelerate".
     *
     * Exit: immediate — any single qualification failure clears
     * the flag AND resets the counter.  Full KCC protection
     * (Kalman, ECN backoff, gain decay) re-engages on the first
     * sign of competition — "aggressive to brake".
     */
    {
        /* Map alone_agg_state_level to max allowed agg_state for comparison.
         * Level 0 = IDLE(0) only, Level 1 = ≤ SUSPECTED(1), Level 2 = ≤ CONFIRMED(2). */
        u8 max_agg;
        switch (kcc_alone_agg_state_level_val) {
        case 0: max_agg = KCC_AGG_IDLE; break;       /* strict: zero aggregation */
        case 2: max_agg = KCC_AGG_CONFIRMED; break;  /* permissive: allow up to CONFIRMED */
        default: max_agg = KCC_AGG_SUSPECTED; break; /* moderate: allow SUSPECTED (default) */
        }
        if (ext->sample_cnt >= kcc_kalman_min_samples_val &&                            /* Kalman must be converged */
            ext->qdelay_avg < (u32)kcc_alone_qdelay_thresh_us_val &&                   /* queue below configurable threshold */
            ext->jitter_ewma < (u32)kcc_alone_jitter_thresh_us_val &&                  /* jitter below configurable threshold */
            ext->ecn_ewma == 0 &&                                                       /* no ECN marks from AQM */
            ext->agg_state <= max_agg) {                                                /* configurable agg strictness */
            /* All five conditions hold on this round boundary.
             * Increment the consecutive confirmation counter.
             * Wrap at 255 (u8 max) to prevent overflow on connections
             * running millions of rounds in single-flow mode — the
             * counter only needs to reach confirm_rounds_val (≤ 32). */
            if (ext->alone_confirm_cnt < KCC_ALONE_CONFIRM_CNT_MAX) {                                      /* guard against u8 wrap */
                ext->alone_confirm_cnt++;                                             /* increment counter */
            }
            if (ext->alone_confirm_cnt >= (u8)kcc_alone_confirm_rounds_val) {        /* N rounds satisfied */
                kcc->alone_on_path = 1;                                               /* activate single-flow mode */
            }
        }
        else {
            /* At least one signal indicates competition or path stress.
             * Clear the flag IMMEDIATELY (aggressive brake) and reset
             * the confirmation counter so re-entry requires another
             * full N rounds of clean conditions. */
            kcc->alone_on_path = 0;                                                   /* exit single-flow mode */
            ext->alone_confirm_cnt = 0;                                               /* reset counter for re-entry */
        }
    }
}
/* ---- Global Kalman BDP startup (cross-connection init_bw) ----------- */
/*
 * Global Kalman-filtered bandwidth estimation.
 *
 * All connections on this host share a single Kalman filter that tracks
 * the steady-state available bandwidth.  Each PROBE_BW cruise-phase
 * sample (pacing_gain == BBR_UNIT) feeds this filter.  New connections
 * query kcc_kf_get_init_bw() to obtain a bootstrapped bandwidth estimate,
 * enabling immediate high-speed startup without the multi-RTT ramp-up
 * penalty of cold-start TCP.
 *
 * Architecture:
 *   kcc_kf_update()       — feed a BW sample (BW_UNIT) into the filter
 *   kcc_kf_get_init_bw()  — return the fair-share, gain-compensated
 *                            initial bandwidth for a new connection
 *
 * State is global (atomic64) because the estimation target — available
 * bandwidth on the bottleneck — is a shared resource across connections.
 */

 /*
  * kcc_kf_compute_R - Compute measurement noise covariance for the
  * Global Kalman BDP filter.
  * @z:   bandwidth sample in BW_UNIT units.
  * @pct: noise percentage (e.g. 5 for 5% of z).
  *
  * R = (z * pct / 100)^2
  *
  * The square models measurement noise as proportional to signal
  * magnitude — at higher bandwidths the absolute measurement jitter
  * is larger (heteroscedastic noise model, Kalman 1960).
  */
static u64 kcc_kf_compute_R(u64 z, u32 pct)                             /* compute measurement noise covariance */
{
    u64 r = z * (u64)pct / KCC_PCT_BASE;                                     /* linear noise: z * pct/100 */
    return r * r;                                                            /* squared => variance (BW_UNIT)^2 */
}

/*
 * kcc_kf_update - Feed a bandwidth sample into the Global Kalman BDP
 * filter using a one-dimensional random-walk model (Kalman 1960).
 * @z:      bandwidth sample in BW_UNIT units.
 * @r_pct:  measurement noise percentage (e.g. 5 for 5% of z).
 * @check:  if true, apply chi-squared innovation gate to reject
 *           transient upward spikes.
 *
 * Returns the updated state estimate x (BW_UNIT).
 *
 * State variables (global atomic64_t / atomic_t):
 *   kcc_kf_x      — estimated available bandwidth (BW_UNIT)
 *   kcc_kf_P      — error covariance
 *   kcc_kf_active — 1 = filter has been seeded
 *
 * Model:
 *   Predict:  x_{k|k-1} = x_{k-1}          (random walk, state constant)
 *             P_{k|k-1} = P_{k-1} + Q       (add process noise)
 *   Update:   K = P / (P + R)               (Kalman gain)
 *             x = x + K * (z - x)            (innovation-weighted update)
 *             P = (1 - K) * P               (Joseph form covariance)
 *
 * Equivalent algebraic form used here (avoids computing K explicitly):
 *             x = (x * R + z * P) / (P + R)
 *             P = (P * R) / (P + R)
 *
 * Fixed-point rescaling prevents 64-bit overflow when P+R exceeds
 * 2^31, using bit-shift scaling (INNOV_SHIFT / VAR_SHIFT = 2× cancel).
 */
static u64 kcc_kf_update(u64 z, u32 r_pct, bool check)                /* feed BW sample into global Kalman filter */
{                                                                          /* begin one-step Kalman filter */
    u64 P = atomic64_read(&kcc_kf_P);                                        /* load current error covariance */
    u64 x = atomic64_read(&kcc_kf_x);                                        /* load current state estimate */
    u64 R = kcc_kf_compute_R(z, r_pct);                                      /* compute measurement noise R */
    u32 shift = 0;                                                           /* bit-shift accumulator for overflow rescaling */
    u64 Pcopy, Rcopy, denom;                                                 /* local copies for rescaling; common denominator */
    s64 delta;                                                               /* signed innovation for chi-squared check */

    /* Predict step: P = P + Q  (random-walk process noise) */
    P += (1ULL << kcc_kf_q_shift_val);                                       /* add Q = 2^q_shift to covariance */

    /* First sample: seed the filter (cold start) */
    if (unlikely(!atomic_read(&kcc_kf_active))) {                            /* filter not yet seeded */
        atomic64_set(&kcc_kf_x, z);                                          /* seed state with first sample */
        atomic64_set(&kcc_kf_P, max(R, 1ULL));                               /* seed covariance, floor at 1 */
        atomic_set(&kcc_kf_active, 1);                                       /* mark filter as active */
        return z;                                                            /* return first estimate = sample */
    }

    /* Chi-squared innovation gate: reject outliers in BOTH directions.
     * A transient spike OR dip should not permanently distort the global
     * steady-state estimate shared across all connections.  The gate
     * compares (innovation² / expected_variance) against a chi-squared
     * threshold — rejecting samples where the deviation is statistically
     * implausible given the filter's current uncertainty P+R.
     *
     * For genuine bandwidth changes (e.g. VPS tenant contention), the
     * filter naturally tracks: as x drifts toward the new steady state,
     * the innovation for subsequent samples shrinks and passes the gate
     * within 2–4 rounds. */
    if (check) {                                                             /* chi-squared gate enabled */
        u64 nu2;                                                             /* innovation magnitude (squared later) */
        u64 S;                                                               /* total uncertainty = P + R */

        delta = (s64)z - (s64)x;                                             /* signed innovation: z - x */
        nu2 = (u64)(delta < 0 ? -delta : delta);                             /* absolute innovation magnitude */
        S = P + R;                                                           /* total uncertainty = P + R */
        if (S > 0) {                                                         /* guard zero division */
            nu2 = (nu2 >> KCC_KF_INNOV_SHIFT) * (nu2 >> KCC_KF_INNOV_SHIFT); /* downscale innovation, then square */
            S >>= KCC_KF_VAR_SHIFT;                                          /* downscale total uncertainty */
            if (S > 0 && nu2 / S >                                           /* chi-squared test: nu²/S > threshold */
                kcc_kf_chi2_num_val / kcc_kf_chi2_den_val) {                 /*   default 384/100 = 3.84 (p≈0.05) */
                return x;                                                    /* reject outlier, keep old estimate */
            }
        }
    }

    /* Fixed-point rescaling: copy P and R onto the stack for 64-bit
     * overflow-safe arithmetic.  If max(P, R, P+R) threatens to exceed
     * 2^31, right-shift all components (halving precision) until safe. */
    Pcopy = P;                                                               /* snapshot P for rescaling */
    Rcopy = R;                                                               /* snapshot R for rescaling */
    {
        u64 max_v = Pcopy;                                                   /* init: track largest component */
        if (Rcopy > max_v) {                                                 /* R is larger */
            max_v = Rcopy;                                                   /* update max to R */
        }

        if (Pcopy + Rcopy > max_v) {                                         /* P+R is the largest */
            max_v = Pcopy + Rcopy;                                           /* update max to P+R */
        }

        /* Fixed-point rescaling: if max(P, R, P+R) threatens to overflow
         * when multiplied in the Kalman update, right-shift all components
         * (halving precision) until the largest fits below the guard. */
        while (max_v >= KCC_KF_OVERFLOW_GUARD) {                             /* component(s) too large */
            Pcopy >>= 1; Rcopy >>= 1; max_v >>= 1; shift++;                  /* halve all components, count shifts */
        }
    }

    denom = Pcopy + Rcopy;                                                   /* denominator = P + R (Kalman weight normaliser) */
    if (denom == 0) {                                                        /* guard against zero denominator */
        return x;                                                            /* cannot update, keep old estimate */
    }

    /* Kalman update (algebraic form, no explicit gain K):
     *   x = (x*R + z*P) / (P+R)     weighted blend of prior and measurement
     *   P = (P*R) / (P+R)           posterior covariance (Joseph form) */
    x = (x * Rcopy + z * Pcopy) / denom;                                     /* innovation-weighted state update */
    P = Pcopy * Rcopy / denom;                                               /* posterior covariance update */

    /* Recover precision: undo any right-shift applied during rescaling */
    if (shift > 0) {                                                         /* precision was lost */
        P <<= shift;                                                         /* restore full-scale covariance */
    }

    /* Covariance floor: enforce minimum P ≥ Q to prevent lock-in.
     * Without this floor, after many updates P → 0 and the filter
     * ignores new measurements entirely (Kalman gain → 0). */
    {
        u64 q = 1ULL << kcc_kf_q_shift_val;                                  /* process noise Q = 2^q_shift */
        if (P < q) {                                                         /* below floor */
            P = q;                                                           /* floor covariance at Q */
        }
    }

    /* Store updated global state.  Only update when x > 0 because a
     * zero estimate provides no useful bandwidth floor. */
    if (x > 0) {                                                             /* valid estimate (positive BW) */
        atomic64_set(&kcc_kf_x, x);                                          /* publish new bandwidth estimate */
        atomic64_set(&kcc_kf_P, P);                                          /* publish new covariance */
    }
    return x;                                                                /* return updated estimate to caller */
}

/*
 * kcc_kf_get_init_bw - Return the fair-share, gain-compensated initial
 * bandwidth estimate for a new connection.
 * @sk: TCP socket (for cwnd/rate sanity checks).
 *
 * Returns a bandwidth estimate in BW_UNIT that a new connection should
 * use for initial pacing and cwnd seeding.  Returns 0 if:
 *   - Global Kalman BDP is disabled (kcc_kf_enable == 0)
 *   - The estimate is below the local cwnd-derived bandwidth floor
 *     (indicating the global estimate has drifted below the connection's
 *     already-probed capacity — the local path is faster)
 *
 * The estimate is discounted (kcc_kf_discount_num/den) and divided by
 * high_gain so that the caller can multiply by BBR_UNIT (neutral gain)
 * for a conservative initial pacing rate that doesn't overshoot the
 * global fair-share bottleneck.
 */
static u64 kcc_kf_get_init_bw(struct sock* sk)                   /* compute bootstrapped initial BW for new conn */
{
    struct tcp_sock* tp = tcp_sk(sk);                                    /* get TCP socket state */
    u64 fair, init_bw;                                                   /* fair-share estimate; discounted init BW */

    /* Guards: return 0 if KF disabled or not yet ready */
    if (!kcc_kf_enable_val || !atomic_read(&kcc_kf_active)) {              /* KF disabled or never seeded */
        return 0;                                                        /* no estimate available */
    }

    fair = (u64)atomic64_read(&kcc_kf_x);                                /* read global fair-share estimate */
    if (fair == 0) {                                                     /* estimate is zero */
        return 0;                                                        /* no useful information */
    }

    /* Discount and gain-compensate:
     *   init_bw = fair * discount_num / discount_den
     *           * BBR_UNIT / high_gain
     * The discount prevents overcommitment; the ÷high_gain allows
     * the caller to apply neutral gain without overdosing. */
    init_bw = fair * (u64)kcc_kf_discount_num_val / (u64)kcc_kf_discount_den_val;  /* apply safety discount */
    init_bw = init_bw * BBR_UNIT / (u64)kcc_high_gain_val;                         /* gain-compensate: divide out high_gain */

    /* Local BW check: if the connection's own cwnd/RTT already exceeds
     * the global estimate, the local path is faster — return 0 to let
     * the connection probe on its own without external guidance. */
    if (init_bw < (u64)tcp_snd_cwnd(tp) * (u64)BBR_UNIT / max_t(u32, tp->srtt_us >> 3, 1U)) { /* local BW > global estimate */
        return 0;                                                        /* global estimate is too conservative */
    }

    return init_bw;                                                      /* return gain-compensated init BW */
}

/* ---- Main Per-ACK Entry Point ----------------------------------------- */

/*
 * kcc_main - Main congestion control callback invoked on each ACK.
 *
 * @sk:  TCP socket.
 * @rs:  rate sample (delivered, interval_us, rtt_us, losses, etc.).
 * @ack: (kernel 6.10+ only) ACK number.
 * @flags: (kernel 6.10+ only) ACK flags.
 *
 * Processing sequence:
 *   1. ACK aggregation confidence evaluation (runs BEFORE model update
 *      so the Kalman filter sees fresh agg_confidence on the same ACK).
 *   2. Global Kalman BDP: feed PROBE_BW cruise-phase samples into the
 *      cross-connection bandwidth estimator.
 *   3. Run kcc_update_model (bandwidth/RTT/loss/Kalman/gain updates).
 *   4. Apply cwnd constraints (ECN backoff, etc.).
 *   5. Set pacing rate using current bw and pacing_gain.
 *   6. Set cwnd using current bw and cwnd_gain.
 *
 * This function reads global module-parameter caches (e.g. kcc_cwnd_gain_val,
 * kcc_kalman_q_val) without READ_ONCE.  This is deliberate:
 *   - A stale value affects at most one ACK; the next ACK corrects it.
 *   - All kernel CC modules (BBR, CUBIC, Westwood, etc.) do the same.
 *   - See "CONCURRENCY & SAFETY MODEL" at struct kcc_ext for the full
 *     justification.
 *
 * This is the single entry point for all KCC per-ACK processing.
 * The function is marked KCC_KFUNC for BPF struct_ops compatibility.
 *
 * Corresponds to kernel BBR v5.4: bbr_main().
 * Kernel BBR's bbr_main() sequence is: bbr_update_model, bbr_set_pacing_rate,
 * bbr_set_cwnd — identical steps 3, 5, 6 in the same order.
 *
 * KCC deviations:
 *   a) Step 1 (ACK aggregation confidence evaluation): kernel BBR does not
 *      have a confidence-gated aggregation system.  BBR's bbr_main() calls
 *      bbr_update_ack_aggregation() and bbr_ack_aggregation_cwnd() within
 *      bbr_update_model() and bbr_set_cwnd() respectively.  KCC evaluates
 *      agg_confidence BEFORE the model update so that the Kalman filter
 *      sees the updated agg_r_scaled (measurement noise inflation) on the
 *      same ACK where aggregation is detected, avoiding a 1-ACK lag.
 *   b) Step 2 (Global Kalman BDP): kernel BBR has no cross-connection
 *      bandwidth sharing.  KCC feeds cruise-phase BW samples into the
 *      global KF to maintain a shared estimate of available bottleneck
 *      bandwidth.  See kcc_kf_update for details.
 *   c) Step 4 (cwnd constraints via kcc_apply_cwnd_constraints): kernel
 *      BBR does not apply ECN EWMA-based backoff to cwnd_gain.  BBR reacts
 *      to ECN per-ACK through the standard TCP cwnd reduction path.
 *   BENEFIT of (a): Aligned detection and compensation on the same ACK;
 *   prevents one-ACK window where the Kalman filter over-trusts an
 *   aggregated RTT sample.  BENEFIT of (b): Fair-share initial bandwidth
 *   for new connections on shared bottlenecks.  BENEFIT of (c): Proactive,
 *   graduated ECN response versus BBR's reactive per-ACK reduction.
 *
 * Type note: The function signature varies by kernel version (6.10+ adds
 * ack and flags parameters).  Both signatures are wrapped by KCC_KFUNC
 * for BPF struct_ops.  The rate_sample parameter 'rs' is const* in both
 * versions, matching kernel BBR's bbr_main() signature.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)                                           /* kernel 6.10+ signature */
KCC_KFUNC void kcc_main(struct sock* sk, u32 ack __maybe_unused, int flags __maybe_unused, const struct rate_sample* rs) /* main ACK handler (6.10+) */
#else                                                                                           /* pre-6.10 signature */
KCC_KFUNC void kcc_main(struct sock* sk, const struct rate_sample* rs)                    /* main ACK handler (legacy) */
#endif
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                               /* get KCC CA state */
    struct kcc_ext* ext;                                                                             /* extended state (may be NULL) */
    u32 bw;                                                                                          /* active bandwidth estimate */

    ext = kcc_ext_get(sk);                                                                           /* retrieve ext (with UAF guard) */

    /* ACK aggregation confidence — must run BEFORE kcc_update_model
     * so that the Kalman filter sees fresh agg_confidence on the same
     * ACK where aggregation is first detected.  Previously evaluated
     * after the model update, which caused a 1-ACK lag: the aggregate
     * RTT sample polluted x_est on the detection ACK before R was
     * raised to compensate. */
    if (likely(kcc_agg_enable_val && ext)) {                                                     /* common case: agg enabled + ext valid */
        u32 extra = kcc_measure_ack_aggregation(sk, rs, ext);
        u16 conf = kcc_evaluate_agg_confidence(sk, ext, extra);
        ext->agg_confidence = conf;
        ext->agg_state = kcc_agg_state_from_confidence(conf);

        /* Watchdog runs AFTER confidence evaluation: demotions persist
         * until the next ACK, preventing the immediate re-promotion bug
         * where confidence scoring overwrites watchdog demotion.
         * Gated: round boundaries always, otherwise only if per-ACK
         * max decay is active (default disabled). */
        if (kcc->round_start || kcc_agg_per_ack_decay_active) {
            kcc_agg_watchdog(sk, ext);
        }
    }

    /* Global Kalman BDP: feed PROBE_BW cruise-phase bandwidth samples
     * into the cross-connection filter.  Cruise phase (gain == BBR_UNIT)
     * provides the cleanest signal of true available bandwidth — no
     * 1.25× overshoot or 0.75× undershoot.  Round boundaries gate
     * feeding to one sample per RTT for numerical stability. */
    if (kcc_kf_enable_val && kcc->round_start &&                                 /* KF enabled + round boundary */
        kcc->mode == KCC_PROBE_BW && kcc->pacing_gain == BBR_UNIT && rs->interval_us > 0) {  /* cruise phase, valid interval */
        u64 bw = (u64)rs->delivered * BW_UNIT / (u64)rs->interval_us;            /* compute instantaneous BW from rate sample */
        if (!atomic_read(&kcc_kf_active)) {                                      /* filter not yet seeded */
            kcc_kf_update(bw, (u32)kcc_kf_startup_r_pct_val, false);            /* seed with startup R pct, no chi2 gate */
        }
        else {                                                                   /* filter already active */
            kcc_kf_update(bw, (u32)kcc_kf_steady_r_pct_val, true);              /* feed with steady R pct, chi2 gate ON */
        }
    }

    kcc_update_model(sk, rs, ext);                                                                   /* full per-ACK model update */

    kcc_apply_cwnd_constraints(sk, ext);                                                              /* loss/qdelay-based cap on cwnd_gain */

    bw = kcc_bw(sk);                                                                                   /* active bw estimate (max_bw or lt_bw) */
    kcc_set_pacing_rate(sk, bw, kcc->pacing_gain);                                                       /* update sk_pacing_rate */

    kcc_set_cwnd(sk, rs, rs->acked_sacked,                                                           /* update tp->snd_cwnd: acked_sacked is u32, always >= 0 */
        bw, kcc->cwnd_gain, ext);                                                                      /* using bw, cwnd_gain, ext */
}
/* ---- Module Callbacks -------------------------------------------------- */

/*
 * kcc_init - Initialize per-connection KCC state when a connection starts
 * using the "kcc" congestion-control algorithm.
 *
 * @sk: TCP socket.
 *
 * Steps:
 *   1. Zero-initialize the struct kcc (ICSK_CA_PRIV slot).
 *   2. Set prev_ca_state = TCP_CA_Open.
 *   3. Bootstrap min_rtt_us from the TCP stack's recorded min RTT.
 *   4. Set min_rtt_stamp to now.
 *   5. Initialize pacing rate from cwnd and SRTT.
 *   6. Enable pacing on the socket.
 *   7. (KCC extension) Global Kalman BDP injection: if the cross-connection
 *      KF has a valid estimate, seed the bandwidth tracker and set initial
 *      cwnd/pacing to the fair-share rate, bypassing cold-start ramp-up.
 *   8. Allocate and initialize extended state (struct kcc_ext) on the heap.
 *      On allocation failure, KCC runs without Kalman/ACK-agg features
 *      (fallback to sliding-window-only min_rtt).
 *
 * Corresponds to kernel BBR v5.4: bbr_init().
 * The core initialisation steps 1-6 match kernel BBR's bbr_init() exactly:
 * memset, snd_ssthresh = TCP_INFINITE_SSTHRESH, prev_ca_state = Open,
 * next_rtt_delivered = 0, min_rtt_us from tcp_min_rtt, min_rtt_stamp,
 * kcc_init_pacing_rate_from_rtt(), and sk_pacing_status enable.
 *
 * KCC deviations:
 *   a) Step 7 (Global Kalman BDP injection): kernel BBR has no
 *      cross-connection bandwidth sharing.  When kcc_kf_enable is active
 *      and the global KF has converged, kcc_init() seeds the sliding-window
 *      max-bw filter, the pacing rate, and the initial cwnd from the
 *      global fair-share estimate.  This enables "dessert-speed" startup
 *      at the fair-share rate without the multi-RTT ramp-up of cold TCP.
 *      WHY: On shared bottlenecks, each new BBR connection spends 2-4 RTTs
 *      in STARTUP before reaching the fair-share rate.  With many short
 *      connections, this wastes bottleneck capacity and increases latency.
 *      The global KF provides a common fair-share estimate learned from
 *      all connections, allowing new connections to start at the correct
 *      rate immediately.
 *      BENEFIT: Near-zero cold-start penalty for short connections on
 *      shared bottlenecks.  STARTUP probes above the fair-share rate if
 *      additional capacity is available, so the KF injection is a floor,
 *      not a ceiling.
 *   b) Step 8: Extended state allocation on the heap (struct kcc_ext).
 *      Kernel BBR stores all state in the in-sock ICSK_CA_PRIV slot (struct
 *      bbr, 104 bytes on x86_64).  KCC's Kalman filter, ECN EWMA, ACK-agg
 *      confidence, and dynamically computed fields exceed this budget, so
 *      extended state is heap-allocated.  On kzalloc failure, KCC degrades
 *      gracefully to sliding-window-only min_rtt (no Kalman, no ACK-agg
 *      confidence, no ECN EWMA, no single-flow detection).
 *      WHILE neither function exists as a separate function in KCC, the
 *      two reset behaviours (re-enter STARTUP vs. enter PROBE_BW) that
 *      BBR splits into bbr_reset_startup_mode and bbr_reset_probe_bw_mode
 *      are both handled by kcc_reset_mode() with the full_bw_reached branch.
 *      BENEFIT: Graceful degradation on memory pressure; full feature set
 *      when allocation succeeds.
 *
 * Type note: struct kcc is zeroed via memset, which also zeros all bitfields
 * (mode = KCC_STARTUP = 0, flags = 0).  snd_ssthresh is set to
 * TCP_INFINITE_SSTHRESH to prevent the stack from imposing its own cwnd
 * clamp.  min_rtt_us is u32, initialised from tcp_min_rtt() which returns
 * u32.  Extended state is allocated with GFP_KERNEL (may sleep).  The
 * function is marked KCC_KFUNC for BPF struct_ops compatibility.
 */
KCC_KFUNC void kcc_init(struct sock* sk)                                             /* per-connection init callback */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                           /* get CA private area */
    struct kcc_ext* ext;                                                                         /* extended state pointer */

    memset(kcc, 0, sizeof(*kcc));                                                                /* zero the CA private area */
    /* Match BBR: set snd_ssthresh to TCP_INFINITE_SSTHRESH so the TCP stack
     * never imposes its own ssthresh-based cwnd clamp.  KCC manages cwnd
     * entirely through its own state machine (STARTUP/DRAIN/PROBE_BW).
     * Without this, memset zeros ssthresh → 0, which can prematurely
     * limit cwnd in several TCP stack code paths. */
    tcp_sk(sk)->snd_ssthresh = TCP_INFINITE_SSTHRESH;
    kcc->prev_ca_state = TCP_CA_Open;                                                             /* initial CA state: Open */
    kcc->next_rtt_delivered = 0;                                                                                   /* match BBR: first ACK starts round 1 immediately (Cardwell et al. 2016) */
    /* Bootstrap min_rtt_us from TCP stack's 3WHS measurement, matching BBR's
     * bbr->min_rtt_us = tcp_min_rtt(tp).  Without this, kcc_bdp() returns
     * TCP_INIT_CWND (~10) until the first RTT sample arrives, starving cwnd. */
    kcc->min_rtt_us = tcp_min_rtt(tcp_sk(sk));
    /* KCC: dessert-speed fast startup — SRTT fallback only when global KF
     * (cross-connection bandwidth sharing) is active.  Otherwise match BBR. */
    if (kcc->min_rtt_us == 0 && kcc_kf_enable_val) {
        struct tcp_sock* tp = tcp_sk(sk);
        kcc->min_rtt_us = tp->srtt_us ? tp->srtt_us >> 3 : USEC_PER_MSEC;
    }
    kcc->min_rtt_stamp = tcp_jiffies32;                                                            /* set initial min_rtt timestamp */

    kcc_init_pacing_rate_from_rtt(sk);                                                              /* initial pacing rate from RTT+cwnd */
    cmpxchg(&sk->sk_pacing_status, SK_PACING_NONE, SK_PACING_NEEDED);                       /* enable pacing, preserve if already set */

    /* Step 1 (Global Kalman BDP): inject cross-connection bandwidth
     * estimate to bootstrap this connection at line rate, bypassing the
     * multi-RTT cold-start ramp-up penalty.  kcc_kf_get_init_bw() returns
     * a gain-compensated value (already ÷ high_gain) so STARTUP's
     * 2.89× gain on the next pacing call hits the fair-share estimate. */
    if (kcc_kf_enable_val && atomic_read(&kcc_kf_active)) {                     /* KF enabled and active */
        u64 init_bw = kcc_kf_get_init_bw(sk);                                   /* get gain-compensated init BW */
        if (init_bw > 0) {                                                      /* valid estimate available */
            struct tcp_sock* tp = tcp_sk(sk);                                    /* get TCP socket state */
            /* Seed the bandwidth tracker with the global estimate */
            minmax_running_max(&kcc->bw, kcc_bw_rt_cycle_len_val, 0, (u32)init_bw); /* seed sliding-window max with KF estimate */
            /* Pacing: neutral gain (BBR_UNIT).  STARTUP will apply
             * high_gain on the first kcc_set_pacing_rate() call. */
            WRITE_ONCE(sk->sk_pacing_rate, kcc_bw_to_pacing_rate(sk, init_bw, BBR_UNIT));
            /* CWND: BDP from the pre-gain init_bw.  Use the standard KCC
             * BDP function with neutral gain for a conservative initial
             * window that doesn't overshoot the bottleneck. */
            {
                u32 init_cwnd = kcc_bdp(sk, (u32)init_bw, BBR_UNIT, NULL);     /* BDP at neutral gain */
                init_cwnd = clamp_t(u32, init_cwnd, TCP_INIT_CWND, KCC_KF_CWND_SEGS_MAX); /* clamp to valid range */
                tp->snd_cwnd = init_cwnd;                                /* seed cwnd with KF-guided BDP */
            }
            kcc->has_seen_rtt = 1;                                              /* mark: bandwidth seen (prevents RTT bootstrap) */
        }
    }

    /* Match BBR: reset LT BW sampling state so lt_last_stamp and lt_last_delivered
     * start from current delivery, not zero (bbr_init:1065). */
    kcc_reset_lt_bw_sampling_interval(sk);

    ext = kzalloc(sizeof(*ext), GFP_KERNEL);                                                          /* allocate extended state block (kzalloc zeroes memory) */
    if (likely(ext)) {                                                                                  /* allocation succeeded */

        ext->p_est = kcc_kalman_p_est_init_val;                                                 /* initialize Kalman covariance (Kalman 1960) */
        ext->q_est = (u32)kcc_kalman_q_val;                                                       /* init Q estimate to base process noise */
        ext->r_est = (u32)kcc_kalman_r_val;                                                       /* init R estimate to base measurement noise */
        ext->ecn_ewma = 0;                                                                                   /* init ECN EWMA: no CE marks yet */
        ext->last_delivered_ce = tcp_sk(sk)->delivered_ce;                                                    /* snapshot initial CE counter */
        ext->ack_epoch_mstamp = tcp_sk(sk)->tcp_mstamp;                                                     /* start aggregation epoch timestamp */
        ext->agg_extra_acked = 0;
        ext->agg_extra_acked_max = 0;
        ext->agg_confidence = 0;
        ext->agg_state = KCC_AGG_IDLE;
        ext->agg_comp_duration = 0;
        ext->agg_r_scaled = kcc_agg_r_multiplier_min_val;                 /* start at configured R floor */
        ext->x_est = 0;                             /* kalman cold start sentinel */
        ext->sample_cnt = 0;                         /* no accepted samples yet */
        ext->consec_reject_cnt = 0;                  /* no rejections yet */
        ext->dyn_probe_rtt_interval_jiffies = 0;      /* disabled until kalman converges */
        kcc->ext = ext;
    }
    else {                                                                                                   /* allocation failed */
        pr_warn_once("KCC: ext alloc failed, advanced features disabled\n");                                     /* warn: running degraded */
    }
}
/*
 * kcc_sndbuf_expand - Return the factor by which the socket send buffer
 * should be expanded relative to cwnd.
 * @sk: TCP socket.
 *
 * Returns the configurable sndbuf expansion factor (default 3× cwnd, BBR standard).
 * This provides enough buffer for pacing without head-of-line blocking.
 */
KCC_KFUNC u32 kcc_sndbuf_expand(struct sock* sk)                                       /* send buffer expansion factor */
{
    return (u32)kcc_sndbuf_expand_factor_val;                                  /* configurable sndbuf expansion factor */
}
/*
 * kcc_undo_cwnd - Handle a TCP undo operation (spurious loss detection).
 * @sk: TCP socket.
 *
 * Resets full_bw detection state and LT BW sampling, then returns the
 * current cwnd (the stack will decide the actual undo).
 */
KCC_KFUNC u32 kcc_undo_cwnd(struct sock* sk)                                             /* handle spurious loss undo */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                               /* get KCC CA state */
    /* Match BBR: reset only full_bw and full_bw_cnt, NOT full_bw_reached.
     * BBR's rationale (Cardwell et al. 2016): once the pipe is known to be
     * full, a spurious loss detection (e.g., reordering misinterpreted as
     * loss) does NOT change the pipe capacity.  Clearing full_bw_reached
     * forces a re-entry into STARTUP mode, causing massive overshoot and
     * unnecessary queue buildup after each spurious undo event. */
    kcc->full_bw = 0;                                                                                /* reset full_bw estimate */
    kcc->full_bw_cnt = 0;                                                                            /* reset full_bw counter */
    kcc_reset_lt_bw_sampling(sk);                                                                      /* clear LT BW state */
    return tcp_snd_cwnd(tcp_sk(sk));                                                                        /* return current cwnd */
}
/*
 * kcc_ssthresh - Return the slow-start threshold after a loss event.
 * @sk: TCP socket.
 *
 * Saves cwnd for later restoration via kcc_save_cwnd(), then returns
 * the current ssthresh (KCC does not modify ssthresh on its own;
 * the TCP stack uses the current value).
 */
KCC_KFUNC u32 kcc_ssthresh(struct sock* sk)                                               /* ssthresh query after loss */
{
    kcc_save_cwnd(sk);                                                                              /* save cwnd for later restore */
    return tcp_sk(sk)->snd_ssthresh;                                                                 /* return current ssthresh */
}
/* ---- Diagnostic Encoding (standard BBR format) ----------------------- */

/*
 * kcc_get_info - Encode KCC state for diagnostic tools (e.g., ss -i).
 * @sk:       TCP socket.
 * @ext_mask: INET_DIAG extension bitmask.
 * @attr:     [out] diagnostic attribute type (INET_DIAG_BBRINFO).
 * @info:     [out] union tcp_cc_info to fill.
 *
 * Outputs a struct tcp_bbr_info compatible with standard BBR diagnostics
 * (Cardwell et al. 2016):
 *   - bbr_bw_lo / bbr_bw_hi: 64-bit bandwidth in bytes/s (via mss_cache conversion).
 *   - bbr_min_rtt:           current min_rtt_us (Kalman or window-based).
 *   - bbr_pacing_gain / bbr_cwnd_gain: current gains in BBR_UNIT.
 *
 * Returns 0 if neither BBR nor VEGAS diagnostic extensions are requested.
 */
static size_t kcc_get_info(struct sock* sk, u32 ext_mask, int* attr,                           /* encode diagnostics for ss */
    union tcp_cc_info* info)                                                         /* output info struct */
{
    if (ext_mask & (1 << (INET_DIAG_BBRINFO - 1)) ||                                              /* BBR_INFO bit set OR */
        ext_mask & (1 << (INET_DIAG_VEGASINFO - 1))) {                                              /* VEGAS_INFO bit set */
        struct tcp_sock* tp = tcp_sk(sk);                                                            /* get TCP socket state */
        struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                              /* get KCC CA state */
        u64 bw_raw;
        u64 bw;
        if (unlikely(!tp->mss_cache)) {
            return 0;
        }

        bw_raw = (u64)kcc_bw(sk) * tp->mss_cache;
        if (bw_raw > U64_MAX / USEC_PER_SEC) {
            bw = U64_MAX;
        }
        else {
            bw = (bw_raw * USEC_PER_SEC) >> BW_SCALE;
        }

        memset(&info->bbr, 0, sizeof(info->bbr));                                                       /* zero the BBR info struct */
        info->bbr.bbr_bw_lo = (u32)bw;                                       /* low 32 bits of BW (plain truncation, matches BBR) */
        info->bbr.bbr_bw_hi = (u32)(bw >> KCC_MSTAMP_HI_SHIFT);                                                     /* high 32 bits of BW */
        info->bbr.bbr_min_rtt = kcc->min_rtt_us;                                                     /* min RTT in us */
        info->bbr.bbr_pacing_gain = kcc->pacing_gain;                                                     /* pacing gain (BBR_UNIT) */
        info->bbr.bbr_cwnd_gain = kcc->cwnd_gain;                                                       /* cwnd gain (BBR_UNIT) */

        *attr = INET_DIAG_BBRINFO;                                                                          /* set diagnostic attribute type */
        return sizeof(info->bbr);                                                                            /* return size of BBR info */
    }
    return 0;                                                                                                 /* no diagnostics requested */
}
/*
 * kcc_set_state - Handle TCP CA state transitions (Open, Disorder, Recovery, Loss).
 * @sk:        TCP socket.
 * @new_state: new CA state.
 *
 * On TCP_CA_Loss (RTO timeout or SACK loss):
 *   - Reset full_bw and full_bw_cnt (allow redetection of peak bandwidth).
 *   - full_bw_reached and FSM mode are preserved: loss does not shrink pipe
 *     capacity; re-entering STARTUP on every loss would cause overshoot.
 *   - If not in LT BW mode, seed LT BW sampling with a synthetic loss event.
 *   - Set round_start to 1, clear packet_conservation flag.
 */
KCC_KFUNC void kcc_set_state(struct sock* sk, u8 new_state)                               /* handle CA state transitions */
{
    struct kcc* kcc = (struct kcc*)inet_csk_ca(sk);                                                /* get KCC CA state */

    if (new_state == TCP_CA_Loss) {
        /* Match BBR (Cardwell et al. 2016): on TCP_CA_Loss, reset full_bw
         * (to allow redetection of the new peak bandwidth) but do NOT clear
         * full_bw_reached or change the FSM mode.  The pipe capacity doesn't
         * suddenly shrink on loss — forcing a re-entry to STARTUP would cause
         * massive overshoot and unnecessary DRAIN cycles. */
        struct rate_sample rs = { .losses = 1 };

        kcc->prev_ca_state = TCP_CA_Loss;
        kcc->full_bw = 0;
        kcc->round_start = 1;    /* treat RTO like end of a round */
        kcc_lt_bw_sampling(sk, &rs);
    }
}
/* ---- Congestion Ops Structure ----------------------------------------- */

/*
 * tcp_kcc_cong_ops - Registration structure for the "kcc" congestion control
 * algorithm in the Linux TCP stack.
 *
 * Fields mapped to the KCC implementation:
 *   .flags          = TCP_CONG_NON_RESTRICTED (no CAP_NET_ADMIN required)
 *   .name           = "kcc" (algorithm name for setsockopt)
 *   .init           = kcc_init (per-connection state allocation)
 *   .release        = kcc_release (per-connection state deallocation)
 *   .cong_control   = kcc_main (main per-ACK callback, Cardwell et al. 2016)
 *   .sndbuf_expand  = kcc_sndbuf_expand (send buffer sizing factor)
 *   .undo_cwnd      = kcc_undo_cwnd (spurious loss undo)
 *   .cwnd_event     = kcc_cwnd_event (congestion event handler)
 *   .ssthresh       = kcc_ssthresh (slow-start threshold query)
 *   .min_tso_segs   = kcc_min_tso_segs (minimum TSO segments)
 *   .get_info       = kcc_get_info (diagnostic state encoding)
 *   .set_state      = kcc_set_state (CA state transition handler)
 */
static struct tcp_congestion_ops tcp_kcc_cong_ops __read_mostly = {                      /* KCC CC ops registration */
    .flags = TCP_CONG_NON_RESTRICTED,                                             /* any user may select this CC */
    .name = "kcc",                                                                /* algorithm name for setsockopt() */
    .owner = THIS_MODULE,                                                           /* module owner reference */
    .init = kcc_init,                                                               /* connection init callback */
    .release = kcc_release,                                                             /* connection close callback */
    .cong_control = kcc_main,                                                                 /* per-ACK processing (Cardwell et al. 2016) */
    .sndbuf_expand = kcc_sndbuf_expand,                                                        /* sndbuf scaling factor */
    .undo_cwnd = kcc_undo_cwnd,                                                             /* cwnd undo handler */
    .cwnd_event = kcc_cwnd_event,                                                              /* CA events handler */
    .ssthresh = kcc_ssthresh,                                                                 /* ssthresh query */
    .min_tso_segs = kcc_min_tso_segs,                                                              /* minimum TSO segs */
    .get_info = kcc_get_info,                                                                    /* diagnostics (ss -i) */
    .set_state = kcc_set_state,                                                                    /* CA state transitions */
};                                                                                                        /* tcp_kcc_cong_ops */

/* ---- Sysctl Interface -------------------------------------------------- */

/*
 * Sysctl table header (registered at /proc/sys/net/kcc/ entries).
 * All entries use the custom kcc_proc_handler which chains to
 * proc_dointvec() and then calls kcc_init_module_params() to
 * recompute all derived values after any write.
 */
static struct ctl_table_header* kcc_ctl_header;                                                       /* sysctl table registration cookie */

/*
 * kcc_proc_handler - Per-entry sysctl handler.
 *
 * Calls proc_dointvec() for standard integer read/write.
 * After a successful write, triggers kcc_init_module_params() to
 * recompute clamped/derived values.
 */
static int kcc_proc_handler(struct ctl_table* ctl, int write,                                          /* sysctl per-entry handler */
    void* buffer, size_t* lenp, loff_t* ppos)                                               /* standard sysctl signature */
{
    int ret = proc_dointvec(ctl, write, buffer, lenp, ppos);                                             /* delegate to kernel handler */
    if (write && ret == 0) {                                                                              /* write succeeded */
        kcc_init_module_params();                                                                           /* re-validate + recompute derived */
    }

    return ret;                                                                                              /* return proc_dointvec result */
}
/*
 * kcc_ctl_table - Sysctl table of all KCC module parameters.
 * The .procname entries are exposed as /proc/sys/net/kcc/ + procname.
 * Array-type parameters (gain_num, gain_den, cycle_decay_mask) use
 * kcc_gain_proc_handler which additionally calls kcc_rebuild_gain_table().
 * A sentinel entry (empty .procname) marks the end of the table.
 */
static struct ctl_table kcc_ctl_table[] = {                                                              /* KCC sysctl registration table */
    /* PROBE_RTT intervals */
    {.procname = "kcc_probe_rtt_base_sec",      .data = &kcc_probe_rtt_base_sec,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..86400s] base PROBE_RTT interval; matches kernel BBR's fixed 10s interval; used when Kalman not converged */
    {.procname = "kcc_probe_rtt_max_sec",       .data = &kcc_probe_rtt_max_sec,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..86400s] max PROBE_RTT interval for long-RTT paths; KCC extension beyond BBR's fixed interval */
    {.procname = "kcc_probe_rtt_dyn_max_sec",   .data = &kcc_probe_rtt_dyn_max_sec,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..86400s] max dynamic interval when Kalman converged; 0=disabled; KCC extension */
    /* CWND and ACK-agg gains */
    {.procname = "kcc_cwnd_gain_num",           .data = &kcc_cwnd_gain_num,           .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100000] CWND gain numerator; effective gain = num/den*BBR_UNIT; BBR default: 2 (2x BDP) */
    {.procname = "kcc_cwnd_gain_den",           .data = &kcc_cwnd_gain_den,           .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100000] CWND gain denominator; BBR default: 1 */
    {.procname = "kcc_extra_acked_gain_num",    .data = &kcc_extra_acked_gain_num,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100000] ACK-agg gain numerator; 0 disables compensation; KCC extension */
    {.procname = "kcc_extra_acked_gain_den",    .data = &kcc_extra_acked_gain_den,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100000] ACK-agg gain denominator; KCC extension */
    /* PROBE_BW gain table arrays (use kcc_gain_proc_handler for rebuild) */
    {.procname = "kcc_gain_num",            .data = kcc_gain_num,            .maxlen = sizeof(kcc_gain_num),          .mode = 0644, .proc_handler = kcc_gain_proc_handler }, /* [256 entries] PROBE_BW cycle gain numerators; default BBR pattern: 5,3,1,1,... (1.25x,0.75x,1.0x) */
    {.procname = "kcc_gain_den",            .data = kcc_gain_den,            .maxlen = sizeof(kcc_gain_den),          .mode = 0644, .proc_handler = kcc_gain_proc_handler }, /* [256 entries] PROBE_BW cycle gain denominators; default BBR pattern: 4,4,1,1,... */
    {.procname = "kcc_cycle_decay_mask",    .data = kcc_cycle_decay_mask,    .maxlen = sizeof(kcc_cycle_decay_mask),  .mode = 0644, .proc_handler = kcc_gain_proc_handler }, /* [8 words=256 bits] decay mask bitmap; 1=slot eligible for qdelay/jitter-based gain decay; default: all 0 (no decay); KCC extension */
    /* Kalman base noise (Kalman 1960) */
    {.procname = "kcc_kalman_q",               .data = &kcc_kalman_q,               .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100000] base process noise covariance Q; KCC-only: kernel BBR has no Kalman filter */
    {.procname = "kcc_kalman_r",               .data = &kcc_kalman_r,               .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100000] base measurement noise covariance R; KCC-only; default 400 */
    /* STARTUP and DRAIN gains (Cardwell et al. 2016) */
    {.procname = "kcc_high_gain_num",           .data = &kcc_high_gain_num,           .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100000] STARTUP pacing gain numerator; BBR default: 2885 (≈2.885x) */
    {.procname = "kcc_high_gain_den",           .data = &kcc_high_gain_den,           .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100000] STARTUP pacing gain denominator; BBR default: 1000 */
    {.procname = "kcc_drain_gain_num",          .data = &kcc_drain_gain_num,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100000] DRAIN pacing gain numerator; BBR default: 347 (≈0.347x = 1/high_gain) */
    {.procname = "kcc_drain_gain_den",          .data = &kcc_drain_gain_den,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100000] DRAIN pacing gain denominator; BBR default: 1000 */
    /* PROBE_BW cycle and full-BW detection */
    {.procname = "kcc_probe_bw_cycle_len",      .data = &kcc_probe_bw_cycle_len,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [2..256] PROBE_BW cycle length (rounded to power-of-two); BBR default: 8 */
    {.procname = "kcc_full_bw_thresh_num",      .data = &kcc_full_bw_thresh_num,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100000] full-BW growth threshold numerator; BBR default: 125 (1.25x) */
    {.procname = "kcc_full_bw_thresh_den",      .data = &kcc_full_bw_thresh_den,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100000] full-BW growth threshold denominator; BBR default: 100 */
    {.procname = "kcc_full_bw_cnt",             .data = &kcc_full_bw_cnt,             .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..3] consecutive rounds without growth to declare full_bw; BBR default: 3 */
    /* Pacing margin */
    {.procname = "kcc_pacing_margin_num",       .data = &kcc_pacing_margin_num,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..50] pacing margin numerator; divisor=100-num*100/den; BBR default: 1 (1%) */
    {.procname = "kcc_pacing_margin_den",       .data = &kcc_pacing_margin_den,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100000] pacing margin denominator; BBR default: 100 */
    /* Inflight gain bounds */
    {.procname = "kcc_inflight_low_gain_num",   .data = &kcc_inflight_low_gain_num,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100000] inflight lower bound numerator (default 100 = 1.0x BDP); KCC: 1.0x vs BBRv2's 1.25x */
    {.procname = "kcc_inflight_low_gain_den",   .data = &kcc_inflight_low_gain_den,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100000] inflight lower bound denominator; BBR default: 100 */
    {.procname = "kcc_inflight_high_gain_num",  .data = &kcc_inflight_high_gain_num,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100000] inflight upper bound numerator (default 200 = 2.0x BDP); BBR default: 200 */
    {.procname = "kcc_inflight_high_gain_den",  .data = &kcc_inflight_high_gain_den,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100000] inflight upper bound denominator; BBR default: 100 */
    /* Kalman bounds (Kalman 1960) */
    {.procname = "kcc_kalman_p_est_max",        .data = &kcc_kalman_p_est_max,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100M] p_est absolute max covariance; KCC-only; default 1,000,000 */
    {.procname = "kcc_kalman_converged_p_est",  .data = &kcc_kalman_converged_p_est,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..1M] p_est convergence threshold; KCC-only; default 500 */
    {.procname = "kcc_recal_p_est_thresh",       .data = &kcc_recal_p_est_thresh,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100M] p_est threshold for PROBE_RTT recalibration trigger; KCC-only; default 250,000 */
    {.procname = "kcc_drain_skip_qdelay_us",     .data = &kcc_drain_skip_qdelay_us,     .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100k us] qdelay below which DRAIN phase is skipped; KCC-only; default 1000us */
    {.procname = "kcc_kalman_q_boost_mult",     .data = &kcc_kalman_q_boost_mult,     .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..10k] Q-boost threshold multiplier; KCC-only; default 4 */
    {.procname = "kcc_kalman_q_max",            .data = &kcc_kalman_q_max,            .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] adaptive Q max ceiling; KCC-only; default 2000 */
    {.procname = "kcc_kalman_q_scale_cap",      .data = &kcc_kalman_q_scale_cap,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..10k] Q adaptation factor cap (min_rtt_us/1000); KCC-only; default 20 */
    {.procname = "kcc_kalman_min_samples",      .data = &kcc_kalman_min_samples,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [3..20] min Kalman samples before min_rtt takeover; KCC-only; default 5 */
    /* RTT / min-RTT tracking */
    {.procname = "kcc_rtt_sample_max_us",       .data = &kcc_rtt_sample_max_us,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..10M us] max RTT sample accepted by Kalman filter; KCC-only; default 500,000us */
    {.procname = "kcc_minrtt_fast_fall_cnt",    .data = &kcc_minrtt_fast_fall_cnt,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..3] consecutive fast-fall samples for immediate min_rtt drop; KCC extension; default 3 */
    {.procname = "kcc_minrtt_sticky_num",       .data = &kcc_minrtt_sticky_num,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..1000] sticky min_rtt decrease numerator; KCC extension; default 75 */
    {.procname = "kcc_minrtt_sticky_den",       .data = &kcc_minrtt_sticky_den,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] sticky min_rtt decrease denominator; KCC extension; default 100 */
    {.procname = "kcc_minrtt_srtt_guard_num",   .data = &kcc_minrtt_srtt_guard_num,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..1000] SRTT guard ratio numerator; KCC extension; default 90 */
    {.procname = "kcc_minrtt_srtt_guard_den",   .data = &kcc_minrtt_srtt_guard_den,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] SRTT guard ratio denominator; KCC extension; default 100 */
    /* BDP / TSO / EDT */
    {.procname = "kcc_bdp_min_rtt_us",          .data = &kcc_bdp_min_rtt_us,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100k us] BDP min-RTT floor; KCC extension; default 1us */
    {.procname = "kcc_probe_cwnd_bonus",        .data = &kcc_probe_cwnd_bonus,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100 segs] extra cwnd bonus during PROBE_BW phase 0; BBR default: 2 */
    {.procname = "kcc_edt_near_now_ns",         .data = &kcc_edt_near_now_ns,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..10M ns] EDT near-now threshold; KCC extension; default 1000ns */
    {.procname = "kcc_min_tso_rate",            .data = &kcc_min_tso_rate,            .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..1B bytes/s] pacing rate threshold for min TSO segs; BBR default: ~1.2M */
    {.procname = "kcc_tso_max_segs",            .data = &kcc_tso_max_segs,            .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..65535 segs] max TSO segments per GSO skb; BBR default: 127 */
    /* Jitter / qdelay probe tuning */
    {.procname = "kcc_jitter_probe_thresh_us",  .data = &kcc_jitter_probe_thresh_us,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100k us] jitter threshold for PROBE_BW gain decay; KCC-only; default 4000 */
    {.procname = "kcc_jitter_probe_scale_us",   .data = &kcc_jitter_probe_scale_us,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k us] jitter scaling divisor for gain decay; KCC-only; default 16000 */
    {.procname = "kcc_qdelay_probe_thresh_us",  .data = &kcc_qdelay_probe_thresh_us,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100k us] qdelay threshold for PROBE_BW gain decay; KCC-only; default 5000 */
    {.procname = "kcc_qdelay_probe_scale_us",   .data = &kcc_qdelay_probe_scale_us,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k us] qdelay scaling divisor for gain decay; KCC-only; default 20000 */
    {.procname = "kcc_jitter_r_thresh_us",      .data = &kcc_jitter_r_thresh_us,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100k us] jitter threshold for adaptive Kalman R; KCC-only; default 2000 */
    {.procname = "kcc_jitter_r_scale",          .data = &kcc_jitter_r_scale,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] jitter scaling divisor for adaptive R; KCC-only; default 8000 */
    {.procname = "kcc_kalman_r_max_boost",      .data = &kcc_kalman_r_max_boost,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..1000] max R boost multiplier; KCC-only; default 8 (max R <= 9x base) */
    /* Long RTT threshold */
    {.procname = "kcc_probe_rtt_long_rtt_us",   .data = &kcc_probe_rtt_long_rtt_us,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..10M us] long-RTT threshold; KCC extension; default 20,000us */
    /* LT BW parameters */
    {.procname = "kcc_lt_intvl_min_rtts",       .data = &kcc_lt_intvl_min_rtts,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..127 RTTs] LT BW min sampling interval; KCC-only; default 4 */
    {.procname = "kcc_lt_intvl_max_mult",       .data = &kcc_lt_intvl_max_mult,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..32] LT BW timeout = mult*min_rtts; KCC-only; default 4 */
    {.procname = "kcc_lt_loss_thresh",          .data = &kcc_lt_loss_thresh,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..65535] LT BW min loss ratio (BBR_UNIT); KCC-only; default 15 */
    {.procname = "kcc_lt_bw_ratio_num",         .data = &kcc_lt_bw_ratio_num,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100k] LT BW relative tolerance numerator; KCC-only; default 1 */
    {.procname = "kcc_lt_bw_ratio_den",         .data = &kcc_lt_bw_ratio_den,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] LT BW relative tolerance denominator; KCC-only; default 8 (12.5%) */
    {.procname = "kcc_lt_bw_diff",              .data = &kcc_lt_bw_diff,              .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100k bytes/s] LT BW absolute tolerance; KCC-only; default 500 */
    {.procname = "kcc_lt_bw_max_rtts",          .data = &kcc_lt_bw_max_rtts,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..4094 RTTs] LT BW max before reset (fits 12 bits); KCC-only; default 48 */
    {.procname = "kcc_lt_bw_inst_qdelay_thresh_us", .data = &kcc_lt_bw_inst_qdelay_thresh_us, .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100k us] LT BW instantaneous qdelay gate; KCC-only; default 5000 */
    /* Kalman core (Kalman 1960) */
    {.procname = "kcc_kalman_p_est_init",       .data = &kcc_kalman_p_est_init,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..10M] initial error covariance p_est; KCC-only; default 1000 */
    {.procname = "kcc_kalman_p_est_floor",      .data = &kcc_kalman_p_est_floor,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] p_est lower bound (prevents over-confidence); KCC-only; default 10 */
    {.procname = "kcc_kalman_outlier_ms",       .data = &kcc_kalman_outlier_ms,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..10k ms] base outlier gate threshold; KCC-only; default 5 */
    {.procname = "kcc_kalman_q_boost_ms",       .data = &kcc_kalman_q_boost_ms,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..5000 ms] Q-boost time constant; KCC-only; default 1 */
    {.procname = "kcc_kalman_qboost_cdwn",       .data = &kcc_kalman_qboost_cdwn,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..255] Q-boost cooldown (min samples between events); KCC-only; default 15 */
    {.procname = "kcc_kalman_scale",            .data = &kcc_kalman_scale,            .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [64..1M] Kalman fixed-point scale (power-of-two); KCC-only; default 1024 */
    {.procname = "kcc_kalman_outlier_jitter_mult_num", .data = &kcc_kalman_outlier_jitter_mult_num, .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..1000] outlier jitter multiplier numerator; KCC-only; default 4 */
    {.procname = "kcc_kalman_outlier_jitter_mult_den", .data = &kcc_kalman_outlier_jitter_mult_den, .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] outlier jitter multiplier denominator; KCC-only; default 1 */
    {.procname = "kcc_kalman_q_min_factor_num", .data = &kcc_kalman_q_min_factor_num, .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..1000] Q min factor numerator; KCC-only; default 10 */
    {.procname = "kcc_kalman_q_min_factor_den", .data = &kcc_kalman_q_min_factor_den, .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] Q min factor denominator; KCC-only; default 1 */
    {.procname = "kcc_kalman_p_est_init_rtt_div_num", .data = &kcc_kalman_p_est_init_rtt_div_num, .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] p_est init RTT divisor numerator; KCC-only; default 10 */
    {.procname = "kcc_kalman_p_est_init_rtt_div_den", .data = &kcc_kalman_p_est_init_rtt_div_den, .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] p_est init RTT divisor denominator; KCC-only; default 1 */
    /* BBR-S covariance-matched noise estimation (Kalman 1960) */
    {.procname = "kcc_kalman_noise_alpha_num",   .data = &kcc_kalman_noise_alpha_num,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100] BBR-S Q learning rate numerator; KCC-only; default 1 */
    {.procname = "kcc_kalman_noise_alpha_den",   .data = &kcc_kalman_noise_alpha_den,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] BBR-S Q learning rate denominator; KCC-only; default 10 */
    {.procname = "kcc_kalman_noise_beta_num",    .data = &kcc_kalman_noise_beta_num,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100] BBR-S R learning rate numerator; KCC-only; default 1 */
    {.procname = "kcc_kalman_noise_beta_den",    .data = &kcc_kalman_noise_beta_den,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] BBR-S R learning rate denominator; KCC-only; default 10 */
    {.procname = "kcc_kalman_q_est_max",         .data = &kcc_kalman_q_est_max,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..2e9] Q estimate upper bound (covariance-matched); KCC-only; default 1e9 */
    {.procname = "kcc_kalman_r_est_max",         .data = &kcc_kalman_r_est_max,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..2e9] R estimate upper bound (covariance-matched); KCC-only; default 1e9 */
    {.procname = "kcc_kalman_q_est_floor",       .data = &kcc_kalman_q_est_floor,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] Q estimate lower bound; KCC-only; default 1 */
    {.procname = "kcc_kalman_r_est_floor",       .data = &kcc_kalman_r_est_floor,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] R estimate lower bound; KCC-only; default 1 */
    {.procname = "kcc_kalman_noise_mode",        .data = &kcc_kalman_noise_mode,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..2] noise mode: 0=off(heuristic), 1=max(default), 2=weighted blend; KCC-only */
    /* ECN */
    {.procname = "kcc_alone_confirm_rounds",     .data = &kcc_alone_confirm_rounds,     .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..32] alone mode hysteresis rounds; KCC-only; default 3 */
    {.procname = "kcc_alone_qdelay_thresh_us",   .data = &kcc_alone_qdelay_thresh_us,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100k us] alone mode max qdelay; KCC-only; default 1000 */
    {.procname = "kcc_alone_jitter_thresh_us",   .data = &kcc_alone_jitter_thresh_us,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100k us] alone mode max jitter; KCC-only; default 2000 */
    {.procname = "kcc_alone_agg_state_level",    .data = &kcc_alone_agg_state_level,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..2] alone mode agg strictness; KCC-only; default 1 */
    {.procname = "kcc_alone_bypass_ecn",         .data = &kcc_alone_bypass_ecn,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0/1] alone mode ECN bypass; KCC-only; default 1=bypass */
    {.procname = "kcc_ecn_enable",              .data = &kcc_ecn_enable,              .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0/1] ECN master switch; KCC extension; default 1=enabled */
    {.procname = "kcc_ecn_backoff_num",         .data = &kcc_ecn_backoff_num,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100] ECN backoff percentage numerator; KCC extension; default 20 */
    {.procname = "kcc_ecn_backoff_den",         .data = &kcc_ecn_backoff_den,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] ECN backoff percentage denominator; KCC extension; default 100 */
    {.procname = "kcc_ecn_qdelay_thresh_us",    .data = &kcc_ecn_qdelay_thresh_us,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100k us] ECN qdelay threshold; KCC extension; default 2000 */
    {.procname = "kcc_ecn_ewma_retained",       .data = &kcc_ecn_ewma_retained,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100] ECN EWMA retained weight; KCC extension; default 3 */
    {.procname = "kcc_ecn_ewma_total",          .data = &kcc_ecn_ewma_total,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] ECN EWMA total weight; KCC extension; default 4 */
    {.procname = "kcc_ecn_idle_decay_num",      .data = &kcc_ecn_idle_decay_num,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..den-1] ECN per-ACK idle decay numerator; KCC-only; default 31 */
    {.procname = "kcc_ecn_idle_decay_den",      .data = &kcc_ecn_idle_decay_den,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [2..100k] ECN per-ACK idle decay denominator; KCC-only; default 32 */
    /* Kalman */
    {.procname = "kcc_kalman_max_consec_reject", .data = &kcc_kalman_max_consec_reject, .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..1000] consecutive reject limit before force-accept; KCC-only; default 25 */
    /* EWMA */
    {.procname = "kcc_ewma_qdelay_num",         .data = &kcc_ewma_qdelay_num,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100] EWMA qdelay numerator (old weight); KCC-only; default 7 */
    {.procname = "kcc_ewma_qdelay_den",         .data = &kcc_ewma_qdelay_den,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] EWMA qdelay denominator; KCC-only; default 8 (7/8=0.875 old weight) */
    {.procname = "kcc_ewma_jitter_num",         .data = &kcc_ewma_jitter_num,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100] EWMA jitter numerator (old weight); KCC-only; default 7 */
    {.procname = "kcc_ewma_jitter_den",         .data = &kcc_ewma_jitter_den,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] EWMA jitter denominator; KCC-only; default 8 */

    /* Misc */
    {.procname = "kcc_probe_bw_cycle_rand",     .data = &kcc_probe_bw_cycle_rand,     .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..cycle_len] PROBE_BW random offset range; BBR default: 8 */
    {.procname = "kcc_probe_bw_up_limit",       .data = &kcc_probe_bw_up_limit,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0/1] limit PROBE_BW up-phase exit; KCC extension; default 0=off */
    /* ACK aggregation and PROBE_RTT duration */
    {.procname = "kcc_extra_acked_max_ms_num",  .data = &kcc_extra_acked_max_ms_num,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100k ms] ACK-agg max window numerator; KCC extension; default 150 */
    {.procname = "kcc_extra_acked_max_ms_den",  .data = &kcc_extra_acked_max_ms_den,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k ms] ACK-agg max window denominator; KCC extension; default 1 */
    /* ACK agg confidence compensation */
    {.procname = "kcc_agg_enable",              .data = &kcc_agg_enable,              .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0/1] agg compensation master switch; KCC extension (BBRplus-inspired); default 1 */
    {.procname = "kcc_agg_confidence_thresh",   .data = &kcc_agg_confidence_thresh,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..10k] min confidence for cwnd comp; KCC-only; default 512 */
    {.procname = "kcc_agg_max_comp_ratio",      .data = &kcc_agg_max_comp_ratio,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100%] max cwnd comp % of BDP; KCC-only; default 75 */
    {.procname = "kcc_agg_max_comp_duration",   .data = &kcc_agg_max_comp_duration,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..128] max consecutive comp RTTs; KCC-only; default 8 */
    {.procname = "kcc_agg_r_hysteresis",        .data = &kcc_agg_r_hysteresis,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100%] R recovery hysteresis % retained/RTT; KCC-only; default 75 */
    {.procname = "kcc_agg_r_multiplier_min",    .data = &kcc_agg_r_multiplier_min,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..10k] R scaling floor (256=1x); KCC-only; default 256 */
    {.procname = "kcc_agg_r_multiplier_max",    .data = &kcc_agg_r_multiplier_max,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..10k] R scaling ceiling (2048=8x); KCC-only; default 2048 */
    {.procname = "kcc_agg_factor3_qdelay_us",    .data = &kcc_agg_factor3_qdelay_us,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100k us] confidence Factor 3 qdelay margin; KCC-only; default 2000 */
    {.procname = "kcc_agg_factor4_ratio_num",    .data = &kcc_agg_factor4_ratio_num,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] Factor 4 ratio numerator; KCC-only; default 3 */
    {.procname = "kcc_agg_factor4_ratio_den",    .data = &kcc_agg_factor4_ratio_den,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] Factor 4 ratio denominator; KCC-only; default 2 (1.5x) */
    {.procname = "kcc_agg_safety_qdelay_us",     .data = &kcc_agg_safety_qdelay_us,     .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100k us] safety guard 1 qdelay; KCC-only; default 4000 */
    {.procname = "kcc_agg_safety_bdp_mult",      .data = &kcc_agg_safety_bdp_mult,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100x] safety guard 3/4 BDP multiplier; KCC-only; default 3 */
    {.procname = "kcc_agg_max_window_ms",        .data = &kcc_agg_max_window_ms,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..10k ms] extra_acked cap window; KCC-only; default 100 */
    {.procname = "kcc_agg_max_decay_pct",        .data = &kcc_agg_max_decay_pct,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100%] watchdog decay pct retained/RTT; KCC-only; default 75 */
    {.procname = "kcc_agg_max_per_ack_decay",    .data = &kcc_agg_max_per_ack_decay,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..128] per-ACK gentle decay; 128=no decay; KCC-only; default 128 */
    {.procname = "kcc_agg_max_per_ack_decay_den", .data = &kcc_agg_max_per_ack_decay_den, .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..65535] per-ACK decay denominator; KCC-only; default 128 */
    {.procname = "kcc_agg_window_rotation_rtts",  .data = &kcc_agg_window_rotation_rtts,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..65535 RTTs] window rotation period; KCC-only (BBRplus-inspired); default 5 */
    {.procname = "kcc_agg_factor_weight",          .data = &kcc_agg_factor_weight,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..1024] per-factor confidence score increment; KCC-only; default 256 */
    {.procname = "kcc_agg_confidence_max",         .data = &kcc_agg_confidence_max,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..65536] confidence scaling denominator; KCC-only; default 1024 */
    {.procname = "kcc_agg_thresh_suspected",       .data = &kcc_agg_thresh_suspected,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..1024] SUSPECTED state threshold; KCC-only; default 256 */
    {.procname = "kcc_agg_thresh_confirmed",       .data = &kcc_agg_thresh_confirmed,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [2..1024] CONFIRMED state threshold; KCC-only; default 512 */
    {.procname = "kcc_agg_thresh_trusted",         .data = &kcc_agg_thresh_trusted,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [3..1024] TRUSTED state threshold; KCC-only; default 768 */
    {.procname = "kcc_probe_rtt_mode_ms_num",   .data = &kcc_probe_rtt_mode_ms_num,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k ms] PROBE_RTT stay duration numerator; BBR default: 200ms */
    {.procname = "kcc_probe_rtt_mode_ms_den",   .data = &kcc_probe_rtt_mode_ms_den,   .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k ms] PROBE_RTT stay duration denominator; BBR default: 1 */
    /* Other misc */
    {.procname = "kcc_bw_rt_cycle_len",         .data = &kcc_bw_rt_cycle_len,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [2..256 RTTs] BW sliding window length; BBR default: 10 */
    {.procname = "kcc_cwnd_min_target",         .data = &kcc_cwnd_min_target,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..1000 segs] absolute min cwnd target; BBR default: 4 */
    /* TSO / Sndbuf / Epoch */
    {.procname = "kcc_tso_headroom_mult",       .data = &kcc_tso_headroom_mult,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..1000] TSO headroom multiplier (×tso_segs_goal); BBR default: 3 */
    {.procname = "kcc_min_tso_rate_div",        .data = &kcc_min_tso_rate_div,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..256] TSO rate threshold divisor; KCC extension; default 8 */
    {.procname = "kcc_sndbuf_expand_factor",    .data = &kcc_sndbuf_expand_factor,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [2..100x] sndbuf expansion factor (×cwnd); BBR default: 3 */
    {.procname = "kcc_ack_epoch_max",           .data = &kcc_ack_epoch_max,           .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [64K..2G bytes] epoch accumulator cap; KCC extension; default 0xFFFFF (~1M) */
    /* Kalman / MinRTT / PROBE_RTT extra */
    {.procname = "kcc_kalman_rtt_dyn_mult",     .data = &kcc_kalman_rtt_dyn_mult,     .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100] dynamic RTT ceiling multiplier; KCC-only; default 2 */
    {.procname = "kcc_kalman_probe_band_mult",  .data = &kcc_kalman_probe_band_mult,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..32] probe interval transition band mult; KCC-only; default 4 */
    {.procname = "kcc_kalman_q_rtt_div",        .data = &kcc_kalman_q_rtt_div,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..1M] Q adaptation RTT divisor (us→ms); KCC-only; default 1000 */
    {.procname = "kcc_minrtt_fast_fall_div",    .data = &kcc_minrtt_fast_fall_div,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..256] min_rtt fast-fall threshold divisor; KCC extension; default 4 */
    {.procname = "kcc_probe_rtt_long_interval_div", .data = &kcc_probe_rtt_long_interval_div, .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..1000] long-RTT probe interval divisor; 1=no scaling; KCC extension; default 1 */
    {.procname = "kcc_lt_bw_ema_num",         .data = &kcc_lt_bw_ema_num,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100] LT BW EMA numerator; KCC-only; default 1 */
    {.procname = "kcc_lt_bw_ema_den",         .data = &kcc_lt_bw_ema_den,         .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] LT BW EMA denominator; KCC-only; default 2 (1/2=EMA) */
    {.procname = "kcc_kalman_noise_avg_num",  .data = &kcc_kalman_noise_avg_num,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100] noise averaging numerator (mode=2); KCC-only; default 1 */
    {.procname = "kcc_kalman_noise_avg_den",  .data = &kcc_kalman_noise_avg_den,  .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] noise averaging denominator (mode=2); KCC-only; default 2 */
    {.procname = "kcc_rtt_mode",              .data = &kcc_rtt_mode,              .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0/1] model RTT strategy: 1=FILTER [default], 0=MIN(x_est,min_rtt); KCC-only */
    {.procname = "kcc_probe_rtt_decouple",    .data = &kcc_probe_rtt_decouple,    .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0/1] skip PROBE_RTT when FILTER mode active; KCC-only; default 1 */
    {.procname = "kcc_tso_segs_low",          .data = &kcc_tso_segs_low,          .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..65535] TSO segments at low pacing rate; KCC extension; default 1 */
    {.procname = "kcc_tso_segs_default",      .data = &kcc_tso_segs_default,      .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..65535] TSO segments at normal pacing rate; BBR default: 2 */
    {.procname = "kcc_extra_acked_win_rtts_max", .data = &kcc_extra_acked_win_rtts_max, .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..65535 RTTs] max dual-window RTTs before rotation; KCC-only; default 31 */
    /* Global Kalman BDP filter (cross-connection bandwidth estimation) */
    {.procname = "kcc_kf_enable",              .data = &kcc_kf_enable,              .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0/1] global Kalman BDP enable; KCC-only; default 0=off */
    {.procname = "kcc_kf_startup_r_pct",       .data = &kcc_kf_startup_r_pct,       .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100%] startup R measurement noise pct; KCC-only; default 20 */
    {.procname = "kcc_kf_steady_r_pct",        .data = &kcc_kf_steady_r_pct,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100%] steady-state R measurement noise pct; KCC-only; default 5 */
    {.procname = "kcc_kf_q_shift",             .data = &kcc_kf_q_shift,             .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..30] process noise shift (Q = 1<<shift); KCC-only; default 20 */
    {.procname = "kcc_kf_chi2_num",            .data = &kcc_kf_chi2_num,            .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] chi-squared innovation gate num; KCC-only; default 384 */
    {.procname = "kcc_kf_chi2_den",            .data = &kcc_kf_chi2_den,            .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] chi-squared innovation gate den; KCC-only; default 100 */
    {.procname = "kcc_kf_discount_num",        .data = &kcc_kf_discount_num,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [0..100] fair-share discount numerator (%); KCC-only; default 50 */
    {.procname = "kcc_kf_discount_den",        .data = &kcc_kf_discount_den,        .maxlen = sizeof(int), .mode = 0644, .proc_handler = kcc_proc_handler }, /* [1..100k] fair-share discount denominator; KCC-only; default 100 */
    {} /* sentinel: end of table */
}; /* end of kcc_ctl_table[] */
/*
 * ---- BTF kfunc Registration (for BPF struct_ops) ----------------------
 *
 * On kernels >= 5.16, KCC registers its callback functions as BTF kfuncs
 * so that BPF struct_ops programs can invoke them.
 *
 * The BTF set macros vary by kernel version:
 *   6.9+: BTF_KFUNCS_START / BTF_KFUNCS_END
 *   6.0+: BTF_SET8_START / BTF_SET8_END
 *   5.16+: BTF_SET_START / BTF_SET_END
 *
 * Additionally, 6.0+ uses BTF_ID_FLAGS with the 'func' flag; pre-6.0
 * uses BTF_ID.  The registration is gated on CONFIG_X86 and
 * CONFIG_DYNAMIC_FTRACE (required for kfunc infrastructure on x86).
 *
 * On 5.18+ the set is registered via register_btf_kfunc_id_set();
 * on 5.16-5.17 it uses register_kfunc_btf_id_set() with a different API.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0)                                            /* kernel 5.16+: BTF support */

BTF_SETS_START(tcp_kcc_check_kfunc_ids)                                                        /* start BTF kfunc ID set */
#ifdef CONFIG_X86                                                                                /* kfunc only on x86 */
#ifdef CONFIG_DYNAMIC_FTRACE                                                                       /* requires dynamic ftrace */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)                                                    /* 6.0+: BTF_ID_FLAGS */
BTF_ID_FLAGS(func, kcc_init)                                                                         /* register kcc_init as kfunc */
BTF_ID_FLAGS(func, kcc_main)                                                                          /* register kcc_main as kfunc */
BTF_ID_FLAGS(func, kcc_sndbuf_expand)                                                                  /* register kcc_sndbuf_expand as kfunc */
BTF_ID_FLAGS(func, kcc_undo_cwnd)                                                                       /* register kcc_undo_cwnd as kfunc */
BTF_ID_FLAGS(func, kcc_cwnd_event)                                                                       /* register kcc_cwnd_event as kfunc */
BTF_ID_FLAGS(func, kcc_ssthresh)                                                                         /* register kcc_ssthresh as kfunc */
BTF_ID_FLAGS(func, kcc_min_tso_segs)                                                                     /* register kcc_min_tso_segs as kfunc */
BTF_ID_FLAGS(func, kcc_set_state)                                                                        /* register kcc_set_state as kfunc */
#else                                                                                                      /* pre-6.0: BTF_ID macro */
BTF_ID(func, kcc_init)                                                                                     /* register kcc_init as kfunc (legacy) */
BTF_ID(func, kcc_main)                                                                                      /* register kcc_main as kfunc (legacy) */
BTF_ID(func, kcc_sndbuf_expand)                                                                             /* register kcc_sndbuf_expand as kfunc (legacy) */
BTF_ID(func, kcc_undo_cwnd)                                                                                 /* register kcc_undo_cwnd as kfunc (legacy) */
BTF_ID(func, kcc_cwnd_event)                                                                                /* register kcc_cwnd_event as kfunc (legacy) */
BTF_ID(func, kcc_ssthresh)                                                                                   /* register kcc_ssthresh as kfunc (legacy) */
BTF_ID(func, kcc_min_tso_segs)                                                                                /* register kcc_min_tso_segs as kfunc (legacy) */
BTF_ID(func, kcc_set_state)                                                                                    /* register kcc_set_state as kfunc (legacy) */
#endif                                                                                                           /* end BTF_ID version switch */
#endif /* CONFIG_DYNAMIC_FTRACE */                                                                                 /* end DYNAMIC_FTRACE gate */
#endif /* CONFIG_X86 */                                                                                             /* end CONFIG_X86 gate */
BTF_SETS_END(tcp_kcc_check_kfunc_ids)                                                                                /* end BTF kfunc ID set */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0)                                                                     /* 5.18+: new registration API */
static const struct btf_kfunc_id_set tcp_kcc_kfunc_set = {                                                              /* BTF kfunc set descriptor */
    .owner = THIS_MODULE,                                                                                                 /* module owner */
    .set = &tcp_kcc_check_kfunc_ids,                                                                                     /* pointer to kfunc ID set */
};                                                                                                                          /* tcp_kcc_kfunc_set */
#else                                                                                                                         /* 5.16-5.17: legacy API */
static DEFINE_KFUNC_BTF_ID_SET(&tcp_kcc_check_kfunc_ids, tcp_kcc_kfunc_btf_set);                                             /* define legacy kfunc set */
#endif                                                                                                                          /* end version switch */

#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0) */                                                                     /* end BTF support block */

/* ---- Module Init / Exit ----------------------------------------------- */

/*
 * kcc_register - Module initialization function.
 *
 * Steps:
 *   1. Verify struct kcc fits within ICSK_CA_PRIV_SIZE (compile-time check).
 *   2. Seed kcc_gain_num[] / kcc_gain_den[] with BBRv1 defaults (Cardwell et al. 2016):
 *      [5/4, 3/4, 1/1, 1/1, 1/1, 1/1, 1/1, 1/1] repeated across 256 slots.
 *      This ensures sysctl reports real values from the start.
 *   3. Call kcc_init_module_params() to clamp and compute all derived values.
 *   4. Register sysctl interface under /proc/sys/net/kcc/.
 *   5. Register BTF kfunc set for BPF struct_ops (5.16+, 5.18+).
 *   6. Register the congestion_control ops with the TCP stack.
 *
 * Cleanup on failure: unregister_sysctl -> return error.
 */
static int __init kcc_register(void)                                                                                        /* module init entry */
{
    static const int dfl_num[KCC_DEFAULT_GAIN_CYCLE_LEN] = { 5,3,1,1,1,1,1,1 };                                                                         /* BBRv1 gain numerator pattern (Cardwell et al. 2016) */
    static const int dfl_den[KCC_DEFAULT_GAIN_CYCLE_LEN] = { 4,4,1,1,1,1,1,1 };                                                                         /* BBRv1 gain denominator pattern */
    int ret = -ENOMEM, i;                                                                                                                /* return code and loop index */

    /* Compile-time guard: struct kcc must fit in the CA private slot */
    BUILD_BUG_ON(sizeof(struct kcc) > ICSK_CA_PRIV_SIZE);                                                                       /* compile-time size check */

    /*
     * Initialize gain arrays with BBRv1 default cycle:
     *   5/4 (=1.25x), 3/4 (=0.75x), 1/1 (=1.0x), 1/1, 1/1, 1/1, 1/1, 1/1.
     * This pattern repeats across the 256-slot table.
     * kcc_rebuild_gain_table() reads from these arrays to compute the
     * effective gains in kcc_cycle_gain_table[].
     */
    for (i = 0; i < KCC_GAIN_SLOTS; i++) {                                                                                      /* iterate all 256 gain slots */
        kcc_gain_num[i] = dfl_num[i % KCC_DEFAULT_GAIN_CYCLE_LEN];                                                                                         /* set numerator from BBRv1 pattern */
        kcc_gain_den[i] = dfl_den[i % KCC_DEFAULT_GAIN_CYCLE_LEN];                                                                                         /* set denominator from BBRv1 pattern */
    }
    kcc_init_module_params();                                                                                                      /* clamp + compute all derived values */

    /* Register sysctl at /proc/sys/net/kcc/ */
    kcc_ctl_header = register_sysctl("net/kcc", kcc_ctl_table);                                                                     /* register sysctl table */
    if (!kcc_ctl_header) {
        pr_warn("KCC: failed to register sysctl\n");
        goto unregister_sysctl;
    }

    /* Register as a TCP congestion control algorithm */
    ret = tcp_register_congestion_control(&tcp_kcc_cong_ops);                                                                            /* register CC ops */
    if (ret) {                                                                                                                             /* registration failed */
        goto unregister_sysctl;                                                                                                              /* clean up */
    }
    /* ---- BTF kfunc registration (kernel >= 5.18) ---- */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0)                                                                                   /* 5.18+: direct registration */
#if defined(CONFIG_X86) && defined(CONFIG_DYNAMIC_FTRACE)
    ret = register_btf_kfunc_id_set(BPF_PROG_TYPE_STRUCT_OPS, &tcp_kcc_kfunc_set);                                                    /* register kfunc set */
    if (ret < 0) {                                                                                                                     /* registration failed */
        goto unregister_cc;                                                                                                              /* clean up: unregister CC */
    }
#endif
#endif
    /* ---- BTF kfunc registration (kernel 5.16-5.17, legacy API) ---- */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0) && LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)                                        /* 5.16-5.17: legacy API */
#if defined(CONFIG_X86) && defined(CONFIG_DYNAMIC_FTRACE)
    ret = register_kfunc_btf_id_set(&bpf_tcp_ca_kfunc_list, &tcp_kcc_kfunc_btf_set);                                                               /* register via legacy API */
    if (ret < 0) {                                                                                                                                   /* registration failed */
        pr_warn("KCC: legacy kfunc registration failed (err %d); BPF struct_ops unavailable\n", ret);                             /* non-fatal: CC still works */
    }
#endif
#endif
    return 0;                                                                                                                                  /* success */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0) && defined(CONFIG_X86) && defined(CONFIG_DYNAMIC_FTRACE)
    unregister_cc:                                                                                                                                /* BTF registration failed after CC registered */
    tcp_unregister_congestion_control(&tcp_kcc_cong_ops);                                                                                   /* unregister CC */
#endif

unregister_sysctl:                                                                                                                                /* error cleanup label */
    if (kcc_ctl_header) {                                                                                                                          /* sysctl was registered */
        unregister_sysctl_table(kcc_ctl_header);                                                                                                     /* unregister sysctl */
        kcc_ctl_header = NULL;                                                                                                                         /* clear header pointer */
    }
    return ret;                                                                                                                                        /* return error code */
}
/*
 * kcc_unregister - Module exit function.
 *
 * Reverse of kcc_register:
 *   1. Unregister legacy BTF kfunc set (5.16-5.17).
 *   2. Unregister congestion control ops.
 *   3. Unregister sysctl table.
 *
 * Note: BTF kfunc sets registered via register_btf_kfunc_id_set() (5.18+)
 * are automatically cleaned up by the kernel on module unload.
 */
static void __exit kcc_unregister(void)                                                                                                              /* module exit handler */
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0) && LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)                                                    /* legacy BTF API */
#if defined(CONFIG_X86) && defined(CONFIG_DYNAMIC_FTRACE)
    unregister_kfunc_btf_id_set(&bpf_tcp_ca_kfunc_list, &tcp_kcc_kfunc_btf_set);                                                                        /* unregister legacy kfunc set */
#endif
#endif
    tcp_unregister_congestion_control(&tcp_kcc_cong_ops);                                                                                                 /* unregister CC ops */
    if (kcc_ctl_header) {                                                                                                                                  /* sysctl table registered */
        unregister_sysctl_table(kcc_ctl_header);                                                                                                             /* unregister sysctl table */
        kcc_ctl_header = NULL;                                                                                                                                 /* clear header pointer */
    }
}

module_init(kcc_register);                                                                                                                                     /* register module init callback */
module_exit(kcc_unregister);                                                                                                                                     /* register module exit callback */

MODULE_AUTHOR("PPP PRIVATE NETWORK(TM) X");                                                                                                                     /* primary module author */
MODULE_AUTHOR("Original BBR: Van Jacobson, Neal Cardwell, Yuchung Cheng, "                                                                                    /* BBR algorithm authors */
    "Soheil Hassas Yeganeh (Google)");                                                                                                                    /* (Cardwell et al. 2016) */
MODULE_LICENSE("Dual BSD/GPL");                                                                                                                                    /* module license identifier */
MODULE_DESCRIPTION("TCP KCC v1.0 - BBRv1 state machine with Kalman-filter propagation-delay estimation");                                                          /* module description */
MODULE_VERSION("1.0");                                                                                                                                             /* module version string */