#!/usr/bin/env python3


GROWTH_NUM = 122
GROWTH_DEN = 1000


class GeodesicEstimator:
    """Simulates the geodesic T_prop estimator from tcp_kcc.c (G1-G4).

    Dual-threshold G3 (fast 10%/4-count, slow 5%/5-count).
    G1/G2 outside if/else. Reset on x_est <= mr*SCALE, not on nu < 0.
    """

    def __init__(self, initial_rtt_us, scale=1024):
        self.x_est = initial_rtt_us * scale
        self.min_rtt_us = initial_rtt_us
        self.confirm_cnt = 0
        self.confirm_slow_cnt = 0
        self.SCALE = scale
        self.theta_fast = 11  # 11/10 = 1.1x
        self.theta_slow = 21  # 21/20 = 1.05x
        self.bdp = initial_rtt_us
        self.history = []
        self.g3_events = 0

    def update(self, z_k_us):
        z_scaled = z_k_us * self.SCALE
        nu_k = z_scaled - self.x_est
        g3_fired_this_step = False

        if nu_k <= 0:
            self.x_est = min(self.x_est, z_scaled)
        else:
            growth = self.x_est * GROWTH_NUM // GROWTH_DEN
            self.x_est = min(self.x_est + growth, z_scaled)

        thresh_fast = (self.min_rtt_us * self.theta_fast * self.SCALE) // 10
        thresh_slow = (self.min_rtt_us * self.theta_slow * self.SCALE) // 20
        mr_scaled = self.min_rtt_us * self.SCALE

        if self.x_est >= thresh_fast:
            self.confirm_cnt += 1
            self.confirm_slow_cnt += 1
        elif self.x_est >= thresh_slow:
            self.confirm_cnt = 0
            self.confirm_slow_cnt += 1
        else:
            self.confirm_cnt = 0
        if self.x_est <= mr_scaled:
            self.confirm_cnt = 0
            self.confirm_slow_cnt = 0

        if self.confirm_cnt >= 4 or self.confirm_slow_cnt >= 5:
            self.min_rtt_us = self.x_est // self.SCALE
            self.confirm_cnt = 0
            self.confirm_slow_cnt = 0
            g3_fired_this_step = True

        x_est_us = self.x_est // self.SCALE
        self.bdp = min(x_est_us, self.min_rtt_us)
        st = dict(
            x_est_us=x_est_us,
            min_rtt_us=self.min_rtt_us,
            bdp_us=self.bdp,
            confirm_cnt=self.confirm_cnt,
            confirm_slow_cnt=self.confirm_slow_cnt,
            nu=nu_k,
            g3_this_step=g3_fired_this_step,
        )
        self.history.append(st)
        return st

    def reset(self, initial_rtt_us):
        self.__init__(initial_rtt_us, self.SCALE)
