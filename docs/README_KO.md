[🇺🇸 English](../README.md) | [🇨🇳 中文](README_CN.md) | [🇹🇼 繁體中文](README_TW.md) | [🇪🇸 Español](README_ES.md) | [🇫🇷 Français](README_FR.md) | [🇷🇺 Русский](README_RU.md) | [🇸🇦 العربية](README_AR.md) | [🇩🇪 Deutsch](README_DE.md) | [🇯🇵 日本語](README_JA.md) | [🇰🇷 한국어](README_KO.md) | [🇮🇹 Italiano](README_IT.md) | [🇵🇹 Português](README_PT.md)

# TCP KCC v1.0 (칼만 혼잡 제어)

공유 대역폭 VPS 환경을 위한, BBRv1 상태 머신과 칼만 필터를 결합하여 전파 지연을 추정하는 TCP 혼잡 제어 모듈입니다.

## 설계 원칙

혼잡 제어 알고리즘은 처리량, 지연 시간, 공정성, 손실 허용 간의 균형을 유지해야 합니다. KCC는 실용적인 접근 방식을 취합니다:

1. BBRv1은 검증된 기반을 제공합니다. 상태 머신, 페이싱, 사이클 게인, STARTUP/DRAIN/PROBE_BW/PROBE_RTT — KCC는 이러한 메커니즘을 수정 없이 채택합니다.

2. 칼만 필터는 추정 정확도를 향상시킵니다. 실제 전파 지연과 큐잉 지연 및 지터를 분리하면 더 정확한 min_rtt 추정이 가능해져, 더 정밀한 BDP 계산, 더 잘 보정된 CWND, 그리고 더 안정적인 페이싱을 제공합니다.

3. 알고리즘 간 역학은 표준 TCP 경쟁 평형을 따릅니다. KCC는 외부 흐름에서 감지된 큐에 반응하여 전송 속도를 인위적으로 제한하지 않습니다. 게인 감쇠(큐 기반 프로브 감소)는 kcc_cycle_decay_mask를 통해 옵트인으로 사용할 수 있지만, 기본적으로 비활성화되어 전체 프로브 강도를 유지합니다.

4. Intra-KCC 공정성이 적극적으로 유지됩니다. 칼만 수렴을 통해 동일 호스트의 KCC 흐름이 일관된 min_rtt 추정치를 공유하게 되어, 순수 BBR 다중 흐름 배포에서 심각한 불공정을 유발하는 승자독식 피드백 루프가 제거됩니다.

## 알고리즘 개요

TCP KCC는 Linux 커널용 로더블 `tcp_kcc.ko`로 구현된 송신측 혼잡 제어 모듈입니다. 혼잡 제어 함수 `kcc_main()`은 `tcp_ack()`에서 각 ACK를 수신할 때마다 호출되며, 커널 수준의 대역폭 및 RTT 샘플과 전송 및 손실 카운트를 포함하는 `rate_sample` 구조체를 전달받습니다. 이 알고리즘은 두 가지 시간 영역에서 작동합니다: **ACK별 고속 경로**는 측정 상태를 업데이트하고 즉각적인 페이싱 및 윈도우 목표를 계산하며, **라운드별 저속 경로**는 상태 전이 조건을 평가하고 게인을 재계산합니다.

핵심 측정 파이프라인은 두 가지 구성 요소로 이루어져 있습니다:

1. **슬라이딩 윈도우 최대 대역폭 필터**(`linux/win_minmax.h`의 `minmax_running_max`): 최근 `kcc_bw_rt_cycle_len`(기본값 10) 라운드를 포함하는 윈도우입니다. BBR 호환 `max_bw` 추정값을 제공합니다.

2. **칼만 필터 전파 지연 추정기**: BBRv1의 슬라이딩 윈도우 최소 RTT를 대체하고, BDP RTT 추정의 기본 소스입니다（[모델 RTT 전략](#모델-rtt-전략) 참조）. `kcc_kalman_scale` × µs 고정소수점 단위로 작동하는 단일 상태 칼만 필터(Kalman 1960)로, 실제 전파 지연을 랜덤 워크로 모델링합니다:
   - 상태: `x[k] = x[k−1] + w[k]`, `w ~ N(0, Q)`
   - 관측: `z[k] = x[k] + v[k]`, `v ~ N(0, R)`

고정소수점 규칙: 대역폭은 `BW_UNIT = 1 << 24`(세그먼트 * 2^24 / µs), `BBR_UNIT = 1 << 8 = 256`을 무차원 게인 단위로 사용합니다.

## 모델 RTT 전략

KCC는 BDP(대역폭 지연 곱) 계산에 사용되는 RTT 추정 전략을 `kcc_rtt_mode`로 구성 가능하게 합니다:

| 모드 | 값 | 동작 | 용도 |
|------|----|------|----------|
| FILTER | 1(기본값) | `x_est_us` 직접 사용 — 원시 칼만/슬라이딩 윈도우 필터 추정치 | 프로덕션 WAN/VPS: 경로 변경에 강함, 처리량 절벽 제로 |
| MIN | 0 | `min(x_est_us, min_rtt_us)` — 칼만 추정치를 윈도우 최소값으로 클램프 | 커널 모듈 안정성 검증, 정적 RTT 링크 |

**FILTER가 기본값인 이유:**

- **경로 변경 내성**: BGP 재라우팅으로 물리적 RTT가 증가한 경우(예: 50ms → 100ms), 칼만 게인 K_k가 수 RTT 내에 새 경로 지연을 추적합니다. MIN 모드는 이전 `min_rtt_us`에 데드락되어 BDP가 절반으로 감소합니다.

- **내장 방어**: 이상치 게이팅이 큐 급증 샘플을 필터에 들어오기 전에 거부합니다. 적응형 Q/R 잡음 추정이 네트워크 잡음이 높을 때 칼만 게인을 감소시켜, 필터가 자연스럽게 일시적인 큐 팽창을 신뢰하지 않고 추정치를 실제 전파 지연 근처에 유지합니다.

- **PROBE_RTT 분리**: FILTER 모드는 `kcc_probe_rtt_decouple` 기능을 활성화 — 칼만 필터가 정기적인 4패킷 드레인 없이 RTT 하한을 추적합니다.

런타임 전환: `echo 0 > /proc/sys/net/kcc/kcc_rtt_mode`로 MIN 모드로 복귀.

## 상태 머신

```
    ┌───> STARTUP ────┐
    │       │         │
    │       ▼         │
    │     DRAIN  ─────┤
    │       │         │
    │       ▼         │
    └─── PROBE_BW ────┘
    │      ^    │
    │      │    │
    │      └────┘
    │
    └─── PROBE_RTT <──┘
```

4가지 모드는 `struct KCC`의 2비트 `mode` 필드로 인코딩됩니다:

- **STARTUP (0)**: 초기 상태. `pacing_gain` ≈ 2.885x(`kcc_high_gain_val`), `cwnd_gain`도 2.885x. 지수적 대역폭 프로빙.
- **DRAIN (1)**: STARTUP 종료 후 진입. `pacing_gain` ≈ 0.347x(`kcc_drain_gain_val`), `cwnd_gain`은 2.885x 유지. STARTUP 중 누적된 큐를 비웁니다.
- **PROBE_BW (2)**: 정상 상태. 256개 슬롯 게인 테이블 순환(기본 8-페이즈 패턴 반복: 1.25x/0.75x/8×1.0x).
- **PROBE_RTT (3)**: 주기적으로 in-flight를 `kcc_cwnd_min_target`(기본값 4 세그먼트)까지 비워 새로운 RTT 샘플을 획득합니다.

### STARTUP → DRAIN

`full_bw_reached`가 설정되면 트리거됩니다——`kcc_full_bw_cnt`(기본값 3)회 연속 라운드에서 `max_bw`가 이전에 관측된 피크 대비 `kcc_full_bw_thresh_val`(기본값 1.25x) 이상 성장하지 못한 경우. 1.0x 게인에서의 BDP가 `snd_ssthresh`에 기록됩니다. `qdelay_avg`는 0으로 리셋되어 STARTUP 큐 누적이 PROBE_BW에 영향을 미치는 것을 방지합니다.

### DRAIN → PROBE_BW

추정된 EDT에서의 in-flight가 1.0x BDP 게인의 목표 in-flight 이하가 되면 트리거됩니다. **드레인 스킵 최적화**: 칼만 필터가 수렴되고 `qdelay_avg`가 `kcc_drain_skip_qdelay_us`(기본값 1000 µs) 미만인 경우 DRAIN 단계를 건너뛰고 조기에 PROBE_BW로 전환합니다.

PROBE_BW 진입 시, 사이클 페이즈 인덱스가 무작위화됩니다: `cycle_idx = len − 1 − rand(kcc_probe_bw_cycle_rand)`(기본값 `len − 1 − rand(8)`). 이는 병목 링크를 공유하는 동시 흐름의 상관관계를 제거합니다.

### PROBE_BW → PROBE_RTT

PROBE_RTT 필터 간격이 만료되면 트리거됩니다——타임스탬프 `min_rtt_stamp`가 계산된 간격 내에 업데이트되지 않은 경우. cwnd는 `prior_cwnd`에 저장되고, 페이싱은 드레인 모드로 설정됩니다.

### PROBE_RTT → PROBE_BW

in-flight가 `kcc_cwnd_min_target`까지 떨어지거나 라운드 경계가 관측된 후, 최소 `kcc_probe_rtt_mode_ms_val`(기본값 200 ms) 이상 그리고 최소 1회 완전 라운드가 관측된 후 종료됩니다. cwnd는 `prior_cwnd`로 복원되고, 표준 PROBE_BW 게인 사이클로 진입합니다.

### 복구 및 손실

- TCP_CA_Loss 시: `full_bw`와 `full_bw_cnt`가 리셋되고, `round_start`가 1로 설정되며, `packet_conservation`이 0으로 클리어됩니다.
- 복구 진입(TCP_CA_Recovery): `packet_conservation` 활성화, cwnd = in-flight + acked.
- 복구 종료: `prior_cwnd`로 복원, `packet_conservation` 클리어.
- `kcc_undo_cwnd()`: `full_bw`와 `full_bw_cnt`를 리셋하고(`full_bw_reached` 유지), LT BW 상태를 클리어합니다.

### 라운드 감지 (BBR 정렬)

`next_rtt_delivered`는 `0`으로 초기화되며(표준 BBR과 동일; Cardwell et al. 2016), 첫 번째 ACK가 즉시 라운드 1 감지를 시작하므로 인위적인 오프셋이 필요하지 않습니다. `prior_delivered >= next_rtt_delivered`일 때 라운드 경계가 감지되며, `interval_us <= 0` 가드(BBR의 `delivered < 0 || interval_us <= 0`와 일치)를 사용하여 0 값 및 음수 간격을 포착하고 측정 파이프라인 오염을 방지합니다.

- `next_rtt_delivered`는 `0`으로 초기화(BBR 준거): 첫 번째 ACK에서 첫 번째 라운드 시작.
- `interval_us <= 0` 검증: BBR에 정확히 일치, 음수 간격 거부.
- `round_start`는 `kcc_update_bw()`의 시작 부분에서, 검증 확인 전에 `0`으로 재설정 — BBR의 `bbr->round_start = 0` 배치와 일치.

## 핵심 측정

### 대역폭 추정

슬라이딩 윈도우 최대 대역폭 필터(`linux/win_minmax.h`의 `minmax_running_max`), 범위는 `kcc_bw_rt_cycle_len`(기본값 10) 라운드. 순시 대역폭 = `delivered × BW_UNIT / interval_us`, ACK마다 계산. 앱 제한 상태가 아니거나 대역폭이 현재 최대값 이상인 경우(BBR 규칙)에만 슬라이딩 윈도우에 공급됩니다.

`lt_use_bw`가 활성화되면, 활성 대역폭 추정이 `lt_bw`(장기 대역폭 추정값)로 전환됩니다.

### 칼만 필터

단일 상태 스칼라 칼만 재귀(O(1) 복잡도):

```
예측:
  x_pred = x_est          (항등 상태 전이)
  p_pred = p_est + Q      (공분산 예측)

갱신:
  innov   = z − x_pred    (혁신)
  K       = p_pred / (p_pred + R)   (칼만 게인 [0,1])
  x_est   = x_pred + K × innov      (상태 갱신)
  p_est   = (1 − K) × p_pred        (사후 공분산)
```

**적응형 프로세스 잡음 Q**:
```
Q_base   = kcc_kalman_q (기본값 100)
q_factor = max(kcc_kalman_q_min_factor_val, min_rtt_us / kcc_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × kcc_kalman_q_scale_cap)
Q        = min(Q, kcc_kalman_q_max)
```

**적응형 측정 잡음 R**:
```
R = R_base + max(0, jitter_ewma − kcc_jitter_r_thresh_us) × R_base / kcc_jitter_r_scale
R = min(R, R_base × kcc_kalman_r_max_boost)
```

**Q-Boost 경로 변경 감지(신뢰도 게이트 + 쿨다운)**: `|innovation| > kcc_kalman_q_boost_thresh_val`(기본값 약 4ms RTT 변화)이고 필터가 수렴된 상태(`p_est ≤ kcc_kalman_converged_p_est_val`, 기본값 500)인 경우, `p_est`가 `kcc_kalman_p_est_init_val`로 재설정되어 칼만 게인이 1.0에 가깝게 증가하여 빠르게 수렴합니다. 연속적인 qboost 이벤트 사이의 `kcc_kalman_qboost_cdwn`(기본값 15) 샘플의 쿨다운은 높은 RTT 지터를 가진 손실 경로에서의 폭주 트리거를 방지합니다.

**이상치 게이팅**: 동적 임계값 `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`. `p_pred ≤ kcc_kalman_converged_p_est_val`인 경우에만 적용됩니다. `kcc_kalman_max_consec_reject`(기본값 25)회 연속 거부 후, 다음 샘플은 강제 수용되어 자기 강화 잠금을 방지합니다.

**공분산 일치 잡음 추정(BBR-S)**: `q_est = (1−α) × q_est + α × (K × innov)²`, `r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`. 결합 모드: 모드 0 = 휴리스틱 전용, 모드 1 = 최대값(기본값), 모드 2 = 가중 혼합.

**칼만 인계**: `x_est > 0`이고 `sample_cnt ≥ kcc_kalman_min_samples`(기본값 5)인 경우, `min_rtt_us`가 `x_est / kcc_kalman_scale`로 대체됩니다. `min_rtt_stamp`는 업데이트되지 않습니다——PROBE_RTT 간격 트리거는 독립적으로 유지됩니다.

**모델 RTT 전략**: BDP 계산에 사용되는 RTT 추정은 `kcc_rtt_mode`로 제어됩니다. FILTER 모드(기본값)에서는 `model_rtt = x_est_us`를 직접 사용 — 칼만/슬라이딩 윈도우 추정치를 클램프 없이 사용. MIN 모드에서는 `model_rtt = min(x_est_us, min_rtt_us)` — 칼만 추정치를 윈도우 최소값으로 클램프하여 BDP가 절대 팽창하지 않음을 보장. FILTER 기본값은 경로 지연이 갑자기 변경될 수 있는 프로덕션 WAN/VPS 배포(BGP 재라우팅, LEO 핸드오버, 모바일 셀 전환)에 권장됩니다.[모델 RTT 전략](#모델-rtt-전략) 참조.

## BBR 개선 사항

### 게인 감쇠

특정 PROBE_BW 페이즈에 대해 256비트 비트맵 `kcc_cycle_decay_mask[]`로 활성화됩니다. 감쇠 공식(수용된 칼만 샘플 기준):

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = p_est의 역 스케일링(완전 신뢰 시 BBR_UNIT)
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                    × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                    × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

칼만 신뢰도 스케일링: `p_est > kcc_kalman_converged_p_est`인 경우, 감쇠가 비례적으로 감소하여 필터가 불확실할 때 과도한 백오프를 방지합니다.

### ECN 백오프

활성화 조건(모두 충족 필요):
1. `kcc_ecn_enable_val != 0`
2. 칼만 수렴됨(`p_est < converged`, `sample_cnt >= min_samples`)
3. `ecn_ewma > 0`(CE 마크 관측)
4. `qdelay_avg > kcc_ecn_qdelay_thresh_us_val`(기본값 2000 µs)
5. 모드가 PROBE_BW가 아님(PROBE_BW에서는 cwnd_gain이 2x로 고정)

프로빙 페이즈 중(`pacing_gain > BBR_UNIT`), ECN 백오프는 `BBR_UNIT² / pacing_gain`에 따라 단계적으로 적용됩니다——1.25x 프로브에서 약 80% 백오프, 2.89x STARTUP 게인에서 약 65%.

ECN 마크 비율 EWMA: 라운드 경계에서 `kcc_ecn_ewma_retained / kcc_ecn_ewma_total`(기본값 3/4)로 업데이트되며, 새로운 CE 마크가 없는 각 ACK에서 `kcc_ecn_idle_decay_num / kcc_ecn_idle_decay_den`(기본값 31/32)의 완만한 감쇠가 적용됩니다.

### 단일 흐름 감지

KCC가 병목 지점에서 흐름이 단독일 가능성이 높다고 감지하면(낮은 큐 지연, 낮은 지터, ECN 마크 없음, ACK 집계 없음), 자동으로 BBR 순수 모드로 전환됩니다:

- `kcc_get_model_rtt()`가 `min_rtt_us`를 직접 반환합니다(단측 측정 잡음으로 인한 작은 양의 편향을 가진 칼만 평활 추정치를 우회).
- `kcc_ecn_backoff()` 응답은 `kcc_alone_bypass_ecn`(기본값 1)로 설정 가능——단일 흐름 경로에는 ECN 표시를 공유할 경쟁 발신자가 없으며, 표시는 AQM 오탐이므로 건너뛰어 BBR의 제로 ECN 동작과 일치시킴. 0으로 설정 시 단독 모드에서도 ECN 백오프 유지(보수적).

이를 통해 KCC와 BBR 간의 단일 흐름 처리량 격차가 제거되며, 다중 흐름 시나리오에서는 KCC의 완전한 보호 루프(칼만, ECN 백오프, 이득 감쇠)가 유지됩니다.

**히스테리시스**: 진입에는 `kcc_alone_confirm_rounds`(기본값 3)개의 연속 적격 라운드가 필요합니다——다중 흐름 경쟁 중 짧은 조용한 기간 동안의 진동을 방지합니다("보수적 가속"). 퇴장: 크루즈 단계 평가 중, 어떤 단일 적격 조건이라도 실패하면 플래그가 지워집니다("공격적 제동").

**설계 트레이드오프**: 패킷 손실은 단일 흐름 비적격 기준으로 사용되지 않습니다——일부 링크(얕은 버퍼, 무선, 가상화 버스트 드롭)에는 경쟁과 무관한 고유 손실이 있습니다. 손실을 다중 흐름 경쟁과 동일시하면 단일 흐름 경로에서 진동이 발생합니다. LT BW 신호(BBR의 손실 트리거 폴리서 감지)는 단일 흐름 판단에 참여하지 않습니다.

**게인 게이팅**: 단일 흐름 평가는 크루즈 단계(`pacing_gain == BBR_UNIT`)에서만 실행됩니다. 프로브 업(1.25x)은 의도적으로 병목을 밀어냅니다——그 큐 압력은 자기 유도적이며 경쟁 신호가 아닙니다. 드레인(0.75x)은 인위적으로 큐를 억제합니다. 평가를 크루즈(정상 상태 균형)로 제한함으로써, 자기 유도적 프로브 업 압력이 더 이상 거짓 단독 모드 종료를 유발하지 않습니다.

적격 조건(라운드 경계에서 다섯 가지 모두 충족되어야 함):
0. 칼만 수렴(`sample_cnt >= kcc_kalman_min_samples`) — qdelay/jitter를 큐 신호로 신뢰
1. `qdelay_avg < kcc_alone_qdelay_thresh_us`(기본값 1000 us) — 큐가 거의 비어 있음
2. `jitter_ewma < kcc_alone_jitter_thresh_us`(기본값 2000 us) — ACK 클록 마이크로 지터만
3. `ecn_ewma == 0` — AQM의 혼잡 마크 없음
4. `agg_state <= max` `kcc_alone_agg_state_level` 기준 (기본값 1) — 3단계 ACK 집계 엄격도:
   - 0 = IDLE만 (가장 엄격, 제로 집계), 1 = ≤ SUSPECTED (기본값, 일시적 집계 허용), 2 = ≤ CONFIRMED (가장 허용적, 지속적 집계만 차단)

### 동적 PROBE_RTT 간격

칼만 `p_est`를 연결별 PROBE_RTT 간격으로 매핑합니다:

```
p_est ≤ converged:              간격 = dyn_max (기본값 30s)
p_est ≥ high (= mult × conv):   간격 = base (기본값 10s)
converged < p_est < high:       선형 보간
```

신뢰도가 높은 경우(`p_est` 낮음) PROBE_RTT 빈도를 줄여 안정적인 경로에서 처리량 지터를 낮춥니다. 신뢰도가 낮은 경우 기존 10초 간격으로 되돌아갑니다.

**흐름별 엔트리 지터**: 모든 공존 흐름이 동시에 PROBE_RTT에 진입하는 것을 방지하기 위해(4개 패킷 집계 ~1.8 Mbps로 드레인한 후 2.89×로 재충전), 각 흐름은 해시 파생 지터(0–845 ms 분포)를 자체 PROBE_RTT 간격에 추가합니다. 임의의 순간에 최대 ~1개의 흐름만 PROBE_RTT에 있어 RTO를 유발하는 동시 드레인/재충전 붕괴를 제거합니다.

### PROBE_RTT 분리 및 스마트 재보정

BBRv1의 PROBE_RTT 메커니즘은 약 10초마다 파이프를 4개 패킷까지 드레인하여 `min_rtt_us`를 측정합니다. 이는 윈도우 기반 최소 RTT 추정기에 필요합니다 — 파이프가 비어 있지 않으면 윈도우는 전파 지연과 큐잉 지연을 구분할 수 없습니다. 대가는 주기적인 처리량 절벽(BBR "톱니파")입니다.

FILTER 모드에서는 칼만 필터가 윈도우를 완전히 대체합니다. 이상치 게이팅과 적응형 잡음 추정을 통해 큐잉 잡음에서 실제 전파 지연을 분리할 수 있습니다 — 파이프 드레인 불필요. 파라미터 `kcc_probe_rtt_decouple`(기본값 1)이 이를 제어합니다:

| 모드 | 값 | 동작 |
|------|----|------|
| 분리 | 1(기본값) | **칼만 정상**(p_est ≤ `kcc_recal_p_est_thresh`): PROBE_RTT 완전 억제 → 처리량 절벽 제로, 동기 붕괴 제로. **칼만 발산**(p_est > 임계값): 안전망으로 기존 PROBE_RTT 자동 발동 → 필터 기준선 복원 후 분리 자동 재개. |
| 기존 | 0 | 맹목적인 주기적 PROBE_RTT 약 10초마다(BBR 호환). |

**스마트 재보정 휴리스틱**(`kcc_kalman_needs_recalibration()`): 안정적인 경로에서 정상 상태 작동 시, 칼만 오차 공분산 p_est가 p_est_floor(~4–10)로 수렴하며, 임계값 `kcc_recal_p_est_thresh`(250,000 = p_est_max의 25%)보다 훨씬 낮습니다. p_est 상승은 필터의 내부 모델이 더 이상 관측을 설명할 수 없음을 나타냅니다 — 일반적으로 경로가 실질적으로 변경된 경우입니다. p_est가 임계값을 초과하면, 1회의 기존 PROBE_RTT 드레인이 필터 기준선을 복원하고, 칼만이 재수렴하여 분리가 자동 재개됩니다.

이를 통해 PROBE_RTT가 **맹목적인 주기적 자해**에서 **지능형 신뢰도 기반 재보정**으로 변모합니다 — 프로토콜은 필터가 신뢰를 잃었다는 경험적 증거가 있을 때만 파이프를 드레인합니다.

`kcc_rtt_mode == 1` 필요. MIN 모드에서는 무효(MIN 모드는 `min_rtt_us` 갱신에 PROBE_RTT 필요).

| 파라미터 | 기본값 | 범위 | 설명 |
|------|------|------|------|
| `kcc_probe_rtt_decouple` | 1 | 0–1 | PROBE_RTT 분리 활성화(FILTER 모드만) |
| `kcc_recal_p_est_thresh` | 250,000 | 1–100,000,000 | 안전망 재보정을 위한 p_est 임계값 |

### LT 대역폭 추정

손실 트리거 기반 하한 추정기. 샘플링 간격은 [4, 16] RTT 범위. 손실률이 5.9%(`kcc_lt_loss_thresh` 기본값 15/256) 이상일 때 유효. 대역폭 `bw = delivered × BW_UNIT / interval_us`.

BBR의 단순 평균(`(bw + lt_bw) >> 1`)과 달리, KCC는 설정 가능한 EMA를 사용합니다(`kcc_lt_bw_ema_num / kcc_lt_bw_ema_den`, 기본값 1/2 = 0.5):

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

활성화 방식이 BBR과 다릅니다: KCC는 첫 번째 유효 간격에서 `lt_bw`를 저장하지만 `lt_use_bw`는 설정**하지 않습니다**; 이전 간격과의 일관성이 필요하며, 측정 잡음으로 인한 잘못된 활성화를 줄입니다.

**이중 임계값 혼잡 게이트**: `lt_use_bw = 1`을 설정하기 전에, 지속적인 EWMA 큐 검사(`qdelay_avg > kcc_ecn_qdelay_thresh_us_val`)와 SRTT 기반 순시 큐 검사(`srtt_us − min_rtt_us > kcc_lt_bw_inst_qdelay_thresh_us`, 기본값 5000 µs)가 모두 평가됩니다. 혼잡이 감지되면 LT BW 샘플링이 중단됩니다. SRTT 검사는 `ext` 할당 없이 작동하여 할당 실패에 대한 안전망을 제공합니다.



### ACK 집계 신뢰도 기반 보상(BBRplus에서 유래)

기존 듀얼 슬롯 extra-acked 추정기 위에 신뢰도 게이트가 적용된 두 번째 레이어를 추가합니다.

**4개의 직교 요소**(각각 `kcc_agg_factor_weight` 포인트(기본값 256) 기여):
1. 칼만 수렴됨(`p_est < converged` + `sample_cnt >= min_samples`)
2. 손실 복구 중이 아님(`icsk_ca_state < TCP_CA_Recovery`)
3. RTT가 `min_rtt_us + kcc_agg_factor3_qdelay_us`(기본값 2ms) 이내로 실제 전파 지연에 가까움
4. `extra_acked`가 `kcc_agg_factor4_ratio_num/den`(기본값 1.5x) 이내로 윈도우화 최대값에 가까움

**4가지 상태**: IDLE(< `kcc_agg_thresh_suspected`=256), SUSPECTED(≥256), CONFIRMED(≥512), TRUSTED(≥768).

**신호 레이어**(항상 활성): 신뢰도가 R 스케일링 계수 `[r_min, r_max]`를 선형 보간. R은 즉시 상승(빠른 응답), 각 RTT에서 `kcc_agg_r_hysteresis`%(기본값 75% 유지, 약 4 RTT 후 기준선 복귀)로 감쇠.

**제어 레이어**(`agg_state ≥ CONFIRMED`): 5계층 안전 게이트 cwnd 보상:
1. 큐 지연 > `kcc_agg_safety_qdelay_us`(기본값 4ms)면 차단
2. 손실 복구 중이면 차단
3. cwnd > `BDP × kcc_agg_safety_bdp_mult`(기본값 3x)면 차단
4. in-flight > 안전 cwnd + TSO 세그먼트 목표면 차단
5. 워치독: `kcc_agg_max_comp_duration`(기본값 8)회 연속 RTT 후 CONFIRMED을 SUSPECTED로 강등

### 드레인 qdelay_avg 리셋

DRAIN으로 전환 시 `qdelay_avg`가 0으로 리셋되어 STARTUP 큐 추정값이 PROBE_BW에 지속되는 것을 방지합니다.

### TSO 제수 적응

`kcc_min_tso_segs()`는 칼만 상태에 따라 속도 임계값 제수를 조정합니다:
- 칼만 수렴됨 + `jitter_ewma < 1000 µs`: 제수 절반(8→4), 더 큰 TSO 버스트
- `jitter_ewma > 4000 µs`: 제수 두 배(8→16), 더 작은 TSO 버스트로 지터 억제

## 페이싱 속도 및 Cwnd

### 페이싱 속도

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // 게인 조정
rate = rate × USEC_PER_SEC >> BW_SCALE            // bytes/s로 변환
rate = rate × margin_div / 100                    // 페이싱 마진(기본값 1%, matching BBR)
```

속도 변경은 즉시 적용되며(평활화 없음), BBR(Cardwell et al. 2016)과 일치합니다. `full_bw_reached` 이후: 모든 속도 변경이 즉시 기록됩니다. STARTUP/DRAIN 중: 증가(`rate > sk_pacing_rate`)만 적용됩니다.

### Cwnd

```
target = BDP(bw, gain, ext)                       // 기본 BDP
// in-flight 경계(비-STARTUP: lo~hi 클램프; STARTUP: lo 플로어만)
target = quantization_budget(target)              // TSO 헤드룸 + 짝수 라운드 + phase-0 보너스
target += ack_agg_bonus + agg_compensation        // ACK 집계 보상

// cwnd 진행
if full_bw_reached:
    cwnd = min(cwnd + acked, target)              // 목표로 수렴
else (STARTUP):
    cwnd = cwnd + acked                          // 지수적 성장

cwnd = max(cwnd, cwnd_min_target)                 // 절대 하한 4
PROBE_RTT mode: cwnd = min(cwnd, cwnd_min_target) // 최소 in-flight
```

## 모듈 파라미터

파라미터는 `/proc/sys/net/kcc/`에서 노출됩니다. 쓰기는 `kcc_init_module_params()`(검증 + 클램핑 + 파생값 계산)를 트리거합니다. 배열 파라미터 쓰기는 `kcc_rebuild_gain_table()`을 트리거합니다.

### PROBE_RTT 간격

| 파라미터 | 기본값 | 최소 | 최대 | 단위 | 설명 |
|-----------|---------|-----|-----|------|-------------|
| `kcc_probe_rtt_base_sec` | 10 | 1 | 86400 | s | 기본 PROBE_RTT 간격 |
| `kcc_probe_rtt_max_sec` | 15 | 1 | 86400 | s | 긴 RTT 경로의 상한 |
| `kcc_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | s | 최대 동적 간격; 0은 비활성화 |

### 게인

| 파라미터 | 기본값 | 최소 | 최대 | 설명 |
|-----------|---------|-----|-----|-------------|
| `kcc_cwnd_gain_num` / `kcc_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | PROBE_BW의 기본 cwnd 게인 |
| `kcc_extra_acked_gain_num` / `kcc_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | ACK 집계 보너스 승수 |
| `kcc_high_gain_num` / `kcc_high_gain_den` | 2885 / 1000 | 0/1 | 100k | STARTUP 게인(≈2.885x) |
| `kcc_drain_gain_num` / `kcc_drain_gain_den` | 347 / 1000 | 0/1 | 100k | DRAIN 게인(≈0.347x) |
| `kcc_inflight_low_gain_num` / `kcc_inflight_low_gain_den` | 100 / 100 | 0/1 | 100k | in-flight 하한(1.0x BDP) |
| `kcc_inflight_high_gain_num` / `kcc_inflight_high_gain_den` | 200 / 100 | 0/1 | 100k | in-flight 상한(2.0x BDP) |
| `kcc_gain_num[i]` / `kcc_gain_den[i]` | BBRv1 패턴(256 슬롯) | 0/1 | — | 슬롯별 페이싱 게인 |
| `kcc_cycle_decay_mask[8]` | 0(모두 0) | 0 | 0x7FFFFFFF | 256비트 감쇠 비트맵 |
| `kcc_probe_bw_up_limit` | 0 | 0 | 1 | 프로브-업 제한 종료（0=꺼짐） |

### 칼만 기본

| 파라미터 | 기본값 | 최소 | 최대 | 설명 |
|-----------|---------|-----|-----|-------------|
| `kcc_kalman_q` | 100 | 0 | 100k | 기본 프로세스 잡음 Q |
| `kcc_kalman_r` | 400 | 0 | 100k | 기본 측정 잡음 R |
| `kcc_kalman_p_est_max` | 1,000,000 | 1 | 100M | p_est 절대 최대값 |
| `kcc_kalman_converged_p_est` | 500 | 1 | 1M | 수렴 임계값 |
| `kcc_kalman_p_est_init` | 1000 | 1 | 10M | 초기 p_est |
| `kcc_kalman_p_est_floor` | 10 | 1 | 100k | p_est 플로어 |
| `kcc_kalman_scale` | 1024 | 64 | 1,048,576 | 고정소수점 스케일(2의 거듭제곱) |
| `kcc_kalman_min_samples` | 5 | 3 | 20 | 인계 전 최소 샘플 수 |
| `kcc_kalman_outlier_ms` | 5 | 0 | 10000 | ms | 이상치 기본 임계값 |
| `kcc_kalman_q_boost_mult` | 4 | 1 | 10000 | Q-Boost 승수 |
| `kcc_kalman_q_boost_ms` | 1 | 0 | 5000 | ms | Q-Boost 시정수 |
| `kcc_kalman_qboost_cdwn` | 15 | 1 | 255 | samples | Q-부스트 쿨다운 |
| `kcc_kalman_q_max` | 2000 | 1 | 100k | Q 상한 |
| `kcc_kalman_q_scale_cap` | 20 | 1 | 10000 | Q 스케일 상한 |
| `kcc_kalman_max_consec_reject` | 25 | 1 | 1000 | 강제 수용 전 최대 연속 거부 횟수 |
| `kcc_rtt_sample_max_us` | 500000 | 1 | 10M | µs | 칼만 RTT 상한 |
| `kcc_kalman_r_max_boost` | 8 | 1 | 1000 | R 최대 부스트 승수 |
| `kcc_kalman_rtt_dyn_mult` | 2 | 1 | 100 | RTT 동적 상한 승수 |
| `kcc_kalman_q_rtt_div` | 1000 | 1 | 1M | Q 적응 RTT 제수 |
| `kcc_kalman_probe_band_mult` | 4 | 1 | 32 | PROBE_RTT 전이 대역 승수 |

### 칼만 추가(num/den 유형)

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `kcc_kalman_outlier_jitter_mult_num/den` | 4 / 1 | 0-1000 / 1-100k | 이상치 지터 승수 |
| `kcc_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | Q 최소 인자 |
| `kcc_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | p_est 초기 RTT 제수 |

### BBR-S 잡음 추정

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `kcc_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | Q 추정 학습률 |
| `kcc_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | R 추정 학습률 |
| `kcc_kalman_noise_mode` | 1 | 0-2 | 결합 모드(0=끄기, 1=최대값, 2=가중 평균) |
| `kcc_kalman_q_est_max` | 1,000,000,000 | 1-2B | Q 추정 상한 |
| `kcc_kalman_r_est_max` | 1,000,000,000 | 1-2B | R 추정 상한 |
| `kcc_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | 추정별 하한 |

### 게인 감쇠(프로빙)

| 파라미터 | 기본값 | 범위 | 단위 | 설명 |
|-----------|---------|-------|------|-------------|
| `kcc_qdelay_probe_thresh_us` | 5000 | 0-100k | µs | qdelay 감쇠 임계값 |
| `kcc_qdelay_probe_scale_us` | 20000 | 1-100k | µs | qdelay 감쇠 스케일 |
| `kcc_jitter_probe_thresh_us` | 4000 | 0-100k | µs | 지터 감쇠 임계값 |
| `kcc_jitter_probe_scale_us` | 16000 | 1-100k | µs | 지터 감쇠 스케일 |

### 적응형 R(지터 기반)

| 파라미터 | 기본값 | 범위 | 단위 | 설명 |
|-----------|---------|-------|------|-------------|
| `kcc_jitter_r_thresh_us` | 2000 | 0-100k | µs | R 증가를 위한 지터 임계값 |
| `kcc_jitter_r_scale` | 8000 | 1-100k | — | R 증가 스케일 제수 |

### ECN

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `kcc_ecn_enable` | 1 | 0-1 | ECN 마스터 스위치 |
| `kcc_ecn_backoff_num` / `kcc_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | ECN 백오프 비율 |
| `kcc_ecn_qdelay_thresh_us` | 2000 | 0-100k | µs | ECN qdelay 임계값 |
| `kcc_ecn_ewma_retained` / `kcc_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | ECN EWMA 가중치 |
| `kcc_ecn_idle_decay_num` / `kcc_ecn_idle_decay_den` | 31 / 32 | 1-100k | 유휴 ECN 감쇠 |

### min_rtt

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `kcc_minrtt_fast_fall_cnt` | 3 | 0-3 | 고속 하강 카운트 |
| `kcc_minrtt_fast_fall_div` | 4 | 1-256 | 고속 하강 임계값 제수 |
| `kcc_minrtt_sticky_num` / `kcc_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | 스티키 하강 비율 |
| `kcc_minrtt_srtt_guard_num` / `kcc_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | SRTT 가드 비율 |

### LT 대역폭

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `kcc_lt_intvl_min_rtts` | 4 | 1-127 | RTTs | 최소 간격 길이 |
| `kcc_lt_intvl_max_mult` | 4 | 1-32 | 간격 타임아웃 승수 |
| `kcc_lt_loss_thresh` | 15 | 1-65535 | BBR_UNIT | 최소 손실률 |
| `kcc_lt_bw_ratio_num` / `kcc_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | 상대 허용 오차 |
| `kcc_lt_bw_diff` | 500 | 0-100k | bytes/s | 절대 허용 오차 |
| `kcc_lt_bw_max_rtts` | 48 | 1-4094 | RTTs | LT BW 최대 활성 RTTs |
| `kcc_lt_bw_ema_num` / `kcc_lt_bw_ema_den` | 1 / 2 | 0-100 / 1-100k | LT BW EMA 가중치 |


### ACK 집계 신뢰도

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `kcc_agg_enable` | 1 | 0-1 | 마스터 스위치 |
| `kcc_agg_confidence_thresh` | 512 | 0-10000 | cwnd 보상 신뢰도 임계값 |
| `kcc_agg_max_comp_ratio` | 75 | 0-100 | % of BDP | cwnd 보상 상한 |
| `kcc_agg_max_comp_duration` | 8 | 1-128 | RTTs | 워치독 타임아웃 |
| `kcc_agg_r_hysteresis` | 75 | 0-100 | % | R 히스테리시스 감쇠 |
| `kcc_agg_r_multiplier_min` / `kcc_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | R 스케일링 범위(256=1x) |
| `kcc_agg_factor3_qdelay_us` | 2000 | 0-100k | µs | 요소 3 qdelay 마진 |
| `kcc_agg_factor4_ratio_num` / `kcc_agg_factor4_ratio_den` | 3 / 2 | 1-100k | 요소 4 비율 |
| `kcc_agg_safety_qdelay_us` | 4000 | 0-100k | µs | 안전 가드 1 qdelay |
| `kcc_agg_safety_bdp_mult` | 3 | 1-100 | 안전 가드 BDP 승수 |
| `kcc_agg_max_window_ms` | 100 | 1-10000 | ms | extra_acked 상한 윈도우 |
| `kcc_agg_max_decay_pct` | 75 | 0-100 | % | 워치독 감쇠율 |
| `kcc_agg_window_rotation_rtts` | 5 | 1-65535 | RTTs | 윈도우 로테이션 주기 |
| `kcc_agg_factor_weight` | 256 | 1-1024 | 요소별 점수 |
| `kcc_agg_confidence_max` | 1024 | 256-65535 | 최대 신뢰도 |

### EWMA 계수

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `kcc_ewma_qdelay_num` / `kcc_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | qdelay EWMA 가중치 |
| `kcc_ewma_jitter_num` / `kcc_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | 지터 EWMA 가중치 |

### 기타

| 파라미터 | 기본값 | 범위 | 설명 |
|-----------|---------|-------|-------------|
| `kcc_probe_bw_cycle_len` | 8 | 2-256 | PROBE_BW 사이클 페이즈 수(2의 거듭제곱) |
| `kcc_probe_bw_cycle_rand` | 8 | 1-cycle_len | 사이클 페이즈 무작위 오프셋 |
| `kcc_full_bw_thresh_num` / `kcc_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | STARTUP 종료 성장 임계값 |
| `kcc_full_bw_cnt` | 3 | 1-3 | 종료할 비성장 라운드 수 |
| `kcc_probe_rtt_mode_ms_num` / `kcc_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | PROBE_RTT 체류 기간 |
| `kcc_pacing_margin_num` / `kcc_pacing_margin_den` | 1 / 100 | 0-50 / 1-100k | 페이싱 마진(1 = 1%) |
| `kcc_probe_cwnd_bonus` | 2 | 0-100 | segs | 페이즈 0 cwnd 보너스 |
| `kcc_bw_rt_cycle_len` | 10 | 2-256 | rounds | 대역폭 슬라이딩 윈도우 길이 |
| `kcc_cwnd_min_target` | 4 | 1-1000 | segs | 최소 cwnd(PROBE_RTT) |
| `kcc_bdp_min_rtt_us` | 1 | 0-100k | µs | BDP min_rtt 플로어 |
| `kcc_edt_near_now_ns` | 1000 | 0-10M | ns | EDT 현재 시간 근접 임계값 |
| `kcc_min_tso_rate` | 1,200,000 | 1-1B | bytes/s | TSO 저속 임계값 |
| `kcc_min_tso_rate_div` | 8 | 1-256 | TSO 속도 제수(적응형 기준) |
| `kcc_tso_max_segs` | 127 | 1-65535 | segs | 최대 TSO 세그먼트 수 |
| `kcc_tso_segs_low` | 1 | 1-65535 | segs | 저속 TSO 세그먼트 수 |
| `kcc_tso_segs_default` | 2 | 1-65535 | segs | 정상 속도 TSO 세그먼트 수 |
| `kcc_tso_headroom_mult` | 3 | 0-1000 | TSO 헤드룸 승수 |
| `kcc_sndbuf_expand_factor` | 3 | 2-100 | 송신 버퍼 확장 인자 |
| `kcc_ack_epoch_max` | 0xFFFFF | 64K-2G | bytes | ACK 에포크 상한 |
| `kcc_extra_acked_max_ms_num` / `kcc_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | 최대 ACK 집계 윈도우 |
| `kcc_rtt_mode` | 1 | 0-1 | 모델 RTT 전략: 1=FILTER(칼만 직접), 0=MIN(클램프) |
| `kcc_probe_rtt_decouple` | 1 | 0-1 | PROBE_RTT 분리 활성화(FILTER 모드만): 4패킷 드레인 억제 |
| `kcc_recal_p_est_thresh` | 250,000 | 1-100M | 안전망 재보정을 위한 p_est 임계값 |
| `kcc_probe_rtt_long_rtt_us` | 20000 | 0-10M | µs | 긴 RTT 임계값 |
| `kcc_probe_rtt_long_interval_div` | 1 | 1-1000 | 긴 RTT 간격 제수 |
| `kcc_drain_skip_qdelay_us` | 1000 | 0-100k | µs | 드레인 스킵 qdelay 임계값 |
| `kcc_alone_confirm_rounds` | 3 | 1-32 | 라운드 | 단일 흐름 모드 활성화 전 확인 라운드 수 |
| `kcc_alone_qdelay_thresh_us` | 1000 | 0-100k | µs | 단일 흐름 감지 최대 큐 지연 |
| `kcc_alone_jitter_thresh_us` | 2000 | 0-100k | µs | 단일 흐름 감지 최대 지터 |
| kcc_alone_agg_state_level | 1 | 0-2 | — | 집계 엄격도 (0=IDLE만, 1=≤SUSPECTED기본값, 2=≤CONFIRMED과도하게공격적) |
| `kcc_alone_bypass_ecn` | 1 | 0-1 | — | 단독 모드 시 ECN 백오프 건너뛰기 (1=건너뛰기, 0=활성 유지) |

## 데이터 경로

```
ACK 도착 (rate_sample)
    │
    ▼
kcc_main()
    │
    ├──► ACK 집계 신뢰도 파이프라인(kcc_agg_enable 활성화 시)
    │      측정 → 평가 → 상태 → 워치독
    │      ├── 신호 레이어: 칼만 R 스케일링(항상 활성)
    │      └── 제어 레이어: cwnd 보상(CONFIRMED+)
    │
    ├──► kcc_update_model()
    │      ├── kcc_update_bw()              슬라이딩 윈도우 최대 BW
    │      ├── kcc_update_ecn_ewma()        ECN-CE 마크 비율
    │      ├── kcc_update_ack_aggregation()  듀얼 윈도우 extra_acked
    │      ├── kcc_update_cycle_phase()     PROBE_BW 페이즈 진행
    │      ├── kcc_check_full_bw_reached()  STARTUP 종료 감지
    │      ├── kcc_check_drain()            DRAIN 진입/종료 + 드레인 스킵
    │      ├── kcc_update_min_rtt()         칼만 + 윈도우 min-RTT + PROBE_RTT
    │      └── 모드별 게인 할당
    │
    ├──► kcc_apply_cwnd_constraints()
    │      └── kcc_ecn_backoff()            ECN 백오프(cwnd_gain만)
    │
    ├──► kcc_set_pacing_rate()              즉시, BBR 규칙
    │
    └──► kcc_set_cwnd()                    BDP + 경계 + 집계 보상
```

## 칼만 필터 내부 흐름

```
RTT 샘플 (rtt_us)
    │
    ├── 유효하지 않음(≥0 and < dynamic_max)? 아니오 → 폐기
    │
    ├── 콜드 스타트(sample_cnt==0)? 예 → 초기화: x_est=z, p_est=max(p_init, rtt_us/div)
    │                                           (RTT 최대 게이트 우회)
    │
    ├── 적응형 Q: Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   적응형 R: R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── 혁신: innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && 쿨다운 만료?
    │   ├── Yes: p_est = p_est_init, cooldown = 15, mark qboost_fired
    │   └── No:  cooldown-- if active
    │
    ├── 예측: p_pred = p_est + Q
    │
    ├── 이상치 게이트: |innov| > dyn_thresh && p_pred ≤ converged?
    │   ├── 예 & reject_cnt < max → 거부, ++consec_reject_cnt, 반환
    │   └── 예 & reject_cnt ≥ max → 강제 수용(잠금 방지)
    │
    └── 칼만 갱신:
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov(음수가 아니도록 클램프)
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── 지터 EWMA 갱신
         ├── qdelay EWMA 갱신
         ├── BBR-S 공분산 일치 잡음 추정
         └── sample_cnt++
```

## 진단

`ss -i`(`INET_DIAG_BBRINFO`)를 통한 BBR 호환 진단 인터페이스:

```
bbr_bw_lo/bbr_bw_hi: 64비트 대역폭 추정값(bytes/s)
bbr_min_rtt:         현재 min_rtt_us
bbr_pacing_gain:     현재 페이싱 게인(BBR_UNIT, 256=1.0x)
bbr_cwnd_gain:       현재 cwnd 게인(BBR_UNIT)
```

## 사용 방법

```sh
# 커널 모듈 컴파일
make

# 개발용 로드 (insmod, 의존성 해결 없음)
sudo make load

# 설치 및 공식 로드 (modprobe)
sudo make install
sudo make modload

# 언로드
sudo make unload

# KCC 알고리즘 선택
echo KCC > /proc/sys/net/ipv4/tcp_congestion_control
```

파라미터 설정은 `/proc/sys/net/kcc/`를 통해 이루어집니다. 예:
```sh
# 특정 PROBE_BW 페이즈에서 게인 감쇠 활성화
echo 1 > /proc/sys/net/kcc/kcc_cycle_decay_mask

# ECN 백오프 감도 조정
echo 30 > /proc/sys/net/kcc/kcc_ecn_backoff_num
```

## 동시성 및 안전 모델

KCC는 의도적으로 자체 데이터 구조에 READ_ONCE/WRITE_ONCE나 RCU를 사용하지 않습니다. 이 설계는 BBR 및 CUBIC과 같은 모든 커널 내 CC 모듈과 일관됩니다.

`kcc_init()`는 프로세스 컨텍스트(소켓 생성 중)에서 실행되며, 그 후에 소켓이 softirq에 노출됩니다. `kcc_release()`는 커널이 이 소켓의 ACK를 처리하는 softirq가 더 이상 없음을 보장한 후에 실행됩니다. 전역 모듈 파라미터의 일시적인 오래된 값은 최대 하나의 ACK에 영향을 미치며, 다음 ACK에서 수정됩니다.

유일한 예외: `sk->sk_pacing_rate` / `sk->sk_pacing_shift`는 사용자 공간이 `setsockopt`를 통해 동시에 수정할 수 있는 소켓 계층 필드이므로, BBR의 WRITE_ONCE/READ_ONCE 규칙이 유지됩니다.

## 성능 요약

테스트 환경: 중국 → 미국 LAX, 212ms RTT, 8개 병렬 흐름, 26% 패킷 손실, 1Gbps 공유 VPS 병목.

| 지표 | KCC v1.0 | BBR (대조군) | 변화율 |
|--------|----------|---------------|-------|
| 평균 처리량 | 1,010 Mbps | 937 Mbps | **+7.8%** |
| Intra-KCC 불공정도 | 3.1× | 6.2× (BBR) | **−50%** |
| 최저 단일 흐름 | 60.6 Mbps | 30.8 Mbps | **+97%** |
| 재전송 | 150K/10s | 137K/10s | +9.5% |
| 3라운드 안정성 | 959 Mbps | 883 Mbps | **+8.6%** |

재전송이 약간 더 높은 것은 손실 상황에서 높은 링크 사용률을 유지하는 데 따른 절충입니다. KCC의 칼만 기반 min_rtt 추정은 더 정확한 BDP 기준선을 제공하여, 동일 경로에서 BBRv1보다 높은 처리량을 유지할 수 있게 합니다.

---
## Global Kalman BDP — 교차 연결 대역폭 주입

KCC v1.0은 서버의 정상 상태 병목 대역폭을 추정하는 선택적 교차 연결 글로벌 칼만 필터를 포함합니다. 이 추정치는 새로운 연결을 보수적으로 낮은 "디저트 속도"로 부트스트랩하는 데 사용됩니다 — 콜드 스타트 램프업을 건너뛸 만큼 빠르면서도 오버슈트를 피할 만큼 느립니다.

### 설계 원칙

필터는 모든 KCC 연결의 PROBE_BW **크루즈 단계**(게인 = 1.0×)에서 대역폭 샘플을 제공받습니다. 크루즈 단계 샘플은 진정한 가용 대역폭의 가장 깨끗한 신호입니다 — 1.25× 프로브 오버슈트도, 0.75× 드레인 언더슈트도 없습니다. 1차원 랜덤 워크 칼만 필터(Kalman 1960)가 글로벌 정상 상태를 추적합니다.

새 연결이 설정되면, 필터의 추정치가 다음을 시딩하는 데 사용됩니다:

| 주입 값 | 목적 |
|----------------|---------|
| `minmax` (max\_bw 추적기) | 첫 몇 개의 더티 ACK 샘플이 대역폭을 0으로 끌어내리지 않도록 슬라이딩 윈도우 대역폭 이력을 시딩 |
| `sk_pacing_rate` | 중립 게인(BBR\_UNIT)에서의 초기 페이싱 속도; STARTUP의 2.89× 게인은 첫 ACK에서 적용됨 |
| `tp->snd_cwnd` | 중립 게인에서 `kcc_bdp()`를 통해 계산된 초기 혼잡 윈도우 |

`kcc_update_bw`의 방어적 하한은 STARTUP 중 처음 몇 RTT의 낮은 전송률 샘플이 주입된 추정치를 덮어쓰는 것을 방지합니다. `kcc_check_full_bw_reached`의 풀 BW 가드는 iperf3 제어 메시지 교환이 STARTUP을 조기 종료하는 것을 방지합니다.

### 디저트 속도 할인 비율

실효 주입 속도는 다음과 같습니다:

```
coeff = (discount_ratio) / high_gain
      = (num / den) / 2.89
```

여기서 `high_gain ≈ 2.89`는 BBR STARTUP 페이싱 배율입니다.

| num | coeff  | 특성 |
|-----|--------|----------------|
|  35 | 12.1%  | 최대 안전성, 최악 경로 |
|  50 | 17.3%  | 중심 축 (기본값) |
|  75 | 25.9%  | 수학적 디저트 최적점 |
|  80 | 27.6%  | 수학적 속도 상한 (초과하지 말 것) |

**참고:** `tcp_write_xmit`는 모든 새 연결에 대해 초기 CWND를 `TCP_INIT_CWND`(10 세그먼트, ≈15 KB)로 강제합니다. CWND는 원격 ACK가 도착할 때만 증가하므로, 디저트 속도는 페이싱 속도의 상한입니다 — 실제 처리량은 충분한 ACK가 수신되어 윈도우가 열릴 때까지 CWND에 의해 제한됩니다.

### 구성

`sysctl`을 통해 활성화:

```bash
sysctl -w net.kcc.kcc_kf_enable=1           # 마스터 활성화 (기본값 0)
sysctl -w net.kcc.kcc_kf_discount_num=50   # 디저트 속도 분자 (기본값 50, 범위 35–75)
```

**주요 sysctl 매개변수** (`/proc/sys/net/kcc/`):

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `kcc_kf_enable` | 0 | 0–1 | 글로벌 칼만 BDP 주입 마스터 활성화 |
| `kcc_kf_discount_num` | 50 | 0–100 | 디저트 속도 분자 (공정 점유 대역폭의 %) |
| `kcc_kf_discount_den` | 100 | 1–100000 | 디저트 속도 분모 |
| `kcc_kf_startup_r_pct` | 20 | 1–100 | 시작 단계 중 측정 노이즈 R% |
| `kcc_kf_steady_r_pct` | 5 | 1–100 | 정상 상태 중 측정 노이즈 R% |
| `kcc_kf_q_shift` | 20 | 0–30 | 프로세스 노이즈 시프트 (Q = 1 << shift) |
| `kcc_kf_chi2_num` | 384 | 1–100000 | 카이제곱 이상치 게이트 분자 |
| `kcc_kf_chi2_den` | 100 | 1–100000 | 카이제곱 이상치 게이트 분모 |

### 첫 1초 성능 (태평양 횡단, 212 ms RTT)

```
KF 미사용:  2.8 Mbps  →  85 Mbps  →  622 Mbps  →  정상 상태
KF 사용:    50 Mbps   →  530 Mbps  →  650 Mbps  →  정상 상태
```

첫 2초 속도가 ~3 Mbps(콜드 스타트)에서 ~50 Mbps(디저트 스타트)로 점프하며, 정상 상태로의 수렴은 2~3초 내에 도달합니다. 재전송은 전체 과정에서 0을 유지합니다.

### 작동 방식

1. 실행 중인 KCC 연결이 PROBE\_BW 크루즈 단계 진입 → 라운드 시작 경계 → `kcc_kf_update(bw, 5%)`에 현재 전송률 샘플을 제공합니다.
2. 칼만 필터가 추정치 `kcc_kf_x`(정상 상태 병목 대역폭의 이동 평균)를 업데이트합니다.
3. **새** 연결이 열릴 때, `kcc_init`이 `kcc_kf_get_init_bw(sk)`를 호출하고, 이는 `fair × discount / high_gain`을 반환합니다 — 게인 보상된 공정 점유 초기 대역폭 추정치입니다.
4. 이 추정치는 `sk_pacing_rate`, `tp->snd_cwnd`, 그리고 `minmax` 대역폭 추적기를 시딩합니다 — 연결이 0이 아닌 디저트 속도에서 시작됩니다.

### 알고리즘 출처

Global Kalman BDP 필터는 저자의 논문 《Linux 커널 공간에서의 전역 정상 상태 대역폭의 칼만 추정 및 공학적 구현에 관하여》(CC BY-SA 4.0)을 기반으로 합니다:
https://blog.csdn.net/liulilittle/article/details/161635652

---

*KCC v1.0 — BBRv1(Cardwell et al. 2016, ACM Queue) 및 칼만 필터(Kalman 1960) 기반.*

## 참고문헌

| Tag | Citation / Link |
|-----|----------------|
| BBR | Cardwell et al., "BBR: Congestion-Based Congestion Control", ACM Queue, Vol. 14 No. 5, 2016 — https://dl.acm.org/doi/10.1145/3009824 |
| BBR-S | "BBR-S: A Low-Latency BBR Modification for Fast-Varying Connections", 2021 — https://ieeexplore.ieee.org/document/9438951 |
| RBBR | "RBBR: A Receiver-Driven BBR in QUIC for Low-Latency in Cellular Networks", 2022 — https://ieeexplore.ieee.org/document/9703289 |
| ERCC | "ERCC: Fine-grained RDMA Congestion Control via Kalman Filter-based Multi-bit ECN Feedback Reconstruction", 2025 — https://dl.acm.org/doi/10.1145/3769270.3770124 |
| Linux BBR | Linux kernel BBR reference — https://github.com/torvalds/linux/blob/master/net/ipv4/tcp_bbr.c |
| Google BBR | BBR project page — https://github.com/google/bbr |
| BBRplus | "BBRplus: Adaptive Cycle Randomization, Drain-to-Target, and ACK Aggregation Compensation for BBR Convergence and Stall Prevention" — https://blog.csdn.net/dog250/article/details/80629551 |
| IETF 101 | "BBR Congestion Control Work at Google IETF 101 Update" — https://datatracker.ietf.org/meeting/101/materials/slides-101-iccrg-an-update-on-bbr-work-at-google-00 |