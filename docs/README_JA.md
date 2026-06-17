[🇺🇸 English](../README.md) | [🇨🇳 中文](README_CN.md) | [🇹🇼 繁體中文](README_TW.md) | [🇪🇸 Español](README_ES.md) | [🇫🇷 Français](README_FR.md) | [🇷🇺 Русский](README_RU.md) | [🇸🇦 العربية](README_AR.md) | [🇩🇪 Deutsch](README_DE.md) | [🇯🇵 日本語](README_JA.md) | [🇰🇷 한국어](README_KO.md) | [🇮🇹 Italiano](README_IT.md) | [🇵🇹 Português](README_PT.md)

---

# TCP KCC v1.0 (カルマン輻輳制御)

共有帯域幅 VPS 環境向け、BBRv1 状態マシンと伝搬遅延推定のためのカルマンフィルタを組み合わせた TCP 輻輳制御モジュール。

## 設計原則

輻輳制御アルゴリズムは、スループット、レイテンシ、公平性、損失耐性のバランスを取る必要があります。KCCは実用的なアプローチを採用します：

1. BBRv1は実績のある基盤を提供します。状態機械、ペーシング、サイクルゲイン、STARTUP/DRAIN/PROBE_BW/PROBE_RTT — KCCはこれらのメカニズムを変更せずに採用しています。

2. カルマンフィルタは推定精度を向上させます。真の伝搬遅延をキューイング遅延やジッタから分離することで、より正確なmin_rtt推定が可能になり、より厳密なBDP計算、より適切に調整されたCWND、より安定したペーシングを実現します。

3. アルゴリズム間のダイナミクスは標準的なTCP競争均衡に従います。KCCは外部フローから検出されたキューに応答して送信レートを人為的に制限しません。ゲイン減衰（キュー based プローブ削減）はkcc_cycle_decay_maskを介してオプトイン可能ですが、完全なプローブ強度を維持するためにデフォルトでは無効です。

4. Intra-KCCの公平性は積極的に維持されます。カルマン収束により、同一ホスト上のKCCフローが一貫したmin_rtt推定値を共有し、純粋なBBRマルチフロー展開で深刻な不公平を引き起こす勝者総取りのフィードバックループを排除します。

## アルゴリズム概要

TCP KCC は、ロード可能な `tcp_kcc.ko` として Linux カーネルに送信側輻輳制御モジュールを実装します。輻輳制御関数 `kcc_main()` は、`tcp_ack()` からの各 ACK で呼び出され、カーネルレベルの帯域幅と RTT サンプル、および配信数と損失数を含む `rate_sample` 構造体を受け取ります。アルゴリズムは 2 つの時間領域で動作します。**ACK ごとの高速パス** は測定状態を更新し、瞬時のペーシングとウィンドウ目標を計算します。**ラウンドごとの低速パス** は状態遷移条件を評価し、ゲインを再計算します。

コア測定パイプラインは 2 つのコンポーネントで構成されます：

1. **スライディングウィンドウ最大帯域幅フィルタ**（`linux/win_minmax.h` の `minmax_running_max`）：最後の `kcc_bw_rt_cycle_len`（デフォルト 10）ラウンドトリップをカバーするウィンドウ。BBR 互換の `max_bw` 推定値を提供します。

2. **カルマンフィルタ伝搬遅延推定器**：BBRv1 のスライディングウィンドウ最小 RTT を置き換え、BDP RTT 推定のデフォルトソース（[モデル RTT 戦略](#モデル-rtt-戦略) 参照）。`kcc_kalman_scale` × µs 固定小数点単位で動作する単一状態カルマンフィルタ（Kalman 1960）で、真の伝搬遅延をランダムウォークとしてモデル化します：
   - 状態：`x[k] = x[k−1] + w[k]`、`w ~ N(0, Q)`
   - 観測：`z[k] = x[k] + v[k]`、`v ~ N(0, R)`

固定小数点の規約：`BW_UNIT = 1 << 24` は帯域幅用（セグメント * 2^24 / µs）、`BBR_UNIT = 1 << 8 = 256` は無次元ゲイン単位。

## モデル RTT 戦略

KCC は、BDP（帯域幅遅延積）計算に使用する RTT 推定の戦略を `kcc_rtt_mode` で構成可能にします：

| モード | 値 | 動作 | 用途 |
|------|----|------|----------|
| FILTER | 1（デフォルト） | `x_est_us` を直接使用 — 生のカルマン/スライディングウィンドウフィルタ推定値 | 本番 WAN/VPS：経路変更に強く、スループットの断崖ゼロ |
| MIN | 0 | `min(x_est_us, min_rtt_us)` — カルマン推定をウィンドウ最小値でクランプ | カーネルモジュール安定性検証、静的RTTリンク |

**FILTER がデフォルトである理由：**

- **経路変更耐性**：BGP リルートで物理 RTT が増加した場合（例：50 ms → 100 ms）、カルマンゲイン K_k が数 RTT 以内に新しい経路遅延に追従。MIN モードは古い `min_rtt_us` にデッドロックし、BDP が半減。

- **組み込み防御**：外れ値ゲーティングがキュー急増サンプルをフィルタに入る前に拒否。適応 Q/R ノイズ推定がネットワークのノイズが高いときにカルマンゲインを低減し、フィルタは自然に一時的なキュー膨張を信用せず、推定値を真の伝播遅延付近に維持。

- **PROBE_RTT 分離**：FILTER モードは `kcc_probe_rtt_decouple` 機能を有効化 — カルマンフィルタが定期的な4パケットドレインなしで RTT 下限を追跡。

実行時切替：`echo 0 > /proc/sys/net/kcc/kcc_rtt_mode` で MIN モードに戻す。

## 状態機械

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

`struct KCC` の 2 ビット `mode` フィールドにエンコードされた 4 つのモード：

- **STARTUP (0)**：初期状態。pacing_gain ≈ 2.885x（`kcc_high_gain_val`）、cwnd_gain も 2.885x。指数関数的な帯域幅プロービング。
- **DRAIN (1)**：STARTUP 終了後に入移行。pacing_gain ≈ 0.347x（`kcc_drain_gain_val`）、cwnd_gain は 2.885x のまま。STARTUP 中に蓄積されたキューを排出。
- **PROBE_BW (2)**：定常状態。256 スロットのゲインテーブルを循環（デフォルトの 8 フェーズパターンを繰り返し：1.25x/0.75x/8×1.0x）。
- **PROBE_RTT (3)**：定期的にインフライトを `kcc_cwnd_min_target`（デフォルト 4 セグメント）まで排出し、新しい RTT サンプルを取得。

### STARTUP → DRAIN

`full_bw_reached` が設定されるとトリガー — `kcc_full_bw_cnt`（デフォルト 3）回の連続ラウンドで `max_bw` が以前のピークと比較して少なくとも `kcc_full_bw_thresh_val`（デフォルト 1.25x）成長しなかった場合。1.0x ゲインでの BDP が `snd_ssthresh` に書き込まれる。`qdelay_avg` はゼロにリセットされ、STARTUP のキュー蓄積が PROBE_BW に影響するのを防ぐ。

### DRAIN → PROBE_BW

推定 EDT でのインフライト ≤ 1.0x BDP ゲインでの目標インフライトの場合にトリガー。**Drain-skip 最適化**：カルマンフィルタが収束し、かつ `qdelay_avg < 動的クリーン閾値`（min_rtt_usの10%、500us下限）の場合、DRAIN フェーズはスキップされ、早期に PROBE_BW に移行する。

PROBE_BW エントリ時に、サイクルフェーズインデックスはランダム化される：`cycle_idx = len − 1 − rand(kcc_probe_bw_cycle_rand)`（デフォルト `len − 1 − rand(8)`）。これにより、ボトルネックリンクを共有する同時フロー間の相関が除去される。

### PROBE_BW → PROBE_RTT

PROBE_RTT フィルタ間隔が期限切れになるとトリガー — 計算された間隔内にタイムスタンプ `min_rtt_stamp` が更新されていない場合。cwnd は `prior_cwnd` に保存され、ペーシングは排出モードに設定される。

### PROBE_RTT → PROBE_BW

インフライトが `kcc_cwnd_min_target` まで低下するか、ラウンド境界が観測された後、少なくとも `kcc_probe_rtt_mode_ms_val`（デフォルト 200 ms）以上かつ少なくとも 1 完全ラウンドが観測された後に終了。cwnd は `prior_cwnd` に復元され、標準 PROBE_BW ゲインサイクルに移行する。

### リカバリと損失

- TCP_CA_Loss 時：`full_bw` と `full_bw_cnt` がリセットされ、`round_start` が 1 に設定され、`packet_conservation` が 0 にクリアされる。
- リカバリエントリ（TCP_CA_Recovery）：`packet_conservation` が有効化、cwnd = inflight + acked。
- リカバリ終了：`prior_cwnd` に復元、`packet_conservation` がクリアされる。
- `kcc_undo_cwnd()`：`full_bw` と `full_bw_cnt` をリセット（`full_bw_reached` は保持）、LT BW 状態をクリア。

### ラウンド検出（BBR アライメント）

`next_rtt_delivered` は `0` に初期化され（標準 BBR と同様; Cardwell et al. 2016）、最初の ACK で即座にラウンド1検出が開始され、人為的なオフセットは発生しません。`prior_delivered >= next_rtt_delivered` でラウンド境界が検出され、`interval_us <= 0` ガード（BBR の `delivered < 0 || interval_us <= 0` に一致）により、ゼロ値および負値の間隔を捕捉し、それらが測定パイプラインを汚染するのを防ぎます。

- `next_rtt_delivered` は `0` に初期化（BBR 準拠）：最初の ACK で最初のラウンドが開始。
- `interval_us <= 0` 検証：BBR に完全一致、負の間隔を拒否。
- `round_start` は `kcc_update_bw()` の先頭で、検証チェックの前に `0` にリセット — BBR の `bbr->round_start = 0` 配置に一致。

## コア計測

### 帯域幅推定

スライディングウィンドウ最大帯域幅フィルタ（`linux/win_minmax.h` の `minmax_running_max`）を `kcc_bw_rt_cycle_len`（デフォルト 10）ラウンドにわたって適用。瞬時 bw = `delivered × BW_UNIT / interval_us` を ACK ごとに計算。アプリ制限されていない場合、または bw ≥ 現在の最大値（BBR ルール）の場合にのみスライディングウィンドウに入力される。

`lt_use_bw` がアクティブな場合、アクティブな帯域幅推定は `lt_bw`（長期帯域幅推定）に切り替わる。

### カルマンフィルタ

単一状態スカラーカルマン再帰（O(1) 複雑度）：

```
予測：
  x_pred = x_est          （恒等状態遷移）
  p_pred = p_est + Q      （共分散予測）

更新：
  innov   = z − x_pred    （イノベーション）
  K       = p_pred / (p_pred + R)   （カルマンゲイン [0,1]）
  x_est   = x_pred + K × innov      （状態更新）
  p_est   = (1 − K) × p_pred        （事後共分散）
```

**適応的プロセスノイズ Q**：
```
Q_base   = kcc_kalman_q（デフォルト 100）
q_factor = max(kcc_kalman_q_min_factor_val, min_rtt_us / kcc_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × kcc_kalman_q_scale_cap)
Q        = min(Q, kcc_kalman_q_max)
```

**適応的測定ノイズ R**：
```
R = R_base + max(0, jitter_ewma − clean_thresh) × R_base / kcc_jitter_r_scale
R = min(R, R_base × kcc_kalman_r_max_boost)
```

**Q-Boost 経路変更検出（信頼度ゲート + クールダウン付き）**: `|innovation| > kcc_kalman_q_boost_thresh_val`（デフォルト約4msのRTTシフト）かつフィルタが収束している場合（`p_est ≤ kcc_kalman_converged_p_est_val`、デフォルト500）、`p_est` が `kcc_kalman_p_est_init_val` にリセットされ、カルマンゲインが1.0に引き上げられて急速な収束を実現します。連続するqboostイベント間の `kcc_kalman_qboost_cdwn`（デフォルト15）サンプルのクールダウンにより、高いRTTジッタを持つ損失の多い経路での暴走トリガーを防止します。

**外れ値ゲート**：動的しきい値 `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`。`p_pred ≤ kcc_kalman_converged_p_est_val` の場合のみ適用。`kcc_kalman_max_consec_reject`（デフォルト 25）回の連続棄却後、次のサンプルは強制受理され、自己強化ロックインを防止。

**共分散整合ノイズ推定（BBR-S）**：`q_est = (1−α) × q_est + α × (K × innov)²`、`r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`。組み合わせモード：モード 0 = ヒューリスティックのみ、モード 1 = 最大値（デフォルト）、モード 2 = 加重混合。

**カルマン制御引き継ぎ**：`x_est > 0` かつ `sample_cnt ≥ kcc_kalman_min_samples`（デフォルト 5）の場合、`min_rtt_us` は `x_est / kcc_kalman_scale` に置き換えられる。`min_rtt_stamp` は更新されない — PROBE_RTT 間隔トリガーは独立したまま。

**モデル RTT 戦略**：BDP 計算に使用される RTT 推定は `kcc_rtt_mode` で制御される。FILTER モード（デフォルト）では、`model_rtt = x_est_us` を直接使用 — カルマン/スライディングウィンドウ推定をクランプなしで使用。MIN モードでは、`model_rtt = min(x_est_us, min_rtt_us)` — カルマン推定をウィンドウ最小値でクランプし、BDP が決して膨張しないことを保証。FILTER デフォルトは、経路遅延が突然変化する可能性のある本番 WAN/VPS デプロイメント（BGP リルート、LEO ハンドオーバー、モバイルセル切替）に推奨。[モデル RTT 戦略](#モデル-rtt-戦略) を参照。

## BBR 拡張機能

### ゲイン減衰

特定の PROBE_BW フェーズに対して 256 ビットビットマップ `kcc_cycle_decay_mask[]` で有効化。減衰式（受理されたカルマンサンプル時）：

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = p_est の逆スケーリング（最大時 BBR_UNIT）
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                    × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                    × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

カルマン信頼度スケーリング：`p_est > kcc_kalman_converged_p_est` の場合、減衰は比例的に減少し、フィルタが不確かな場合の過度なバックオフを回避。

### ECN バックオフ

活性化条件（すべて成立する必要あり）：
1. `kcc_ecn_enable_val != 0`
2. カルマン収束済み（`p_est < converged`、`sample_cnt >= min_samples`）
3. `ecn_ewma > 0`（CE マーク観測）
4. `qdelay_avg > 動的輻輳閾値`（min_rtt_usの25%、500us下限）
5. モードが PROBE_BW ではない（PROBE_BW では cwnd_gain は 2x で固定）

プロービングフェーズ中（`pacing_gain > BBR_UNIT`）、ECN バックオフは `BBR_UNIT² / pacing_gain` で段階的に適用 — 1.25x プローブで約 80% のバックオフ、2.89x STARTUP ゲインで約 65%。

ECN マーク比率 EWMA：ラウンド境界で `kcc_ecn_ewma_retained / kcc_ecn_ewma_total`（デフォルト 3/4）によって更新され、新しい CE マークのない各 ACK で `kcc_ecn_idle_decay_num / kcc_ecn_idle_decay_den`（デフォルト 31/32）の穏やかな ACK ごとの減衰が行われる。

### 単一フロー検出

KCC がボトルネック上でフローが単独であると推定した場合（低キュー遅延、低ジッター、ECN マークなし、ACK 集約なし）、自動的に BBR ピュアモードに移行します：

- `kcc_get_model_rtt()` は `min_rtt_us` を直接返します（片側測定ノイズによる小さい正バイアスを持つカルマン平滑化推定をバイパス）。
- `kcc_ecn_backoff()` の応答は `kcc_alone_bypass_ecn`（デフォルト 1）で設定可能——単一フローパスには ECN マークを共有する競合送信者が存在せず、マークは AQM の誤検出であるため、スキップして BBR のゼロ ECN 動作に一致させる。0 に設定すると単独モードでも ECN バックオフを維持（保守的）。

これにより、KCC と BBR の間の単一フロースループット格差が解消され、マルチフローシナリオでは KCC の完全な保護ループ（カルマン、ECN バックオフ、ゲイン減衰）が維持されます。

**ヒステリシス**: エントリには `kcc_alone_confirm_rounds`（デフォルト 3）回の連続した適格ラウンドが必要です——マルチフロー競合中の短い静穏期間での振動を防止します（「保守的に加速」）。退出：巡航フェーズ評価中に、いずれかの適格条件が失敗するとフラグがクリアされます（「積極的に制動」）。

**設計上のトレードオフ**：パケットロスは単一フロー非適格条件として使用されません——一部のリンク（浅いバッファ、無線、仮想化バーストドロップ）には競合とは無関係な固有のロスがあります。ロスをマルチフロー競合と同一視すると、単一フローパスで振動が発生します。LT BW 信号（BBR のロス検出ポリサー検出）は単一フロー判定に参加しません。

**ゲインゲーティング**：単一フロー評価はクルーズフェーズ（`pacing_gain == BBR_UNIT`）でのみ実行されます。プローブアップ（1.25x）は意図的にボトルネックを押し上げます——そのキュー圧力は自己誘発的であり、競合シグナルではありません。ドレイン（0.75x）はキューを人為的に抑制します。評価をクルーズ（定常状態の均衡）に制限することで、自己誘発的なプローブアップ圧力が誤った単独モード退出を引き起こすことはなくなります。

適格条件（ラウンド境界で 5 つすべてが満たされる必要があります）：
0. カルマン収束（`sample_cnt >= kcc_kalman_min_samples`） — qdelay/jitter をキュー信号として信頼
1. `qdelay_avg < 動的クリーン閾値` — キューがほぼ空
2. `jitter_ewma < 動的輻輳閾値` — ACK クロックのマイクロジッターのみ
3. `ecn_ewma == 0` — AQM からの輻輳マークなし
4. `agg_state <= max` `kcc_alone_agg_state_level` に従う（デフォルト 1）— 3段階の ACK 集約厳格度:
   - 0 = IDLE のみ（最も厳格、ゼロ集約）、1 = ≤ SUSPECTED（デフォルト、一時的集約を許容）、2 = ≤ CONFIRMED（最も許容的、持続的集約のみ遮断）

### 動的 PROBE_RTT 間隔

カルマン `p_est` を接続ごとの PROBE_RTT 間隔にマッピング：

```
p_est ≤ converged：              間隔 = dyn_max（デフォルト 30 秒）
p_est ≥ high（= mult × conv）：   間隔 = base（デフォルト 10 秒）
converged < p_est < high：       線形補間
```

信頼度が高い（低 `p_est`）場合に PROBE_RTT 頻度を減らし、安定した経路でのスループットジッタを低減。信頼度が低い場合は従来の 10 秒間隔に戻る。

**フローエントリジッタ**: すべての共存フローが同時に PROBE_RTT に入るのを防ぐため（4 パケット集約 ~1.8 Mbps まで排出し、その後 2.89× で再充填）、各フローはハッシュ派生ジッタ（0–845 ms の分布）を自身の PROBE_RTT 間隔に追加します。任意の瞬間に最大 ~1 フローが PROBE_RTT にあり、RTO を誘発する同時排出/再充填の崩壊を排除します。

### PROBE_RTT 分離とスマート再校正

BBRv1 の PROBE_RTT 機構は、約10秒ごとにパイプを4パケットまでドレインして `min_rtt_us` を測定する。これはウィンドウベースの最小RTT推定器に必要 — パイプが空でない限り、ウィンドウは伝播遅延とキューイング遅延を区別できない。代償は周期的なスループット断崖（BBR「のこぎり波」）。

FILTER モードでは、カルマンフィルタがウィンドウを完全に置き換える。外れ値ゲーティングと適応ノイズ推定により、キューイングノイズから真の伝播遅延を分離可能 — パイプドレイン不要。パラメータ `kcc_probe_rtt_decouple`（デフォルト 1）がこれを制御：

| モード | 値 | 動作 |
|------|----|------|
| 分離 | 1（デフォルト） | **カルマン健全**（p_est ≤ `kcc_recal_p_est_thresh`）：PROBE_RTT を完全抑制 → スループット断崖ゼロ、同期崩壊ゼロ。**カルマン発散**（p_est > 閾値）：安全網として従来の PROBE_RTT を自動発動 → フィルタベースラインを復元し、分離が自動再開。 |
| 従来 | 0 | 盲目的な周期的 PROBE_RTT 約10秒ごと（BBR互換）。 |

**スマート再校正ヒューリスティック**（`kcc_kalman_needs_recalibration()`）：安定経路上の定常動作では、カルマン誤差共分散 p_est が p_est_floor（~4–10）に収束し、閾値 `kcc_recal_p_est_thresh`（250,000 = p_est_max の25%）よりはるかに低い。p_est の上昇は、フィルタの内部モデルが観測をもはや説明できないことを示す — 通常は経路が実質的に変化した場合。p_est が閾値を超えると、1回の従来型 PROBE_RTT ドレインがフィルタベースラインを復元；カルマンが再収束し、分離が自動再開。

これにより PROBE_RTT が **盲目的な周期的自傷行為** から **インテリジェントな信頼度駆動再校正** へと変革される — プロトコルは、フィルタが信頼を失ったという経験的証拠がある場合にのみパイプをドレインする。

`kcc_rtt_mode == 1` が必要。MIN モードでは無効（MIN モードは `min_rtt_us` の更新に PROBE_RTT が必要）。

| パラメータ | デフォルト | 範囲 | 説明 |
|------|------|------|------|
| `kcc_probe_rtt_decouple` | 1 | 0–1 | PROBE_RTT 分離を有効化（FILTER モードのみ） |
| `kcc_recal_p_est_thresh` | 25,000 | 1–100,000,000 | 安全網再校正を発動する p_est 閾値 |

### LT 帯域幅推定

損失トリガー型下限推定器。サンプリング間隔は [4, 16] RTT の範囲。損失率 ≥ 5.9%（`kcc_lt_loss_thresh` デフォルト 15/256）の場合に有効。帯域幅 `bw = delivered × BW_UNIT / interval_us`。

BBR の単純平均（`(bw + lt_bw) >> 1`）とは異なり、KCC は設定可能な EMA（`kcc_lt_bw_ema_num / kcc_lt_bw_ema_den`、デフォルト 1/2 = 0.5）を使用：

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

活性化は BBR と異なる：KCC は最初の有効な間隔で `lt_bw` を保存するが、`lt_use_bw` は設定しない。前の間隔との整合性が必要 — 測定ノイズによる誤活性化を低減。

**二重閾値輻輳ゲート**: `lt_use_bw = 1` を設定する前に、永続的 EWMA キュー検査（`qdelay_avg > 動的輻輳閾値`）と SRTT ベースの瞬時キュー検査（`srtt_us − min_rtt_us > 瞬時輻輳閾値`、デフォルト 5000 µs）の両方が評価されます。輻輳が検出されると、LT BW サンプリングは中止されます。SRTT 検査は `ext` 割り当てなしで動作し、割り当て失敗に対するセーフティネットを提供します。




### 信頼度ベース ACK 集約補償（BBRplus 由来）

従来のデュアルスロット extra-acked 推定器の上に信頼度ゲート付きの第 2 層を追加。

**4 つの直交因子**（各因子は `kcc_agg_factor_weight` ポイント（デフォルト 256）を寄与）：
1. カルマン収束済み（`p_est < converged` + `sample_cnt >= min_samples`）
2. 損失リカバリ中でない（`icsk_ca_state < TCP_CA_Recovery`）
3. RTT が真の伝搬遅延から `min_rtt_us + 動的クリーン閾値` 以内
4. `extra_acked` がウィンドウ最大値の `kcc_agg_factor4_ratio_num/den`（デフォルト 1.5x）以内

**4 つの状態**：IDLE（< `kcc_agg_thresh_suspected`=256）、SUSPECTED（≥256）、CONFIRMED（≥512）、TRUSTED（≥768）。

**信号層**（常時活性）：信頼度が R スケーリング係数 `[r_min, r_max]` を線形補間。R は瞬時に上昇（高速応答）、RTT ごとに `kcc_agg_r_hysteresis`%（デフォルト 75% 保持、約 4 RTT でベースラインに）で減衰。

**制御層**（`agg_state ≥ CONFIRMED`）：5 層の安全ゲート付き cwnd 補償：
1. キュー遅延 > `動的輻輳閾値` の場合はブロック
2. 損失リカバリ中はブロック
3. cwnd > `BDP × kcc_agg_safety_bdp_mult`（デフォルト 3x）の場合はブロック
4. インフライト > 安全 cwnd + TSO セグメント目標の場合はブロック
5. ウォッチドッグ：`kcc_agg_max_comp_duration`（デフォルト 8）回の連続 RTT 後に CONFIRMED→SUSPECTED に降格

### DRAIN 時 qdelay_avg リセット

DRAIN への移行時に `qdelay_avg` はゼロにリセットされ、STARTUP のキュー推定が PROBE_BW に持続するのを防止。

### TSO 除数適応

`kcc_min_tso_segs()` はカルマン状態に基づいてレートしきい値除数を調整：
- カルマン収束済み + `jitter_ewma < 1000 µs`：除数が半減（8→4）、より大きな TSO バースト
- `jitter_ewma > 4000 µs`：除数が倍増（8→16）、より小さな TSO バーストでジッタ抑制

## ペーシングレートと Cwnd

### ペーシングレート

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // ゲイン調整
rate = rate × USEC_PER_SEC >> BW_SCALE            // バイト/秒に変換
rate = rate × margin_div / 100                    // ペーシングマージン（デフォルト 1%、matching BBR）
```

レート変更は即座に適用され（平滑化なし）、BBR と同様（Cardwell et al. 2016）。`full_bw_reached` 後：すべてのレート変更は即座に書き込まれる。STARTUP/DRAIN 中：増加のみ適用（`rate > sk_pacing_rate`）。

### Cwnd

```
target = BDP(bw, gain, ext)                       // 基本 BDP
target = quantization_budget(target)              // TSO ヘッドルーム + 偶数ラウンド + フェーズ 0 ボーナス
target += ack_agg_bonus + agg_compensation        // ACK 集約補償

// cwnd 進行
if full_bw_reached:
    cwnd = min(cwnd + acked, target)              // 目標に収束
else (STARTUP):
    cwnd = cwnd + acked                          // 指数関数的成長

cwnd = max(cwnd, cwnd_min_target)                 // 絶対フロア 4
PROBE_RTT モード：cwnd = min(cwnd, cwnd_min_target) // 最小インフライト
```

## モジュールパラメータ

パラメータは `/proc/sys/net/kcc/` で公開。書き込みは `kcc_init_module_params()`（検証 + クランプ + 派生値計算）をトリガー。配列パラメータの書き込みは `kcc_rebuild_gain_table()` をトリガー。

### PROBE_RTT 間隔

| パラメータ | デフォルト | 最小 | 最大 | 単位 | 説明 |
|-----------|---------|-----|-----|------|-------------|
| `kcc_probe_rtt_base_sec` | 10 | 1 | 86400 | 秒 | 基本 PROBE_RTT 間隔 |
| `kcc_probe_rtt_max_sec` | 15 | 1 | 86400 | 秒 | 長 RTT 経路の上限 |
| `kcc_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | 秒 | 最大動的間隔、0 で無効 |

### ゲイン

| パラメータ | デフォルト | 最小 | 最大 | 説明 |
|-----------|---------|-----|-----|-------------|
| `kcc_cwnd_gain_num` / `kcc_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | PROBE_BW のベースライン cwnd ゲイン |
| `kcc_extra_acked_gain_num` / `kcc_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | ACK 集約ボーナス乗数 |
| `kcc_high_gain_num` / `kcc_high_gain_den` | 2885 / 1000 | 0/1 | 100k | STARTUP ゲイン（≈2.885x） |
| `kcc_drain_gain_num` / `kcc_drain_gain_den` | 347 / 1000 | 0/1 | 100k | DRAIN ゲイン（≈0.347x） |
| `kcc_gain_num[i]` / `kcc_gain_den[i]` | BBRv1 パターン（256 スロット） | 0/1 | — | スロットごとのペーシングゲイン |
| `kcc_cycle_decay_mask[8]` | 0（すべてゼロ） | 0 | 0x7FFFFFFF | 256 ビット減衰ビットマップ |
| `kcc_probe_bw_up_limit` | 0 | 0 | 1 | プローブアップの制限付き終了（0=オフ） |

### カルマンベース

| パラメータ | デフォルト | 最小 | 最大 | 説明 |
|-----------|---------|-----|-----|-------------|
| `kcc_kalman_q` | 100 | 0 | 100k | 基本プロセスノイズ Q |
| `kcc_kalman_r` | 400 | 0 | 100k | 基本測定ノイズ R |
| `kcc_kalman_p_est_max` | 1,000,000 | 1 | 100M | p_est 絶対最大値 |
| `kcc_kalman_converged_p_est` | 500 | 1 | 1M | 収束しきい値 |
| `kcc_kalman_p_est_init` | 1000 | 1 | 10M | 初期 p_est |
| `kcc_kalman_p_est_floor` | 10 | 1 | 100k | p_est フロア |
| `kcc_kalman_scale` | 1024 | 64 | 1,048,576 | 固定小数点スケール（2 のべき乗） |
| `kcc_kalman_min_samples` | 5 | 3 | 20 | 引き継ぎ前の最小サンプル数 |
| `kcc_kalman_outlier_ms` | 5 | 0 | 10000 | ms | 外れ値基本しきい値 |
| `kcc_kalman_q_boost_mult` | 4 | 1 | 10000 | Q-boost 乗数 |
| `kcc_kalman_q_boost_ms` | 1 | 0 | 5000 | ms | Q-boost 時定数 |
| `kcc_kalman_qboost_cdwn` | 8 | 1 | 255 | samples | Q-boost クールダウン |
| `kcc_kalman_q_max` | 2000 | 1 | 100k | Q 上限 |
| `kcc_kalman_q_scale_cap` | 50 | 1 | 10000 | Q スケール上限 |
| `kcc_kalman_max_consec_reject` | 25 | 1 | 1000 | 強制受理前の最大連続棄却数 |
| `kcc_rtt_sample_max_us` | 500000 | 1 | 10M | µs | カルマン RTT 上限 |
| `kcc_kalman_r_max_boost` | 8 | 1 | 1000 | R 最大ブースト乗数 |
| `kcc_kalman_rtt_dyn_mult` | 2 | 1 | 100 | RTT 動的上限乗数 |
| `kcc_kalman_q_rtt_div` | 1000 | 1 | 1M | Q 適応 RTT 除数 |
| `kcc_kalman_probe_band_mult` | 4 | 1 | 32 | PROBE_RTT 遷移帯域乗数 |

### カルマン追加（num/den 型）

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `kcc_kalman_outlier_jitter_mult_num/den` | 3 / 1 | 0-1000 / 1-100k | 外れ値ジッタ乗数 |
| `kcc_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | Q 最小係数 |
| `kcc_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | p_est 初期化 RTT 除数 |

### BBR-S ノイズ推定

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `kcc_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | Q 推定学習率 |
| `kcc_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | R 推定学習率 |
| `kcc_kalman_noise_mode` | 1 | 0-2 | 組み合わせモード（0=オフ、1=最大、2=加重平均） |
| `kcc_kalman_q_est_max` | 50,000 | 1-2B | Q 推定上限 |
| `kcc_kalman_r_est_max` | 32,000 | 1-2B | R 推定上限 |
| `kcc_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | 推定ごとの下限 |

### ゲイン減衰（プロービング）

| パラメータ | デフォルト | 範囲 | 単位 | 説明 |
|-----------|---------|-------|------|-------------|
| `kcc_qdelay_probe_scale_us` | 20000 | 1-100k | µs | qdelay 減衰スケール |
| `kcc_jitter_probe_scale_us` | 16000 | 1-100k | µs | ジッタ減衰スケール |

### 動的キュー遅延閾値

| パラメータ | デフォルト | 範囲 | 単位 | 説明 |
| `qdelay_clean_bp` | 1000 | 1-10000 | ‱ | クリーン閾値（10% × min_rtt_us） |
| `qdelay_cong_bp` | 2500 | 1-10000 | ‱ | 輻輳閾値（25% × min_rtt_us） |
| `qdelay_floor_us` | 500 | 1-100k | µs | 絶対下限、これを下回るとRTTパーセンテージが上書きされる |

### 適応的 R（ジッタ駆動）

| パラメータ | デフォルト | 範囲 | 単位 | 説明 |
|-----------|---------|-------|------|-------------|
| `kcc_jitter_r_scale` | 8000 | 1-100k | — | R 増加スケール除数 |

### ECN

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `kcc_ecn_enable` | 1 | 0-1 | ECN マスタースイッチ |
| `kcc_ecn_backoff_num` / `kcc_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | ECN バックオフ割合 |
| `kcc_ecn_ewma_retained` / `kcc_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | ECN EWMA 重み |
| `kcc_ecn_idle_decay_num` / `kcc_ecn_idle_decay_den` | 31 / 32 | 1-100k | アイドル ECN 減衰 |

### min_rtt

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `kcc_minrtt_fast_fall_cnt` | 3 | 0-3 | 高速低下カウント |
| `kcc_minrtt_fast_fall_div` | 4 | 1-256 | 高速低下しきい値除数 |
| `kcc_minrtt_sticky_num` / `kcc_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | 粘着低下比率 |
| `kcc_minrtt_srtt_guard_num` / `kcc_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | SRTT ガード比率 |

### LT 帯域幅

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `kcc_lt_intvl_min_rtts` | 4 | 1-127 | RTT | 最小間隔長 |
| `kcc_lt_intvl_max_mult` | 4 | 1-32 | 間隔タイムアウト乗数 |
| `kcc_lt_loss_thresh` | 25 | 1-65535 | BBR_UNIT | 最小損失率 |
| `kcc_lt_bw_ratio_num` / `kcc_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | 相対許容差 |
| `kcc_lt_bw_diff` | 500 | 0-100k | バイト/秒 | 絶対許容差 |
| `kcc_lt_bw_max_rtts` | 48 | 1-4094 | RTT | LT BW 最大アクティブ RTT 数 |
| `kcc_lt_bw_ema_num` / `kcc_lt_bw_ema_den` | 1 / 2 | 0-100 / 1-100k | LT BW EMA 重み |


### ACK 集約信頼度

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `kcc_agg_enable` | 1 | 0-1 | マスタースイッチ |
| `kcc_agg_confidence_thresh` | 512 | 0-10000 | cwnd 補償信頼度しきい値 |
| `kcc_agg_max_comp_ratio` | 50 | 0-100 | BDP の % | cwnd 補償上限 |
| `kcc_agg_max_comp_duration` | 8 | 1-128 | RTT | ウォッチドッグタイムアウト |
| `kcc_agg_r_hysteresis` | 75 | 0-100 | % | R ヒステリシス減衰 |
| `kcc_agg_r_multiplier_min` / `kcc_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | R スケーリング範囲（256=1x） |
| `kcc_agg_factor4_ratio_num` / `kcc_agg_factor4_ratio_den` | 3 / 2 | 1-100k | 因子 4 比率 |
| `kcc_agg_safety_bdp_mult` | 3 | 1-100 | 安全ガード BDP 乗数 |
| `kcc_agg_max_window_ms` | 100 | 1-10000 | ms | extra_acked 上限ウィンドウ |
| `kcc_agg_max_decay_pct` | 75 | 0-100 | % | ウォッチドッグ減衰率 |
| `kcc_agg_window_rotation_rtts` | 5 | 1-65535 | RTT | ウィンドウ回転期間 |
| `kcc_agg_factor_weight` | 256 | 1-1024 | 因子ごとのスコア |
| `kcc_agg_confidence_max` | 1024 | 256-65535 | 最大信頼度 |

### EWMA 係数

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `kcc_ewma_qdelay_num` / `kcc_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | qdelay EWMA 重み |
| `kcc_ewma_jitter_num` / `kcc_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | ジッタ EWMA 重み |

### その他

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `kcc_probe_bw_cycle_len` | 8 | 2-256 | PROBE_BW サイクルフェーズ数（2 のべき乗） |
| `kcc_probe_bw_cycle_rand` | 8 | 1-cycle_len | サイクルフェーズランダムオフセット |
| `kcc_full_bw_thresh_num` / `kcc_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | STARTUP 終了成長しきい値 |
| `kcc_full_bw_cnt` | 3 | 1-3 | 終了までの非成長ラウンド数 |
| `kcc_probe_rtt_mode_ms_num` / `kcc_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | PROBE_RTT 滞在時間 |
| `kcc_pacing_margin_num` / `kcc_pacing_margin_den` | 1 / 100 | 0-50 / 1-100k | ペーシングマージン（1 = 1%） |
| `kcc_probe_cwnd_bonus` | 2 | 0-100 | セグ | フェーズ 0 cwnd ボーナス |
| `kcc_bw_rt_cycle_len` | 10 | 2-256 | ラウンド | BW スライディングウィンドウ長 |
| `kcc_cwnd_min_target` | 4 | 1-1000 | セグ | 最小 cwnd（PROBE_RTT） |
| `kcc_bdp_min_rtt_us` | 1 | 0-100k | µs | BDP min_rtt フロア |
| `kcc_edt_near_now_ns` | 1000 | 0-10M | ns | EDT ニアナウしきい値 |
| `kcc_min_tso_rate` | 1,200,000 | 1-1B | バイト/秒 | TSO 低レートしきい値 |
| `kcc_min_tso_rate_div` | 8 | 1-256 | TSO レート除数（適応的ベース） |
| `kcc_tso_max_segs` | 127 | 1-65535 | セグ | 最大 TSO セグメント数 |
| `kcc_tso_segs_low` | 1 | 1-65535 | セグ | 低レート時 TSO セグメント数 |
| `kcc_tso_segs_default` | 2 | 1-65535 | セグ | 通常レート時 TSO セグメント数 |
| `kcc_tso_headroom_mult` | 3 | 0-1000 | TSO ヘッドルーム乗数 |
| `kcc_sndbuf_expand_factor` | 3 | 2-100 | 送信バッファ拡張係数 |
| `kcc_ack_epoch_max` | 0xFFFFF | 64K-2G | バイト | ACK エポック上限 |
| `kcc_extra_acked_max_ms_num` / `kcc_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | 最大 ACK 集約ウィンドウ |
| `kcc_rtt_mode` | 1 | 0-1 | モデル RTT 戦略：1=FILTER（カルマン直接）、0=MIN（クランプ） |
| `kcc_probe_rtt_decouple` | 1 | 0-1 | PROBE_RTT 分離（FILTER モードのみ）：4パケットドレイン抑制 |
| `kcc_recal_p_est_thresh` | 25,000 | 1-100M | 安全網再校正のカルマン p_est 閾値 |
| `kcc_probe_rtt_long_rtt_us` | 20000 | 0-10M | µs | 長 RTT しきい値 |
| `kcc_probe_rtt_long_interval_div` | 1 | 1-1000 | 長 RTT 間隔除数 |
| `kcc_alone_confirm_rounds` | 3 | 1-32 | ラウンド | 単一フローモードを有効にする前の確認ラウンド数 |
| `kcc_alone_agg_state_level` | 1 | 0-2 | — | 集約厳格度（0=IDLEのみ, 1=≤SUSPECTEDデフォルト, 2=≤CONFIRMED過度に攻撃的） |
| `kcc_alone_bypass_ecn` | 0 | 0-1 | — | 単独モード時の ECN バックオフスキップ（1=スキップ, 0=有効のまま） |

## データパス

```
ACK 到着（rate_sample）
    │
    ▼
kcc_main()
    │
    ├──► ACK 集約信頼度パイプライン（kcc_agg_enable 時）
    │      測定 → 評価 → 状態 → ウォッチドッグ
    │      ├── 信号層：カルマン R スケーリング（常時活性）
    │      └── 制御層：cwnd 補償（CONFIRMED+）
    │
    ├──► kcc_update_model()
    │      ├── kcc_update_bw()              スライディングウィンドウ最大 BW
    │      ├── kcc_update_ecn_ewma()        ECN-CE マーク比率
    │      ├── kcc_update_ack_aggregation()  デュアルウィンドウ extra_acked
    │      ├── kcc_update_cycle_phase()     PROBE_BW フェーズ進行
    │      ├── kcc_check_full_bw_reached()  STARTUP 終了検出
    │      ├── kcc_check_drain()            DRAIN 入退移行 + DRAIN スキップ
    │      ├── kcc_update_min_rtt()         カルマン + ウィンドウ min-RTT + PROBE_RTT
    │      └── モード固有のゲイン割り当て
    │
    ├──► kcc_apply_cwnd_constraints()
    │      └── kcc_ecn_backoff()            ECN バックオフ（cwnd_gain のみ）
    │
    ├──► kcc_set_pacing_rate()              即時、BBR ルール
    │
    └──► kcc_set_cwnd()                     BDP + 集約補償
```

## カルマンフィルタ内部フロー

```
RTT サンプル（rtt_us）
    │
    ├── 無効（≥0 かつ < dynamic_max）？いいえ → 破棄
    │
    ├── コールドスタート（sample_cnt==0）？はい → 初期化：x_est=z、p_est=max(p_init, rtt_us/div)
    │                                          （RTT 最大ゲートをバイパス）
    │
    ├── 適応的 Q：Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   適応的 R：R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── イノベーション：innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && クールダウン期限切れ？
    │   ├── Yes: p_est = p_est_init, cooldown = 15, mark qboost_fired
    │   └── No:  cooldown-- if active
    │
    ├── 予測：p_pred = p_est + Q
    │
    ├── 外れ値ゲート：|innov| > dyn_thresh && p_pred ≤ converged？
    │   ├── はい & reject_cnt < max → 棄却、++consec_reject_cnt、戻る
    │   └── はい & reject_cnt ≥ max → 強制受理（アンチロック）
    │
    └── カルマン更新：
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov（非負にクランプ）
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── ジッタ EWMA 更新
         ├── qdelay EWMA 更新
         ├── BBR-S 共分散整合ノイズ推定
         └── sample_cnt++
```

## 診断

`ss -i`（`INET_DIAG_BBRINFO`）経由の BBR 互換診断インターフェース：

```
bbr_bw_lo/bbr_bw_hi：64 ビット帯域幅推定（バイト/秒）
bbr_min_rtt：         現在の min_rtt_us
bbr_pacing_gain：     現在のペーシングゲイン（BBR_UNIT、256=1.0x）
bbr_cwnd_gain：       現在の cwnd ゲイン（BBR_UNIT）
```

## 使用方法

```sh
# カーネルモジュールのコンパイル
make

# 開発用ロード（insmod、依存関係の解決なし）
sudo make load

# インストールと正式ロード（modprobe）
sudo make install
sudo make modload

# アンロード
sudo make unload

# KCC アルゴリズムの選択
echo KCC > /proc/sys/net/ipv4/tcp_congestion_control
```

パラメータ設定は `/proc/sys/net/kcc/` を介して行います。例：
```sh
# 特定の PROBE_BW フェーズでゲイン減衰を有効化
echo 1 > /proc/sys/net/kcc/kcc_cycle_decay_mask

# ECN バックオフ感度の調整
echo 30 > /proc/sys/net/kcc/kcc_ecn_backoff_num
```

## 並行性と安全性モデル

KCC は意図的に自身のデータ構造に READ_ONCE/WRITE_ONCE や RCU を使用しません。この設計は BBR や CUBIC などの全カーネル内 CC モジュールと一貫性があります。

`kcc_init()` はプロセスコンテキスト（ソケット作成時）で実行され、その後にソケットが任意の softirq に公開されます。`kcc_release()` は、カーネルがこのソケットの ACK を処理している softirq がもう存在しないことを保証した後に実行されます。グローバルモジュールパラメータの一時的な古い値は、最大でも 1 つの ACK に影響し、次の ACK で修正されます。

唯一の例外：`sk->sk_pacing_rate` / `sk->sk_pacing_shift` はソケット層のフィールドであり、ユーザー空間が `setsockopt` を介して同時に変更できるため、BBR の WRITE_ONCE/READ_ONCE 規則が保存されます。

## パフォーマンスサマリー

テスト環境：中国 → 米国LAX、212ms RTT、8並列フロー、26%パケットロス、1 Gbps共有VPSボトルネック。

| メトリック | KCC v1.0 | BBR（制御） | 差分 |
|--------|----------|---------------|-------|
| 平均スループット | 1,010 Mbps | 937 Mbps | **+7.8%** |
| Intra-KCC不公平性 | 3.1× | 6.2×（BBR） | **−50%** |
| 最悪単一フロー | 60.6 Mbps | 30.8 Mbps | **+97%** |
| 再送信 | 150K/10s | 137K/10s | +9.5% |
| ラウンド3安定性 | 959 Mbps | 883 Mbps | **+8.6%** |

再送信はやや高めです — これは損失下で高いリンク利用率を維持するためのトレードオフと一致しています。KCCのカルマン拡張min_rtt推定により、より正確なBDPベースラインが提供され、アルゴリズムが同一経路上でBBRv1よりも高いスループットを維持できるようになります。

## Global Kalman BDP — クロスコネクション帯域幅注入

KCC v1.0 には、サーバーの定常ボトルネック帯域幅を推定するオプションのクロスコネクショングローバルカルマンフィルタが含まれています。この推定値は、新しい接続を控えめな低い「デザートスピード」で起動するために使用されます — コールドスタートの立ち上がりをスキップできるほど速く、オーバーシュートを回避できるほど遅い速度です。

### 設計原理

フィルタは、すべての KCC 接続の PROBE\_BW **クルーズフェーズ**（ゲイン = 1.0×）からの帯域幅サンプルを供給されます。クルーズフェーズのサンプルは、真の利用可能帯域幅の最もクリーンな信号です — 1.25× のプローブオーバーシュートも 0.75× のドレインアンダーシュートもありません。一次元ランダムウォークカルマンフィルタ（Kalman 1960）がグローバルな定常状態を追跡します。

新しい接続が確立されると、フィルタの推定値が以下の初期値として使用されます：

| 注入値 | 目的 |
|----------------|---------|
| `minmax`（max_bw トラッカー） | 最初の数個のダーティ ACK サンプルがゼロに引きずらないように、スライディングウィンドウ帯域幅履歴を初期化 |
| `sk_pacing_rate` | ニュートラルゲイン（BBR\_UNIT）での初期ペーシングレート；STARTUP の 2.89× ゲインは最初の ACK で適用 |
| `tp->snd_cwnd` | ニュートラルゲインで `kcc_bdp()` により計算された初期輻輳ウィンドウ |

`kcc_update_bw` の防御的下限は、STARTUP 中の最初の数 RTT の低配送レートサンプルが注入された推定値を上書きするのを防ぎます。`kcc_check_full_bw_reached` の全帯域幅ガードは、iperf3 制御メッセージ交換が STARTUP を早期に終了させるのを防ぎます。

### デザートスピード割引率

有効注入速度は以下です：

```
coeff = (discount_ratio) / high_gain
      = (num / den) / 2.89
```

ここで `high_gain ≈ 2.89` は BBR STARTUP ペーシング乗数です。

| num | coeff  | 特性 |
|-----|--------|----------------|
|  35 | 12.1%  | 最大安全性、最悪パス |
|  50 | 17.3%  | 中心軸（デフォルト） |
|  75 | 25.9%  | 数学的デザートスイートスポット |
|  80 | 27.6%  | 数学的レート上限（超えるべきではない） |

**注意：** `tcp_write_xmit` は、すべての新しい接続に対して `TCP_INIT_CWND`（10 セグメント、約 15 KB）の初期 CWND を強制します。CWND はリモート ACK が到着したときにのみ増加するため、デザートスピードはペーシングレートの上限です — 実際のスループットは、ウィンドウを開くのに十分な ACK を受信するまで CWND によって制限されます。

### 設定

`sysctl` で有効化：

```bash
sysctl -w net.kcc.kcc_kf_enable=0           # master enable (default 0)
sysctl -w net.kcc.kcc_kf_discount_num=50   # dessert-speed numerator (default 50, range 35–75)
```

**主要 sysctl パラメータ** (`/proc/sys/net/kcc/`)：

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|---------|-------|-------------|
| `kcc_kf_enable` | 0 | 0–1 | グローバルカルマン BDP 注入のマスター有効化 |
| `kcc_kf_discount_num` | 50 | 0–100 | デザートスピード分子（公平シェア BW に対する %） |
| `kcc_kf_discount_den` | 100 | 1–100000 | デザートスピード分母 |
| `kcc_kf_steady_mode` | 0 | 0/1 | — | ステディモード：有効時、KF の瞬間的な低下を無視し、単調増加ピーク（kf_x_steady）を初期帯域幅に使用 |
| `kcc_kf_startup_r_pct` | 20 | 1–100 | 起動フェーズ中の測定ノイズ R% |
| `kcc_kf_steady_r_pct` | 5 | 1–100 | 定常状態中の測定ノイズ R% |
| `kcc_kf_q_shift` | 20 | 0–30 | プロセスノイズシフト（Q = 1 << shift） |
| `kcc_kf_chi2_num` | 384 | 1–100000 | カイ二乗外れ値ゲート分子 |
| `kcc_kf_chi2_den` | 100 | 1–100000 | カイ二乗外れ値ゲート分母 |

kcc_kf_steady_mode が有効（1）の場合、新規接続の初期帯域幅は KF 推定の単調増加ピーク（kf_x_steady）を使用し、直前の接続終了によって低下した可能性のあるリアルタイム推定は無視される。これにより安定経路上でのコールドスタート飢餓を防止する。モード無効化時にピークはゼロにリセットされ、再有効化時にクリーンな状態から開始する。

### 初秒パフォーマンス（太平洋横断、212 ms RTT）

```
KFなし：  2.8 Mbps  →  85 Mbps  →  622 Mbps  →  定常
KFあり：  50 Mbps   →  530 Mbps  →  650 Mbps  →  定常
```

初秒速度は約 3 Mbps（コールドスタート）から約 50 Mbps（デザートスタート）に跳ね上がり、2～3 秒以内に定常状態への収束に達します。再送信は全体を通してゼロのままです。

### 動作の仕組み

1. 実行中の KCC 接続が PROBE\_BW クルーズフェーズに入る → ラウンド開始境界 → 現在の配送レートサンプルで `kcc_kf_update(bw, 5%)` に供給。
2. カルマンフィルタが推定値 `kcc_kf_x`（定常ボトルネック帯域幅の移動平均）を更新。
3. **新しい**接続が開かれると、`kcc_init` が `kcc_kf_get_init_bw(sk)` を呼び出し、`fair × discount / high_gain` — ゲイン補償された公平シェア初期帯域幅推定値を返す。
4. この推定値が `sk_pacing_rate`、`tp->snd_cwnd`、`minmax` 帯域幅トラッカーを初期化 — 接続はゼロではなくデザートスピードで開始。

### アルゴリズムソース

Global Kalman BDP フィルターは著者の論文『Linuxカーネル空間における大域的定常帯域幅のカルマン推定と工学的実装について』（CC BY-SA 4.0）に基づいています：
https://blog.csdn.net/liulilittle/article/details/161635652

---

*KCC v1.0 — BBRv1（Cardwell et al. 2016, ACM Queue）とカルマンフィルタ（Kalman 1960）に基づいて構築。*

## 参考文献

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