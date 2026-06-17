[🇺🇸 English](../README.md) | [🇨🇳 中文](README_CN.md) | [🇹🇼 繁體中文](README_TW.md) | [🇪🇸 Español](README_ES.md) | [🇫🇷 Français](README_FR.md) | [🇷🇺 Русский](README_RU.md) | [🇸🇦 العربية](README_AR.md) | [🇩🇪 Deutsch](README_DE.md) | [🇯🇵 日本語](README_JA.md) | [🇰🇷 한국어](README_KO.md) | [🇮🇹 Italiano](README_IT.md) | [🇵🇹 Português](README_PT.md)

---

# TCP KCC v1.0 (Kalman-Überlastkontrolle)

TCP-Überlastungssteuerungsmodul für Shared-Bandwidth-VPS-Umgebungen, das die BBRv1-Zustandsmaschine mit einem Kalman-Filter zur Schätzung der Ausbreitungsverzögerung kombiniert.

## Design-Prinzipien

Überlastungssteuerungsalgorithmen müssen Durchsatz, Latenz, Fairness und Verlusttoleranz ausbalancieren. KCC verfolgt einen pragmatischen Ansatz:

1. BBRv1 bietet eine bewährte Grundlage. Zustandsmaschine, Pacing, Zyklusverstärkungen, STARTUP/DRAIN/PROBE_BW/PROBE_RTT — KCC übernimmt diese Mechanismen ohne Änderung.

2. Der Kalman-Filter verbessert die Schätzgenauigkeit. Die Trennung der wahren Ausbreitungsverzögerung von Warteschlangenverzögerung und Jitter ergibt eine genauere min_rtt-Schätzung, was eine präzisere BDP-Berechnung, besser kalibriertes CWND und stabileres Pacing ermöglicht.

3. Die Inter-Algorithmus-Dynamik folgt dem standardmäßigen TCP-Wettbewerbsgleichgewicht. KCC begrenzt seine Senderate nicht künstlich als Reaktion auf von externen Flüssen erkannte Warteschlangen. Gain Decay (warteschlangenbasierte Sondenreduzierung) ist optional über kcc_cycle_decay_mask verfügbar, aber standardmäßig deaktiviert, um die volle Sondenintensität zu erhalten.

4. Intra-KCC-Fairness wird aktiv aufrechterhalten. Die Kalman-Konvergenz stellt sicher, dass KCC-Flüsse auf demselben Host eine konsistente min_rtt-Schätzung teilen, wodurch die Winner-takes-all-Rückkopplungsschleife eliminiert wird, die in reinen BBR-Multi-Flow-Bereitstellungen zu schwerwiegender Unfairness führt.

## Algorithmenübersicht

TCP KCC implementiert ein senderseitiges Überlastungssteuerungsmodul für den Linux-Kernel als ladbares `tcp_kcc.ko`. Die Überlastungssteuerungsfunktion `kcc_main()` wird bei jedem ACK von `tcp_ack()` aufgerufen und erhält eine `rate_sample`-Struktur, die Kernel-Bandbreiten- und RTT-Messwerte sowie Liefer- und Verlustzähler enthält. Der Algorithmus arbeitet in zwei zeitlichen Regimen: einem **pro-ACK-Schnellpfad**, der den Messzustand aktualisiert und sofortige Pacing- und Fensterziele berechnet, und einem **pro-Runde-Langsampfad**, der Zustandsübergangsbedingungen auswertet und Verstärkungen neu berechnet.

Die zentrale Messpipeline besteht aus zwei Komponenten:

1. **Maximalbandbreitenfilter mit gleitendem Fenster** (`minmax_running_max` aus `linux/win_minmax.h`): Fenster über die letzten `kcc_bw_rt_cycle_len` (Standard 10) Umläufe. Liefert die BBR-kompatible `max_bw`-Schätzung.

2. **Kalman-Filter-Ausbreitungsverzögerungsschätzer**: ersetzt BBRv1's gleitendes Fenster-Minimum-RTT und ist die Standardquelle für die BDP-RTT-Schätzung (siehe [Modell-RTT-Strategie](#modell-rtt-strategie)). Ein Einzustands-Kalman-Filter (Kalman 1960), der in `kcc_kalman_scale` × µs Festkomma-Einheiten arbeitet und die wahre Ausbreitungsverzögerung als Zufallsbewegung modelliert:
   - Zustand: `x[k] = x[k−1] + w[k]`, `w ~ N(0, Q)`
   - Beobachtung: `z[k] = x[k] + v[k]`, `v ~ N(0, R)`

Festkomma-Konventionen: `BW_UNIT = 1 << 24` für Bandbreite (Segmente * 2^24 / µs), `BBR_UNIT = 1 << 8 = 256` als dimensionslose Verstärkungseinheit.

## Modell-RTT-Strategie

KCC führt eine konfigurierbare Strategie für die RTT-Schätzung ein, die bei der BDP-Berechnung (Bandbreite-Verzögerungs-Produkt) verwendet wird, gesteuert durch `kcc_rtt_mode`:

| Modus | Wert | Verhalten | Anwendungsfall |
|-------|------|-----------|---------------|
| FILTER | 1 (Standard) | Direkte Verwendung von `x_est_us` — der rohen Kalman/Gleitfenster-Schätzung | Produktions-WAN/VPS: robust gegenüber Routenänderungen, kein Null-Durchsatz-Abruch |
| MIN | 0 | `min(x_est_us, min_rtt_us)` — Kalman-Schätzung gegen Fensterminimum klammern | Kernel-Modul-Stabilitätsverifikation; statische-RTT-Links |

**Warum FILTER der Standard ist:**

- **Resilienz bei Routenänderungen**: Wenn eine BGP-Umleitung die physische RTT erhöht (z. B. 50 ms → 100 ms), reagiert die Kalman-Verstärkung K_k innerhalb weniger RTTs und zieht die Schätzung auf die neue Pfadlatenz. Der MIN-Modus blockiert auf der alten `min_rtt_us`, bis das Fenster abläuft, und halbiert das BDP.

- **Integrierte Abwehrmechanismen**: Die Ausreißersperre verwirft Warteschlangen-Spitzenwerte, bevor sie in den Filter gelangen. Die adaptive Q/R-Rauschschätzung reduziert die Kalman-Verstärkung bei verrauschtem Netzwerk, sodass der Filter natürlicherweise vorübergehendem Queue-Bloat misstraut und die Schätzung nahe der wahren Ausbreitungsverzögerung hält.

- **PROBE_RTT-Entkopplung**: Der FILTER-Modus aktiviert die Funktion `kcc_probe_rtt_decouple` — der Kalman-Filter verfolgt das RTT-Minimum ohne die periodische 4-Paket-Entleerung.

Laufzeitumschaltung: `echo 0 > /proc/sys/net/kcc/kcc_rtt_mode` zur Rückkehr zum MIN-Modus.

## Zustandsmaschine

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

Vier Modi, kodiert als 2-Bit-`mode`-Feld in `struct KCC`:

- **STARTUP (0)**: Anfangszustand. `pacing_gain` ≈ 2,885x (`kcc_high_gain_val`), `cwnd_gain` ebenfalls 2,885x. Exponentielle Bandbreitenerkundung.
- **DRAIN (1)**: Wird nach STARTUP-Austritt betreten. `pacing_gain` ≈ 0,347x (`kcc_drain_gain_val`), `cwnd_gain` bleibt bei 2,885x. Entleert die während STARTUP aufgebaute Warteschlange.
- **PROBE_BW (2)**: Stationärer Zustand. Durchläuft eine 256-Slot-Verstärkungstabelle (Standard 8-Phasen-Muster wiederholt: 1,25x/0,75x/8×1,0x).
- **PROBE_RTT (3)**: Entleert periodisch den Inflight-Verkehr auf `kcc_cwnd_min_target` (Standard 4 Segmente), um eine frische RTT-Messung zu erhalten.

### STARTUP → DRAIN

Ausgelöst, wenn `full_bw_reached` gesetzt ist — nach `kcc_full_bw_cnt` (Standard 3) aufeinanderfolgenden Runden, in denen `max_bw` nicht um mindestens `kcc_full_bw_thresh_val` (Standard 1,25x) gegenüber dem zuvor beobachteten Spitzenwert wächst. Das BDP bei 1,0x Verstärkung wird in `snd_ssthresh` geschrieben. `qdelay_avg` wird auf Null zurückgesetzt, um zu verhindern, dass der STARTUP-Warteschlangenaufbau PROBE_BW beeinflusst.

### DRAIN → PROBE_BW

Ausgelöst, wenn der geschätzte Inflight-Verkehr bei EDT ≤ Ziel-Inflight bei 1,0x BDP-Verstärkung. **Drain-Überspring-Optimierung**: wenn der Kalman-Filter konvergiert ist UND `qdelay_avg < der dynamische Sauber-Schwellwert` (10% von min_rtt_us mit 500us Untergrenze), wird die DRAIN-Phase übersprungen — frühzeitige Umwandlung zu PROBE_BW.

Beim Eintritt in PROBE_BW wird der Zyklenphasenindex randomisiert: `cycle_idx = len − 1 − rand(kcc_probe_bw_cycle_rand)` (Standard `len − 1 − rand(8)`), was parallele Ströme, die sich einen Engpasslink teilen, dekorreliert.

### PROBE_BW → PROBE_RTT

Ausgelöst, wenn das PROBE_RTT-Filterintervall abläuft — der Zeitstempel `min_rtt_stamp` wurde innerhalb des berechneten Intervalls nicht aktualisiert. cwnd wird in `prior_cwnd` gespeichert, Pacing wird auf Entleeren gesetzt.

### PROBE_RTT → PROBE_BW

Nachdem der Inflight-Verkehr auf `kcc_cwnd_min_target` fällt oder eine Rundengrenze beobachtet wird, besteht für mindestens `kcc_probe_rtt_mode_ms_val` (Standard 200 ms) und mindestens eine beobachtete vollständige Runde, dann Austritt. cwnd wird auf mindestens `prior_cwnd` wiederhergestellt, Pacing wird vorübergehend mit `kcc_high_gain_val` für schnelles Rohrfüllen überschrieben.

### Wiederherstellung und Verlust

- Bei TCP_CA_Loss: `full_bw` und `full_bw_cnt` werden zurückgesetzt, `round_start` auf 1 gesetzt, `packet_conservation` auf 0 gelöscht.
- Wiederherstellungseintritt (TCP_CA_Recovery): `packet_conservation` aktiviert, cwnd = Inflight + bestätigt.
- Wiederherstellungsaustritt: auf `prior_cwnd` zurückgesetzt, `packet_conservation` gelöscht.
- `kcc_undo_cwnd()`: setzt `full_bw` und `full_bw_cnt` zurück (unter Beibehaltung von `full_bw_reached`), löscht den LT-BW-Zustand.

### Rundenerkennung (BBR-Angleichung)

Rundengrenzen werden gemäß BBR (Cardwell et al. 2016) erkannt: wenn `prior_delivered` den Wert von `next_rtt_delivered` durch einen unsigned `!before()`-Vergleich erreicht oder überschreitet. `next_rtt_delivered` wird auf `0` initialisiert — wie bei Stock-BBR — sodass der erste ACK sofort Runde 1 startet, unabhängig von der Zustellung von Handshake-Segmenten. Die Rate-Sample-Validierung verwendet `interval_us <= 0` (nicht `== 0`), um BBRs exakte Schutzbedingung zu treffen und negative Intervalle abzufangen.

- `next_rtt_delivered` auf `0` initialisiert (BBR-Parität): erste Runde beginnt mit dem ersten ACK.
- `interval_us <= 0`-Validierung: entspricht exakt BBR, verwirft negative Intervalle.
- `round_start` wird am Anfang von `kcc_update_bw()` auf `0` zurückgesetzt, vor der Validierungsprüfung — entsprechend BBRs `bbr->round_start = 0`-Platzierung.

## Kernmessungen

### Bandbreitenschätzung

Maximalbandbreitenfilter mit gleitendem Fenster (`minmax_running_max` aus `linux/win_minmax.h`) über `kcc_bw_rt_cycle_len` (Standard 10) Runden. Momentane Bandbreite = `delivered × BW_UNIT / interval_us`, pro ACK berechnet. Wird nur dann in das gleitende Fenster eingespeist, wenn die Anwendung nicht begrenzt ist oder wenn die Bandbreite ≥ aktuelle Maximalbandbreite ist (BBR-Regel).

Wenn `lt_use_bw` aktiv ist, wechselt die aktive Bandbreitenschätzung zu `lt_bw` (Langzeit-Bandbreitenschätzung).

### Kalman-Filter

Skalare Einzustands-Kalman-Rekursion (O(1)-Komplexität):

```
Vorhersage:
  x_pred = x_est          (Identitätszustandsübergang)
  p_pred = p_est + Q      (Kovarianzvorhersage)

Aktualisierung:
  innov   = z − x_pred    (Innovation)
  K       = p_pred / (p_pred + R)   (Kalman-Verstärkung [0,1])
  x_est   = x_pred + K × innov      (Zustandsaktualisierung)
  p_est   = (1 − K) × p_pred        (posteriore Kovarianz)
```

**Adaptives Prozessrauschen Q**:
```
Q_base   = kcc_kalman_q (Standard 100)
q_factor = max(kcc_kalman_q_min_factor_val, min_rtt_us / kcc_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × kcc_kalman_q_scale_cap)
Q        = min(Q, kcc_kalman_q_max)
```

**Adaptives Messrauschen R**:
```
R = R_base + max(0, jitter_ewma − clean_thresh) × R_base / kcc_jitter_r_scale
R = min(R, R_base × kcc_kalman_r_max_boost)
```

**Q-Boost-Pfadänderungserkennung (konfidenzgesteuert + Abkühlung)**: wenn `|innovation| > kcc_kalman_q_boost_thresh_val` (Standard ≈ 4 ms RTT-Verschiebung) UND der Filter konvergiert ist (`p_est ≤ kcc_kalman_converged_p_est_val`, Standard 500), wird `p_est` auf `kcc_kalman_p_est_init_val` zurückgesetzt, wodurch die Kalman-Verstärkung für schnelle Konvergenz in Richtung 1,0 erhöht wird. Eine Abkühlung von `kcc_kalman_qboost_cdwn` (Standard 15) Abtastwerten zwischen aufeinanderfolgenden Q-Boost-Ereignissen verhindert unkontrolliertes Auslösen auf verlustbehafteten Pfaden mit hohem RTT-Jitter.

**Ausreißer-Sperre**: dynamischer Schwellwert `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`. Wird nur angewendet, wenn `p_pred ≤ kcc_kalman_converged_p_est_val`. Nach `kcc_kalman_max_consec_reject` (Standard 25) aufeinanderfolgenden Ablehnungen wird die nächste Messung zwangsweise akzeptiert, um eine sich selbst verstärkende Blockade zu verhindern.

**Kovarianz-angepasste Rauschschätzung (BBR-S)**: `q_est = (1−α) × q_est + α × (K × innov)²`, `r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`. Kombinationsmodus: Modus 0 = nur heuristisch, Modus 1 = max (Standard), Modus 2 = gewichtete Mischung.

**Kalman-Übernahme**: wenn `x_est > 0` und `sample_cnt ≥ kcc_kalman_min_samples` (Standard 5), wird `min_rtt_us` durch `x_est / kcc_kalman_scale` ersetzt. `min_rtt_stamp` wird nicht aktualisiert — der PROBE_RTT-Intervallauslöser bleibt unabhängig.

**Modell-RTT-Strategie**: Die für die BDP-Berechnung verwendete RTT-Schätzung wird durch `kcc_rtt_mode` gesteuert. Im FILTER-Modus (Standard) wird `model_rtt = x_est_us` direkt verwendet — die Kalman/Gleitfenster-Schätzung ohne Begrenzung. Im MIN-Modus wird `model_rtt = min(x_est_us, min_rtt_us)` verwendet — die Kalman-Schätzung wird gegen das Fensterminimum begrenzt, um sicherzustellen, dass das BDP niemals aufgebläht wird. Der FILTER-Standard wird für Produktions-WAN/VPS-Bereitstellungen empfohlen, bei denen sich die Pfadlatenz abrupt ändern kann (BGP-Umleitungen, LEO-Übergaben, Mobilfunkzellenwechsel). Siehe [Modell-RTT-Strategie](#modell-rtt-strategie).

## BBR-Erweiterungen

### Verstärkungsabfall

Aktiviert durch die 256-Bit-Bitmap `kcc_cycle_decay_mask[]` für bestimmte PROBE_BW-Phasen. Abfallformel (bei akzeptierter Kalman-Messung):

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = inverse Skalierung von p_est (BBR_UNIT bei voll)
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                     × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                     × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

Kalman-Konfidenzskalierung: wenn `p_est > kcc_kalman_converged_p_est`, wird der Abfall proportional reduziert, was übermäßigen Rückgang bei unsicherem Filter vermeidet.

### ECN-Rücknahme

Aktivierungsbedingungen (alle müssen erfüllt sein):
1. `kcc_ecn_enable_val != 0`
2. Kalman konvergiert (`p_est < converged`, `sample_cnt >= min_samples`)
3. `ecn_ewma > 0` (CE-Markierungen beobachtet)
4. `qdelay_avg > der dynamische Überlastungs-Schwellwert` (25% von min_rtt_us mit 500us Untergrenze)
5. Modus ist NICHT PROBE_BW (cwnd_gain ist in PROBE_BW fest auf 2x)

Während der Erkundungsphasen (`pacing_gain > BBR_UNIT`) wird die ECN-Rücknahme durch `BBR_UNIT² / pacing_gain` abgestuft — ~80% Rücknahme bei 1,25x-Sonde, ~65% bei 2,89x-STARTUP-Verstärkung.

ECN-Markierungsverhältnis EWMA: wird an Rundengrenzen durch `kcc_ecn_ewma_retained / kcc_ecn_ewma_total` (Standard 3/4) aktualisiert, mit sanftem pro-ACK-Abfall von `kcc_ecn_idle_decay_num / kcc_ecn_idle_decay_den` (Standard 31/32) bei jedem ACK ohne neue CE-Markierungen.

### Einzelfluss-Erkennung

Wenn KCC erkennt, dass der Fluss wahrscheinlich allein am Engpass ist (niedrige Warteschlangenverzögerung, niedriger Jitter, keine ECN-Markierungen, keine ACK-Aggregation), wechselt es automatisch in einen reinen BBR-Modus:

- `kcc_get_model_rtt()` gibt direkt `min_rtt_us` zurück (vermeidet die geglättete Kalman-Schätzung, die aufgrund des einseitigen Messrauschens eine kleine positive Verzerrung aufweist).
- `kcc_ecn_backoff()` ist über `kcc_alone_bypass_ecn` (Standard 1) konfigurierbar — auf einem Einzelfluss-Pfad sind ECN-Markierungen Fehlalarme des AQM, da kein anderer Sender konkurriert. Das Überspringen entspricht dem Null-ECN-Verhalten von BBR. Auf 0 setzen, um ECN-Backoff auch im Alleinmodus beizubehalten (konservativ).

Dies beseitigt die Leistungslücke bei Einzelflüssen zwischen KCC und BBR, während die vollständige Schutzschleife von KCC (Kalman, ECN-Rücknahme, Verstärkungsabfall) für Multi-Fluss-Szenarien erhalten bleibt.

**Hysterese**: Der Eintritt erfordert `kcc_alone_confirm_rounds` (Standard 3) aufeinanderfolgende qualifizierte Runden — vermeidet Oszillationen während kurzer Ruhephasen im Multi-Fluss-Wettbewerb ("konservativ beim Beschleunigen"). Austritt: Während der Cruise-Phasen-Bewertung löscht jeder einzelne Qualifikationsfehler das Flag ("aggressiv beim Abbremsen").

**Design-Entscheidung**: Paketverlust wird NICHT als Einzelfluss-Disqualifikator verwendet — einige Verbindungen (flache Puffer, drahtlos, Virtualisierungs-Burst-Drops) haben inhärente Verluste, die nichts mit Wettbewerb zu tun haben. Verluste mit Multi-Fluss-Wettbewerb gleichzusetzen, verursacht Oszillationen auf Einzelfluss-Pfaden. Das LT BW-Signal (BBRs verlustausgelöste Policer-Erkennung) nimmt nicht an der Einzelfluss-Beurteilung teil.

**Gain-Gating**: Die Einzelfluss-Bewertung läuft nur während der Cruise-Phase (`pacing_gain == BBR_UNIT`). Probe-Up (1,25x) drückt absichtlich gegen den Engpass — sein Warteschlangendruck ist selbstinduziert und kein Wettbewerbssignal. Drain (0,75x) unterdrückt die Warteschlange künstlich. Indem die Bewertung auf Cruise (das stationäre Gleichgewicht) beschränkt wird, verursacht der selbstinduzierte Probe-Up-Druck keine falschen Alone-Mode-Austritte mehr.

Qualifikationsbedingungen (alle fünf müssen an einer Rundengrenze erfüllt sein):
0. Kalman konvergiert (`sample_cnt >= kcc_kalman_min_samples`) — qdelay/jitter als Warteschlangensignale vertrauen
1. `qdelay_avg < der dynamische Sauber-Schwellwert` — Warteschlange fast leer
2. `jitter_ewma < der dynamische Überlastungs-Schwellwert` — nur ACK-Takt-Mikrojitter
3. `ecn_ewma == 0` — keine Überlastungsmarkierungen von AQM
4. `agg_state <= max` gemäß `kcc_alone_agg_state_level` (Standard 1) — drei konfigurierbare ACK-Aggregationsstufen: 0 = nur IDLE (strengste, keine Aggregation), 1 = ≤ SUSPECTED (Standard, erlaubt vorübergehende Aggregation), 2 = ≤ CONFIRMED (permessivste, blockiert nur persistente Aggregation)

### Dynamisches PROBE_RTT-Intervall

Bildet Kalman `p_est` auf ein verbindungsspezifisches PROBE_RTT-Intervall ab:

```
p_est ≤ converged:              interval = dyn_max (Standard 30s)
p_est ≥ high (= mult × conv):   interval = base (Standard 10s)
converged < p_est < high:       lineare Interpolation
```

Reduziert die PROBE_RTT-Häufigkeit bei hoher Konfidenz (niedrigem `p_est`), was den Durchsatz-Jitter auf stabilen Pfaden verringert. Kehrt zum klassischen 10-Sekunden-Intervall zurück, wenn die Konfidenz niedrig ist.

**Per-Flow-Eintrags-Jitter**: Um zu verhindern, dass alle koexistierenden Flüsse gleichzeitig in PROBE_RTT eintreten (Entleeren auf 4 Pakete aggregiert ~1.8 Mbps, dann Nachfüllen mit 2.89×), fügt jeder Fluss einen hash-abgeleiteten Jitter (0–845 ms Streuung) zu seinem PROBE_RTT-Intervall hinzu. Zu jedem Zeitpunkt ist maximal ~1 Fluss in PROBE_RTT, wodurch der RTO-induzierende gleichzeitige Entleerungs-/Nachfüllkollaps beseitigt wird.

### PROBE_RTT-Entkopplung & intelligente Neukalibrierung

BBRv1s PROBE_RTT-Mechanismus entleert das Rohr alle ~10 Sekunden auf 4 Pakete, um `min_rtt_us` zu messen. Dies ist für einen fensterbasierten Min-RTT-Schätzer notwendig — das Fenster kann Ausbreitungsverzögerung nicht von Warteschlangenverzögerung unterscheiden, es sei denn, das Rohr ist leer. Der Preis ist ein periodischer Durchsatzabfall (die BBR-"Sägezahn").

Im FILTER-Modus ersetzt der Kalman-Filter das Fenster vollständig. Er kann Warteschlangenrauschen von der wahren Ausbreitungsverzögerung durch Ausreißersperre und adaptive Rauschschätzung trennen — keine Rohrentleerung erforderlich. Der Parameter `kcc_probe_rtt_decouple` (Standard 1) steuert dies:

| Modus | Wert | Verhalten |
|-------|------|-----------|
| Entkoppelt | 1 (Standard) | **Kalman gesund** (p_est ≤ `kcc_recal_p_est_thresh`): PROBE_RTT vollständig unterdrücken → keine Durchsatzabbrüche, keine Sync-Kollapsen. **Kalman divergiert** (p_est > Schwellwert): traditionelle PROBE_RTT als Sicherheitsnetz auslösen → stellt die Filter-Baseline wieder her, dann wird die Entkopplung fortgesetzt. |
| Traditionell | 0 | Blinde periodische PROBE_RTT alle ~10s (BBR-kompatibel). |

**Intelligente Neukalibrierungsheuristik** (`kcc_kalman_needs_recalibration()`): Im stationären Betrieb auf einem stabilen Pfad konvergiert die Kalman-Fehlerkovarianz p_est zu p_est_floor (~4–10), weit unter dem Schwellwert `kcc_recal_p_est_thresh` (250.000 = 25 % von p_est_max). Ein steigendes p_est signalisiert, dass das interne Modell des Filters die Beobachtungen nicht mehr erklärt — typischerweise, weil sich der Pfad wesentlich geändert hat. Wenn p_est den Schwellwert überschreitet, stellt eine einzige traditionelle PROBE_RTT-Entleerung die Filter-Baseline wieder her; der Kalman konvergiert erneut und die Entkopplung wird automatisch fortgesetzt.

Dies verwandelt PROBE_RTT von einer **blinden periodischen Selbstverstümmelung** in eine **intelligente vertrauensbasierte Neukalibrierung** — das Protokoll entleert das Rohr nur, wenn es empirische Beweise dafür gibt, dass der Filter das Vertrauen verloren hat.

Erfordert `kcc_rtt_mode == 1`. Im MIN-Modus wirkungslos (der MIN-Modus ist auf PROBE_RTT angewiesen, um `min_rtt_us` zu aktualisieren).

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `kcc_probe_rtt_decouple` | 1 | 0–1 | PROBE_RTT-Entkopplung aktivieren (nur FILTER-Modus) |
| `kcc_recal_p_est_thresh` | 25000 | 1–100.000.000 | p_est-Schwellenwert für Sicherheitsnetz-Neukalibrierung |

### LT-Bandbreitenschätzung

Verlustgetriggerter Untergrenzenschätzer. Das Abtastintervall umfasst [4, 16] RTTs. Gültig, wenn das Verlustverhältnis ≥ 5,9% (`kcc_lt_loss_thresh` Standard 15/256). Bandbreite `bw = delivered × BW_UNIT / interval_us`.

Im Gegensatz zu BBRs einfachem Durchschnitt (`(bw + lt_bw) >> 1`) verwendet KCC einen konfigurierbaren EMA (`kcc_lt_bw_ema_num / kcc_lt_bw_ema_den`, Standard 1/2 = 0,5):

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

Die Aktivierung unterscheidet sich von BBR: KCC speichert `lt_bw` beim ersten gültigen Intervall, setzt aber NICHT `lt_use_bw`; Konsistenz mit einem vorherigen Intervall ist erforderlich — reduziert Fehlaktivierung durch Messrauschen.

**Doppelschwellen-Überlastungstor**: Bevor `lt_use_bw = 1` gesetzt wird, werden sowohl eine persistente EWMA-Warteschlangenprüfung (`qdelay_avg > der dynamische Überlastungs-Schwellwert`) ALS AUCH eine sofortige SRTT-basierte Warteschlangenprüfung (`srtt_us − min_rtt_us > der sofortige Überlastungs-Schwellwert`, Standard 5000 µs) ausgewertet. Wenn eine Überlastung erkannt wird, wird die LT-BW-Abtastung abgebrochen. Die SRTT-Prüfung funktioniert ohne `ext`-Zuweisung und bietet ein Sicherheitsnetz gegen Zuweisungsfehler.




### ACK-Aggregations-Konfidenzbasierte Kompensation (BBRplus-inspiriert)

Fügt eine konfidenzgesteuerte zweite Schicht über dem traditionellen Dual-Slot-Extra-Acked-Schätzer hinzu.

**Vier orthogonale Faktoren** (jeder trägt `kcc_agg_factor_weight` Punkte bei, Standard 256):
1. Kalman konvergiert (`p_est < converged` + `sample_cnt >= min_samples`)
2. Nicht in Verlustwiederherstellung (`icsk_ca_state < TCP_CA_Recovery`)
3. RTT innerhalb von `min_rtt_us + der dynamische Sauber-Schwellwert` der wahren Ausbreitungsverzögerung
4. `extra_acked` innerhalb von `kcc_agg_factor4_ratio_num/den` (Standard 1,5x) des fensterbasierten Maximums

**Vier Zustände**: IDLE (< `kcc_agg_thresh_suspected`=256), VERDÄCHTIG (≥256), BESTÄTIGT (≥512), VERTRAUENSWÜRDIG (≥768).

**Signalschicht** (immer aktiv): Konfidenz interpoliert linear den R-Skalierungsfaktor `[r_min, r_max]`. R steigt sofort an (schnelle Reaktion), fällt mit `kcc_agg_r_hysteresis`% (Standard 75% beibehalten, ~4 RTTs zur Basislinie) pro RTT.

**Kontrollschicht** (`agg_state ≥ CONFIRMED`): fünffach sicherheitsgesteuerte cwnd-Kompensation:
1. Blockiert, wenn die Warteschlangenverzögerung > `der dynamische Überlastungs-Schwellwert`
2. Blockiert während der Verlustwiederherstellung
3. Blockiert, wenn cwnd > `BDP × kcc_agg_safety_bdp_mult` (Standard 3x)
4. Blockiert, wenn Inflight > sicheres cwnd + TSO-Segmentziel
5. Watchdog: stuft BESTÄTIGT→VERDÄCHTIG nach `kcc_agg_max_comp_duration` (Standard 8) aufeinanderfolgenden RTTs herab

### qdelay_avg-Zurücksetzung in DRAIN

Beim Übergang zu DRAIN wird `qdelay_avg` auf Null zurückgesetzt, wodurch verhindert wird, dass die STARTUP-Warteschlangenschätzung in PROBE_BW fortbesteht.

### TSO-Divisor-Anpassung

`kcc_min_tso_segs()` passt den Ratenschwellwertdivisor basierend auf dem Kalman-Zustand an:
- Kalman konvergiert + `jitter_ewma < 1000 µs`: Divisor halbiert (8→4), größere TSO-Bursts
- `jitter_ewma > 4000 µs`: Divisor verdoppelt (8→16), kleinere TSO-Bursts zur Unterdrückung von Jitter

## Pacing-Rate und Cwnd

### Pacing-Rate

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // Verstärkungsanpassung
rate = rate × USEC_PER_SEC >> BW_SCALE            // Umrechnung in bytes/s
rate = rate × margin_div / 100                    // Pacing-Marge (Standard 1%, matching BBR)
```

Ratenänderungen werden sofort angewendet (keine Glättung), entsprechend BBR (Cardwell et al. 2016). Nach `full_bw_reached`: alle Ratenänderungen werden sofort geschrieben. In STARTUP/DRAIN: nur Erhöhungen werden angewendet (`rate > sk_pacing_rate`).

### Cwnd

```
target = BDP(bw, gain, ext)                       // Basis-BDP
target = quantization_budget(target)              // TSO-Headroom + gerade Runde + Phase-0-Bonus
target += ack_agg_bonus + agg_compensation        // ACK-Aggregationskompensation

// cwnd-Fortschritt
if full_bw_reached:
    cwnd = min(cwnd + acked, target)              // zum Ziel konvergieren
else (STARTUP):
    cwnd = cwnd + acked                          // exponentielles Wachstum

cwnd = max(cwnd, cwnd_min_target)                 // absolute Untergrenze 4
PROBE_RTT-Modus: cwnd = min(cwnd, cwnd_min_target) // minimaler Inflight
```

## Modulparameter

Parameter werden unter `/proc/sys/net/kcc/` bereitgestellt. Schreibvorgänge lösen `kcc_init_module_params()` aus (Validierung + Begrenzung + Berechnung abgeleiteter Werte). Array-Parameter-Schreibvorgänge lösen `kcc_rebuild_gain_table()` aus.

### PROBE_RTT-Intervalle

| Parameter | Standard | Min | Max | Einheit | Beschreibung |
|-----------|----------|-----|-----|---------|-------------|
| `kcc_probe_rtt_base_sec` | 10 | 1 | 86400 | s | Basis-PROBE_RTT-Intervall |
| `kcc_probe_rtt_max_sec` | 15 | 1 | 86400 | s | Obergrenze für Lang-RTT-Pfade |
| `kcc_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | s | Max. dynamisches Intervall; 0 deaktiviert |

### Verstärkungen

| Parameter | Standard | Min | Max | Beschreibung |
|-----------|----------|-----|-----|-------------|
| `kcc_cwnd_gain_num` / `kcc_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | Basis-cwnd-Verstärkung für PROBE_BW |
| `kcc_extra_acked_gain_num` / `kcc_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | ACK-Aggregations-Bonusmultiplikator |
| `kcc_high_gain_num` / `kcc_high_gain_den` | 2885 / 1000 | 0/1 | 100k | STARTUP-Verstärkung (≈2,885x) |
| `kcc_drain_gain_num` / `kcc_drain_gain_den` | 347 / 1000 | 0/1 | 100k | DRAIN-Verstärkung (≈0,347x) |
| `kcc_gain_num[i]` / `kcc_gain_den[i]` | BBRv1-Muster (256 Slots) | 0/1 | — | Pro-Slot-Pacing-Verstärkung |
| `kcc_cycle_decay_mask[8]` | 0 (alle Null) | 0 | 0x7FFFFFFF | 256-Bit-Abfall-Bitmap |
| `kcc_probe_bw_up_limit` | 0 | 0 | 1 | Begrenzte Probe-Up-Beendigung (0=aus) |

### Kalman-Basis

| Parameter | Standard | Min | Max | Beschreibung |
|-----------|----------|-----|-----|-------------|
| `kcc_kalman_q` | 100 | 0 | 100k | Basis-Prozessrauschen Q |
| `kcc_kalman_r` | 400 | 0 | 100k | Basis-Messrauschen R |
| `kcc_kalman_p_est_max` | 1.000.000 | 1 | 100M | p_est absolutes Maximum |
| `kcc_kalman_converged_p_est` | 500 | 1 | 1M | Konvergenzschwellwert |
| `kcc_kalman_p_est_init` | 1000 | 1 | 10M | Anfängliches p_est |
| `kcc_kalman_p_est_floor` | 10 | 1 | 100k | p_est-Untergrenze |
| `kcc_kalman_scale` | 1024 | 64 | 1.048.576 | Festkomma-Skalierung (Zweierpotenz) |
| `kcc_kalman_min_samples` | 5 | 3 | 20 | Mindestmessungen vor Übernahme |
| `kcc_kalman_outlier_ms` | 5 | 0 | 10000 | ms | Basis-Ausreißerschwellwert |
| `kcc_kalman_q_boost_mult` | 4 | 1 | 10000 | Q-Boost-Multiplikator |
| `kcc_kalman_q_boost_ms` | 1 | 0 | 5000 | ms | Q-Boost-Zeitkonstante |
| `kcc_kalman_qboost_cdwn` | 8 | 1 | 255 | samples | Q-Boost-Abklingzeit |
| `kcc_kalman_q_max` | 2000 | 1 | 100k | Q-Obergrenze |
| `kcc_kalman_q_scale_cap` | 50 | 1 | 10000 | Q-Skalierungsbegrenzung |
| `kcc_kalman_max_consec_reject` | 25 | 1 | 1000 | Max. aufeinanderfolgende Ablehnungen vor Zwangsannahme |
| `kcc_rtt_sample_max_us` | 500000 | 1 | 10M | µs | Kalman-RTT-Obergrenze |
| `kcc_kalman_r_max_boost` | 8 | 1 | 1000 | R-Max-Boost-Multiplikator |
| `kcc_kalman_rtt_dyn_mult` | 2 | 1 | 100 | RTT-dynamischer-Obergrenzen-Multiplikator |
| `kcc_kalman_q_rtt_div` | 1000 | 1 | 1M | Q-Anpassungs-RTT-Divisor |
| `kcc_kalman_probe_band_mult` | 4 | 1 | 32 | PROBE_RTT-Übergangsband-Multiplikator |

### Kalman-Zusätze (num/den-Typ)

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `kcc_kalman_outlier_jitter_mult_num/den` | 3 / 1 | 0-1000 / 1-100k | Ausreißer-Jitter-Multiplikator |
| `kcc_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | Q-Minimalfaktor |
| `kcc_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | p_est-Init-RTT-Divisor |

### BBR-S-Rauschschätzung

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `kcc_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | Q-Schätzungs-Lernrate |
| `kcc_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | R-Schätzungs-Lernrate |
| `kcc_kalman_noise_mode` | 1 | 0-2 | Kombinationsmodus (0=aus, 1=max, 2=gewichteter Durchschnitt) |
| `kcc_kalman_q_est_max` | 50000 | 1-2 Mrd. | Q-Schätzungs-Obergrenze |
| `kcc_kalman_r_est_max` | 32000 | 1-2 Mrd. | R-Schätzungs-Obergrenze |
| `kcc_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | Untergrenze pro Schätzung |

### Verstärkungsabfall (Erkundung)

| Parameter | Standard | Bereich | Einheit | Beschreibung |
|-----------|----------|---------|---------|-------------|
| `kcc_qdelay_probe_scale_us` | 20000 | 1-100k | µs | qdelay-Abfallskalierung |
| `kcc_jitter_probe_scale_us` | 16000 | 1-100k | µs | Jitter-Abfallskalierung |

### Dynamische Warteschlangenverzögerungs-Schwellenwerte

| Parameter | Standard | Bereich | Einheit | Beschreibung |
|-----------|----------|---------|---------|-------------|
| `kcc_qdelay_clean_bp` | 1000 | 1-10000 | ‱ | Sauberer Schwellwert (10 % von min_rtt_us) |
| `kcc_qdelay_cong_bp` | 2500 | 1-10000 | ‱ | Überlastungsschwellwert (25 % von min_rtt_us) |
| `kcc_qdelay_floor_us` | 500 | 1-100k | µs | Absoluter Mindestwert für RTT-Prozentsatz-Überschreibung |

### Adaptives R (Jitter-gesteuert)

| Parameter | Standard | Bereich | Einheit | Beschreibung |
|-----------|----------|---------|---------|-------------|
| `kcc_jitter_r_scale` | 8000 | 1-100k | — | R-Erhöhungs-Skalierungsdivisor |

### ECN

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `kcc_ecn_enable` | 1 | 0-1 | ECN-Hauptschalter |
| `kcc_ecn_backoff_num` / `kcc_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | ECN-Rücknahme-Anteil |
| `kcc_ecn_ewma_retained` / `kcc_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | ECN-EWMA-Gewichte |
| `kcc_ecn_idle_decay_num` / `kcc_ecn_idle_decay_den` | 31 / 32 | 1-100k | Leerlauf-ECN-Abfall |

### min_rtt

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `kcc_minrtt_fast_fall_cnt` | 3 | 0-3 | Schnellabfall-Zähler |
| `kcc_minrtt_fast_fall_div` | 4 | 1-256 | Schnellabfall-Schwellwertdivisor |
| `kcc_minrtt_sticky_num` / `kcc_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | Haftendes-Abfall-Verhältnis |
| `kcc_minrtt_srtt_guard_num` / `kcc_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | SRTT-Schutzverhältnis |

### LT-Bandbreite

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `kcc_lt_intvl_min_rtts` | 4 | 1-127 | RTTs | Minimale Intervalllänge |
| `kcc_lt_intvl_max_mult` | 4 | 1-32 | Intervall-TimeOut-Multiplikator |
| `kcc_lt_loss_thresh` | 25 | 1-65535 | BBR_UNIT | Mindestverlustverhältnis |
| `kcc_lt_bw_ratio_num` / `kcc_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | Relative Toleranz |
| `kcc_lt_bw_diff` | 500 | 0-100k | bytes/s | Absolute Toleranz |
| `kcc_lt_bw_max_rtts` | 48 | 1-4094 | RTTs | LT BW max. aktive RTTs |
| `kcc_lt_bw_ema_num` / `kcc_lt_bw_ema_den` | 1 / 2 | 0-100 / 1-100k | LT BW EMA-Gewicht |


### ACK-Aggregations-Konfidenz

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `kcc_agg_enable` | 1 | 0-1 | Hauptschalter |
| `kcc_agg_confidence_thresh` | 512 | 0-10000 | cwnd-Kompensations-Konfidenzschwellwert |
| `kcc_agg_max_comp_ratio` | 50 | 0-100 | % des BDP | cwnd-Kompensationsgrenze |
| `kcc_agg_max_comp_duration` | 8 | 1-128 | RTTs | Watchdog-Timeout |
| `kcc_agg_r_hysteresis` | 75 | 0-100 | % | R-Hysterese-Abfall |
| `kcc_agg_r_multiplier_min` / `kcc_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | R-Skalierungsbereich (256=1x) |
| `kcc_agg_factor4_ratio_num` / `kcc_agg_factor4_ratio_den` | 3 / 2 | 1-100k | Faktor-4-Verhältnis |
| `kcc_agg_safety_bdp_mult` | 3 | 1-100 | Sicherheitsschutz-BDP-Multiplikator |
| `kcc_agg_max_window_ms` | 100 | 1-10000 | ms | extra_acked-Grenzfenster |
| `kcc_agg_max_decay_pct` | 75 | 0-100 | % | Watchdog-Abfallrate |
| `kcc_agg_window_rotation_rtts` | 5 | 1-65535 | RTTs | Fensterrotationsperiode |
| `kcc_agg_factor_weight` | 256 | 1-1024 | Punktzahl pro Faktor |
| `kcc_agg_confidence_max` | 1024 | 256-65535 | Maximale Konfidenz |

### EWMA-Koeffizienten

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `kcc_ewma_qdelay_num` / `kcc_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | qdelay-EWMA-Gewicht |
| `kcc_ewma_jitter_num` / `kcc_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | Jitter-EWMA-Gewicht |

### Sonstiges

| Parameter | Standard | Bereich | Beschreibung |
|-----------|----------|---------|-------------|
| `kcc_probe_bw_cycle_len` | 8 | 2-256 | PROBE_BW-Zyklenphasen (Zweierpotenz) |
| `kcc_probe_bw_cycle_rand` | 8 | 1-cycle_len | Zufälliger Phasenversatz |
| `kcc_full_bw_thresh_num` / `kcc_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | STARTUP-Austritts-Wachstumsschwellwert |
| `kcc_full_bw_cnt` | 3 | 1-3 | Nicht-Wachstumsrunden zum Austritt |
| `kcc_probe_rtt_mode_ms_num` / `kcc_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | PROBE_RTT-Verweildauer |
| `kcc_pacing_margin_num` / `kcc_pacing_margin_den` | 1 / 100 | 0-50 / 1-100k | Pacing-Marge (1 = 1%) |
| `kcc_probe_cwnd_bonus` | 2 | 0-100 | Segmente | Phase-0-cwnd-Bonus |
| `kcc_bw_rt_cycle_len` | 10 | 2-256 | Runden | BW-Fensterlänge (gleitend) |
| `kcc_cwnd_min_target` | 4 | 1-1000 | Segmente | Min. cwnd (PROBE_RTT) |
| `kcc_bdp_min_rtt_us` | 1 | 0-100k | µs | BDP-min_rtt-Untergrenze |
| `kcc_edt_near_now_ns` | 1000 | 0-10M | ns | EDT-Beinahe-Jetzt-Schwellwert |
| `kcc_min_tso_rate` | 1.200.000 | 1-1 Mrd. | bytes/s | TSO-Niedrigratenschwellwert |
| `kcc_min_tso_rate_div` | 8 | 1-256 | TSO-Ratendivisor (adaptive Basis) |
| `kcc_tso_max_segs` | 127 | 1-65535 | Segmente | Max. TSO-Segmente |
| `kcc_tso_headroom_mult` | 3 | 0-1000 | TSO-Headroom-Multiplikator |
| `kcc_sndbuf_expand_factor` | 3 | 2-100 | Sendepuffer-Expansionsfaktor |
| `kcc_ack_epoch_max` | 0xFFFFF | 64K-2G | bytes | ACK-Epochengrenze |
| `kcc_extra_acked_max_ms_num` / `kcc_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | Max. ACK-Aggregationsfenster |
| `kcc_probe_rtt_decouple` | 1 | 0-1 | PROBE_RTT-Entkopplung aktivieren (nur FILTER-Modus) |
| `kcc_rtt_mode` | 1 | 0-1 | Modell-RTT-Strategie: 1=FILTER (direkt Kalman), 0=MIN (geklammert) |
| `kcc_recal_p_est_thresh` | 25000 | 1-100M | p_est-Schwellenwert für Sicherheitsnetz-Neukalibrierung |
| `kcc_probe_rtt_long_rtt_us` | 20000 | 0-10M | µs | Lang-RTT-Schwellwert |
| `kcc_probe_rtt_long_interval_div` | 1 | 1-1000 | Lang-RTT-Intervall-Divisor |
| `kcc_alone_confirm_rounds` | 3 | 1-32 | Runden | Runden vor Aktivierung des Einzelfluss-Modus |
| kcc_alone_agg_state_level | 1 | 0-2 | — | Aggregationsstrenge (0=nur IDLE, 1=≤SUSPECTED Standard, 2=≤CONFIRMED zu aggressiv) |
| `kcc_alone_bypass_ecn` | 0 | 0-1 | — | ECN-Backoff im Alleinmodus überspringen (1=überspringen, 0=aktiv) |

## Datenpfad

```
ACK kommt an (rate_sample)
    │
    ▼
kcc_main()
    │
    ├──► ACK-Aggregations-Konfidenzpipeline (wenn kcc_agg_enable)
    │      messen → bewerten → zustand → watchdog
    │
    ├──► kcc_update_model()
    │      ├── kcc_update_bw()              Max-BW mit gleitendem Fenster
    │      ├── kcc_update_ecn_ewma()        ECN-CE-Markierungsverhältnis
    │      ├── kcc_update_ack_aggregation()  Doppelfenster-extra_acked
    │      ├── kcc_update_cycle_phase()     PROBE_BW-Phasenfortschritt
    │      ├── kcc_check_full_bw_reached()  STARTUP-Austrittserkennung
    │      ├── kcc_check_drain()            DRAIN-Eintritt/Austritt + Drain-Überspringen
    │      ├── kcc_update_min_rtt()         Kalman + Fenster-min-RTT + PROBE_RTT
    │      └── Modus-spezifische Verstärkungszuweisung
    │
    ├──► kcc_apply_cwnd_constraints()
    │      └── kcc_ecn_backoff()            ECN-Rücknahme (nur cwnd_gain)
    │
    ├──► kcc_set_pacing_rate()              sofortig, BBR-Regel
    │
    └──► kcc_set_cwnd()                    BDP + Agg-Kompensation
```

## Kalman-Filter Interner Ablauf

```
RTT-Messung (rtt_us)
    │
    ├── Ungültig (≥0 und < dynamic_max)? Ja → verwerfen
    │
    ├── Kaltstart (sample_cnt==0)? Ja → init: x_est=z, p_est=max(p_init, rtt_us/div)
    │                                          (umgeht RTT-Max-Sperre)
    │
    ├── Adaptives Q: Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   Adaptives R: R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── Innovation: innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && Cooldown abgelaufen?
    │   ├── Yes: p_est = p_est_init, cooldown = 15, mark qboost_fired
    │   └── No:  cooldown-- if active
    │
    ├── Vorhersage: p_pred = p_est + Q
    │
    ├── Ausreißer-Sperre: |innov| > dyn_thresh && p_pred ≤ converged?
    │   ├── Ja & reject_cnt < max → ablehnen, ++consec_reject_cnt, zurück
    │   └── Ja & reject_cnt ≥ max → Zwangsannahme (Anti-Sperre)
    │
    └── Kalman-Aktualisierung:
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov (auf nicht-negativ begrenzt)
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── Jitter-EWMA-Aktualisierung
         ├── qdelay-EWMA-Aktualisierung
         ├── BBR-S-Kovarianz-angepasste Rauschschätzung
         └── sample_cnt++
```

## Diagnose

BBR-kompatible Diagnoseschnittstelle über `ss -i` (`INET_DIAG_BBRINFO`):

```
bbr_bw_lo/bbr_bw_hi: 64-Bit-Bandbreitenschätzung (bytes/s)
bbr_min_rtt:         aktuelles min_rtt_us
bbr_pacing_gain:     aktuelle Pacing-Verstärkung (BBR_UNIT, 256=1,0x)
bbr_cwnd_gain:       aktuelle cwnd-Verstärkung (BBR_UNIT)
```

## Verwendung

```sh
# Compile kernel module
make

# Dev load (insmod, no dependency resolution)
sudo make load

# Install and formal load (modprobe)
sudo make install
sudo make modload

# Unload
sudo make unload

# Select KCC algorithm
echo KCC > /proc/sys/net/ipv4/tcp_congestion_control
```

Die Parameterkonfiguration erfolgt über `/proc/sys/net/kcc/`. Zum Beispiel:
```sh
# Enable gain decay on specific PROBE_BW phases
echo 1 > /proc/sys/net/kcc/kcc_cycle_decay_mask

# Adjust ECN backoff sensitivity
echo 30 > /proc/sys/net/kcc/kcc_ecn_backoff_num
```

## Parallelitäts- und Sicherheitsmodell

KCC verwendet bewusst kein READ_ONCE/WRITE_ONCE oder RCU für seine eigenen Datenstrukturen. Dieses Design ist konsistent mit allen Kernel-internen CC-Modulen wie BBR und CUBIC.

`kcc_init()` wird im Prozesskontext ausgeführt (während der Socket-Erstellung), bevor der Socket irgendeinem Softirq ausgesetzt wird. `kcc_release()` wird ausgeführt, nachdem der Kernel garantiert, dass kein Softirq mehr die ACKs dieses Sockets verarbeitet. Ein transienter veralteter Wert eines globalen Modulparameters betrifft höchstens einen ACK und wird beim nächsten ACK korrigiert.

Die einzige Ausnahme: `sk->sk_pacing_rate` / `sk->sk_pacing_shift` sind Socket-Layer-Felder, die Userspace gleichzeitig über `setsockopt` ändern kann, daher wird BBRs WRITE_ONCE/READ_ONCE-Konvention beibehalten.

## Leistungszusammenfassung

Testumgebung: China → USA LAX, 212 ms RTT, 8 parallele Flüsse, 26 % Paketverlust, 1 Gbps gemeinsam genutzter VPS-Engpass.

| Metrik | KCC v1.0 | BBR (Kontrolle) | Delta |
|--------|----------|---------------|-------|
| Durchschnittlicher Durchsatz | 1.010 Mbps | 937 Mbps | **+7,8 %** |
| Intra-KCC-Unfairness | 3,1× | 6,2× (BBR) | **−50 %** |
| Schlechtester Einzelfluss | 60,6 Mbps | 30,8 Mbps | **+97 %** |
| Wiederholungsübertragungen | 150K/10s | 137K/10s | +9,5 % |
| Runde-3-Stabilität | 959 Mbps | 883 Mbps | **+8,6 %** |

Die Wiederholungsübertragungen sind etwas höher — ein Kompromiss, der mit der Aufrechterhaltung einer hohen Linkauslastung bei Paketverlust vereinbar ist. Die Kalman-gestützte min_rtt-Schätzung von KCC liefert eine genauere BDP-Baseline, sodass der Algorithmus auf demselben Pfad einen höheren Durchsatz als BBRv1 aufrechterhalten kann.

## Global Kalman BDP — Verbindungsübergreifende Bandbreiteninjektion

KCC v1.0 enthält einen optionalen verbindungsübergreifenden globalen Kalman-Filter, der die stationäre Engpassbandbreite des Servers schätzt. Diese Schätzung wird verwendet, um neue Verbindungen mit einer konservativ niedrigen „Dessert-Geschwindigkeit" zu starten — schnell genug, um den Kaltstart-Hochlauf zu überspringen, langsam genug, um Überschwingen zu vermeiden.

### Entwurfsprinzip

Der Filter wird mit Bandbreitenproben aus der PROBE\_BW **Cruise-Phase** (Gain = 1,0×) aller KCC-Verbindungen gespeist. Cruise-Phasen-Proben sind das sauberste Signal der tatsächlich verfügbaren Bandbreite — keine 1,25×-Probe-Überschwingung, keine 0,75×-Drain-Unterschwingung. Ein eindimensionaler Random-Walk-Kalman-Filter (Kalman 1960) verfolgt den globalen stationären Zustand.

Wenn eine neue Verbindung aufgebaut wird, wird die Schätzung des Filters verwendet, um Folgendes zu initialisieren:

| Injizierter Wert | Zweck |
|----------------|---------|
| `minmax` (max\_bw tracker) | Füllt den gleitenden Fenster-Bandbreitenverlauf vor, sodass die ersten wenigen unsauberen ACK-Proben ihn nicht auf Null ziehen |
| `sk_pacing_rate` | Anfängliche Pacing-Rate bei neutralem Gain (BBR\_UNIT); der 2,89×-Gain von STARTUP wird beim ersten ACK angewendet |
| `tp->snd_cwnd` | Initiales Congestion-Window, berechnet über `kcc_bdp()` bei neutralem Gain |

Eine defensive Untergrenze in `kcc_update_bw` verhindert, dass die ersten paar RTTs mit niedrigen Lieferraten-Proben die injizierte Schätzung während STARTUP überschreiben. Ein Full-BW-Guard in `kcc_check_full_bw_reached` verhindert, dass der iperf3-Kontrollnachrichtenaustausch STARTUP vorzeitig beendet.

### Dessert-Geschwindigkeits-Rabattverhältnis

Die effektive Injektionsgeschwindigkeit beträgt:

```
coeff = (discount_ratio) / high_gain
      = (num / den) / 2.89
```

wobei `high_gain ≈ 2.89` der BBR-STARTUP-Pacing-Multiplikator ist.

| num | coeff  | characteristic |
|-----|--------|----------------|
|  35 | 12.1%  | Maximale Sicherheit, Worst-Path |
|  50 | 17.3%  | Mittelachse (Standard) |
|  75 | 25.9%  | Mathematischer Dessert-Sweet-Spot |
|  80 | 27.6%  | Mathematische Ratenobergrenze (sollte nicht überschritten werden) |

**Hinweis:** `tcp_write_xmit` erzwingt ein anfängliches CWND von `TCP_INIT_CWND` (10 Segmente, ≈15 KB) für jede neue Verbindung. CWND wächst nur, wenn entfernte ACKs eintreffen, sodass die Dessert-Geschwindigkeit eine Obergrenze für die Pacing-Rate darstellt — der tatsächliche Durchsatz ist CWND-begrenzt, bis genügend ACKs empfangen wurden, um das Fenster zu öffnen.

### Konfiguration

Aktivierung per `sysctl`:

```bash
sysctl -w net.kcc.kcc_kf_enable=0           # master enable (default 0)
sysctl -w net.kcc.kcc_kf_discount_num=50   # dessert-speed numerator (default 50, range 35–75)
```

**Wichtige sysctl-Parameter** (`/proc/sys/net/kcc/`):

| Parameter | Standard | Bereich | Beschreibung |
|-----------|---------|-------|-------------|
| `kcc_kf_enable` | 0 | 0–1 | Master-Aktivierung für die globale Kalman-BDP-Injektion |
| `kcc_kf_discount_num` | 50 | 0–100 | Dessert-Geschwindigkeitszähler (% der Fair-Share-BW) |
| \`kcc_kf_discount_den\` | 100 | 1–100000 | Dessert-Geschwindigkeitsnenner |
| \`kcc_kf_steady_mode\` | 0 | 0/1 | — | Steady-Modus: verwendet bei Aktivierung den monoton steigenden Peak (kf_x_steady) für init_bw und ignoriert vorübergehende KF-Absenkungen |
| \`kcc_kf_startup_r_pct\` | 20 | 1–100 | Messrauschen R% während der Startup-Phase |
| \`kcc_kf_steady_r_pct\` | 5 | 1–100 | Messrauschen R% während des stationären Zustands |
| \`kcc_kf_q_shift\` | 20 | 0–30 | Prozessrauschen-Shift (Q = 1 << shift) |
| \`kcc_kf_chi2_num\` | 384 | 1–100000 | Chi-Quadrat-Ausreißerschwelle Zähler |
| \`kcc_kf_chi2_den\` | 100 | 1–100000 | Chi-Quadrat-Ausreißerschwelle Nenner |

Wenn kcc_kf_steady_mode aktiviert ist (1), verwendet die anfängliche Bandbreite neuer Verbindungen den monoton steigenden Peak der KF-Schätzung (kf_x_steady) anstelle der Live-Schätzung, die seit der letzten Hochdurchsatz-Verbindung abgesunken sein könnte. Dies verhindert Kaltstart-Mangel auf stabilen Pfaden. Der Peak wird beim Deaktivieren auf Null zurückgesetzt, was einen sauberen Neustart bei erneuter Aktivierung ermöglicht.

### Leistung in der ersten Sekunde (Trans-Pazifik, 212 ms RTT)

```
Without KF:  2.8 Mbps  →  85 Mbps  →  622 Mbps  →  steady
With KF:     50 Mbps   →  530 Mbps  →  650 Mbps  →  steady
```

Die Geschwindigkeit in der ersten Sekunde springt von ~3 Mbps (Kaltstart) auf ~50 Mbps (Dessert-Start), und die Konvergenz zum stationären Zustand wird innerhalb von 2–3 Sekunden erreicht. Wiederholungsübertragungen bleiben durchgehend bei null.

### Funktionsweise

1. Eine laufende KCC-Verbindung tritt in die PROBE\_BW Cruise-Phase ein → Round-Start-Grenze → füttert `kcc_kf_update(bw, 5%)` mit der aktuellen Lieferraten-Probe.
2. Der Kalman-Filter aktualisiert seine Schätzung `kcc_kf_x` (einen gleitenden Durchschnitt der stationären Engpassbandbreite).
3. Wenn eine **neue** Verbindung geöffnet wird, ruft `kcc_init` die Funktion `kcc_kf_get_init_bw(sk)` auf, die `fair × discount / high_gain` zurückgibt — eine gain-kompensierte, fair-share-initiale Bandbreitenschätzung.
4. Diese Schätzung initialisiert `sk_pacing_rate`, `tp->snd_cwnd` und den `minmax`-Bandbreiten-Tracker — die Verbindung startet mit der Dessert-Geschwindigkeit anstatt bei null.

### Algorithmus-Quelle

Der Global Kalman BDP-Filter basiert auf dem Artikel des Autors *Zur Kalman-Schätzung und technischen Implementierung globaler stationärer Bandbreite im Linux-Kernel* (CC BY-SA 4.0):
https://blog.csdn.net/liulilittle/article/details/161635652

---

*KCC v1.0 — basiert auf BBRv1 (Cardwell et al. 2016, ACM Queue) und dem Kalman-Filter (Kalman 1960).*

## Referenzen

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