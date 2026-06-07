[🇺🇸 English](../README.md) | [🇨🇳 中文](README_CN.md) | [🇹🇼 繁體中文](README_TW.md) | [🇪🇸 Español](README_ES.md) | [🇫🇷 Français](README_FR.md) | [🇷🇺 Русский](README_RU.md) | [🇸🇦 العربية](README_AR.md) | [🇩🇪 Deutsch](README_DE.md) | [🇯🇵 日本語](README_JA.md) | [🇰🇷 한국어](README_KO.md) | [🇮🇹 Italiano](README_IT.md) | [🇵🇹 Português](README_PT.md)

---

# TCP KCC v1.0 (Controllo di Congestione di Kalman)

Modulo di controllo della congestione TCP per ambienti VPS a larghezza di banda condivisa che combina la macchina a stati BBRv1 con un filtro di Kalman per la stima del ritardo di propagazione.

## Principi di Progettazione

Gli algoritmi di controllo della congestione devono bilanciare throughput, latenza, equità e tolleranza alle perdite. KCC adotta un approccio pragmatico:

1. BBRv1 fornisce una base consolidata. Macchina a stati, pacing, guadagni di ciclo, STARTUP/DRAIN/PROBE_BW/PROBE_RTT — KCC adotta questi meccanismi senza modifiche.

2. Il filtro di Kalman migliora la precisione della stima. Separare il ritardo di propagazione reale dal ritardo di accodamento e dal jitter produce una stima min_rtt più accurata, consentendo un calcolo BDP più preciso, un CWND meglio calibrato e un pacing più stabile.

3. Le dinamiche inter-algoritmo seguono l'equilibrio competitivo TCP standard. KCC non limita artificialmente la sua velocità di invio in risposta alla coda rilevata da flussi esterni. Il decadimento del guadagno (riduzione della sonda basata sulla coda) è disponibile come opt-in tramite kcc_cycle_decay_mask ma disabilitato per impostazione predefinita per preservare l'intensità completa della sonda.

4. L'equità intra-KCC è mantenuta attivamente. La convergenza di Kalman garantisce che i flussi KCC sullo stesso host condividano una stima min_rtt coerente, eliminando il ciclo di feedback vincitore-prende-tutto che causa una grave inequità nelle distribuzioni multi-flusso di BBR puro.

## Panoramica dell'Algoritmo

TCP KCC implementa un modulo di controllo della congestione lato mittente per il kernel Linux come modulo caricabile `tcp_kcc.ko`. La funzione di controllo della congestione `kcc_main()` viene invocata ad ogni ACK da `tcp_ack()`, ricevendo una struttura `rate_sample` che contiene campioni di larghezza di banda e RTT del kernel insieme a contatori di consegna e perdita. L'algoritmo opera in due regimi temporali: un **percorso rapido per-ACK** che aggiorna lo stato delle misurazioni e calcola obiettivi istantanei di pacing e finestra, e un **percorso lento per-round** che valuta le condizioni di transizione di stato e ricalcola i guadagni.

La pipeline di misurazione centrale è composta da due componenti:

1. **Filtro di larghezza di banda massima a finestra scorrevole** (`minmax_running_max` da `linux/win_minmax.h`): finestra che copre gli ultimi `kcc_bw_rt_cycle_len` (default 10) round trip. Fornisce la stima `max_bw` compatibile con BBR.

2. **Stimatore del ritardo di propagazione tramite filtro di Kalman**: sostituisce l'RTT minimo a finestra scorrevole di BBRv1 ed è la fonte predefinita per la stima RTT del BDP (vedere [Strategia RTT del Modello](#strategia-rtt-del-modello)). Un filtro di Kalman a stato singolo (Kalman 1960) che opera in unità in virgola fissa di `kcc_kalman_scale` × µs, modellando il ritardo di propagazione reale come una camminata casuale:
   - Stato: `x[k] = x[k−1] + w[k]`, `w ~ N(0, Q)`
   - Osservazione: `z[k] = x[k] + v[k]`, `v ~ N(0, R)`

Convenzioni in virgola fissa: `BW_UNIT = 1 << 24` per la larghezza di banda (segmenti * 2^24 / µs), `BBR_UNIT = 1 << 8 = 256` come unità di guadagno adimensionale.

## Strategia RTT del Modello

KCC introduce una strategia configurabile per la stima RTT utilizzata nel calcolo del BDP (Prodotto Larghezza di Banda-Ritardo), controllata da `kcc_rtt_mode`:

| Modalità | Valore | Comportamento | Caso d'uso |
|----------|--------|---------------|------------|
| FILTER | 1 (predefinito) | Uso diretto di `x_est_us` — la stima grezza del filtro di Kalman/a finestra scorrevole | WAN/VPS di produzione: resiliente ai cambi di rotta, nessun crollo di throughput |
| MIN | 0 | `min(x_est_us, min_rtt_us)` — limitare la stima di Kalman contro il minimo finestrato | Verifica stabilità modulo kernel; collegamenti a RTT statico |

**Perché FILTER è il predefinito:**

- **Resilienza ai cambi di rotta**: Quando un rerouting BGP aumenta l'RTT fisico (es. 50 ms → 100 ms), il guadagno di Kalman K_k reagisce entro pochi RTT e porta la stima alla nuova latenza. La modalità MIN si blocca sul vecchio `min_rtt_us` fino alla scadenza della finestra, dimezzando il BDP.

- **Difese integrate**: Il cancello outlier scarta i campioni di picco di coda prima che entrino nel filtro. La stima adattativa del rumore Q/R riduce il guadagno di Kalman quando la rete è rumorosa, quindi il filtro naturalmente diffida del gonfiamento di coda transitorio e mantiene la stima vicino al vero ritardo di propagazione.

- **Disaccoppiamento PROBE_RTT**: La modalità FILTER abilita la funzione `kcc_probe_rtt_decouple` — il filtro di Kalman traccia il pavimento RTT senza richiedere lo svuotamento periodico di 4 pacchetti.

Commutazione a runtime: `echo 0 > /proc/sys/net/kcc/kcc_rtt_mode` per tornare alla modalità MIN.

## Macchina a Stati

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

Quattro modalità codificate come campo `mode` a 2 bit in `struct KCC`:

- **STARTUP (0)**: Stato iniziale. `pacing_gain` ≈ 2,885x (`kcc_high_gain_val`), anche `cwnd_gain` a 2,885x. Esplorazione esponenziale della larghezza di banda.
- **DRAIN (1)**: Entrato dopo l'uscita da STARTUP. `pacing_gain` ≈ 0,347x (`kcc_drain_gain_val`), `cwnd_gain` rimane a 2,885x. Drena la coda accumulata durante STARTUP.
- **PROBE_BW (2)**: Stato stazionario. Cicla attraverso una tabella di guadagni a 256 slot (pattern predefinito a 8 fasi ripetuto: 1,25x/0,75x/8×1,0x).
- **PROBE_RTT (3)**: Drena periodicamente il traffico in volo a `kcc_cwnd_min_target` (default 4 segmenti) per ottenere un campione RTT fresco.

### STARTUP → DRAIN

Attivato quando `full_bw_reached` è impostato — dopo `kcc_full_bw_cnt` (default 3) round consecutivi in cui `max_bw` non riesce a crescere di almeno `kcc_full_bw_thresh_val` (default 1,25x) rispetto al picco osservato in precedenza. Il BDP con guadagno 1,0x viene scritto in `snd_ssthresh`. `qdelay_avg` viene azzerato per evitare che l'accumulo della coda di STARTUP influenzi PROBE_BW.

### DRAIN → PROBE_BW

Attivato quando il traffico in volo stimato a EDT ≤ traffico in volo target con guadagno BDP 1,0x. **Ottimizzazione salto DRAIN**: quando il filtro di Kalman è converguto E `qdelay_avg` è inferiore a `kcc_drain_skip_qdelay_us` (default 1000 µs), la fase DRAIN viene saltata — conversione anticipata a PROBE_BW.

All'ingresso di PROBE_BW, l'indice di fase del ciclo viene randomizzato: `cycle_idx = len − 1 − rand(kcc_probe_bw_cycle_rand)` (default `len − 1 − rand(8)`), che decorrela i flussi concorrenti che condividono un collegamento congestionato.

### PROBE_BW → PROBE_RTT

Attivato quando l'intervallo del filtro PROBE_RTT scade — il timestamp `min_rtt_stamp` non è stato aggiornato entro l'intervallo calcolato. cwnd viene salvato in `prior_cwnd`, il pacing viene impostato per drenare.

### PROBE_RTT → PROBE_BW

Dopo che il traffico in volo scende a `kcc_cwnd_min_target` o viene osservato un limite di round, persiste per almeno `kcc_probe_rtt_mode_ms_val` (default 200 ms) e almeno un round completo osservato, poi esce. cwnd viene ripristinato ad almeno `prior_cwnd`, il pacing viene temporaneamente sovrascritto con `kcc_high_gain_val` per un rapido riempimento del tubo.

### Recupero e Perdita

- Su TCP_CA_Loss: `full_bw` e `full_bw_cnt` vengono resettati, `round_start` impostato a 1, `packet_conservation` azzerato a 0. Se LT BW non è attivo, inietta un evento di perdita sintetico per attivare il campionamento LT.
- Ingresso recupero (TCP_CA_Recovery): `packet_conservation` abilitato, cwnd = in volo + accusato.
- Uscita recupero: ripristinato a `prior_cwnd`, `packet_conservation` azzerato.
- `kcc_undo_cwnd()`: resetta `full_bw` e `full_bw_cnt` (preservando `full_bw_reached`), azzera lo stato LT BW.

### Rilevamento round (Allineamento BBR)

I confini dei round vengono rilevati secondo BBR, Cardwell et al. 2016: quando `prior_delivered` raggiunge o supera `next_rtt_delivered` tramite confronto unsigned `!before()`. `next_rtt_delivered` è inizializzato a `0` — come BBR standard — quindi il primo ACK avvia immediatamente il round 1, indipendentemente dalla consegna dei segmenti di handshake. La convalida dei campioni di velocità usa `interval_us <= 0` (non `== 0`) per corrispondere alla guardia esatta di BBR, intercettando intervalli negativi.

- `next_rtt_delivered` inizializzato a `0` (parità BBR): il primo round inizia al primo ACK.
- Convalida `interval_us <= 0`: corrisponde esattamente a BBR, rifiuta intervalli negativi.
- `round_start` viene azzerato a `0` all'inizio di `kcc_update_bw()`, prima del controllo di convalida — corrispondente al posizionamento `bbr->round_start = 0` di BBR.

## Misurazioni Principali

### Stima della Larghezza di Banda

Filtro di larghezza di banda massima a finestra scorrevole (`minmax_running_max` da `linux/win_minmax.h`) su `kcc_bw_rt_cycle_len` (default 10) round. bw istantaneo = `delivered × BW_UNIT / interval_us` calcolato per ACK. Alimentato nella finestra scorrevole solo quando non è limitato dall'applicazione o quando bw ≥ bw massimo corrente (regola BBR).

Quando `lt_use_bw` è attivo, la stima attiva della larghezza di banda passa a `lt_bw` (stima di larghezza di banda a lungo termine).

### Filtro di Kalman

Ricorsione di Kalman scalare a stato singolo (complessità O(1)):

```
Predici:
  x_pred = x_est          (transizione di stato identità)
  p_pred = p_est + Q      (predizione covarianza)

Aggiorna:
  innov   = z − x_pred    (innovazione)
  K       = p_pred / (p_pred + R)   (guadagno di Kalman [0,1])
  x_est   = x_pred + K × innov      (aggiornamento stato)
  p_est   = (1 − K) × p_pred        (covarianza a posteriori)
```

**Rumore di processo adattativo Q**:
```
Q_base   = kcc_kalman_q (default 100)
q_factor = max(kcc_kalman_q_min_factor_val, min_rtt_us / kcc_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × kcc_kalman_q_scale_cap)
Q        = min(Q, kcc_kalman_q_max)
```

**Rumore di misurazione adattativo R**:
```
R = R_base + max(0, jitter_ewma − kcc_jitter_r_thresh_us) × R_base / kcc_jitter_r_scale
R = min(R, R_base × kcc_kalman_r_max_boost)
```

**Rilevamento cambio percorso Q-Boost (con gate di confidenza + cooldown)**: quando `|innovation| > kcc_kalman_q_boost_thresh_val` (default ≈ 4 ms di spostamento RTT) E il filtro è converguto (`p_est ≤ kcc_kalman_converged_p_est_val`, default 500), `p_est` viene reimpostato a `kcc_kalman_p_est_init_val`, portando il guadagno di Kalman verso 1.0 per una rapida convergenza. Un cooldown di `kcc_kalman_qboost_cdwn` (default 15) campioni tra eventi qboost successivi previene l'attivazione incontrollata su percorsi con perdite e alto jitter RTT.

**Cancello outlier**: soglia dinamica `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`. Applicato solo quando `p_pred ≤ kcc_kalman_converged_p_est_val`. Dopo `kcc_kalman_max_consec_reject` (default 25) rifiuti consecutivi, il campione successivo viene forzatamente accettato per prevenire un blocco auto-rafforzante.

**Stima del rumore tramite covarianza accoppiata (BBR-S)**: `q_est = (1−α) × q_est + α × (K × innov)²`, `r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`. Modalità di combinazione: modalità 0 = solo euristico, modalità 1 = max (default), modalità 2 = miscela pesata.

**Presa in carico di Kalman**: quando `x_est > 0` e `sample_cnt ≥ kcc_kalman_min_samples` (default 5), `min_rtt_us` viene sostituito da `x_est / kcc_kalman_scale`. `min_rtt_stamp` non viene aggiornato — il trigger dell'intervallo PROBE_RTT rimane indipendente.

**Strategia RTT del Modello**: La stima RTT utilizzata per il calcolo del BDP è controllata da `kcc_rtt_mode`. In modalità FILTER (predefinito), viene usato `model_rtt = x_est_us` direttamente — la stima di Kalman/a finestra scorrevole senza limitazione. In modalità MIN, `model_rtt = min(x_est_us, min_rtt_us)` — la stima di Kalman viene limitata contro il minimo finestrato per garantire che il BDP non si gonfi mai. Il valore predefinito FILTER è raccomandato per distribuzioni di produzione WAN/VPS dove la latenza del percorso può cambiare bruscamente (rerouting BGP, handover LEO, cambi di cella mobile). Vedere [Strategia RTT del Modello](#strategia-rtt-del-modello).

## Miglioramenti BBR

### Decadimento del Guadagno

Abilitato dalla bitmap a 256 bit `kcc_cycle_decay_mask[]` per fasi specifiche di PROBE_BW. Formula di decadimento (su campione Kalman accettato):

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = scalatura inversa di p_est (BBR_UNIT al massimo)
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                     × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                     × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

Scalatura della confidenza di Kalman: quando `p_est > kcc_kalman_converged_p_est`, il decadimento viene proporzionalmente ridotto, evitando un arretramento eccessivo quando il filtro è incerto.

### Arretramento ECN

Condizioni di attivazione (tutte devono essere soddisfatte):
1. `kcc_ecn_enable_val != 0`
2. Kalman converguto (`p_est < converged`, `sample_cnt >= min_samples`)
3. `ecn_ewma > 0` (marchi CE osservati)
4. `qdelay_avg > kcc_ecn_qdelay_thresh_us_val` (default 2000 µs)
5. La modalità NON è PROBE_BW (cwnd_gain è fisso a 2x in PROBE_BW)

Durante le fasi di esplorazione (`pacing_gain > BBR_UNIT`), l'arretramento ECN viene graduato da `BBR_UNIT² / pacing_gain` — ~80% di arretramento a sonda 1,25x, ~65% a guadagno STARTUP 2,89x.

Rapporto marchio ECN EWMA: aggiornato ai limiti di round da `kcc_ecn_ewma_retained / kcc_ecn_ewma_total` (default 3/4), con delicato decadimento per-ACK di `kcc_ecn_idle_decay_num / kcc_ecn_idle_decay_den` (default 31/32) su ogni ACK senza nuovi marchi CE.

### Rilevamento Flusso Singolo

Quando KCC rileva che il flusso è probabilmente da solo nel collo di bottiglia (basso ritardo di coda, basso jitter, nessun marchio ECN, nessuna aggregazione ACK, nessuna larghezza di banda LT), passa automaticamente a una modalità BBR pura:

- `kcc_get_model_rtt()` restituisce direttamente `min_rtt_us` (evitando la stima livellata di Kalman, che ha un piccolo bias positivo dovuto al rumore di misura unilaterale).
- `kcc_ecn_backoff()` è configurabile tramite `kcc_alone_bypass_ecn` (predefinito 1) — su un percorso a flusso singolo, i marchi ECN sono falsi positivi dell'AQM perché non c'è un altro mittente in competizione. Saltarlo corrisponde al comportamento ECN zero di BBR. Impostare a 0 per mantenere il backoff ECN anche in modalità singola (conservativo).
- La condizione LT BW (policer) è configurabile tramite `kcc_alone_bypass_lt_bw` (predefinito 1) — un percorso a flusso singolo non ha policer, quindi LT BW non può attivarsi legittimamente. Saltarla evita uscite spurie dalla modalità singola causate da falsi trigger. Impostare a 0 per il comportamento rigoroso originale.

Ciò elimina il divario di prestazioni in flusso singolo tra KCC e BBR, preservando al contempo il ciclo di protezione completo di KCC (Kalman, arretramento ECN, decadimento del guadagno, larghezza di banda LT) per scenari multi-flusso.

**Isteresi**: L'ingresso richiede `kcc_alone_confirm_rounds` (default 3) round consecutivi qualificati — evitando oscillazioni durante brevi periodi di calma nella competizione multi-flusso ("conservativo per accelerare"). L'uscita è immediata — qualsiasi fallimento di qualifica cancella il flag e resetta il contatore di conferma ("aggressivo per rallentare").

Condizioni di qualifica (tutte e sei devono essere soddisfatte al confine di un turno):
0. Kalman convergente (`sample_cnt >= kcc_kalman_min_samples`) — fidarsi di qdelay/jitter come segnali di coda
1. `qdelay_avg < kcc_alone_qdelay_thresh_us` (predefinito 1000 us) — coda quasi vuota
2. `jitter_ewma < kcc_alone_jitter_thresh_us` (predefinito 2000 us) — solo micro-jitter di clock ACK
3. `ecn_ewma == 0` — nessun contrassegno di congestione da AQM
4. `lt_use_bw == 0` — non in modalità a velocità limitata rilevata dal policer
5. `agg_state <= max` secondo `kcc_alone_agg_state_level` (predefinito 1) — tre livelli di rigore di aggregazione ACK: 0 = solo IDLE (più rigido, zero aggregazione), 1 = ≤ SUSPECTED (predefinito, consente aggregazione transitoria), 2 = ≤ CONFIRMED (più permissivo, blocca solo aggregazione persistente)

### Intervallo PROBE_RTT Dinamico

Mappa `p_est` di Kalman su un intervallo PROBE_RTT per connessione:

```
p_est ≤ converged:              interval = dyn_max (default 30s)
p_est ≥ high (= mult × conv):   interval = base (default 10s)
converged < p_est < high:       interpolazione lineare
```

Riduce la frequenza di PROBE_RTT quando la confidenza è alta (`p_est` basso), diminuendo il jitter del throughput su percorsi stabili. Torna all'intervallo classico di 10 secondi quando la confidenza è bassa.

**Jitter di ingresso per flusso**: Per evitare che tutti i flussi coesistenti entrino contemporaneamente in PROBE_RTT (svuotandosi a 4 pacchetti aggregati ~1.8 Mbps e poi ricaricandosi a 2.89×), ogni flusso aggiunge un jitter derivato da hash (distribuzione 0–845 ms) al proprio intervallo PROBE_RTT. Al massimo ~1 flusso è in PROBE_RTT in qualsiasi istante, eliminando il collasso simultaneo svuotamento/ricarica che induce RTO.

### Disaccoppiamento PROBE_RTT e ricalibrazione intelligente

Il meccanismo PROBE_RTT di BBRv1 svuota il tubo a 4 pacchetti ogni ~10 secondi per misurare `min_rtt_us`. Ciò è necessario per uno stimatore min-RTT basato su finestra — la finestra non può distinguere il ritardo di propagazione dal ritardo di coda a meno che il tubo non sia vuoto. Il costo è un calo periodico di throughput (la "sega" di BBR).

In modalità FILTER, il filtro di Kalman sostituisce completamente la finestra. Può separare il rumore di coda dal vero ritardo di propagazione attraverso il cancello outlier e la stima adattativa del rumore — nessuno svuotamento del tubo richiesto. Il parametro `kcc_probe_rtt_decouple` (predefinito 1) controlla questo:

| Modalità | Valore | Comportamento |
|----------|--------|---------------|
| Disaccoppiato | 1 (predefinito) | **Kalman sano** (p_est ≤ `kcc_recal_p_est_thresh`): sopprimere PROBE_RTT completamente → zero cali di throughput, zero collassi sincroni. **Kalman divergente** (p_est > soglia): attivare automaticamente il PROBE_RTT tradizionale come rete di sicurezza → ripristina la linea di base del filtro, poi il disaccoppiamento riprende. |
| Tradizionale | 0 | PROBE_RTT periodico cieco ogni ~10s (compatibile BBR). |

**Euristica di ricalibrazione intelligente** (`kcc_kalman_needs_recalibration()`): In funzionamento stabile su un percorso stabile, la covarianza d'errore p_est di Kalman converge a p_est_floor (~4–10), ben al di sotto della soglia `kcc_recal_p_est_thresh` (250.000 = 25% di p_est_max). Un p_est in aumento segnala che il modello interno del filtro non spiega più le osservazioni — tipicamente perché il percorso è cambiato materialmente. Quando p_est supera la soglia, un singolo svuotamento PROBE_RTT tradizionale ripristina la linea di base del filtro; il Kalman riconverge e il disaccoppiamento riprende automaticamente.

Questo trasforma PROBE_RTT **da un'automutilazione periodica cieca** in **una ricalibrazione intelligente basata sulla fiducia** — il protocollo svuota il tubo solo quando ha prove empiriche che il filtro ha perso fiducia.

Richiede `kcc_rtt_mode == 1`. Inefficace in modalità MIN (la modalità MIN dipende da PROBE_RTT per aggiornare `min_rtt_us`).

| Parametro | Predefinito | Intervallo | Descrizione |
|-----------|-------------|------------|-------------|
| `kcc_probe_rtt_decouple` | 1 | 0–1 | Abilitare disaccoppiamento PROBE_RTT (solo modalità FILTER) |
| `kcc_recal_p_est_thresh` | 250.000 | 1–100.000.000 | Soglia p_est per rete di sicurezza di ricalibrazione |

### Stima della Larghezza di Banda LT

Stimatore del limite inferiore attivato da perdita. L'intervallo di campionamento copre [4, 16] RTT. Valido quando il rapporto di perdita ≥ 5,9% (`kcc_lt_loss_thresh` default 15/256). Larghezza di banda `bw = delivered × BW_UNIT / interval_us`.

A differenza della media semplice di BBR (`(bw + lt_bw) >> 1`), KCC utilizza un EMA configurabile (`kcc_lt_bw_ema_num / kcc_lt_bw_ema_den`, default 1/2 = 0,5):

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

L'attivazione differisce da BBR: KCC memorizza `lt_bw` al primo intervallo valido ma NON imposta `lt_use_bw`; è richiesta coerenza con un intervallo precedente — riduce la falsa attivazione da rumore di misurazione.

**Cancello di congestione a doppia soglia**: Prima di impostare `lt_use_bw = 1`, vengono valutati sia un controllo EWMA persistente della coda (`qdelay_avg > kcc_ecn_qdelay_thresh_us_val`) che un controllo istantaneo della coda basato su SRTT (`srtt_us − min_rtt_us > kcc_lt_bw_inst_qdelay_thresh_us`, default 5000 µs). Quando viene rilevata congestione, il campionamento LT BW viene interrotto. Il controllo SRTT funziona senza allocazione `ext`, fornendo una rete di sicurezza contro il fallimento dell'allocazione.

Boost della sonda LT BW (`kcc_lt_bw_probe_pct`, default 10%): amplifica `pacing_gain` di `1 + probe_pct/100` su tutte le fasi PROBE_BW. Componente rampa: aumento di `+1% ogni 8 RTT`, limitato a `2 × probe_pct`.

Auto-recupero LT BW (`kcc_lt_restore_ratio_num/den`, default 5/4 = 1,25x): quando `max_bw > lt_bw × ratio` per `kcc_lt_restore_consec_acks` (default 3) ACK consecutivi, LT BW esce automaticamente e la normale esplorazione PROBE_BW riprende.

### Compensazione Basata sulla Confidenza di Aggregazione ACK (ispirata a BBRplus)

Aggiunge un secondo strato con cancello di confidenza sopra lo stimatore tradizionale a doppio slot extra-acked.

**Quattro fattori ortogonali** (ciascuno contribuisce `kcc_agg_factor_weight` punti, default 256):
1. Kalman converguto (`p_est < converged` + `sample_cnt >= min_samples`)
2. Non in recupero perdita (`icsk_ca_state < TCP_CA_Recovery`)
3. RTT entro `min_rtt_us + kcc_agg_factor3_qdelay_us` (default 2ms) dal ritardo di propagazione reale
4. `extra_acked` entro `kcc_agg_factor4_ratio_num/den` (default 1,5x) del massimo finestrato

**Quattro stati**: INATTIVO (< `kcc_agg_thresh_suspected`=256), SOSPETTO (≥256), CONFERMATO (≥512), FIDATO (≥768).

**Strato di segnale** (sempre attivo): la confidenza interpola linearmente il fattore di scala R `[r_min, r_max]`. R sale istantaneamente (risposta rapida), decade al `kcc_agg_r_hysteresis`% (default 75% trattenuto, ~4 RTT per tornare alla baseline) per RTT.

**Strato di controllo** (`agg_state ≥ CONFIRMED`): compensazione cwnd con cancello di sicurezza a cinque livelli:
1. Blocca se il ritardo di coda > `kcc_agg_safety_qdelay_us` (default 4ms)
2. Blocca durante il recupero perdita
3. Blocca se cwnd > `BDP × kcc_agg_safety_bdp_mult` (default 3x)
4. Blocca se in volo > cwnd sicuro + obiettivo segmenti TSO
5. Watchdog: declassa CONFERMATO→SOSPETTO dopo `kcc_agg_max_comp_duration` (default 8) RTT consecutivi

### Reset di qdelay_avg in DRAIN

Alla transizione verso DRAIN, `qdelay_avg` viene azzerato, impedendo alla stima della coda di STARTUP di persistere in PROBE_BW.

### Adattamento del Divisore TSO

`kcc_min_tso_segs()` regola il divisore della soglia di velocità in base allo stato di Kalman:
- Kalman converguto + `jitter_ewma < 1000 µs`: divisore dimezzato (8→4), raffiche TSO più grandi
- `jitter_ewma > 4000 µs`: divisore raddoppiato (8→16), raffiche TSO più piccole per sopprimere il jitter

## Tasso di Pacing e Cwnd

### Tasso di Pacing

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // regolazione guadagno
rate = rate × USEC_PER_SEC >> BW_SCALE            // conversione in bytes/s
rate = rate × margin_div / 100                    // margine di pacing (default 1%, matching BBR)
```

Le variazioni di velocità vengono applicate immediatamente (nessun livellamento), come in BBR (Cardwell et al. 2016). Dopo `full_bw_reached`: tutte le variazioni di velocità vengono scritte immediatamente. In STARTUP/DRAIN: vengono applicati solo gli aumenti (`rate > sk_pacing_rate`).

### Cwnd

```
target = BDP(bw, gain, ext)                       // BDP base
// limiti traffico in volo (non-STARTUP: clamp lo~hi; STARTUP: solo pavimento lo)
target = quantization_budget(target)              // margine TSO + round pari + bonus fase-0
target += ack_agg_bonus + agg_compensation        // compensazione aggregazione ACK

// progressione cwnd
if full_bw_reached:
    cwnd = min(cwnd + acked, target)              // convergere al target
else (STARTUP):
    cwnd = cwnd + acked                          // crescita esponenziale

cwnd = max(cwnd, cwnd_min_target)                 // pavimento assoluto 4
modalità PROBE_RTT: cwnd = min(cwnd, cwnd_min_target) // traffico in volo minimo
```

## Parametri del Modulo

I parametri sono esposti sotto `/proc/sys/net/kcc/`. Le scritture attivano `kcc_init_module_params()` (validazione + limitazione + calcolo del valore derivato). Le scritture di parametri array attivano `kcc_rebuild_gain_table()`.

### Intervalli PROBE_RTT

| Parametro | Default | Min | Max | Unità | Descrizione |
|-----------|---------|-----|-----|-------|-------------|
| `kcc_probe_rtt_base_sec` | 10 | 1 | 86400 | s | Intervallo PROBE_RTT base |
| `kcc_probe_rtt_max_sec` | 15 | 1 | 86400 | s | Limite superiore per percorsi RTT lungo |
| `kcc_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | s | Intervallo dinamico max; 0 disabilita |

### Guadagni

| Parametro | Default | Min | Max | Descrizione |
|-----------|---------|-----|-----|-------------|
| `kcc_cwnd_gain_num` / `kcc_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | Guadagno cwnd base per PROBE_BW |
| `kcc_extra_acked_gain_num` / `kcc_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | Moltiplicatore bonus aggregazione ACK |
| `kcc_high_gain_num` / `kcc_high_gain_den` | 2885 / 1000 | 0/1 | 100k | Guadagno STARTUP (≈2,885x) |
| `kcc_drain_gain_num` / `kcc_drain_gain_den` | 347 / 1000 | 0/1 | 100k | Guadagno DRAIN (≈0,347x) |
| `kcc_inflight_low_gain_num` / `kcc_inflight_low_gain_den` | 100 / 100 | 0/1 | 100k | Limite inferiore traffico in volo (1,0x BDP) |
| `kcc_inflight_high_gain_num` / `kcc_inflight_high_gain_den` | 200 / 100 | 0/1 | 100k | Limite superiore traffico in volo (2,0x BDP) |
| `kcc_gain_num[i]` / `kcc_gain_den[i]` | Pattern BBRv1 (256 slot) | 0/1 | — | Guadagno pacing per slot |
| `kcc_cycle_decay_mask[8]` | 0 (tutti zero) | 0 | 0x7FFFFFFF | Bitmap decadimento a 256 bit |
| `kcc_probe_bw_up_limit` | 0 | 0 | 1 | Uscita limitata probe-up (0=spento) |

### Kalman Base

| Parametro | Default | Min | Max | Descrizione |
|-----------|---------|-----|-----|-------------|
| `kcc_kalman_q` | 100 | 0 | 100k | Rumore di processo base Q |
| `kcc_kalman_r` | 400 | 0 | 100k | Rumore di misurazione base R |
| `kcc_kalman_p_est_max` | 1.000.000 | 1 | 100M | p_est massimo assoluto |
| `kcc_kalman_converged_p_est` | 500 | 1 | 1M | Soglia di convergenza |
| `kcc_kalman_p_est_init` | 1000 | 1 | 10M | p_est iniziale |
| `kcc_kalman_p_est_floor` | 10 | 1 | 100k | Pavimento p_est |
| `kcc_kalman_scale` | 1024 | 64 | 1.048.576 | Scala in virgola fissa (potenza di due) |
| `kcc_kalman_min_samples` | 5 | 3 | 20 | Campioni minimi prima della presa in carico |
| `kcc_kalman_outlier_ms` | 5 | 0 | 10000 | ms | Soglia base outlier |
| `kcc_kalman_q_boost_mult` | 4 | 1 | 10000 | Moltiplicatore Q-boost |
| `kcc_kalman_q_boost_ms` | 1 | 0 | 5000 | ms | Costante di tempo Q-boost |
| `kcc_kalman_qboost_cdwn` | 15 | 1 | 255 | samples | Raffreddamento Q-boost |
| `kcc_kalman_q_max` | 2000 | 1 | 100k | Soffitto Q |
| `kcc_kalman_q_scale_cap` | 20 | 1 | 10000 | Limite scala Q |
| `kcc_kalman_max_consec_reject` | 25 | 1 | 1000 | Max rifiuti consecutivi prima di forzare accettazione |
| `kcc_rtt_sample_max_us` | 500000 | 1 | 10M | µs | Soffitto RTT Kalman |
| `kcc_kalman_r_max_boost` | 8 | 1 | 1000 | Moltiplicatore boost max R |
| `kcc_kalman_rtt_dyn_mult` | 2 | 1 | 100 | Moltiplicatore soffitto dinamico RTT |
| `kcc_kalman_q_rtt_div` | 1000 | 1 | 1M | Divisore RTT adattamento Q |
| `kcc_kalman_probe_band_mult` | 4 | 1 | 32 | Moltiplicatore banda transizione PROBE_RTT |

### Kalman Extra (tipo num/den)

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `kcc_kalman_outlier_jitter_mult_num/den` | 4 / 1 | 0-1000 / 1-100k | Moltiplicatore jitter per outlier |
| `kcc_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | Fattore minimo Q |
| `kcc_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | Divisore RTT inizializzazione p_est |

### Stima del Rumore BBR-S

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `kcc_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | Tasso di apprendimento stima Q |
| `kcc_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | Tasso di apprendimento stima R |
| `kcc_kalman_noise_mode` | 1 | 0-2 | Modalità combinazione (0=spento, 1=max, 2=media pesata) |
| `kcc_kalman_q_est_max` | 1.000.000.000 | 1-2 Mld | Limite superiore stima Q |
| `kcc_kalman_r_est_max` | 1.000.000.000 | 1-2 Mld | Limite superiore stima R |
| `kcc_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | Limite inferiore per stima |

### Decadimento del Guadagno (Esplorazione)

| Parametro | Default | Intervallo | Unità | Descrizione |
|-----------|---------|------------|-------|-------------|
| `kcc_qdelay_probe_thresh_us` | 5000 | 0-100k | µs | Soglia decadimento qdelay |
| `kcc_qdelay_probe_scale_us` | 20000 | 1-100k | µs | Scala decadimento qdelay |
| `kcc_jitter_probe_thresh_us` | 4000 | 0-100k | µs | Soglia decadimento jitter |
| `kcc_jitter_probe_scale_us` | 16000 | 1-100k | µs | Scala decadimento jitter |

### R Adattativo (Guidato dal Jitter)

| Parametro | Default | Intervallo | Unità | Descrizione |
|-----------|---------|------------|-------|-------------|
| `kcc_jitter_r_thresh_us` | 2000 | 0-100k | µs | Soglia jitter per aumento R |
| `kcc_jitter_r_scale` | 8000 | 1-100k | — | Divisore scala aumento R |

### ECN

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `kcc_ecn_enable` | 1 | 0-1 | Interruttore principale ECN |
| `kcc_ecn_backoff_num` / `kcc_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | Frazione arretramento ECN |
| `kcc_ecn_qdelay_thresh_us` | 2000 | 0-100k | µs | Soglia qdelay ECN |
| `kcc_ecn_ewma_retained` / `kcc_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | Pesi EWMA ECN |
| `kcc_ecn_idle_decay_num` / `kcc_ecn_idle_decay_den` | 31 / 32 | 1-100k | Decadimento ECN inattivo |

### min_rtt

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `kcc_minrtt_fast_fall_cnt` | 3 | 0-3 | Conteggio caduta rapida |
| `kcc_minrtt_fast_fall_div` | 4 | 1-256 | Divisore soglia caduta rapida |
| `kcc_minrtt_sticky_num` / `kcc_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | Rapporto caduta persistente |
| `kcc_minrtt_srtt_guard_num` / `kcc_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | Rapporto guardia SRTT |

### Larghezza di Banda LT

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `kcc_lt_intvl_min_rtts` | 4 | 1-127 | RTT | Lunghezza intervallo minima |
| `kcc_lt_intvl_max_mult` | 4 | 1-32 | Moltiplicatore timeout intervallo |
| `kcc_lt_loss_thresh` | 15 | 1-65535 | BBR_UNIT | Rapporto perdita minimo |
| `kcc_lt_bw_ratio_num` / `kcc_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | Tolleranza relativa |
| `kcc_lt_bw_diff` | 500 | 0-100k | bytes/s | Tolleranza assoluta |
| `kcc_lt_bw_max_rtts` | 48 | 1-4094 | RTT | RTT attivi max LT BW |
| `kcc_lt_bw_probe_pct` | 10 | 0-100 | % | Boost sonda LT BW |

### Auto-Recupero LT

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `kcc_lt_restore_ratio_num` / `kcc_lt_restore_ratio_den` | 5 / 4 | 0-100k / 1-100k | Rapporto attivazione recupero |
| `kcc_lt_restore_consec_acks` | 3 | 1-31 | Conteggio ACK consecutivi attivazione |

### Confidenza di Aggregazione ACK

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `kcc_agg_enable` | 1 | 0-1 | Interruttore principale |
| `kcc_agg_confidence_thresh` | 512 | 0-10000 | Soglia confidenza compensazione cwnd |
| `kcc_agg_max_comp_ratio` | 75 | 0-100 | % del BDP | Limite compensazione cwnd |
| `kcc_agg_max_comp_duration` | 8 | 1-128 | RTT | Timeout watchdog |
| `kcc_agg_r_hysteresis` | 75 | 0-100 | % | Decadimento isteresi R |
| `kcc_agg_r_multiplier_min` / `kcc_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | Intervallo scala R (256=1x) |
| `kcc_agg_factor3_qdelay_us` | 2000 | 0-100k | µs | Margine qdelay fattore 3 |
| `kcc_agg_factor4_ratio_num` / `kcc_agg_factor4_ratio_den` | 3 / 2 | 1-100k | Rapporto fattore 4 |
| `kcc_agg_safety_qdelay_us` | 4000 | 0-100k | µs | Guardia sicurezza 1 qdelay |
| `kcc_agg_safety_bdp_mult` | 3 | 1-100 | Moltiplicatore BDP guardia sicurezza |
| `kcc_agg_max_window_ms` | 100 | 1-10000 | ms | Finestra limite extra_acked |
| `kcc_agg_max_decay_pct` | 75 | 0-100 | % | Tasso decadimento watchdog |
| `kcc_agg_window_rotation_rtts` | 5 | 1-65535 | RTT | Periodo rotazione finestra |
| `kcc_agg_factor_weight` | 256 | 1-1024 | Punteggio per fattore |
| `kcc_agg_confidence_max` | 1024 | 256-65535 | Confidenza massima |

### Coefficienti EWMA

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `kcc_ewma_qdelay_num` / `kcc_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | Peso EWMA qdelay |
| `kcc_ewma_jitter_num` / `kcc_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | Peso EWMA jitter |

### Vari

| Parametro | Default | Intervallo | Descrizione |
|-----------|---------|------------|-------------|
| `kcc_probe_bw_cycle_len` | 8 | 2-256 | Fasi ciclo PROBE_BW (potenza di due) |
| `kcc_probe_bw_cycle_rand` | 8 | 1-cycle_len | Offset casuale fase ciclo |
| `kcc_full_bw_thresh_num` / `kcc_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | Soglia crescita uscita STARTUP |
| `kcc_full_bw_cnt` | 3 | 1-3 | Round senza crescita per uscire |
| `kcc_probe_rtt_mode_ms_num` / `kcc_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | Durata permanenza PROBE_RTT |
| `kcc_pacing_margin_num` / `kcc_pacing_margin_den` | 1 / 100 | 0-50 / 1-100k | Margine pacing (1 = 1%) |
| `kcc_probe_cwnd_bonus` | 2 | 0-100 | segm | Bonus cwnd fase-0 |
| `kcc_bw_rt_cycle_len` | 10 | 2-256 | round | Lunghezza finestra scorrevole BW |
| `kcc_cwnd_min_target` | 4 | 1-1000 | segm | Cwnd minimo (PROBE_RTT) |
| `kcc_bdp_min_rtt_us` | 1 | 0-100k | µs | Pavimento min_rtt BDP |
| `kcc_edt_near_now_ns` | 1000 | 0-10M | ns | Soglia EDT quasi-ora |
| `kcc_min_tso_rate` | 1.200.000 | 1-1 Mld | bytes/s | Soglia bassa velocità TSO |
| `kcc_min_tso_rate_div` | 8 | 1-256 | Divisore velocità TSO (base adattativa) |
| `kcc_tso_max_segs` | 127 | 1-65535 | segm | Segmenti TSO massimi |
| `kcc_tso_headroom_mult` | 3 | 0-1000 | Moltiplicatore margine TSO |
| `kcc_sndbuf_expand_factor` | 3 | 2-100 | Fattore espansione buffer invio |
| `kcc_ack_epoch_max` | 0xFFFFF | 64K-2G | bytes | Limite epoca ACK |
| `kcc_extra_acked_max_ms_num` / `kcc_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | Finestra max aggregazione ACK |
| `kcc_probe_rtt_decouple` | 1 | 0-1 | Abilitare disaccoppiamento PROBE_RTT (solo modalità FILTER) |
| `kcc_rtt_mode` | 1 | 0-1 | Strategia RTT del Modello: 1=FILTER (Kalman diretto), 0=MIN (limitato) |
| `kcc_recal_p_est_thresh` | 250.000 | 1-100M | Soglia p_est per rete di sicurezza di ricalibrazione |
| `kcc_probe_rtt_long_rtt_us` | 20000 | 0-10M | µs | Soglia RTT lungo |
| `kcc_probe_rtt_long_interval_div` | 1 | 1-1000 | Divisore intervallo RTT lungo |
| `kcc_drain_skip_qdelay_us` | 1000 | 0-100k | µs | Soglia qdelay salto DRAIN |
| `kcc_alone_confirm_rounds` | 3 | 1-32 | round | Round prima di attivare la modalità flusso singolo |
| kcc_alone_qdelay_thresh_us | 1000 | 0-100k | µs | Ritardo di coda max per rilevamento flusso singolo |
| kcc_alone_jitter_thresh_us | 2000 | 0-100k | µs | Jitter max per rilevamento flusso singolo |
| kcc_alone_agg_state_level | 1 | 0-2 | — | Rigore di aggregazione (0=solo IDLE, 1=≤SUSPECTED predef., 2=≤CONFIRMED troppo aggressivo) |
| `kcc_alone_bypass_ecn` | 1 | 0-1 | — | Salta backoff ECN in modalità singola (1=salta, 0=attivo) |
| `kcc_alone_bypass_lt_bw` | 1 | 0-1 | — | Salta condizione LT BW in modalità singola (1=salta, 0=attivo) |

## Percorso Dati

```
ACK Arriva (rate_sample)
    │
    ▼
kcc_main()
    │
    ├──► Pipeline confidenza aggregazione ACK (quando kcc_agg_enable)
    │      misurare → valutare → stato → watchdog
    │      ├── Strato segnale: scalatura R Kalman (sempre attivo)
    │      └── Strato controllo: compensazione cwnd (CONFERMATO+)
    │
    ├──► kcc_update_model()
    │      ├── kcc_update_bw()              BW massima a finestra scorrevole
    │      ├── kcc_update_ecn_ewma()        Rapporto marchio ECN-CE
    │      ├── kcc_update_ack_aggregation()  extra_acked a doppia finestra
    │      ├── kcc_update_cycle_phase()     Avanzamento fase PROBE_BW
    │      ├── kcc_check_full_bw_reached()  Rilevamento uscita STARTUP
    │      ├── kcc_check_drain()            Ingresso/uscita DRAIN + salto DRAIN
    │      ├── kcc_update_min_rtt()         Kalman + finestra min-RTT + PROBE_RTT
    │      └── Assegnazione guadagno specifica modalità
    │
    ├──► kcc_apply_cwnd_constraints()
    │      └── kcc_ecn_backoff()            Arretramento ECN (solo cwnd_gain)
    │
    ├──► kcc_set_pacing_rate()              immediato, regola BBR
    │
    └──► kcc_set_cwnd()                    BDP + limiti + compensazione agg
```

## Flusso Interno del Filtro di Kalman

```
Campione RTT (rtt_us)
    │
    ├── Invalido (≥0 e < dynamic_max)? Sì → scarta
    │
    ├── Avvio a freddo (sample_cnt==0)? Sì → init: x_est=z, p_est=max(p_init, rtt_us/div)
    │                                              (bypassa cancello max RTT)
    │
    ├── Q adattativo: Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   R adattativo: R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── Innovazione: innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && raffreddamento scaduto?
    │   ├── Yes: p_est = p_est_init, cooldown = 15, mark qboost_fired
    │   └── No:  cooldown-- if active
    │
    ├── Predici: p_pred = p_est + Q
    │
    ├── Cancello outlier: |innov| > dyn_thresh && p_pred ≤ converged?
    │   ├── Sì e reject_cnt < max → rifiuta, ++consec_reject_cnt, ritorna
    │   └── Sì e reject_cnt ≥ max → forza accettazione (anti-blocco)
    │
    └── Aggiornamento Kalman:
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov (bloccato non negativo)
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── Aggiornamento EWMA jitter
         ├── Aggiornamento EWMA qdelay
         ├── Stima rumore tramite covarianza accoppiata BBR-S
         └── sample_cnt++
```

## Diagnostica

Interfaccia diagnostica compatibile BBR tramite `ss -i` (`INET_DIAG_BBRINFO`):

```
bbr_bw_lo/bbr_bw_hi: stima larghezza di banda 64 bit (bytes/s)
bbr_min_rtt:         min_rtt_us corrente
bbr_pacing_gain:     guadagno pacing corrente (BBR_UNIT, 256=1,0x)
bbr_cwnd_gain:       guadagno cwnd corrente (BBR_UNIT)
```

## Utilizzo

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

La configurazione dei parametri avviene tramite `/proc/sys/net/kcc/`. Per esempio:
```sh
# Enable gain decay on specific PROBE_BW phases
echo 1 > /proc/sys/net/kcc/kcc_cycle_decay_mask

# Adjust ECN backoff sensitivity
echo 30 > /proc/sys/net/kcc/kcc_ecn_backoff_num
```

## Modello di concorrenza e sicurezza

KCC deliberatamente non usa READ_ONCE/WRITE_ONCE o RCU per le proprie strutture dati. Questo progetto è coerente con tutti i moduli CC intra-kernel come BBR e CUBIC.

`kcc_init()` viene eseguito nel contesto del processo (durante la creazione del socket), prima che il socket sia esposto a qualsiasi softirq. `kcc_release()` viene eseguito dopo che il kernel garantisce che nessun softirq stia ancora elaborando gli ACK di questo socket. Un valore obsoleto transitorio di un parametro globale del modulo influisce al massimo su un ACK, corretto al prossimo ACK.

La sola eccezione: `sk->sk_pacing_rate` / `sk->sk_pacing_shift` sono campi del livello socket che lo spazio utente può modificare simultaneamente tramite `setsockopt`, quindi la convenzione WRITE_ONCE/READ_ONCE di BBR è preservata.

## Riepilogo delle Prestazioni

Ambiente di test: Cina → US LAX, 212ms RTT, 8 flussi paralleli, 26% di perdita pacchetti, 1 Gbps collo di bottiglia VPS condiviso.

| Metrica | KCC v1.0 | BBR (controllo) | Delta |
|--------|----------|---------------|-------|
| Throughput medio | 1.010 Mbps | 937 Mbps | **+7,8%** |
| Disuguaglianza intra-KCC | 3,1× | 6,2× (BBR) | **−50%** |
| Flusso singolo peggiore | 60,6 Mbps | 30,8 Mbps | **+97%** |
| Ritrasmissioni | 150K/10s | 137K/10s | +9,5% |
| Stabilità al round 3 | 959 Mbps | 883 Mbps | **+8,6%** |

Le ritrasmissioni sono leggermente superiori — un compromesso coerente con il mantenimento di un'elevata utilizzazione del collegamento in presenza di perdite. La stima min_rtt potenziata dal filtro di Kalman di KCC fornisce una baseline BDP più accurata, consentendo all'algoritmo di sostenere un throughput maggiore rispetto a BBRv1 sullo stesso percorso.

## Global Kalman BDP — Iniezione di Banda Inter-Connessioni

KCC v1.0 include un filtro di Kalman globale inter-connessioni opzionale che stima la larghezza di banda di collo di bottiglia a regime del server. Questa stima viene utilizzata per inizializzare le nuove connessioni a una « velocità dessert » conservativa — abbastanza veloce da saltare l'avvio a freddo, abbastanza lenta da evitare il superamento.

### Principio di Progettazione

Il filtro è alimentato con campioni di larghezza di banda dalla **fase di crociera** PROBE\_BW (guadagno = 1,0×) di tutte le connessioni KCC. I campioni della fase di crociera sono il segnale più pulito della larghezza di banda realmente disponibile — nessun superamento della sonda a 1,25×, nessun sotto-superamento del drenaggio a 0,75×. Un filtro di Kalman a camminata casuale unidimensionale (Kalman 1960) traccia lo stato stazionario globale.

Quando viene stabilita una nuova connessione, la stima del filtro viene utilizzata per inizializzare:

| Valore iniettato | Scopo |
|----------------|---------|
| `minmax` (max\_bw tracker) | Inizializza lo storico della larghezza di banda a finestra scorrevole in modo che i primi campioni ACK sporchi non lo portino a zero |
| `sk_pacing_rate` | Tasso di pacing iniziale a guadagno neutro (BBR\_UNIT); il guadagno 2,89× di STARTUP viene applicato al primo ACK |
| `tp->snd_cwnd` | Finestra di congestione iniziale calcolata tramite `kcc_bdp()` a guadagno neutro |

Un limite inferiore difensivo in `kcc_update_bw` impedisce ai primi RTT di campioni a basso tasso di consegna di sovrascrivere la stima iniettata durante STARTUP. Un guardiano Full-BW in `kcc_check_full_bw_reached` impedisce allo scambio di messaggi di controllo iperf3 di terminare prematuramente STARTUP.

### Rapporto di Sconto della Velocità Dessert

La velocità di iniezione effettiva è:

```
coeff = (discount_ratio) / high_gain
      = (num / den) / 2.89
```

dove `high_gain ≈ 2.89` è il moltiplicatore di pacing BBR STARTUP.

| num | coeff  | characteristic |
|-----|--------|----------------|
|  35 | 12.1%  | Massima sicurezza, percorso peggiore |
|  50 | 17.3%  | Asse centrale (predefinito) |
|  75 | 25.9%  | Punto ottimale matematico del dessert |
|  80 | 27.6%  | Tetto matematico del tasso (da non superare) |

**Nota:** `tcp_write_xmit` impone un CWND iniziale di `TCP_INIT_CWND` (10 segmenti, ≈15 KB) per ogni nuova connessione. CWND cresce solo quando arrivano ACK remoti, quindi la velocità dessert è un limite superiore al tasso di pacing — il throughput effettivo è limitato da CWND fino a quando non sono stati ricevuti ACK sufficienti per aprire la finestra.

### Configurazione

Abilitazione tramite `sysctl`:

```bash
sysctl -w net.kcc.kcc_kf_enable=1           # master enable (default 0)
sysctl -w net.kcc.kcc_kf_discount_num=50   # dessert-speed numerator (default 50, range 35–75)
```

**Parametri sysctl chiave** (`/proc/sys/net/kcc/`):

| Parametro | Predefinito | Intervallo | Descrizione |
|-----------|---------|-------|-------------|
| `kcc_kf_enable` | 0 | 0–1 | Abilitazione principale per l'iniezione globale Kalman BDP |
| `kcc_kf_discount_num` | 50 | 0–100 | Numeratore della velocità dessert (% della BP in quota equa) |
| `kcc_kf_discount_den` | 100 | 1–100000 | Denominatore della velocità dessert |
| `kcc_kf_startup_r_pct` | 20 | 1–100 | Rumore di misura R% durante la fase di avvio |
| `kcc_kf_steady_r_pct` | 5 | 1–100 | Rumore di misura R% durante il regime stazionario |
| `kcc_kf_q_shift` | 20 | 0–30 | Shift del rumore di processo (Q = 1 << shift) |
| `kcc_kf_chi2_num` | 384 | 1–100000 | Numeratore della soglia outlier chi quadrato |
| `kcc_kf_chi2_den` | 100 | 1–100000 | Denominatore della soglia outlier chi quadrato |

### Prestazioni al Primo Secondo (Trans-Pacifico, 212 ms RTT)

```
Without KF:  2.8 Mbps  →  85 Mbps  →  622 Mbps  →  steady
With KF:     50 Mbps   →  530 Mbps  →  650 Mbps  →  steady
```

La velocità al primo secondo salta da ~3 Mbps (avvio a freddo) a ~50 Mbps (avvio dessert), e la convergenza allo stato stazionario viene raggiunta entro 2–3 secondi. Le ritrasmissioni rimangono a zero per tutto il tempo.

### Come Funziona

1. Una connessione KCC in esecuzione entra nella fase di crociera PROBE\_BW → confine di inizio round → alimenta `kcc_kf_update(bw, 5%)` con il campione corrente del tasso di consegna.
2. Il filtro di Kalman aggiorna la sua stima `kcc_kf_x` (una media mobile della larghezza di banda di collo di bottiglia a regime).
3. Quando una **nuova** connessione si apre, `kcc_init` chiama `kcc_kf_get_init_bw(sk)` che restituisce `fair × discount / high_gain` — una stima della larghezza di banda iniziale compensata in guadagno e in quota equa.
4. Questa stima inizializza `sk_pacing_rate`, `tp->snd_cwnd` e il tracciatore di larghezza di banda `minmax` — la connessione parte alla velocità dessert anziché da zero.

### Fonte dell'Algoritmo

Il filtro Global Kalman BDP si basa sull'articolo dell'autore *Sulla stima di Kalman e l'implementazione ingegneristica della larghezza di banda globale a regime nel kernel Linux* (CC BY-SA 4.0):
https://blog.csdn.net/liulilittle/article/details/161635652

---

*KCC v1.0 — costruito su BBRv1 (Cardwell et al. 2016, ACM Queue) e il filtro di Kalman (Kalman 1960).*

## Riferimenti

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