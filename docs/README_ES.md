[🇺🇸 English](../README.md) | [🇨🇳 中文](README_CN.md) | [🇹🇼 繁體中文](README_TW.md) | [🇪🇸 Español](README_ES.md) | [🇫🇷 Français](README_FR.md) | [🇷🇺 Русский](README_RU.md) | [🇸🇦 العربية](README_AR.md) | [🇩🇪 Deutsch](README_DE.md) | [🇯🇵 日本語](README_JA.md) | [🇰🇷 한국어](README_KO.md) | [🇮🇹 Italiano](README_IT.md) | [🇵🇹 Português](README_PT.md)

---

# TCP KCC v1.0 (Control de Congestión Kalman)

Módulo de control de congestión TCP para entornos VPS de ancho de banda compartido que combina la máquina de estados de BBRv1 con un filtro de Kalman para la estimación del retardo de propagación.

## Principios de Diseño

Los algoritmos de control de congestión deben equilibrar el rendimiento, la latencia, la equidad y la tolerancia a pérdidas. KCC adopta un enfoque pragmático:

1. BBRv1 proporciona una base probada. Máquina de estados, pacing, ganancias de ciclo, STARTUP/DRAIN/PROBE_BW/PROBE_RTT — KCC adopta estos mecanismos sin modificación.

2. El filtro de Kalman mejora la precisión de la estimación. Separar el retardo de propagación real del retardo de cola y la fluctuación produce una estimación min_rtt más precisa, permitiendo un cálculo de BDP más ajustado, un CWND mejor calibrado y un pacing más estable.

3. La dinámica entre algoritmos sigue el equilibrio competitivo TCP estándar. KCC no limita artificialmente su tasa de envío en respuesta a la cola detectada de flujos externos. La reducción de ganancia (disminución de sonda basada en cola) está disponible como opción mediante kcc_cycle_decay_mask pero deshabilitada por defecto para preservar la intensidad completa de la sonda.

4. La equidad intra-KCC se mantiene activamente. La convergencia de Kalman asegura que los flujos KCC en el mismo host compartan una estimación min_rtt consistente, eliminando el bucle de retroalimentación del ganador-se-lleva-todo que causa una inequidad severa en implementaciones BBR puras de múltiples flujos.

## Resumen del Algoritmo

TCP KCC implementa un módulo de control de congestión del lado del emisor para el kernel de Linux como un módulo cargable `tcp_kcc.ko`. La función de control de congestión `kcc_main()` se invoca en cada ACK desde `tcp_ack()`, recibiendo una estructura `rate_sample` que contiene muestras de ancho de banda y RTT del kernel junto con contadores de entrega y pérdida. El algoritmo opera en dos regímenes temporales: un **camino rápido por-ACK** que actualiza el estado de medición y calcula objetivos instantáneos de pacing y ventana, y un **camino lento por-ronda** que evalúa condiciones de transición de estado y recalcula ganancias.

El pipeline de medición central consta de dos componentes:

1. **Filtro de ancho de banda máximo de ventana deslizante** (`minmax_running_max` de `linux/win_minmax.h`): ventana que cubre las últimas `kcc_bw_rt_cycle_len` (por defecto 10) rondas. Proporciona la estimación `max_bw` compatible con BBR.

2. **Estimador de retardo de propagación por filtro de Kalman**: reemplaza el RTT mínimo de ventana deslizante de BBRv1 y es la fuente predeterminada para la estimación de RTT del BDP (ver [Estrategia de RTT del Modelo](#estrategia-de-rtt-del-modelo)). Un filtro de Kalman de estado único (Kalman 1960) que opera en unidades de punto fijo de `kcc_kalman_scale` × µs, modelando el retardo de propagación real como un camino aleatorio:
   - Estado: `x[k] = x[k−1] + w[k]`, `w ~ N(0, Q)`
   - Observación: `z[k] = x[k] + v[k]`, `v ~ N(0, R)`

Convenciones de punto fijo: `BW_UNIT = 1 << 24` para ancho de banda (segmentos * 2^24 / µs), `BBR_UNIT = 1 << 8 = 256` como unidad de ganancia adimensional.

## Estrategia de RTT del Modelo

KCC introduce una estrategia configurable para la estimación de RTT utilizada en el cálculo del BDP (Producto Ancho de Banda-Retardo), controlada por `kcc_rtt_mode`:

| Modo | Valor | Comportamiento | Caso de uso |
|------|-------|----------------|-------------|
| FILTER | 1 (predeterminado) | Uso directo de `x_est_us` — la estimación bruta del filtro de Kalman/ventana deslizante | WAN/VPS de producción: resistente a cambios de ruta, sin caída de rendimiento |
| MIN | 0 | `min(x_est_us, min_rtt_us)` — limitar la estimación de Kalman contra el mínimo de ventana | Verificación de estabilidad del módulo del kernel; enlaces de RTT estático |

**Por qué FILTER es el predeterminado:**

- **Resiliencia a cambios de ruta**: Cuando un rerouteo BGP aumenta el RTT físico (ej. 50 ms → 100 ms), la ganancia de Kalman K_k reacciona en pocos RTT y ajusta la estimación a la nueva latencia. El modo MIN se bloquea en el antiguo `min_rtt_us` hasta que expire la ventana, dividiendo el BDP por la mitad.

- **Defensas integradas**: La compuerta de valores atípicos descarta muestras de pico de cola antes de que ingresen al filtro. La estimación adaptativa de ruido Q/R reduce la ganancia de Kalman cuando la red es ruidosa, por lo que el filtro naturalmente desconfía del bloqueo de cola transitorio y mantiene la estimación cerca del verdadero retardo de propagación.

- **Desacoplamiento de PROBE_RTT**: El modo FILTER activa la función `kcc_probe_rtt_decouple` — el filtro de Kalman sigue el piso de RTT sin necesidad del vaciado periódico de 4 paquetes.

Conmutación en tiempo de ejecución: `echo 0 > /proc/sys/net/kcc/kcc_rtt_mode` para volver al modo MIN.

## Máquina de Estados

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

Cuatro modos codificados como el campo `mode` de 2 bits en `struct KCC`:

- **STARTUP (0)**: Estado inicial. `pacing_gain` ≈ 2.885x (`kcc_high_gain_val`), `cwnd_gain` también 2.885x. Sondeo exponencial de ancho de banda.
- **DRAIN (1)**: Ingresado después de la salida de STARTUP. `pacing_gain` ≈ 0.347x (`kcc_drain_gain_val`), `cwnd_gain` permanece en 2.885x. Drena la cola acumulada durante STARTUP.
- **PROBE_BW (2)**: Estado estable. Cicla a través de una tabla de ganancias de 256 ranuras (patrón de 8 fases por defecto repetido: 1.25x/0.75x/8×1.0x).
- **PROBE_RTT (3)**: Drena periódicamente el tráfico en vuelo a `kcc_cwnd_min_target` (por defecto 4 segmentos) para obtener una muestra RTT fresca.

### STARTUP → DRAIN

Se activa cuando `full_bw_reached` se establece — después de `kcc_full_bw_cnt` (por defecto 3) rondas consecutivas donde `max_bw` no logra crecer al menos `kcc_full_bw_thresh_val` (por defecto 1.25x) en comparación con el pico observado previamente. El BDP con ganancia 1.0x se escribe en `snd_ssthresh`. `qdelay_avg` se reinicia a cero para evitar que la acumulación de la cola de STARTUP afecte a PROBE_BW.

### DRAIN → PROBE_BW

Se activa cuando el tráfico en vuelo estimado a EDT ≤ tráfico en vuelo objetivo con ganancia BDP de 1.0x. **Optimización de salto de DRAIN**: cuando el filtro de Kalman está convergido Y `qdelay_avg < el umbral dinámico limpio` (10% de min_rtt_us con piso de 500us), la fase DRAIN se omite — se convierte temprano a PROBE_BW.

Al ingresar a PROBE_BW, el índice de fase del ciclo se aleatoriza: `cycle_idx = len − 1 − rand(kcc_probe_bw_cycle_rand)` (por defecto `len − 1 − rand(8)`), lo que descorrelaciona flujos concurrentes que comparten un enlace congestionado.

### PROBE_BW → PROBE_RTT

Se activa cuando el intervalo del filtro PROBE_RTT expira — la marca de tiempo `min_rtt_stamp` no se ha actualizado dentro del intervalo calculado. cwnd se guarda en `prior_cwnd`, el pacing se establece para drenar.

### PROBE_RTT → PROBE_BW

Después de que el tráfico en vuelo cae a `kcc_cwnd_min_target` o se observa un límite de ronda, persiste durante al menos `kcc_probe_rtt_mode_ms_val` (por defecto 200 ms) y al menos una ronda completa observada, luego sale. cwnd se restaura al menos a `prior_cwnd`, el pacing se sobrescribe temporalmente con `kcc_high_gain_val` para un relleno rápido de tubería.

### Recuperación y Pérdida

- En TCP_CA_Loss: `full_bw` y `full_bw_cnt` se reinician, `round_start` se establece en 1, `packet_conservation` se limpia a 0.
- Entrada de recuperación (TCP_CA_Recovery): `packet_conservation` habilitado, cwnd = en vuelo + acusado.
- Salida de recuperación: se restaura a `prior_cwnd`, `packet_conservation` limpio.
- `kcc_undo_cwnd()`: reinicia `full_bw` y `full_bw_cnt` (preservando `full_bw_reached`), limpia el estado de LT BW.

### Detección de ronda (Alineación BBR)

Los límites de ronda se detectan según BBR, Cardwell et al. 2016: cuando `prior_delivered` alcanza o supera `next_rtt_delivered` mediante una comparación sin signo `!before()`. `next_rtt_delivered` se inicializa a `0` — igual que BBR estándar — por lo que el primer ACK inicia inmediatamente la ronda 1, independientemente de la entrega de segmentos de handshake. La validación de muestras de tasa usa `interval_us <= 0` (no `== 0`) para coincidir con la guarda exacta de BBR, detectando intervalos negativos.

- `next_rtt_delivered` inicializado a `0` (paridad BBR): la primera ronda comienza en el primer ACK.
- Validación `interval_us <= 0`: coincide exactamente con BBR, rechaza intervalos negativos.
- `round_start` se reinicia a `0` al inicio de `kcc_update_bw()`, antes de la verificación de validación — coincidiendo con la colocación `bbr->round_start = 0` de BBR.

## Mediciones Centrales

### Estimación de Ancho de Banda

Filtro de ancho de banda máximo de ventana deslizante (`minmax_running_max` de `linux/win_minmax.h`) sobre `kcc_bw_rt_cycle_len` (por defecto 10) rondas. bw instantáneo = `delivered × BW_UNIT / interval_us` calculado por ACK. Se alimenta a la ventana deslizante solo cuando no está limitado por aplicación o cuando bw ≥ bw máximo actual (regla BBR).

Cuando `lt_use_bw` está activo, la estimación de ancho de banda activa cambia a `lt_bw` (estimación de ancho de banda a largo plazo).

### Filtro de Kalman

Recursión de Kalman escalar de estado único (complejidad O(1)):

```
Predecir:
  x_pred = x_est          (transición de estado identidad)
  p_pred = p_est + Q      (predicción de covarianza)

Actualizar:
  innov   = z − x_pred    (innovación)
  K       = p_pred / (p_pred + R)   (ganancia de Kalman [0,1])
  x_est   = x_pred + K × innov      (actualización de estado)
  p_est   = (1 − K) × p_pred        (covarianza posterior)
```

**Ruido de proceso adaptativo Q**:
```
Q_base   = kcc_kalman_q (por defecto 100)
q_factor = max(kcc_kalman_q_min_factor_val, min_rtt_us / kcc_kalman_q_rtt_div)
Q        = min(Q_base × q_factor, Q_base × kcc_kalman_q_scale_cap)
Q        = min(Q, kcc_kalman_q_max)
```

**Ruido de medición adaptativo R**:
```
R = R_base + max(0, jitter_ewma − clean_thresh) × R_base / kcc_jitter_r_scale
R = min(R, R_base × kcc_kalman_r_max_boost)
```

**Detección de cambio de ruta Q-Boost (con compuerta de confianza + enfriamiento)**: cuando `|innovation| > kcc_kalman_q_boost_thresh_val` (predeterminado ≈ 4 ms de desplazamiento RTT) Y el filtro ha convergido (`p_est ≤ kcc_kalman_converged_p_est_val`, predeterminado 500), `p_est` se restablece a `kcc_kalman_p_est_init_val`, aumentando la ganancia de Kalman hacia 1.0 para una convergencia rápida. Un enfriamiento de `kcc_kalman_qboost_cdwn` (predeterminado 15) muestras entre eventos qboost sucesivos evita la activación descontrolada en rutas con pérdidas y alta fluctuación RTT.

**Compuerta de valores atípicos**: umbral dinámico `dyn_thresh = max(outlier_ms × 1000 × scale, jitter_ewma × outlier_jitter_mult × scale)`. Se aplica solo cuando `p_pred ≤ kcc_kalman_converged_p_est_val`. Después de `kcc_kalman_max_consec_reject` (por defecto 25) rechazos consecutivos, la siguiente muestra se fuerza a aceptar para evitar un bloqueo auto-reforzante.

**Estimación de ruido por covarianza coincidente (BBR-S)**: `q_est = (1−α) × q_est + α × (K × innov)²`, `r_est = (1−β) × r_est + β × max(0, innov² − p_pred)`. Modo de combinación: modo 0 = solo heurístico, modo 1 = max (por defecto), modo 2 = mezcla ponderada.

**Toma de control de Kalman**: cuando `x_est > 0` y `sample_cnt ≥ kcc_kalman_min_samples` (por defecto 5), `min_rtt_us` se reemplaza por `x_est / kcc_kalman_scale`. `min_rtt_stamp` no se actualiza — el disparador del intervalo PROBE_RTT permanece independiente.

**Estrategia de RTT del Modelo**: La estimación de RTT utilizada para el cálculo del BDP es controlada por `kcc_rtt_mode`. En modo FILTER (predeterminado), se usa `model_rtt = x_est_us` directamente — la estimación de Kalman/ventana deslizante sin limitación. En modo MIN, `model_rtt = min(x_est_us, min_rtt_us)` — la estimación de Kalman se limita contra el mínimo de ventana para garantizar que el BDP nunca se infla. El valor predeterminado FILTER se recomienda para implementaciones de producción WAN/VPS donde la latencia de la ruta puede cambiar abruptamente (rerouteos BGP, traspasos LEO, cambios de celda móvil). Ver [Estrategia de RTT del Modelo](#estrategia-de-rtt-del-modelo).

## Mejoras BBR

### Decaimiento de Ganancia

Habilitado por el mapa de bits de 256 bits `kcc_cycle_decay_mask[]` para fases específicas de PROBE_BW. Fórmula de decaimiento (en muestra de Kalman aceptada):

```
max_red       = probe_gain − BBR_UNIT
conf_scale    = escalado inverso de p_est (BBR_UNIT a pleno)
qdelay_decay  = min(max(0, qdelay_avg − qthresh) × BBR_UNIT / qscale, max_red)
                     × conf_scale / BBR_UNIT
jitter_decay  = min(max(0, jitter_ewma − jthresh) × BBR_UNIT / jscale, remaining)
                     × conf_scale / BBR_UNIT
effective     = max(probe_gain − qdelay_decay − jitter_decay, BBR_UNIT)
```

Escalado de confianza de Kalman: cuando `p_est > kcc_kalman_converged_p_est`, el decaimiento se reduce proporcionalmente, evitando una retroceso excesivo cuando el filtro es incierto.

### Retroceso ECN

Condiciones de activación (todas deben cumplirse):
1. `kcc_ecn_enable_val != 0`
2. Kalman convergido (`p_est < converged`, `sample_cnt >= min_samples`)
3. `ecn_ewma > 0` (marcas CE observadas)
4. `qdelay_avg > el umbral dinámico de congestión` (25% de min_rtt_us con piso de 500us)
5. El modo NO es PROBE_BW (cwnd_gain es fijo en 2x en PROBE_BW)

Durante las fases de sondeo (`pacing_gain > BBR_UNIT`), el retroceso ECN es graduado por `BBR_UNIT² / pacing_gain` — ~80% de retroceso en sonda 1.25x, ~65% en ganancia STARTUP 2.89x.

Relación de marca ECN EWMA: se actualiza en los límites de ronda por `kcc_ecn_ewma_retained / kcc_ecn_ewma_total` (por defecto 3/4), con decaimiento suave por-ACK de `kcc_ecn_idle_decay_num / kcc_ecn_idle_decay_den` (por defecto 31/32) en cada ACK sin nuevas marcas CE.

### Detección de Flujo Único

Cuando KCC detecta que el flujo probablemente está solo en el cuello de botella (bajo retardo de cola, baja fluctuación, sin marcas ECN, sin agregación de ACK), realiza una transición automática a un modo BBR puro:

- `kcc_get_model_rtt()` devuelve `min_rtt_us` directamente (evitando la estimación suavizada de Kalman, que tiene un pequeño sesgo positivo debido al ruido de medición unilateral).
- `kcc_ecn_backoff()` se puede configurar mediante `kcc_alone_bypass_ecn` (predeterminado 1) — en una ruta de flujo único, las marcas ECN son falsos positivos del AQM porque no hay otro emisor compitiendo. Saltarlo iguala el comportamiento ECN cero de BBR. Configúrelo en 0 para mantener el backoff ECN incluso en modo individual (conservador).

Esto elimina la brecha de rendimiento en flujo único entre KCC y BBR, preservando al mismo tiempo el bucle de protección completo de KCC (Kalman, retroceso ECN, decaimiento de ganancia) para escenarios de múltiples flujos.

**Histéresis**: La entrada requiere `kcc_alone_confirm_rounds` (predeterminado 3) rondas consecutivas calificadas — evitando oscilaciones durante breves períodos de calma en la competencia de múltiples flujos ("conservador para acelerar"). Salida: durante la evaluación en fase de crucero, cualquier fallo de calificación único borra la bandera ("agresivo para frenar").

**Compensación de diseño**: La pérdida de paquetes NO se utiliza como descalificador de flujo único — algunos enlaces (búferes poco profundos, inalámbricos, caídas de ráfaga de virtualización) tienen pérdidas inherentes no relacionadas con la competencia. Equiparar la pérdida con la competencia de múltiples flujos provoca oscilaciones en rutas de flujo único. La señal LT BW (detección de policer activada por pérdidas de BBR) no participa en el juicio de flujo único.

**Gain gating**: la evaluación de flujo único se ejecuta solo durante la fase de crucero (`pacing_gain == BBR_UNIT`). El probe-up (1.25x) presiona intencionalmente el cuello de botella — su presión de cola es autoinducida y no una señal de competencia. El drenaje (0.75x) suprime artificialmente la cola. Al limitar la evaluación a crucero (el equilibrio estacionario), la presión de probe-up autoinducida ya no causa falsas salidas del modo individual.

Condiciones de calificación (las cinco deben cumplirse en un límite de ronda):
0. Kalman convergido (`sample_cnt >= kcc_kalman_min_samples`) — confiar en qdelay/jitter como señales de cola
1. `qdelay_avg < el umbral dinámico limpio` — cola casi vacía
2. `jitter_ewma < el umbral dinámico de congestión` — solo micro-fluctuación de reloj ACK
3. `ecn_ewma == 0` — sin marcas de congestión de AQM
4. `agg_state <= max` según `kcc_alone_agg_state_level` (predeterminado 1) — tres niveles de rigor de agregación ACK: 0 = solo IDLE (más estricto, cero agregación), 1 = ≤ SUSPECTED (predeterminado, permite agregación transitoria), 2 = ≤ CONFIRMED (más permisivo, solo bloquea agregación persistente)

### Intervalo PROBE_RTT Dinámico

Mapa `p_est` de Kalman a un intervalo PROBE_RTT por conexión:

```
p_est ≤ converged:              interval = dyn_max (por defecto 30s)
p_est ≥ high (= mult × conv):   interval = base (por defecto 10s)
converged < p_est < high:       interpolación lineal
```

Reduce la frecuencia de PROBE_RTT cuando la confianza es alta (`p_est` bajo), reduciendo la fluctuación de rendimiento en rutas estables. Vuelve al intervalo clásico de 10 segundos cuando la confianza es baja.

**Jitter de entrada por flujo**: Para evitar que todos los flujos coexistentes entren en PROBE_RTT simultáneamente (drenando a 4 paquetes agregados ~1.8 Mbps y luego recargando a 2.89×), cada flujo agrega un jitter derivado de hash (distribución de 0–845 ms) a su intervalo PROBE_RTT. Como máximo ~1 flujo está en PROBE_RTT en cualquier instante, eliminando el colapso simultáneo de drenaje/recarga que induce RTO.

### Desacoplamiento de PROBE_RTT y recalibración inteligente

El mecanismo PROBE_RTT de BBRv1 drena el tubo a 4 paquetes cada ~10 segundos para medir `min_rtt_us`. Esto es necesario para un estimador min-RTT basado en ventana — la ventana no puede distinguir el retardo de propagación del retardo de cola a menos que el tubo esté vacío. El costo es una caída periódica de rendimiento (la "sierra" de BBR).

En modo FILTER, el filtro de Kalman reemplaza completamente la ventana. Puede separar el ruido de cola del verdadero retardo de propagación mediante la compuerta de valores atípicos y la estimación adaptativa de ruido — no se requiere drenaje del tubo. El parámetro `kcc_probe_rtt_decouple` (predeterminado 1) controla esto:

| Modo | Valor | Comportamiento |
|------|-------|----------------|
| Desacoplado | 1 (predeterminado) | **Kalman saludable** (p_est ≤ `kcc_recal_p_est_thresh`): suprimir PROBE_RTT completamente → cero caídas de rendimiento, cero colapsos sincrónicos. **Kalman divergido** (p_est > umbral): activar automáticamente PROBE_RTT tradicional como red de seguridad → restaura la línea base del filtro, luego el desacoplamiento se reanuda. |
| Tradicional | 0 | PROBE_RTT periódico ciego cada ~10s (compatible con BBR). |

**Heurística de recalibración inteligente** (`kcc_kalman_needs_recalibration()`): En operación estable en una ruta estable, la covarianza de error p_est de Kalman converge a p_est_floor (~4–10), muy por debajo del umbral `kcc_recal_p_est_thresh` (250,000 = 25% de p_est_max). Un p_est creciente señala que el modelo interno del filtro ya no explica las observaciones — típicamente porque la ruta ha cambiado materialmente. Cuando p_est excede el umbral, un único drenaje PROBE_RTT tradicional restaura la línea base del filtro; el Kalman reconverge y el desacoplamiento se reanuda automáticamente.

Esto transforma PROBE_RTT **de una automutilación periódica ciega** en **una recalibración inteligente basada en confianza** — el protocolo solo drena el tubo cuando tiene evidencia empírica de que el filtro ha perdido la confianza.

Requiere `kcc_rtt_mode == 1`. Sin efecto en modo MIN (el modo MIN depende de PROBE_RTT para actualizar `min_rtt_us`).

| Parámetro | Predeterminado | Rango | Descripción |
|-----------|----------------|-------|-------------|
| `kcc_probe_rtt_decouple` | 1 | 0–1 | Activar desacoplamiento PROBE_RTT (solo modo FILTER) |
| `kcc_recal_p_est_thresh` | 25000 | 1–100,000,000 | Umbral p_est para red de seguridad de recalibración |

### Estimación de Ancho de Banda LT

Estimador de límite inferior activado por pérdida. El intervalo de muestreo abarca [4, 16] RTTs. Válido cuando la relación de pérdida ≥ 5.9% (`kcc_lt_loss_thresh` por defecto 15/256). Ancho de banda `bw = delivered × BW_UNIT / interval_us`.

A diferencia del promedio simple de BBR (`(bw + lt_bw) >> 1`), KCC usa un EMA configurable (`kcc_lt_bw_ema_num / kcc_lt_bw_ema_den`, por defecto 1/2 = 0.5):

```
lt_bw = (bw_new × en + lt_bw × (ed − en)) / ed
```

La activación difiere de BBR: KCC almacena `lt_bw` en el primer intervalo válido pero NO establece `lt_use_bw`; se requiere consistencia con un intervalo anterior — reduce la activación falsa por ruido de medición.

**Compuerta de congestión de doble umbral**: Antes de establecer `lt_use_bw = 1`, se evalúan tanto una verificación de cola EWMA persistente (`qdelay_avg > el umbral dinámico de congestión`) como una verificación de cola instantánea basada en SRTT (`srtt_us − min_rtt_us > el umbral instantáneo de congestión`, predeterminado 5000 µs). Cuando se detecta congestión, el muestreo LT BW se aborta. La verificación SRTT funciona sin asignación `ext`, proporcionando una red de seguridad contra fallas de asignación.




### Compensación Basada en Confianza de Agregación ACK (inspirado en BBRplus)

Agrega una segunda capa con compuerta de confianza sobre el estimador tradicional de doble ranura extra-acked.

**Cuatro factores ortogonales** (cada uno contribuye `kcc_agg_factor_weight` puntos, por defecto 256):
1. Kalman convergido (`p_est < converged` + `sample_cnt >= min_samples`)
2. No en recuperación de pérdida (`icsk_ca_state < TCP_CA_Recovery`)
3. RTT dentro de `min_rtt_us + el umbral dinámico limpio` del retardo de propagación real
4. `extra_acked` dentro de `kcc_agg_factor4_ratio_num/den` (por defecto 1.5x) del máximo ventaneado

**Cuatro estados**: IDLE (< `kcc_agg_thresh_suspected`=256), SOSPECHOSO (≥256), CONFIRMADO (≥512), CONFIABLE (≥768).

**Capa de señal** (siempre activa): la confianza interpola linealmente el factor de escalado R `[r_min, r_max]`. R sube instantáneamente (respuesta rápida), decae a `kcc_agg_r_hysteresis`% (por defecto 75% retenido, ~4 RTTs a la línea base) por RTT.

**Capa de control** (`agg_state ≥ CONFIRMED`): compensación de cwnd con compuerta de seguridad de cinco capas:
1. Bloquea si el retardo de cola > `el umbral dinámico de congestión`
2. Bloquea durante la recuperación de pérdida
3. Bloquea si cwnd > `BDP × kcc_agg_safety_bdp_mult` (por defecto 3x)
4. Bloquea si en vuelo > cwnd seguro + objetivo de segmentos TSO
5. Vigilante: degrada CONFIRMADO→SOSPECHOSO después de `kcc_agg_max_comp_duration` (por defecto 8) RTTs consecutivos

### Reinicio de qdelay_avg en DRAIN

En la transición a DRAIN, `qdelay_avg` se reinicia a cero, evitando que la estimación de cola de STARTUP persista en PROBE_BW.

### Adaptación del Divisor TSO

`kcc_min_tso_segs()` ajusta el divisor del umbral de tasa basado en el estado de Kalman:
- Kalman convergido + `jitter_ewma < 1000 µs`: divisor reducido a la mitad (8→4), ráfagas TSO más grandes
- `jitter_ewma > 4000 µs`: divisor duplicado (8→16), ráfagas TSO más pequeñas para suprimir la fluctuación

## Tasa de Pacing y Cwnd

### Tasa de Pacing

```
rate = bw × mss × pacing_gain >> BBR_SCALE      // ajuste de ganancia
rate = rate × USEC_PER_SEC >> BW_SCALE            // convertir a bytes/s
rate = rate × margin_div / 100                    // margen de pacing (por defecto 1%, matching BBR)
```

Los cambios de tasa se aplican inmediatamente (sin suavizado), coincidiendo con BBR (Cardwell et al. 2016). Después de `full_bw_reached`: todos los cambios de tasa se escriben inmediatamente. En STARTUP/DRAIN: solo se aplican aumentos (`rate > sk_pacing_rate`).

### Cwnd

```
target = BDP(bw, gain, ext)                       // BDP base
target = quantization_budget(target)              // espacio libre TSO + ronda par + bonificación fase-0
target += ack_agg_bonus + agg_compensation        // compensación de agregación ACK

// progresión de cwnd
if full_bw_reached:
    cwnd = min(cwnd + acked, target)              // converger al objetivo
else (STARTUP):
    cwnd = cwnd + acked                          // crecimiento exponencial

cwnd = max(cwnd, cwnd_min_target)                 // piso absoluto 4
modo PROBE_RTT: cwnd = min(cwnd, cwnd_min_target) // tráfico en vuelo mínimo
```

## Parámetros del Módulo

Los parámetros se exponen bajo `/proc/sys/net/kcc/`. Las escrituras activan `kcc_init_module_params()` (validación + sujeción + cálculo de valor derivado). Las escrituras de parámetros de matriz activan `kcc_rebuild_gain_table()`.

### Intervalos PROBE_RTT

| Parámetro | Por defecto | Mín | Máx | Unidad | Descripción |
|-----------|-------------|-----|-----|--------|-------------|
| `kcc_probe_rtt_base_sec` | 10 | 1 | 86400 | s | Intervalo base PROBE_RTT |
| `kcc_probe_rtt_max_sec` | 15 | 1 | 86400 | s | Límite superior para rutas de RTT largo |
| `kcc_probe_rtt_dyn_max_sec` | 30 | 0 | 86400 | s | Intervalo dinámico máximo; 0 deshabilita |

### Ganancias

| Parámetro | Por defecto | Mín | Máx | Descripción |
|-----------|-------------|-----|-----|-------------|
| `kcc_cwnd_gain_num` / `kcc_cwnd_gain_den` | 2 / 1 | 0/1 | 100k | Ganancia de cwnd base para PROBE_BW |
| `kcc_extra_acked_gain_num` / `kcc_extra_acked_gain_den` | 1 / 1 | 0/1 | 100k/100k | Multiplicador de bonificación de agregación ACK |
| `kcc_high_gain_num` / `kcc_high_gain_den` | 2885 / 1000 | 0/1 | 100k | Ganancia STARTUP (≈2.885x) |
| `kcc_drain_gain_num` / `kcc_drain_gain_den` | 347 / 1000 | 0/1 | 100k | Ganancia DRAIN (≈0.347x) |
| `kcc_gain_num[i]` / `kcc_gain_den[i]` | Patrón BBRv1 (256 ranuras) | 0/1 | — | Ganancia de pacing por ranura |
| `kcc_cycle_decay_mask[8]` | 0 (todos cero) | 0 | 0x7FFFFFFF | Mapa de bits de decaimiento de 256 bits |
| `kcc_probe_bw_up_limit` | 0 | 0 | 1 | Salida limitada de sondeo ascendente (0=apagado) |

### Kalman Base

| Parámetro | Por defecto | Mín | Máx | Descripción |
|-----------|-------------|-----|-----|-------------|
| `kcc_kalman_q` | 100 | 0 | 100k | Ruido de proceso base Q |
| `kcc_kalman_r` | 400 | 0 | 100k | Ruido de medición base R |
| `kcc_kalman_p_est_max` | 1,000,000 | 1 | 100M | p_est máximo absoluto |
| `kcc_kalman_converged_p_est` | 500 | 1 | 1M | Umbral de convergencia |
| `kcc_kalman_p_est_init` | 1000 | 1 | 10M | p_est inicial |
| `kcc_kalman_p_est_floor` | 10 | 1 | 100k | p_est mínimo |
| `kcc_kalman_scale` | 1024 | 64 | 1,048,576 | Escala de punto fijo (potencia de dos) |
| `kcc_kalman_min_samples` | 5 | 3 | 20 | Muestras mínimas antes de toma de control |
| `kcc_kalman_outlier_ms` | 5 | 0 | 10000 | ms | Umbral base de valores atípicos |
| `kcc_kalman_q_boost_mult` | 4 | 1 | 10000 | Multiplicador Q-boost |
| `kcc_kalman_q_boost_ms` | 1 | 0 | 5000 | ms | Constante de tiempo Q-boost |
| `kcc_kalman_qboost_cdwn` | 8 | 1 | 255 | samples | Enfriamiento Q-boost |
| `kcc_kalman_q_max` | 2000 | 1 | 100k | Techo Q |
| `kcc_kalman_q_scale_cap` | 50 | 1 | 10000 | Límite de escala Q |
| `kcc_kalman_max_consec_reject` | 25 | 1 | 1000 | Máximo de rechazos consecutivos antes de forzar aceptación |
| `kcc_rtt_sample_max_us` | 500000 | 1 | 10M | µs | Techo de RTT de Kalman |
| `kcc_kalman_r_max_boost` | 8 | 1 | 1000 | Multiplicador de impulso máximo R |
| `kcc_kalman_rtt_dyn_mult` | 2 | 1 | 100 | Multiplicador de techo dinámico RTT |
| `kcc_kalman_q_rtt_div` | 1000 | 1 | 1M | Divisor RTT de adaptación Q |
| `kcc_kalman_probe_band_mult` | 4 | 1 | 32 | Multiplicador de banda de transición PROBE_RTT |

### Kalman Extras (tipo num/den)

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `kcc_kalman_outlier_jitter_mult_num/den` | 3 / 1 | 0-1000 / 1-100k | Multiplicador de fluctuación para valores atípicos |
| `kcc_kalman_q_min_factor_num/den` | 10 / 1 | 0-1000 / 1-100k | Factor mínimo Q |
| `kcc_kalman_p_est_init_rtt_div_num/den` | 10 / 1 | 1-100k / 1-100k | Divisor RTT de inicialización p_est |

### Estimación de Ruido BBR-S

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `kcc_kalman_noise_alpha_num/den` | 1 / 10 | 0-100 / 1-100k | Tasa de aprendizaje de estimación Q |
| `kcc_kalman_noise_beta_num/den` | 1 / 10 | 0-100 / 1-100k | Tasa de aprendizaje de estimación R |
| `kcc_kalman_noise_mode` | 1 | 0-2 | Modo de combinación (0=off, 1=max, 2=promedio ponderado) |
| `kcc_kalman_q_est_max` | 50000 | 1-2B | Límite superior de estimación Q |
| `kcc_kalman_r_est_max` | 32000 | 1-2B | Límite superior de estimación R |
| `kcc_kalman_q_est_floor` / `r_est_floor` | 1 | 1-100k | Límite inferior por estimación |

### Decaimiento de Ganancia (Sondeo)

| Parámetro | Por defecto | Rango | Unidad | Descripción |
|-----------|-------------|-------|--------|-------------|
| `kcc_qdelay_probe_scale_us` | 20000 | 1-100k | µs | Escala de decaimiento qdelay |
| `kcc_jitter_probe_scale_us` | 16000 | 1-100k | µs | Escala de decaimiento de fluctuación |

### Umbrales Dinámicos de Retardo de Cola

| Parámetro | Por defecto | Rango | Unidad | Descripción |
|-----------|-------------|-------|--------|-------------|
| `kcc_qdelay_clean_bp` | 1000 | 1-10000 | ‱ | Umbral limpio (10 % de min_rtt_us) |
| `kcc_qdelay_cong_bp` | 2500 | 1-10000 | ‱ | Umbral de congestión (25 % de min_rtt_us) |
| `kcc_qdelay_floor_us` | 500 | 1-100k | µs | Piso absoluto para anulación de porcentaje RTT |

### R Adaptativo (Impulsado por Fluctuación)

| Parámetro | Por defecto | Rango | Unidad | Descripción |
|-----------|-------------|-------|--------|-------------|
| `kcc_jitter_r_scale` | 8000 | 1-100k | — | Divisor de escala de aumento R |

### ECN

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `kcc_ecn_enable` | 1 | 0-1 | Interruptor maestro ECN |
| `kcc_ecn_backoff_num` / `kcc_ecn_backoff_den` | 20 / 100 | 0-100 / 1-100k | Fracción de retroceso ECN |
| `kcc_ecn_ewma_retained` / `kcc_ecn_ewma_total` | 3 / 4 | 0-100 / 1-100k | Pesos EWMA ECN |
| `kcc_ecn_idle_decay_num` / `kcc_ecn_idle_decay_den` | 31 / 32 | 1-100k | Decaimiento ECN inactivo |

### min_rtt

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `kcc_minrtt_fast_fall_cnt` | 3 | 0-3 | Contador de caída rápida |
| `kcc_minrtt_fast_fall_div` | 4 | 1-256 | Divisor de umbral de caída rápida |
| `kcc_minrtt_sticky_num` / `kcc_minrtt_sticky_den` | 75 / 100 | 0-1000 / 1-100k | Relación de caída persistente |
| `kcc_minrtt_srtt_guard_num` / `kcc_minrtt_srtt_guard_den` | 90 / 100 | 0-1000 / 1-100k | Relación de guarda SRTT |

### Ancho de Banda LT

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `kcc_lt_intvl_min_rtts` | 4 | 1-127 | RTTs | Longitud mínima de intervalo |
| `kcc_lt_intvl_max_mult` | 4 | 1-32 | Multiplicador de tiempo de espera de intervalo |
| `kcc_lt_loss_thresh` | 25 | 1-65535 | BBR_UNIT | Relación de pérdida mínima |
| `kcc_lt_bw_ratio_num` / `kcc_lt_bw_ratio_den` | 1 / 8 | 0-100k / 1-100k | Tolerancia relativa |
| `kcc_lt_bw_diff` | 500 | 0-100k | bytes/s | Tolerancia absoluta |
| `kcc_lt_bw_max_rtts` | 48 | 1-4094 | RTTs | RTTs activos máximos de LT BW |
| `kcc_lt_bw_ema_num` / `kcc_lt_bw_ema_den` | 1 / 2 | 0-100 / 1-100k | Peso EMA de LT BW |


### Confianza de Agregación ACK

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `kcc_agg_enable` | 1 | 0-1 | Interruptor maestro |
| `kcc_agg_confidence_thresh` | 512 | 0-10000 | Umbral de confianza de compensación cwnd |
| `kcc_agg_max_comp_ratio` | 50 | 0-100 | % de BDP | Límite de compensación cwnd |
| `kcc_agg_max_comp_duration` | 8 | 1-128 | RTTs | Tiempo de espera del vigilante |
| `kcc_agg_r_hysteresis` | 75 | 0-100 | % | Decaimiento de histéresis R |
| `kcc_agg_r_multiplier_min` / `kcc_agg_r_multiplier_max` | 256 / 2048 | 1-10000 | Rango de escalado R (256=1x) |
| `kcc_agg_factor4_ratio_num` / `kcc_agg_factor4_ratio_den` | 3 / 2 | 1-100k | Relación del factor 4 |
| `kcc_agg_safety_bdp_mult` | 3 | 1-100 | Multiplicador BDP de guarda de seguridad |
| `kcc_agg_max_window_ms` | 100 | 1-10000 | ms | Ventana de límite extra_acked |
| `kcc_agg_max_decay_pct` | 75 | 0-100 | % | Tasa de decaimiento del vigilante |
| `kcc_agg_window_rotation_rtts` | 5 | 1-65535 | RTTs | Período de rotación de ventana |
| `kcc_agg_factor_weight` | 256 | 1-1024 | Puntuación por factor |
| `kcc_agg_confidence_max` | 1024 | 256-65535 | Confianza máxima |

### Coeficientes EWMA

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `kcc_ewma_qdelay_num` / `kcc_ewma_qdelay_den` | 7 / 8 | 0-100 / 1-100k | Peso EWMA qdelay |
| `kcc_ewma_jitter_num` / `kcc_ewma_jitter_den` | 7 / 8 | 0-100 / 1-100k | Peso EWMA de fluctuación |

### Misceláneos

| Parámetro | Por defecto | Rango | Descripción |
|-----------|-------------|-------|-------------|
| `kcc_probe_bw_cycle_len` | 8 | 2-256 | Fases del ciclo PROBE_BW (potencia de dos) |
| `kcc_probe_bw_cycle_rand` | 8 | 1-cycle_len | Desplazamiento aleatorio de fase de ciclo |
| `kcc_full_bw_thresh_num` / `kcc_full_bw_thresh_den` | 125 / 100 | 0-100k / 1-100k | Umbral de crecimiento de salida de STARTUP |
| `kcc_full_bw_cnt` | 3 | 1-3 | Rondas sin crecimiento para salir |
| `kcc_probe_rtt_mode_ms_num` / `kcc_probe_rtt_mode_ms_den` | 200 / 1 | 1-100k | Duración de permanencia en PROBE_RTT |
| `kcc_pacing_margin_num` / `kcc_pacing_margin_den` | 1 / 100 | 0-50 / 1-100k | Margen de pacing (1 = 1%) |
| `kcc_probe_cwnd_bonus` | 2 | 0-100 | segs | Bonificación cwnd de fase 0 |
| `kcc_bw_rt_cycle_len` | 10 | 2-256 | rondas | Longitud de ventana deslizante de BW |
| `kcc_cwnd_min_target` | 4 | 1-1000 | segs | Cwnd mínimo (PROBE_RTT) |
| `kcc_bdp_min_rtt_us` | 1 | 0-100k | µs | Piso min_rtt de BDP |
| `kcc_edt_near_now_ns` | 1000 | 0-10M | ns | Umbral EDT casi-ahora |
| `kcc_min_tso_rate` | 1,200,000 | 1-1B | bytes/s | Umbral de tasa baja TSO |
| `kcc_min_tso_rate_div` | 8 | 1-256 | Divisor de tasa TSO (base adaptativa) |
| `kcc_tso_max_segs` | 127 | 1-65535 | segs | Segmentos TSO máximos |
| `kcc_tso_headroom_mult` | 3 | 0-1000 | Multiplicador de espacio libre TSO |
| `kcc_sndbuf_expand_factor` | 3 | 2-100 | Factor de expansión de búfer de envío |
| `kcc_ack_epoch_max` | 0xFFFFF | 64K-2G | bytes | Límite de época ACK |
| `kcc_extra_acked_max_ms_num` / `kcc_extra_acked_max_ms_den` | 150 / 1 | 0-100k / 1-100k | Ventana máxima de agregación ACK |
| `kcc_probe_rtt_decouple` | 1 | 0-1 | Activar desacoplamiento PROBE_RTT (solo modo FILTER) |
| `kcc_rtt_mode` | 1 | 0-1 | Estrategia de RTT del Modelo: 1=FILTER (Kalman directo), 0=MIN (limitado) |
| `kcc_recal_p_est_thresh` | 25000 | 1-100M | Umbral p_est para red de seguridad de recalibración |
| `kcc_probe_rtt_long_rtt_us` | 20000 | 0-10M | µs | Umbral de RTT largo |
| `kcc_probe_rtt_long_interval_div` | 1 | 1-1000 | Divisor de intervalo de RTT largo |
| `kcc_alone_confirm_rounds` | 3 | 1-32 | rondas | Rondas antes de activar el modo de flujo único |
| kcc_alone_agg_state_level | 1 | 0-2 | — | Rigor de agregación (0=solo IDLE, 1=≤SUSPECTED predet., 2=≤CONFIRMED demasiado agresivo) |
| `kcc_alone_bypass_ecn` | 0 | 0-1 | — | Omitir backoff ECN en modo individual (1=omitir, 0=activo) |

## Ruta de Datos

```
Llega ACK (rate_sample)
    │
    ▼
kcc_main()
    │
    ├──► Pipeline de confianza de agregación ACK (cuando kcc_agg_enable)
    │      medir → evaluar → estado → vigilante
    │
    ├──► kcc_update_model()
    │      ├── kcc_update_bw()              BW máximo de ventana deslizante
    │      ├── kcc_update_ecn_ewma()        Relación de marca ECN-CE
    │      ├── kcc_update_ack_aggregation()  extra_acked de doble ventana
    │      ├── kcc_update_cycle_phase()     Avance de fase PROBE_BW
    │      ├── kcc_check_full_bw_reached()  Detección de salida de STARTUP
    │      ├── kcc_check_drain()            Entrada/salida de DRAIN + salto de DRAIN
    │      ├── kcc_update_min_rtt()         Kalman + ventana min-RTT + PROBE_RTT
    │      └── Asignación de ganancia específica del modo
    │
    ├──► kcc_apply_cwnd_constraints()
    │      └── kcc_ecn_backoff()            Retroceso ECN (solo cwnd_gain)
    │
    ├──► kcc_set_pacing_rate()              inmediato, regla BBR
    │
    └──► kcc_set_cwnd()                    BDP + compensación agg
```

## Flujo Interno del Filtro de Kalman

```
Muestra RTT (rtt_us)
    │
    ├── ¿Inválida (≥0 y < dynamic_max)? Sí → descartar
    │
    ├── ¿Arranque en frío (sample_cnt==0)? Sí → init: x_est=z, p_est=max(p_init, rtt_us/div)
    │                                              (omite compuerta máxima RTT)
    │
    ├── Q adaptativo: Q_base × max(q_min_factor, min_rtt_us / q_rtt_div)
    │   R adaptativo: R_base + max(0, jitter − jr_thresh) × R_base / jr_scale
    │
    ├── Innovación: innov = z − x_est
    │
    ├── Q-Boost: |innov| > boost_thresh && p_est ≤ converged && ¿enfriamiento expirado?
    │   ├── Yes: p_est = p_est_init, cooldown = 15, mark qboost_fired
    │   └── No:  cooldown-- if active
    │
    ├── Predecir: p_pred = p_est + Q
    │
    ├── Compuerta de valores atípicos: ¿|innov| > dyn_thresh && p_pred ≤ converged?
    │   ├── Sí y reject_cnt < max → rechazar, ++consec_reject_cnt, retornar
    │   └── Sí y reject_cnt ≥ max → forzar aceptación (anti-bloqueo)
    │
    └── Actualización de Kalman:
         ├── K = p_pred / (p_pred + R)
         ├── x_est += K × innov (sujeto a no negativo)
         ├── p_est = max(p_floor, (1 − K) × p_pred)
         ├── Actualización EWMA de fluctuación
         ├── Actualización EWMA de qdelay
         ├── Estimación de ruido por covarianza coincidente BBR-S
         └── sample_cnt++
```

## Diagnósticos

Interfaz de diagnóstico compatible con BBR a través de `ss -i` (`INET_DIAG_BBRINFO`):

```
bbr_bw_lo/bbr_bw_hi: estimación de ancho de banda de 64 bits (bytes/s)
bbr_min_rtt:         min_rtt_us actual
bbr_pacing_gain:     ganancia de pacing actual (BBR_UNIT, 256=1.0x)
bbr_cwnd_gain:       ganancia de cwnd actual (BBR_UNIT)
```

## Uso

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

La configuración de parámetros es a través de `/proc/sys/net/kcc/`. Por ejemplo:
```sh
# Enable gain decay on specific PROBE_BW phases
echo 1 > /proc/sys/net/kcc/kcc_cycle_decay_mask

# Adjust ECN backoff sensitivity
echo 30 > /proc/sys/net/kcc/kcc_ecn_backoff_num
```

## Modelo de concurrencia y seguridad

KCC deliberadamente no usa READ_ONCE/WRITE_ONCE ni RCU para sus propias estructuras de datos. Este diseño es consistente con todos los módulos CC dentro del kernel como BBR y CUBIC.

`kcc_init()` se ejecuta en contexto de proceso (durante la creación del socket), antes de que el socket esté expuesto a cualquier softirq. `kcc_release()` se ejecuta después de que el kernel garantiza que ningún softirq sigue procesando los ACK de este socket. Un valor obsoleto transitorio de un parámetro global del módulo afecta como máximo a un ACK, corregido en el siguiente ACK.

La única excepción: `sk->sk_pacing_rate` / `sk->sk_pacing_shift` son campos de la capa de socket que el espacio de usuario puede modificar simultáneamente mediante `setsockopt`, por lo que se preserva la convención WRITE_ONCE/READ_ONCE de BBR.

## Resumen de Rendimiento

Entorno de prueba: China → EE. UU. LAX, RTT de 212 ms, 8 flujos paralelos, 26 % de pérdida de paquetes, cuello de botella VPS compartido de 1 Gbps.

| Métrica | KCC v1.0 | BBR (control) | Delta |
|---------|----------|---------------|-------|
| Rendimiento promedio | 1,010 Mbps | 937 Mbps | **+7.8 %** |
| Inequidad intra-KCC | 3.1× | 6.2× (BBR) | **−50 %** |
| Peor flujo individual | 60.6 Mbps | 30.8 Mbps | **+97 %** |
| Retransmisiones | 150K/10s | 137K/10s | +9.5 % |
| Estabilidad en ronda 3 | 959 Mbps | 883 Mbps | **+8.6 %** |

Las retransmisiones son ligeramente más altas — un compromiso coherente con el mantenimiento de una alta utilización del enlace bajo pérdidas. La estimación min_rtt aumentada por Kalman de KCC proporciona una línea base BDP más precisa, permitiendo que el algoritmo mantenga un rendimiento mayor que BBRv1 en la misma ruta.

## Global Kalman BDP — Inyección de Ancho de Banda entre Conexiones

KCC v1.0 incluye un Filtro de Kalman Global opcional entre conexiones que estima el ancho de banda del cuello de botella en estado estacionario del servidor. Esta estimación se utiliza para iniciar nuevas conexiones a una "velocidad de postre" conservadoramente baja — lo suficientemente rápida para saltarse el arranque en frío, lo suficientemente lenta para evitar sobrepasarse.

### Principio de Diseño

El filtro se alimenta con muestras de ancho de banda de la **fase de crucero** PROBE\_BW (ganancia = 1.0×) de todas las conexiones KCC. Las muestras de la fase de crucero son la señal más limpia del ancho de banda disponible real — sin el sobrepaso de sonda de 1.25×, sin la subestimación de drenaje de 0.75×. Un filtro de Kalman unidimensional de caminata aleatoria (Kalman 1960) rastrea el estado estacionario global.

Cuando se establece una nueva conexión, la estimación del filtro se utiliza para sembrar:

| Valor inyectado | Propósito |
|----------------|---------|
| `minmax` (max\_bw tracker) | Sembrar el historial de ancho de banda de ventana deslizante para que las primeras muestras sucias de ACK no lo arrastren a cero |
| `sk_pacing_rate` | Tasa de pacing inicial con ganancia neutral (BBR\_UNIT); la ganancia de 2.89× de STARTUP se aplica en el primer ACK |
| `tp->snd_cwnd` | Ventana de congestión inicial calculada mediante `kcc_bdp()` con ganancia neutral |

Un piso defensivo en `kcc_update_bw` evita que los primeros RTTs de muestras de baja tasa de entrega sobrescriban la estimación inyectada durante STARTUP. Una protección de BW completo en `kcc_check_full_bw_reached` evita que el intercambio de mensajes de control de iperf3 termine prematuramente STARTUP.

### Relación de Descuento de Velocidad de Postre

La velocidad de inyección efectiva es:

```
coeff = (discount_ratio) / high_gain
      = (num / den) / 2.89
```

donde `high_gain ≈ 2.89` es el multiplicador de pacing de STARTUP de BBR.

| num | coeff  | característica |
|-----|--------|----------------|
|  35 | 12.1%  | máxima seguridad, peor ruta |
|  50 | 17.3%  | eje central (predeterminado) |
|  75 | 25.9%  | punto dulce matemático de postre |
|  80 | 27.6%  | techo matemático de tasa (no debe excederse) |

**Nota:** `tcp_write_xmit` impone un CWND inicial de `TCP_INIT_CWND` (10 segmentos, ≈15 KB) para cada nueva conexión. El CWND solo crece cuando llegan ACKs remotos, por lo que la velocidad de postre es un límite superior de la tasa de pacing — el rendimiento real está limitado por CWND hasta que se hayan recibido suficientes ACKs para abrir la ventana.

### Configuración

Habilitar mediante `sysctl`:

```bash
sysctl -w net.kcc.kcc_kf_enable=0           # habilitación maestra (predeterminado 0)
sysctl -w net.kcc.kcc_kf_discount_num=50   # numerador de velocidad de postre (predet. 50, rango 35–75)
```

**Parámetros sysctl clave** (`/proc/sys/net/kcc/`):

| Parámetro | Predet. | Rango | Descripción |
|-----------|---------|-------|-------------|
| `kcc_kf_enable` | 0 | 0–1 | Habilitación maestra para inyección global Kalman BDP |
| `kcc_kf_discount_num` | 50 | 0–100 | Numerador de velocidad de postre (% del BW de participación justa) |
| \`kcc_kf_discount_den\` | 100 | 1–100000 | Denominador de velocidad de postre |
| \`kcc_kf_steady_mode\` | 0 | 0/1 | — | Modo estable: al activarse, usa el pico monótono (kf_x_steady) para init_bw, ignorando caídas transitorias del KF |
| \`kcc_kf_startup_r_pct\` | 20 | 1–100 | R% de ruido de medición durante la fase de arranque |
| \`kcc_kf_steady_r_pct\` | 5 | 1–100 | R% de ruido de medición durante estado estacionario |
| \`kcc_kf_q_shift\` | 20 | 0–30 | Desplazamiento de ruido de proceso (Q = 1 << shift) |
| \`kcc_kf_chi2_num\` | 384 | 1–100000 | Numerador de compuerta de valores atípicos chi-cuadrado |
| \`kcc_kf_chi2_den\` | 100 | 1–100000 | Denominador de compuerta de valores atípicos chi-cuadrado |

Cuando kcc_kf_steady_mode está habilitado (1), el ancho de banda inicial para nuevas conexiones usa el pico monótono de la estimación KF (kf_x_steady) en lugar de la estimación en vivo, que puede haber disminuido desde la última conexión de alto rendimiento. Esto evita la inanición de arranque en frío en rutas estables. El pico se reinicia a cero al deshabilitar el modo, permitiendo un inicio limpio al reactivarlo.

### Rendimiento en el Primer Segundo (Transpacífico, 212 ms RTT)

```
Sin KF:  2.8 Mbps  →  85 Mbps  →  622 Mbps  →  estable
Con KF:  50 Mbps   →  530 Mbps  →  650 Mbps  →  estable
```

La velocidad del primer segundo salta de ~3 Mbps (arranque en frío) a ~50 Mbps (arranque de postre), y la convergencia al estado estacionario se alcanza en 2–3 segundos. Las retransmisiones permanecen en cero en todo momento.

### Cómo Funciona

1. Una conexión KCC en ejecución entra en la fase de crucero PROBE\_BW → límite de inicio de ronda → alimenta `kcc_kf_update(bw, 5%)` con la muestra actual de tasa de entrega.
2. El filtro de Kalman actualiza su estimación `kcc_kf_x` (un promedio móvil del ancho de banda del cuello de botella en estado estacionario).
3. Cuando se abre una **nueva** conexión, `kcc_init` llama a `kcc_kf_get_init_bw(sk)` que devuelve `fair × discount / high_gain` — una estimación de ancho de banda inicial de participación justa, compensada por ganancia.
4. Esta estimación siembra `sk_pacing_rate`, `tp->snd_cwnd` y el rastreador de ancho de banda `minmax` — la conexión comienza a la velocidad de postre en lugar de desde cero.

### Fuente del Algoritmo

El filtro Global Kalman BDP se basa en el artículo del autor *Sobre la estimación de Kalman y la implementación de ingeniería del ancho de banda global en estado estacionario en el kernel de Linux* (CC BY-SA 4.0):
https://blog.csdn.net/liulilittle/article/details/161635652

---

*KCC v1.0 — construido sobre BBRv1 (Cardwell et al. 2016, ACM Queue) y el filtro de Kalman (Kalman 1960).*

## Referencias

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