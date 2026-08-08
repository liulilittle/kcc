// cusum_vs_g2.js
// ============================================================
// G2 vs CUSUM 全频谱公平对比
// 修复1：增加运行最小值更新（v<=0 时 x_est = min(x_est, z)）
// 修复2：路径变化时重置 S_pos 和 detected 标志
// ============================================================

const SCALE = 1024;

// -------------------- 数学工具 --------------------
function gauss(rng, mean = 0, std = 1) {
  const u1 = rng(), u2 = rng();
  return mean + std * Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
}

function seedRandom(seed) {
  return function () {
    seed = (seed * 1664525 + 1013904223) & 0xffffffff;
    return (seed >>> 0) / 0xffffffff;
  };
}

function avg(arr) { return arr.length ? arr.reduce((a, b) => a + b, 0) / arr.length : NaN; }
function median(arr) {
  if (!arr.length) return NaN;
  const s = [...arr].sort((a, b) => a - b);
  const m = Math.floor(s.length / 2);
  return s.length % 2 !== 0 ? s[m] : (s[m - 1] + s[m]) / 2;
}
function percentile(arr, p) {
  if (!arr.length) return NaN;
  const s = [...arr].sort((a, b) => a - b);
  const idx = Math.max(0, Math.ceil(s.length * p / 100) - 1);
  return s[idx];
}

// ======================== CUSUM 检测器 ========================
class CUSUMDetector {
  constructor(T_prop, sigma, delta = null, h = null) {
    this.x = T_prop * SCALE;          // 定点 x_est，现在包含运行最小值更新
    this.mr = T_prop;                 // min_rtt (µs)
    this.sigma = sigma;

    // CUSUM 参数
    this.delta = delta || Math.floor(T_prop * 0.05);   // 默认漂移敏感度 5% T_prop
    this.h     = h     || Math.floor(T_prop * SCALE * 0.15); // 默认阈值 15% T_prop

    this.S_pos    = 0;    // 正向累积和
    this.detected = false;

    // 探测逻辑 (与 GeodesicEstimator 保持一致)
    this.probe    = 0;
    this.currentT = T_prop;
  }

  setPathChanged(newT_prop) {
    this.currentT = newT_prop;
    // 关键修复：重置累积状态和检测标志，防止预热阶段遗留状态影响检测
    this.S_pos = 0;
    this.detected = false;
  }

  step(rng, rtt, ql = 0) {
    this.probe++;
    let actualRTT = rtt;

    // 探测 (与 GeodesicEstimator 一致)
    if (this.probe >= 2000) {
      this.probe = 0;
      if (ql === 0) {
        actualRTT = this.currentT + Math.round(gauss(rng, 0, this.sigma));
      }
    }

    // 更新 min_rtt
    if (actualRTT < this.mr) this.mr = actualRTT;

    const z = actualRTT * SCALE;
    const v = z - this.x;

    // ----- 增加运行最小值更新（与 G2 的 G1 分支一致） -----
    if (v <= 0) {
      this.x = Math.min(this.x, z);
      // 下降信号出现，重置部分累积（可选，增强抗噪）
      // 这里不重置 S_pos，因为 CUSUM 原本设计就是单向累积，下降不参与正向累积
    } else {
      // CUSUM 正向累积和
      const drift = this.delta * SCALE;
      if (v > drift) {
        const contribution = v - drift;
        this.S_pos = Math.max(0, this.S_pos + contribution);
      }
    }

    // 检测触发：同时更新 this.x 和 this.mr
    if (this.S_pos >= this.h && !this.detected) {
      this.detected = true;
      this.x   = Math.max(this.x, z);         // 将 x_est 更新为当前观测值
      this.mr  = Math.max(this.mr, actualRTT); // 将 min_rtt 更新为当前观测值
    }
  }

  bdp() {
    const xUs = Math.floor(this.x / SCALE);
    return xUs < this.mr ? xUs : this.mr;
  }
}

// ======================== 原版 G2 估计器 ========================
class GeodesicEstimator {
  constructor(T_prop, sigma) {
    this.x      = T_prop * SCALE;
    this.mr     = T_prop;
    this.conf   = 0;
    this.T      = T_prop;
    this.probe  = 0;
    this.sigma  = sigma;
    this.pathGrown = false;
  }

  setPathChanged(newT_prop) {
    this.T = newT_prop;
    this.pathGrown = true;
    this.conf = 0;
    this.confSlow = 0;
  }

  step(rng, rtt, ql = 0) {
    this.probe++;
    let actualRTT = rtt;

    if (this.probe >= 2000) {
      this.probe = 0;
      if (ql === 0) {
        actualRTT = this.T + Math.round(gauss(rng, 0, this.sigma));
      }
    }

    const z = actualRTT * SCALE;
    const v = z - this.x;

    if (v <= 0) {
      this.x = Math.min(this.x, z);
    } else {
      const growth = Math.floor((this.x * 12) / 100);
      this.x = Math.min(this.x + growth, z);
    }

    // G3 dual-threshold: fast 10%/3, slow 5%/50
    const threshFast = Math.floor((this.mr * 11 * SCALE) / 10);
    const threshSlow = Math.floor((this.mr * 21 * SCALE) / 20);
    if (this.x > threshFast) {
      this.conf++;
      this.confSlow = (this.confSlow || 0) + 1;
    } else if (this.x > threshSlow) {
      this.confSlow = (this.confSlow || 0) + 1;
    } else if (this.x <= this.mr * SCALE) {
      this.conf = 0;
      this.confSlow = 0;
    }

    if (this.conf >= 3 || this.confSlow >= 50) {
      const oldMr = this.mr;
      this.mr = Math.floor(this.x / SCALE);
      this.conf = 0;
      this.confSlow = 0;
      if (this.pathGrown && this.mr > oldMr) {
        this.T = this.mr;
      }
    }
  }

  bdp() {
    const xUs = Math.floor(this.x / SCALE);
    return xUs < this.mr ? xUs : this.mr;
  }
}

// ======================== G2 vs CUSUM 对比测试 ========================
function compareG2vsCUSUM() {
  const RTTs = [
    25, 50, 100, 200, 500, 1000, 1400, 2000, 5000, 10000,
    50000, 100000, 300000, 500000, 1000000
  ];
  const GROWTHS = [5, 10, 25, 50, 100, 200];
  const SEEDS = 20;
  const MAX_STEPS = 500;

  console.log('='.repeat(180));
  console.log('G2 (12% GEOMETRIC) vs CUSUM (δ=5% T_prop, h=15% T_prop)');
  console.log('='.repeat(180));
  console.log(
    'RTT(µs)'.padStart(9) +
    'Amp%'.padStart(6) +
    'G2_Det%'.padStart(9) +
    'G2_Med'.padStart(8) +
    'G2_P90'.padStart(8) +
    'CUSUM_Det%'.padStart(11) +
    'CUSUM_Med'.padStart(10) +
    'CUSUM_P90'.padStart(10) +
    'Winner'.padStart(8) +
    'Note'.padStart(15)
  );
  console.log('-'.repeat(180));

  for (const T of RTTs) {
    const sigma = Math.max(1, Math.floor(T / 100));
    for (const amp of GROWTHS) {
      const Tnew = T + Math.floor(T * amp / 100);
      if (Tnew === T) continue;

      // G2 测试
      const g2Delays = [];
      let g2Missed = 0;
      for (let seed = 0; seed < SEEDS; seed++) {
        const rng = seedRandom(T * 1000 + amp + seed);
        const est = new GeodesicEstimator(T, sigma);

        for (let i = 0; i < 2000; i++) {
          est.step(rng, Math.max(1, T + Math.round(gauss(rng, 0, sigma))));
        }

        est.setPathChanged(Tnew);

        let detected = false;
        for (let s = 1; s <= MAX_STEPS; s++) {
          est.step(rng, Math.max(1, Tnew + Math.round(gauss(rng, 0, sigma))));
          if (est.bdp() > T + Math.floor(T * 0.02)) {
            g2Delays.push(s);
            detected = true;
            break;
          }
        }
        if (!detected) g2Missed++;
      }

      // CUSUM 测试 (δ=5% T_prop, h=15% T_prop)
      const cusumDelays = [];
      let cusumMissed = 0;
      for (let seed = 0; seed < SEEDS; seed++) {
        const rng = seedRandom(T * 1000 + amp + seed + 100000);
        const est = new CUSUMDetector(T, sigma);

        for (let i = 0; i < 2000; i++) {
          est.step(rng, Math.max(1, T + Math.round(gauss(rng, 0, sigma))));
        }

        est.setPathChanged(Tnew);

        let detected = false;
        for (let s = 1; s <= MAX_STEPS; s++) {
          est.step(rng, Math.max(1, Tnew + Math.round(gauss(rng, 0, sigma))));
          if (est.bdp() > T + Math.floor(T * 0.02)) {
            cusumDelays.push(s);
            detected = true;
            break;
          }
        }
        if (!detected) cusumMissed++;
      }

      const g2DetPct = (g2Delays.length / SEEDS * 100).toFixed(1);
      const g2Med    = g2Delays.length ? median(g2Delays).toFixed(1) : '-';
      const g2P90    = g2Delays.length ? String(percentile(g2Delays, 90)) : '-';

      const csDetPct = (cusumDelays.length / SEEDS * 100).toFixed(1);
      const csMed    = cusumDelays.length ? median(cusumDelays).toFixed(1) : '-';
      const csP90    = cusumDelays.length ? String(percentile(cusumDelays, 90)) : '-';

      const g2MedVal = g2Delays.length ? median(g2Delays) : Infinity;
      const csMedVal = cusumDelays.length ? median(cusumDelays) : Infinity;
      const winner   = g2MedVal < csMedVal ? 'G2' : (csMedVal < g2MedVal ? 'CUSUM' : 'TIE');
      const note     = (csDetPct < 100) ? 'CUSUM MISS' : '';

      console.log(
        String(T).padStart(9) +
        (String(amp) + '%').padStart(6) +
        (g2DetPct + '%').padStart(9) +
        String(g2Med).padStart(8) +
        g2P90.padStart(8) +
        (csDetPct + '%').padStart(11) +
        String(csMed).padStart(10) +
        csP90.padStart(10) +
        winner.padStart(8) +
        note.padStart(15)
      );
    }
  }
}

// ======================== 主程序 ========================
console.log('\nCUSUM vs G2 COMPARISON\n');
console.log('说明：');
console.log('1. CUSUM 增加运行最小值更新（v<=0 时 x_est = min(x_est, z)）');
console.log('2. setPathChanged 时重置 S_pos 和 detected 标志');
console.log('确保 bdp() = min(x_est, min_rtt) 立即超过旧基线\n');

compareG2vsCUSUM();
