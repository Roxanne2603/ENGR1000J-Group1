/*
 * VL53L0X 首件验收 · 第 3 步：测距 / 有效率 / 重复性 / 最坏无新数据时长
 *
 * ⚠️ v4（2026-10-04，据 Codex 第三轮复核意见补统计缺口）
 *   v1 bug：只排除 RangeStatus==4，把 sigma(1)/signal(2)/min-range(3) 当有效，且没查返回值。
 *   v2 已修：valid = (err == VL53L0X_ERROR_NONE && m.RangeStatus == 0)
 *   v3 已修：(A) 统计口径 —— 新增「有效读数间隔」，并区分 [all] 尝试间隔 / [valid] 有效间隔；
 *            (B) 记录缺口 —— 取消逐样本开关，CSV 全程输出；
 *            (C) 补 true_mm 列；取消 gap 的上限过滤（长空档正是要抓的东西）。
 *   v4 再补（Codex 指出：「末尾失败时汇总仅 100ms，但已失联至少 800ms」）：
 *       (D) ⭐ **首失联**（开始 → 第一个有效读数）
 *       (E) ⭐ **尾失联**（最后一个有效读数 → 结束）—— 末段连续失败时这里会很大，
 *            而 v3 的汇总**完全看不出来**
 *       (F) **实际时长**（不是名义的 SAMPLE_SECONDS）
 *       (G) ⭐ **最坏无新数据时长 = max(首失联, 有效间隔最大值, 尾失联)**
 *            ← **控制侧真正要引用的就是这一个数**，v3 缺了首尾两段会低估它
 *
 * 需要装库：Adafruit VL53L0X（+ 依赖 Adafruit BusIO）
 * 板：ESP32-WROOM-32E ｜ 接线：VCC→3V3 GND→GND SDA→GPIO21 SCL→GPIO22 XSH→3V3
 *
 * 【用法】
 *   1. 改 TRUE_MM = 本档实际距离（卷尺从【光学窗口前平面】量到靶面）
 *   2. 上传 → 开串口 115200 → 等 SAMPLE_SECONDS 秒 → 抄末尾汇总
 *   3. 300 s 长跑：约 15,000 行 CSV（≈400 KB），必须【存成文件】，别只截图
 *
 * 注意：940nm 是不可见红外，别把眼睛贴到发射窗上看。
 */

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define PIN_SDA  21
#define PIN_SCL  22

// ── 每次测一档要改这两行 ──────────────────────────────
#define TRUE_MM             100    // 本档实际距离（mm），卷尺量
#define SAMPLE_SECONDS       20    // 采样时长（秒）；稳定性测试改 300
// ─────────────────────────────────────────────────

Adafruit_VL53L0X lox;

// 总计
uint32_t attempts = 0, validN = 0, apiErrN = 0;
uint32_t statusCount[8] = {0};
double   sum = 0, sumSq = 0;
uint16_t vMin = 65535, vMax = 0;
uint32_t maxAbsErr = 0;

// 尝试间隔（吞吐口径）
uint32_t tsPrev = 0, maxGapMs = 0, tStart = 0;
double   sumGapMs = 0; uint32_t gapN = 0;

// 有效读数间隔（控制口径）
uint32_t tsLastValid = 0, maxValidGapMs = 0;
double   sumValidGapMs = 0; uint32_t validGapN = 0;

// 连续失败
uint32_t consecFail = 0, maxConsecFail = 0;

// 🆕 v4：首尾失联与实际时长
uint32_t tsFirstValid = 0, endTs = 0;

bool     phaseDone = false;
uint32_t nextProgress = 0;

void printSummary() {
  uint32_t actualMs = endTs - tStart;
  uint32_t firstGap = tsFirstValid ? (tsFirstValid - tStart) : actualMs;
  uint32_t tailGap  = tsLastValid  ? (endTs - tsLastValid)  : actualMs;
  uint32_t worst    = firstGap;
  if (maxValidGapMs > worst) worst = maxValidGapMs;
  if (tailGap > worst)       worst = tailGap;

  Serial.println();
  Serial.println("──────── 汇总 ────────");
  Serial.printf("实际距离 TRUE_MM   = %u mm\n", (unsigned)TRUE_MM);
  Serial.printf("实际时长           = %.1f s  （名义 %d s）\n",
                actualMs / 1000.0, SAMPLE_SECONDS);
  Serial.printf("尝试次数 attempts  = %u\n", (unsigned)attempts);
  Serial.printf("有效样本 valid     = %u\n", (unsigned)validN);
  Serial.printf("有效率 valid rate  = %.1f %%   (分母 = attempts)\n",
                attempts ? 100.0 * (double)validN / (double)attempts : 0.0);
  Serial.printf("API 失败 api_error = %u\n", (unsigned)apiErrN);
  Serial.print  ("RangeStatus 分布   = ");
  for (int i = 0; i < 8; i++) Serial.printf("[%d]=%u ", i, (unsigned)statusCount[i]);
  Serial.println();
  Serial.println("  (0=有效 1=sigma 2=signal 3=min-range 4=phase 7=API错误/其它)");

  if (validN) {
    double mean = sum / (double)validN;
    double var  = (sumSq / (double)validN) - mean * mean;
    double sd   = var > 0 ? sqrt(var) : 0.0;
    Serial.printf("均值 mean          = %.1f mm\n", mean);
    Serial.printf("均值偏差 bias      = %+.1f mm  (mean − TRUE_MM)\n", mean - (double)TRUE_MM);
    Serial.printf("标准差 sigma       = %.2f mm   (总体口径：除以 N)\n", sd);
    Serial.printf("最小 / 最大        = %u / %u mm\n", vMin, vMax);
    Serial.printf("最大绝对误差       = %u mm\n", (unsigned)maxAbsErr);
  } else {
    Serial.println("!!! 没有任何有效样本 —— 不要据此判合格");
  }

  Serial.println();
  Serial.println("-- 延迟（三种口径，别混用）--");
  if (gapN) {
    double mg = sumGapMs / (double)gapN;
    Serial.printf("[all]   尝试间隔      平均 %.1f ms (≈ %.1f Hz)   最大 %u ms\n",
                  mg, 1000.0 / mg, (unsigned)maxGapMs);
  }
  if (validGapN) {
    double mv = sumValidGapMs / (double)validGapN;
    Serial.printf("[valid] 有效读数间隔  平均 %.1f ms (≈ %.1f Hz)   最大 %u ms\n",
                  mv, 1000.0 / mv, (unsigned)maxValidGapMs);
  } else {
    Serial.println("[valid] 有效读数不足 2 个，无法给出有效读数间隔");
  }
  // 🆕 v4
  Serial.printf("[head]  首失联（开始→首个有效） = %u ms\n", (unsigned)firstGap);
  Serial.printf("[tail]  尾失联（末个有效→结束） = %u ms   <-- 末段连续失败时这里会很大\n",
                (unsigned)tailGap);
  Serial.printf("[fail]  最大连续失败 = %u 次\n", (unsigned)maxConsecFail);

  Serial.println();
  Serial.printf("★ 最坏无新数据时长 = max(首失联, 有效间隔最大, 尾失联) = %u ms\n",
                (unsigned)worst);
  Serial.println("  ↑↑ 控制侧要引用的就是这一个数（v3 缺首尾两段、会低估它）");
  Serial.println("──────────────────────");
  Serial.println("把本条 CSV 与汇总【存成文件】交给我填验收记录。下一档：改 TRUE_MM 重新上传。");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  Serial.println();
  Serial.println("===== VL53L0X 第 3 步 v4：测距 / 有效率 / 最坏无新数据时长 =====");
  Serial.printf("本档 TRUE_MM=%u mm，采样 %d 秒\n", (unsigned)TRUE_MM, SAMPLE_SECONDS);

  if (!lox.begin()) {
    Serial.println("!! lox.begin() 失败 —— 回到第 1 步确认 0x29");
    Serial.println("   若刚做过改地址，必须先给【模块】断电（不是按 EN），让地址回 0x29");
    while (true) delay(1000);
  }
  Serial.println("OK 初始化成功");

  // ── 阶段 A：调用吞吐率（含有效性判定）──
  Serial.println("--- 阶段 A：调用吞吐率（100 次连续调用）");
  uint32_t t0 = millis();
  uint32_t okA = 0, apiA = 0;
  for (int i = 0; i < 100; i++) {
    VL53L0X_RangingMeasurementData_t m = {};
    VL53L0X_Error err = lox.rangingTest(&m, false);
    if (err != VL53L0X_ERROR_NONE) apiA++;
    else if (m.RangeStatus == 0)   okA++;
  }
  uint32_t dtA = millis() - t0;
  Serial.printf("    100 次调用 %u ms → 调用吞吐 %.1f 次/s\n",
                (unsigned)dtA, 100000.0 / (double)dtA);
  Serial.printf("    其中 有效 %u、API 失败 %u、其它无效 %u\n",
                (unsigned)okA, (unsigned)apiA, (unsigned)(100 - okA - apiA));
  Serial.println("    注意：这只是「调用吞吐」，不等于阶段 B 的持续有效更新率。");

  Serial.println();
  Serial.println("--- 阶段 B：持续采样   CSV: time_ms,true_mm,range_mm,api_error,range_status");

  tStart = millis();
  tsPrev = tStart;
  tsLastValid = 0;
  tsFirstValid = 0;
  nextProgress = 5000;
}

void loop() {
  if (phaseDone) { delay(500); return; }

  VL53L0X_RangingMeasurementData_t m = {};
  VL53L0X_Error err = lox.rangingTest(&m, false);
  uint32_t now = millis();

  attempts++;
  bool valid = (err == VL53L0X_ERROR_NONE && m.RangeStatus == 0);

  if (err != VL53L0X_ERROR_NONE) {
    apiErrN++;
    statusCount[7]++;
    Serial.printf("%u,%u,,%d,-1\n", (unsigned)now, (unsigned)TRUE_MM, (int)err);
  } else {
    uint8_t st = m.RangeStatus;
    statusCount[st < 7 ? st : 7]++;
    if (valid) {
      if (tsFirstValid == 0) tsFirstValid = now;      // 🆕 v4 首失联
      validN++;
      sum   += m.RangeMilliMeter;
      sumSq += (double)m.RangeMilliMeter * (double)m.RangeMilliMeter;
      if (m.RangeMilliMeter < vMin) vMin = m.RangeMilliMeter;
      if (m.RangeMilliMeter > vMax) vMax = m.RangeMilliMeter;
      uint32_t e = (m.RangeMilliMeter > TRUE_MM)
                 ? (uint32_t)(m.RangeMilliMeter - TRUE_MM)
                 : (uint32_t)(TRUE_MM - m.RangeMilliMeter);
      if (e > maxAbsErr) maxAbsErr = e;
    }
    Serial.printf("%u,%u,%u,0,%u\n", (unsigned)now, (unsigned)TRUE_MM,
                  m.RangeMilliMeter, st);
  }

  // ── 连续失败 / 有效读数间隔 ──
  if (valid) {
    if (tsLastValid != 0) {
      uint32_t vg = now - tsLastValid;
      if (vg > 0) {                       // ⚠️ 不做上限过滤：长空档正是要抓的东西
        sumValidGapMs += vg; validGapN++;
        if (vg > maxValidGapMs) maxValidGapMs = vg;
      }
    }
    tsLastValid = now;
    consecFail = 0;
  } else {
    consecFail++;
    if (consecFail > maxConsecFail) maxConsecFail = consecFail;
  }

  // ── 尝试间隔（吞吐口径）──
  uint32_t gap = now - tsPrev;
  tsPrev = now;
  if (gap > 0) {                          // ⚠️ 不做上限过滤
    sumGapMs += gap; gapN++;
    if (gap > maxGapMs) maxGapMs = gap;
  }

  uint32_t elapsed = now - tStart;
  if (SAMPLE_SECONDS >= 30 && elapsed >= nextProgress) {
    Serial.printf("[进度] %u s / %d s，尝试 %u，有效 %u，连续失败最多 %u\n",
                  (unsigned)(elapsed / 1000), SAMPLE_SECONDS,
                  (unsigned)attempts, (unsigned)validN, (unsigned)maxConsecFail);
    nextProgress += 5000;
  }

  if (elapsed >= (uint32_t)SAMPLE_SECONDS * 1000UL) {
    phaseDone = true;
    endTs = now;                          // 🆕 v4 尾失联要用
    printSummary();
  }
}
