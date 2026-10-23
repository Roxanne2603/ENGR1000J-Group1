/*
 * MD520 台架测试 · 第 2 步：**中断/高速计数压力自检**（不需要 12V、不需要示波器、不需要外部信号源）
 * ===========================================================================
 * v2（2026-10-05 10:1x，据 Codex 复核修三处缺陷）
 *   [1] `q` 命令原来只把引脚拉低、**定时器仍在跑**（等于没停）→ 现在**真正 disable 定时器**
 *   [2] 扫频的"通过判定"原来**只检查 A 相**（B 不计数也会判通过）→ 现在 **A / B / |4x| 三项都判**
 *   [3] 定时器 API 改为 **Arduino-ESP32 2.x / 3.x 双分支**（3.x 的 timerBegin 签名变了）
 *   [4] 启动时打印**实际编译所用的核心版本**，便于留档
 *
 * ⚠️ **本 sketch 尚未实机编译**（编写环境无 Arduino 工具链）→ 上传前若有报错，请把原文发回。
 *
 * ---------------------------------------------------------------------------
 * 目的：解决"手转只覆盖了目标转速的约 2.7%，高速计数完全未验证"这个空白。
 * 原理：**让 ESP32 自己产生已知频率的正交方波，喂给自己的编码器输入引脚**，
 *       再用与 step1 **完全相同**的解码计数逻辑去数 → 期望 vs 实测一比，
 *       即可量化"**在多少 Hz 开始丢计数**"，从而给出安全余量。
 *
 * 目标工况参考（550rpm、CPR≈206）：A 相边沿 ≈1888/s，B 相边沿 ≈1888/s。
 *
 * ---------------------------------------------------------------------------
 * ⚠️ 接线（**先把电机的 A/B 杜邦线从 ESP32 上拔掉**，否则两侧同时驱动引脚会打架）
 *     GPIO25 ——> GPIO18      （自己发的 A 相，接进"编码器输入 A"）
 *     GPIO26 ——> GPIO19      （自己发的 B 相，接进"编码器输入 B"）
 *     电机编码器的 VCC/GND 可以继续留着（无害），但 A/B 必须离开 18/19。
 *
 * 串口命令（115200，行尾"换行符"）
 *   h        帮助
 *   s        **自动扫频**：100 → 200 → 500 → 1k → 2k → 5k → 10k Hz，每档 3 s，逐档对比
 *   f <hz>   手动设频率（正交频率 = 每相每秒的边沿数），例如 f 1888
 *   r        清零并重新计时
 *   p        打印当前状态
 *   q        **真正停掉信号发生器**（disable 定时器 + 引脚拉低）
 *
 * 判读：✅ 三项误差都 <0.1% → 该频率可靠；❌ 任一明显偏小 → 记该频率为上限
 *
 * 边界（如实说明）：
 *   - 本测试验证的是**中断吞吐与解码计数**，**不能替代 12V 下真实电机的转速实测**；
 *   - 发生器与解码器同芯片、互相竞争 CPU，比真实工况更严苛 ⇒ 结果**偏保守**（安全方向）。
 */

#include <Arduino.h>
#include <esp_arduino_version.h>

// ---- 被测输入（与 step1 完全一致的脚）--------------------------------------
static const uint8_t PIN_A_IN = 18;
static const uint8_t PIN_B_IN = 19;
// ---- 信号发生器输出 --------------------------------------------------------
static const uint8_t PIN_A_OUT = 25;
static const uint8_t PIN_B_OUT = 26;

// ---- 与 step1 相同的解码计数 ----------------------------------------------
volatile int32_t  cnt4x = 0;
volatile int32_t  cntA  = 0;
volatile int32_t  cntB  = 0;
volatile uint32_t jumpCnt = 0;
volatile uint8_t  prevState = 0;
volatile uint8_t  prevA = 0, prevB = 0;

// ⚠️ 与 step1_encoder v3 保持完全一致（含其符号约定 —— Codex 已确认"正负属方向约定，
//    反转时符号相反即自洽"，**不需要为符号而改表**），以便两边结果可互相印证。
static const int8_t QTAB[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void IRAM_ATTR encISR() {
  uint8_t a = (uint8_t)digitalRead(PIN_A_IN);
  uint8_t b = (uint8_t)digitalRead(PIN_B_IN);
  uint8_t curr = (uint8_t)((a << 1) | b);
  uint8_t from = prevState;

  if ((uint8_t)(from ^ curr) == 0x03) jumpCnt++;
  cnt4x += QTAB[(from << 2) | curr];
  prevState = curr;

  if (a && !prevA) cntA++;
  if (b && !prevB) cntB++;
  prevA = a;
  prevB = b;
}

// ---- 正交信号发生器（硬件定时器，频率精确可控）----------------------------
hw_timer_t *genTimer = NULL;
volatile uint32_t genStep = 0;
volatile uint64_t genTicks = 4;         // 每步的 tick 数（1 tick = 1 µs）
volatile bool     genOn = false;
// 00 -> 01 -> 11 -> 10 循环（标准正向正交）
static const uint8_t QSTATE[4][2] = {{0,0},{0,1},{1,1},{1,0}};

void IRAM_ATTR onGenTimer() {
  genStep = (genStep + 1) & 3;
  digitalWrite(PIN_A_OUT, QSTATE[genStep][0]);
  digitalWrite(PIN_B_OUT, QSTATE[genStep][1]);
}

// 设定报警周期（µs）—— 2.x / 3.x API 双分支
static inline void applyAlarm(uint64_t ticks) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  timerAlarm(genTimer, ticks, true, 0);
#else
  timerAlarmWrite(genTimer, ticks, true);
#endif
}

static void setGenFreq(float hz) {
  if (hz < 1.0f) hz = 1.0f;
  if (hz > 20000.0f) hz = 20000.0f;
  uint64_t ticks = (uint64_t)(1000000.0f / (4.0f * hz) + 0.5f);   // 一拍 = 1/4 正交周期
  if (ticks < 3) ticks = 3;          // 保护：定时器太快会把 CPU 拖垮
  genTicks = ticks;

  applyAlarm(ticks);
  timerAlarmEnable(genTimer);        // 重新起动（含从 stopGen 之后恢复）
  genOn = true;

  Serial.printf(">> 请求正交频率 %.1f Hz  → 实际 %.2f Hz（每步 %llu µs）\n",
                hz, 1000000.0f / (4.0f * (float)ticks), (unsigned long long)ticks);
  Serial.println(F("   （期望值按【实际频率】计算，所以对比是严格的）"));
}

// ★ v2 修复 [1]：真正停掉发生器（原来只拉低引脚，定时器仍在写）
static void stopGen() {
  timerAlarmDisable(genTimer);
  genOn = false;
  digitalWrite(PIN_A_OUT, 0);
  digitalWrite(PIN_B_OUT, 0);
  Serial.println(F(">> 信号已停止：定时器已 disable，输出引脚已拉低。"));
  Serial.println(F("   （核对方法：敲 p，若 A/B 边沿数不再增长即为真停）"));
}

// ---- 测量窗口 --------------------------------------------------------------
static uint32_t winStart = 0;

static void resetWin() {
  noInterrupts();
  cnt4x = 0; cntA = 0; cntB = 0; jumpCnt = 0;
  interrupts();
  winStart = millis();
}

// 打印一次对比；返回"三项误差里绝对值最大的一项"（%）
static float reportOnce(bool verbose) {
  uint32_t ms = millis() - winStart;
  float t = ms / 1000.0f;
  float actualHz = 1000000.0f / (4.0f * (float)genTicks);

  noInterrupts();
  int32_t a = cntA, b = cntB, q = cnt4x;
  uint32_t j = jumpCnt;
  interrupts();

  float expA = actualHz * t;          // 每相期望边沿数
  float expQ = 4.0f * expA;           // 期望 |4x|
  float errA = (expA > 1.0f) ? ((a - expA) / expA * 100.0f) : 0.0f;
  float errB = (expA > 1.0f) ? ((b - expA) / expA * 100.0f) : 0.0f;
  float errQ = (expQ > 1.0f) ? ((fabsf((float)q) - expQ) / expQ * 100.0f) : 0.0f;

  if (verbose) {
    Serial.printf("[档位] 实际频率 %.2f Hz   窗口 %.2f s\n", actualHz, t);
    Serial.printf("   A 边沿   期望 %.0f  实测 %ld   误差 %+.3f %%\n", expA, (long)a, errA);
    Serial.printf("   B 边沿   期望 %.0f  实测 %ld   误差 %+.3f %%\n", expA, (long)b, errB);
    Serial.printf("   |4x|     期望 %.0f  实测 %ld   误差 %+.3f %%\n", expQ, (long)q, errQ);
    Serial.printf("   非法双位跳变 = %lu 次\n", (unsigned long)j);
  }

  // ★ v2 修复 [2]：判定取三项里最差的一项，避免"B 不计数也判通过"
  float worst = fabsf(errA);
  if (fabsf(errB) > worst) worst = fabsf(errB);
  if (fabsf(errQ) > worst) worst = fabsf(errQ);
  return worst;
}

static void printHelp() {
  Serial.println(F("---- MD520 中断/高速计数压力自检 v2 ----"));
  Serial.println(F(" 接线: GPIO25->GPIO18, GPIO26->GPIO19（电机 A/B 先拔掉）"));
  Serial.println(F(" s        自动扫频 100/200/500/1k/2k/5k/10k Hz，每档 3 s"));
  Serial.println(F(" f <hz>   手动设频率（例如 f 1888）"));
  Serial.println(F(" r        清零并重新计时"));
  Serial.println(F(" p        打印当前状态（也可用来验证 q 是否真停）"));
  Serial.println(F(" q        停止信号（真正 disable 定时器）"));
  Serial.println(F(" h        帮助"));
}

static void runSweep() {
  const float list[] = {100, 200, 500, 1000, 2000, 5000, 10000};
  const int n = sizeof(list) / sizeof(list[0]);
  float firstBad = 0;

  Serial.println(F("======== 扫频开始（每档 3 s）========"));
  for (int i = 0; i < n; i++) {
    setGenFreq(list[i]);
    delay(50);
    resetWin();
    delay(3000);
    float worst = reportOnce(true);
    if (worst > 0.1f && firstBad == 0) firstBad = list[i];
    Serial.printf("   → 本档最差误差 %+.3f %%  ⇒ %s\n\n",
                  worst, (worst <= 0.1f) ? "通过" : "❌ 不通过");
  }
  stopGen();
  Serial.println(F("======== 扫频结束 ========"));
  if (firstBad == 0) {
    Serial.println(F(">>> ✅ 直到 10 kHz，A/B/|4x| 三项误差都 <0.1%：中断计数余量充足。"));
    Serial.println(F("    （550rpm 只需约 1888 Hz ⇒ 余量 ≥5 倍）"));
  } else {
    Serial.printf(">>> ⚠️ 首次不通过的频率 = **%.0f Hz**：请记录为上限。\n", firstBad);
    Serial.println(F("    550rpm 需求约 1888 Hz → 若上限低于 1900 Hz，必须改用硬件计数单元或优化 ISR。"));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  delay(600);

  pinMode(PIN_A_IN, INPUT);
  pinMode(PIN_B_IN, INPUT);
  pinMode(PIN_A_OUT, OUTPUT);
  pinMode(PIN_B_OUT, OUTPUT);
  digitalWrite(PIN_A_OUT, 0);
  digitalWrite(PIN_B_OUT, 0);

  prevA = (uint8_t)digitalRead(PIN_A_IN);
  prevB = (uint8_t)digitalRead(PIN_B_IN);
  prevState = (uint8_t)((prevA << 1) | prevB);
  attachInterrupt(digitalPinToInterrupt(PIN_A_IN), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B_IN), encISR, CHANGE);

  // 定时器：做成 1 µs 一拍
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  genTimer = timerBegin(1000000);                    // 3.x：直接给频率 1 MHz
  timerAttachInterrupt(genTimer, &onGenTimer);
#else
  genTimer = timerBegin(0, 80, true);                // 2.x：80 MHz / 80 = 1 MHz
  timerAttachInterrupt(genTimer, &onGenTimer, true);
#endif
  setGenFreq(100.0f);

  resetWin();

  Serial.println();
  Serial.println(F("== MD520 中断/高速计数压力自检 v2 =="));
  Serial.printf("Arduino-ESP32 核心版本：%d.%d.%d\n",
                ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  Serial.println(F("⚠️ 接线: GPIO25->GPIO18, GPIO26->GPIO19；电机 A/B 必须先从 18/19 拔掉"));
  printHelp();
  Serial.println(F("建议先敲 s 跑一遍扫频。"));
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;
    char cmd = (char)tolower(line[0]);

    if (cmd == 'h') {
      printHelp();
    } else if (cmd == 's') {
      runSweep();
      resetWin();
    } else if (cmd == 'f') {
      float hz = line.substring(1).toFloat();
      if (hz >= 1.0f) {
        setGenFreq(hz);
        resetWin();
      } else {
        Serial.println(F(">> 用法: f 1888"));
      }
    } else if (cmd == 'r') {
      resetWin();
      Serial.println(F(">> 已清零并重新计时。"));
    } else if (cmd == 'p') {
      if (!genOn) Serial.println(F(">> 信号未运行（敲 f <hz> 或 s 启动）"));
      reportOnce(true);
    } else if (cmd == 'q') {
      stopGen();
    } else {
      Serial.println(F(">> 未知命令，敲 h 看帮助。"));
    }
  }
}
