/*
 * MD520Z19_12V 编码器台架测试  v3
 *   —— 手转数脉冲 / 正交性检查 / CPR 标定 / 空载转速表
 * ---------------------------------------------------------------------------
 * v3（2026-10-05，据 Codex A6 第二轮复核修订）：
 *   [1] **不再自动回写 CPR** —— v2 只要 A/B 非零就赋值，会让人误读成"标定通过"。
 *       现在只**打印建议命令** `s <值>`，由操作者核对本轮数据后**手动采用**；
 *       并明确提示"异常轮的 CPR 不要用于后续转速换算"。
 *   [2] 正交提示阈值由 `4 ± 0.5` 收紧为**偏离 4.000 超过 0.05 即提示复测**
 *       （并标明这是**程序提示、不是厂家验收公差**）。
 *   [3] 明确写上 **j = 0 不能证明没有丢计数**；一轮标定必须是**纯单向**。
 *   [4] 帮助文字补齐：**正反向各 10 圈 × 各 2 轮**，每轮先 `c` 清零。
 *
 * 配套文档：到货验收-MD520.md  §3（编码器功能初筛） §5（空载转速）
 * 板子    ：ESP32-WROOM-32E（Arduino IDE 选 "ESP32 Dev Module"），串口 115200
 *
 * 接线（编码器 4 根，官方支持 3.3-5V，**只用 3.3V**）：
 *   编码器 VCC -> 3V3
 *   编码器 GND -> GND      ★ AB 信号回路必须与 ESP32 共地
 *   编码器 A   -> GPIO18
 *   编码器 B   -> GPIO19
 *   电机 2 根动力线 -> 分别绝缘悬空（§3 不需要动力）
 *   ⚠️ 线序未按 到货验收-MD520.md §2.2 确认前，不要接这一步
 *   ⚠️ 若为诊断把 A/B 改接到 GPIO4/5，**必须同步改下面的 PIN_A/PIN_B 并重新编译上传**
 *      —— 只换导线，程序仍然读 18/19
 *
 * 串口命令（115200，行尾选 "换行符"）：
 *   h        帮助
 *   r        计数器清零并开始计时
 *   p        打印当前计数（含方向符号与非法跳变数）
 *   c        CPR 标定：清零 -> 单向手转整 N 圈 -> 输入圈数回车
 *   v        转速模式（用当前 CPR 换算 rpm）
 *   s <数>   手动设置 CPR（默认 209）
 *   q        退出当前模式
 *
 * 判据（详见流程文档 §3.2）：
 *   1) A、B 都应检出脉冲；零计数先做五步诊断，不要直接判单通道
 *   2) **整 10 圈、纯单向、转速稳定**下，`abs(4x)/A` 与 `abs(4x)/B` 应 **接近 4.000**
 *      （看到 3.5 / 4.5 **不要**直接判正常 —— 那只是程序提示框，不是验收公差）
 *   3) `非法双位跳变 = 0` **不能证明没有丢计数**（漏掉整组周期可能不产生非法跳变）
 *   4) 正反向**各 10 圈 × 各 2 轮**；正反向应给相同 CPR、相反符号
 *   5) 209 / 836 是"11 线 x 名义 1:19"的推算值，必须以实测为准
 */

#include <Arduino.h>

static const uint8_t PIN_A = 18;   // 不是 strapping / flash 脚
static const uint8_t PIN_B = 19;

// ---- 中断里维护的计数 ------------------------------------------------------
volatile int32_t  cnt4x    = 0;    // 4 倍频净计数（带符号 = 带方向）
volatile int32_t  cntA     = 0;    // A 相上升沿（只增）
volatile int32_t  cntB     = 0;    // B 相上升沿（只增）
volatile uint32_t jumpCnt  = 0;    // 非法双位跳变次数（00<->11）
volatile uint8_t  prevState = 0;
volatile uint8_t  prevA = 0, prevB = 0;

// 正交解码表：index = (prevState<<2) | currState -> ±1 / 0
static const int8_t QTAB[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void IRAM_ATTR encISR() {
  uint8_t a = (uint8_t)digitalRead(PIN_A);
  uint8_t b = (uint8_t)digitalRead(PIN_B);
  uint8_t curr = (uint8_t)((a << 1) | b);
  uint8_t from = prevState;

  // 两位同时变化 = 非法跳变（丢边 / 同步跳变 / 接线或电平问题）
  if ((uint8_t)(from ^ curr) == 0x03) jumpCnt++;

  cnt4x += QTAB[(from << 2) | curr];
  prevState = curr;

  if (a && !prevA) cntA++;
  if (b && !prevB) cntB++;
  prevA = a;
  prevB = b;
}

// ---- 状态 -----------------------------------------------------------------
enum Mode { MODE_IDLE, MODE_CPR_WAIT, MODE_RPM };
Mode mode = MODE_IDLE;

float    cpr      = 209.0f;   // 单通道每输出圈脉冲数（**只由 s 命令手动设置**，v3 起不再自动回写）
uint32_t tMark    = 0;
int32_t  lastA    = 0;
uint32_t lastPrint = 0;

static void resetCounters() {
  noInterrupts();
  cnt4x = 0; cntA = 0; cntB = 0; jumpCnt = 0;
  interrupts();
  tMark = millis();
  lastA = 0;
  lastPrint = millis();
}

static void printCounts() {
  noInterrupts();
  int32_t a = cntA, b = cntB, q = cnt4x;
  uint32_t j = jumpCnt;
  interrupts();

  const char *dir = (q > 0) ? "正向(+)" : (q < 0) ? "反向(-)" : "静止/净位移为0";
  Serial.printf("[计数] A=%ld  B=%ld  4x=%ld (%s)  非法跳变=%lu  已计时 %.2f s\n",
                (long)a, (long)b, (long)q, dir, (unsigned long)j,
                (millis() - tMark) / 1000.0f);
}

static void printHelp() {
  Serial.println(F("---- MD520 编码器台架 v3 ----"));
  Serial.println(F(" h        帮助"));
  Serial.println(F(" r        计数器清零并开始计时"));
  Serial.println(F(" p        打印当前计数（含方向符号与非法跳变）"));
  Serial.println(F(" c        CPR 标定：清零 -> 单向手转整 N 圈 -> 输入圈数回车"));
  Serial.println(F(" v        转速模式（用当前 CPR 换算 rpm）"));
  Serial.println(F(" s <数>   手动设置 CPR（默认 209）"));
  Serial.println(F(" q        退出当前模式"));
  Serial.printf (" 当前生效的 CPR = %.2f\n", cpr);
  Serial.println(F(" 测试协议：正方向 10 圈 ×2 轮 + 反方向 10 圈 ×2 轮；【每轮先敲 c 清零】，"));
  Serial.println(F("           停稳后再开始、纯单向、起止标记固定（换方向前先取尽齿隙）。"));
  Serial.println(F(" 提示：标定不会自动采用，核对数据后用 s 手动输入建议值。"));
}

// 标定结果报告
static void reportCalib(float turns) {
  noInterrupts();
  int32_t a = cntA, b = cntB, q = cnt4x;
  uint32_t j = jumpCnt;
  interrupts();

  float secs = (millis() - tMark) / 1000.0f;
  float cprA = a / turns;
  float cprB = b / turns;
  float rA = (a != 0) ? (fabsf((float)q) / (float)a) : 0.0f;
  float rB = (b != 0) ? (fabsf((float)q) / (float)b) : 0.0f;

  Serial.println(F("------ CPR 标定结果 ------"));
  Serial.printf("圈数 %.3f   用时 %.2f s\n", turns, secs);
  Serial.printf("A 上升沿/圈 = %.2f      B 上升沿/圈 = %.2f\n", cprA, cprB);
  Serial.printf("abs(4x)/圈  = %.2f\n", fabsf((float)q) / turns);
  Serial.printf("abs(4x)/A = %.3f     abs(4x)/B = %.3f   (纯单向整圈应接近 4.000)\n", rA, rB);
  Serial.printf("净方向符号 q = %ld  -> %s   (符号本身不是故障，只需固定约定)\n",
                (long)q, (q > 0) ? "正向(+)" : (q < 0) ? "反向(-)" : "未转动/净位移0");
  Serial.printf("非法双位跳变 = %lu 次   (应接近 0)\n", (unsigned long)j);

  if (a == 0 || b == 0) {
    Serial.println(F(">>> ⚠️ 本次未检出 A 和/或 B 脉冲。这【不等于】已确认单通道："));
    Serial.println(F("    请按 到货验收-MD520.md §3.3 走五步诊断："));
    Serial.println(F("    1 查 3.3V 供电与插接  2 核线序(丝印/官方图/卖家说明)"));
    Serial.println(F("    3 换 GPIO 复测        4 交换 A/B 看现象是否随之移动"));
    Serial.println(F("    5 用已知正常信号源验证采集通道"));
    Serial.println(F("    五步走完仍缺一相且与引脚无关 -> 才写“确认缺通道”，按项目规定退货。"));
  } else {
    // ★ v3：不再自动赋值 —— 只给"待采用"的建议值，由操作者核对后手动 s
    Serial.printf(">>> 建议采用值：  s %.2f      （= A 相上升沿 / 输出轴圈）\n", cprA);
    Serial.println(F("    ⚠️ 程序【不会】自动采用。先核对本轮数据是否满足："));
    Serial.println(F("       ① 非法跳变 j = 0   ② abs(4x)/A 与 /B 接近 4.000"));
    Serial.println(F("       ③ 纯单向、整圈、起止标记固定   ④ 圈数填对了"));
    Serial.println(F("    ⚠️ 本轮若判为异常，这个 CPR【不要】用于后续转速换算。"));
    Serial.println(F("    （当前生效的 CPR 仍是 printHelp 里显示的值，直到你手动 s 为止）"));
    Serial.println(F("    下一步：反方向 10 圈 ×2 轮、同方向再 1 轮（每轮先敲 c 清零）。"));
  }

  if (j > 0) {
    Serial.println(F(">>> ⚠️ 出现非法双位跳变：可能有丢边/两相同时跳变/接线或电平问题。"));
    Serial.println(F("    请手转更慢复测；仍出现则记录原始 A/B/q/圈数并核对线序。"));
  } else {
    Serial.println(F("    注：j = 0 【不能】证明没有丢计数（漏掉整组周期也可能不产生非法跳变）。"));
  }
  if (a != 0 && b != 0 && (fabsf(rA - 4.0f) > 0.05f || fabsf(rB - 4.0f) > 0.05f)) {
    Serial.println(F(">>> ⚠️ abs(4x)/A 或 /B 偏离 4.000 超过 0.05。"));
    Serial.println(F("    （这是程序提示，不是厂家验收公差）先慢转复测；仍有偏差请保留原始数据，"));
    Serial.println(F("    并查：是否混入反向行程 / 起止边界 / 接触不良 / 方向解码。"));
  }
  mode = MODE_IDLE;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);   // 别让 readStringUntil 在转速模式下长时间阻塞
  delay(600);

  pinMode(PIN_A, INPUT);   // 官方说明：编码器内置上拉整形，可直接读
  pinMode(PIN_B, INPUT);
  prevA = (uint8_t)digitalRead(PIN_A);
  prevB = (uint8_t)digitalRead(PIN_B);
  prevState = (uint8_t)((prevA << 1) | prevB);

  attachInterrupt(digitalPinToInterrupt(PIN_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), encISR, CHANGE);

  tMark = millis();
  Serial.println();
  Serial.println(F("== MD520 编码器台架测试 v3 =="));
  Serial.println(F("接线: VCC->3V3, GND->GND, A->GPIO18, B->GPIO19; 动力线绝缘悬空"));
  Serial.println(F("前提: 线序已按 到货验收-MD520.md §2.2 确认（本批已实物确认 M+ M- GND VCC B A）"));
  Serial.println(F("提醒: 先做完上电前检查（线缆两端拔开的逐线/跨线短接检查），再上电。"));
  printHelp();
}

void loop() {
  // 串口命令
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    char cmd = (char)tolower(line[0]);

    if (cmd == 'h') {
      printHelp();
    } else if (cmd == 'r') {
      resetCounters();
      Serial.println(F("已清零。请开始手转输出轴（慢慢匀速，尽量单向）。"));
    } else if (cmd == 'p') {
      printCounts();
    } else if (cmd == 'c') {
      resetCounters();
      mode = MODE_CPR_WAIT;
      Serial.println(F(">> CPR 标定：请【停稳后】单向缓慢转动输出轴【整 N 圈】（建议 10 圈，"));
      Serial.println(F("   固定起止标记以避免回程间隙）。转完【停稳】再输入圈数回车（例如 10）。"));
      Serial.println(F("   一轮里不要来回找标记；换方向前先取尽齿隙、并重新敲 c 清零。"));
    } else if (cmd == 'v') {
      resetCounters();
      mode = MODE_RPM;
      Serial.printf(">> 转速模式：CPR=%.2f，每 0.5 s 打印 rpm。\n", cpr);
      Serial.println(F("   ⚠️ 需 12V 动力，且【编码器 GND 必须与 ESP32 GND 共地】。"));
      Serial.println(F("   ⚠️ 手转正常不代表高速计数准确：550rpm 时 A 约 1916 沿/s。"));
    } else if (cmd == 's') {
      float v = line.substring(1).toFloat();
      if (v > 1.0f) {
        cpr = v;
        Serial.printf(">> CPR 已设为 %.2f\n", cpr);
      } else {
        Serial.println(F(">> 用法: s 209"));
      }
    } else if (cmd == 'q') {
      mode = MODE_IDLE;
      Serial.println(F(">> 回到空闲模式。"));
    } else if (mode == MODE_CPR_WAIT) {
      float turns = line.toFloat();
      if (turns >= 0.5f) {
        reportCalib(turns);
      } else {
        Serial.println(F(">> 请输入圈数（正数，例如 10）"));
      }
    } else {
      Serial.println(F(">> 未知命令，敲 h 看帮助。"));
    }
  }

  // 转速模式：每 0.5 s 用 A 相脉冲算一次 rpm
  if (mode == MODE_RPM && (millis() - lastPrint) >= 500) {
    uint32_t now = millis();
    float dt = (now - lastPrint) / 1000.0f;
    lastPrint = now;

    noInterrupts();
    int32_t a = cntA;
    uint32_t j = jumpCnt;
    interrupts();

    int32_t dA = a - lastA;
    lastA = a;

    float rpm = (dA / cpr) / dt * 60.0f;
    Serial.printf("[转速] %.1f rpm   (dr=%ld pulse / %.2fs, CPR=%.2f, 非法跳变=%lu)\n",
                  rpm, (long)dA, dt, cpr, (unsigned long)j);
  }
}
