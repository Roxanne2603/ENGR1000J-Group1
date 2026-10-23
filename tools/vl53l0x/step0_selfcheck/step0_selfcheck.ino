/*
 * VL53L0X 首件验收 · 第 0 步：程序侧自检
 *
 * 用途：在【不用万用表】的前提下，拿到"供电与总线是否够用"的程序侧证据。
 *
 * ⚠️ 它【不能】替代什么（别把本 sketch 的结论说过头）：
 *   - 不能测 VCC–GND 电阻（短路）           ← 但本件已成功上电，A1 事实上已过
 *   - 不能测模块实际拿到的电压（A2 仍标注"未测"）
 *   - 不能判定板上 U2 是不是 LDO（A3 仍标注"未闭环"；
 *     不过我们一律用 3.3V、不需要 5V，所以该问题不影响任何决策）
 *   - 不能测电流（A4 仍标注"未测"）
 *
 * ✅ 它能给出：
 *   ① 复位原因 —— 若出现 BROWNOUT，说明供电塌过
 *   ② 启动计数 —— 存在 NVS，跑完一圈回来看数字有没有变（= 有没有偷偷复位）
 *   ③ 总线裕量 —— 100 kHz 与 400 kHz 各读 200 次的有效数，判断上拉/线长/供电是否勉强
 *   ④ 心跳 —— 确认程序一直活着
 *
 * 长跑的有效率 / 最大连续失败 / 最大有效读数间隔 → 用 step3_range v3 做。
 *
 * 板：ESP32-WROOM-32E ｜ 接线：VCC→3V3 GND→GND SDA→GPIO21 SCL→GPIO22 XSH→3V3
 */

#include <Wire.h>
#include <Preferences.h>
#include "esp_system.h"

#define PIN_SDA   21
#define PIN_SCL   22
#define ADDR_L0X  0x29
#define ID_REG    0xC0
#define ID_VALUE  0xEE
#define BUS_TRIES 200

Preferences prefs;
uint32_t bootCount = 0;

const char* rstReason(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON   正常上电 / 按了 EN";
    case ESP_RST_EXT:       return "EXT       外部复位脚";
    case ESP_RST_SW:        return "SW        软件重启";
    case ESP_RST_PANIC:     return "PANIC     程序崩溃";
    case ESP_RST_INT_WDT:   return "INT_WDT   中断看门狗";
    case ESP_RST_TASK_WDT:  return "TASK_WDT  任务看门狗";
    case ESP_RST_WDT:       return "WDT       其它看门狗";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP 深睡唤醒";
    case ESP_RST_BROWNOUT:  return ">>> BROWNOUT  供电塌过！ <<<";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

// 在指定 I2C 时钟下读 200 次型号 ID，返回成功次数
int busTest(uint32_t hz) {
  Wire.setClock(hz);
  delay(10);
  int ok = 0;
  for (int i = 0; i < BUS_TRIES; i++) {
    Wire.beginTransmission(ADDR_L0X);
    Wire.write(ID_REG);
    if (Wire.endTransmission(false) != 0) continue;
    if (Wire.requestFrom((uint8_t)ADDR_L0X, (size_t)1) != 1) continue;
    if (Wire.read() == ID_VALUE) ok++;
  }
  return ok;
}

void setup() {
  Serial.begin(115200);
  delay(1200);

  esp_reset_reason_t r = esp_reset_reason();

  prefs.begin("vlcheck", false);
  bootCount = prefs.getUInt("boot", 0) + 1;
  prefs.putUInt("boot", bootCount);

  Wire.begin(PIN_SDA, PIN_SCL);

  Serial.println();
  Serial.println("===== VL53L0X 第 0 步：程序侧自检 =====");
  Serial.printf("复位原因   : %s\n", rstReason(r));
  Serial.printf("启动计数   : 第 %u 次（存在 NVS，掉电不丢）\n", (unsigned)bootCount);
  Serial.println("             ↑ 跑完一圈回来看这个数字有没有变；变了 = 偷偷复位过");
  Serial.println("             ↑ 若复位原因显示 BROWNOUT → 供电塌过，这条比万用表更能说明问题");
  Serial.println();

  if (r == ESP_RST_BROWNOUT) {
    Serial.println("!!!!! 本次是 BROWNOUT 复位 —— 先查供电（线细/接触/负载），再谈其它 !!!!!");
    Serial.println();
  }

  // ── 总线裕量：100 kHz vs 400 kHz ──
  Serial.println("--- 总线裕量测试（各读 200 次型号 ID）");
  int ok100 = busTest(100000);
  Serial.printf("    100 kHz : 有效 %3d / %d\n", ok100, BUS_TRIES);
  int ok400 = busTest(400000);
  Serial.printf("    400 kHz : 有效 %3d / %d\n", ok400, BUS_TRIES);
  Wire.setClock(400000);

  Serial.println();
  Serial.println("怎么读：");
  Serial.println("  两档都 200/200        → 总线裕量好 ✅");
  Serial.println("  100k 好、400k 明显差  → 上拉/线长/供电勉强 ⚠️（先用 100 kHz 跑）");
  Serial.println("  两档都差              → 先查接线（SDA/SCL 是否接反、插到底、XSH 接高）");
  Serial.println();

  // ── 回归：扫描 + 型号 ID ──
  Serial.println("--- 回归确认");
  Wire.beginTransmission(ADDR_L0X);
  if (Wire.endTransmission() != 0) {
    Serial.println("    !! 0x29 无应答 —— 回到 step1 排查，不要继续");
  } else {
    Wire.beginTransmission(ADDR_L0X);
    Wire.write(ID_REG);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)ADDR_L0X, (size_t)1);
    uint8_t id = Wire.available() ? Wire.read() : 0xFF;
    Serial.printf("    0x29 应答 ✅   型号 ID 0xC0 = 0x%02X %s\n",
                  id, id == ID_VALUE ? "✅" : "⚠️");
  }

  Serial.println();
  Serial.println("下一步：");
  Serial.println("  1) 记下【复位原因】与【启动计数】");
  Serial.println("  2) 换上 step3_range（v3）跑长跑，看有效率 / 最大连续失败 / 最大有效读数间隔");
  Serial.println("  3) 跑完回来再看本 sketch 的启动计数有没有变");
  Serial.println();
  Serial.println("（万用表的 A2/A4 仍标『未测』—— 本 sketch 只是代理，不能宣称已测）");
}

void loop() {
  static uint32_t t = 0;
  if (millis() - t >= 10000) {
    t = millis();
    Serial.printf("[心跳] 运行 %u s，启动计数 %u（若这行重新从 0 开始，说明复位了）\n",
                  (unsigned)(millis() / 1000), (unsigned)bootCount);
  }
  delay(50);
}
