/*
 * VL53L0X 首件验货 · 第 2 步：XSHUT 功能测试
 *
 * 为什么必须先测它：5 个 VL53L0X 出厂地址都是 0x29，只能靠 XSHUT
 * 逐个上电、逐个改地址。XSHUT 无效 → 5 路测距方案直接作废。
 *
 * 板：ESP32-WROOM-32E
 * 接线：VCC→3V3  GND→GND  SDA→GPIO21  SCL→GPIO22
 *       XSH→GPIO4   ← 注意：本步不再接 3V3，改接 GPIO4
 *
 * 判据：拉低后 0x29 从总线消失；拉高后重新出现
 */

#include <Wire.h>

#define PIN_SDA    21
#define PIN_SCL    22
#define PIN_XSHUT   4
#define ADDR_L0X   0x29

bool present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  pinMode(PIN_XSHUT, OUTPUT);

  Serial.println();
  Serial.println("===== VL53L0X 验货 · 第 2 步：XSHUT =====");

  Serial.println("--- XSHUT = HIGH（应能扫到 0x29）");
  digitalWrite(PIN_XSHUT, HIGH);
  delay(100);
  Serial.printf("    0x29 存在？ %s\n", present(ADDR_L0X) ? "✅ 是" : "❌ 否");

  Serial.println("--- XSHUT = LOW（应从总线消失）");
  digitalWrite(PIN_XSHUT, LOW);
  delay(100);
  Serial.printf("    0x29 存在？ %s\n", present(ADDR_L0X)
                ? "❌ 仍在 —— XSHUT 无效，5 路方案作废" : "✅ 已消失（XSHUT 有效）");

  Serial.println("--- XSHUT = HIGH（应重新出现）");
  digitalWrite(PIN_XSHUT, HIGH);
  delay(100);
  Serial.printf("    0x29 存在？ %s\n", present(ADDR_L0X) ? "✅ 是" : "❌ 否");

  // ---- 改地址演练：为第二批 4 个模块做准备 ----
  Serial.println("--- 改地址演练：0x29 → 0x30");
  digitalWrite(PIN_XSHUT, LOW);
  delay(50);
  digitalWrite(PIN_XSHUT, HIGH);   // 刚上电，应处于出厂默认地址
  delay(50);

  Wire.beginTransmission(ADDR_L0X);
  Wire.write(0x8A);                // I2C_SLAVE_DEVICE_ADDRESS
  Wire.write(0x30);                // 新地址（低 7 位）
  uint8_t err = Wire.endTransmission();
  Serial.printf("    写入 0x8A 返回：%u %s\n", err, err == 0 ? "✅" : "❌");
  delay(50);

  Serial.printf("    新地址 0x30 应答？ %s\n", present(0x30) ? "✅ 是" : "❌ 否");
  Serial.printf("    旧地址 0x29 应答？ %s（应为否）\n",
                present(ADDR_L0X) ? "⚠️ 仍在 —— 改地址可能没生效" : "✅ 已不在");

  Serial.println();
  Serial.println("提示：改地址是 RAM 里的，重新上电后会回到 0x29。");
  Serial.println("      若改地址失败：断电重来，趁刚上电立即写 0x8A。");
}

void loop() {
  delay(1000);
}
