/*
 * VL53L0X 首件验货 · 第 1 步：I2C 扫描 + 读型号 ID
 *
 * 不需要任何第三方库，只用 Wire.h
 * 板：ESP32-WROOM-32E（经典 ESP32）
 * 接线：VCC→3V3  GND→GND  SDA→GPIO21  SCL→GPIO22  XSH→3V3
 *
 * 判据：扫到 0x29；0xC0 读回 0xEE
 * 扫不到 0x29 → 直接退货，不用往下走
 */

#include <Wire.h>

#define PIN_SDA   21
#define PIN_SCL   22
#define ADDR_L0X  0x29

void scanBus() {
  int n = 0;
  for (uint8_t a = 0x01; a < 0x7F; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  发现设备 @ 0x%02X\n", a);
      n++;
    }
  }
  if (n == 0) Serial.println("  （总线上没有任何设备）");
}

uint8_t readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom((uint8_t)addr, (size_t)1) != 1) return 0xFF;
  return Wire.read();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);          // 先用 100 kHz，稳

  Serial.println();
  Serial.println("===== VL53L0X 验货 · 第 1 步：I2C 扫描 + 型号 ID =====");

  Serial.println("[1] 扫描 I2C 总线：");
  scanBus();

  Serial.println("[2] 读 0x29 的型号信息：");
  Wire.beginTransmission(ADDR_L0X);
  if (Wire.endTransmission() != 0) {
    Serial.println("  ❌ 0x29 无应答。依次检查：");
    Serial.println("     - VCC 是否真的接到 3.3V（不是 5V）");
    Serial.println("     - SDA/SCL 是否接反");
    Serial.println("     - XSH 是否接到了 3V3（悬空 = 模块不使能）");
    return;
  }

  uint8_t id  = readReg(ADDR_L0X, 0xC0);   // IDENTIFICATION_MODEL_ID
  uint8_t rev = readReg(ADDR_L0X, 0xC2);   // revision

  Serial.printf("  0xC0 (型号 ID)   = 0x%02X   %s\n", id,
                id == 0xEE ? "✅ 符合 VL53L0X" : "⚠️ 不是 0xEE —— 记下实际值，别急着判假货");
  Serial.printf("  0xC2 (revision)  = 0x%02X   %s\n", rev,
                (rev == 0x00 || rev == 0xFF) ? "⚠️ 可疑（疑似通信异常）" : "✅ 有合理读数");

  Serial.println();
  Serial.println("下一步：把 XSH 从 3V3 改接到 GPIO4，跑 step2_xshut");
}

void loop() {
  delay(1000);
}
