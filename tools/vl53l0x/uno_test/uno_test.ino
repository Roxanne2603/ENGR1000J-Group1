/* UNO R3 / ATmega328P, one VL53L0X. A14, 2026-10-09.
 * Use a bidirectional I2C level shifter: UNO A4/A5 on 5V side,
 * sensor SDA/SCL on 3.3V side. Sensor VCC remains 3.3V.
 * Library: Adafruit VL53L0X. Original ESP sketches are unchanged.
 * CSV and summary describe this bench run, not vehicle safety.
 */
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <math.h>

const uint16_t TRUE_MM = 200; // Change to measured 100/200/300/500.
const bool XSH_GPIO_TEST = false; // Only true with optional D7 wiring.
const uint8_t XSH_PIN = 7;
Adafruit_VL53L0X lox;
bool ready = false, running = false;
uint32_t startMs, durationMs, attempts, validN, apiErrors, wireErrors;
uint32_t firstValid, lastValid, maxValidGap, failRun, maxFail;
uint32_t prevAttempt, attemptGaps, maxAttemptGap, maxCallMs;
uint32_t statusCounts[8];
float mean, m2, sumAttemptGap, sumValidGap;
uint16_t minRange, maxRange, maxError;

void clearWireFlag() {
#if defined(WIRE_HAS_TIMEOUT)
  Wire.clearWireTimeoutFlag();
#endif
}
bool wireTimedOut() {
#if defined(WIRE_HAS_TIMEOUT)
  return Wire.getWireTimeoutFlag();
#else
  return false;
#endif
}
bool readByte(uint8_t reg, uint8_t &value) {
  clearWireFlag();
  Wire.beginTransmission(0x29);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)0x29, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return !wireTimedOut();
}
bool ack29() {
  clearWireFlag();
  Wire.beginTransmission(0x29);
  return Wire.endTransmission() == 0 && !wireTimedOut();
}
void scan() {
  Serial.println(F("SCAN (one sensor, 100kHz):"));
  uint8_t found = 0;
  for (uint8_t address = 1; address < 127; address++) {
    clearWireFlag();
    Wire.beginTransmission(address);
    uint8_t e = Wire.endTransmission();
    if (e == 0 && !wireTimedOut()) {
      Serial.print(F("ACK 0x")); Serial.println(address, HEX); found++;
    }
    if (wireTimedOut()) { Serial.println(F("WIRE_TIMEOUT: check bus")); break; }
  }
  if (!found) Serial.println(F("NO_ACK: check wiring/power; not a return verdict"));
  uint8_t model, rev;
  if (readByte(0xC0, model) && readByte(0xC2, rev)) {
    Serial.print(F("model=0x")); Serial.print(model, HEX);
    Serial.print(F(" revision=0x")); Serial.println(rev, HEX);
    Serial.println(F("Fresh reset reference: EE / 10; not proof of quality"));
  }
}
void xshTest() {
  if (!XSH_GPIO_TEST) {
    Serial.println(F("XSH test disabled: see optional D7 / 10k wiring")); return;
  }
  ready = false;
  digitalWrite(XSH_PIN, LOW); pinMode(XSH_PIN, INPUT); delay(100);
  Serial.print(F("XSH released ACK=")); Serial.println(ack29());
  digitalWrite(XSH_PIN, LOW); pinMode(XSH_PIN, OUTPUT); delay(100);
  Serial.print(F("XSH low ACK=")); Serial.println(ack29());
  pinMode(XSH_PIN, INPUT); // Release only: NEVER OUTPUT HIGH / INPUT_PULLUP.
  delay(100);
  Serial.print(F("XSH released ACK=")); Serial.println(ack29());
  Serial.println(F("Expected 1,0,1. Address reset only; multi-sensor not tested."));
}
void summary(uint32_t endMs) {
  running = false;
  uint32_t elapsed = endMs - startMs;
  uint32_t head = validN ? firstValid - startMs : elapsed;
  uint32_t tail = validN ? endMs - lastValid : elapsed;
  uint32_t worst = max(head, max(tail, maxValidGap));
  Serial.println(F("# SUMMARY / observed finite run"));
  Serial.print(F("# true_mm=")); Serial.println(TRUE_MM);
  Serial.print(F("# actual_ms=")); Serial.println(elapsed);
  Serial.print(F("# attempts=")); Serial.print(attempts);
  Serial.print(F(" valid=")); Serial.print(validN);
  Serial.print(F(" api_errors=")); Serial.print(apiErrors);
  Serial.print(F(" wire_timeouts=")); Serial.println(wireErrors);
  Serial.print(F("# valid_percent="));
  if (attempts) Serial.println(100.0f * validN / attempts, 2); else Serial.println(F("NA"));
  Serial.print(F("# valid_per_second_whole_run="));
  if (elapsed) Serial.println(1000.0f * validN / elapsed, 2); else Serial.println(F("NA"));
  Serial.print(F("# status_bins_0_to_6_and_other="));
  for (uint8_t i = 0; i < 8; i++) { Serial.print(statusCounts[i]); Serial.print(i == 7 ? '\n' : ','); }
  if (validN) {
    Serial.print(F("# mean_mm=")); Serial.print(mean, 2);
    Serial.print(F(" bias_mm=")); Serial.println(mean - TRUE_MM, 2);
    Serial.print(F("# population_sd_mm=")); Serial.println(sqrt(m2 / validN), 3);
    Serial.print(F("# sample_sd_mm="));
    if (validN > 1) Serial.println(sqrt(m2 / (validN - 1)), 3); else Serial.println(F("NA"));
    Serial.print(F("# min_mm=")); Serial.print(minRange);
    Serial.print(F(" max_mm=")); Serial.print(maxRange);
    Serial.print(F(" max_abs_error_mm=")); Serial.println(maxError);
  } else Serial.println(F("# NO_VALID_SAMPLES"));
  Serial.print(F("# attempt_gap_mean_ms="));
  if (attemptGaps) Serial.println(sumAttemptGap / attemptGaps, 2); else Serial.println(F("NA"));
  Serial.print(F("# attempt_gap_max_ms=")); Serial.println(maxAttemptGap);
  Serial.print(F("# complete_valid_gap_mean_ms="));
  if (validN > 1) Serial.println(sumValidGap / (validN - 1), 2); else Serial.println(F("NA"));
  Serial.print(F("# complete_valid_gap_max_ms="));
  if (validN > 1) Serial.println(maxValidGap); else Serial.println(F("NA"));
  Serial.print(F("# first_valid_wait_ms=")); Serial.println(head);
  Serial.print(F("# tail_age_observed_ms=")); Serial.println(tail);
  Serial.print(F("# max_no_new_data_observed_ms=")); Serial.println(worst);
  Serial.print(F("# max_consecutive_failures=")); Serial.println(maxFail);
  Serial.print(F("# max_call_ms=")); Serial.println(maxCallMs);
  Serial.println(F("# Head/tail are boundary observations; no future latency bound."));
  Serial.println(F("DONE. 1=scan 2=20s 3=300s 0=stop x=optional XSH"));
}
void startRun(uint32_t seconds) {
  if (!ready) {
    clearWireFlag();
    Serial.println(F("INIT Adafruit VL53L0X DEFAULT..."));
    bool ok = lox.begin();
    Wire.setClock(100000);
    if (!ok || wireTimedOut()) {
      Serial.println(F("INIT_FAILED: check bus/power/reset; do not infer bad sensor")); return;
    }
    ready = true;
  }
  attempts = validN = apiErrors = wireErrors = 0;
  firstValid = lastValid = maxValidGap = failRun = maxFail = 0;
  prevAttempt = attemptGaps = maxAttemptGap = maxCallMs = 0;
  mean = m2 = sumAttemptGap = sumValidGap = 0;
  minRange = 65535; maxRange = maxError = 0;
  for (uint8_t i = 0; i < 8; i++) statusCounts[i] = 0;
  durationMs = seconds * 1000UL;
  Serial.println(F("time_ms,true_mm,range_mm,api_error,range_status,wire_timeout,call_ms,valid"));
  Serial.flush(); startMs = millis(); running = true;
}
void setup() {
  if (XSH_GPIO_TEST) { digitalWrite(XSH_PIN, LOW); pinMode(XSH_PIN, INPUT); }
  Serial.begin(115200);
  Wire.begin(); Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true); // Bus protection, not vehicle response guarantee.
#endif
  delay(500);
  Serial.println(F("UNO VL53L0X A14 v1 / I2C LEVEL SHIFTER REQUIRED"));
  Serial.println(F("1=scan+ID 2=20s 3=300s 0=stop x=optional XSH"));
  Serial.print(F("TRUE_MM=")); Serial.println(TRUE_MM);
}
void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '0' && running) { summary(millis()); return; }
    if (!running) {
      if (c == '1') scan();
      else if (c == '2') startRun(20);
      else if (c == '3') startRun(300);
      else if (c == 'x' || c == 'X') xshTest();
    }
  }
  if (!running) return;
  if ((uint32_t)(millis() - startMs) >= durationMs) { summary(millis()); return; }
  clearWireFlag();
  VL53L0X_RangingMeasurementData_t m = {};
  uint32_t callStart = millis();
  VL53L0X_Error err = lox.rangingTest(&m, false);
  uint32_t now = millis(), callMs = now - callStart;
  bool wt = wireTimedOut();
  bool valid = err == VL53L0X_ERROR_NONE && !wt && m.RangeStatus == 0;
  attempts++;
  if (err != VL53L0X_ERROR_NONE) apiErrors++;
  if (wt) wireErrors++;
  if (callMs > maxCallMs) maxCallMs = callMs;
  if (attempts > 1) {
    uint32_t gap = now - prevAttempt;
    sumAttemptGap += gap; attemptGaps++;
    if (gap > maxAttemptGap) maxAttemptGap = gap;
  }
  prevAttempt = now;
  if (err == VL53L0X_ERROR_NONE && !wt) statusCounts[m.RangeStatus < 7 ? m.RangeStatus : 7]++;
  if (valid) {
    if (!validN) firstValid = now;
    else {
      uint32_t gap = now - lastValid; sumValidGap += gap;
      if (gap > maxValidGap) maxValidGap = gap;
    }
    validN++; lastValid = now; failRun = 0;
    float delta = m.RangeMilliMeter - mean;
    mean += delta / validN; m2 += delta * (m.RangeMilliMeter - mean);
    minRange = min(minRange, m.RangeMilliMeter); maxRange = max(maxRange, m.RangeMilliMeter);
    uint16_t e = m.RangeMilliMeter > TRUE_MM ? m.RangeMilliMeter - TRUE_MM : TRUE_MM - m.RangeMilliMeter;
    maxError = max(maxError, e);
  } else { failRun++; if (failRun > maxFail) maxFail = failRun; }
  Serial.print(now); Serial.print(','); Serial.print(TRUE_MM); Serial.print(',');
  if (err == VL53L0X_ERROR_NONE && !wt) Serial.print(m.RangeMilliMeter);
  Serial.print(','); Serial.print((int)err); Serial.print(',');
  if (err == VL53L0X_ERROR_NONE && !wt) Serial.print(m.RangeStatus); else Serial.print(-1);
  Serial.print(','); Serial.print(wt ? 1 : 0); Serial.print(',');
  Serial.print(callMs); Serial.print(','); Serial.println(valid ? 1 : 0);
  if ((uint32_t)(millis() - startMs) >= durationMs) summary(millis());
}
