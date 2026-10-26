// ESP32-WROOM-32E / ESP32 Dev Module / Arduino-ESP32 3.x
// Serial monitor 115200. Send ONE command at a time; Enter is optional.
// Servo signal GPIO14, separate verified 5V, common GND.
// Keep rear motor power disconnected. Lift front wheels.
// v5: serial presets 150/400/650/800/1000us; default650, bounded targets.
// User reports 1375us approximately centered; actual angles unmeasured.
#include <Arduino.h>

constexpr uint8_t SERVO_PIN = 14;
constexpr int DEFAULT_CENTER_US = 1375; // Edit this after center calibration.
constexpr int DEFAULT_SPAN_US = 650;
constexpr int MIN_US = 500;             // Provisional command guard, NOT a hardware rating.
constexpr int MAX_US = 2500;
static_assert(DEFAULT_CENTER_US - DEFAULT_SPAN_US >= MIN_US &&
              DEFAULT_CENTER_US + DEFAULT_SPAN_US <= MAX_US,
              "Default targets must fit the test envelope.");
constexpr uint32_t TIMEOUT_MS = 15000;
int centerUs = DEFAULT_CENTER_US;
int outputUs = DEFAULT_CENTER_US;
int targetUs = DEFAULT_CENTER_US;
int spanUs = DEFAULT_SPAN_US;
bool enabled = false;
uint32_t lastCommandMs = 0, lastStepMs = 0;

int steeringTarget(int direction) {
  return constrain(centerUs + direction * spanUs, MIN_US, MAX_US);
}

void printConfig() {
  Serial.printf("CONFIG span=%d left=%d right=%d limits=%d..%d us; actual spans left=%d right=%d\n",
                spanUs, steeringTarget(-1), steeringTarget(+1), MIN_US, MAX_US,
                centerUs - steeringTarget(-1), steeringTarget(+1) - centerUs);
}

void selectSpan(int requested) {
  if (enabled && (outputUs != centerUs || targetUs != centerUs)) {
    Serial.println("PRESET refused: send 1, wait until centered, then select preset.");
    return;
  }
  spanUs = requested;
  Serial.println("PRESET selected; no movement. Send 2 or 3 separately.");
  printConfig();
}

void stopServo() {
  if (enabled) {
    ledcWrite(SERVO_PIN, 0);
    ledcDetach(SERVO_PIN);
  }
  enabled = false;
  digitalWrite(SERVO_PIN, LOW);
  pinMode(SERVO_PIN, OUTPUT);
}

bool writePulse(int us) {
  const uint32_t duty = (uint64_t(us) * 32768 + 10000) / 20000;
  return ledcWrite(SERVO_PIN, duty);
}

void moveTo(int us) {
  if (us < MIN_US || us > MAX_US) {
    Serial.println("LIMIT: command rejected.");
    return;
  }
  if (!enabled) {
    Serial.println("Send 1 first to enable at center.");
    return;
  }
  targetUs = us;
  lastCommandMs = millis();
  Serial.printf("TARGET=%dus CENTER=%dus (commands, not measured angles)\n", targetUs, centerUs);
}

void centerServo() {
  if (!enabled) {
    enabled = ledcAttachChannel(SERVO_PIN, 50, 15, 8);
    if (!enabled) {
      stopServo();
      Serial.println("FAULT: PWM attach failed.");
      return;
    }
    if (!writePulse(centerUs)) {
      stopServo();
      Serial.println("FAULT: PWM write failed.");
      return;
    }
    outputUs = targetUs = centerUs;
    lastCommandMs = lastStepMs = millis();
    Serial.printf("ENABLED center=%dus. First movement may be sudden.\n", centerUs);
  } else {
    moveTo(centerUs);
  }
}

void trimCenter(int delta) {
  if (!enabled) { Serial.println("Send 1 first."); return; }
  const int next = centerUs + delta;
  if (next < DEFAULT_CENTER_US - 25 || next > DEFAULT_CENTER_US + 25) {
    Serial.println("LIMIT: center trim restricted to default center +/-25us.");
    return;
  }
  centerUs = next;
  moveTo(centerUs);
  Serial.printf("CENTER=%dus; RAM only. Set DEFAULT_CENTER_US to keep it after reboot.\n", centerUs);
  printConfig();
}

void setup() {
  const uint8_t motorPins[] = {25, 26, 32, 33};
  for (uint8_t pin : motorPins) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  stopServo();
  Serial.begin(115200);
  delay(300);
  Serial.println("CAR STEERING v5 - PWM OFF AT BOOT - GPIO14");
  Serial.println("1/c=center+enable; 2/l=left; 3/r=right; 0/x/!=PWM OFF.");
  Serial.println("-/+=center trim -/+5us; 9=status. 15s idle stops PWM.");
  Serial.println("PRESETS: 4=150us 5=400us 6=650us 7=800us 8=1000us. Default650.");
  Serial.printf("Center=%dus left=%dus right=%dus. Observe actual directions.\n",
                centerUs, steeringTarget(-1), steeringTarget(+1));
  printConfig();
  Serial.println("Cut physical servo power if jammed. PWM OFF does not cut power.");
}

void loop() {
  uint32_t now = millis();
  if (enabled && uint32_t(now - lastCommandMs) >= TIMEOUT_MS) {
    stopServo();
    Serial.println("TIMEOUT: PWM OFF. Send 1 to enable again.");
  }
  char command = 0;
  bool stopRequested = false;
  for (uint8_t n = 0; n < 32 && Serial.available(); ++n) {
    const char c = char(Serial.read());
    if (c == '0' || c == 'x' || c == '!') stopRequested = true;
    else if (c != '\r' && c != '\n' && c != ' ') command = c;
  }
  if (stopRequested) { stopServo(); Serial.println("STOP: PWM OFF."); return; }
  switch (command) {
    case '1': case 'c': centerServo(); break;
    case '2': case 'l': moveTo(steeringTarget(-1)); break;
    case '3': case 'r': moveTo(steeringTarget(+1)); break;
    case '4': selectSpan(150); break;
    case '5': selectSpan(400); break;
    case '6': selectSpan(650); break;
    case '7': selectSpan(800); break;
    case '8': selectSpan(1000); break;
    case '-': trimCenter(-5); break;
    case '+': trimCenter(5); break;
    case '9': Serial.printf("PWM=%s center=%d output=%d target=%d us\n",
                  enabled ? "ON" : "OFF", centerUs, outputUs, targetUs);
              printConfig(); break;
    case 0: break;
    default: Serial.println("Use 1/2/3/0; presets4..8; -/+/9."); break;
  }
  now = millis();
  if (enabled && outputUs != targetUs && uint32_t(now - lastStepMs) >= 20) {
    lastStepMs = now;
    const int distance = targetUs - outputUs;
    const int step = abs(distance) < 5 ? abs(distance) : 5;
    outputUs += distance > 0 ? step : -step;
    if (!writePulse(outputUs)) {
      stopServo(); Serial.println("FAULT: PWM write failed."); return;
    }
    if (outputUs == targetUs) Serial.printf("OUTPUT=%dus; check physical wheels.\n", outputUs);
  }
  delay(1);
}
