// A10: isolated first test, ESP32-WROOM-32E GPIO14, external servo power.
// Remove the steering linkage before arming. Cutting PWM is NOT cutting power.
#include <Arduino.h>
#include <esp_arduino_version.h>

constexpr uint8_t SERVO_PIN = 14;
constexpr uint8_t PWM_BITS = 15;
constexpr uint8_t PWM_CHANNEL = 0;  // Arduino-ESP32 2.x only
constexpr uint32_t PWM_HZ = 50;
constexpr uint32_t PERIOD_US = 20000;
constexpr int CENTER_US = 1500;
constexpr int MIN_US = 1450;
constexpr int MAX_US = 1550;
constexpr int STEP_US = 25;
constexpr uint32_t TIMEOUT_MS = 5000;

bool attached = false;
bool active = false;
int commandedUs = CENTER_US;
uint32_t lastMotionMs = 0;
char lineBuffer[8];
uint8_t lineLength = 0;
bool lineOverflow = false;

bool pwmBegin() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  attached = ledcAttach(SERVO_PIN, PWM_HZ, PWM_BITS);
#else
  if (ledcSetup(PWM_CHANNEL, PWM_HZ, PWM_BITS) <= 0) return false;
  ledcAttachPin(SERVO_PIN, PWM_CHANNEL);
  attached = true;
#endif
  return attached;
}

bool pwmWriteUs(int pulseUs) {
  const uint32_t duty = static_cast<uint32_t>(
      (static_cast<uint64_t>(pulseUs) * (1UL << PWM_BITS) + PERIOD_US / 2) / PERIOD_US);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcWrite(SERVO_PIN, duty);
#else
  ledcWrite(PWM_CHANNEL, duty);
  return true;
#endif
}

void pwmStop() {
  if (attached) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(SERVO_PIN, 0);
    ledcDetach(SERVO_PIN);
#else
    ledcWrite(PWM_CHANNEL, 0);
    ledcDetachPin(SERVO_PIN);
#endif
  }
  attached = false;
  active = false;
  digitalWrite(SERVO_PIN, LOW);
  pinMode(SERVO_PIN, OUTPUT);
}

void showStatus() {
  Serial.printf("PWM=%s, command=%d us (NOT measured angle)\n",
                active ? "ON" : "OFF", commandedUs);
}

void showHelp() {
  Serial.println("a: arm at 1500us; n: 1500us; + / -: change by 25us");
  Serial.println("p: status; x: PWM off; h: help. One command per line.");
  Serial.println("Range 1450..1550us. PWM stops 5s after last motion command.");
  Serial.println("FIRST TEST: DISCONNECT STEERING LINKAGE; USE EXTERNAL 5V + COMMON GND.");
  Serial.println("Emergency: CUT SERVO POWER. x/timeout does NOT guarantee release.");
}

void setPulse(int pulseUs) {
  if (pulseUs < MIN_US || pulseUs > MAX_US) {
    Serial.println("LIMIT: command rejected; unchanged.");
    return;
  }
  if (!pwmWriteUs(pulseUs)) {
    pwmStop();
    Serial.println("ERROR: PWM write failed; cut servo power and check.");
    return;
  }
  commandedUs = pulseUs;
  active = true;
  lastMotionMs = millis();
  showStatus();
}

void handleLine() {
  lineBuffer[lineLength] = '\0';
  String command(lineBuffer);
  command.trim();
  if (lineOverflow || command.length() > 1) {
    Serial.println("ERROR: send ONE command per line.");
  } else if (command.length() == 1) {
    const char c = command.charAt(0);
    if (c == 'x') {
      pwmStop();
      Serial.println("PWM OFF. Servo power is still ON unless physically switched off.");
    } else if (c == 'h') {
      showHelp();
    } else if (c == 'p') {
      showStatus();
    } else if (c == 'a') {
      if (active) {
        Serial.println("Already armed. Use n / + / - or x.");
      } else if (!pwmBegin()) {
        pwmStop();
        Serial.println("ERROR: PWM attach failed; cut servo power and check board/core.");
      } else {
        setPulse(CENTER_US);
      }
    } else if (c == 'n' || c == '+' || c == '-') {
      if (!active) Serial.println("PWM OFF: use a only after clearing the mechanism.");
      else setPulse(c == 'n' ? CENTER_US : commandedUs + (c == '+' ? STEP_US : -STEP_US));
    } else {
      Serial.println("Unknown command. h for help.");
    }
  }
  lineLength = 0;
  lineOverflow = false;
}

void setup() {
  digitalWrite(SERVO_PIN, LOW);
  pinMode(SERVO_PIN, OUTPUT);
  Serial.begin(115200);
  delay(300);
  Serial.println("\nA10 SERVO FIRST TEST - GPIO14 - PWM OFF AT BOOT");
  showHelp();
}

void loop() {
  if (active && static_cast<uint32_t>(millis() - lastMotionMs) >= TIMEOUT_MS) {
    pwmStop();
    Serial.println("TIMEOUT: PWM OFF. Cut servo power when finished.");
  }
  // Bound input work so a flood of serial data cannot indefinitely block timeout checks.
  for (uint8_t count = 0; count < 32 && Serial.available(); ++count) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') handleLine();
    else if (lineLength < sizeof(lineBuffer) - 1) lineBuffer[lineLength++] = c;
    else lineOverflow = true;
  }
}
