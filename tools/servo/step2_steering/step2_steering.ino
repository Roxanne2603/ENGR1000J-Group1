// A11 v2: manual steering test with numeric aliases and input echo.
// GPIO14, separate verified 5V, common ground.
// Lift front wheels and verify the center/linkage before arming. No auto sweep.
#include <Arduino.h>
#include <esp_arduino_version.h>

constexpr uint8_t SERVO_PIN = 14;
constexpr uint8_t PWM_BITS = 15;
constexpr uint8_t PWM_CHANNEL = 0;
constexpr uint32_t PWM_HZ = 50;
constexpr uint32_t PERIOD_US = 20000;
constexpr int ABS_MIN_US = 1100;  // Test envelope, NOT a verified mechanical limit.
constexpr int ABS_MAX_US = 1900;
constexpr int MAX_AMPLITUDE_US = 300;
constexpr int JOG_US = 50;
constexpr int RAMP_STEP_US = 5;
constexpr uint32_t RAMP_INTERVAL_MS = 20;
constexpr uint32_t TIMEOUT_MS = 15000;

bool attached = false;
bool active = false;
bool reversed = false;
int centerUs = 1500;
int amplitudeUs = 100;
int currentUs = 1500;  // Last OUTPUT command, not measured position.
int targetUs = 1500;
uint32_t lastMotionMs = 0;
uint32_t lastRampMs = 0;
char lineBuffer[32];
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

void stopPwm() {
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
  targetUs = currentUs;
  digitalWrite(SERVO_PIN, LOW);
  pinMode(SERVO_PIN, OUTPUT);
}

void showStatus() {
  Serial.printf("PWM=%s center=%d span=+/- %d output=%d target=%d us reverse=%s\n",
                active ? "ON" : "OFF", centerUs, amplitudeUs, currentUs, targetUs,
                reversed ? "YES" : "NO");
  Serial.println("output/target are COMMANDS, not measured wheel angles.");
}

void showHelp() {
  Serial.println("DIGITS: 1=arm/center, 2=one side, 3=other side, 0=PWM OFF, 9=status");
  Serial.println("a: arm at center; l/r: center -/+ span; n: center; +/-: jog 50us");
  Serial.println("b 100/200/300: span (no motion); c 1500: center (PWM OFF only)");
  Serial.println("d: reverse l/r labels; p: status; x: PWM OFF; h: help");
  Serial.println("One command per line. No automatic sweep. 15s idle -> PWM OFF.");
  Serial.println("Lift wheels. Confirm center/linkage. l/r directions MUST be observed.");
  Serial.println("Emergency: CUT SERVO POWER. PWM OFF does NOT guarantee release.");
}

bool readNumber(String text, int &number) {
  text.trim();
  if (text.length() == 0 || text.length() > 4) return false;
  for (unsigned int i = 0; i < text.length(); ++i) {
    if (text.charAt(i) < '0' || text.charAt(i) > '9') return false;
  }
  number = text.toInt();
  return true;
}

bool validEnvelope(int center, int span) {
  return span >= 25 && span <= MAX_AMPLITUDE_US &&
         center - span >= ABS_MIN_US && center + span <= ABS_MAX_US;
}

void moveTo(int pulseUs) {
  if (!active) {
    Serial.println("PWM OFF: check mechanism, then send 1 (or a).");
    return;
  }
  if (pulseUs < centerUs - amplitudeUs || pulseUs > centerUs + amplitudeUs) {
    Serial.println("LIMIT: target rejected; unchanged.");
    return;
  }
  targetUs = pulseUs;
  lastMotionMs = millis();
  showStatus();
}

void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) return;
  Serial.printf("[RX] <%s>\n", command.c_str());
  // Accept full-width digits from an IME; keep original input visible above.
  command.replace("０", "0");
  command.replace("１", "1");
  command.replace("２", "2");
  command.replace("３", "3");
  command.replace("４", "4");
  command.replace("５", "5");
  command.replace("６", "6");
  command.replace("７", "7");
  command.replace("８", "8");
  command.replace("９", "9");
  command.replace("　", " ");
  command.trim();
  command.toLowerCase();
  if (command == "0") command = "x";
  else if (command == "1") command = active ? "n" : "a";
  else if (command == "2") command = "l";
  else if (command == "3") command = "r";
  else if (command == "9") command = "p";
  if (command == "x") {
    stopPwm();
    Serial.println("PWM OFF. Switch servo power OFF to remove power.");
  } else if (command == "p") {
    showStatus();
  } else if (command == "h") {
    showHelp();
  } else if (command == "a") {
    if (active) {
      Serial.println("Already armed. Use l/r/n/+/-.");
    } else if (!pwmBegin() || !pwmWriteUs(centerUs)) {
      stopPwm();
      Serial.println("ERROR: PWM initialization failed. Cut servo power.");
    } else {
      active = true;
      currentUs = targetUs = centerUs;
      lastMotionMs = lastRampMs = millis();
      Serial.println("ARMED at center. First physical movement may be sudden.");
      showStatus();
    }
  } else if (command == "n") {
    moveTo(centerUs);
  } else if (command == "l" || command == "r") {
    int direction = command == "l" ? -1 : 1;
    if (reversed) direction = -direction;
    moveTo(centerUs + direction * amplitudeUs);
  } else if (command == "+" || command == "-") {
    moveTo(targetUs + (command == "+" ? JOG_US : -JOG_US));
  } else if (command == "d") {
    reversed = !reversed;
    showStatus();
  } else if (command.startsWith("b ") || command.startsWith("c ")) {
    int number = 0;
    if (!readNumber(command.substring(2), number)) {
      Serial.println("ERROR: expected digits, e.g. b 200 / c 1500.");
      return;
    }
    if (command.charAt(0) == 'c') {
      if (active) {
        Serial.println("Use x before changing center.");
      } else if (!validEnvelope(number, amplitudeUs)) {
        Serial.println("LIMIT: center/span exceeds 1100..1900us envelope.");
      } else {
        centerUs = number;
        currentUs = targetUs = centerUs;
        showStatus();
      }
    } else if (!validEnvelope(centerUs, number)) {
      Serial.println("LIMIT: span 25..300us; center +/- span must fit 1100..1900us.");
    } else if (active && (currentUs < centerUs - number || currentUs > centerUs + number ||
                          targetUs < centerUs - number || targetUs > centerUs + number)) {
      Serial.println("Return to center and wait before reducing span.");
    } else {
      amplitudeUs = number;
      showStatus();  // Changing span never commands movement by itself.
    }
  } else {
    Serial.println("Unknown command. Send a single digit: 1 / 2 / 3 / 0 / 9, then Enter.");
  }
}

void setup() {
  digitalWrite(SERVO_PIN, LOW);
  pinMode(SERVO_PIN, OUTPUT);
  Serial.begin(115200);
  delay(300);
  Serial.println("\nA11 v2 DIGIT STEERING TEST - GPIO14 - PWM OFF AT BOOT");
  showHelp();
  showStatus();
}

void loop() {
  uint32_t now = millis();
  if (active && static_cast<uint32_t>(now - lastMotionMs) >= TIMEOUT_MS) {
    stopPwm();
    Serial.println("TIMEOUT: PWM OFF. Servo may still hold; cut power when finished.");
  }
  // Parse input before ramping, so x cancels the target without waiting for a sweep.
  for (uint8_t count = 0; count < 64 && Serial.available(); ++count) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      lineBuffer[lineLength] = '\0';
      if (lineOverflow) Serial.println("ERROR: line too long, rejected.");
      else handleCommand(String(lineBuffer));
      lineLength = 0;
      lineOverflow = false;
    } else if (lineLength < sizeof(lineBuffer) - 1) {
      lineBuffer[lineLength++] = c;
    } else {
      lineOverflow = true;
    }
  }
  now = millis();
  if (active && currentUs != targetUs &&
      static_cast<uint32_t>(now - lastRampMs) >= RAMP_INTERVAL_MS) {
    lastRampMs = now;
    const int distance = targetUs - currentUs;
    const int step = abs(distance) < RAMP_STEP_US ? abs(distance) : RAMP_STEP_US;
    const int nextUs = currentUs + (distance > 0 ? step : -step);
    if (!pwmWriteUs(nextUs)) {
      stopPwm();
      Serial.println("ERROR: PWM write failed. Cut servo power.");
    } else {
      currentUs = nextUs;
      if (currentUs == targetUs) Serial.printf("REACHED command %d us (not measured).\n", currentUs);
    }
  }
}
