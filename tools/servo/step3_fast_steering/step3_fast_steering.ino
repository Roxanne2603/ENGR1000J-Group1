// A12 v3: steering presets up to +/-400us within the existing test envelope.
// GPIO14, separate verified 5V, common ground.
// Lift front wheels and verify the center/linkage before arming. No movement at boot; a cycle requires explicit command 6.
#include <Arduino.h>
#include <esp_arduino_version.h>

constexpr uint8_t SERVO_PIN = 14;
constexpr uint8_t PWM_BITS = 15;
constexpr uint8_t PWM_CHANNEL = 0;
constexpr uint32_t PWM_HZ = 50;
constexpr uint32_t PERIOD_US = 20000;
constexpr int ABS_MIN_US = 1100;  // Test envelope, NOT a verified mechanical limit.
constexpr int ABS_MAX_US = 1900;
constexpr int MAX_AMPLITUDE_US = 400;
constexpr int SMALL_SPAN_US = 100;
constexpr int MEDIUM_SPAN_US = 200;
constexpr int LARGE_SPAN_US = 300;
constexpr int EXTRA350_SPAN_US = 350;
constexpr int EXTRA400_SPAN_US = 400;
constexpr int JOG_US = 50;
constexpr int RAMP_STEP_US = 5;
constexpr int FAST_STEP_US = 25;
constexpr uint32_t RAMP_INTERVAL_MS = 20;
constexpr uint32_t TIMEOUT_MS = 15000;
constexpr uint32_t CYCLE_DWELL_MS = 800;

bool attached = false;
bool active = false;
bool reversed = false;
uint8_t speedMode = 0;  // 0=original ramp, 1=fast ramp, 2=direct target command.
bool cycleRunning = false;
uint8_t cycleStage = 0;
bool cycleDwelling = false;
uint32_t cycleDwellStartedMs = 0;
int centerUs = 1500;
int amplitudeUs = SMALL_SPAN_US;
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
  cycleRunning = false;
  cycleDwelling = false;
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
  Serial.printf("speed=%s single_cycle=%s\n",
                speedMode == 0 ? "SLOW" : (speedMode == 1 ? "FAST" : "DIRECT"),
                cycleRunning ? "RUNNING" : "OFF");
  Serial.printf("span_preset=%s\n", amplitudeUs == SMALL_SPAN_US ? "SMALL" :
                (amplitudeUs == MEDIUM_SPAN_US ? "MEDIUM" :
                 (amplitudeUs == LARGE_SPAN_US ? "LARGE" :
                  (amplitudeUs == EXTRA350_SPAN_US ? "EXTRA350" :
                   (amplitudeUs == EXTRA400_SPAN_US ? "EXTRA400" : "CUSTOM")))));
}

void showHelp() {
  Serial.println("DIGITS: 1=arm/center, 2=one side, 3=other side, 0=PWM OFF, 9=status");
  Serial.println("4=FAST ramp, 5=DIRECT target, 8=SLOW ramp (selecting speed does not move)");
  Serial.println("6=ONE cycle: side/center/other side/center, then PWM OFF");
  Serial.println("10=SMALL +/-100us, 11=MEDIUM +/-200us, 12=LARGE +/-300us");
  Serial.println("13=EXTRA350 +/-350us, 14=EXTRA400 +/-400us (unverified mechanical range)");
  Serial.println("Presets only set span. Return to center before changing span while PWM ON.");
  Serial.println("a: arm at center; l/r: center -/+ span; n: center; +/-: jog 50us");
  Serial.println("b 100..400: span (no motion); c 1500: center (PWM OFF only)");
  Serial.println("d: reverse l/r labels; p: status; x: PWM OFF; h: help");
  Serial.println("One command per line. No movement at boot. 15s idle -> PWM OFF.");
  Serial.println("6 requires PWM ON, output command at center, FAST or DIRECT mode.");
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
  else if (command == "4") command = "fast";
  else if (command == "5") command = "direct";
  else if (command == "6") command = "test";
  else if (command == "8") command = "slow";
  else if (command == "9") command = "p";
  else if (command == "10") command = String("b ") + SMALL_SPAN_US;
  else if (command == "11") command = String("b ") + MEDIUM_SPAN_US;
  else if (command == "12") command = String("b ") + LARGE_SPAN_US;
  else if (command == "13") command = String("b ") + EXTRA350_SPAN_US;
  else if (command == "14") command = String("b ") + EXTRA400_SPAN_US;
  // Manual position commands cancel the single cycle before setting a new target.
  if (cycleRunning && (command == "a" || command == "n" || command == "l" ||
                       command == "r" || command == "+" || command == "-")) {
    cycleRunning = false;
    cycleDwelling = false;
    Serial.println("Single cycle cancelled by manual command.");
  }
  if (cycleRunning && command != "x" && command != "p" && command != "h") {
    Serial.println("Cycle running: 0 to stop; manual 1/2/3 cancels and takes control.");
    return;
  }
  if (command == "x") {
    stopPwm();
    Serial.println("PWM OFF. Switch servo power OFF to remove power.");
  } else if (command == "p") {
    showStatus();
  } else if (command == "h") {
    showHelp();
  } else if (command == "fast" || command == "direct" || command == "slow") {
    if (active && currentUs != targetUs) {
      Serial.println("Wait for the current command ramp to finish before changing speed.");
      return;
    }
    speedMode = command == "slow" ? 0 : (command == "fast" ? 1 : 2);
    showStatus();
  } else if (command == "test") {
    if (!active || currentUs != centerUs || targetUs != centerUs || speedMode == 0) {
      Serial.println("Use 1, wait for center command, select 4 or 5, then 6.");
      return;
    }
    cycleRunning = true;
    cycleStage = 0;
    cycleDwelling = false;
    Serial.println("ONE cycle started. 0 cancels PWM; cut power for mechanical trouble.");
    moveTo(centerUs + (reversed ? amplitudeUs : -amplitudeUs));
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
      Serial.println("LIMIT: span 25..400us; center +/- span must fit 1100..1900us.");
    } else if (active && (currentUs != centerUs || targetUs != centerUs)) {
      Serial.println("Return to center with 1 and wait before changing span.");
    } else {
      amplitudeUs = number;
      showStatus();  // Changing span never commands movement by itself.
    }
  } else {
    Serial.println("Unknown command. Send 0/1/2/3/4/5/6/8/9 or 10/11/12/13/14, then Enter.");
  }
}

void setup() {
  digitalWrite(SERVO_PIN, LOW);
  pinMode(SERVO_PIN, OUTPUT);
  Serial.begin(115200);
  delay(300);
  Serial.println("\nA12 v3 EXTENDED STEERING TEST - GPIO14 - PWM OFF AT BOOT");
  showHelp();
  showStatus();
}

void loop() {
  uint32_t now = millis();
  if (active && static_cast<uint32_t>(now - lastMotionMs) >= TIMEOUT_MS) {
    stopPwm();
    Serial.println("TIMEOUT: PWM OFF. Servo may still hold; cut power when finished.");
  }
  // Input, timeout and cycle updates are non-blocking; 0 can cancel at any stage.
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
    const int maxStep = speedMode == 0 ? RAMP_STEP_US :
                        (speedMode == 1 ? FAST_STEP_US : abs(distance));
    const int step = abs(distance) < maxStep ? abs(distance) : maxStep;
    const int nextUs = currentUs + (distance > 0 ? step : -step);
    if (!pwmWriteUs(nextUs)) {
      stopPwm();
      Serial.println("ERROR: PWM write failed. Cut servo power.");
    } else {
      currentUs = nextUs;
      if (currentUs == targetUs) Serial.printf("REACHED command %d us (not measured).\n", currentUs);
    }
  }
  if (active && cycleRunning && currentUs == targetUs) {
    if (!cycleDwelling) {
      cycleDwelling = true;
      cycleDwellStartedMs = millis();
    } else if (static_cast<uint32_t>(millis() - cycleDwellStartedMs) >= CYCLE_DWELL_MS) {
      cycleDwelling = false;
      ++cycleStage;
      if (cycleStage >= 4) {
        stopPwm();
        Serial.println("ONE CYCLE DONE: PWM OFF. Switch servo power OFF when finished.");
      } else {
        const int nextTarget = cycleStage == 2
            ? centerUs + (reversed ? -amplitudeUs : amplitudeUs) : centerUs;
        moveTo(nextTarget);
      }
    }
  }
}
