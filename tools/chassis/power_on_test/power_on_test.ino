// A15 v1.2 - one automatic bench test per ESP32 power-on / EN reset.
// Wiring: A9 parallel splitter v2.5; Arduino-ESP32 3.x; ESP32-WROOM-32E.
// Lift ALL wheels. USB powers only the MCU. Motor/servo power must be OFF
// during upload. After switching actuator power ON, press EN to start again.
// No battery-sense wire exists: switching actuator power alone cannot restart.
#include <Arduino.h>
#include <esp_system.h>

constexpr uint8_t MOTOR_PINS[] = {25, 26, 32, 33}; // AN1, AN2, BN1, BN2
constexpr uint8_t SERVO_PIN = 14;
constexpr uint32_t MOTOR_HZ = 20000;
constexpr uint8_t MOTOR_BITS = 8;
constexpr uint8_t MOTOR_DUTY = 77; // 30.2%; command, NOT measured speed/voltage.
constexpr uint32_t SERVO_HZ = 50;
constexpr uint8_t SERVO_BITS = 15;
constexpr uint8_t SERVO_CHANNEL = 8; // ESP32 LOW group, independent of motors 0..3.
constexpr int CENTER_US = 1500;
constexpr int SPAN_US = 300;
constexpr bool STEERING_REVERSED = false; // Actual left/right must be observed.
constexpr uint32_t BOOT_WAIT_MS = 3000;
constexpr uint32_t MOTOR_RUN_MS = 2000;
constexpr uint32_t BRAKE_MS = 300;
constexpr uint32_t SETTLE_MS = 700;
constexpr uint32_t CENTER_HOLD_MS = 500;
constexpr uint32_t TURN_HOLD_MS = 1000;
constexpr uint32_t RAMP_INTERVAL_MS = 20;
constexpr int RAMP_STEP_US = 25;

enum class Stage : uint8_t {
  BootWait, MotorRun, Brake, Settle, InitialCenter, Left, Center, Right,
  Done, Aborted, Fault
};
// Explicit declarations keep Arduino's generated prototypes after the enum.
void stopAll(Stage stoppedStage, const char *reason);
void turnTo(Stage next, int us, const char *label);
Stage stage = Stage::BootWait;
bool motorAttached[4] = {};
bool servoAttached = false;
uint32_t stageStartedMs = 0;
uint32_t lastRampMs = 0;
uint32_t motorStartedMs = 0;
int outputUs = CENTER_US; // Last command; no position feedback.
int targetUs = CENTER_US;
bool holding = false;
uint32_t holdStartedMs = 0;

void message(const char *text) {
  Serial.printf("[%lu ms] %s\n", static_cast<unsigned long>(millis()), text);
}

void forceMotorLow() {
  // Detach as well as writing zero: force GPIO low even on LEDC errors.
  for (uint8_t i = 0; i < 4; ++i) {
    if (motorAttached[i]) {
      ledcWrite(MOTOR_PINS[i], 0);
      ledcDetach(MOTOR_PINS[i]);
      motorAttached[i] = false;
    }
    digitalWrite(MOTOR_PINS[i], LOW);
    pinMode(MOTOR_PINS[i], OUTPUT);
  }
}

void stopServo() {
  if (servoAttached) {
    ledcWrite(SERVO_PIN, 0);
    ledcDetach(SERVO_PIN);
    servoAttached = false;
  }
  digitalWrite(SERVO_PIN, LOW);
  pinMode(SERVO_PIN, OUTPUT);
}

void stopAll(Stage stoppedStage, const char *reason) {
  forceMotorLow();
  stopServo();
  stage = stoppedStage;
  message(reason);
}

bool writeMotorValues(uint8_t an1, uint8_t an2, uint8_t bn1, uint8_t bn2) {
  const uint8_t duties[] = {an1, an2, bn1, bn2};
  for (uint8_t i = 0; i < 4; ++i) {
    if (!ledcWrite(MOTOR_PINS[i], duties[i])) {
      stopAll(Stage::Fault, "FAULT: motor PWM write. Cut actuator power.");
      return false;
    }
  }
  return true;
}

bool writeServoUs(int us) {
  if (us < CENTER_US - SPAN_US || us > CENTER_US + SPAN_US) return false;
  const uint32_t duty = (static_cast<uint64_t>(us) * (1UL << SERVO_BITS)
                         + 10000) / 20000;
  return ledcWrite(SERVO_PIN, duty);
}

void turnTo(Stage next, int us, const char *label) {
  stage = next;
  targetUs = us;
  holding = false;
  lastRampMs = millis();
  message(label);
}

void setup() {
  // Establish low outputs before configuring serial or PWM.
  forceMotorLow();
  stopServo();
  Serial.begin(115200);
  delay(250);
  message("A15 v1.2 ONE SHOT - A9 v2.5 - ESP32-WROOM-32E");
  message("Lift all wheels. 0 / x / ! = immediate software stop; EN restarts.");
  message("USB cannot sense actuator power. Switch actuator power ON, then press EN.");
  message("Sequence: wait 3s, rear motors 2s, stop, front left / center / right, PWM OFF.");
  Serial.printf("motor duty=%u/255; servo center=%d span=+/-%d us; reverse=%u\n",
                MOTOR_DUTY, CENTER_US, SPAN_US, STEERING_REVERSED);
  const esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.printf("reset_reason=%d\n", static_cast<int>(resetReason));
  if (resetReason != ESP_RST_POWERON && resetReason != ESP_RST_EXT) {
    stopAll(Stage::Aborted, "AUTO BLOCKED after non-power/EN reset. Inspect power, then EN.");
    return;
  }
  for (uint8_t i = 0; i < 4; ++i) {
    // All motor channels share compatible 20kHz/8bit settings in HIGH group.
    motorAttached[i] = ledcAttachChannel(MOTOR_PINS[i], MOTOR_HZ, MOTOR_BITS, i);
    // Use configuration/write return values. Core 3.3.12 ledcReadFreq returns
    // 0 at duty 0, including immediately after a pending hardware duty update.
    // It is unsuitable as a synchronous initialization/write success check.
    if (!motorAttached[i] || !ledcWrite(MOTOR_PINS[i], 0)) {
      stopAll(Stage::Fault, "FAULT: motor PWM initialization. Cut actuator power.");
      return;
    }
  }
  stageStartedMs = millis();
  message("COUNTDOWN: automatic motor command in 3 seconds.");
}

void loop() {
  uint32_t now = millis();
  // Check deadline before serial work; neither steering nor printing extends run.
  if (stage == Stage::MotorRun
      && static_cast<uint32_t>(now - motorStartedMs) >= MOTOR_RUN_MS) {
    // Manufacturer's dual-input drive convention: 1/1 is the zero drive command.
    if (!writeMotorValues(255, 255, 255, 255)) return;
    stage = Stage::Brake;
    stageStartedMs = millis();
    Serial.printf("[%lu ms] MOTOR STOP command; drive interval=%lu ms\n",
                  static_cast<unsigned long>(stageStartedMs),
                  static_cast<unsigned long>(stageStartedMs - motorStartedMs));
  }
  for (uint8_t count = 0; count < 32 && Serial.available(); ++count) {
    const char c = static_cast<char>(Serial.read());
    if (c == '0' || c == 'x' || c == 'X' || c == '!') {
      stopAll(Stage::Aborted, "ABORTED: all outputs LOW; cut actuator power. EN to restart.");
      return;
    }
  }
  now = millis();
  switch (stage) {
    case Stage::BootWait:
      if (static_cast<uint32_t>(now - stageStartedMs) >= BOOT_WAIT_MS) {
        // Same electrical polarity on A/B; chassis forward direction uncalibrated.
        // Yahboom reference: IN1=255, IN2=255-command, rather than a third PWM pin.
        motorStartedMs = millis();
        if (!writeMotorValues(255, 255 - MOTOR_DUTY, 255, 255 - MOTOR_DUTY)) return;
        stage = Stage::MotorRun;
        message("REAR MOTORS RUN: 77/255 for 2000ms (not measured RPM).");
      }
      break;
    case Stage::Brake:
      if (static_cast<uint32_t>(now - stageStartedMs) >= BRAKE_MS) {
        forceMotorLow();
        stage = Stage::Settle;
        stageStartedMs = millis();
        message("MOTOR INPUTS LOW. Waiting for wheel coast-down; no speed feedback.");
      }
      break;
    case Stage::Settle:
      if (static_cast<uint32_t>(now - stageStartedMs) >= SETTLE_MS) {
        servoAttached = ledcAttachChannel(SERVO_PIN, SERVO_HZ, SERVO_BITS, SERVO_CHANNEL);
        if (!servoAttached || !writeServoUs(CENTER_US)) {
          stopAll(Stage::Fault, "FAULT: servo PWM initialization. Cut actuator power.");
          return;
        }
        outputUs = targetUs = CENTER_US;
        turnTo(Stage::InitialCenter, CENTER_US, "INITIAL CENTER command 1500us.");
      }
      break;
    case Stage::InitialCenter:
    case Stage::Left:
    case Stage::Center:
    case Stage::Right:
      if (outputUs != targetUs
          && static_cast<uint32_t>(now - lastRampMs) >= RAMP_INTERVAL_MS) {
        lastRampMs = now;
        const int distance = targetUs - outputUs;
        const int step = abs(distance) < RAMP_STEP_US ? abs(distance) : RAMP_STEP_US;
        outputUs += distance > 0 ? step : -step;
        if (!writeServoUs(outputUs)) {
          stopAll(Stage::Fault, "FAULT: servo PWM write. Cut actuator power.");
          return;
        }
      }
      if (outputUs == targetUs) {
        if (!holding) {
          holding = true;
          holdStartedMs = now;
          Serial.printf("[%lu ms] OUTPUT command=%d us; not a position measurement.\n",
                        static_cast<unsigned long>(now), outputUs);
        } else if (static_cast<uint32_t>(now - holdStartedMs)
                   >= (stage == Stage::InitialCenter ? CENTER_HOLD_MS : TURN_HOLD_MS)) {
          if (stage == Stage::InitialCenter) {
            turnTo(Stage::Left, CENTER_US + (STEERING_REVERSED ? SPAN_US : -SPAN_US),
                   "LEFT command (observe actual direction).");
          } else if (stage == Stage::Left) {
            turnTo(Stage::Center, CENTER_US, "CENTER command.");
          } else if (stage == Stage::Center) {
            turnTo(Stage::Right, CENTER_US + (STEERING_REVERSED ? -SPAN_US : SPAN_US),
                   "RIGHT command (observe actual direction).");
          } else {
            stopAll(Stage::Done, "DONE: one shot finished; all PWM OFF. No automatic repeat.");
          }
        }
      }
      break;
    default:
      break; // MotorRun waits for deadline; terminal states never restart.
  }
  delay(1);
}
