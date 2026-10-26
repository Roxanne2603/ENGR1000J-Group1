#pragma once
#include <Arduino.h>

namespace cfg {
constexpr uint8_t MOTOR[4] = {25, 26, 32, 33};
constexpr uint8_t ENC[2][2] = {{18, 19}, {34, 35}};
constexpr uint8_t SERVO = 14;
constexpr int MOTOR_SIGN[2] = {+1, -1};
constexpr int ENC_SIGN[2] = {+1, -1};
constexpr float CPR[2] = {824.2f, 825.7f}; // Initial manual estimates only.
constexpr float CIRCUMFERENCE_MM = 650.0f / 3.0f;
constexpr int CENTER_US = 1375, LEFT_US = 975, RIGHT_US = 1775;
constexpr int DUTY = 50;
constexpr uint32_t DEFAULT_DRIVE_MS = 1000, LONG_DRIVE_MS = 3000, MAX_DRIVE_MS = 3000;
constexpr uint32_t BRAKE_MS = 2000, LEASE_MS = 80;
constexpr uint32_t SETTLE_MS = 500, LOW_SPEED_MS = 300, BRAKE_TIMEOUT_MS = 5000;
constexpr int SERVO_STEP_US = 25;
constexpr uint32_t PRE_CENTER_HOLD_MS = 1000, HOME_HOLD_MS = 1500;
constexpr float LOW_SPEED_MPS = 0.03f; // Engineering gate, not stopping-distance acceptance.
}

enum class Stage : uint8_t {
  IDLE, PRE_CENTER, COUNTDOWN, STRAIGHT, BRAKE1, STEER_LEFT, LEFT,
  BRAKE2, STEER_RIGHT, RIGHT, BRAKE3, HOME, DONE, STOPPED, FAULT
};

struct Command {
  int duty[2] = {0, 0};
  int us = cfg::CENTER_US;
  bool servo = false, brake = false, watch = false;
  uint32_t stamp = 0, legStarted = 0, legDuration = cfg::DEFAULT_DRIVE_MS;
  uint8_t leg = 0;
};

struct DriveTrace {
  uint8_t leg = 0;
  uint32_t begin = 0, end = 0;
};

struct Applied {
  int a[2] = {0, 0}, b[2] = {0, 0};
  int servoUs = 0; // Pulse command written, not angle feedback.
};
