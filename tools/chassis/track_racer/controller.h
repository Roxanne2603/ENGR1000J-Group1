#pragma once
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "config.h"
namespace racer {
inline float clip(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }
inline float rad(float deg) { return deg * 0.01745329252f; }
struct Sample {
  float mm = 0;
  uint32_t stamp = 0; // Exposure START, not time the result was printed.
  bool valid = false;
};
struct Input {
  Sample s[3]; // front, left, right
  uint32_t now = 0;
  float speed[2] = {0, 0};
  float dt = 0.02f;
};
struct Output {
  float speed = 0, steer = 0, cap = 0;
  const char* stop = nullptr;
};
inline bool fresh(const Sample& s, uint32_t now, uint32_t age) {
  return s.valid && isfinite(s.mm) && s.mm > 0 && uint32_t(now - s.stamp) <= age;
}
// Solve v*T + v^2/(2*a) <= available distance, SI units.
inline float stopCap(float d, float a, float t) {
  if (!(d > 0 && a > 0 && t >= 0)) return 0;
  return sqrtf(a*a*t*t + 2*a*d) - a*t;
}
inline bool calibrated() {
  if (!cfg::COMMISSIONED || !cfg::DIRECTION_CHECKED || !cfg::MOUNTING_CHECKED || !cfg::SWEPT_PATH_CHECKED) return false;
  if (!(cfg::WIDTH_MM > 0 && cfg::WIDTH_MM + 2*cfg::MIN_CLEAR_MM < 400 &&
        cfg::WHEELBASE_MM > 0 && cfg::REAR_TRACK_MM > 0 &&
        cfg::FRONT_INSET_MM >= 0 && cfg::SIDE_INSET_MM[0] >= 0 && cfg::SIDE_INSET_MM[1] >= 0 &&
        cfg::MIN_TURN_RADIUS_MM > 0 && cfg::BRAKE_MPS2 > 0 && cfg::REACTION_S > 0 &&
        cfg::LATERAL_MPS2 > 0 && cfg::MAX_WHEEL_MPS >= cfg::INITIAL_MAX_MPS &&
        cfg::BRAKE_HOLD_MS*0.001f >= 1.2f*cfg::MAX_WHEEL_MPS/cfg::BRAKE_MPS2 &&
        cfg::LEFT_ANGLE_DEG > 0 && cfg::LEFT_ANGLE_DEG < 60 &&
        cfg::RIGHT_ANGLE_DEG > 0 && cfg::RIGHT_ANGLE_DEG < 60 &&
        cfg::CENTER_US >= 1100 && cfg::CENTER_US <= 1900 &&
        cfg::LEFT_US >= 1100 && cfg::LEFT_US <= 1900 &&
        cfg::RIGHT_US >= 1100 && cfg::RIGHT_US <= 1900 &&
        (cfg::LEFT_US-cfg::CENTER_US)*cfg::SERVO_LEFT_SIGN > 0 &&
        (cfg::RIGHT_US-cfg::CENTER_US)*cfg::SERVO_LEFT_SIGN < 0)) return false;
  for (int i=0; i<2; ++i)
    if (!(cfg::WHEEL_MM[i] > 0 && cfg::CPR4[i] > 0 && cfg::FF_DUTY_PER_MPS[i] > 0 &&
          cfg::FF_STATIC[i] >= 0 && cfg::FF_STATIC[i] < cfg::MAX_DUTY &&
          abs(cfg::MOTOR_SIGN[i]) == 1 && abs(cfg::ENCODER_SIGN[i]) == 1)) return false;
  for (int i=0; i<3; ++i)
    if (!(cfg::ERROR_MM[i] > 0 && cfg::CERTIFIED_MAX_MM[i] > 200 && isfinite(cfg::BIAS_MM[i]))) return false;
  return true;
}
class Controller {
  float prevError = 0, derivative = 0, speedCmd = 0;
  uint32_t sideStamp = 0;
  int turnSign = 0;
public:
  void reset() { prevError=derivative=speedCmd=0; sideStamp=0; turnSign=0; }
  Output step(const Input& in) {
    Output out;
    if (!calibrated()) { out.stop="UNCALIBRATED"; return out; }
    const float wheelbaseM=fmaxf(cfg::WHEELBASE_MM*0.001f,0.001f);
    const float maxK=1000.0f/fmaxf(cfg::MIN_TURN_RADIUS_MM,1.0f);
    const float brake=fmaxf(cfg::BRAKE_MPS2,0.001f);
    if (!(isfinite(in.speed[0]) && isfinite(in.speed[1]) &&
          in.dt>0 && in.dt<=0.06f)) { out.stop="BAD_CONTROL_INPUT"; return out; }
    for (int i=0;i<3;++i) {
      if (!fresh(in.s[i],in.now,i==0?cfg::FRONT_AGE_MS:cfg::SIDE_AGE_MS)) {
        out.stop="RANGE_INVALID_OR_STALE"; return out;
      }
      if (in.s[i].mm > cfg::CERTIFIED_MAX_MM[i]) { out.stop="RANGE_OUTSIDE_CALIBRATION"; return out; }
    }
    const float front=in.s[0].mm-cfg::FRONT_INSET_MM-cfg::ERROR_MM[0];
    const float left=in.s[1].mm-cfg::SIDE_INSET_MM[0]-cfg::ERROR_MM[1];
    const float right=in.s[2].mm-cfg::SIDE_INSET_MM[1]-cfg::ERROR_MM[2];
    if (fminf(left,right)<=cfg::MIN_CLEAR_MM) { out.stop="SIDE_CLEARANCE"; return out; }
    if (front<=cfg::FRONT_RESERVE_MM) { out.stop="FRONT_CLEARANCE"; return out; }
    // Bias-corrected nominal distances for steering; safety above subtracts uncertainty.
    const float l=in.s[1].mm-cfg::SIDE_INSET_MM[0];
    const float r=in.s[2].mm-cfg::SIDE_INSET_MM[1];
    const float target=(400-cfg::WIDTH_MM)*0.5f;
    const float wallError=cfg::FOLLOW_RIGHT?target-r:l-target;
    const float inferredWidth=l+r+cfg::WIDTH_MM;
    // Centre only in plausible narrow corridors. At the circular island stay
    // on the selected wall instead of steering across the island toward a mean.
    const float error=(inferredWidth>=330 && inferredWidth<=500)?(l-r)*0.5f:wallError;
    const uint32_t stamp=cfg::FOLLOW_RIGHT?in.s[2].stamp:in.s[1].stamp;
    if (stamp!=sideStamp) {
      if (sideStamp && uint32_t(stamp-sideStamp)<500) {
        const float ds=uint32_t(stamp-sideStamp)*0.001f;
        derivative=0.6f*derivative+0.4f*clip((error-prevError)/ds,-600,600);
      }
      sideStamp=stamp; prevError=error;
    }
    float steer=cfg::WALL_KP_RAD_PER_MM*error+cfg::WALL_KD_RAD_S_PER_MM*derivative;
    const float measured=fmaxf(0,fmaxf(in.speed[0],in.speed[1]));
    const float turnAt=500+measured*500;
    if (front<turnAt) {
      if (!turnSign) turnSign=(l>r+25)?1:((r>l+25)?-1:(cfg::FOLLOW_RIGHT?1:-1));
      const float span=rad(turnSign>0?cfg::LEFT_ANGLE_DEG:cfg::RIGHT_ANGLE_DEG);
      const float weight=clip((turnAt-front)/(turnAt-cfg::FRONT_RESERVE_MM),0,1);
      steer=(1-weight)*steer+weight*turnSign*span;
    } else if (front>turnAt+150) turnSign=0;
    steer=clip(steer,-rad(cfg::RIGHT_ANGLE_DEG),rad(cfg::LEFT_ANGLE_DEG));
    // Do not request tighter curvature than the measured steering envelope.
    const float k=clip(tanf(steer)/wheelbaseM,-maxK,maxK);
    steer=atanf(k*wheelbaseM);
    const float age=uint32_t(in.now-in.s[0].stamp)*0.001f;
    const float t=cfg::REACTION_S+age+cfg::COMMAND_LEASE_MS*0.001f;
    const float available=(front-cfg::FRONT_RESERVE_MM)*0.001f;
    const float frontCap=stopCap(available,brake,t);
    if (measured*t+measured*measured/(2*brake)>available) {
      out.stop="BRAKE_ENVELOPE"; return out;
    }
    float cap=fminf(cfg::INITIAL_MAX_MPS,frontCap);
    if (fabsf(k)>0.01f) cap=fminf(cap,sqrtf(cfg::LATERAL_MPS2/fabsf(k)));
    const float sideFactor=clip((fminf(left,right)-cfg::MIN_CLEAR_MM)/
                               (cfg::SIDE_SLOW_MM-cfg::MIN_CLEAR_MM),0,1);
    cap=fminf(cap,cfg::INITIAL_MAX_MPS*sideFactor);
    cap=fminf(cap,cfg::MAX_WHEEL_MPS/(1+fabsf(k)*cfg::REAR_TRACK_MM*0.0005f));
    // Never enforce a minimum speed or slew-limit a required decrease.
    speedCmd=fminf(cap,speedCmd+cfg::ACCEL_MPS2*in.dt);
    out.speed=speedCmd; out.cap=cap; out.steer=steer;
    return out;
  }
};
}
