// A17 v1.0 -- ESP32-WROOM-32E / Arduino-ESP32 3.3.12 / Adafruit VL53L0X 1.2.5
// Boot never drives. Read README before BENCH / ARM / RUN. No Wi-Fi dependency.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "controller.h"

constexpr uint8_t MP[4]={25,26,32,33}, EP[2][2]={{18,19},{34,35}};
constexpr uint8_t XSH[3]={4,13,17}, SERVO=14; // front, left, right: confirmed vehicle wiring
enum class Mode : uint8_t { IDLE, ARMED, COUNTDOWN, RUNNING, BENCH, STOPPED, FAULT };
Mode mode=Mode::IDLE;
portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
struct Command {
  int duty[2]={0,0};
  int us=1500;
  bool servo=false, brake=false, watch=false;
  uint32_t stamp=0;
} request;
volatile bool guardTrip=false, pwmFault=false;
volatile int32_t ticks[2]={0,0};
volatile uint32_t jumps[2]={0,0};
volatile uint8_t lastAB[2]={0,0};
portMUX_TYPE encMux=portMUX_INITIALIZER_UNLOCKED;
DRAM_ATTR int8_t QT[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

void IRAM_ATTR encoder(unsigned w) {
  portENTER_CRITICAL_ISR(&encMux);
  const uint8_t ab=(digitalRead(EP[w][0])<<1)|digitalRead(EP[w][1]);
  const uint8_t old=lastAB[w];
  if ((old^ab)==3) ++jumps[w];
  ticks[w]+=QT[(old<<2)|ab]; lastAB[w]=ab;
  portEXIT_CRITICAL_ISR(&encMux);
}
void IRAM_ATTR encL() { encoder(0); }
void IRAM_ATTR encR() { encoder(1); }

// The actuator task is the ONLY runtime PWM writer. It never accesses I2C or
// Serial. A blocked sensor call cannot refresh the 80ms drive lease.
void actuatorTask(void*) {
  uint32_t brakeSince=0;
  bool braking=false;
  for (;;) {
    Command c;
    portENTER_CRITICAL(&mux); c=request; portEXIT_CRITICAL(&mux);
    const uint32_t now=millis();
    const bool expired=c.watch && uint32_t(now-c.stamp)>cfg::COMMAND_LEASE_MS;
    if (expired) guardTrip=true;
    const bool stop=c.brake || guardTrip || pwmFault;
    if (stop && !braking) { braking=true; brakeSince=now; }
    if (!stop) braking=false;
    bool ok=true;
    for (int w=0;w<2;++w) {
      int a=0,b=0;
      if (stop && uint32_t(now-brakeSince)<cfg::BRAKE_HOLD_MS) a=b=255;
      else if (!stop && c.duty[w]) {
        const int d=constrain(abs(c.duty[w]),0,cfg::MAX_DUTY);
        // Same complementary brake-PWM convention as A15 / Yahboom example.
        if (c.duty[w]>0) { a=255; b=255-d; }
        else { b=255; a=255-d; }
      }
      // Zero command means brake during RUN, coast only in explicit idle/off.
      else if (!stop && c.watch) a=b=255;
      const bool first=ledcWrite(MP[w*2],a);
      const bool second=ledcWrite(MP[w*2+1],b);
      ok=first && second && ok;
    }
    // Hold last steering during braking, never abruptly centre moving wheels.
    const uint32_t servoDuty=c.servo ? uint32_t(uint64_t(c.us)*32768/20000) : 0;
    ok=ledcWrite(SERVO,servoDuty) && ok;
    if (!ok) {
      pwmFault=true;
      for (uint8_t p:MP) { ledcDetach(p); digitalWrite(p,LOW); pinMode(p,OUTPUT); }
      ledcDetach(SERVO); digitalWrite(SERVO,LOW); pinMode(SERVO,OUTPUT);
      // Cannot trust PWM anymore. Remain low until reset.
      for (;;) vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

Adafruit_VL53L0X tof[3];
racer::Sample ranges[3];
const uint8_t sequence[4]={0,1,0,2}; // Front twice per frame, one emitter at a time.
uint8_t seq=0;
bool measuring=false, sensorsReady=false;
uint32_t exposureStart=0, stageAt=0, runAt=0, lastControl=0, lastPrint=0;
uint32_t goodSince=0, lastGood[3]={0,0,0}, worstGap[3]={0,0,0};
bool haveGood[3]={false,false,false};
float velocity[2]={0,0}, integral[2]={0,0}, distanceMm=0;
int32_t prevTicks[2]={0,0};
uint32_t prevJumps[2]={0,0}, stallAt[2]={0,0};
racer::Controller controller;
racer::Output latest;
Command desired;
char line[40]; uint8_t lineLen=0;
bool benchEnabled=false;
uint32_t benchAt=0;
uint32_t benchDuration=300;

void publish() {
  desired.stamp=millis();
  portENTER_CRITICAL(&mux); request=desired; portEXIT_CRITICAL(&mux);
}
void stop(const char* reason, bool fault=true) {
  desired.duty[0]=desired.duty[1]=0;
  desired.watch=false; desired.brake=true; publish();
  mode=fault?Mode::FAULT:Mode::STOPPED;
  Serial.printf("STOP,%lu,%s; reset EN to re-arm\n",(unsigned long)millis(),reason);
}
bool healthy(uint32_t now) {
  for (int i=0;i<3;++i)
    if (!racer::fresh(ranges[i],now,i?cfg::SIDE_AGE_MS:cfg::FRONT_AGE_MS) ||
        ranges[i].mm>cfg::CERTIFIED_MAX_MM[i]) return false;
  return sensorsReady && !guardTrip && !pwmFault;
}

bool initSensors() {
  Wire.begin(21,22); Wire.setClock(400000); Wire.setTimeOut(10);
  for (uint8_t p:XSH) { digitalWrite(p,LOW); pinMode(p,OUTPUT); }
  delay(10);
  for (int i=0;i<3;++i) {
    // Release to module pull-up: do not drive an unknown XSH voltage HIGH.
    pinMode(XSH[i],INPUT); delay(10);
    if (!tof[i].begin(0x30+i,false,&Wire)) return false;
    if (!tof[i].setMeasurementTimingBudgetMicroSeconds(20000)) return false;
    Serial.printf("TOF_INIT,%d,0x%02X,budget_us=%lu\n",i,0x30+i,
                  (unsigned long)tof[i].getMeasurementTimingBudgetMicroSeconds());
  }
  return true;
}
void pollRange() {
  if (!sensorsReady) return;
  const int i=sequence[seq];
  if (!measuring) {
    exposureStart=millis();
    if (!tof[i].startRange()) { sensorsReady=false; stop("RANGE_START_API"); return; }
    measuring=true; return;
  }
  // isRangeComplete returns true on API error too. Check Status before read.
  const bool ready=tof[i].isRangeComplete();
  if (tof[i].Status!=VL53L0X_ERROR_NONE) { sensorsReady=false; stop("RANGE_POLL_API"); return; }
  if (!ready) {
    if (uint32_t(millis()-exposureStart)>cfg::MEASURE_TIMEOUT_MS) {
      sensorsReady=false; stop("RANGE_TIMEOUT");
    }
    return;
  }
  // Avoid readRangeResult(): old library versions touch uninitialized status
  // on some errors. Check API result and RangeStatus==0 explicitly.
  VL53L0X_RangingMeasurementData_t m={};
  VL53L0X_Error err=tof[i].getRangingMeasurement(&m,false);
  if (err==VL53L0X_ERROR_NONE) err=tof[i].clearInterruptMask(false);
  const uint32_t now=millis();
  const bool valid=err==VL53L0X_ERROR_NONE && m.RangeStatus==0 &&
                   m.RangeMilliMeter>0 && m.RangeMilliMeter<8190;
  ranges[i].valid=valid;
  ranges[i].stamp=exposureStart;
  ranges[i].mm=m.RangeMilliMeter-cfg::BIAS_MM[i];
  if (valid) {
    if (haveGood[i]) worstGap[i]=max(worstGap[i],uint32_t(exposureStart-lastGood[i]));
    lastGood[i]=exposureStart; haveGood[i]=true;
  }
  // Nonblocking telemetry: skip if USB/UART buffer full; never postpone control.
  char row[150];
  int n=snprintf(row,sizeof(row),"R,%lu,%d,%lu,%u,%d,%u,%u,%lu\n",
      (unsigned long)now,i,(unsigned long)exposureStart,m.RangeMilliMeter,
      int(err),m.RangeStatus,valid,(unsigned long)worstGap[i]);
  if (n>0 && n<int(sizeof(row)) && Serial.availableForWrite()>=n) Serial.write((uint8_t*)row,n);
  if (!valid && (mode==Mode::RUNNING || mode==Mode::COUNTDOWN)) stop("RANGE_STATUS_OR_API");
  measuring=false; seq=(seq+1)%4;
}

void readEncoders(float dt) {
  int32_t t[2]; uint32_t j[2];
  portENTER_CRITICAL(&encMux);
  for (int w=0;w<2;++w) { t[w]=ticks[w]; j[w]=jumps[w]; }
  portEXIT_CRITICAL(&encMux);
  for (int w=0;w<2;++w) {
    // Unsigned subtraction handles counter wrap; convert delta back to signed.
    int32_t delta=int32_t(uint32_t(t[w])-uint32_t(prevTicks[w]));
    const float path=(cfg::CPR4[w]>0)?delta*cfg::ENCODER_SIGN[w]*
                     (cfg::WHEEL_MM[w]*3.14159265359f)/cfg::CPR4[w]:0;
    velocity[w]=path*0.001f/dt;
    if (mode==Mode::RUNNING) distanceMm+=0.5f*path;
    if (mode==Mode::RUNNING && j[w]!=prevJumps[w]) stop("ENCODER_ILLEGAL_TRANSITION");
    prevTicks[w]=t[w]; prevJumps[w]=j[w];
  }
}
void control(uint32_t now) {
  const float dt=uint32_t(now-lastControl)*0.001f;
  lastControl=now; readEncoders(dt);
  if (guardTrip || pwmFault) {
    if (mode!=Mode::FAULT) stop(guardTrip?"ACTUATOR_LEASE_EXPIRED":"PWM_API");
    return;
  }
  if (healthy(now)) { if (!goodSince) goodSince=now; } else goodSince=0;
  if (mode==Mode::BENCH) {
    if (uint32_t(now-benchAt)>=benchDuration) {
      desired.brake=true; desired.duty[0]=desired.duty[1]=0; desired.watch=false; publish(); mode=Mode::IDLE;
      Serial.printf("BENCH_BRAKE,%lu,command_ms=%lu,vL=%.3f,vR=%.3f\n",
        (unsigned long)now,(unsigned long)(now-benchAt),velocity[0],velocity[1]);
    }
    else publish();
    return;
  }
  if (mode==Mode::ARMED && uint32_t(now-stageAt)>10000) { stop("ARM_TIMEOUT",false); return; }
  if (mode==Mode::COUNTDOWN) {
    if (!healthy(now)) { stop("COUNTDOWN_RANGE"); return; }
    if (uint32_t(now-stageAt)<3000) return;
    mode=Mode::RUNNING; runAt=now; controller.reset(); distanceMm=0;
    integral[0]=integral[1]=0; stallAt[0]=stallAt[1]=now;
    Serial.println("RUNNING");
  }
  if (mode!=Mode::RUNNING) return;
  if (dt>0.06f) { stop("CONTROL_DEADLINE"); return; }
  if (uint32_t(now-runAt)>cfg::MAX_RUN_MS) { stop("MAX_RUN_TIME",false); return; }
  if (cfg::FINISH_DISTANCE_MM>0 && distanceMm>=cfg::FINISH_DISTANCE_MM) {
    stop("ODOMETRY_DISTANCE_REACHED_NOT_LINE_DETECTED",false); return;
  }
  racer::Input in; in.now=now; in.dt=dt;
  for (int i=0;i<3;++i) in.s[i]=ranges[i];
  in.speed[0]=velocity[0]; in.speed[1]=velocity[1];
  latest=controller.step(in);
  if (latest.stop) { stop(latest.stop); return; }
  const float k=tanf(latest.steer)/(cfg::WHEELBASE_MM*0.001f);
  const float split=racer::clip(k*cfg::REAR_TRACK_MM*0.0005f,-0.8f,0.8f);
  for (int w=0;w<2;++w) {
    const float target=latest.speed*(w?1+split:1-split);
    if (velocity[w]<-0.05f) { stop("WHEEL_REVERSED"); return; }
    if (velocity[w]>cfg::MAX_WHEEL_MPS*1.15f) { stop("OVERSPEED"); return; }
    if (target>0.06f && velocity[w]<0.015f) {
      if (uint32_t(now-stallAt[w])>500) { stop("STALL_OR_ENCODER_LOSS"); return; }
    } else stallAt[w]=now;
    float duty=0;
    if (velocity[w]>target+0.025f) {
      // A lower target must actually command braking, not retain static feedforward.
      integral[w]=0;
    } else if (target>0.005f) {
      const float e=target-velocity[w];
      const float ff=cfg::FF_STATIC[w]+cfg::FF_DUTY_PER_MPS[w]*target;
      const float raw=ff+cfg::SPEED_KP*e+integral[w];
      // Conditional integrator: do not wind up into saturation.
      if ((raw>0 && raw<cfg::MAX_DUTY) || (raw>=cfg::MAX_DUTY && e<0) || (raw<=0 && e>0))
        integral[w]=racer::clip(integral[w]+cfg::SPEED_KI*e*dt,-30,30);
      duty=racer::clip(ff+cfg::SPEED_KP*e+integral[w],0,cfg::MAX_DUTY);
    } else integral[w]=0;
    desired.duty[w]=int(lroundf(duty))*cfg::MOTOR_SIGN[w];
  }
  const float ratio=fabsf(latest.steer)/racer::rad(latest.steer>=0?cfg::LEFT_ANGLE_DEG:cfg::RIGHT_ANGLE_DEG);
  desired.us=cfg::CENTER_US+int((latest.steer>=0?cfg::LEFT_US-cfg::CENTER_US:cfg::RIGHT_US-cfg::CENTER_US)*ratio);
  desired.us=constrain(desired.us,1100,1900);
  desired.servo=true; desired.watch=true; desired.brake=false; publish();
}

void command(const char* s) {
  if (mode==Mode::FAULT || mode==Mode::STOPPED) {
    Serial.println("Latched stop: inspect cause, then EN. No automatic recovery."); return;
  }
  if (strcmp(s,"ARM")==0) {
    if (mode!=Mode::IDLE || !racer::calibrated() || !healthy(millis()) ||
        !goodSince || uint32_t(millis()-goodSince)<1000) {
      Serial.println("ARM refused: complete config, direction, geometry, brake and 1s fresh ranges."); return;
    }
    desired.servo=true; desired.us=cfg::CENTER_US; desired.brake=false; publish();
    mode=Mode::ARMED; stageAt=millis(); Serial.println("ARMED: RUN within 10s.");
  } else if (strcmp(s,"RUN")==0 && mode==Mode::ARMED) {
    mode=Mode::COUNTDOWN; stageAt=millis(); Serial.println("COUNTDOWN 3s; ! stops.");
  } else if (strcmp(s,"BENCH")==0 && mode==Mode::IDLE) {
    benchEnabled=true; Serial.println("BENCH enabled: lift all wheels; JOG L / JOG R = 300ms duty50.");
  } else if (benchEnabled && mode==Mode::IDLE &&
             (strcmp(s,"JOG L")==0 || strcmp(s,"JOG R")==0)) {
    const int w=(s[4]=='L')?0:1;
    desired.duty[0]=desired.duty[1]=0; desired.duty[w]=50*cfg::MOTOR_SIGN[w];
    desired.watch=true; desired.brake=false; benchAt=millis(); benchDuration=300; mode=Mode::BENCH; publish();
  } else if (benchEnabled && mode==Mode::IDLE && strcmp(s,"CAL ROLL")==0) {
    // Deliberate bounded MANUAL calibration, not autonomous RUN. Only after
    // wheel directions are checked. Operator supplies a clear test straight.
    if (!cfg::DIRECTION_CHECKED || cfg::CPR4[0]<=0 || cfg::CPR4[1]<=0 ||
        cfg::WHEEL_MM[0]<=0 || cfg::WHEEL_MM[1]<=0) {
      Serial.println("CAL ROLL refused: verify forward polarity and wheel/encoder calibration first."); return;
    }
    desired.duty[0]=50*cfg::MOTOR_SIGN[0]; desired.duty[1]=50*cfg::MOTOR_SIGN[1];
    desired.servo=true; desired.us=cfg::CENTER_US;
    desired.watch=true; desired.brake=false; benchAt=millis(); benchDuration=700;
    mode=Mode::BENCH; publish();
    Serial.printf("CAL_ROLL_START,%lu,duty=50,duration_ms=700\n",(unsigned long)benchAt);
  } else if (benchEnabled && mode==Mode::IDLE &&
            (strcmp(s,"CENTER")==0 || strcmp(s,"LEFT")==0 || strcmp(s,"RIGHT")==0)) {
    // First manual steering checks use only +/-100us, not unmeasured full span.
    desired.servo=true; desired.brake=false;
    desired.us=cfg::CENTER_US+(strcmp(s,"LEFT")==0?100*cfg::SERVO_LEFT_SIGN:
                            (strcmp(s,"RIGHT")==0?-100*cfg::SERVO_LEFT_SIGN:0)); publish();
  } else Serial.println("Commands: BENCH, JOG L, JOG R, CAL ROLL, CENTER, LEFT, RIGHT, ARM, RUN; !/x/0 stop.");
}
void serialInput() {
  for (int n=0;n<48 && Serial.available();++n) {
    const char c=Serial.read();
    if (c=='!' || c=='x' || c=='X' || c=='0') { stop("USER_STOP",false); lineLen=0; return; }
    if (c=='\n' || c=='\r') { if (lineLen) { line[lineLen]=0; command(line); lineLen=0; } }
    else if (lineLen<sizeof(line)-1) line[lineLen++]=c;
    else { lineLen=0; stop("SERIAL_LINE_TOO_LONG"); return; }
  }
}
void setup() {
  for (uint8_t p:MP) { digitalWrite(p,LOW); pinMode(p,OUTPUT); }
  digitalWrite(SERVO,LOW); pinMode(SERVO,OUTPUT);
  Serial.setTxBufferSize(2048); Serial.begin(115200); delay(200);
  bool ok=true;
  for (int i=0;i<4;++i) { const bool a=ledcAttachChannel(MP[i],20000,8,i); ok=a&&ok; }
  ok=ledcAttachChannel(SERVO,50,15,8)&&ok;
  if (!ok) {
    for(uint8_t p:MP) { ledcDetach(p); digitalWrite(p,LOW); pinMode(p,OUTPUT); }
    ledcDetach(SERVO); digitalWrite(SERVO,LOW); pinMode(SERVO,OUTPUT);
    mode=Mode::FAULT; Serial.println("PWM_INIT_FAILED: outputs low, EN required."); return;
  }
  if (xTaskCreatePinnedToCore(actuatorTask,"actuator",3072,nullptr,3,nullptr,0)!=pdPASS) {
    mode=Mode::FAULT; Serial.println("ACTUATOR_TASK_FAILED"); return;
  }
  for (int w=0;w<2;++w) {
    pinMode(EP[w][0],INPUT); pinMode(EP[w][1],INPUT); // 34/35 have no internal pulls.
    lastAB[w]=(digitalRead(EP[w][0])<<1)|digitalRead(EP[w][1]);
  }
  attachInterrupt(EP[0][0],encL,CHANGE); attachInterrupt(EP[0][1],encL,CHANGE);
  attachInterrupt(EP[1][0],encR,CHANGE); attachInterrupt(EP[1][1],encR,CHANGE);
  Serial.println("A17 v1.0: boot IDLE; calibrated=false until measured config completed.");
  Serial.printf("TOF_MAP,front=%u,left=%u,right=%u\n",XSH[0],XSH[1],XSH[2]);
  Serial.printf("DRIVE_SIGNS,motor=%d/%d,encoder=%d/%d,direction_checked=%d,commissioned=%d\n",
    cfg::MOTOR_SIGN[0],cfg::MOTOR_SIGN[1],cfg::ENCODER_SIGN[0],cfg::ENCODER_SIGN[1],
    int(cfg::DIRECTION_CHECKED),int(cfg::COMMISSIONED));
  Serial.printf("WHEEL_CAL,cpr4=%.1f/%.1f,diameter_mm=%.2f/%.2f\n",
    cfg::CPR4[0],cfg::CPR4[1],cfg::WHEEL_MM[0],cfg::WHEEL_MM[1]);
  Serial.println("R,time_ms,sensor,exposure_ms,raw_mm,api_error,range_status,valid,worst_gap_ms");
  sensorsReady=initSensors();
  if (!sensorsReady) stop("TOF_INIT");
  lastControl=millis();
}
void loop() {
  serialInput();
  uint32_t now=millis();
  if (uint32_t(now-lastControl)>=20) control(now);
  pollRange();
  now=millis();
  if (uint32_t(now-lastPrint)>=100) {
    lastPrint=now;
    int32_t t[2]; portENTER_CRITICAL(&encMux); t[0]=ticks[0];t[1]=ticks[1]; portEXIT_CRITICAL(&encMux);
    char row[220];
    int n=snprintf(row,sizeof(row),"C,%lu,%u,%.3f,%.3f,%ld,%ld,%.1f,%.3f,%.3f,%.3f,%d,%d,%d,%lu,%lu,%lu\n",
       (unsigned long)now,unsigned(mode),velocity[0],velocity[1],(long)t[0],(long)t[1],distanceMm,
       latest.speed,latest.cap,latest.steer,desired.duty[0],desired.duty[1],desired.us,
       (unsigned long)(now-ranges[0].stamp),(unsigned long)(now-ranges[1].stamp),(unsigned long)(now-ranges[2].stamp));
    if(n>0 && n<int(sizeof(row)) && Serial.availableForWrite()>=n) Serial.write((uint8_t*)row,n);
  }
  delay(1);
}
