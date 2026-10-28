// A20 v2.0: START/START1=1s per leg, START3=3s. Explicit start only.
// Straight -> brake -> steer left -> left arc -> brake -> right arc -> brake -> home.
#include <Arduino.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include "sequence.h"

portMUX_TYPE outputMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
Command request, desired;
Applied applied;
DriveTrace traces[8];
uint8_t traceCount = 0, printedTraces = 0;
volatile bool leaseFault = false, pwmFault = false;
volatile int32_t ticks[2] = {0, 0};
volatile uint32_t jumps[2] = {0, 0};
volatile uint8_t lastAB[2] = {0, 0};
DRAM_ATTR int8_t QT[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
Stage stage = Stage::IDLE;
uint32_t stageAt = 0, lastControl = 0, lastServoStep = 0, lastPrint = 0;
uint32_t lowSince = 0, settledSince = 0;
int targetUs = cfg::CENTER_US;
uint32_t selectedDriveMs = cfg::DEFAULT_DRIVE_MS;
int32_t currentTicks[2] = {0,0}, prevTicks[2] = {0,0}, legTicks[2] = {0,0};
uint32_t currentJumps[2] = {0,0}, prevJumps[2] = {0,0};
float velocity[2] = {0,0};
char line[24];
uint8_t lineLen = 0;
bool lineOverflow = false, ready = false;

void IRAM_ATTR encoder(unsigned wheel) {
  portENTER_CRITICAL_ISR(&encoderMux);
  uint8_t ab = (digitalRead(cfg::ENC[wheel][0]) << 1) | digitalRead(cfg::ENC[wheel][1]);
  uint8_t old = lastAB[wheel];
  if ((old ^ ab) == 3) ++jumps[wheel];
  ticks[wheel] += QT[(old << 2) | ab];
  lastAB[wheel] = ab;
  portEXIT_CRITICAL_ISR(&encoderMux);
}
void IRAM_ATTR encoderL() { encoder(0); }
void IRAM_ATTR encoderR() { encoder(1); }

const char* name(Stage s) {
  switch (s) {
    case Stage::IDLE: return "IDLE";
    case Stage::PRE_CENTER: return "PRE_CENTER";
    case Stage::COUNTDOWN: return "COUNTDOWN";
    case Stage::STRAIGHT: return "STRAIGHT";
    case Stage::BRAKE1: return "BRAKE_STRAIGHT";
    case Stage::STEER_LEFT: return "STEER_LEFT";
    case Stage::LEFT: return "LEFT_ARC";
    case Stage::BRAKE2: return "BRAKE_LEFT";
    case Stage::STEER_RIGHT: return "STEER_RIGHT";
    case Stage::RIGHT: return "RIGHT_ARC";
    case Stage::BRAKE3: return "BRAKE_RIGHT";
    case Stage::HOME: return "RETURN_CENTER";
    case Stage::DONE: return "DONE";
    case Stage::STOPPED: return "STOPPED";
    default: return "FAULT";
  }
}
bool driving(Stage s) { return s==Stage::STRAIGHT || s==Stage::LEFT || s==Stage::RIGHT; }
bool braking(Stage s) { return s==Stage::BRAKE1 || s==Stage::BRAKE2 || s==Stage::BRAKE3; }
bool active(Stage s) { return s!=Stage::IDLE && s!=Stage::DONE && s!=Stage::STOPPED && s!=Stage::FAULT; }

void publish() {
  desired.stamp = millis();
  portENTER_CRITICAL(&outputMux); request = desired; portEXIT_CRITICAL(&outputMux);
}

void actuatorTask(void*) {
  bool wasBraking = false, wasDriving = false;
  uint32_t brakeSince = 0, driveBegin = 0;
  uint8_t driveLeg = 0;
  int writtenA[2]={-1,-1}, writtenB[2]={-1,-1}, writtenServoUs=-1;
  for (;;) {
    Command c;
    portENTER_CRITICAL(&outputMux); c=request; portEXIT_CRITICAL(&outputMux);
    uint32_t now=millis();
    if (c.watch && uint32_t(now-c.stamp)>cfg::LEASE_MS) leaseFault=true;
    bool anyDuty = c.duty[0] || c.duty[1];
    // Independent per-leg cutoff even if the main state transition runs late.
    bool legDeadline = anyDuty && uint32_t(now-c.legStarted)>=min(c.legDuration,cfg::MAX_DRIVE_MS);
    bool stopping = c.brake || leaseFault || pwmFault || legDeadline;
    if (stopping && !wasBraking) brakeSince=now;
    wasBraking=stopping;
    bool driveNow=anyDuty && !stopping;
    if (driveNow && !wasDriving) { driveBegin=now; driveLeg=c.leg; }
    if (!driveNow && wasDriving) {
      portENTER_CRITICAL(&outputMux);
      if (traceCount<8) traces[traceCount++] = {driveLeg, driveBegin, now};
      portEXIT_CRITICAL(&outputMux);
    }
    wasDriving=driveNow;
    Applied next;
    bool ok=true;
    for (int w=0; w<2; ++w) {
      int a=0,b=0;
      if (stopping && uint32_t(now-brakeSince)<cfg::BRAKE_MS) a=b=255;
      else if (driveNow) {
        int d=constrain(abs(c.duty[w]),0,cfg::DUTY);
        if (c.duty[w]>0) { a=255; b=255-d; }
        else if (c.duty[w]<0) { b=255; a=255-d; }
      } else if (!stopping && c.watch) a=b=255;
      // Keep the watchdog loop independent of redundant peripheral updates.
      bool first=(a==writtenA[w]) || ledcWrite(cfg::MOTOR[2*w],a);
      bool second=(b==writtenB[w]) || ledcWrite(cfg::MOTOR[2*w+1],b);
      ok=first && second && ok;
      if (first) writtenA[w]=a;
      if (second) writtenB[w]=b;
      next.a[w]=a; next.b[w]=b;
    }
    // Preserve last steering during braking; disable it after an abort brake hold.
    bool servoOn=c.servo && !(stopping && !c.watch && uint32_t(now-brakeSince)>=cfg::BRAKE_MS);
    uint32_t servoDuty=servoOn ? (uint64_t(c.us)*32768+10000)/20000 : 0;
    int servoUs=servoOn ? c.us : 0;
    bool servoOk=(servoUs==writtenServoUs) || ledcWrite(cfg::SERVO,servoDuty);
    if (servoOk) writtenServoUs=servoUs;
    ok=servoOk && ok;
    next.servoUs=servoOn ? c.us : 0;
    portENTER_CRITICAL(&outputMux); applied=next; portEXIT_CRITICAL(&outputMux);
    if (!ok) {
      pwmFault=true;
      for (uint8_t pin:cfg::MOTOR) { ledcDetach(pin); digitalWrite(pin,LOW); pinMode(pin,OUTPUT); }
      ledcDetach(cfg::SERVO); digitalWrite(cfg::SERVO,LOW); pinMode(cfg::SERVO,OUTPUT);
      portENTER_CRITICAL(&outputMux); applied=Applied{}; portEXIT_CRITICAL(&outputMux);
      for (;;) vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void enter(Stage next) {
  stage=next; stageAt=millis(); lowSince=settledSince=0;
  desired.duty[0]=desired.duty[1]=0;
  desired.leg=0; desired.brake=braking(next);
  desired.watch=active(next); desired.servo=active(next);
  if (next==Stage::PRE_CENTER || next==Stage::COUNTDOWN) { targetUs=cfg::CENTER_US; desired.us=cfg::CENTER_US; }
  if (next==Stage::STEER_LEFT) targetUs=cfg::LEFT_US;
  if (next==Stage::STEER_RIGHT) targetUs=cfg::RIGHT_US;
  if (next==Stage::HOME) targetUs=cfg::CENTER_US;
  if (driving(next)) {
    desired.duty[0]=cfg::DUTY*cfg::MOTOR_SIGN[0];
    desired.duty[1]=cfg::DUTY*cfg::MOTOR_SIGN[1];
    desired.legStarted=stageAt;
    desired.legDuration=selectedDriveMs;
    desired.leg=next==Stage::STRAIGHT ? 1 : next==Stage::LEFT ? 2 : 3;
    legTicks[0]=currentTicks[0]; legTicks[1]=currentTicks[1];
  }
  publish();
  Serial.printf("STAGE,%lu,%s,us=%d,duty=%d/%d,drive_ms=%lu\n",(unsigned long)stageAt,name(stage),desired.us,desired.duty[0],desired.duty[1],(unsigned long)selectedDriveMs);
  if (next==Stage::DONE) Serial.println("DONE: one sequence; motor pins LOW, servo PWM OFF. START repeats deliberately.");
}

void stop(const char* reason, bool fault) {
  if (stage==Stage::STOPPED || stage==Stage::FAULT) return;
  stage=fault ? Stage::FAULT : Stage::STOPPED;
  stageAt=millis(); desired.duty[0]=desired.duty[1]=0;
  desired.brake=true; desired.watch=false; // Hold last steering during the brake interval.
  publish();
  Serial.printf("STOP,%lu,%s; %s latched, reset only with actuator power off.\n",(unsigned long)stageAt,reason,name(stage));
}

void readEncoders(uint32_t now) {
  float dt=(now-lastControl)*0.001f;
  if (dt<=0) return;
  portENTER_CRITICAL(&encoderMux);
  for (int w=0;w<2;++w) { currentTicks[w]=ticks[w]; currentJumps[w]=jumps[w]; }
  portEXIT_CRITICAL(&encoderMux);
  for (int w=0;w<2;++w) {
    int32_t delta=int32_t(uint32_t(currentTicks[w])-uint32_t(prevTicks[w]));
    velocity[w]=delta*cfg::ENC_SIGN[w]*cfg::CIRCUMFERENCE_MM/cfg::CPR[w]*0.001f/dt;
    if (active(stage) && currentJumps[w]!=prevJumps[w]) stop("ENCODER_ILLEGAL_TRANSITION",true);
    prevTicks[w]=currentTicks[w]; prevJumps[w]=currentJumps[w];
  }
}

void update(uint32_t now) {
  readEncoders(now); lastControl=now;
  if (leaseFault || pwmFault) { stop(leaseFault?"OUTPUT_LEASE_EXPIRED":"PWM_API",true); return; }
  if (!active(stage)) return;
  uint32_t elapsed=now-stageAt;
  if (stage==Stage::PRE_CENTER) {
    if (elapsed>=cfg::PRE_CENTER_HOLD_MS) enter(Stage::COUNTDOWN);
  } else if (stage==Stage::COUNTDOWN) {
    if (elapsed>=3000) enter(Stage::STRAIGHT);
  } else if (driving(stage)) {
    for (int w=0;w<2;++w) {
      int32_t delta=int32_t(uint32_t(currentTicks[w])-uint32_t(legTicks[w]));
      if (elapsed>=120 && delta*cfg::ENC_SIGN[w]<-8) { stop("ENCODER_REVERSE",true); return; }
    }
    if (elapsed>=selectedDriveMs) {
      Serial.printf("LEG_END,%lu,%s,request_ms=%lu\n",(unsigned long)now,name(stage),(unsigned long)elapsed);
      enter(stage==Stage::STRAIGHT?Stage::BRAKE1:stage==Stage::LEFT?Stage::BRAKE2:Stage::BRAKE3);
    }
  } else if (braking(stage)) {
    bool slow=fabsf(velocity[0])<cfg::LOW_SPEED_MPS && fabsf(velocity[1])<cfg::LOW_SPEED_MPS;
    if (slow) { if (!lowSince) lowSince=now; } else lowSince=0;
    if (elapsed>cfg::BRAKE_TIMEOUT_MS) { stop("BRAKE_NOT_SETTLED",true); return; }
    if (elapsed>=cfg::BRAKE_MS && lowSince && uint32_t(now-lowSince)>=cfg::LOW_SPEED_MS)
      enter(stage==Stage::BRAKE1?Stage::STEER_LEFT:stage==Stage::BRAKE2?Stage::STEER_RIGHT:Stage::HOME);
  } else { // Turn steering only after braking stages, with the motor commands zero.
    if (uint32_t(now-lastServoStep)>=20 && desired.us!=targetUs) {
      int delta=targetUs-desired.us;
      desired.us += delta>0 ? min(delta,cfg::SERVO_STEP_US) : max(delta,-cfg::SERVO_STEP_US);
      lastServoStep=now;
    }
    if (desired.us==targetUs) { if (!settledSince) settledSince=now; } else settledSince=0;
    uint32_t hold=stage==Stage::HOME ? cfg::HOME_HOLD_MS : cfg::SETTLE_MS;
    if (settledSince && uint32_t(now-settledSince)>=hold)
      enter(stage==Stage::STEER_LEFT?Stage::LEFT:stage==Stage::STEER_RIGHT?Stage::RIGHT:Stage::DONE);
  }
  publish();
}

void status() {
  Applied a;
  portENTER_CRITICAL(&outputMux); a=applied; portEXIT_CRITICAL(&outputMux);
  Serial.printf("STATUS stage=%s duty=%d/%d us=%d servo_applied=%d motor_raw=%d/%d/%d/%d lease=%d pwm_fault=%d drive_ms=%lu\n",
      name(stage),desired.duty[0],desired.duty[1],desired.us,a.servoUs,a.a[0],a.b[0],a.a[1],a.b[1],leaseFault,pwmFault,(unsigned long)selectedDriveMs);
}

void command(const char* text) {
  if (strcmp(text,"STATUS")==0) { status(); return; }
  uint32_t duration=cfg::DEFAULT_DRIVE_MS;
  if (strcmp(text,"START3")==0) duration=cfg::LONG_DRIVE_MS;
  else if (strcmp(text,"START")!=0 && strcmp(text,"START1")!=0) {
    Serial.println("Commands: START/START1=1s each leg, START3=3s each leg, STATUS; 0/x/! stops."); return;
  }
  if (!ready || leaseFault || pwmFault || (stage!=Stage::IDLE && stage!=Stage::DONE)) {
    Serial.println("START refused: busy or latched stop/fault."); return;
  }
  portENTER_CRITICAL(&outputMux); traceCount=0; portEXIT_CRITICAL(&outputMux);
  printedTraces=0;
  selectedDriveMs=duration;
  enter(Stage::PRE_CENTER);
}

void serialInput() {
  for (int n=0;n<64 && Serial.available();++n) {
    char c=Serial.read();
    if (c=='0' || c=='!' || c=='x' || c=='X') { stop("USER_STOP",false); lineLen=0; lineOverflow=false; return; }
    if (c=='\r' || c=='\n') {
      if (lineOverflow) Serial.println("Input too long; command discarded.");
      else if (lineLen) { line[lineLen]=0; command(line); }
      lineLen=0; lineOverflow=false;
    } else if (c!=' ' && c!='\t') {
      if (lineLen<sizeof(line)-1) line[lineLen++]=toupper((unsigned char)c);
      else lineOverflow=true;
    }
  }
}

void telemetry(uint32_t now) {
  if (uint32_t(now-lastPrint)<100) return;
  lastPrint=now;
  char row[240];
  int n=snprintf(row,sizeof(row),"G,%lu,%s,%d,%d,%d,%d,%ld,%ld,%.3f,%.3f\n",
    (unsigned long)now,name(stage),desired.duty[0],desired.duty[1],desired.us,desired.brake,
    (long)currentTicks[0],(long)currentTicks[1],velocity[0],velocity[1]);
  if (n>0 && n<int(sizeof(row)) && Serial.availableForWrite()>=n) Serial.write((uint8_t*)row,n);
  DriveTrace trace;
  bool available=false;
  portENTER_CRITICAL(&outputMux);
  if (printedTraces<traceCount) { trace=traces[printedTraces++]; available=true; }
  portEXIT_CRITICAL(&outputMux);
  if (available) Serial.printf("DRIVE_APPLIED,%u,begin=%lu,end=%lu,ms=%lu\n",trace.leg,
    (unsigned long)trace.begin,(unsigned long)trace.end,(unsigned long)(trace.end-trace.begin));
}

void setup() {
  Serial.setTxBufferSize(1024); Serial.begin(115200);
  for (uint8_t pin:cfg::MOTOR) { digitalWrite(pin,LOW); pinMode(pin,OUTPUT); }
  digitalWrite(cfg::SERVO,LOW); pinMode(cfg::SERVO,OUTPUT);
  bool ok=true;
  for (int i=0;i<4;++i) {
    bool attached=ledcAttachChannel(cfg::MOTOR[i],20000,8,i);
    bool zero=attached && ledcWrite(cfg::MOTOR[i],0);
    ok=ok && zero;
  }
  bool servo=ledcAttachChannel(cfg::SERVO,50,15,8);
  ok=(servo && ledcWrite(cfg::SERVO,0)) && ok;
  for (int w=0;w<2;++w) {
    pinMode(cfg::ENC[w][0],INPUT); pinMode(cfg::ENC[w][1],INPUT);
    lastAB[w]=(digitalRead(cfg::ENC[w][0])<<1)|digitalRead(cfg::ENC[w][1]);
    attachInterrupt(digitalPinToInterrupt(cfg::ENC[w][0]),w?encoderR:encoderL,CHANGE);
    attachInterrupt(digitalPinToInterrupt(cfg::ENC[w][1]),w?encoderR:encoderL,CHANGE);
  }
  if (ok) ok=xTaskCreatePinnedToCore(actuatorTask,"ground_outputs",4096,nullptr,3,nullptr,0)==pdPASS;
  if (!ok) {
    pwmFault=true; stage=Stage::FAULT;
    for (uint8_t pin:cfg::MOTOR) { ledcDetach(pin); digitalWrite(pin,LOW); pinMode(pin,OUTPUT); }
    ledcDetach(cfg::SERVO); digitalWrite(cfg::SERVO,LOW); pinMode(cfg::SERVO,OUTPUT);
  }
  ready=ok; lastControl=lastServoStep=millis();
  Serial.println("A20 v2.0 GROUND SEQUENCE - BOOT IDLE - START REQUIRED");
  Serial.printf("CONFIG center=%d left=%d right=%d duty=%d default_ms=%lu long_ms=%lu brake_ms=%lu servo_step_us=%d pre_center_ms=%lu home_hold_ms=%lu; no range avoidance\n",
      cfg::CENTER_US,cfg::LEFT_US,cfg::RIGHT_US,cfg::DUTY,(unsigned long)cfg::DEFAULT_DRIVE_MS,
      (unsigned long)cfg::LONG_DRIVE_MS,(unsigned long)cfg::BRAKE_MS,cfg::SERVO_STEP_US,
      (unsigned long)cfg::PRE_CENTER_HOLD_MS,(unsigned long)cfg::HOME_HOLD_MS);
  Serial.println("Order: center/hold, countdown, straight, brake, left arc, brake, right arc, brake, center/hold, DONE.");
  Serial.println("Commands: START/START1 = 1s each leg, START3 = 3s each leg; STATUS; 0/x/! stops and latches.");
  Serial.println("Use open test area, rear power off while flashing. Keep D057 disconnected with USB.");
  if (!ok) Serial.println("FAULT: PWM/task setup failed. Outputs forced LOW.");
}

void loop() {
  serialInput();
  uint32_t now=millis();
  if (uint32_t(now-lastControl)>=20) update(now);
  telemetry(now);
  delay(1);
}
