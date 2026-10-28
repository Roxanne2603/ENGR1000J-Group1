// A22 v2.14: right-wall-only start, left entry then right arc with 1.5s held front-wall left escape, history flags.
#include <Arduino.h>
#include <Wire.h>
#include <esp_system.h>
#include <math.h>
#include <ctype.h>
#include <atomic>
#include "policy.h"
#include "fault_history.h"
#include "src/driver/Adafruit_VL53L0X.h"
#include "src/driver/transport_guard.h"
#include "src/driver/timing_guard.h"

race::Range ranges[3];
bool rightOnlyStart=true;uint8_t leftReacquired=0;
std::atomic<bool> transportFault{false},distanceTrip{false},motionGuard{false};
void onRange(uint8_t i,uint16_t mm,uint32_t at,bool valid) {
  ranges[i].observe(mm,at,valid);
  if(i==1&&rightOnlyStart){
    if(ranges[i].valid&&mm>65&&mm<=600){if(++leftReacquired>=3){rightOnlyStart=false;Serial.println("START_OPENING_EXIT,left_wall_reacquired");}}
    else leftReacquired=0;
  }
  if(motionGuard.load()&&ranges[i].valid&&mm<=(i?race::SIDE_HARD:race::FRONT_HARD))distanceTrip.store(true);
}
void onSensorError(uint8_t i){ranges[i].valid=false;transportFault.store(true);}
uint32_t budgetFor(uint8_t i){return i?33000:50000;}
#include "sensors.h"

constexpr uint8_t MOTOR[4]={25,26,32,33},ENC[2][2]={{18,19},{34,35}},SERVO=14;
constexpr int MOTOR_SIGN[2]={1,-1},ENC_SIGN[2]={1,-1};
constexpr float CPR[2]={824.2f,825.7f},CIRC_MM=650.0f/3.0f;
portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED,encMux=portMUX_INITIALIZER_UNLOCKED;
volatile int32_t ticks[2]={0,0};volatile uint32_t jumps[2]={0,0};volatile uint8_t lastAB[2]={0,0};
DRAM_ATTR int8_t QT[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
std::atomic<bool> leaseFault{false},pwmFault{false},deadlineTrip{false};
struct Command {
  int duty[2]={0,0},us=race::CENTER;bool servo=false,brake=false,watch=false,drive=false;
  uint32_t stamp=0,begun=0;
};
struct Applied {int a[2]={0,0},b[2]={0,0},us=0;};
Command desired,requested;Applied applied;
Stage stage=Stage::WAIT;
uint32_t stageAt=0,stableAt=0,lastControl=0,lastPrint=0,slowAt=0,runAt=0,progressAt=0;
int32_t currentTicks[2]={0,0},prevTicks[2]={0,0},progressTicks[2]={0,0};uint32_t prevJumps[2]={0,0};
float velocity[2]={0,0},targetSpeed[2]={0,0},distanceMm[2]={0,0};
race::Controller controller;race::WheelLoop wheel[2];
bool endedFault=false,feedbackFault=false,degraded=false;
bool firstIslandPhase=true,firstIslandTurn=false,firstIslandArc=false;
uint32_t firstIslandTurnAt=0,firstIslandArcAt=0;float firstIslandStartMm=0,firstIslandArcStartMm=0;
const char* stopReason="NONE";
history::Store faultHistory;history::Record pendingHistory;bool historyPending=false,historySaved=false;
void queueHistory(const char* reason,Stage origin) {
  if(historyPending||historySaved)return;
  pendingHistory.ms=millis();pendingHistory.reset=uint8_t(esp_reset_reason());pendingHistory.origin=uint8_t(origin);pendingHistory.opening=rightOnlyStart;
  pendingHistory.flags=uint8_t((firstIslandPhase?1:0)|(firstIslandTurn?2:0)|(firstIslandArc?16:0));
  if(controller.turnCommit()>0)pendingHistory.flags|=4; else if(controller.turnCommit()<0)pendingHistory.flags|=8;
  pendingHistory.servo_us=int16_t(desired.us);
  pendingHistory.turn=int8_t(desired.us>race::CENTER?-1:(desired.us<race::CENTER?1:0));
  strncpy(pendingHistory.reason,reason,sizeof(pendingHistory.reason)-1);
  for(int i=0;i<3;++i){pendingHistory.raw[i]=ranges[i].lastRaw;pendingHistory.age[i]=millis()-ranges[i].goodAt;if(ranges[i].valid)pendingHistory.valid|=1<<i;}
  historyPending=true;
}
void saveHistoryWhenOff() {
  if(!historyPending||historySaved||(stage!=Stage::FAULT&&stage!=Stage::DONE)||desired.watch||desired.drive)return;
  Applied a;portENTER_CRITICAL(&mux);a=applied;portEXIT_CRITICAL(&mux);
  if(a.us||a.a[0]||a.a[1]||a.b[0]||a.b[1])return;
  historySaved=true;bool ok=faultHistory.append(pendingHistory);
  Serial.printf("HISTORY_SAVED,ok=%d,reason=%s\n",ok,pendingHistory.reason);
}

void IRAM_ATTR encoder(unsigned w) {
  portENTER_CRITICAL_ISR(&encMux);
  uint8_t ab=(digitalRead(ENC[w][0])<<1)|digitalRead(ENC[w][1]),old=lastAB[w];
  if((old^ab)==3)++jumps[w];ticks[w]+=QT[(old<<2)|ab];lastAB[w]=ab;
  portEXIT_CRITICAL_ISR(&encMux);
}
void IRAM_ATTR encL(){encoder(0);} void IRAM_ATTR encR(){encoder(1);}
const char* name(Stage s) {
  switch(s){case Stage::WAIT:return "WAIT10";case Stage::CENTER:return "CENTER";
    case Stage::COUNTDOWN:return "COUNTDOWN3";case Stage::RUN:return "RUN";
    case Stage::BRAKE:return "BRAKE";case Stage::HOME:return "HOME";
    case Stage::DONE:return "DONE";default:return "FAULT";}
}
const char* issueName(race::Issue i) {
  switch(i){case race::Issue::OK:return "OK";case race::Issue::FRONT_HARD:return "FRONT_LE_30";
    case race::Issue::SIDE_HARD:return "SIDE_LE_50";case race::Issue::FRONT_RESERVE:return "FRONT_EARLY_BRAKE";
    case race::Issue::SIDE_RESERVE:return "SIDE_EARLY_BRAKE";case race::Issue::FRONT_UNKNOWN:return "FRONT_UNKNOWN";
    case race::Issue::LEFT_UNKNOWN:return "LEFT_UNKNOWN";case race::Issue::RIGHT_UNKNOWN:return "RIGHT_UNKNOWN";
    case race::Issue::NO_TURN_SPACE:return "NO_TURN_SPACE";default:return "BAD_CONTROL_INPUT";}
}
bool activeApproach(){return stage==Stage::CENTER||stage==Stage::COUNTDOWN||stage==Stage::RUN;}
void publish(){desired.stamp=millis();portENTER_CRITICAL(&mux);requested=desired;portEXIT_CRITICAL(&mux);}
void forceLow(){for(uint8_t p:MOTOR){ledcDetach(p);digitalWrite(p,LOW);pinMode(p,OUTPUT);}ledcDetach(SERVO);digitalWrite(SERVO,LOW);pinMode(SERVO,OUTPUT);}
void actuatorTask(void*) {
  bool wasBraking=false,armedEver=false;uint32_t brakeAt=0;
  int writtenA[2]={-1,-1},writtenB[2]={-1,-1},writtenUs=-1;
  for(;;) {
    Command c;portENTER_CRITICAL(&mux);c=requested;portEXIT_CRITICAL(&mux);
    uint32_t now=millis();if(c.watch)armedEver=true;
    if(c.watch&&race::elapsed(now,c.stamp,race::LEASE_MS+1))leaseFault.store(true);
    if(c.drive&&race::elapsed(now,c.begun,race::MAX_RUN_MS))deadlineTrip.store(true);
    bool fatal=leaseFault.load()||transportFault.load()||distanceTrip.load()||deadlineTrip.load();
    bool stopping=c.brake||(fatal&&armedEver);
    if(stopping&&!wasBraking)brakeAt=now;wasBraking=stopping;
    Applied a;bool ok=true;
    for(int w=0;w<2;++w) {
      auto p=race::motorPins(c.duty[w],MOTOR_SIGN[w],!stopping&&(c.watch||c.drive),stopping&&uint32_t(now-brakeAt)<2000);
      a.a[w]=p.a;a.b[w]=p.b;
      bool x=a.a[w]==writtenA[w]||ledcWrite(MOTOR[2*w],a.a[w]);
      bool y=a.b[w]==writtenB[w]||ledcWrite(MOTOR[2*w+1],a.b[w]);ok=x&&y&&ok;
      if(x)writtenA[w]=a.a[w];if(y)writtenB[w]=a.b[w];
    }
    bool servoOn=c.servo&&!(stopping&&uint32_t(now-brakeAt)>=2000);
    a.us=servoOn?constrain(c.us,race::LEFT,race::RIGHT):0;
    bool z=a.us==writtenUs||ledcWrite(SERVO,a.us?(uint64_t(a.us)*32768+10000)/20000:0);
    if(z)writtenUs=a.us;
    portENTER_CRITICAL(&mux);applied=a;portEXIT_CRITICAL(&mux);
    if(!ok||!z){pwmFault.store(true);forceLow();for(;;)vTaskDelay(pdMS_TO_TICKS(10));}
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
void enter(Stage next) {
  stage=next;stageAt=millis();slowAt=0;desired.duty[0]=desired.duty[1]=0;
  desired.brake=false;desired.drive=next==Stage::RUN;
  desired.watch=next==Stage::CENTER||next==Stage::COUNTDOWN||next==Stage::RUN||next==Stage::HOME;
  desired.servo=desired.watch;motionGuard.store(next==Stage::CENTER||next==Stage::COUNTDOWN||next==Stage::RUN);
  if(next==Stage::CENTER||next==Stage::COUNTDOWN)desired.us=race::CENTER;
  if(next==Stage::RUN) {
    controller.reset();for(auto& w:wheel)w.reset();runAt=stageAt;desired.begun=runAt;progressAt=runAt;
    firstIslandPhase=true;firstIslandTurn=false;firstIslandArc=false;firstIslandTurnAt=0;firstIslandArcAt=0;firstIslandStartMm=0;firstIslandArcStartMm=0;
    for(int w=0;w<2;++w){progressTicks[w]=currentTicks[w];distanceMm[w]=0;}
  }
  publish();Serial.printf("STAGE,%lu,%s\n",(unsigned long)stageAt,name(stage));
}
void stop(const char* why,bool fault) {
  if(stage==Stage::BRAKE||stage==Stage::HOME||stage==Stage::DONE||stage==Stage::FAULT)return;
  queueHistory(why,stage);stopReason=why;endedFault=fault;motionGuard.store(false);
  desired.duty[0]=desired.duty[1]=0;desired.drive=false;
  if(stage==Stage::WAIT){desired=Command{};enter(Stage::FAULT);}
  else {stage=Stage::BRAKE;stageAt=millis();slowAt=0;desired.brake=true;desired.watch=true;publish();}
  Serial.printf("STOP,%lu,%s,latched=1\n",(unsigned long)millis(),why);
}
race::Input frame(uint32_t now) {
  race::Input in;in.now=now;for(int i=0;i<3;++i)in.s[i]=ranges[i];
  in.rightOnlyStart=rightOnlyStart;
  in.firstIslandMode=firstIslandPhase;
  in.firstIslandTurn=firstIslandTurn;
  in.firstIslandArc=firstIslandArc;
  in.speed[0]=velocity[0];in.speed[1]=velocity[1];return in;
}
void update(uint32_t now) {
  float dt=uint32_t(now-lastControl)*.001f;lastControl=now;uint32_t j[2];
  portENTER_CRITICAL(&encMux);for(int w=0;w<2;++w){currentTicks[w]=ticks[w];j[w]=jumps[w];}portEXIT_CRITICAL(&encMux);
  for(int w=0;w<2;++w) {
    int32_t d=int32_t(uint32_t(currentTicks[w])-uint32_t(prevTicks[w]));
    float raw=d*ENC_SIGN[w]*CIRC_MM/CPR[w]*.001f/dt;velocity[w]=.55f*velocity[w]+.45f*raw;
    if(j[w]!=prevJumps[w]&&(activeApproach()||stage==Stage::BRAKE||stage==Stage::HOME))feedbackFault=true;
    if(stage==Stage::RUN){
      distanceMm[w]+=d*ENC_SIGN[w]*CIRC_MM/CPR[w];
      if(raw<-.08f||distanceMm[w]<-2)stop("ENCODER_REVERSE",true);
      if(fabsf(raw)>.55f||fabsf(velocity[w])>race::OVERSPEED)stop("OVERSPEED",true);
    }
    prevTicks[w]=currentTicks[w];prevJumps[w]=j[w];
  }
  if(leaseFault.load()||pwmFault.load()) {
    stopReason=leaseFault.load()?"LEASE_EXPIRED":"PWM_API";queueHistory(stopReason,stage);desired.drive=false;
    desired.duty[0]=desired.duty[1]=0;desired.watch=false;desired.brake=true;motionGuard.store(false);stage=Stage::FAULT;publish();return;
  }
  if(activeApproach()&&(transportFault.load()||distanceTrip.load()||feedbackFault)) {
    stop(transportFault.load()?"I2C_OR_TIMEOUT":distanceTrip.load()?"RAW_HARD_FLOOR":"ENCODER_ILLEGAL",true);
  }
  if(stage==Stage::HOME&&(feedbackFault||transportFault.load())){desired.watch=false;desired.servo=false;desired.brake=true;stage=Stage::FAULT;stopReason="HOME_FEEDBACK_UNCERTAIN";}
  if(stage==Stage::WAIT) {
    if(transportFault.load()||tof::fatalInit||!tof::available[0]||!tof::available[1]||!tof::available[2]){stop("INIT_FAILED",true);return;}
    if(race::ready(frame(now))){if(!stableAt)stableAt=now;}else stableAt=0;
    if(race::elapsed(now,stageAt,10000)) {
      if(stableAt&&race::elapsed(now,stableAt,500))enter(Stage::CENTER);
      else stop("START_RANGES_NOT_READY",true);
    }
  } else if(activeApproach()) {
    if(stage==Stage::RUN&&firstIslandPhase) {
      if(!firstIslandTurn&&!firstIslandArc&&ranges[0].valid&&ranges[0].mm<480&&ranges[1].valid&&ranges[1].mm>80){
        firstIslandTurn=true;firstIslandTurnAt=now;firstIslandStartMm=(distanceMm[0]+distanceMm[1])*0.5f;
      }
      if(firstIslandTurn) {
        float travelled=(distanceMm[0]+distanceMm[1])*0.5f-firstIslandStartMm;
        if(travelled>=250.0f||(ranges[0].valid&&ranges[0].mm<300)){
          firstIslandTurn=false;firstIslandArc=true;firstIslandArcAt=now;firstIslandArcStartMm=(distanceMm[0]+distanceMm[1])*0.5f;
        }
      } else if(firstIslandArc) {
        float travelled=(distanceMm[0]+distanceMm[1])*0.5f-firstIslandArcStartMm;
        bool maxTurn=race::elapsed(now,firstIslandArcAt,35000);
        if(travelled>=3000.0f||maxTurn){firstIslandPhase=false;firstIslandArc=false;}
      }
    }
    auto o=controller.step(frame(now));degraded=o.degraded;
    if(o.issue!=race::Issue::OK){stop(issueName(o.issue),true);return;}
    if(stage==Stage::CENTER&&race::elapsed(now,stageAt,1000))enter(Stage::COUNTDOWN);
    else if(stage==Stage::COUNTDOWN&&race::elapsed(now,stageAt,3000))enter(Stage::RUN);
    else if(stage==Stage::RUN) {
      if(race::elapsed(now,runAt,race::MAX_RUN_MS)||deadlineTrip.load()){stop("MAX_RUN_TIME",false);return;}
      if(rightOnlyStart&&(race::elapsed(now,runAt,20000)||race::maxf(distanceMm[0],distanceMm[1])>=1500)){
        stop("START_OPENING_LIMIT",true);return;
      }
      desired.us=race::approach(desired.us,o.us);
      for(int w=0;w<2;++w){targetSpeed[w]=o.speed[w];desired.duty[w]=wheel[w].step(o.speed[w],velocity[w],dt);}
      if(race::elapsed(now,progressAt,700)) {
        for(int w=0;w<2;++w) {
          int32_t delta=int32_t(uint32_t(currentTicks[w])-uint32_t(progressTicks[w]))*ENC_SIGN[w];
          if(desired.duty[w]>=28&&targetSpeed[w]>.06f&&delta<3){stop("STALL_OR_ENCODER",true);return;}
          progressTicks[w]=currentTicks[w];
        }
        progressAt=now;
      }
    }
  } else if(stage==Stage::BRAKE) {
    bool slow=fabsf(velocity[0])<.03f&&fabsf(velocity[1])<.03f;
    if(slow){if(!slowAt)slowAt=now;}else slowAt=0;
    if(race::elapsed(now,stageAt,5000)||feedbackFault){
      desired.watch=false;desired.servo=false;stage=Stage::FAULT;stopReason="BRAKE_FEEDBACK_UNCERTAIN";
    } else if(race::elapsed(now,stageAt,2000)&&slowAt&&race::elapsed(now,slowAt,300)) {
      if(transportFault.load()){desired.watch=false;desired.servo=false;stage=Stage::FAULT;}
      else {desired.drive=false;publish();distanceTrip.store(false);deadlineTrip.store(false);enter(Stage::HOME);}
    }
  } else if(stage==Stage::HOME) {
    desired.us=race::approach(desired.us,race::CENTER);
    if(desired.us!=race::CENTER)stageAt=now;
    else if(race::elapsed(now,stageAt,1500))enter(endedFault?Stage::FAULT:Stage::DONE);
  }
  publish();
}
void status() {
  Applied a;portENTER_CRITICAL(&mux);a=applied;portEXIT_CRITICAL(&mux);
  uint32_t now=millis();
  Serial.printf("STATUS A22v2.14 stage=%s reason=%s duty=%d/%d servo_us=%d raw=%d/%d/%d/%d lease=%d transport=%d degraded=%d open_start=%d reset=%d\n",
    name(stage),stopReason,desired.duty[0],desired.duty[1],a.us,a.a[0],a.b[0],a.a[1],a.b[1],leaseFault.load(),transportFault.load(),degraded,rightOnlyStart,int(esp_reset_reason()));
  for(int i=0;i<3;++i)Serial.printf("RANGE,%d,last_raw=%d,last_good=%d,valid=%d,seen=%d,age_ms=%lu,closing=%.1f\n",i,ranges[i].lastRaw,ranges[i].mm,ranges[i].valid,ranges[i].seen,(unsigned long)(now-ranges[i].goodAt),ranges[i].closing);
  Serial.println("CONFIG side_aim=130 side_soft=100 side_hard=50 front_soft=50 front_hard=30 early_stop_F80/S65 center=1375 left=575 right=2175; cruise=0.20mps duty_max=70 run_max=180s; no finish-line detection");
}
char input[24];uint8_t inputLen=0;bool overflow=false;
void serialInput() {
  for(int n=0;n<48&&Serial.available();++n) {
    char c=char(Serial.read());
    if(c=='!'||c=='0'||c=='x'||c=='X'){stop("USER_STOP",true);inputLen=0;overflow=false;return;}
    if(c=='\r'||c=='\n') {
      input[inputLen]=0;if(!overflow&&!strcmp(input,"STATUS"))status();
      else if(!overflow&&!strcmp(input,"HISTORY")&&!desired.watch)faultHistory.print();
      else if(inputLen)Serial.println("STATUS; !/0/x stops; reset/power restarts one countdown. No serial RUN bypass.");
      inputLen=0;overflow=false;
    } else if(c!=' '&&c!='\t'){if(inputLen<sizeof(input)-1)input[inputLen++]=toupper((unsigned char)c);else overflow=true;}
  }
}
bool sensorReadOK(uint8_t i){return !checked_i2c::latch.failed()&&tof::sensor[i].Status==VL53L0X_ERROR_NONE;}
bool verifyProfiles() {
  for(uint8_t i=0;i<3;++i){auto& s=tof::sensor[i];if(!tof::available[i])return false;
    checked_i2c::latch.clear();uint32_t b=s.getMeasurementTimingBudgetMicroSeconds();if(!sensorReadOK(i))return false;
    checked_i2c::latch.clear();auto signal=s.getLimitCheckValue(VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE);if(!sensorReadOK(i))return false;
    checked_i2c::latch.clear();auto sigma=s.getLimitCheckValue(VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE);if(!sensorReadOK(i))return false;
    checked_i2c::latch.clear();auto ignore=s.getLimitCheckEnable(VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD);if(!sensorReadOK(i))return false;
    checked_i2c::latch.clear();auto sigOn=s.getLimitCheckEnable(VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE);if(!sensorReadOK(i))return false;
    checked_i2c::latch.clear();auto sigmaOn=s.getLimitCheckEnable(VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE);if(!sensorReadOK(i))return false;
    Serial.printf("SENSOR_CONFIG,%u,budget_us=%lu,signal_mcps=%.5f,sigma_mm=%.1f,range_ignore=%u,signal_check=%u,sigma_check=%u\n",i,(unsigned long)b,signal/65536.0,sigma/65536.0,ignore,sigOn,sigmaOn);
    if(b<budgetFor(i)-2000||b>budgetFor(i)+2000||ignore||sigOn!=1||sigmaOn!=1)return false;
  }return true;
}
bool recoverAndInit() {
  for(int attempt=0;attempt<2;++attempt) {
    // Only called in setup while every motor command is LOW and drive/watch are false.
    tof::shutdownAll();delay(40);Wire.end();
    pinMode(21,INPUT_PULLUP);pinMode(22,OUTPUT_OPEN_DRAIN);digitalWrite(22,HIGH);delayMicroseconds(10);
    if(digitalRead(22)==HIGH) {
      if(digitalRead(21)==LOW){
        for(int pulse=0;pulse<9;++pulse){digitalWrite(22,LOW);delayMicroseconds(10);digitalWrite(22,HIGH);delayMicroseconds(10);}
        pinMode(21,OUTPUT_OPEN_DRAIN);digitalWrite(21,LOW);delayMicroseconds(10);
        digitalWrite(22,HIGH);delayMicroseconds(10);digitalWrite(21,HIGH);delayMicroseconds(10);
      }
      pinMode(21,INPUT_PULLUP);pinMode(22,INPUT_PULLUP);
      if(digitalRead(21)==HIGH&&digitalRead(22)==HIGH&&Wire.begin(21,22)) {
        Wire.setClock(400000);Wire.setTimeOut(20);tof::fatalInit=false;tof::current=tof::frame=0;
        tof::initSensors();
        if(!tof::fatalInit&&verifyProfiles()){Serial.printf("BUS_INIT_OK,attempt=%d\n",attempt+1);return true;}
      }
    }
    tof::shutdownAll();Serial.printf("BUS_INIT_RETRY,attempt=%d\n",attempt+1);delay(100);
  }
  tof::abortInit("BUS_INIT_FAILED_AFTER_2_ATTEMPTS");return false;
}
void setup() {
  for(uint8_t p:MOTOR){digitalWrite(p,LOW);pinMode(p,OUTPUT);}digitalWrite(SERVO,LOW);pinMode(SERVO,OUTPUT);
  Serial.setTxBufferSize(2048);Serial.begin(115200);delay(500);
  Serial.println("A22 v2.14 CONTINUOUS TRACK RACE - RIGHT WALL ONLY START - AUTO ONCE 10+1+3s");
  faultHistory.load();faultHistory.print();
  bool ok=true;
  for(int i=0;i<4;++i){bool a=ledcAttachChannel(MOTOR[i],20000,8,i);bool b=a&&ledcWrite(MOTOR[i],0);ok=b&&ok;}
  bool a=ledcAttachChannel(SERVO,50,15,8);ok=(a&&ledcWrite(SERVO,0))&&ok;
  for(int w=0;w<2;++w){pinMode(ENC[w][0],INPUT);pinMode(ENC[w][1],INPUT);lastAB[w]=(digitalRead(ENC[w][0])<<1)|digitalRead(ENC[w][1]);
    attachInterrupt(digitalPinToInterrupt(ENC[w][0]),w?encR:encL,CHANGE);attachInterrupt(digitalPinToInterrupt(ENC[w][1]),w?encR:encL,CHANGE);}
  if(ok)ok=xTaskCreatePinnedToCore(actuatorTask,"race_output",4096,nullptr,3,nullptr,0)==pdPASS;
  if(!ok){pwmFault.store(true);forceLow();stage=Stage::FAULT;stopReason="PWM_INIT";}
  if(!recoverAndInit())transportFault.store(true);
  if(esp_reset_reason()!=ESP_RST_POWERON){stage=Stage::FAULT;stopReason="RESET_REASON_LOCK";}
  if(stage==Stage::FAULT)queueHistory(stopReason,stage);
  stageAt=lastControl=millis();publish();status();
}
void loop() {
  serialInput();uint32_t now=millis();if(uint32_t(now-lastControl)>=20)update(now);
  tof::pollSensors();now=millis();if(uint32_t(now-lastControl)>=20)update(now);
  saveHistoryWhenOff();
  if(uint32_t(now-lastPrint)>=100){lastPrint=now;char row[230];
    int mask=int(ranges[0].valid)|(int(ranges[1].valid)<<1)|(int(ranges[2].valid)<<2);
    int n=snprintf(row,sizeof(row),"RACE,%lu,%s,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%d,%d\n",(unsigned long)now,name(stage),
      ranges[0].lastRaw,ranges[1].lastRaw,ranges[2].lastRaw,desired.us,desired.duty[0],desired.duty[1],velocity[0],velocity[1],targetSpeed[0],targetSpeed[1],degraded,mask);
    if(n>0&&n<int(sizeof(row))&&Serial.availableForWrite()>=n)Serial.write((uint8_t*)row,n);}
  delay(1);
}
