#pragma once
// Checked single-emitter scheduler derived from installed_three_v2; all LONG mode; budgets F50/L33/R33ms.
namespace tof {
constexpr uint8_t SDA_PIN=21, SCL_PIN=22;
constexpr uint8_t XSH[3]={4,13,17}, ADDR[3]={0x30,0x31,0x32};
constexpr uint8_t MOTOR[4]={25,26,32,33}, SERVO=14;
constexpr uint32_t SAMPLE_TIMEOUT_MS=100, BETWEEN_MS=0;
const char* const NAME[3]={"FRONT","LEFT","RIGHT"};
Adafruit_VL53L0X sensor[3];
bool available[3]={false,false,false}, fatalInit=false, measuring=false, pendingClear=false;
uint8_t current=0,frame=0;
constexpr uint8_t ORDER[4]={0,1,0,2};
void advance(){frame=(frame+1)%4;current=ORDER[frame];}
uint32_t startedAt=0,nextAt=0,windowAt=0;
char input[24]; uint8_t inputLen=0; bool inputOverflow=false;

struct Counters {
  uint32_t attempts=0, completed=0, valid=0, invalid=0, errors=0;
  uint32_t lastValid=0, maxGap=0;
  uint64_t sum=0;
  uint16_t minMm=65535,maxMm=0,lastMm=0;
  bool everValid=false;
};
Counters counters[3];

const char* faultName(checked_i2c::Kind k) {
  switch(k) {
    case checked_i2c::Kind::TX_BUFFER:return "TX_BUFFER";
    case checked_i2c::Kind::TX_END:return "TX_END";
    case checked_i2c::Kind::SHORT_READ:return "SHORT_READ";
    case checked_i2c::Kind::READ_EMPTY:return "READ_EMPTY";
    case checked_i2c::Kind::BUS_START:return "BUS_START";
    default:return "NONE";
  }
}
void reportI2C(const char* context) {
  auto f=checked_i2c::latch.first;
  if (f.kind!=checked_i2c::Kind::OK)
    Serial.printf("I2C_FAULT,%lu,%s,kind=%s,addr=0x%02X,reg=0x%02X,wire=%u,expected=%u,actual=%u\n",
      (unsigned long)millis(),context,faultName(f.kind),unsigned(f.address),unsigned(f.reg),
      unsigned(f.wireCode),unsigned(f.expected),unsigned(f.actual));
}
void shutdown(uint8_t i) { digitalWrite(XSH[i],LOW); pinMode(XSH[i],OUTPUT); available[i]=false; }
void shutdownAll() { for(uint8_t i=0;i<3;++i)shutdown(i); measuring=false; }
void abortInit(const char* reason) {
  Serial.printf("INIT_ABORT,%s\n",reason); shutdownAll(); fatalInit=true;
  Serial.println("All XSH held LOW. Save output, disconnect USB and power before checking wiring.");
}

enum class Probe { PRESENT, NO_ACK_OR_FAIL, BUS_ERROR };
Probe probe(uint8_t addr, const char* context) {
  Wire.beginTransmission(addr);
  uint8_t rc=Wire.endTransmission();
  if (rc==0) return Probe::PRESENT;
  // ESP32 Wire maps ESP_FAIL and ESP_ERR_NOT_FOUND to 2; it is not proof of absence.
  if (rc==2) return Probe::NO_ACK_OR_FAIL;
  Serial.printf("PROBE_ERROR,%lu,%s,addr=0x%02X,wire=%u\n",
    (unsigned long)millis(),context,unsigned(addr),unsigned(rc));
  return Probe::BUS_ERROR;
}
bool probeAbsent(uint8_t addr, const char* context) {
  Probe p=probe(addr,context);
  if (p==Probe::BUS_ERROR) { abortInit("PROBE_BUS_ERROR"); return false; }
  if (p==Probe::PRESENT) { abortInit("UNEXPECTED_ADDRESS_RESPONDS"); return false; }
  return true;
}
bool verifyAvailable() {
  for(uint8_t j=0;j<3;++j) if (available[j]) {
    Probe p=probe(ADDR[j],"VERIFY_PREVIOUS");
    if (p!=Probe::PRESENT) { abortInit(p==Probe::BUS_ERROR?"PROBE_BUS_ERROR":"PREVIOUS_ADDRESS_LOST"); return false; }
  }
  return true;
}
void resetCounters() {
  windowAt=millis();
  for(uint8_t i=0;i<3;++i)counters[i]=Counters{};
  pendingClear=false;
  Serial.printf("COUNTERS_CLEARED,%lu\n",(unsigned long)windowAt);
}
void initSensors() {
  shutdownAll(); delay(20);
  for(uint8_t address: {uint8_t(0x29),uint8_t(0x30),uint8_t(0x31),uint8_t(0x32)})
    if(!probeAbsent(address,"ALL_XSH_LOW")) return;
  for(uint8_t i=0;i<3;++i) {
    if(!verifyAvailable() || !probeAbsent(0x29,"BEFORE_RELEASE")) return;
    pinMode(XSH[i],INPUT); delay(15); // Release to module pull-up; never drive high.
    Probe p=probe(0x29,"AFTER_RELEASE");
    if(p==Probe::BUS_ERROR) { abortInit("PROBE_BUS_ERROR"); return; }
    bool seen=p==Probe::PRESENT, ok=false;
    checked_i2c::latch.clear();
    if(seen) {
      const bool begun=sensor[i].begin(ADDR[i],false,&Wire,Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
      if(checked_i2c::latch.failed()) reportI2C("BEGIN");
      ok=begun && !checked_i2c::latch.failed();
      if(ok) {
        checked_i2c::latch.clear();
        const bool budgetOK=sensor[i].setMeasurementTimingBudgetMicroSeconds(budgetFor(i));
        if(checked_i2c::latch.failed()) reportI2C("SET_BUDGET");
        ok=budgetOK && !checked_i2c::latch.failed();
      }
    }
    if(!ok) {
      Serial.printf("INIT_FAIL,%s,xsh=%u,probe=%s,api=%d\n",NAME[i],unsigned(XSH[i]),
                    seen?"ACK":"NO_ACK_OR_BUS_FAIL",int(sensor[i].Status));
      shutdown(i); delay(15);
      if(!probeAbsent(0x29,"AFTER_FAILED_SHUTDOWN") ||
         !probeAbsent(ADDR[i],"AFTER_FAILED_SHUTDOWN") || !verifyAvailable()) return;
      continue;
    }
    Probe assigned=probe(ADDR[i],"CHECK_ASSIGNED");
    if(assigned!=Probe::PRESENT) { abortInit(assigned==Probe::BUS_ERROR?"PROBE_BUS_ERROR":"ASSIGNED_ADDRESS_LOST"); return; }
    if(!probeAbsent(0x29,"AFTER_ASSIGNMENT") || !verifyAvailable()) return;
    available[i]=true;
    Serial.printf("INIT_OK,%s,xsh=%u,address=0x%02X,budget_requested_us=%lu\n",NAME[i],unsigned(XSH[i]),
                  unsigned(ADDR[i]),(unsigned long)budgetFor(i));
  }
  if(!verifyAvailable())return;
  Serial.printf("INIT_SUMMARY,FRONT=%s,LEFT=%s,RIGHT=%s\n",
    available[0]?"OK":"FAIL",available[1]?"OK":"FAIL",available[2]?"OK":"FAIL");
  Serial.println("DATA,end_ms,sensor,start_ms,elapsed_ms,raw_mm,api_error,range_status,valid");
  Serial.println("RangeStatus/API/I2C validity retained; no range certification claim.");
  resetCounters();
}
void reportStats() {
  const uint32_t now=millis(), duration=uint32_t(now-windowAt);
  Serial.printf("STATS_WINDOW,start=%lu,end=%lu,elapsed_ms=%lu,clear_pending=%d\n",
    (unsigned long)windowAt,(unsigned long)now,(unsigned long)duration,int(pendingClear));
  for(uint8_t i=0;i<3;++i) {
    const Counters& c=counters[i];
    uint32_t tail=uint32_t(now-(c.everValid?c.lastValid:windowAt));
    uint32_t worst=max(c.maxGap,tail);
    Serial.printf("STATS,%s,enabled=%d,attempts=%lu,completed=%lu,valid=%lu,invalid=%lu,errors=%lu,",
      NAME[i],int(available[i]),(unsigned long)c.attempts,(unsigned long)c.completed,
      (unsigned long)c.valid,(unsigned long)c.invalid,(unsigned long)c.errors);
    if(c.everValid) Serial.printf("min_mm=%u,max_mm=%u,last_mm=%u,mean_mm=%.1f,",
      unsigned(c.minMm),unsigned(c.maxMm),unsigned(c.lastMm),double(c.sum)/double(c.valid));
    else Serial.print("min_mm=NA,max_mm=NA,last_mm=NA,mean_mm=NA,");
    Serial.printf("max_no_valid_ms=%lu\n",(unsigned long)worst);
  }
  Serial.println("STATS_END (if a measurement is in progress, attempts may exceed completed+errors by 1)");
}
void failCurrent(const char* reason,int api) {
  const uint32_t now=millis();
  ++counters[current].errors;
  onSensorError(current);
  reportI2C(reason);
  Serial.printf("SENSOR_OFF,%lu,%s,%s,api=%d,elapsed_ms=%lu\n",
    (unsigned long)now,NAME[current],reason,api,(unsigned long)uint32_t(now-startedAt));
  shutdown(current); measuring=false; advance(); nextAt=now+BETWEEN_MS;
}
bool expired() { return timing_guard::expired(millis(),startedAt,SAMPLE_TIMEOUT_MS); }
void pollSensors() {
  if(pendingClear && !measuring) resetCounters();
  if(fatalInit || (!available[0]&&!available[1]&&!available[2]))return;
  if(!measuring) {
    if(int32_t(millis()-nextAt)<0)return;
    for(uint8_t n=0;n<3&&!available[current];++n)advance();
    if(!available[current])return;
    startedAt=millis(); ++counters[current].attempts;
    checked_i2c::latch.clear();
    const bool started=sensor[current].startRange();
    if(!started || checked_i2c::latch.failed()) { failCurrent("START_FAILED",int(sensor[current].Status)); return; }
    if(expired()) { failCurrent("SAMPLE_TIMEOUT",0); return; }
    measuring=true; return;
  }
  checked_i2c::latch.clear();
  const bool done=sensor[current].isRangeComplete();
  if(sensor[current].Status!=VL53L0X_ERROR_NONE || checked_i2c::latch.failed()) {
    failCurrent("POLL_FAILED",int(sensor[current].Status)); return;
  }
  if(expired()) { failCurrent("SAMPLE_TIMEOUT",0); return; }
  if(!done)return;
  VL53L0X_RangingMeasurementData_t sample={};
  checked_i2c::latch.clear();
  VL53L0X_Error error=sensor[current].getRangingMeasurement(&sample,false);
  if(error==VL53L0X_ERROR_NONE && !checked_i2c::latch.failed())
    error=sensor[current].clearInterruptMask(false);
  if(error!=VL53L0X_ERROR_NONE || checked_i2c::latch.failed()) { failCurrent("READ_OR_CLEAR_FAILED",int(error)); return; }
  if(expired()) { failCurrent("SAMPLE_TIMEOUT",0); return; }
  const uint32_t now=millis();
  const bool valid=sample.RangeStatus==0 && sample.RangeMilliMeter>0 && sample.RangeMilliMeter<8190;
  onRange(current,sample.RangeMilliMeter,startedAt,valid);
  Counters& c=counters[current]; ++c.completed;
  if(valid) {
    uint32_t gap=uint32_t(now-(c.everValid?c.lastValid:windowAt)); c.maxGap=max(c.maxGap,gap);
    c.lastValid=now; c.everValid=true; ++c.valid; c.lastMm=sample.RangeMilliMeter;
    c.minMm=min(c.minMm,c.lastMm); c.maxMm=max(c.maxMm,c.lastMm); c.sum+=c.lastMm;
  } else ++c.invalid;
  measuring=false; advance(); nextAt=millis()+BETWEEN_MS;
}
void serialInput() {
  for(int n=0;n<48 && Serial.available();++n) {
    char c=char(Serial.read());
    if(c=='\r'||c=='\n') {
      if(inputOverflow) Serial.println("Command too long; discarded.");
      else if(inputLen) {
        input[inputLen]=0;
        if(!strcmp(input,"STATS"))reportStats();
        else if(!strcmp(input,"CLEAR")) { pendingClear=true; Serial.println("CLEAR requested; waits for current measurement to finish."); }
        else Serial.println("Commands: CLEAR, STATS (newline). They never move motors.");
      }
      inputLen=0;inputOverflow=false;
    } else if(c!=' '&&c!='\t') {
      if(inputLen<sizeof(input)-1)input[inputLen++]=char(toupper((unsigned char)c));
      else inputOverflow=true;
    }
  }
}
}
