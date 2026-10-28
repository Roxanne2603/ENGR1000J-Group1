#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <string.h>
namespace history {
struct Record {
  uint32_t seq=0,ms=0;uint8_t reset=0,origin=0,valid=0,opening=0,flags=0;
  int32_t raw[3]={0,0,0};uint32_t age[3]={0,0,0};
  int16_t servo_us=0;int8_t turn=0;
  char reason[40]={0};
};
struct Blob {uint32_t magic=0x52323148,version=3,seq=0;Record item[8];};
class Store {
  Blob data;
public:
  void load(){Preferences p;if(!p.begin("race_history",true))return;
    if(p.getBytesLength("events")==sizeof(Blob)){Blob b;if(p.getBytes("events",&b,sizeof(b))==sizeof(b)&&b.magic==data.magic&&b.version==3)data=b;}p.end();}
  bool append(Record r){Blob b=data;r.seq=++b.seq;b.item[(r.seq-1)%8]=r;
    Preferences p;if(!p.begin("race_history",false))return false;
    bool ok=p.putBytes("events",&b,sizeof(b))==sizeof(b);p.end();if(ok)data=b;return ok;}
  void print()const {
    Serial.printf("HISTORY_BEGIN,latest=%lu\n",(unsigned long)data.seq);
    uint32_t first=data.seq>7?data.seq-7:1;
    for(uint32_t i=first;i<=data.seq;++i){const auto& r=data.item[(i-1)%8];
      Serial.printf("HISTORY,seq=%lu,ms=%lu,reset=%u,origin=%u,reason=%s,F=%ld,L=%ld,R=%ld,valid=%u,age=%lu/%lu/%lu,open_start=%u,servo=%d,turn=%d,flags=%u\n",
        (unsigned long)r.seq,(unsigned long)r.ms,r.reset,r.origin,r.reason,(long)r.raw[0],(long)r.raw[1],(long)r.raw[2],r.valid,
        (unsigned long)r.age[0],(unsigned long)r.age[1],(unsigned long)r.age[2],r.opening,r.servo_us,r.turn,r.flags);}
    Serial.println("HISTORY_END (power source unknown; user sequence supplies context)");
  }
};
}
