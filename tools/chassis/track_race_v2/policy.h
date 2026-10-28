#pragma once
#include <stdint.h>

enum class Stage:uint8_t { WAIT, CENTER, COUNTDOWN, RUN, BRAKE, HOME, DONE, FAULT };
namespace race {
constexpr int CENTER=1375,LEFT=575,RIGHT=2175,MAX_DUTY=70;
constexpr int FRONT_HARD=30,SIDE_HARD=50,FRONT_RESERVE=80,SIDE_RESERVE=65,SIDE_AIM=130;
constexpr uint32_t MAX_RUN_MS=180000,LEASE_MS=80,FRONT_AGE=220,SIDE_AGE=320;
constexpr uint32_t FRONT_GRACE=320,SIDE_GRACE=500;
constexpr float CRUISE=.20f,OVERSPEED=.35f;
constexpr float clip(float x,float a,float b){return x<a?a:x>b?b:x;}
constexpr float absf(float x){return x<0?-x:x;}
constexpr float maxf(float a,float b){return a>b?a:b;}
constexpr float minf(float a,float b){return a<b?a:b;}
constexpr bool elapsed(uint32_t now,uint32_t at,uint32_t limit){return uint32_t(now-at)>=limit;}
struct Range {
  int mm=0,lastRaw=0;float filtered=0,closing=0;
  uint32_t started=0,goodAt=0;bool valid=false,seen=false;
  constexpr void observe(int raw,uint32_t at,bool good) {
    lastRaw=raw;started=at;valid=good&&raw>0&&raw<8190;
    if(!valid)return;
    bool openingChange=seen&&((mm>400&&raw<=400)||(mm<=400&&raw>400));
    if(seen && !openingChange && uint32_t(at-goodAt)>0 && uint32_t(at-goodAt)<1000) {
      float rate=(mm-raw)*1000.0f/uint32_t(at-goodAt);
      closing=.55f*closing+.45f*clip(rate,-600,600);
      filtered=.65f*filtered+.35f*raw;
    } else {filtered=float(raw);closing=0;}
    mm=raw;goodAt=at;seen=true;
  }
};
constexpr bool fresh(const Range& r,uint32_t now,uint32_t age) {
  return r.seen&&r.valid&&uint32_t(now-r.goodAt)<=age;
}
enum class Issue:uint8_t {
  OK,FRONT_HARD,SIDE_HARD,FRONT_RESERVE,SIDE_RESERVE,
  FRONT_UNKNOWN,LEFT_UNKNOWN,RIGHT_UNKNOWN,NO_TURN_SPACE,BAD_INPUT
};
struct Input { Range s[3];uint32_t now=0;float speed[2]={0,0};bool rightOnlyStart=false;bool firstIslandMode=false;bool firstIslandTurn=false;bool firstIslandArc=false; };
struct Output { int us=CENTER;float speed[2]={0,0};Issue issue=Issue::OK;bool degraded=false; };
constexpr bool ready(const Input& in) {
  bool left=fresh(in.s[1],in.now,SIDE_AGE);
  bool allowedLeft=left?in.s[1].mm>80:(in.rightOnlyStart&&(!in.s[1].seen||in.s[1].mm>100));
  int frontRequired=in.rightOnlyStart&&!left?250:150;
  return fresh(in.s[0],in.now,FRONT_AGE)&&fresh(in.s[2],in.now,SIDE_AGE)
    &&allowedLeft&&in.s[0].mm>=frontRequired&&in.s[2].mm>80;
}
constexpr float predicted(const Range& r,uint32_t now,float forward=0) {
  float rate=maxf(maxf(0,r.closing),forward*1000);
  return r.mm-rate*(uint32_t(now-r.goodAt)*.001f+.18f);
}
class Controller {
  float prevError=0,derivative=0;
  uint32_t sideAt=0,leftMark=0,rightMark=0,turnAt=0;
  int turnSign=0,prevFront=0;
  bool escaping=false;uint32_t escapeAt=0;
public:
  constexpr void reset(){prevError=derivative=0;sideAt=leftMark=rightMark=turnAt=0;turnSign=0;prevFront=0;escaping=false;escapeAt=0;}
  constexpr int turnCommit() const {return turnSign;}
  constexpr Output step(const Input& in) {
    Output o;const Range& f=in.s[0];const Range& l=in.s[1];const Range& r=in.s[2];
    if(!(in.speed[0]>=-1&&in.speed[0]<=2&&in.speed[1]>=-1&&in.speed[1]<=2)){o.issue=Issue::BAD_INPUT;return o;}
    if(f.seen&&f.mm<=FRONT_HARD){o.issue=Issue::FRONT_HARD;return o;}
    if((l.seen&&l.mm<=SIDE_HARD)||(r.seen&&r.mm<=SIDE_HARD)){o.issue=Issue::SIDE_HARD;return o;}
    bool ff=fresh(f,in.now,FRONT_AGE),lf=fresh(l,in.now,SIDE_AGE),rf=fresh(r,in.now,SIDE_AGE);
    bool frontOpening=ff&&prevFront>0&&f.mm>prevFront+5;
    prevFront=ff?f.mm:0;
    if(in.firstIslandArc&&ff&&f.mm<300&&r.mm<300){escaping=true;escapeAt=in.now;}
    if(escaping&&elapsed(in.now,escapeAt,1500))escaping=false;
    if(!in.firstIslandArc&&!in.firstIslandTurn)escaping=false;
    // User confirmed this specific start has a right wall and an open left side.
    bool frontGrace=!ff&&f.seen&&uint32_t(in.now-f.goodAt)<=FRONT_GRACE&&f.mm>250&&rf&&r.mm>120
      &&((lf&&l.mm>120)||(in.rightOnlyStart&&!lf));
    bool openStart=in.rightOnlyStart&&!lf&&(ff||frontGrace)&&rf&&f.mm>=250&&r.mm>80;
    // Invalid never means clear. Only a bounded coast-down on previously distant samples is allowed.
    if(!ff&&!frontGrace){
      o.issue=Issue::FRONT_UNKNOWN;return o;
    }
    if(!lf && !openStart && !(l.seen&&uint32_t(in.now-l.goodAt)<=SIDE_GRACE&&l.mm>120&&rf&&ff&&r.mm>120&&f.mm>250)){
      o.issue=Issue::LEFT_UNKNOWN;return o;
    }
    if(!rf && !(r.seen&&uint32_t(in.now-r.goodAt)<=SIDE_GRACE&&r.mm>120&&lf&&ff&&l.mm>120&&f.mm>250)){
      o.issue=Issue::RIGHT_UNKNOWN;return o;
    }
    o.degraded=!ff||!lf||!rf;
    float measured=maxf(0,maxf(in.speed[0],in.speed[1]));
    if(ff&&(f.mm<=FRONT_RESERVE||predicted(f,in.now,measured)<=50)){o.issue=Issue::FRONT_RESERVE;return o;}
    bool leftReserve=lf&&(l.mm<=SIDE_RESERVE||predicted(l,in.now)<=60);
    bool rightReserve=rf&&(r.mm<=SIDE_RESERVE||predicted(r,in.now)<=60);
    if(in.firstIslandTurn)rightReserve=false; // Forced left: right is the outside wall; SIDE_HARD still applies.
    if(in.firstIslandArc)leftReserve=false;   // Right arc: left is the outside wall; SIDE_HARD still applies.
    if(leftReserve||rightReserve){o.issue=Issue::SIDE_RESERVE;return o;}
    float lv=l.filtered,rv=r.filtered,error=0;
    // First island: go straight until the entrance, then commit left; no right-wall pre-turn.
    if(in.firstIslandMode&&f.mm>480)error=0;
    else if(rf&&rv<=350)error=(lf&&lv<=350)?(lv-rv)*.5f:SIDE_AIM-rv;
    else if(lf&&lv<=350)error=lv-SIDE_AIM;
    else if(rf&&f.mm>550)error=clip(SIDE_AIM-rv,-75,75);
    if(l.goodAt!=leftMark||r.goodAt!=rightMark) {
      if(sideAt&&uint32_t(in.now-sideAt)>0&&uint32_t(in.now-sideAt)<600)
        derivative=.5f*derivative+.5f*clip((error-prevError)*1000/uint32_t(in.now-sideAt),-500,500);
      prevError=error;sideAt=in.now;leftMark=l.goodAt;rightMark=r.goodAt;
    }
    float turn=clip((2.8f*error+.22f*derivative)/800,-1,1); // +left => lower pulse.
    if(ff&&f.mm>550&&!in.firstIslandTurn)turnSign=0;
    if(turnSign && ((turnSign>0&&(!lf||l.mm<90))||(turnSign<0&&(!rf||r.mm<90))))turnSign=0;
    if(turnSign && elapsed(in.now,turnAt,650)) {
      if(frontOpening&&!in.firstIslandTurn)turnSign=0;
      else if(elapsed(in.now,turnAt,4000))turnSign=0;
    }
    if(openStart)turnSign=0;
    if(in.firstIslandTurn&&ff&&lf&&l.mm>80&&!openStart) {
      if(turnSign!=1){turnSign=1;turnAt=in.now;}
    } else if(ff&&f.mm<480&&!turnSign&&!openStart&&!frontOpening) {
      bool canL=lf&&l.mm>110,canR=rf&&r.mm>110;
      if(!canL&&!canR&&f.mm<150){o.issue=Issue::NO_TURN_SPACE;return o;}
      if(canL||canR) {
        if(canL&&!canR)turnSign=1;
        else if(canR&&!canL)turnSign=-1;
        else if(l.mm>r.mm+30)turnSign=1;
        else if(r.mm>l.mm+30)turnSign=-1;
        else turnSign=r.mm>150?-1:1; // Open island entrance: take the right branch.
        turnAt=in.now;
      }
    }
    if(ff&&turnSign&&(f.mm<500||in.firstIslandTurn)) {
      float w=clip((480.0f-f.mm)/200,0,1);
      w=maxf(w,0.85f);
      turn=(1-w)*turn+w*turnSign;
    }
    // Right-hand arc around the first island: keep ~250mm from the right wall; avoid cutting the inner wall.
    if(in.firstIslandArc&&ff) {
      if(escaping) {
        turn=0.6f;
      } else {
        float cmd=rf?clip((250.0f-r.mm)/200.0f,-0.6f,0.6f):-0.2f;
        if(f.mm<280.0f&&r.mm>300.0f)cmd=minf(cmd,-0.7f);
        if(f.mm<180.0f&&r.mm>300.0f)cmd=-0.9f;
        turn=cmd;
      }
    }
    // Side avoidance overrides a front-turn commitment, including in S bends.
    if(lf&&rf&&l.mm<100&&r.mm<100)turn=clip((l.mm-r.mm)/35.0f,-1,1);
    else {
      if(lf&&l.mm<100)turn=minf(turn,-clip((100.0f-l.mm)/35,0,1));
      if(rf&&r.mm<100)turn=maxf(turn,clip((100.0f-r.mm)/35,0,1));
    }
    if(!lf&&!openStart)turn=minf(turn,0);if(!rf)turn=maxf(turn,0);
    // First island: if the island wall on the left gets close, stop adding left and steer away.
    if(in.firstIslandTurn&&lf&&l.mm<260){
      turn=minf(turn,0);
      turn=minf(turn,-clip((260.0f-l.mm)/100,0,1));
    }
    if(openStart)turn=clip(turn,-.35f,.35f); // Bounded corrections toward the declared open side.
    turn=clip(turn,-1,1);o.us=int(CENTER-800*turn);
    float cap=CRUISE,side=minf(l.mm,r.mm);
    if(ff){if(f.mm<180)cap=.07f;else if(f.mm<300)cap=.08f;else if(f.mm<450)cap=.12f;}
    if(side<100)cap=minf(cap,.08f);else if(side<130)cap=minf(cap,.12f);
    if(absf(turn)>.5f)cap=minf(cap,.10f);else if(absf(turn)>.2f)cap=minf(cap,.14f);
    if(openStart)cap=minf(cap,ff?.08f:.06f);
    else if(o.degraded)cap=minf(cap,.06f);
    o.speed[0]=cap*(turn>0?1-.30f*turn:1);
    o.speed[1]=cap*(turn<0?1+.30f*turn:1);
    return o;
  }
};
// Explicit engineering feedforward/PI initial values, not measured loaded calibration.
class WheelLoop {
  float integral=0,target=0;
public:
  constexpr void reset(){integral=target=0;}
  constexpr int step(float wanted,float measured,float dt) {
    target=minf(wanted,target+.25f*dt); // Required reductions are immediate.
    if(target<=.005f){integral=0;return 0;}
    float e=target-measured;
    float proposed=clip(integral+e*dt,-.30f,.30f);
    float effort=15+200*target+90*e+50*proposed;
    if(effort>0&&effort<MAX_DUTY)integral=proposed;
    if(measured>target+.04f)return 0; // Active brake on overspeed relative to demand.
    return int(clip(effort,0,MAX_DUTY));
  }
};
constexpr int approach(int current,int wanted){return current+int(clip(float(wanted-current),-60,60));}
struct Pins {int a,b;};
constexpr Pins motorPins(int duty,int sign,bool enabled,bool brake) {
  if(brake)return {255,255};
  if(!enabled)return {0,0};
  int d=int(clip(float(duty),0,MAX_DUTY));
  return sign>0?Pins{255,255-d}:Pins{255-d,255};
}
}
