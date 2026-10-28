#include "../policy.h"
constexpr race::Range r(int mm,uint32_t at=1000){race::Range x;x.observe(mm,at,true);return x;}
constexpr race::Input input(int f,int l,int rr,uint32_t now=1100){race::Input x;x.now=now;x.s[0]=r(f,now-100);x.s[1]=r(l,now-100);x.s[2]=r(rr,now-100);return x;}
constexpr bool hardFloors(){
  race::Controller c;
  if(c.step(input(30,130,130)).issue!=race::Issue::FRONT_HARD)return false;
  if(c.step(input(1000,50,130)).issue!=race::Issue::SIDE_HARD)return false;
  if(c.step(input(1000,130,50)).issue!=race::Issue::SIDE_HARD)return false;
  if(c.step(input(75,130,130)).issue!=race::Issue::FRONT_RESERVE)return false;
  return c.step(input(1000,64,130)).issue==race::Issue::SIDE_RESERVE;
}
constexpr bool directionsAndOpenings(){
  race::Controller a,b,c,d,e;
  if(a.step(input(1000,130,130)).us!=race::CENTER)return false;
  if(b.step(input(1000,95,180)).us<=race::CENTER)return false;
  if(c.step(input(1000,180,95)).us>=race::CENTER)return false;
  if(d.step(input(1000,700,130)).us!=race::CENTER)return false;
  auto o=e.step(input(300,300,300));
  return o.issue==race::Issue::OK&&o.us>race::CENTER&&o.speed[1]<o.speed[0];
}
constexpr bool narrowAndReacquire(){
  race::Controller c;auto x=input(1000,90,90);auto o=c.step(x);
  if(!race::ready(x)||o.issue!=race::Issue::OK||o.us!=race::CENTER)return false;
  race::Range y=r(1000);y.observe(140,1180,true);
  if(y.closing!=0||y.filtered!=140)return false;
  y.observe(8190,1200,false);return !y.valid&&y.mm==140&&y.lastRaw==8190;
}
constexpr bool hairpinAndS(){
  race::Controller c;
  auto a=c.step(input(300,250,120));if(a.us>=race::CENTER)return false;
  auto open=input(900,130,130,1200);c.step(open);
  auto reverse=input(300,120,250,1300);
  auto b=c.step(reverse);return b.us>race::CENTER;
}
constexpr bool unknownAndGrace(){
  race::Controller c;auto x=input(1000,140,140);x.s[0].valid=false;
  auto o=c.step(x);if(o.issue!=race::Issue::OK||!o.degraded||o.speed[0]>.061f)return false;
  x.now=1321;x.s[1]=r(140,1321);x.s[2]=r(140,1321);
  if(c.step(x).issue!=race::Issue::FRONT_UNKNOWN)return false;
  x=input(1000,140,140);x.s[0]=race::Range{};
  if(c.step(x).issue!=race::Issue::FRONT_UNKNOWN)return false;
  x=input(100,140,140);x.s[0].valid=false;
  return c.step(x).issue==race::Issue::FRONT_UNKNOWN;
}
constexpr bool prediction(){race::Controller c;auto x=input(110,140,140);x.speed[0]=x.speed[1]=.25f;
  if(c.step(x).issue!=race::Issue::FRONT_RESERVE)return false;
  x=input(1000,100,140);x.s[1].closing=180;
  return c.step(x).issue==race::Issue::SIDE_RESERVE;
}
constexpr bool motorAndWheel(){
  auto l=race::motorPins(50,1,true,false),rr=race::motorPins(50,-1,true,false);
  auto off=race::motorPins(50,1,false,false),brake=race::motorPins(50,1,true,true);
  if(!(l.a==255&&l.b==205&&rr.a==205&&rr.b==255&&off.a==0&&off.b==0&&brake.a==255&&brake.b==255))return false;
  race::WheelLoop w;int max=0;
  for(int i=0;i<100;++i){int d=w.step(.20f,0,.02f);if(d>max)max=d;if(d<0||d>race::MAX_DUTY)return false;}
  return max>40&&w.step(.05f,.20f,.02f)==0;
}
constexpr bool singleRightStart(){
  race::Controller c;auto x=input(500,140,130);x.s[1]=race::Range{};
  if(race::ready(x)||c.step(x).issue==race::Issue::OK)return false;
  x.rightOnlyStart=true;if(!race::ready(x))return false;
  auto o=c.step(x);if(o.issue!=race::Issue::OK||o.speed[0]>.081f||o.us!=race::CENTER)return false;
  x.s[2]=r(95);o=c.step(x);if(o.issue!=race::Issue::OK||o.us>=race::CENTER||o.us<race::CENTER-281)return false;
  x.s[0]=race::Range{};if(race::ready(x)||c.step(x).issue!=race::Issue::FRONT_UNKNOWN)return false;
  x=input(500,140,130);x.rightOnlyStart=true;x.s[1]=race::Range{};x.now=1250;x.s[2]=r(130,1250);
  o=c.step(x);if(o.issue!=race::Issue::OK||o.speed[0]>.061f)return false;
  x.now=1321;x.s[2]=r(130,1321);if(c.step(x).issue!=race::Issue::FRONT_UNKNOWN)return false;
  x=input(500,50,130);x.rightOnlyStart=true;
  return !race::ready(x)&&c.step(x).issue==race::Issue::SIDE_HARD;
}
constexpr bool islandTurnHold(){
  race::Controller c;
  for(int i=0;i<30;++i){
    int f=450-i*10;
    auto o=c.step(input(f,663,497,1000+i*100));
    if(o.issue!=race::Issue::OK)return false;
  }
  return c.turnCommit()==1;
}
constexpr bool islandTurnRelease(){
  race::Controller c;
  for(int i=0;i<10;++i)c.step(input(450-i*10,663,497,1000+i*100));
  auto o=c.step(input(600,663,497,3000));
  return c.turnCommit()==0&&o.issue==race::Issue::OK;
}
constexpr bool islandNoReversal(){
  race::Controller c;
  for(int i=0;i<20;++i){
    int f=480-i*12,l=632-i*8,r=502+i*8;
    auto o=c.step(input(f,l,r,1000+i*100));
    if(o.issue!=race::Issue::OK)return false;
    if(o.us>race::CENTER)return false;
  }
  return c.turnCommit()==1;
}
constexpr bool forcedLeftIsland(){
  race::Controller c;
  for(int i=0;i<20;++i){
    auto x=input(480-i*12,370,716,1000+i*100);
    x.firstIslandMode=true;x.firstIslandTurn=true;
    auto o=c.step(x);
    if(o.issue!=race::Issue::OK)return false;
    if(o.us>race::CENTER)return false;
  }
  return c.turnCommit()==1;
}
constexpr bool forcedLeftAtFarFront(){
  race::Controller c;auto x=input(744,300,114);
  x.firstIslandMode=true;x.firstIslandTurn=true;
  auto o=c.step(x);
  return o.issue==race::Issue::OK&&o.us<race::CENTER&&c.turnCommit()==1;
}
constexpr bool forcedLeftSuppressesOutsideReserve(){
  race::Controller c;auto x=input(744,300,60);
  x.firstIslandMode=true;x.firstIslandTurn=true;
  auto o=c.step(x);
  return o.issue==race::Issue::OK&&o.us<race::CENTER&&c.turnCommit()==1;
}
constexpr bool firstIslandLeftRepulsion(){
  race::Controller c;auto x=input(200,149,994);
  x.firstIslandMode=true;x.firstIslandTurn=true;
  auto o=c.step(x);
  return o.issue==race::Issue::OK&&o.us>race::CENTER;
}
constexpr bool firstIslandLeftRepulsionEarly(){
  race::Controller c;auto x=input(200,192,930);
  x.firstIslandMode=true;x.firstIslandTurn=true;
  auto o=c.step(x);
  return o.issue==race::Issue::OK&&o.us>race::CENTER;
}
constexpr bool firstIslandArcRight(){
  race::Controller c;auto x=input(400,354,775);
  x.firstIslandMode=true;x.firstIslandArc=true;
  auto o=c.step(x);
  return o.issue==race::Issue::OK&&o.us>race::CENTER;
}
constexpr bool firstIslandArcFollow(){
  race::Controller c;auto x=input(400,300,250);
  x.firstIslandMode=true;x.firstIslandArc=true;
  auto o=c.step(x);
  return o.issue==race::Issue::OK&&o.us>=race::CENTER-20&&o.us<=race::CENTER+20;
}
constexpr bool firstIslandArcTightRight(){
  race::Controller c;auto x=input(250,889,374);
  x.firstIslandMode=true;x.firstIslandArc=true;
  auto o=c.step(x);
  return o.issue==race::Issue::OK&&o.us>1900;
}
constexpr bool firstIslandArcNearIsland(){
  race::Controller c;auto x=input(250,300,180);
  x.firstIslandMode=true;x.firstIslandArc=true;
  auto o=c.step(x);
  return o.issue==race::Issue::OK&&o.us<race::CENTER;
}
constexpr bool firstIslandArcFrontEscape(){
  race::Controller c;auto x=input(200,1028,177);
  x.firstIslandMode=true;x.firstIslandArc=true;
  auto o=c.step(x);
  return o.issue==race::Issue::OK&&o.us<race::CENTER;
}
constexpr bool firstIslandArcEscapeHold(){
  race::Controller c;auto a=input(200,1028,177,1000);
  a.firstIslandMode=true;a.firstIslandArc=true;c.step(a);
  auto b=input(400,900,500,1500);b.firstIslandMode=true;b.firstIslandArc=true;
  auto o=c.step(b);
  return o.issue==race::Issue::OK&&o.us<race::CENTER;
}
constexpr bool firstIslandArcEscapeRelease(){
  race::Controller c;auto a=input(200,1028,177,1000);
  a.firstIslandMode=true;a.firstIslandArc=true;c.step(a);
  auto b=input(400,900,500,2600);b.firstIslandMode=true;b.firstIslandArc=true;
  auto o=c.step(b);
  return o.issue==race::Issue::OK&&o.us>race::CENTER;
}
static_assert(hardFloors(),"Raw requested floors and earlier reserve stops");
static_assert(directionsAndOpenings(),"Correct steering and outer-wall island entry");
static_assert(narrowAndReacquire(),"Narrow passage stays centred; reappearing wall is not a false closing-speed spike");
static_assert(hairpinAndS(),"Hairpin chooses left and next bend can choose right");
static_assert(unknownAndGrace(),"Missing is never infinite: bounded distant-history grace only");
static_assert(prediction(),"Approaching wall brakes before raw hard limit");
static_assert(motorAndWheel(),"Forward motor polarity, off/brake, bounded PI and immediate reduction");
static_assert(singleRightStart(),"Declared open-left start follows right wall slowly; front/right and hard floors remain mandatory");
static_assert(race::fresh(r(500,0xfffffff0),0x60,180),"Sample age handles clock rollover");
static_assert(race::elapsed(0x2bf10,0xfffffff0,180000),"Run deadline handles rollover");
static_assert(!race::ready(input(120,130,130))&&race::ready(input(500,130,130)),"Startup requires observed clearance");
static_assert(islandTurnHold(),"Island approach keeps the turn committed past 1.2s");
static_assert(islandTurnRelease(),"Turn commitment releases once the front opens past 550mm");
static_assert(islandNoReversal(),"Island turn is latched left despite side distances swapping");
static_assert(forcedLeftIsland(),"First island forces left even when the right side is wider");
static_assert(forcedLeftAtFarFront(),"Forced left continues when the front is far but the right wall is close");
static_assert(forcedLeftSuppressesOutsideReserve(),"Forced left suppresses the outside early-brake but keeps SIDE_HARD");
static_assert(firstIslandLeftRepulsion(),"First island steers away when the left island wall gets close");
static_assert(firstIslandLeftRepulsionEarly(),"First island starts steering right before the left wall is at the brake limit");
static_assert(firstIslandArcRight(),"First island right arc turns right when the island is far on the right");
static_assert(firstIslandArcFollow(),"First island right arc holds ~220mm from the island wall");
static_assert(firstIslandArcTightRight(),"First island arc turns tighter right near the right-hand bend");
static_assert(firstIslandArcNearIsland(),"First island arc does not turn into a close island wall");
static_assert(firstIslandArcFrontEscape(),"First island arc turns left away from a close front/right wall");
static_assert(firstIslandArcEscapeHold(),"Front-wall left escape is held for at least 1.5s");
static_assert(firstIslandArcEscapeRelease(),"Front-wall left escape releases after 1.5s clear");
