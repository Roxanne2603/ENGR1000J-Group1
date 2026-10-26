// Compile a byte-identical controller.h with SYNTHETIC fixture_config.h.
// Values below are testing fixtures and must never be copied into the car.
#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include "controller.h"

racer::Input input(float f=1500,float l=100,float r=100) {
  racer::Input in; in.now=10000;
  in.s[0]={f,9990,true};in.s[1]={l,9980,true};in.s[2]={r,9970,true};
  return in;
}
int main() {
  assert(racer::calibrated());
  int checks=0;
  for(float a:{0.15f,0.4f,1.5f}) for(float t:{0.0f,0.1f,0.5f})
    for(float d:{0.001f,0.1f,0.5f,2.0f}) {
      float v=racer::stopCap(d,a,t);
      assert(v>=0 && fabsf(v*t+v*v/(2*a)-d)<1e-4f); ++checks;
    }
  assert(racer::stopCap(0,1,0.1f)==0);
  {
    racer::Controller c;auto o=c.step(input());
    assert(!o.stop && o.speed>0 && o.speed<=cfg::ACCEL_MPS2*0.02f+1e-6f);
  }
  {
    racer::Controller c;assert(c.step(input(1500,140,60)).steer>0);
    c.reset();assert(c.step(input(1500,60,140)).steer<0);
  }
  {
    racer::Controller c;auto in=input();in.s[0].valid=false;
    assert(c.step(in).stop);in=input();in.s[1].stamp=0;assert(c.step(in).stop);
    in=input();in.s[2].mm=NAN;assert(c.step(in).stop);
    in=input();in.s[0].mm=2500;assert(c.step(in).stop);
    in=input();in.speed[0]=NAN;assert(c.step(in).stop);
  }
  {
    racer::Controller c;assert(c.step(input(50)).stop);
    assert(c.step(input(1500,20,150)).stop);
    auto in=input(300);in.speed[0]=in.speed[1]=0.8f;
    assert(c.step(in).stop && strcmp(c.step(in).stop,"BRAKE_ENVELOPE")==0);
  }
  {
    racer::Controller c;auto in=input();
    for(int n=0;n<300;++n) c.step(in);
    auto o=c.step(in);assert(o.speed>0.19f && o.speed<=cfg::INITIAL_MAX_MPS);
    in=input(1500,46,154);o=c.step(in);
    assert(!o.stop && o.speed<0.03f); // Required decrease is not slew-limited.
    assert(o.speed<=o.cap+1e-6f);     // No unsafe minimum-speed floor.
  }
  {
    racer::Controller c;auto in=input(250,250,100);
    for(int n=0;n<300;++n) c.step(in);
    auto o=c.step(in);assert(!o.stop && o.steer>0);
    float k=tanf(o.steer)/(cfg::WHEELBASE_MM*0.001f);
    assert(o.speed*o.speed*fabsf(k)<=cfg::LATERAL_MPS2+1e-6f);
    assert(o.speed*(1+fabsf(k)*cfg::REAR_TRACK_MM*0.0005f)<=cfg::MAX_WHEEL_MPS+1e-6f);
  }
  {
    racer::Sample s={100,0xfffffff0u,true};
    assert(racer::fresh(s,0x20u,50));assert(!racer::fresh(s,0x20u,40));
  }
  std::printf("PASS: %d stopping-envelope combinations; steering, stale/invalid/NaN, clearances, overspeed envelope, acceleration/deceleration, curvature, wheel cap, millis wrap.\n",checks);
}
