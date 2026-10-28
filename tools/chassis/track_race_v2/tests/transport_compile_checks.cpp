// Fault injection against the production checked_i2c templates; compile-time only.
#include "transport_guard.h"
#include "timing_guard.h"
struct FakeWire {
  int writes=0, begins=0, readPos=0;
  int failWriteAt=0, endCode=0, received=2, failReadAt=-1;
  constexpr void beginTransmission(uint8_t) { ++begins; }
  constexpr size_t write(uint8_t) { ++writes; return writes==failWriteAt ? 0u : 1u; }
  constexpr uint8_t endTransmission() { return uint8_t(endCode); }
  constexpr size_t requestFrom(uint8_t,size_t,bool) { return size_t(received); }
  constexpr int read() { int i=readPos++; return i==failReadAt ? -1 : 40+i; }
};
constexpr bool testSuccess() {
  FakeWire w; checked_i2c::Latch l; uint8_t src[2]={1,2},out[2]={0,0};
  if(!checked_i2c::write(w,l,0x30,0x12,src,2))return false;
  if(!checked_i2c::read(w,l,0x30,0x14,out,2))return false;
  return !l.failed() && out[0]==40 && out[1]==41 && w.begins==2;
}
constexpr bool testNackAndSticky() {
  FakeWire w; w.endCode=2; checked_i2c::Latch l; uint8_t src[1]={1};
  if(checked_i2c::write(w,l,0x30,0x12,src,1))return false;
  if(l.first.kind!=checked_i2c::Kind::TX_END || l.first.wireCode!=2)return false;
  w.endCode=0;
  if(checked_i2c::write(w,l,0x30,0x13,src,1) || w.begins!=1)return false;
  l.clear(); return checked_i2c::write(w,l,0x30,0x13,src,1) && !l.failed();
}
constexpr bool testTimeout() {
  FakeWire w; w.endCode=5; checked_i2c::Latch l; uint8_t src[1]={1};
  return !checked_i2c::write(w,l,0x31,0x14,src,1) &&
         l.first.kind==checked_i2c::Kind::TX_END && l.first.wireCode==5;
}
constexpr bool testBufferFailure() {
  FakeWire w; w.failWriteAt=2; checked_i2c::Latch l; uint8_t src[1]={1};
  return !checked_i2c::write(w,l,0x30,0x12,src,1) && l.first.kind==checked_i2c::Kind::TX_BUFFER;
}
constexpr bool testRegisterFailure() {
  FakeWire w; w.endCode=2; checked_i2c::Latch l; uint8_t out[2]={3,4};
  return !checked_i2c::read(w,l,0x30,0x14,out,2) && l.first.kind==checked_i2c::Kind::TX_END && out[0]==0 && out[1]==0;
}
constexpr bool testShortRead() {
  FakeWire w; w.received=1; checked_i2c::Latch l; uint8_t out[2]={3,4};
  return !checked_i2c::read(w,l,0x30,0x14,out,2) && l.first.kind==checked_i2c::Kind::SHORT_READ &&
         l.first.expected==2 && l.first.actual==1 && out[0]==0 && out[1]==0;
}
constexpr bool testUnexpectedEmpty() {
  FakeWire w; w.failReadAt=1; checked_i2c::Latch l; uint8_t out[2]={3,4};
  return !checked_i2c::read(w,l,0x30,0x14,out,2) && l.first.kind==checked_i2c::Kind::READ_EMPTY &&
         l.first.expected==2 && l.first.actual==1 && out[0]==0 && out[1]==0;
}
static_assert(testSuccess(),"Complete transactions return bytes");
static_assert(testNackAndSticky(),"Wire error cannot be overwritten by a later upstream SDK success");
static_assert(testTimeout(),"Wire timeout is not treated as success");
static_assert(testBufferFailure(),"Wire write-buffer failure is caught");
static_assert(testRegisterFailure(),"Read register transmission failure is caught");
static_assert(testShortRead(),"Short read is invalid and destination is cleared");
static_assert(testUnexpectedEmpty(),"Unexpected empty read invalidates all returned bytes");
static_assert(!timing_guard::expired(0x00000067u,0xfffffff0u,120),"119ms across rollover is not expired");
static_assert(timing_guard::expired(0x00000068u,0xfffffff0u,120),"120ms across rollover is expired");
