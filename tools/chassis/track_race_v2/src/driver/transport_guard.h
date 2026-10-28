#pragma once
#include <stdint.h>
#include <stddef.h>

// This checked transport is shared by the sketch and its fault-injection tests.
// A latch prevents upstream SDK code from overwriting an earlier I2C failure.
namespace checked_i2c {
enum class Kind : uint8_t { OK, TX_BUFFER, TX_END, SHORT_READ, READ_EMPTY, BUS_START };
struct Fault {
  Kind kind = Kind::OK;
  uint8_t address = 0, reg = 0, wireCode = 0;
  uint16_t expected = 0, actual = 0;
};
struct Latch {
  Fault first = {};
  constexpr bool failed() const { return first.kind != Kind::OK; }
  constexpr void clear() { first = {}; }
  constexpr void note(Fault f) { if (!failed() && f.kind != Kind::OK) first = f; }
};

template<class Bus>
constexpr bool write(Bus& bus, Latch& latch, uint8_t addr, uint8_t reg,
                     const uint8_t* data, uint32_t count) {
  if (latch.failed()) return false;
  bus.beginTransmission(addr);
  bool queued = bus.write(reg) == 1;
  for (uint32_t i=0;i<count;++i) if (bus.write(data[i]) != 1) queued=false;
  uint8_t rc=bus.endTransmission(); // Always release the Wire transaction.
  if (!queued) latch.note({Kind::TX_BUFFER,addr,reg,rc,0,0});
  else if (rc!=0) latch.note({Kind::TX_END,addr,reg,rc,0,0});
  return !latch.failed();
}

template<class Bus>
constexpr bool read(Bus& bus, Latch& latch, uint8_t addr, uint8_t reg,
                    uint8_t* data, uint32_t count) {
  for (uint32_t i=0;i<count;++i) data[i]=0; // Never decode uninitialised bytes.
  if (latch.failed()) return false;
  bus.beginTransmission(addr);
  const bool queued=bus.write(reg)==1;
  const uint8_t rc=bus.endTransmission();
  if (!queued) latch.note({Kind::TX_BUFFER,addr,reg,rc,0,0});
  else if (rc!=0) latch.note({Kind::TX_END,addr,reg,rc,0,0});
  if (latch.failed()) return false;
  const size_t got=bus.requestFrom(addr,static_cast<size_t>(count),true);
  if (got!=count) {
    latch.note({Kind::SHORT_READ,addr,reg,0,static_cast<uint16_t>(count),static_cast<uint16_t>(got)});
    return false;
  }
  for (uint32_t i=0;i<count;++i) {
    const int value=bus.read();
    if (value<0) {
      for (uint32_t j=0;j<count;++j) data[j]=0;
      latch.note({Kind::READ_EMPTY,addr,reg,0,static_cast<uint16_t>(count),static_cast<uint16_t>(i)});
      return false;
    }
    data[i]=static_cast<uint8_t>(value);
  }
  return true;
}
extern Latch latch;
}
