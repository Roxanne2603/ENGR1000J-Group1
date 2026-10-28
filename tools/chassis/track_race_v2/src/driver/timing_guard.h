#pragma once
#include <stdint.h>
namespace timing_guard {
constexpr bool expired(uint32_t now,uint32_t start,uint32_t limit) {
  return uint32_t(now-start)>=limit;
}
}
