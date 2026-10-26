#include <cassert>
#include <cstdio>
#include "controller.h"
int main() {
  assert(!racer::calibrated());
  racer::Controller controller;
  racer::Input in;
  assert(controller.step(in).stop);
  std::puts("PASS: delivered production configuration refuses autonomous arming until measured commissioning.");
}
