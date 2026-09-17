#pragma once

#include "hal/physical/dummy_uno_r3/timer.hpp"
#include "sixsim/hal/timer.hpp"

namespace sixsim::hal {

class PhysicalDummyUnoR3Sensors {
 public:
  Timer& timer() { return timer_; }

 private:
  PhysicalDummyUnoR3Timer timer_;
};

}  // namespace sixsim::hal
