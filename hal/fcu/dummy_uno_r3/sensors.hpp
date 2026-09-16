#pragma once

#include "hal/fcu/dummy_uno_r3/timer.hpp"
#include "sixsim/hal/timer.hpp"

namespace sixsim::hal {

class DummyUnoR3Sensors {
 public:
  Timer& timer() { return timer_; }

 private:
  DummyUnoR3Timer timer_;
};

}  // namespace sixsim::hal
