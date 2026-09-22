#pragma once

#include "hal/physical/dummy_uno_r3/timer.hpp"
#include "sixsim/hal/timer.hpp"

namespace sixsim::hal {

class Physical_dummy_uno_r3_Sensors {
 public:
  ::sixsim::hal::timer& timer() { return timer_; }

 private:
  Physical_dummy_uno_r3_Timer timer_;
};

}  // namespace sixsim::hal
