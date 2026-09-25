#pragma once

#include "hal/physical/irec_2526_fcu/altimeter.hpp"
#include "hal/physical/irec_2526_fcu/timer.hpp"
#include "sixsim/hal/altimeter.hpp"
#include "sixsim/hal/timer.hpp"

namespace sixsim::hal {

class Physical_irec_2526_fcu_Sensors {
 public:
  void begin() { altimeter_.begin(); }

  ::sixsim::hal::altimeter& altimeter() { return altimeter_; }
  ::sixsim::hal::timer& timer() { return timer_; }

 private:
  Physical_irec_2526_fcu_Altimeter altimeter_;
  Physical_irec_2526_fcu_Timer timer_;
};

}  // namespace sixsim::hal
