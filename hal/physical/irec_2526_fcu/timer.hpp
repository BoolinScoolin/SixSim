#pragma once

#include "sixsim/hal/timer.hpp"

#include <Arduino.h>

namespace sixsim::hal {

class Physical_irec_2526_fcu_Timer final : public timer {
 public:
  double read() override {
    return static_cast<double>(millis()) / 1000.0;
  }
};

}  // namespace sixsim::hal
