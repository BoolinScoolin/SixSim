#pragma once

#include "sixsim/hal/timer.hpp"

#include <Arduino.h>

namespace sixsim::hal {

class Physical_dummy_uno_r3_Timer final : public timer {
 public:
  double read() override {
    return static_cast<double>(millis()) / 1000.0;
  }
};

}  // namespace sixsim::hal
