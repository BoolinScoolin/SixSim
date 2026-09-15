#pragma once

#include "sixsim/hal/timer.hpp"

#include <Arduino.h>

namespace sixsim::hal {

class UnoR3Timer final : public Timer {
 public:
  double read() override {
    return static_cast<double>(millis()) / 1000.0;
  }
};

}  // namespace sixsim::hal
