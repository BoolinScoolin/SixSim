#pragma once

#include "sixsim/hal/serial.hpp"

#include <Arduino.h>

namespace sixsim::hal {

class Physical_irec_2526_fcu_Serial final : public serial {
 public:
  void begin() override {
    ::Serial.begin(config().baud_rate);
  }

  void print(const char* text) override { ::Serial.print(text); }

  void print(double value) override { ::Serial.print(value); }

  void println() override { ::Serial.println(); }
};

}  // namespace sixsim::hal
