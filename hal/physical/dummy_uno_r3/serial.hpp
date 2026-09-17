#pragma once

#include "sixsim/hal/serial.hpp"

#include <Arduino.h>

namespace sixsim::hal {

class PhysicalDummyUnoR3Serial final : public Serial {
 public:
  void begin(uint32_t baud_rate) override {
    ::Serial.begin(baud_rate);
  }

  void print(const char* text) override { ::Serial.print(text); }

  void print(double value) override { ::Serial.print(value); }

  void println() override { ::Serial.println(); }
};

}  // namespace sixsim::hal
