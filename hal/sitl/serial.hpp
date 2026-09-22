#pragma once

#include "sixsim/hal/serial.hpp"

#include <iostream>

namespace sixsim::hal {

class Sitl_serial final : public serial {
 public:
  void begin() override {}

  void print(const char* text) override { std::cout << text; }

  void print(double value) override { std::cout << value; }

  void println() override { std::cout << '\n'; }
};

}  // namespace sixsim::hal
