#pragma once

#include "sixsim/hal/serial.hpp"

#include <iostream>

namespace sixsim::hal {

class SitlSerial final : public Serial {
 public:
  void begin(std::uint32_t) override {}

  void print(const char* text) override { std::cout << text; }

  void print(double value) override { std::cout << value; }

  void println() override { std::cout << '\n'; }
};

}  // namespace sixsim::hal
