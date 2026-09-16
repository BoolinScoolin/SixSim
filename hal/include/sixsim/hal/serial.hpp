#pragma once

#include <stdint.h>

namespace sixsim::hal {

class Serial {
 public:
  virtual ~Serial() = default;

  virtual void begin(uint32_t baud_rate) = 0;
  virtual void print(const char* text) = 0;
  virtual void print(double value) = 0;
  virtual void println() = 0;
};

}  // namespace sixsim::hal
