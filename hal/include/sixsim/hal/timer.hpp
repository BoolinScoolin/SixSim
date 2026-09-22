#pragma once

namespace sixsim::hal {

class timer {
 public:
  virtual ~timer() = default;

  virtual double read() = 0;
};

}  // namespace sixsim::hal
