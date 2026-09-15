#pragma once

namespace sixsim::hal {

class Timer {
 public:
  virtual ~Timer() = default;

  virtual double read() = 0;
};

}  // namespace sixsim::hal
