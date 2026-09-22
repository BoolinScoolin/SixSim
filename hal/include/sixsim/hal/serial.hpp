#pragma once

#include <stdint.h>

namespace sixsim::hal {

class serial {
 public:
  struct config_type {
    uint32_t baud_rate{};
  };

  virtual ~serial() = default;

  config_type& config() { return config_; }
  const config_type& config() const { return config_; }

  virtual void begin() = 0;
  virtual void print(const char* text) = 0;
  virtual void print(double value) = 0;
  virtual void println() = 0;

 private:
  config_type config_;
};

}  // namespace sixsim::hal
