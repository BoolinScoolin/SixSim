#pragma once

namespace sixsim::hal {

class timer {
 public:
  struct config_type {
    double multiplier{1.0};
  };

  virtual ~timer() = default;

  config_type& config() { return config_; }
  const config_type& config() const { return config_; }

  virtual double read() = 0;

 private:
  config_type config_;
};

}  // namespace sixsim::hal
