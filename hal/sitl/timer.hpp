#pragma once

#include "hal/sitl/sensors.hpp"
#include "sixsim/hal/timer.hpp"

#include <utility>

namespace sixsim::hal {

class SitlTimer : public Timer, public SitlSensor {
 public:
  explicit SitlTimer(std::string name) : SitlSensor(std::move(name)) {}

  void update(const sim::SensorTruthInputs& inputs) override {
    time_s_ = inputs.time.simtime_s;
  }

  double read() override { return time_s_; }

 private:
  double time_s_{};
};

}  // namespace sixsim::hal
