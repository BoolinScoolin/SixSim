#pragma once

#include "hal/sitl/sensors.hpp"

namespace sixsim::hal {

class SitlHardware {
 public:
  SitlSensors& sensors() { return sensors_; }
  const SitlSensors& sensors() const { return sensors_; }

 private:
  SitlSensors sensors_;
};

}  // namespace sixsim::hal
