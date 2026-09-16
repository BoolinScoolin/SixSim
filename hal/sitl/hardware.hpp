#pragma once

#include "hal/sitl/sensors.hpp"
#include "hal/sitl/serial.hpp"

namespace sixsim::hal {

class SitlHardware {
 public:
  SitlSensors& sensors() { return sensors_; }
  const SitlSensors& sensors() const { return sensors_; }
  SitlSerial& serial() { return serial_; }
  const SitlSerial& serial() const { return serial_; }

 private:
  SitlSensors sensors_;
  SitlSerial serial_;
};

}  // namespace sixsim::hal
