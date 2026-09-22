#pragma once

#include "hal/sitl/sensors.hpp"
#include "hal/sitl/serial.hpp"

namespace sixsim::hal {

class SitlDeviceRegistry {
 public:
  SitlSensors& sensors() { return sensors_; }
  const SitlSensors& sensors() const { return sensors_; }
  Sitl_serial& serial() { return serial_; }
  const Sitl_serial& serial() const { return serial_; }

 private:
  SitlSensors sensors_;
  Sitl_serial serial_;
};

}  // namespace sixsim::hal
