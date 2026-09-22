#pragma once

#include "hal/physical/dummy_uno_r3/sensors.hpp"
#include "hal/physical/dummy_uno_r3/serial.hpp"
#include "sixsim/hal/serial.hpp"

namespace sixsim::hal {

class PhysicalDummyUnoR3Devices {
 public:
  void begin() { serial_.begin(); }

  PhysicalDummyUnoR3Sensors& sensors() { return sensors_; }
  const PhysicalDummyUnoR3Sensors& sensors() const { return sensors_; }
  ::sixsim::hal::serial& serial() { return serial_; }

 private:
  PhysicalDummyUnoR3Sensors sensors_;
  PhysicalDummyUnoR3Serial serial_;
};

}  // namespace sixsim::hal
