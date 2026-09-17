#pragma once

#include "hal/physical/dummy_uno_r3/sensors.hpp"
#include "hal/physical/dummy_uno_r3/serial.hpp"
#include "sixsim/hal/serial.hpp"

namespace sixsim::hal {

class PhysicalDummyUnoR3Devices {
 public:
  void begin(uint32_t serial_baud_rate) { serial_.begin(serial_baud_rate); }

  PhysicalDummyUnoR3Sensors& sensors() { return sensors_; }
  const PhysicalDummyUnoR3Sensors& sensors() const { return sensors_; }
  Serial& serial() { return serial_; }

 private:
  PhysicalDummyUnoR3Sensors sensors_;
  PhysicalDummyUnoR3Serial serial_;
};

}  // namespace sixsim::hal
