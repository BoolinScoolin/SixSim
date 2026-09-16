#pragma once

#include "hal/fcu/dummy_uno_r3/serial.hpp"
#include "hal/fcu/dummy_uno_r3/sensors.hpp"
#include "sixsim/hal/serial.hpp"

namespace sixsim::hal {

class DummyUnoR3Hardware {
 public:
  void begin() { serial_.begin(115200); }

  DummyUnoR3Sensors& sensors() { return sensors_; }
  const DummyUnoR3Sensors& sensors() const { return sensors_; }
  Serial& serial() { return serial_; }

 private:
  DummyUnoR3Sensors sensors_;
  DummyUnoR3Serial serial_;
};

}  // namespace sixsim::hal
