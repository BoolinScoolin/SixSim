#pragma once

#include "hal/physical/dummy_uno_r3/sensors.hpp"
#include "hal/physical/dummy_uno_r3/serial.hpp"
#include "sixsim/hal/serial.hpp"

namespace sixsim::hal {

class Physical_dummy_uno_r3_Devices {
 public:
  void begin() { serial_.begin(); }

  Physical_dummy_uno_r3_Sensors& sensors() { return sensors_; }
  const Physical_dummy_uno_r3_Sensors& sensors() const { return sensors_; }
  ::sixsim::hal::serial& serial() { return serial_; }

 private:
  Physical_dummy_uno_r3_Sensors sensors_;
  Physical_dummy_uno_r3_Serial serial_;
};

}  // namespace sixsim::hal
