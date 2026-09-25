#pragma once

#include "hal/physical/irec_2526_fcu/sensors.hpp"
#include "hal/physical/irec_2526_fcu/serial.hpp"
#include "sixsim/hal/serial.hpp"

namespace sixsim::hal {

class Physical_irec_2526_fcu_Devices {
 public:
  void begin() {
    serial_.begin();
    sensors_.begin();
  }

  Physical_irec_2526_fcu_Sensors& sensors() { return sensors_; }
  const Physical_irec_2526_fcu_Sensors& sensors() const { return sensors_; }
  ::sixsim::hal::serial& serial() { return serial_; }

 private:
  Physical_irec_2526_fcu_Sensors sensors_;
  Physical_irec_2526_fcu_Serial serial_;
};

}  // namespace sixsim::hal
