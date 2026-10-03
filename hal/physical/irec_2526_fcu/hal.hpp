#pragma once

#include "hal/physical/irec_2526_fcu/devices.hpp"
#include "sixsim/hal/profiles/irec_2526_fcu.hpp"

#include <Arduino.h>
#include <stdint.h>

namespace sixsim::hal::irec_2526_fcu {

class PhysicalHal {
 public:
  uint64_t cycle_ticks() const {
    return static_cast<uint64_t>(millis());
  }

  Physical_irec_2526_fcu_Devices& devices() { return devices_; }
  const Physical_irec_2526_fcu_Devices& devices() const { return devices_; }

 private:
  Physical_irec_2526_fcu_Devices devices_;
};

using PhysicalFcu = Fcu<PhysicalHal>;

}  // namespace sixsim::hal::irec_2526_fcu
