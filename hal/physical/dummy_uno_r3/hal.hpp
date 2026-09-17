#pragma once

#include "hal/physical/dummy_uno_r3/devices.hpp"
#include "sixsim/hal/profiles/dummy_uno_r3.hpp"

#include <Arduino.h>
#include <stdint.h>

namespace sixsim::hal {

class PhysicalDummyUnoR3Hal {
 public:
  uint64_t cycle_ticks() const {
    return static_cast<uint64_t>(millis());
  }

  PhysicalDummyUnoR3Devices& devices() { return devices_; }
  const PhysicalDummyUnoR3Devices& devices() const { return devices_; }

 private:
  PhysicalDummyUnoR3Devices devices_;
};

using PhysicalDummyUnoR3Fcu = DummyUnoR3Fcu<PhysicalDummyUnoR3Hal>;

}  // namespace sixsim::hal
