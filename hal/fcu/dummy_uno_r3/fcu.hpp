#pragma once

#include "hal/fcu/dummy_uno_r3/hardware.hpp"
#include "sixsim/flight/flight_computer.hpp"

namespace sixsim::hal {

class DummyUnoR3Fcu final : public flight::FlightComputer {
 public:
  explicit DummyUnoR3Fcu(flight::FlightTimingConfig timing)
      : FlightComputer(timing) {}

  DummyUnoR3Hardware& hardware() { return hardware_; }
  const DummyUnoR3Hardware& hardware() const { return hardware_; }

 protected:
  uint64_t read_ticks() const override {
    return static_cast<uint64_t>(millis());
  }

 private:
  DummyUnoR3Hardware hardware_;
};

}  // namespace sixsim::hal
