#pragma once

#include "sixsim/flight/flight_computer.hpp"

namespace sixsim::hal::dummy_uno_r3 {

template <typename Hal>
class Fcu final : public flight::FlightComputer {
 public:
  explicit Fcu(flight::FlightTimingConfig timing)
      : FlightComputer(timing) {}

  Fcu(flight::FlightTimingConfig timing, Hal hal)
      : FlightComputer(timing),
        hal_(static_cast<Hal&&>(hal)) {}

  Hal& hal() { return hal_; }
  const Hal& hal() const { return hal_; }

  auto& devices() { return hal_.devices(); }
  const auto& devices() const { return hal_.devices(); }

 protected:
  uint64_t read_ticks() const override { return hal_.cycle_ticks(); }

 private:
  Hal hal_;
};

}  // namespace sixsim::hal::dummy_uno_r3
