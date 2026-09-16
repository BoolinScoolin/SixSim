#pragma once

#include "sixsim/flight/flight_scheduler.hpp"

#include <assert.h>
#include <stdint.h>

namespace sixsim::flight {

struct FlightTimingConfig {
  uint64_t base_tick_hz;
  uint64_t cycle_rate_hz;
};

class FlightComputer {
 public:
  explicit FlightComputer(FlightTimingConfig timing)
      : base_tick_hz_(timing.base_tick_hz),
        scheduler_(ticks_per_cycle(timing)) {}

  uint64_t base_tick_hz() const { return base_tick_hz_; }

  CycleUpdate check_cycle() { return scheduler_.update(read_ticks()); }

 protected:
  virtual uint64_t read_ticks() const = 0;

 private:
  static uint64_t ticks_per_cycle(FlightTimingConfig timing) {
    assert(timing.base_tick_hz > 0);
    assert(timing.cycle_rate_hz > 0);
    assert(timing.base_tick_hz % timing.cycle_rate_hz == 0);
    return timing.base_tick_hz / timing.cycle_rate_hz;
  }

  uint64_t base_tick_hz_;
  FlightScheduler scheduler_;
};

}  // namespace sixsim::flight
