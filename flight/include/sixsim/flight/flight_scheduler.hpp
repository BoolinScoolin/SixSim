#pragma once

#include <assert.h>
#include <stdint.h>

namespace sixsim::flight {

struct CycleUpdate {
  bool due;
  uint64_t skipped_cycles;
};

class FlightScheduler {
 public:
  // ticks_per_cycle must be positive.
  explicit FlightScheduler(uint64_t ticks_per_cycle)
      : ticks_per_cycle_(ticks_per_cycle) {
    assert(ticks_per_cycle_ > 0);
  }

  // Tick snapshots must be nondecreasing within a run.
  CycleUpdate update(uint64_t tick_count) {
    const auto cycle = tick_count / ticks_per_cycle_;

    if (!started_) {
      started_ = true;
      last_cycle_ = cycle;
      return {true, cycle};
    }

    if (cycle == last_cycle_) {
      return {false, 0};
    }

    const auto skipped_cycles = cycle - last_cycle_ - 1;
    last_cycle_ = cycle;
    return {true, skipped_cycles};
  }

 private:
  uint64_t ticks_per_cycle_;
  uint64_t last_cycle_{0};
  bool started_{false};
};

}  // namespace sixsim::flight
