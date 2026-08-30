#pragma once

#include "sixsim/math/comparison.hpp"
#include "sixsim/sim/sim_general.hpp"

namespace sixsim::sim {

struct SimulationEventFlag {
  bool triggered{};
  double trigger_time_s{};
};

// Event-specific fields.
struct SimulationEvents {
  SimulationEventFlag stop_simulation{};
};

template <typename State>
void evaluate_simulation_events(const SimTime& time,
                                const State& state,
                                const SimulationConfig& config,
                                SimulationEvents& events) {
  (void)state;

  if (!events.stop_simulation.triggered &&
      math::compare_geq(time.simtime_s, config.stop_simulation_time_s)) {
    events.stop_simulation.triggered = true;
    events.stop_simulation.trigger_time_s = time.simtime_s;
  }
}

}  // namespace sixsim::sim
