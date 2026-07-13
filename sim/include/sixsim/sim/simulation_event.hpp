#pragma once

#include "sixsim/sim/rigid_body_state.hpp"
#include "sixsim/sim/simulation_config.hpp"
#include "sixsim/sim/sim_time.hpp"

namespace sixsim::sim {

struct SimulationEventFlag {
  bool triggered{};
  double trigger_time_s{};
};

// Event-specific fields.
struct SimulationEvents {
  SimulationEventFlag stop_simulation{};
};

void evaluate_simulation_events(const SimTime& time,
                                const RigidBodyState& state,
                                const SimulationConfig& config,
                                SimulationEvents& events);

}  // namespace sixsim::sim
