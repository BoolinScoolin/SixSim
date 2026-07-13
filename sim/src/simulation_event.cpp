#include "sixsim/sim/simulation_event.hpp"

#include "sixsim/math/comparison.hpp"

namespace sixsim::sim {

void evaluate_simulation_events(const SimTime& time,
                                const RigidBodyState& state,
                                const SimulationConfig& config,
                                SimulationEvents& events) {
  (void)state;

  // Event-specific evaluation logic goes here.
  if (!events.stop_simulation.triggered &&
      math::compare_geq(time.simtime_s, config.stop_simulation_time_s)) {
    events.stop_simulation.triggered = true;
    events.stop_simulation.trigger_time_s = time.simtime_s;
  }
}

}  // namespace sixsim::sim
