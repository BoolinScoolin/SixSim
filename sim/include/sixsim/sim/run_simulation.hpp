#pragma once

#include "sixsim/sim/sim_general.hpp"
#include "sixsim/sim/simulation_event.hpp"

#include <filesystem>

namespace sixsim::sim {

template <typename State, typename StepFunction>
State run_loop(const SimulationConfig& config,
               StepFunction step,
               State state,
               SimulationEvents& events,
               SimTime& time) {
  while (!events.stop_simulation.triggered) {
    evaluate_simulation_events(time, state, config, events);
    if (events.stop_simulation.triggered) {
      break;
    }

    state = step(time, state, config.dt_s);
    time.simtime_s += config.dt_s;
  }

  return state;
}

void run_simulation(const std::filesystem::path& output_directory = {});

}  // namespace sixsim::sim
