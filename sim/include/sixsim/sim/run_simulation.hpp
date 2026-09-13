#pragma once

#include "sixsim/sim/scenario.hpp"
#include "sixsim/sim/sim_general.hpp"
#include "sixsim/sim/simulation_event.hpp"

#include <filesystem>

namespace sixsim::sim {

class Simulation {
 public:
  explicit Simulation(std::filesystem::path output_directory = {});

  void run();

 private:
  RigidBodyState update_vehicle_state(const SimTime& time,
                                      const RigidBodyState& state,
                                      double dt_s,
                                      const AuxiliaryContext& auxiliary);

  std::filesystem::path output_directory_;
  Scenario scenario_;
  SimTime time_{};
  SimulationEvents events_{};
};

}  // namespace sixsim::sim
