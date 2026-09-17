#pragma once

#include "scenario_config.hpp"
#include "sixsim/sim/scenario.hpp"
#include "sixsim/sim/sim_general.hpp"
#include "sixsim/sim/simulation_event.hpp"

#include <filesystem>
#include <vector>

namespace sixsim::sim {

class Simulation {
 public:
  explicit Simulation(std::filesystem::path output_directory = {});

 void run();

 private:
  void update_vehicle_state(ConfiguredVehicle& vehicle);

  std::filesystem::path output_directory_;
  Scenario scenario_;
  std::vector<ConfiguredVehicle> vehicles_;
  SimTime time_{};
  SimulationEvents events_{};
};

}  // namespace sixsim::sim
