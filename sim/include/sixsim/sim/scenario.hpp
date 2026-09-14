#pragma once

#include "sixsim/sim/models/aerodynamics.hpp"
#include "sixsim/sim/models/atmosphere.hpp"
#include "sixsim/sim/models/gravity.hpp"
#include "sixsim/sim/models/propulsion.hpp"
#include "sixsim/sim/models/wind.hpp"
#include "sixsim/sim/sim_general.hpp"

#include "sim/models/dynamics/rigid_body.hpp"

#include <filesystem>
#include <memory>

namespace sixsim::sim {

struct Environment {
  double origin_altitude_msl_m{};
  std::unique_ptr<AtmosphereModel> atmosphere;
  std::unique_ptr<WindModel> wind;
  std::unique_ptr<GravityModel> gravity;
};

struct Vehicle {
  RigidBodyState state{};
  MassProperties mass_properties{};
  double unloaded_mass_kg{};
  ActuatorState actuator{};
  std::unique_ptr<AerodynamicsModel> aerodynamics;
  std::unique_ptr<PropulsionModel> propulsion;
};

struct Scenario {
  SimulationConfig simulation{};
  double logging_rate_hz{};
  std::filesystem::path source_scenario_path;
  std::filesystem::path default_run_directory;
  Environment environment;
  Vehicle vehicle;
};

}  // namespace sixsim::sim
