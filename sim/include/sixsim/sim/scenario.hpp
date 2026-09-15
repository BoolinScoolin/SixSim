#pragma once

#include "sixsim/sim/models/aerodynamics.hpp"
#include "sixsim/sim/models/atmosphere.hpp"
#include "sixsim/sim/models/gravity.hpp"
#include "sixsim/sim/models/propulsion.hpp"
#include "sixsim/sim/models/wind.hpp"
#include "sixsim/sim/sim_general.hpp"

#include "hal/sitl/fcu.hpp"
#include "sim/models/dynamics/rigid_body.hpp"

#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace sixsim::sim {

struct Environment {
  double origin_altitude_msl_m{};
  std::unique_ptr<AtmosphereModel> atmosphere;
  std::unique_ptr<WindModel> wind;
  std::unique_ptr<GravityModel> gravity;
};

struct Vehicle {
  std::string name{"vehicle_0"};
  bool dynamics_enabled{true};
  std::vector<hal::SitlFcu> fcus;
  RigidBodyState state{};
  MassProperties mass_properties{};
  double unloaded_mass_kg{};
  ActuatorState actuator{};
  std::unique_ptr<AerodynamicsModel> aerodynamics;
  std::unique_ptr<PropulsionModel> propulsion;

  void update_fcus(const SimTime& time, const Environment& environment) {
    const SensorTruthInputs inputs{time, state, environment};
    for (hal::SitlFcu& fcu : fcus) {
      fcu.update_sensors(inputs);
    }
  }
};

inline const Vehicle* find_vehicle(const std::vector<Vehicle>& vehicles,
                                   std::string_view name) {
  for (const Vehicle& vehicle : vehicles) {
    if (vehicle.name == name) {
      return &vehicle;
    }
  }
  return nullptr;
}

struct Scenario {
  SimulationConfig simulation{};
  double logging_rate_hz{};
  std::filesystem::path source_scenario_path;
  std::filesystem::path default_run_directory;
  Environment environment;
};

}  // namespace sixsim::sim
