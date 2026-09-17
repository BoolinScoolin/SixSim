#pragma once

#include "sixsim/flight/run_cycle.hpp"
#include "sixsim/sim/models/aerodynamics.hpp"
#include "sixsim/sim/models/atmosphere.hpp"
#include "sixsim/sim/models/gravity.hpp"
#include "sixsim/sim/models/propulsion.hpp"
#include "sixsim/sim/models/wind.hpp"
#include "sixsim/sim/sim_general.hpp"

#include "sim/models/dynamics/rigid_body.hpp"

#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace sixsim::sim {

struct Environment {
  double origin_altitude_msl_m{};
  std::unique_ptr<AtmosphereModel> atmosphere;
  std::unique_ptr<WindModel> wind;
  std::unique_ptr<GravityModel> gravity;
};

struct Scenario {
  SimulationConfig simulation{};
  double logging_rate_hz{};
  std::filesystem::path source_scenario_path;
  std::filesystem::path default_run_directory;
  Environment environment;
};

template <typename Fcu>
void update_fcu(Fcu& fcu,
                const SensorTruthInputs& inputs,
                double dt_s)
{
  fcu.hal().update_sensors(inputs);

  if (fcu.check_cycle().due)
  {
    flight::run_cycle(fcu);
  }

  const auto ticks =
      static_cast<uint64_t>(dt_s * fcu.base_tick_hz());
  fcu.hal().advance_ticks(ticks);
}

template <typename Fcus>
struct Vehicle {
  explicit Vehicle(Fcus configured_fcus)
      : fcus(std::move(configured_fcus)) {}

  std::string name{"vehicle_0"};
  bool dynamics_enabled{true};
  Fcus fcus;
  RigidBodyState state{};
  MassProperties mass_properties{};
  double unloaded_mass_kg{};
  ActuatorState actuator{};
  std::unique_ptr<AerodynamicsModel> aerodynamics;
  std::unique_ptr<PropulsionModel> propulsion;

  void update_fcus(const SimTime& time, const Scenario& scenario) {
    const SensorTruthInputs inputs{time, state, scenario.environment};
    std::apply(
        [&](auto&... fcu) {
          (update_fcu(fcu, inputs, scenario.simulation.dt_s), ...);
        },
        fcus);
  }
};

template <typename VehicleType>
inline const VehicleType* find_vehicle(
    const std::vector<VehicleType>& vehicles,
    std::string_view name) {
  for (const VehicleType& vehicle : vehicles) {
    if (vehicle.name == name) {
      return &vehicle;
    }
  }
  return nullptr;
}

}  // namespace sixsim::sim
