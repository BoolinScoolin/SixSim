#pragma once

#include "sixsim/math/vector3.hpp"
#include "sixsim/sim/models/actuator.hpp"

namespace sixsim::sim {

class AerodynamicsModel;
class AtmosphereModel;
class GravityModel;
class PropulsionModel;
class WindModel;

struct SimTime {
  double simtime_s{};
};

struct SimulationConfig {
  double dt_s{};
  double stop_simulation_time_s{};
};

struct MassProperties {
  double mass_kg{};
  math::Vector3 inertia_body_kgm2{};
};

struct VehicleContext {
  MassProperties& mass_properties;
  double unloaded_mass_kg{};
  const ActuatorState& actuator;
};

struct AuxiliaryContext {
  VehicleContext& vehicle;
  const AtmosphereModel& atmosphere_model;
  const WindModel& wind_model;
  const GravityModel& gravity_model;
  const AerodynamicsModel& aerodynamics_model;
  const PropulsionModel& propulsion_model;
};

struct ForceMoment {
  math::Vector3 force_body_n{};
  math::Vector3 moment_body_nm{};
};

inline ForceMoment combine_force_moment(const ForceMoment& a,
                                        const ForceMoment& b) {
  ForceMoment combined{};
  combined.force_body_n = a.force_body_n + b.force_body_n;
  combined.moment_body_nm = a.moment_body_nm + b.moment_body_nm;
  return combined;
}

inline ForceMoment combine_force_moment(const ForceMoment& gravity,
                                        const ForceMoment& aerodynamics,
                                        const ForceMoment& propulsion) {
  return combine_force_moment(
      combine_force_moment(gravity, aerodynamics), propulsion);
}

}  // namespace sixsim::sim
