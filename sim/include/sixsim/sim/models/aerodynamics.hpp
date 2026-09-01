#pragma once

#include "sixsim/math/quaternion.hpp"
#include "sixsim/math/vector3.hpp"
#include "sixsim/sim/models/atmosphere.hpp"
#include "sixsim/sim/models/wind.hpp"
#include "sixsim/sim/sim_general.hpp"

#include "sim/models/dynamics/rigid_body.hpp"

#include <cmath>

namespace sixsim::sim {

struct AerodynamicState {
  math::Vector3 air_relative_velocity_body_mps{};
  double airspeed_mps{};
  double angle_of_attack_rad{};
  double sideslip_rad{};
  double dynamic_pressure_pa{};
  double mach{};
};

inline AerodynamicState compute_aerodynamic_state(
    const RigidBodyState& state,
    const AtmosphereState& atmosphere,
    const WindState& wind) {
  AerodynamicState aerodynamic_state{};

  const math::Vector3 vehicle_velocity_ned_mps =
      math::rotate(state.q_body2ned, state.velocity_body_mps);
  const math::Vector3 air_relative_velocity_ned_mps =
      vehicle_velocity_ned_mps - wind.wind_ned_mps;
  aerodynamic_state.air_relative_velocity_body_mps =
      math::rotate(math::conjugate(state.q_body2ned),
                   air_relative_velocity_ned_mps);

  const math::Vector3& velocity_body =
      aerodynamic_state.air_relative_velocity_body_mps;
  aerodynamic_state.airspeed_mps = math::norm(velocity_body);
  aerodynamic_state.dynamic_pressure_pa =
      0.5 * atmosphere.density_kg_per_m3 *
      aerodynamic_state.airspeed_mps * aerodynamic_state.airspeed_mps;

  if (aerodynamic_state.airspeed_mps > 0.0) {
    aerodynamic_state.angle_of_attack_rad =
        std::atan2(velocity_body.z, velocity_body.x);
    aerodynamic_state.sideslip_rad =
        std::asin(velocity_body.y / aerodynamic_state.airspeed_mps);
  }

  if (atmosphere.speed_of_sound_m_per_s > 0.0) {
    aerodynamic_state.mach =
        aerodynamic_state.airspeed_mps / atmosphere.speed_of_sound_m_per_s;
  }

  return aerodynamic_state;
}

class AerodynamicsModel {
 public:
  virtual ~AerodynamicsModel() = default;

  virtual ForceMoment evaluate(const RigidBodyState& state,
                               const AerodynamicState& aerodynamic_state,
                               const VehicleContext& vehicle) const = 0;
};

}  // namespace sixsim::sim
