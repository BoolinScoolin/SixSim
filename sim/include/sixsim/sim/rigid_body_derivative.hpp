#pragma once

#include "sixsim/math/quaternion.hpp"
#include "sixsim/math/vector3.hpp"
#include "sixsim/sim/force_moment.hpp"
#include "sixsim/sim/mass_properties.hpp"
#include "sixsim/sim/rigid_body_state.hpp"

namespace sixsim::sim {

struct RigidBodyDerivative {
  math::Vector3 position_ned_derivative_mps{};
  math::Vector3 velocity_body_derivative_mps2{};
  math::Quaternion q_body2ned_derivative{0.0, 0.0, 0.0, 0.0};
  math::Vector3 omega_body_derivative_rps2{};
};

inline RigidBodyDerivative combine_derivatives(
    const RigidBodyDerivative& kinematic,
    const RigidBodyDerivative& dynamic) {
  RigidBodyDerivative derivative{};
  derivative.position_ned_derivative_mps =
      kinematic.position_ned_derivative_mps +
      dynamic.position_ned_derivative_mps;
  derivative.velocity_body_derivative_mps2 =
      kinematic.velocity_body_derivative_mps2 +
      dynamic.velocity_body_derivative_mps2;
  derivative.q_body2ned_derivative =
      kinematic.q_body2ned_derivative + dynamic.q_body2ned_derivative;
  derivative.omega_body_derivative_rps2 =
      kinematic.omega_body_derivative_rps2 +
      dynamic.omega_body_derivative_rps2;
  return derivative;
}

inline RigidBodyDerivative kinematic_derivative(const RigidBodyState& state) {
  RigidBodyDerivative derivative{};
  derivative.position_ned_derivative_mps =
      math::rotate(state.q_body2ned, state.velocity_body_mps);
  derivative.q_body2ned_derivative =
      math::derivative_from_body_rate(state.q_body2ned, state.omega_body_rps);
  return derivative;
}

inline math::Vector3 compute_accel_body(const ForceMoment& force_moment,
                                        const MassProperties& mass_properties) {
  return force_moment.force_body_n / mass_properties.mass_kg;
}

inline math::Vector3 compute_omega_dot_body(
    const ForceMoment& force_moment,
    const MassProperties& mass_properties,
    const math::Vector3& omega_body_rps) {
  const math::Vector3 angular_momentum_body{
      mass_properties.inertia_body_kgm2.x * omega_body_rps.x,
      mass_properties.inertia_body_kgm2.y * omega_body_rps.y,
      mass_properties.inertia_body_kgm2.z * omega_body_rps.z,
  };
  const math::Vector3 net_moment_body =
      force_moment.moment_body_nm -
      math::cross(omega_body_rps, angular_momentum_body);

  return {
      net_moment_body.x / mass_properties.inertia_body_kgm2.x,
      net_moment_body.y / mass_properties.inertia_body_kgm2.y,
      net_moment_body.z / mass_properties.inertia_body_kgm2.z,
  };
}

inline RigidBodyDerivative dynamic_derivative(
    const RigidBodyState& state,
    const ForceMoment& force_moment,
    const MassProperties& mass_properties) {
  RigidBodyDerivative derivative{};
  derivative.velocity_body_derivative_mps2 =
      compute_accel_body(force_moment, mass_properties);
  derivative.omega_body_derivative_rps2 =
      compute_omega_dot_body(force_moment, mass_properties, state.omega_body_rps);
  return derivative;
}

inline RigidBodyDerivative rigid_body_derivative(
    const RigidBodyState& state,
    const ForceMoment& force_moment,
    const MassProperties& mass_properties) {
  return combine_derivatives(
      kinematic_derivative(state),
      dynamic_derivative(state, force_moment, mass_properties));
}

}  // namespace sixsim::sim
