#pragma once

#include "sixsim/sim/force_moment.hpp"
#include "sixsim/sim/mass_properties.hpp"
#include "sixsim/sim/rigid_body_derivative.hpp"
#include "sixsim/sim/rigid_body_state.hpp"

namespace sixsim::sim {

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

}  // namespace sixsim::sim
