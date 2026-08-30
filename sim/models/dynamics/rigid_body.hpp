#pragma once

#include "sixsim/math/quaternion.hpp"
#include "sixsim/math/vector3.hpp"
#include "sixsim/sim/models/gravity.hpp"
#include "sixsim/sim/sim_general.hpp"

namespace sixsim::sim {

struct RigidBodyState {
  math::Vector3 position_ned_m{};
  math::Vector3 velocity_body_mps{};
  math::Quaternion q_body2ned{};
  math::Vector3 omega_body_rps{};
};

struct RigidBodyDerivative {
  math::Vector3 position_ned_derivative_mps{};
  math::Vector3 velocity_body_derivative_mps2{};
  math::Quaternion q_body2ned_derivative{0.0, 0.0, 0.0, 0.0};
  math::Vector3 omega_body_derivative_rps2{};
};

inline RigidBodyState normalize_rigid_body_state(RigidBodyState state) {
  state.q_body2ned = math::positive_scalar(math::normalized(state.q_body2ned));
  return state;
}

inline RigidBodyState post_step_rigid_body(RigidBodyState state) {
  return normalize_rigid_body_state(state);
}

inline RigidBodyDerivative operator+(const RigidBodyDerivative& lhs,
                                     const RigidBodyDerivative& rhs) {
  return {
      lhs.position_ned_derivative_mps + rhs.position_ned_derivative_mps,
      lhs.velocity_body_derivative_mps2 + rhs.velocity_body_derivative_mps2,
      lhs.q_body2ned_derivative + rhs.q_body2ned_derivative,
      lhs.omega_body_derivative_rps2 + rhs.omega_body_derivative_rps2,
  };
}

inline RigidBodyDerivative operator*(const RigidBodyDerivative& derivative,
                                     double scalar) {
  return {
      derivative.position_ned_derivative_mps * scalar,
      derivative.velocity_body_derivative_mps2 * scalar,
      derivative.q_body2ned_derivative * scalar,
      derivative.omega_body_derivative_rps2 * scalar,
  };
}

inline RigidBodyDerivative operator*(double scalar,
                                     const RigidBodyDerivative& derivative) {
  return derivative * scalar;
}

inline RigidBodyState operator+(const RigidBodyState& state,
                                const RigidBodyDerivative& derivative) {
  return {
      state.position_ned_m + derivative.position_ned_derivative_mps,
      state.velocity_body_mps + derivative.velocity_body_derivative_mps2,
      state.q_body2ned + derivative.q_body2ned_derivative,
      state.omega_body_rps + derivative.omega_body_derivative_rps2,
  };
}

inline RigidBodyDerivative combine_derivatives(
    const RigidBodyDerivative& kinematic,
    const RigidBodyDerivative& dynamic) {
  return kinematic + dynamic;
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
