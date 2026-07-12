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
  math::Quaternion q_body2ned_derivative{};
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

}  // namespace sixsim::sim

#include "sixsim/sim/rigid_body_dynamics.hpp"
#include "sixsim/sim/rigid_body_kinematics.hpp"

namespace sixsim::sim {

inline RigidBodyDerivative rigid_body_derivative(
    const RigidBodyState& state,
    const ForceMoment& force_moment,
    const MassProperties& mass_properties) {
  return combine_derivatives(
      kinematic_derivative(state),
      dynamic_derivative(state, force_moment, mass_properties));
}

}  // namespace sixsim::sim
