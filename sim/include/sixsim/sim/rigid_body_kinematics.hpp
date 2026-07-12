#pragma once

#include "sixsim/math/quaternion.hpp"
#include "sixsim/sim/rigid_body_derivative.hpp"
#include "sixsim/sim/rigid_body_state.hpp"

namespace sixsim::sim {

inline RigidBodyDerivative kinematic_derivative(const RigidBodyState& state) {
  RigidBodyDerivative derivative{};
  derivative.position_ned_derivative_mps =
      math::rotate(state.q_body2ned, state.velocity_body_mps);
  derivative.q_body2ned_derivative =
      math::derivative_from_body_rate(state.q_body2ned, state.omega_body_rps);
  return derivative;
}

}  // namespace sixsim::sim
