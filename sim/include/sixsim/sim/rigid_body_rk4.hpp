#pragma once

#include "sixsim/sim/rigid_body_derivative.hpp"
#include "sixsim/sim/rigid_body_state.hpp"

namespace sixsim::sim {

namespace detail {

inline RigidBodyState advance(const RigidBodyState& state,
                              const RigidBodyDerivative& derivative,
                              double dt_s) {
  return {
      state.position_ned_m + derivative.position_ned_derivative_mps * dt_s,
      state.velocity_body_mps + derivative.velocity_body_derivative_mps2 * dt_s,
      state.q_body2ned + derivative.q_body2ned_derivative * dt_s,
      state.omega_body_rps + derivative.omega_body_derivative_rps2 * dt_s,
  };
}

inline RigidBodyDerivative weighted_sum(const RigidBodyDerivative& k1,
                                        const RigidBodyDerivative& k2,
                                        const RigidBodyDerivative& k3,
                                        const RigidBodyDerivative& k4) {
  return {
      (k1.position_ned_derivative_mps + 2.0 * k2.position_ned_derivative_mps +
       2.0 * k3.position_ned_derivative_mps + k4.position_ned_derivative_mps) /
          6.0,
      (k1.velocity_body_derivative_mps2 +
       2.0 * k2.velocity_body_derivative_mps2 +
       2.0 * k3.velocity_body_derivative_mps2 +
       k4.velocity_body_derivative_mps2) /
          6.0,
      (k1.q_body2ned_derivative + 2.0 * k2.q_body2ned_derivative +
       2.0 * k3.q_body2ned_derivative + k4.q_body2ned_derivative) /
          6.0,
      (k1.omega_body_derivative_rps2 + 2.0 * k2.omega_body_derivative_rps2 +
       2.0 * k3.omega_body_derivative_rps2 + k4.omega_body_derivative_rps2) /
          6.0,
  };
}

}  // namespace detail

template <typename DerivativeFunction>
RigidBodyState integrate_rk4(const RigidBodyState& state,
                             double dt_s,
                             DerivativeFunction derivative_function) {
  const RigidBodyDerivative k1 = derivative_function(state);
  const RigidBodyDerivative k2 =
      derivative_function(detail::advance(state, k1, 0.5 * dt_s));
  const RigidBodyDerivative k3 =
      derivative_function(detail::advance(state, k2, 0.5 * dt_s));
  const RigidBodyDerivative k4 =
      derivative_function(detail::advance(state, k3, dt_s));

  return detail::advance(state, detail::weighted_sum(k1, k2, k3, k4), dt_s);
}

inline RigidBodyState integrate_rigid_body_rk4(
    const RigidBodyState& state,
    const ForceMoment& force_moment,
    const MassProperties& mass_properties,
    double dt_s) {
  RigidBodyState next_state =
      integrate_rk4(state, dt_s, [&](const RigidBodyState& current_state) {
    return rigid_body_derivative(current_state, force_moment, mass_properties);
  });
  next_state.q_body2ned =
      math::positive_scalar(math::normalized(next_state.q_body2ned));
  return next_state;
}

}  // namespace sixsim::sim
