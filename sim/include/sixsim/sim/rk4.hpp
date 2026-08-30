#pragma once

namespace sixsim::sim {

template <typename State>
State no_post_step(const State& state) {
  return state;
}

// State must support State + Derivative. Derivative must support addition and
// scalar multiplication. post_step must return the corrected State.
template <typename State, typename DerivativeFunction, typename PostStepFunction>
State step_rk4(const State& state,
               double dt_s,
               DerivativeFunction derivative_function,
               PostStepFunction post_step) {
  const auto k1 = derivative_function(state);
  const auto k2 = derivative_function(state + k1 * (0.5 * dt_s));
  const auto k3 = derivative_function(state + k2 * (0.5 * dt_s));
  const auto k4 = derivative_function(state + k3 * dt_s);

  return post_step(state + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * (dt_s / 6.0));
}

template <typename State, typename DerivativeFunction>
State step_rk4(const State& state,
               double dt_s,
               DerivativeFunction derivative_function) {
  return step_rk4(state, dt_s, derivative_function, no_post_step<State>);
}

}  // namespace sixsim::sim
