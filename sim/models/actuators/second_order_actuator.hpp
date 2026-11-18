#pragma once

#include "sixsim/math/constants.hpp"
#include "sixsim/sim/models/actuator.hpp"
#include "sixsim/sim/rk4.hpp"

#include <algorithm>
#include <stdexcept>

namespace sixsim::sim {

struct SecondOrderActuatorParams {
  double wn_rps{};
  double zeta{};
  double min_deflection{-math::infinity};
  double max_deflection{math::infinity};
  double max_deflection_rate{math::infinity};
  double max_deflection_acceleration{math::infinity};
};

class SecondOrderActuatorModel final : public ActuatorModel {
 public:
  explicit SecondOrderActuatorModel(
      std::string name,
      const SecondOrderActuatorParams& params,
      const ActuatorState& initial_state = {})
      : ActuatorModel(std::move(name), initial_state), params_(params) {
    if (!(params_.min_deflection <= params_.max_deflection)) {
      throw std::invalid_argument("actuator deflection bounds must be ordered");
    }
    if (!(params_.max_deflection_rate >= 0.0)) {
      throw std::invalid_argument(
          "maximum actuator deflection rate must be nonnegative");
    }
    if (!(params_.max_deflection_acceleration >= 0.0)) {
      throw std::invalid_argument(
          "maximum actuator deflection acceleration must be nonnegative");
    }
  }

  ActuatorState evaluate(double dt_s) override {
    const ActuatorCommand held_command = command();
    const auto evaluate_derivative =
        [&](const ActuatorState& stage_state, double) {
          return derivative(stage_state, held_command);
        };
    ActuatorState next_state = step_rk4(state_, dt_s, evaluate_derivative);
    next_state.deflection_rate = std::clamp(next_state.deflection_rate,
                                           -params_.max_deflection_rate,
                                           params_.max_deflection_rate);
    next_state.deflection = std::clamp(next_state.deflection,
                                      params_.min_deflection,
                                      params_.max_deflection);
    // A hard stop removes outward velocity independently of the drive's acceleration limit.
    if ((next_state.deflection <= params_.min_deflection &&
         next_state.deflection_rate < 0.0) ||
        (next_state.deflection >= params_.max_deflection &&
         next_state.deflection_rate > 0.0)) {
      next_state.deflection_rate = 0.0;
    }
    state_ = next_state;
    return state_;
  }

 private:
  ActuatorDerivative derivative(const ActuatorState& state,
                                const ActuatorCommand& command) const {
    const double deflection = std::clamp(state.deflection,
                                        params_.min_deflection,
                                        params_.max_deflection);
    double rate = std::clamp(state.deflection_rate,
                             -params_.max_deflection_rate,
                             params_.max_deflection_rate);
    if ((deflection <= params_.min_deflection && rate < 0.0) ||
        (deflection >= params_.max_deflection && rate > 0.0)) {
      rate = 0.0;
    }
    double acceleration =
        params_.wn_rps * params_.wn_rps *
            (command.commanded_deflection - deflection) -
        2.0 * params_.zeta * params_.wn_rps * rate;
    acceleration = std::clamp(acceleration,
                              -params_.max_deflection_acceleration,
                              params_.max_deflection_acceleration);
    if ((rate >= params_.max_deflection_rate && acceleration > 0.0) ||
        (rate <= -params_.max_deflection_rate && acceleration < 0.0)) {
      acceleration = 0.0;
    }
    if (rate == 0.0 &&
        ((deflection <= params_.min_deflection && acceleration < 0.0) ||
         (deflection >= params_.max_deflection && acceleration > 0.0))) {
      acceleration = 0.0;
    }
    return {rate, acceleration};
  }

  SecondOrderActuatorParams params_;
};

}  // namespace sixsim::sim
