#pragma once

#include <string>
#include <string_view>
#include <utility>

namespace sixsim::sim {

struct ActuatorCommand {
  double commanded_deflection{};
};

struct ActuatorState {
  double deflection{};
  double deflection_rate{};
};

struct ActuatorDerivative {
  double deflection_rate{};
  double deflection_acceleration{};
};

inline ActuatorDerivative operator+(const ActuatorDerivative& lhs,
                                    const ActuatorDerivative& rhs) {
  return {lhs.deflection_rate + rhs.deflection_rate,
          lhs.deflection_acceleration + rhs.deflection_acceleration};
}

inline ActuatorDerivative operator*(const ActuatorDerivative& derivative,
                                    double scalar) {
  return {derivative.deflection_rate * scalar,
          derivative.deflection_acceleration * scalar};
}

inline ActuatorDerivative operator*(double scalar,
                                    const ActuatorDerivative& derivative) {
  return derivative * scalar;
}

inline ActuatorState operator+(const ActuatorState& state,
                               const ActuatorDerivative& increment) {
  return {state.deflection + increment.deflection_rate,
          state.deflection_rate + increment.deflection_acceleration};
}

class ActuatorModel {
 public:
  explicit ActuatorModel(std::string name,
                         const ActuatorState& initial_state = {})
      : name_(std::move(name)), state_(initial_state) {}
  virtual ~ActuatorModel() = default;

  std::string_view name() const { return name_; }

  const ActuatorState& state() const { return state_; }

  ActuatorCommand& command() { return command_; }
  const ActuatorCommand& command() const { return command_; }

  virtual ActuatorState evaluate(double dt_s) = 0;

 private:
  std::string name_;
  ActuatorCommand command_{};

 protected:
  ActuatorState state_;
};

}  // namespace sixsim::sim
