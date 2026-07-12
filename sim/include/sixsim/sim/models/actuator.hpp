#pragma once

namespace sixsim::sim {

struct ActuatorCommand {
  double command_unit{};
};

struct ActuatorState {
  double achieved_unit{};
};

class ActuatorModel {
 public:
  virtual ~ActuatorModel() = default;

  virtual ActuatorState evaluate(const ActuatorCommand& command,
                                 double dt_s) = 0;
};

}  // namespace sixsim::sim
