#pragma once

#include "sixsim/sim/models/actuator.hpp"

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace sixsim::sim {

class Actuators {
 public:
  template <typename Actuator, typename... Args>
  Actuator& create_actuator(std::string name, Args&&... args) {
    if (name.empty()) {
      throw std::invalid_argument("actuator name must not be empty");
    }
    for (const std::unique_ptr<ActuatorModel>& actuator : actuators_) {
      if (actuator->name() == name) {
        throw std::invalid_argument("duplicate actuator name: " + name);
      }
    }

    auto actuator =
        std::make_unique<Actuator>(std::move(name), std::forward<Args>(args)...);
    Actuator& actuator_reference = *actuator;
    actuators_.push_back(std::move(actuator));
    return actuator_reference;
  }

  std::size_t get_actuator_index(std::string_view name) const {
    for (std::size_t index = 0; index < actuators_.size(); ++index) {
      if (actuators_[index]->name() == name) {
        return index;
      }
    }

    throw std::invalid_argument("required actuator not found: " +
                                std::string{name});
  }

  const ActuatorState& state(std::size_t index) const {
    return actuators_.at(index)->state();
  }

  void update(double dt_s) {
    for (const auto& actuator : actuators_) {
      actuator->evaluate(dt_s);
    }
  }

 private:
  std::vector<std::unique_ptr<ActuatorModel>> actuators_;
};

}  // namespace sixsim::sim
