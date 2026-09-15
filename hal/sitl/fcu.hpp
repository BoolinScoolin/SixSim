#pragma once

#include "hal/sitl/hardware.hpp"
#include "sixsim/sim/sim_general.hpp"

#include <string>
#include <string_view>
#include <utility>

namespace sixsim::hal {

class SitlFcu {
 public:
  explicit SitlFcu(std::string revision) : revision_(std::move(revision)) {}

  std::string_view revision() const { return revision_; }
  SitlHardware& hardware() { return hardware_; }
  const SitlHardware& hardware() const { return hardware_; }

  template <typename Sensor, typename... Args>
  Sensor& create_sensor(std::string name, Args&&... args) {
    return hardware_.sensors().create_sensor<Sensor>(
        std::move(name), std::forward<Args>(args)...);
  }

  template <typename Sensor>
  Sensor* find_sensor(std::string_view name) {
    return hardware_.sensors().find_sensor<Sensor>(name);
  }

  template <typename Sensor>
  const Sensor* find_sensor(std::string_view name) const {
    return hardware_.sensors().find_sensor<Sensor>(name);
  }

  void update_sensors(const sim::SensorTruthInputs& inputs) {
    hardware_.sensors().update(inputs);
  }

 private:
  std::string revision_;
  SitlHardware hardware_;
};

}  // namespace sixsim::hal
