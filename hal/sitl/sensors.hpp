#pragma once

#include "sixsim/sim/sim_general.hpp"

#include <memory>
#include <string>
#include <string_view>
#include <stdexcept>
#include <utility>
#include <vector>

namespace sixsim::hal {

class SitlSensor {
 public:
  explicit SitlSensor(std::string name) : name_(std::move(name)) {}
  virtual ~SitlSensor() = default;

  std::string_view name() const { return name_; }
  virtual void update(const sim::SensorTruthInputs& inputs) = 0;

 private:
  std::string name_;
};

class SitlSensors {
 public:
  template <typename Sensor, typename... Args>
  Sensor& create_sensor(std::string name, Args&&... args) {
    if (name.empty()) {
      throw std::invalid_argument("SITL sensor name must not be empty");
    }
    for (const std::unique_ptr<SitlSensor>& sensor : sensors_) {
      if (sensor->name() == name) {
        throw std::invalid_argument("duplicate SITL sensor name: " + name);
      }
    }

    auto sensor =
        std::make_unique<Sensor>(std::move(name), std::forward<Args>(args)...);
    Sensor& sensor_reference = *sensor;
    sensors_.push_back(std::move(sensor));
    return sensor_reference;
  }

  template <typename Interface>
  Interface& require(std::string_view name) {
    for (const std::unique_ptr<SitlSensor>& sensor : sensors_) {
      if (sensor->name() != name) {
        continue;
      }

      Interface* interface = dynamic_cast<Interface*>(sensor.get());
      if (interface == nullptr) {
        throw std::invalid_argument(
            "SITL sensor does not implement the required interface: " +
            std::string{name});
      }
      return *interface;
    }

    throw std::invalid_argument(
        "required SITL sensor not found: " + std::string{name});
  }

  void update(const sim::SensorTruthInputs& inputs) {
    for (const std::unique_ptr<SitlSensor>& sensor : sensors_) {
      sensor->update(inputs);
    }
  }

 private:
  std::vector<std::unique_ptr<SitlSensor>> sensors_;
};

}  // namespace sixsim::hal
