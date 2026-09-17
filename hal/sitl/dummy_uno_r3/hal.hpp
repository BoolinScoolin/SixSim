#pragma once

#include "hal/sitl/device_registry.hpp"
#include "hal/sitl/tick_counter.hpp"
#include "sixsim/hal/generated/dummy_uno_r3/sitl_devices.hpp"
#include "sixsim/hal/profiles/dummy_uno_r3.hpp"
#include "sixsim/sim/sim_general.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

namespace sixsim::hal {

class SitlDummyUnoR3Hal {
 public:
  explicit SitlDummyUnoR3Hal(
      std::unique_ptr<SitlDeviceRegistry> device_registry)
      : device_registry_(std::move(device_registry)),
        devices_(require_registry(device_registry_)) {}

  uint64_t cycle_ticks() const { return tick_counter_.read(); }
  void advance_ticks(uint64_t ticks) { tick_counter_.advance(ticks); }
  void update_sensors(const sim::SensorTruthInputs& inputs) {
    device_registry_->sensors().update(inputs);
  }

  generated::dummy_uno_r3::SitlDevices& devices() { return devices_; }
  const generated::dummy_uno_r3::SitlDevices& devices() const {
    return devices_;
  }

 private:
  static SitlDeviceRegistry& require_registry(
      const std::unique_ptr<SitlDeviceRegistry>& device_registry) {
    if (device_registry == nullptr) {
      throw std::invalid_argument(
          "SITL dummy Uno R3 requires a device registry");
    }
    return *device_registry;
  }

  std::unique_ptr<SitlDeviceRegistry> device_registry_;
  generated::dummy_uno_r3::SitlDevices devices_;
  SitlTickCounter tick_counter_;
};

using SitlDummyUnoR3Fcu = DummyUnoR3Fcu<SitlDummyUnoR3Hal>;

}  // namespace sixsim::hal
