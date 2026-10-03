#pragma once

#include "hal/sitl/device_registry.hpp"
#include "hal/sitl/tick_counter.hpp"
#include "sixsim/hal/generated/irec_2526_fcu/sitl_devices.hpp"
#include "sixsim/hal/profiles/irec_2526_fcu.hpp"
#include "sixsim/sim/sim_general.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

namespace sixsim::hal::irec_2526_fcu {

class SitlHal {
 public:
  explicit SitlHal(
      std::unique_ptr<SitlDeviceRegistry> device_registry)
      : device_registry_(std::move(device_registry)),
        devices_(require_registry(device_registry_)) {}

  uint64_t cycle_ticks() const { return tick_counter_.read(); }
  void advance_ticks(uint64_t ticks) { tick_counter_.advance(ticks); }
  void update_sensors(const sim::SensorTruthInputs& inputs) {
    device_registry_->sensors().update(inputs);
  }

  generated::irec_2526_fcu::SitlDevices& devices() { return devices_; }
  const generated::irec_2526_fcu::SitlDevices& devices() const {
    return devices_;
  }

 private:
  static SitlDeviceRegistry& require_registry(
      const std::unique_ptr<SitlDeviceRegistry>& device_registry) {
    if (device_registry == nullptr) {
      throw std::invalid_argument(
          "SITL FCU requires a device registry");
    }
    return *device_registry;
  }

  std::unique_ptr<SitlDeviceRegistry> device_registry_;
  generated::irec_2526_fcu::SitlDevices devices_;
  SitlTickCounter tick_counter_;
};

using SitlFcu = Fcu<SitlHal>;

}  // namespace sixsim::hal::irec_2526_fcu
