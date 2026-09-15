#pragma once

#include "sixsim/hal/altimeter.hpp"
#include "sixsim/sim/scenario.hpp"
#include "hal/sitl/sensors.hpp"

#include <utility>

namespace sixsim::hal {

class SitlAltimeter : public Altimeter, public SitlSensor {
 public:
  explicit SitlAltimeter(std::string name) : SitlSensor(std::move(name)) {}

  void update(const sim::SensorTruthInputs& inputs) override {
    sample_.altitude_msl_m =
        inputs.environment.origin_altitude_msl_m -
        inputs.vehicle_state.position_ned_m.z;
    sample_.measurement_time_s = inputs.time.simtime_s;
    sample_.valid = true;
  }

  AltimeterSample read() override {
    return sample_;
  }

 private:
  AltimeterSample sample_{};
};

}  // namespace sixsim::hal
