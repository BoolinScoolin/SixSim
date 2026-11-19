#pragma once

#include "sixsim/sim/models/atmosphere.hpp"

namespace sixsim::sim {

// Standard dry atmosphere from -5 km to 84.852 km geopotential altitude.
class Isa1976Atmosphere final : public AtmosphereModel {
 public:
  explicit Isa1976Atmosphere(double origin_altitude_msl_m);

  // Throws std::invalid_argument for nonfinite altitude and std::out_of_range
  // for altitude outside the supported range.
  AtmosphereState evaluate(const SimTime& time,
                           const math::Vector3& position_ned_m) const override;

 private:
  double origin_altitude_msl_m_{};
};

}  // namespace sixsim::sim
