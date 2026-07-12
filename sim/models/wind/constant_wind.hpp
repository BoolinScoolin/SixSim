#pragma once

#include "sixsim/sim/models/wind.hpp"

namespace sixsim::sim {

class ConstantWind final : public WindModel {
 public:
  explicit ConstantWind(const math::Vector3& wind_ned_mps);

  WindState evaluate(const SimTime& time,
                     const math::Vector3& position_ned_m) const override;

 private:
  math::Vector3 wind_ned_mps_{};
};

}  // namespace sixsim::sim
