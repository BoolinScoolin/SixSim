#pragma once

#include "sixsim/sim/models/gravity.hpp"

namespace sixsim::sim {

class ConstantGravity final : public GravityModel {
 public:
  explicit ConstantGravity(double gravity_mps2);

  GravityState evaluate(const SimTime& time,
                        const math::Vector3& position_ned_m) const override;

 private:
  double gravity_mps2_{};
};

}  // namespace sixsim::sim
