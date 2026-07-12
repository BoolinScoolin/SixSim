#pragma once

#include "sixsim/sim/models/atmosphere.hpp"

namespace sixsim::sim {

class ConstantAtmosphere final : public AtmosphereModel {
 public:
  explicit ConstantAtmosphere(const AtmosphereState& atmosphere);

  AtmosphereState evaluate(const SimTime& time,
                           const math::Vector3& position_ned_m) const override;

 private:
  AtmosphereState atmosphere_{};
};

}  // namespace sixsim::sim
