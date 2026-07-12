#pragma once

#include "sixsim/math/vector3.hpp"
#include "sixsim/sim/sim_time.hpp"

namespace sixsim::sim {

struct GravityState {
  math::Vector3 acceleration_ned_mps2{};
};

class GravityModel {
 public:
  virtual ~GravityModel() = default;

  virtual GravityState evaluate(const SimTime& time,
                                const math::Vector3& position_ned_m) const = 0;
};

}  // namespace sixsim::sim
