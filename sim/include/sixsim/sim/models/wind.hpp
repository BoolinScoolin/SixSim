#pragma once

#include "sixsim/math/vector3.hpp"
#include "sixsim/sim/sim_time.hpp"

namespace sixsim::sim {

struct WindState {
  math::Vector3 wind_ned_mps{};
};

class WindModel {
 public:
  virtual ~WindModel() = default;

  virtual WindState evaluate(const SimTime& time,
                             const math::Vector3& position_ned_m) const = 0;
};

}  // namespace sixsim::sim
