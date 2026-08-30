#pragma once

#include "sixsim/math/quaternion.hpp"
#include "sixsim/math/vector3.hpp"
#include "sixsim/sim/sim_general.hpp"

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

inline ForceMoment gravity_force_moment_body(
    const GravityState& gravity,
    const MassProperties& mass_properties,
    const math::Quaternion& q_body2ned) {
  ForceMoment force_moment{};
  const math::Vector3 force_ned =
      gravity.acceleration_ned_mps2 * mass_properties.mass_kg;
  force_moment.force_body_n =
      math::rotate(math::conjugate(q_body2ned), force_ned);
  return force_moment;
}

}  // namespace sixsim::sim
