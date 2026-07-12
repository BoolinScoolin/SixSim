#pragma once

#include "sixsim/math/quaternion.hpp"
#include "sixsim/math/vector3.hpp"

namespace sixsim::sim {

struct RigidBodyState {
  math::Vector3 position_ned_m{};
  math::Vector3 velocity_body_mps{};
  math::Quaternion q_body2ned{};
  math::Vector3 omega_body_rps{};
};

}  // namespace sixsim::sim
