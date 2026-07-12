#pragma once

#include "sixsim/math/quaternion.hpp"
#include "sixsim/math/vector3.hpp"

namespace sixsim::sim {

struct RigidBodyState {
  math::Vector3 position_ned_m{};
  math::Vector3 velocity_ned_mps{};
  math::Quaternion q_body2ned{};
  math::Vector3 omega_body_rps{};
};

struct RigidBodyDerivative {
  math::Vector3 position_ned_derivative_mps{};
  math::Vector3 velocity_ned_derivative_mps2{};
  math::Quaternion q_body2ned_derivative{};
  math::Vector3 omega_body_derivative_rps2{};
};

}  // namespace sixsim::sim
